#include <zephyr/sys/printk.h>
#include <zephyr/settings/settings.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include "board.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* Define Bluetooth Mesh operation codes (opcodes)*/
#define OP_ONOFF_GET       BT_MESH_MODEL_OP_2(0x82, 0x01)
#define OP_ONOFF_SET       BT_MESH_MODEL_OP_2(0x82, 0x02)
#define OP_ONOFF_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x03)
#define OP_ONOFF_STATUS    BT_MESH_MODEL_OP_2(0x82, 0x04)
#define OP_SEQ_NUMBER      BT_MESH_MODEL_OP_2(0x82, 0x05)
#define SLEEP_TIME_MS	1
/* Define button nodes from the devicetree */
#define SW0_NODE	DT_ALIAS(sw0)
#define SW1_NODE	DT_ALIAS(sw1)
#define SW2_NODE	DT_ALIAS(sw2)
#define SW3_NODE	DT_ALIAS(sw3)

/* Function declarations for button press handlers */
void button1_pressed(struct k_work *work);
void button2_pressed(struct k_work *work);
void button3_pressed(struct k_work *work);
void button4_pressed(struct k_work *work);



// Array of gpio_callback structures for handling GPIO pin interrupt callbacks for up to 4 buttons.
static struct gpio_callback button_cb_data[4];

// Array of k_work structures used to define deferred work tasks for handling button presses in a thread context.
static struct k_work button_works[4];

// External declaration of an array of Bluetooth Mesh model structures
extern struct bt_mesh_model models[];

static int reschedule_interval_ms = 500;
static uint32_t current_seq_num = 0;
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);
static const char *const onoff_str[] = { "off", "on" };		//onoff_str: An array of strings representing "off" and "on" states for easy logging.
static uint8_t dev_uuid[16];

/* Array of buttons */
static const struct gpio_dt_spec buttons[] = {
	GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0}),
	GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios, {0}),
	GPIO_DT_SPEC_GET_OR(SW2_NODE, gpios, {0}),
	GPIO_DT_SPEC_GET_OR(SW3_NODE, gpios, {0})
};

/* Array of callback functions */
static void (*button_handlers[])(struct k_work *work) = {
	button1_pressed,
	button2_pressed,
	button3_pressed,
	button4_pressed
};



/* Generic button pressed callback */
void button_callback(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
	for (int i = 0; i < ARRAY_SIZE(buttons); i++) {
		if (pins & BIT(buttons[i].pin)) {
			k_work_submit(&button_works[i]);
		}
	}
}


/*--------------------------------------------------------------------------Functions definitions------------------------------------------------------------------*/

static void attention_on(const struct bt_mesh_model *mod)			//attention_on and attention_off control an LED to indicate the device's attention state as part of the mesh health model.
{
	board_led_set(true);
}

static void attention_off(const struct bt_mesh_model *mod)
{
	board_led_set(false);
}

static const struct bt_mesh_health_srv_cb health_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_cb,
};

static struct {
	bool val;
	uint8_t tid;
	uint16_t src;
	uint32_t transition_time;
	struct k_work_delayable work;
} onoff;													//onoff: A structure to keep track of the current state, the source address of the last command, 

/* OnOff messages' transition time and remaining time fields are encoded as an
 * 8 bit value with a 6 bit step field and a 2 bit resolution field.
 * The resolution field maps to:
 * 0: 100 ms
 * 1: 1 s
 * 2: 10 s
 * 3: 20 min
 */
static const uint32_t time_res[] = {			//time_res: An array defining time resolutions for decoding and encoding transition times.
	100,
	MSEC_PER_SEC,
	10 * MSEC_PER_SEC,
	10 * 60 * MSEC_PER_SEC,
};

static inline int32_t model_time_decode(uint8_t val)		//model_time_decode and model_time_encode are functions to convert between milliseconds and a compact format used in mesh messages, which combines a resolution and step count to fit into a single byte.
{
	uint8_t resolution = (val >> 6) & BIT_MASK(2);
	uint8_t steps = val & BIT_MASK(6);

	if (steps == 0x3f) {
		return SYS_FOREVER_MS;
	}

	return steps * time_res[resolution];
}

static inline uint8_t model_time_encode(int32_t ms)
{
	if (ms == SYS_FOREVER_MS) {
		return 0x3f;
	}
	for (int i = 0; i < ARRAY_SIZE(time_res); i++) {
		if (ms >= BIT_MASK(6) * time_res[i]) {
			continue;
		}
		uint8_t steps = DIV_ROUND_UP(ms, time_res[i]);
		return steps | (i << 6);
	}
	return 0x3f;
}

static int onoff_status_send(const struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx)
{
	uint32_t remaining;
	BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_STATUS, 3);
	bt_mesh_model_msg_init(&buf, OP_ONOFF_STATUS);

	remaining = k_ticks_to_ms_floor32(
			    k_work_delayable_remaining_get(&onoff.work)) +
		    onoff.transition_time;

	/* Check using remaining time instead of "work pending" to make the
	 * onoff status send the right value on instant transitions. As the
	 * work item is executed in a lower priority than the mesh message
	 * handler, the work will be pending even on instant transitions.
	 */
	if (remaining) {
		net_buf_simple_add_u8(&buf, !onoff.val);
		net_buf_simple_add_u8(&buf, onoff.val);
		net_buf_simple_add_u8(&buf, model_time_encode(remaining));
	} else {
		net_buf_simple_add_u8(&buf, onoff.val);
	}
	
	return bt_mesh_model_send(model, ctx, &buf, NULL, NULL);
}

struct mesh_message {
    uint32_t seq_num;
    };

/** Send an OnOff Set message with sequence number from the Generic OnOff Client to all nodes. */

static int gen_onoff_send_with_seq(bool val)
{
    if (models[3].keys[0] == BT_MESH_KEY_UNUSED) {
        printk("The Generic OnOff Client must be bound to a key before sending.\n");
        return -ENOENT;
    }
    struct mesh_message msg = {
        .seq_num = current_seq_num++
    };
    struct bt_mesh_msg_ctx ctx = {
        .app_idx = models[3].keys[0],
        .addr = BT_MESH_ADDR_ALL_NODES,
        .send_ttl = BT_MESH_TTL_DEFAULT,
    };
    BT_MESH_MODEL_BUF_DEFINE(buf, OP_SEQ_NUMBER, sizeof(msg));
    bt_mesh_model_msg_init(&buf, OP_SEQ_NUMBER);
    net_buf_simple_add_mem(&buf, &msg, sizeof(msg));
    printk("Sending message: Seq Num: %u\n", msg.seq_num);
    return bt_mesh_model_send(&models[3], &ctx, &buf, NULL, NULL);
}

static void onoff_timeout(struct k_work *work)			//onoff_timeout handles the completion of a state transition due to delay or transition time, updating the LED state accordingly.
{
	if (onoff.transition_time) {
		/* Start transition.
		 *
		 * The LED should be on as long as the transition is in
		 * progress, regardless of the target value, according to the
		 * Bluetooth Mesh Model specification, section 3.1.1.
		 */
		board_led_set(true);
		k_work_reschedule(&onoff.work, K_MSEC(onoff.transition_time));
		onoff.transition_time = 0;
		return;
	}
	board_led_set(onoff.val);
}

/* Generic OnOff Server message handlers */

static int gen_onoff_get(const struct bt_mesh_model *model,					//Functions like gen_onoff_get, gen_onoff_set, and gen_onoff_set_unack handle incoming messages to get, set, or set without acknowledgment the OnOff state of the node.
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	onoff_status_send(model, ctx);
	return 0;
}

static int gen_onoff_set_unack(const struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf)
{
	uint8_t val = net_buf_simple_pull_u8(buf);
	uint8_t tid = net_buf_simple_pull_u8(buf);
	int32_t trans = 0;
	int32_t delay = 0;

	if (buf->len) {
		trans = model_time_decode(net_buf_simple_pull_u8(buf));
		delay = net_buf_simple_pull_u8(buf) * 5;
	}

	/* Only perform change if the message wasn't a duplicate and the
	 * value is different.
	 */
	if (tid == onoff.tid && ctx->addr == onoff.src) {
		/* Duplicate */
		return 0;
	}

	if (val == onoff.val) {
		/* No change */
		return 0;
	}

	printk("set: %s delay: %d ms time: %d ms\n", onoff_str[val], delay,
	       trans);

	onoff.tid = tid;
	onoff.src = ctx->addr;
	onoff.val = val;
	onoff.transition_time = trans;

	/* Schedule the next action to happen on the delay, and keep
	 * transition time stored, so it can be applied in the timeout.
	 */
	k_work_reschedule(&onoff.work, K_MSEC(delay));

	return 0;
}

static int gen_onoff_set(const struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf)
{
	(void)gen_onoff_set_unack(model, ctx, buf);
	onoff_status_send(model, ctx);
	

	return 0;
}

static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
	{ OP_ONOFF_GET,       BT_MESH_LEN_EXACT(0), gen_onoff_get },
	{ OP_ONOFF_SET,       BT_MESH_LEN_MIN(2),   gen_onoff_set },
	{ OP_ONOFF_SET_UNACK, BT_MESH_LEN_MIN(2),   gen_onoff_set_unack },
	BT_MESH_MODEL_OP_END,
};

/* Generic OnOff Client */

static int gen_onoff_status(const struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf)
{
	uint8_t present = net_buf_simple_pull_u8(buf);

	if (buf->len) {
		uint8_t target = net_buf_simple_pull_u8(buf);
		int32_t remaining_time =
			model_time_decode(net_buf_simple_pull_u8(buf));

		printk("OnOff status: %s -> %s: (%d ms)\n", onoff_str[present],
		       onoff_str[target], remaining_time);
		return 0;
	}

	printk("OnOff status: %s\n", onoff_str[present]);

	return 0;
}

static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
	{OP_ONOFF_STATUS, BT_MESH_LEN_MIN(1), gen_onoff_status},
	BT_MESH_MODEL_OP_END,
};

/* This application only needs one element to contain its models */
	struct bt_mesh_model models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op, NULL,
		      NULL),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op, NULL,
		      NULL),
};

static const struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

/* Provisioning */

static int output_number(bt_mesh_output_action_t action, uint32_t number)
{
	printk("OOB Number: %u\n", number);

	board_output_number(action, number);

	return 0;
}

static void prov_complete(uint16_t net_idx, uint16_t addr)
{
	board_prov_complete();
}

static void prov_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

static const struct bt_mesh_prov prov = 
{
	.uuid = dev_uuid,
	.output_size = 4,
	.output_actions = BT_MESH_DISPLAY_NUMBER,
	.output_number = output_number,
	.complete = prov_complete,
	.reset = prov_reset,
};

/** Send an OnOff Set message from the Generic OnOff Client to all nodes. */
static int gen_onoff_send(bool val)
{
    struct bt_mesh_msg_ctx ctx = {
        .app_idx = models[3].keys[0], /* Use the bound key */
        .addr = BT_MESH_ADDR_ALL_NODES,
        .send_ttl = BT_MESH_TTL_DEFAULT,
    };
    static uint8_t tid;

    // Check if the app_idx is set to a valid key index
    if (ctx.app_idx == BT_MESH_KEY_UNUSED) {
        printk("The Generic OnOff Client must be bound to a key before sending.\n");
        return -ENOENT; // No such entry error to indicate the key is not bound
    }

    BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_SET_UNACK, 2);
    bt_mesh_model_msg_init(&buf, OP_ONOFF_SET_UNACK);
    net_buf_simple_add_u8(&buf, val);
    net_buf_simple_add_u8(&buf, tid++);

    //printk("Broadcaster sending message: %s\n", onoff_str[val]);

    return bt_mesh_model_send(&models[3], &ctx, &buf, NULL, NULL);

}

static void broadcast_message(struct k_work *work)
{
    if (bt_mesh_is_provisioned() && models[3].keys[0] != BT_MESH_KEY_UNUSED) {
        onoff.val = !onoff.val; // Toggle the LED state
        board_led_set(onoff.val);
        // Send the OnOff state to all nodes
        gen_onoff_send(onoff.val);
		uint32_t seq = bt_mesh_next_seq();  // Fetch the next sequence number.
		//gen_onoff_send_with_seq(onoff.val);  // Toggle the value as needed
    	printk("Rescheduling broadcast. Current interval: %d ms\n", reschedule_interval_ms);
        printk("seq %d ms\n", seq);
        // Reschedule the work to run again after one second only if successful
       
            k_work_reschedule(&onoff.work, K_MSEC(reschedule_interval_ms));      
    } else {
        // Do not reschedule if not provisioned or key not bound
        printk("Cannot broadcast: either not provisioned or no key bound.\n");
    }
}


static void button_pressed(struct k_work *work)
{
    if (bt_mesh_is_provisioned()) {
        if (models[3].keys[0] != BT_MESH_KEY_UNUSED) {
            printk("Device provisioned and key bound, initiating broadcast.\n");
            broadcast_message(work);
        } else {
            printk("Generic OnOff Client not bound to a key.\n");
        }
    } else {
        printk("Device not provisioned.\n");
    }
}

static void bt_ready(int err)
{
	if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    printk("Bluetooth initialized\n");
    err = bt_mesh_init(&prov, &comp);
    if (err) {
        printk("Initializing mesh failed (err %d)\n", err);
        return;
    }

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }
    bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
    printk("Mesh initialized\n");
    k_work_init_delayable(&onoff.work, broadcast_message);
   // k_work_reschedule(&onoff.work, K_SECONDS(0.1));
}
void button1_pressed(struct k_work *work) {
    printk("Button 1 pressed. Setting time to 1000 ms.\n");
    reschedule_interval_ms = 1000;
	//k_work_init(&button_works[0], button_pressed);
	button_pressed(work);
	
}

void button2_pressed(struct k_work *work) {
    printk("Button 1 pressed. Setting time to 500 ms.\n");
    reschedule_interval_ms = 500;
	button_pressed(work);
}

void button3_pressed(struct k_work *work) {
    printk("Button 1 pressed. Setting time to 100 ms.\n");
    reschedule_interval_ms = 100;
	button_pressed(work);
}

void button4_pressed(struct k_work *work) {
    printk("Button 1 pressed. Setting time to 50 ms.\n");
    reschedule_interval_ms = 50;
	button_pressed(work);
}

/*------------------------------------------------------------------------------------------ MAIN ---------------------------------------------------------------------------*/
int main(void)
{	
		
    int err = -1;
	
    printk("Initializing...\n");
    if (IS_ENABLED(CONFIG_HWINFO)) {
        err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
    }
    if (err < 0) {
        dev_uuid[0] = 0xdd;
        dev_uuid[1] = 0xdd;
    }

	int ret;
	for (int i = 0; i < ARRAY_SIZE(buttons); i++) {
		if (!gpio_is_ready_dt(&buttons[i])) {
			printk("Error: button device %s is not ready\n", buttons[i].port->name);
			return 0;
		}

	ret = gpio_pin_configure_dt(&buttons[i], GPIO_INPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure %s pin %d\n", ret, buttons[i].port->name, buttons[i].pin);
			return 0;
		}

		ret = gpio_pin_interrupt_configure_dt(&buttons[i], GPIO_INT_EDGE_TO_ACTIVE);
		if (ret != 0) {
			printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, buttons[i].port->name, buttons[i].pin);
			return 0;
		}
		k_work_init(&button_works[i], button_handlers[i]);
		gpio_init_callback(&button_cb_data[i], button_callback, BIT(buttons[i].pin));
		gpio_add_callback(buttons[i].port, &button_cb_data[i]);
		k_work_init_delayable(&onoff.work, onoff_timeout);
		err = board_init(&button_works[i]);
    	if (err) {
        printk("Board init failed (err: %d)\n", err);
        return 0;
    }
	}
	err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
    }
	
	while (1) {
		k_msleep(SLEEP_TIME_MS);
	}
   
       
    
    /* Initialize the Bluetooth Subsystem */
    
    return 0;
}