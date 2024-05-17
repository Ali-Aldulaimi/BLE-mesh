#include <zephyr/sys/printk.h>
#include <zephyr/settings/settings.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/bluetooth/mesh/access.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include "board.h"
#include <zephyr/logging/log.h>
#include <power/reboot.h>


//LOG_MODULE_REGISTER(mesh_observer, LOG_LEVEL_DBG);


// Definitions of macro mesh operations
#define OP_ONOFF_GET       BT_MESH_MODEL_OP_2(0x82, 0x01)
#define OP_ONOFF_SET       BT_MESH_MODEL_OP_2(0x82, 0x02)
#define OP_ONOFF_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x03)
#define OP_ONOFF_STATUS    BT_MESH_MODEL_OP_2(0x82, 0x04)
#define OP_SEQ_NUMBER      BT_MESH_MODEL_OP_2(0x82, 0x05)  // Operation code for sequence number

bool onoff_state = false; 

static struct {
	bool val;
	uint8_t tid;
	uint16_t src;
	uint32_t transition_time;
	struct k_work_delayable work;
} onoff;

static uint8_t dev_uuid[16];  // array to store device uuid
/*static int rebroadcast(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, uint32_t seq_num, uint8_t ttl) {
        struct net_buf_simple *msg = NET_BUF_SIMPLE(2 + 4 + 4);
        net_buf_simple_init(msg, 0);
        bt_mesh_model_msg_init(msg, OP_SEQ_NUMBER);
        net_buf_simple_add_le32(msg, seq_num); // Re-add the sequence number

        struct bt_mesh_msg_ctx new_ctx = *ctx;
        new_ctx.send_ttl = ttl - 1;

        int err = bt_mesh_model_send(model, &new_ctx, msg, NULL, NULL);
        if (err) {
            printk("Error rebroadcasting: %d\n", err);
            return err;
        } else {
            printk("Rebroadcasting seq_num %u with new TTL %u\n", seq_num, new_ctx.send_ttl);
        }
    
    return 0;
}*/

// Handler for receiving messages with sequence numbers
static int PLR(uint16_t seq_num) {
    // Static variables to keep track of sequence and statistics
    static uint16_t last_seq_num = 0;
	static uint16_t first_seq_seq = 0;
    static bool is_first_packet = true;
    static uint16_t total_missed_packets = 0;
    static uint16_t total_received_packets = 0;

    // Handling the first packet received
    if (is_first_packet) {
        first_seq_seq = seq_num;
		last_seq_num = seq_num ;  // Initialize the last sequence number with the first received sequence
        is_first_packet = false; // Mark that the first packet has now been received
        total_received_packets++; // Start counting packets with the first one received
    } else {
        // Normal handling for all subsequent packets
        if (seq_num > last_seq_num) {
            // Calculating missed packets if there's a gap
            total_missed_packets += (seq_num - last_seq_num - 1);
            last_seq_num = seq_num; // Update the last received sequence number
            total_received_packets++; // Increment the total received packets count
        } else if (seq_num == last_seq_num) {
            // Handle duplicate packet
            printf("Duplicate packet received, seq_num: %u\n", seq_num);
        } else {
            // Handle out of order packet
            printf("Out of order packet received, seq_num: %u, last_seq_num: %u\n", seq_num, last_seq_num);
        }
    }

    // Calculate Packet Loss Ratio (PLR)
    uint32_t total_packets_considered = total_received_packets + total_missed_packets;
    uint32_t plr = (total_missed_packets * 10000) / total_packets_considered;

	printk("First received packet: %u\n", first_seq_seq);
    printk("Total missed packets: %u\n", total_missed_packets);
    printk("Total received packets: %u\n", total_received_packets);
    printk("Received seq_num %u, PLR: %u.%02u%%\n", seq_num, plr / 100, plr % 100);
    printk("......................\n");

    return 0;
}

static const uint32_t time_res[] = {
	100,
	MSEC_PER_SEC,
	10 * MSEC_PER_SEC,
	10 * 60 * MSEC_PER_SEC,
};
static inline int32_t model_time_decode(uint8_t val)
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

static int gen_onoff_set_unack(const struct bt_mesh_model *model,
			       struct bt_mesh_msg_ctx *ctx,
			       struct net_buf_simple *buf)
{	

	if (buf->len < 4) {
        printk("Buffer too short\n");
        return -EINVAL;
    }
	uint8_t val = net_buf_simple_pull_u8(buf);
	uint16_t seq = net_buf_simple_pull_le16(buf);
	uint8_t tid = net_buf_simple_pull_u8(buf);
	
	int32_t trans = 0;
	int32_t delay = 0;
	printk("Receiving onoff:%d, tid:%d ,src:%d ,seq:%u \n", val, tid, ctx->addr ,seq);

	PLR(seq);
	/*if (ctx->recv_ttl < 1) {
		rebroadcast(model, ctx, seq, ctx->recv_ttl);
	}*/


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

	//printk("set: %s delay: %d ms time: %d ms\n", onoff_str[val], delay,
	//       trans);

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

// Definition of the OnOff Client Model Operations
static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
    {OP_ONOFF_STATUS, BT_MESH_LEN_MIN(4), PLR},
   // {OP_SEQ_NUMBER, BT_MESH_LEN_EXACT(4), message_received},  
    BT_MESH_MODEL_OP_END,
};

static int onoff_status_send(const struct bt_mesh_model *model,
			     struct bt_mesh_msg_ctx *ctx)
{
	uint32_t remaining;

	BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_STATUS, 6);
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


static int gen_onoff_set(const struct bt_mesh_model *model,
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	(void)gen_onoff_set_unack(model, ctx, buf);
	onoff_status_send(model, ctx);

	return 0;
}
static int gen_onoff_get(const struct bt_mesh_model *model,
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	onoff_status_send(model, ctx);
	return 0;
}

static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
	{ OP_ONOFF_GET,       BT_MESH_LEN_EXACT(0), gen_onoff_get },
	{ OP_ONOFF_SET,       BT_MESH_LEN_MIN(2),   gen_onoff_set },
	{ OP_ONOFF_SET_UNACK, BT_MESH_LEN_MIN(2),   gen_onoff_set_unack },
	BT_MESH_MODEL_OP_END,
};


// Mesh Model Declaration
static struct bt_mesh_model models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op, NULL, NULL),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op, NULL, NULL),
};

// Mesh Element Declaration
static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, models, BT_MESH_MODEL_NONE),
};

// Mesh Composition
static const struct bt_mesh_comp comp = {
    .cid = BT_COMP_ID_LF,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};
static struct k_timer reset_timer;
static void reset_timer_expiry(struct k_timer *timer_id)
{
    printk("Timer expired. Resetting the system.\n");
    sys_reboot(SYS_REBOOT_COLD);
}

// Bluetooth ready callback
static void bt_ready(int err) {
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");
     // Generate a unique UUID
   
    //Generate UUID Using HWINFO:   
    static const struct bt_mesh_prov prov = {
        .uuid = dev_uuid,
        // Other provisioning properties here, like output size, output actions, etc.
    };

    err = bt_mesh_init(&prov, &comp);
    if (err) {
        printk("Initializing mesh failed (err %d)\n", err);
        return;
    }

    printk("Mesh initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

    printk("Mesh initialized\n");

    if (bt_mesh_is_provisioned()) {
        printk("Already provisioned\n");
    } else {
        printk("Not provisioned\n");
    }
	k_timer_init(&reset_timer, reset_timer_expiry, NULL);
    k_timer_start(&reset_timer, K_SECONDS(62), K_NO_WAIT);
}

static void onoff_timeout(struct k_work *work)
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



int main(void)
{
	static struct k_work button_work;
	int err = -1;

	printk("Initializing...\n");

	if (IS_ENABLED(CONFIG_HWINFO)) {
		err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
	}

	if (err < 0) {
		dev_uuid[0] = 0xdd;
		dev_uuid[1] = 0xdd;
	}

	//k_work_init(&button_work, button_pressed);

	err = board_init(&button_work);
	if (err) {
		printk("Board init failed (err: %d)\n", err);
		return 0;
	}

	k_work_init_delayable(&onoff.work, onoff_timeout);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
	return 0;
}