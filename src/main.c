/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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


#include "board.h"
// Include logging for easier debugging of mesh network issues.
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mesh_observer, LOG_LEVEL_DBG);


// Definitions for mesh message operations
#define OP_ONOFF_GET       BT_MESH_MODEL_OP_2(0x82, 0x01)
#define OP_ONOFF_SET       BT_MESH_MODEL_OP_2(0x82, 0x02)
#define OP_ONOFF_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x03)
#define OP_ONOFF_STATUS    BT_MESH_MODEL_OP_2(0x82, 0x04)
#define OP_SEQ_NUMBER      BT_MESH_MODEL_OP_2(0x82, 0x05)  // Operation code for sequence number
// In other C files where onoff_state is used
bool onoff_state = false; // Default state, adjust as necessary



static uint8_t dev_uuid[16] = {0xdd, 0xdd};  // Example: Initialize with specific values if needed

// Forward declaration of callback functions
// Handler for receiving messages with sequence numbers
static int message_received(const struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx,
                             struct net_buf_simple *buf) {
    uint32_t seq_num = net_buf_simple_pull_le32(buf);
    static uint32_t last_seq_num = 0;
    static uint32_t missed_packets = 0;

    if (seq_num != last_seq_num + 1) {
        missed_packets += (seq_num - last_seq_num - 1);
    }

    printk("Received seq_num %u, Missed packets: %u\n", seq_num, missed_packets);

    last_seq_num = seq_num;
	return 0;
}

static int gen_onoff_get(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf);
static int gen_onoff_set(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf);
static int gen_onoff_set_unack(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf);

// Definition of the OnOff Client Model Operations
static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
    {OP_ONOFF_STATUS, BT_MESH_LEN_MIN(1), message_received},
    {OP_SEQ_NUMBER, BT_MESH_LEN_EXACT(4), message_received},  // Assuming the seq_num is exactly 4 bytes long
    BT_MESH_MODEL_OP_END,
};

static int onoff_status_send(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, bool onoff_state) {
    BT_MESH_MODEL_BUF_DEFINE(msg, OP_ONOFF_STATUS, 2);
    bt_mesh_model_msg_init(&msg, OP_ONOFF_STATUS);
    net_buf_simple_add_u8(&msg, onoff_state);

    return bt_mesh_model_send(model, ctx, &msg, NULL, NULL);
}

static int gen_onoff_get(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf) {
    // Here we assume `onoff_state` is a global or static variable tracking the OnOff state
    extern bool onoff_state;
    return onoff_status_send(model, ctx, onoff_state);
}

static int gen_onoff_set(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf) {
    bool new_state = net_buf_simple_pull_u8(buf);
    // Here we assume `onoff_state` is a global or static variable tracking the OnOff state
    extern bool onoff_state;
    onoff_state = new_state;

    // Optionally respond with the new state
    onoff_status_send(model, ctx, onoff_state);
    return 0;
}

static int gen_onoff_set_unack(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf) {
    bool new_state = net_buf_simple_pull_u8(buf);
    // Here we assume `onoff_state` is a global or static variable tracking the OnOff state
    extern bool onoff_state;
    onoff_state = new_state;
    return 0;
};
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



// Bluetooth ready callback
static void bt_ready(int err) {
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

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
}





int main(void) {
    
    int err;

    printk("Initializing...\n");

    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth enable failed (err %d)\n", err);
    }

    return 0;
}