/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.2
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "bleprph.h"
//#include "services/ans/ble_svc_ans.h"

/*** Maximum number of characteristics with the notify flag ***/

#define MAX_CONNECTIONS 2




/* Value handle storage for characteristics — must exist before initializer uses &... */
static uint16_t ctrl_rpm_handle;
static uint16_t ctrl_angle_handle;
static uint16_t ctrl_light_handle;
static uint16_t ctrl_power_handle;

static uint16_t stat_rpm_handle;
static uint16_t stat_angle_handle;
static uint16_t stat_light_handle;
static uint16_t stat_power_handle;



// Array to store up to two connection handles
static uint16_t conn_handles[MAX_CONNECTIONS] = {BLE_HS_CONN_HANDLE_NONE, BLE_HS_CONN_HANDLE_NONE};

// Utility to add a connection handle
int add_connection_handle(uint16_t conn_handle) {
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (conn_handles[i] == BLE_HS_CONN_HANDLE_NONE) {
            conn_handles[i] = conn_handle;
            return i;
        }
    }
    return -1; // No space
}

// Utility to remove a connection handle
void remove_connection_handle(uint16_t conn_handle) {
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (conn_handles[i] == conn_handle) {
            conn_handles[i] = BLE_HS_CONN_HANDLE_NONE;
        }
    }
}

// Utility to count active connections
int count_active_connections(void) {
    int count = 0;
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (conn_handles[i] != BLE_HS_CONN_HANDLE_NONE) {
            count++;
        }
    }
    return count;
}

// Utility to get index by connection handle
int get_connection_index(uint16_t conn_handle) {
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (conn_handles[i] == conn_handle) {
            return i;
        }
    }
    return -1;
}





static int
gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                struct ble_gatt_access_ctxt *ctxt,
                void *arg);






/* --- Add these global control/status variables (choose types you need) --- */
static uint32_t g_ctrl_rpm = 0;
static uint32_t g_ctrl_angle = 0;
static uint8_t  g_ctrl_light = 0;
static uint8_t  g_ctrl_power = 0;

static uint32_t g_stat_rpm = 0;
static uint32_t g_stat_angle = 0;
static uint8_t  g_stat_light = 0;
static uint8_t  g_stat_power = 0;

/* --- Simple accessors used by the read path (implement as you like) --- */
static inline uint32_t current_ctrl_rpm(void)   { return g_ctrl_rpm; }
static inline uint32_t current_ctrl_angle(void) { return g_ctrl_angle; }
static inline uint8_t  current_ctrl_light(void) { return g_ctrl_light; }
static inline uint8_t  current_ctrl_power(void) { return g_ctrl_power; }

static inline uint32_t current_stat_rpm(void)   { return g_stat_rpm; }
static inline uint32_t current_stat_angle(void) { return g_stat_angle; }
static inline uint8_t  current_stat_light(void) { return g_stat_light; }
static inline uint8_t  current_stat_power(void) { return g_stat_power; }

/* --- Simple "scheduler" stubs. In real code, post work to a task. --- */
static void schedule_set_rpm(uint32_t rpm)
{
    /* For demo: apply immediately and update status + notify subscribers */
    g_stat_rpm = rpm;
    /* stat_rpm_handle must be defined earlier in your file */
    ble_gatts_chr_updated(stat_rpm_handle);
}

static void schedule_set_angle(uint32_t angle)
{
    g_stat_angle = angle;
    ble_gatts_chr_updated(stat_angle_handle);
}

static void schedule_set_light(uint8_t light)
{
    g_stat_light = light;
    ble_gatts_chr_updated(stat_light_handle);
}

static void schedule_set_power(uint8_t power)
{
    g_stat_power = power;
    ble_gatts_chr_updated(stat_power_handle);
}







// Definitions of our two custom services

static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x2d, 0x71, 0xa2, 0x59, 0xb4, 0x58, 0xc8, 0x12,
                     0x99, 0x99, 0x43, 0x95, 0x12, 0x2f, 0x46, 0x59);

/* A characteristic that can be subscribed to */
static uint8_t gatt_svr_chr_val;
static uint16_t gatt_svr_chr_val_handle;
static const ble_uuid128_t gatt_svr_chr_uuid =
    BLE_UUID128_INIT(0x00, 0x00, 0x00, 0x00, 0x11, 0x11, 0x11, 0x11,
                     0x22, 0x22, 0x22, 0x22, 0x33, 0x33, 0x33, 0x33);

/* A custom descriptor */
static uint8_t gatt_svr_dsc_val;
static const ble_uuid128_t gatt_svr_dsc_uuid =
    BLE_UUID128_INIT(0x01, 0x01, 0x01, 0x01, 0x12, 0x12, 0x12, 0x12,
                     0x23, 0x23, 0x23, 0x23, 0x34, 0x34, 0x34, 0x34);



// Example 128-bit UUIDs for two custom services
static const ble_uuid128_t control_svc_uuid = BLE_UUID128_INIT(
    0xAA,0xAA,0xAA,0xAA, 0xAA,0xAA,0x11,0x00, 0x10,0x01,0x10,0x11, 0xAA,0xAA,0xAA,0xAA);
static const ble_uuid128_t status_svc_uuid  = BLE_UUID128_INIT(
    0xAA,0xAA,0xAA,0xAA, 0xAA,0xAA,0x32,0x43, 0x54,0x65,0x76,0x87, 0xAA,0xAA,0xAA,0xAA);

// Characteristic UUIDs for RPM, Angle, Light, Power (Control service)
static const ble_uuid128_t ctrl_rpm_uuid   = BLE_UUID128_INIT(
    0x01,0xC7,0xBE,0xEF, 0xEF,0xBE,0xAD,0xDE, 0x90,0xAB,0xCD,0xEF, 0xFE,0xDC,0xBA,0x98);
static const ble_uuid128_t ctrl_angle_uuid = BLE_UUID128_INIT(
    0x02,0xC7,0xBE,0xEF, 0xEF,0xBE,0xAD,0xDE, 0x90,0xAB,0xCD,0xEF, 0xFE,0xDC,0xBA,0x98); 
static const ble_uuid128_t ctrl_light_uuid = BLE_UUID128_INIT(
    0x03,0xC7,0xBE,0xEF, 0xEF,0xBE,0xAD,0xDE, 0x90,0xAB,0xCD,0xEF, 0xFE,0xDC,0xBA,0x98);
static const ble_uuid128_t ctrl_power_uuid = BLE_UUID128_INIT(
    0x04,0xC7,0xBE,0xEF, 0xEF,0xBE,0xAD,0xDE, 0x90,0xAB,0xCD,0xEF, 0xFE,0xDC,0xBA,0x98);
// Characteristic UUIDs for RPM, Angle, Light, Power (Status service)
static const ble_uuid128_t stat_rpm_uuid   = BLE_UUID128_INIT(
    0x01,0x57,0xBE,0xEF, 0xEF,0xBE,0xAD,0xDE, 0x90,0xAB,0xCD,0xEF, 0xFE,0xDC,0xBA,0x98);
static const ble_uuid128_t stat_angle_uuid = BLE_UUID128_INIT(
    0x02,0x57,0xBE,0xEF, 0xEF,0xBE,0xAD,0xDE, 0x90,0xAB,0xCD,0xEF, 0xFE,0xDC,0xBA,0x98);
static const ble_uuid128_t stat_light_uuid = BLE_UUID128_INIT(
    0x03,0x57,0xBE,0xEF, 0xEF,0xBE,0xAD,0xDE, 0x90,0xAB,0xCD,0xEF, 0xFE,0xDC,0xBA,0x98);
static const ble_uuid128_t stat_power_uuid = BLE_UUID128_INIT(
    0x04,0x57,0xBE,0xEF, 0xEF,0xBE,0xAD,0xDE, 0x90,0xAB,0xCD,0xEF, 0xFE,0xDC,0xBA,0x98);



static const struct ble_gatt_chr_def control_chrs[] = {
    { .uuid = &ctrl_rpm_uuid.u,   .access_cb = gatt_svc_access, .val_handle = &ctrl_rpm_handle,   .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },
    { .uuid = &ctrl_angle_uuid.u, .access_cb = gatt_svc_access, .val_handle = &ctrl_angle_handle, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },
    { .uuid = &ctrl_light_uuid.u, .access_cb = gatt_svc_access, .val_handle = &ctrl_light_handle, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },
    { .uuid = &ctrl_power_uuid.u, .access_cb = gatt_svc_access, .val_handle = &ctrl_power_handle, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },
    { 0 }  /* End of characteristics */
};
static const struct ble_gatt_chr_def status_chrs[] = {
    { .uuid = &stat_rpm_uuid.u,   .access_cb = gatt_svc_access, .val_handle = &stat_rpm_handle,   .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY },
    { .uuid = &stat_angle_uuid.u, .access_cb = gatt_svc_access, .val_handle = &stat_angle_handle, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY },
    { .uuid = &stat_light_uuid.u, .access_cb = gatt_svc_access, .val_handle = &stat_light_handle, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY },
    { .uuid = &stat_power_uuid.u, .access_cb = gatt_svc_access, .val_handle = &stat_power_handle, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY },
    { 0 }
};


//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------






static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /*** Service ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /*** This characteristic can be subscribed to by writing 0x00 and 0x01 to the CCCD ***/
                .uuid = &gatt_svr_chr_uuid.u,
                .access_cb = gatt_svc_access,
#if CONFIG_EXAMPLE_ENCRYPTION
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
                BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#else
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#endif
                .val_handle = &gatt_svr_chr_val_handle,
                .descriptors = (struct ble_gatt_dsc_def[])
                { {
                      .uuid = &gatt_svr_dsc_uuid.u,
#if CONFIG_EXAMPLE_ENCRYPTION
                      .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
#else
                      .att_flags = BLE_ATT_F_READ,
#endif
                      .access_cb = gatt_svc_access,
                    }, {
                      0, /* No more descriptors in this characteristic */
                    }
                },
            }, {
                0, /* No more characteristics in this service. */
            }
        },
    },

    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &control_svc_uuid.u,
        .characteristics = control_chrs,
    },

    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &status_svc_uuid.u,
        .characteristics = status_chrs,
    },

    {
        0, /* No more services. */
    },
};

static int
gatt_svr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
               void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}


/**
 * Access callback whenever a characteristic/descriptor is read or written to.
 * Here reads and writes need to be handled.
 * ctxt->op tells weather the operation is read or write and
 * weather it is on a characteristic or descriptor,
 * ctxt->dsc->uuid tells which characteristic/descriptor is accessed.
 * attr_handle give the value handle of the attribute being accessed.
 * Accordingly do:
 *     Append the value to ctxt->om if the operation is READ
 *     Write ctxt->om to the value if the operation is WRITE
 **/


static int
gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;

    /* Helpful debug log */
    MODLOG_DFLT(INFO, "gatt_access: op=%d conn=%d handle=%d\n",
                ctxt->op, conn_handle, attr_handle);

    switch (ctxt->op) {

    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (attr_handle == ctrl_rpm_handle) {
            uint32_t v = current_ctrl_rpm();
            rc = os_mbuf_append(ctxt->om, &v, sizeof(v));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (attr_handle == ctrl_angle_handle) {
            uint32_t v = current_ctrl_angle();
            rc = os_mbuf_append(ctxt->om, &v, sizeof(v));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (attr_handle == ctrl_light_handle) {
            uint8_t v = current_ctrl_light();
            rc = os_mbuf_append(ctxt->om, &v, sizeof(v));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (attr_handle == ctrl_power_handle) {
            uint8_t v = current_ctrl_power();
            rc = os_mbuf_append(ctxt->om, &v, sizeof(v));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (attr_handle == stat_rpm_handle) {
            uint32_t v = current_stat_rpm();
            rc = os_mbuf_append(ctxt->om, &v, sizeof(v));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (attr_handle == stat_angle_handle) {
            uint32_t v = current_stat_angle();
            rc = os_mbuf_append(ctxt->om, &v, sizeof(v));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (attr_handle == stat_light_handle) {
            uint8_t v = current_stat_light();
            rc = os_mbuf_append(ctxt->om, &v, sizeof(v));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (attr_handle == stat_power_handle) {
            uint8_t v = current_stat_power();
            rc = os_mbuf_append(ctxt->om, &v, sizeof(v));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        /* Unknown read - return an error (do not assert) */
        return BLE_ATT_ERR_UNLIKELY;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (attr_handle == ctrl_rpm_handle) {
            /* expect 4 bytes uint32_t */
            rc = gatt_svr_write(ctxt->om, sizeof(uint32_t), sizeof(uint32_t), &g_ctrl_rpm, NULL);
            if (rc == 0) {
                /* schedule hardware action; notify on completion */
                schedule_set_rpm(g_ctrl_rpm);
            }
            return rc;
        }
        if (attr_handle == ctrl_angle_handle) {
            rc = gatt_svr_write(ctxt->om, sizeof(uint32_t), sizeof(uint32_t), &g_ctrl_angle, NULL);
            if (rc == 0) schedule_set_angle(g_ctrl_angle);
            return rc;
        }
        if (attr_handle == ctrl_light_handle) {
            rc = gatt_svr_write(ctxt->om, sizeof(uint8_t), sizeof(uint8_t), &g_ctrl_light, NULL);
            if (rc == 0) schedule_set_light(g_ctrl_light);
            return rc;
        }
        if (attr_handle == ctrl_power_handle) {
            rc = gatt_svr_write(ctxt->om, sizeof(uint8_t), sizeof(uint8_t), &g_ctrl_power, NULL);
            if (rc == 0) schedule_set_power(g_ctrl_power);
            return rc;
        }

        /* Writes to status chars are not permitted */
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;

    case BLE_GATT_ACCESS_OP_READ_DSC:
        /* If you have a custom descriptor to serve, check its handle/uuid here.
           For now, only serve the example custom descriptor gatt_svr_dsc_val. */
        if (arg && ctxt->dsc && ble_uuid_cmp(ctxt->dsc->uuid, &gatt_svr_dsc_uuid.u) == 0) {
            rc = os_mbuf_append(ctxt->om, &gatt_svr_dsc_val, sizeof(gatt_svr_dsc_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return BLE_ATT_ERR_UNLIKELY;

    case BLE_GATT_ACCESS_OP_WRITE_DSC:
        /* Accept descriptor writes (this covers CCCD writes from clients).
           NimBLE handles CCCD state automatically. Return success. */
        return 0;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}



void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

int
gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    //ble_svc_ans_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    /* Setting a value for the read-only descriptor */
    gatt_svr_dsc_val = 0x99;

    return 0;
}
