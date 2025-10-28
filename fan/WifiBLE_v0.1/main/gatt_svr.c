/* gatt_svr.c
 * Modified: add a unified 128-byte "packet" characteristic that accepts
 * JSON-like packets for both control and Wi-Fi provisioning.
 *
 * Assumes wifi_cred_queue is defined/created by your wifi_manager.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "bleprph.h"

//#include "os_mbuf.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

/* wifi_cred.h defines wifi_credentials_t */
#include "wifi_cred.h"

//extern QueueHandle_t wifi_cred_queue;


static const char *TAG = "gatt_svr";

/*** Maximum number of characteristics with the notify flag ***/
#define MAX_CONNECTIONS 2

/* existing control/status handles (unchanged) */
static uint16_t ctrl_rpm_handle;
static uint16_t ctrl_angle_handle;
static uint16_t ctrl_light_handle;
static uint16_t ctrl_power_handle;

static uint16_t stat_rpm_handle;
static uint16_t stat_angle_handle;
static uint16_t stat_light_handle;
static uint16_t stat_power_handle;

/* NEW: packet characteristic handle */
static uint16_t packet_handle;

/* connection handle tracking (your previous code) */
static uint16_t conn_handles[MAX_CONNECTIONS] = {BLE_HS_CONN_HANDLE_NONE, BLE_HS_CONN_HANDLE_NONE};
int add_connection_handle(uint16_t conn_handle) {
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (conn_handles[i] == BLE_HS_CONN_HANDLE_NONE) {
            conn_handles[i] = conn_handle;
            return i;
        }
    }
    return -1;
}
void remove_connection_handle(uint16_t conn_handle) {
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (conn_handles[i] == conn_handle) {
            conn_handles[i] = BLE_HS_CONN_HANDLE_NONE;
        }
    }
}
int count_active_connections(void) {
    int count = 0;
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (conn_handles[i] != BLE_HS_CONN_HANDLE_NONE) count++;
    }
    return count;
}
int get_connection_index(uint16_t conn_handle) {
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (conn_handles[i] == conn_handle) return i;
    }
    return -1;
}

/* --- Global control/status variables (unchanged) --- */
static uint32_t g_ctrl_rpm = 0;
static uint32_t g_ctrl_angle = 0;
static uint8_t  g_ctrl_light = 0;
static uint8_t  g_ctrl_power = 0;

static uint32_t g_stat_rpm = 0;
static uint32_t g_stat_angle = 0;
static uint8_t  g_stat_light = 0;
static uint8_t  g_stat_power = 0;

/* Accessors */
static inline uint32_t current_ctrl_rpm(void)   { return g_ctrl_rpm; }
static inline uint32_t current_ctrl_angle(void) { return g_ctrl_angle; }
static inline uint8_t  current_ctrl_light(void) { return g_ctrl_light; }
static inline uint8_t  current_ctrl_power(void) { return g_ctrl_power; }

static inline uint32_t current_stat_rpm(void)   { return g_stat_rpm; }
static inline uint32_t current_stat_angle(void) { return g_stat_angle; }
static inline uint8_t  current_stat_light(void) { return g_stat_light; }
static inline uint8_t  current_stat_power(void) { return g_stat_power; }

/* schedule_* functions already in your code — they update status and notify */
static void schedule_set_rpm(uint32_t rpm)
{
    g_stat_rpm = rpm;
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

/* --- GATT UUIDs (unchanged + new packet UUID) --- */
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x2d,0x71,0xa2,0x59,0xb4,0x58,0xc8,0x12,0x99,0x99,0x43,0x95,0x12,0x2f,0x46,0x59);
static const ble_uuid128_t control_svc_uuid = BLE_UUID128_INIT(
    0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0x11,0x00,0x10,0x01,0x10,0x11,0xAA,0xAA,0xAA,0xAA);
static const ble_uuid128_t status_svc_uuid  = BLE_UUID128_INIT(
    0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0x32,0x43,0x54,0x65,0x76,0x87,0xAA,0xAA,0xAA,0xAA);




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

/* NEW: unified packet characteristic UUID */
static const ble_uuid128_t packet_uuid = BLE_UUID128_INIT(
    0x10,0x10,0x10,0x10, 0x10,0x10,0x10,0x10, 0x20,0x20,0x20,0x20, 0x30,0x30,0x30,0x30);

/* GATT server characteristic and descriptor handles */
static uint16_t gatt_svr_chr_val_handle;
static uint8_t gatt_svr_dsc_val;

static const ble_uuid128_t gatt_svr_chr_uuid =
    BLE_UUID128_INIT(0x00, 0x00, 0x00, 0x00, 0x11, 0x11, 0x11, 0x11,
                     0x22, 0x22, 0x22, 0x22, 0x33, 0x33, 0x33, 0x33);

/* A custom descriptor */
static uint8_t gatt_svr_dsc_val;
static const ble_uuid128_t gatt_svr_dsc_uuid =
    BLE_UUID128_INIT(0x01, 0x01, 0x01, 0x01, 0x12, 0x12, 0x12, 0x12,
                     0x23, 0x23, 0x23, 0x23, 0x34, 0x34, 0x34, 0x34);


// /* declare the control/status characteristic arrays (reuse your arrays) */
// extern const struct ble_gatt_chr_def control_chrs[];
// extern const struct ble_gatt_chr_def status_chrs[];

/* local helper: safe write of om into flat buffer */
static int
gatt_svr_write_flat(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
               void *dst, uint16_t *out_len)
{
    uint16_t om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    int rc = ble_hs_mbuf_to_flat(om, dst, max_len, out_len);
    if (rc != 0) return BLE_ATT_ERR_UNLIKELY;
    return 0;
}

/* ---------- Simple string parsers for your packet format ---------- */

/* find the value of a numeric key "Key: <num>" in `s`. Returns true if found. */
static bool extract_int_field(const char *s, const char *key, int *out)
{
    const char *p = strstr(s, key);
    if (!p) return false;
    p += strlen(key);
    // skip separators and spaces and possible colon
    while (*p && ( *p==' ' || *p==':' || *p==',' || *p=='{' || *p=='}')) p++;
    if (!*p) return false;
    // read integer
    int val;
    int scanned = sscanf(p, "%d", &val);
    if (scanned == 1) {
        *out = val;
        return true;
    }
    return false;
}

/* find the string value of a key "SSID: 'name'" or "SSID: \"name\"" or "SSID: name" */
static bool extract_str_field(const char *s, const char *key, char *out, size_t maxlen)
{
    const char *p = strstr(s, key);
    if (!p) return false;
    p += strlen(key);
    while (*p && ( *p==' ' || *p==':' || *p==',' || *p=='{' || *p=='}')) p++;
    if (!*p) return false;
    // Accept quoted or unquoted form
    char quote = 0;
    if (*p == '"' || *p == '\'') { quote = *p; p++; }
    size_t i = 0;
    while (*p && i+1 < maxlen) {
        if (quote) {
            if (*p == quote) break;
        } else {
            // unquoted: stop at comma or closing brace
            if (*p == ',' || *p == '}' ) break;
        }
        out[i++] = *p++;
    }
    out[i] = '\0';
    // trim trailing spaces
    while (i > 0 && (out[i-1] == ' ' || out[i-1] == '\r' || out[i-1] == '\n' || out[i-1] == '\t')) {
        out[--i] = '\0';
    }
    return i > 0;
}

/* ---------- Main GATT access handler (modified) ---------- */

static int
gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    MODLOG_DFLT(INFO, "gatt_access: op=%d conn=%d handle=%d\n",
                ctxt->op, conn_handle, attr_handle);

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        /* delegate to your existing read handlers (unchanged) */
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

        /* unknown read */
        return BLE_ATT_ERR_UNLIKELY;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        /* Control small numeric characteristic writes (unchanged behavior) */
        if (attr_handle == ctrl_rpm_handle) {
            rc = gatt_svr_write_flat(ctxt->om, sizeof(uint32_t), sizeof(uint32_t), &g_ctrl_rpm, NULL);
            if (rc == 0) schedule_set_rpm(g_ctrl_rpm);
            return rc;
        }
        if (attr_handle == ctrl_angle_handle) {
            rc = gatt_svr_write_flat(ctxt->om, sizeof(uint32_t), sizeof(uint32_t), &g_ctrl_angle, NULL);
            if (rc == 0) schedule_set_angle(g_ctrl_angle);
            return rc;
        }
        if (attr_handle == ctrl_light_handle) {
            rc = gatt_svr_write_flat(ctxt->om, sizeof(uint8_t), sizeof(uint8_t), &g_ctrl_light, NULL);
            if (rc == 0) schedule_set_light(g_ctrl_light);
            return rc;
        }
        if (attr_handle == ctrl_power_handle) {
            rc = gatt_svr_write_flat(ctxt->om, sizeof(uint8_t), sizeof(uint8_t), &g_ctrl_power, NULL);
            if (rc == 0) schedule_set_power(g_ctrl_power);
            return rc;
        }

        /* NEW: unified packet characteristic (text/JSON-like) */
        if (attr_handle == packet_handle) {
            /* maximum 128 bytes; expect at least 1 byte */
            const uint16_t MAX_PKT = 128;
            uint16_t got = 0;
            char buf[MAX_PKT + 1];
            memset(buf, 0, sizeof(buf));
            rc = gatt_svr_write_flat(ctxt->om, 1, MAX_PKT, buf, &got);
            if (rc != 0) return rc;

            buf[got] = '\0'; /* ensure nul-terminated for strstr/sscanf */

            ESP_LOGI(TAG, "Received packet (%u bytes): %s", (unsigned)got, buf);

            /* Decide packet type: control attributes or wifi provisioning.
               We look for tokens - "Wifi" or "SSID" to detect provisioning.
               Control keys: Speed, Angle, Light, Power
            */

            bool handled = false;

            /* If packet mentions Wifi or SSID -> treat as provisioning */
            if (strstr(buf, "Wifi") != NULL || strstr(buf, "SSID") != NULL || strstr(buf, "ssid") != NULL) {
                wifi_credentials_t cred;
                memset(&cred, 0, sizeof(cred));
                char ssid[33] = {0};
                char pass[65] = {0};
                bool got_ssid = extract_str_field(buf, "SSID", ssid, sizeof(ssid));
                if (!got_ssid) {
                    got_ssid = extract_str_field(buf, "ssid", ssid, sizeof(ssid));
                }
                bool got_pass = extract_str_field(buf, "PASS", pass, sizeof(pass));
                if (!got_pass) {
                    got_pass = extract_str_field(buf, "Pass", pass, sizeof(pass));
                    if (!got_pass) got_pass = extract_str_field(buf, "pass", pass, sizeof(pass));
                }

                if (got_ssid) {
                    cred.ssid_len = (uint8_t)strnlen(ssid, sizeof(cred.ssid));
                    strncpy(cred.ssid, ssid, sizeof(cred.ssid)-1);
                    if (got_pass) {
                        cred.pass_len = (uint8_t)strnlen(pass, sizeof(cred.pass));
                        strncpy(cred.pass, pass, sizeof(cred.pass)-1);
                    } else {
                        cred.pass_len = 0;
                        cred.pass[0] = '\0';
                    }

                    /* Send credentials to wifi manager via queue
                     * Note: this is safe from NimBLE host context.
                     */
                    extern QueueHandle_t wifi_cred_queue; /* must be created by wifi_manager_init() */
                    if (wifi_cred_queue) {
                        BaseType_t ok = xQueueSend(wifi_cred_queue, &cred, 0);
                        if (ok == pdTRUE) {
                            ESP_LOGI(TAG, "Provisioning queued SSID='%s' (len=%u) pass_len=%u", cred.ssid, cred.ssid_len, cred.pass_len);
                            handled = true;
                        } else {
                            ESP_LOGW(TAG, "wifi_cred_queue full or not available");
                            /* return an ATT error to the writer */
                            return BLE_ATT_ERR_UNLIKELY;
                        }
                    } else {
                        ESP_LOGE(TAG, "wifi_cred_queue not initialised");
                        return BLE_ATT_ERR_UNLIKELY;
                    }
                } else {
                    ESP_LOGW(TAG, "SSID not found in provisioning packet");
                    return BLE_ATT_ERR_UNLIKELY;
                }
            } else {
                /* Control packet: parse numeric fields and apply state actions directly.
                   We update g_ctrl_* variables and call schedule_set_* which notifies subscribers.
                   Because this callback runs on NimBLE host, calling schedule_set_*() is acceptable.
                */
                int val;
                if (extract_int_field(buf, "Speed", &val) || extract_int_field(buf, "speed", &val)) {
                    if (val >= 0) {
                        g_ctrl_rpm = (uint32_t)val;
                        schedule_set_rpm(g_ctrl_rpm);
                        handled = true;
                    }
                }
                if (extract_int_field(buf, "Angle", &val) || extract_int_field(buf, "angle", &val)) {
                    if (val >= 0) {
                        g_ctrl_angle = (uint32_t)val;
                        schedule_set_angle(g_ctrl_angle);
                        handled = true;
                    }
                }
                if (extract_int_field(buf, "Light", &val) || extract_int_field(buf, "light", &val)) {
                    if (val >= 0) {
                        g_ctrl_light = (uint8_t)val;
                        schedule_set_light(g_ctrl_light);
                        handled = true;
                    }
                }
                if (extract_int_field(buf, "Power", &val) || extract_int_field(buf, "power", &val)) {
                    if (val >= 0) {
                        g_ctrl_power = (uint8_t)val;
                        schedule_set_power(g_ctrl_power);
                        handled = true;
                    }
                }
            }

            if (!handled) {
                ESP_LOGW(TAG, "Packet not handled or no known keys found");
                /* returning an ATT error informs writer of failure; use 0 if you prefer success */
                return 0;
            }

            /* success */
            return 0;
        } /* end packet_handle case */

        /* write to status chars not permitted */
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;

    case BLE_GATT_ACCESS_OP_READ_DSC:
        if (arg && ctxt->dsc && ble_uuid_cmp(ctxt->dsc->uuid, &gatt_svr_dsc_uuid.u) == 0) {
            rc = os_mbuf_append(ctxt->om, &gatt_svr_dsc_val, sizeof(gatt_svr_dsc_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return BLE_ATT_ERR_UNLIKELY;

    case BLE_GATT_ACCESS_OP_WRITE_DSC:
        /* CCCD writes etc. */
        return 0;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/* register callback unchanged from your code */
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf_uuid[BLE_UUID_STR_LEN];
    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf_uuid),
                    ctxt->svc.handle);
        break;
    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf_uuid),
                    ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;
    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf_uuid),
                    ctxt->dsc.handle);
        break;
    default:
        assert(0);
    }
}

/* Build GATT services: insert the packet characteristic into an appropriate service.
   Here we append it to the existing primary service (gatt_svr_svc_uuid) for simplicity.
   You may instead add it to the control service.
*/

/* --- existing control and status arrays (you already have them) --- */

static const struct ble_gatt_chr_def control_chrs_local[] = {
    { .uuid = &ctrl_rpm_uuid.u,   .access_cb = gatt_svc_access, .val_handle = &ctrl_rpm_handle,   .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },
    { .uuid = &ctrl_angle_uuid.u, .access_cb = gatt_svc_access, .val_handle = &ctrl_angle_handle, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },
    { .uuid = &ctrl_light_uuid.u, .access_cb = gatt_svc_access, .val_handle = &ctrl_light_handle, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },
    { .uuid = &ctrl_power_uuid.u, .access_cb = gatt_svc_access, .val_handle = &ctrl_power_handle, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },

    /* NEW unified packet char - writable up to 128 bytes */
    {
        .uuid = &packet_uuid.u,
        .access_cb = gatt_svc_access,
        .val_handle = &packet_handle,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP
    },

    { 0 }  /* End */
};

static const struct ble_gatt_chr_def status_chrs_local[] = {
    { .uuid = &stat_rpm_uuid.u,   .access_cb = gatt_svc_access, .val_handle = &stat_rpm_handle,   .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY },
    { .uuid = &stat_angle_uuid.u, .access_cb = gatt_svc_access, .val_handle = &stat_angle_handle, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY },
    { .uuid = &stat_light_uuid.u, .access_cb = gatt_svc_access, .val_handle = &stat_light_handle, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY },
    { .uuid = &stat_power_uuid.u, .access_cb = gatt_svc_access, .val_handle = &stat_power_handle, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY },
    { 0 }
};

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &gatt_svr_chr_uuid.u,
                .access_cb = gatt_svc_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
                .val_handle = &gatt_svr_chr_val_handle,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = &gatt_svr_dsc_uuid.u,
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = gatt_svc_access,
                    }, {
                        0
                    }
                },
            }, {
                0
            }
        },
    },

    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &control_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def *)control_chrs_local,
    },

    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &status_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def *)status_chrs_local,
    },

    { 0 }
};

/* register and init are same as your original code */
int gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }
    /* your descriptor init */
    gatt_svr_dsc_val = 0x99;
    return 0;
}

