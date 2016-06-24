/* 
 *  ble_oled.c   BLE surface for OLED access.
 *  Copyright (c) 2016 Robin Callender. All Rights Reserved.
 */
#include <stdio.h>
#include <string.h>

#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_timer.h"
#include "app_timer_appsh.h"

#include "config.h"
#include "ble_oled.h"
#include "oled.h"
#include "dbglog.h"
#include "ble_dfu.h"

static char  m_display[MAX_DISPLAY_LENGTH];

/* 
 *  Function for handling the Connect event.
 *
 *  param[in]   p_oled      OLED structure.
 *  param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_oled_t * p_oled, ble_evt_t * p_ble_evt)
{
    p_oled->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/*
 *  Function for handling the Disconnect event.
 *
 *  param[in]   p_oled      OLED structure.
 *  param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_oled_t * p_oled, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_oled->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/** Function for handling write events to the Displasy characteristic.
 *
 *  param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_oled_display_write(ble_oled_t * p_oled, ble_gatts_evt_write_t * p_evt_write)
{
    char * string = (char*) &p_evt_write->data;

    if (strlen(string) >= MAX_DISPLAY_LENGTH) {
        return;
    }

    oled_puts(string);
}

/*
 *  Function for handling the Write event.
 *
 *  param[in]   p_oled      Temp structure.
 *  param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_oled_t * p_oled, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t  * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    switch (p_evt_write->context.char_uuid.uuid) {

        case OLED_UUID_DISPLAY_CHAR:
            if (p_evt_write->len != sizeof(uint16_t)) {
                //printf("on_write: bad len: %d\n", p_evt_write->len);
                break;
            }
            on_oled_display_write(p_oled, p_evt_write);
            break;


        case OLED_UUID_SERVICE:
        case BLE_UUID_BATTERY_LEVEL_CHAR:
        case BLE_UUID_GATT_CHARACTERISTIC_SERVICE_CHANGED:
        case BLE_DFU_CTRL_PT_UUID:
            // Quietly ignore these events.
            break;

        default:
            printf("unknown uuid: 0x%x\n",
                   (unsigned) p_evt_write->context.char_uuid.uuid);
            break;
    }  
}


/*
 *  Function for handling the BLE events.
 *
 *  param[in]   p_oled      Temp Service structure.
 *  param[in]   p_ble_evt   Event received from the BLE stack.
 */
void ble_oled_on_ble_evt(ble_oled_t * p_oled, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id) {

        case BLE_GAP_EVT_CONNECTED:
            PUTS("evt_id: CONNECTED");
            on_connect(p_oled, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_oled, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_oled, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            break;

        case BLE_GATTS_EVT_HVC:
            break;

        case BLE_GATTS_EVT_SC_CONFIRM:
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            break;

        default:
            break;
    }
}

/*
 *  Function for adding the Display characteristic.
 */
static uint32_t display_char_add(ble_oled_t * p_oled)
{
    ble_gatts_char_md_t  char_md;
    ble_gatts_attr_t     attr_char_value;
    ble_uuid_t           ble_uuid;
    ble_gatts_attr_md_t  attr_md;
    ble_gatts_char_pf_t  char_pf;
    ble_gatts_attr_md_t  desc_md;

    static const uint8_t user_desc[] = "Display";

    memset(&desc_md, 0, sizeof(desc_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&desc_md.read_perm);
    desc_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_pf, 0, sizeof(char_pf));
    char_pf.format = BLE_GATT_CPF_FORMAT_UTF8S;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read         = 1;
    char_md.char_props.write        = 1;
    char_md.p_char_user_desc        = (uint8_t*) &user_desc;
    char_md.char_user_desc_size     = sizeof(user_desc);
    char_md.char_user_desc_max_size = sizeof(user_desc);
    char_md.p_char_pf               = &char_pf;
    char_md.p_user_desc_md          = &desc_md;
    char_md.p_cccd_md               = NULL;
    char_md.p_sccd_md               = NULL;

    ble_uuid.type = p_oled->uuid_type;
    ble_uuid.uuid = OLED_UUID_DISPLAY_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_USER;              // NOTE using app storage
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(m_display);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(m_display);
    attr_char_value.p_value      = (uint8_t*) &m_display;

    return sd_ble_gatts_characteristic_add(p_oled->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_oled->display_char_handles);
}

/*
 *  Function for initializing OLED BLE usage.
 */
uint32_t ble_oled_init(ble_oled_t * p_oled)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_oled->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add service
    ble_uuid128_t base_uuid = {OLED_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_oled->uuid_type);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }
    
    ble_uuid.type = p_oled->uuid_type;
    ble_uuid.uuid = OLED_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, 
                                        &ble_uuid, 
                                        &p_oled->service_handle);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    err_code = display_char_add(p_oled);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    return NRF_SUCCESS;
}
