/* 
 *  ble_oled.h
 *  Copyright (c) 2016 Robin Callender. All Rights Reserved.
 */

#ifndef BLE_OLED_H__
#define BLE_OLED_H__

#include <stdint.h>
#include <stdbool.h>

#include "ble.h"
#include "ble_srv_common.h"

//
//  0000XXXX-0eea-67d1-a44a-bb3ce36fced6
// 
#define OLED_UUID_BASE {0xd6, 0xce, 0x6f, 0xe3, 0x3c, 0xbb, 0x4a, 0xa4, 0xd1, 0x67, 0xea, 0x0e, 0x00, 0x00, 0x00, 0x00}
#define OLED_UUID_SERVICE            0xfad0
#define OLED_UUID_DISPLAY_CHAR       0xfad1

// Forward declaration of the ble_oled_t type. 
typedef struct _ble_oled   ble_oled_t;

/*  OLED Service structure. 
 *  This contains various status information for the service. 
 */
typedef struct _ble_oled {
    uint16_t                       service_handle;
    ble_gatts_char_handles_t       display_char_handles;
    uint8_t                        uuid_type;
    uint16_t                       conn_handle; 
} ble_oled_t;


/* OLED Service Handle */
extern ble_oled_t  g_oled_service;


/*  Function for initializing the OLED Service.
 *
 * @param[out]  p_oled       OLED Service structure. This structure will have to be supplied by
 *                           the application. It will be initialized by this function, and will later
 *                           be used to identify this particular service instance.
 * @param[in]   p_oled_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_oled_init(ble_oled_t * p_oled);

/*  Function for handling the Application's BLE Stack events.
 *  Handles all events from the BLE stack of interest to the LED Button Service.
 *
 *   param[in]   p_oled     OLED Service structure.
 *   param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_oled_on_ble_evt(ble_oled_t * p_oled, ble_evt_t * p_ble_evt);


#endif // BLE_OLED_H__
