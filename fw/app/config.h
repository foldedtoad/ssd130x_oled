/* 
 *  Copyright (c) 2016 Robin Callender. All Rights Reserved.
 */

#ifndef _CONFIG_H__
#define _CONFIG_H__

#include <stdint.h>
#include "boards.h"

#define ADVERTISING_LED_PIN             BSP_LED_0                                   /* Is on when device is advertising. */
#define CONNECTED_LED_PIN               BSP_LED_1                                   /* Is on when device has connected. */

#define DEFAULT_TEMPERATURE_MEASURE_INTERVAL  60                                    /* Temperature measure interval is 60 secs default */

#define SSD130X_SCL                     7                                           /* I2C Clock pin for SSD130X device. */
#define SSD130X_SDA                     30                                          /* I2C Data pin for SSD130X device. */
#define SSD130X_RST                     0

#define DEVICE_NAME                     "OLED"                                      /* Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "maker"                                     /* Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                64                                          /* The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                           /* The advertising timeout (in units of seconds). (0 == no timeout) */
//#define APP_ADV_TIMEOUT_IN_SECONDS      20                                          /* The advertising timeout (in units of seconds). (0 == no timeout) */

#define ADVERT_BLINK_INTERVAL_SHORT     APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)   /* Advertising Blink interval (ticks). */
#define ADVERT_BLINK_INTERVAL_LONG      APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /* Advertising Blink interval (ticks). */

#define APP_TIMER_PRESCALER             0                                           /* Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            5                                           /* Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         5                                           /* Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /* Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /* Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /* Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /* Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /* Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /* Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /* Number of atoledts before giving up the connection parameter negotiation. */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /* Battery level measurement interval (ticks). */

#ifdef USE_BATTERY_SIMULATOR 
#define MIN_BATTERY_LEVEL               81                                          /* Minimum battery level as returned by the simulated measurement function. */
#define MAX_BATTERY_LEVEL               100                                         /* Maximum battery level as returned by the simulated measurement function. */
#define BATTERY_LEVEL_INCREMENT         1                                           /* Value by which the battery level is incremented/decremented for each call to the simulated measurement function. */
#endif  

#define APP_GPIOTE_MAX_USERS            3                                           /* Maximum number of users of the GPIOTE handler. */

#define SEC_PARAM_TIMEOUT               30                                          /* Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /* Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /* Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /* No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /* Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /* Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /* Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /* Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /* Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /* Maximum number of events in the scheduler queue. */

#define APP_SERVICE_HANDLE_START         0x000C                                     /* Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX                   0xFFFF                                     /* Max handle value in BLE. */

#define LED_ON(led)                     nrf_gpio_pin_clear(led)
#define LED_OFF(led)                    nrf_gpio_pin_set(led)

/*
 *  Prototypes and Externs exported from main.c
 */
uint32_t app_timer_ticks(uint32_t ms);
uint32_t service_changed_indicate(void);

#endif /* _CONFIG_H__ */
