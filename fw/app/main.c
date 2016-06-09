/* 
 *  Copyright (c) 2016 Robin Callender. All Rights Reserved.
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "config.h"

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf_adc.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "app_scheduler.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_timer_appsh.h"
#include "ble_error_log.h"
#include "device_manager.h" 
#include "app_gpiote.h"
#include "ble_debug_assert_handler.h"
#include "pstorage.h"

#include "boards.h"
#include "pstorage_platform.h"
#include "ble_oled.h"
#include "ssd130x.h"
#include "dbglog.h"
#include "uart.h"

#ifdef BLE_DFU_APP_SUPPORT
    #include "ble_dfu.h"
    #include "dfu_app_handler.h"

    #define IS_SRVC_CHANGED_CHARACT_PRESENT true
#else
    #define IS_SRVC_CHANGED_CHARACT_PRESENT false
#endif

/* Security options */
static ble_gap_sec_params_t      m_sec_params;              /* Security requirements for this application. */

static uint16_t                  m_conn_handle = BLE_CONN_HANDLE_INVALID;    /* Handle of the current connection. */


/* OLED Service Handle */
ble_oled_t                       g_oled_service;

uint8_t                          g_tx_buffer_count_max;

static app_timer_id_t            m_advertising_timer_id;    /* Timer for toggling awaiting-pairing LED */

static dm_application_instance_t m_app_handle;              /* Application identifier allocated by device manager. */

static bool  m_memory_access_in_progress = false;           /* Flag to keep track of ongoing operations on persistent memory. */

#ifdef BLE_DFU_APP_SUPPORT
static ble_dfu_t                  m_dfus;
#endif

/* Battery Service related  */
static ble_bas_t                 m_bas;                    /* Structure used to identify the battery service. */
static app_timer_id_t            m_battery_timer_id;       /* Battery timer. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS     1200    /* Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION      3       /* The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS    270     /* Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */

#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 255) * ADC_PRE_SCALING_COMPENSATION)

/*
 *  Function for error handling, which is called when an error has occurred.
 *
 *  warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 *  param[in] error_code  Error code supplied to the handler.
 *  param[in] line_num    Line number where the handler is called.
 *  param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
#if defined(DEBUG)
    PRINTF("app_error_handler: 0x%x, %d, %s", (unsigned)error_code, (int)line_num, (char*)p_file_name);

    __disable_irq();
    __BKPT(0);
    while (1) { /* spin */}

#else
    // On assert, the system can only recover with a reset.   
    NVIC_SystemReset();
#endif
}


/*
 *  Callback function for asserts in the SoftDevice.
 *
 *  details This function will be called in case of an assert in the SoftDevice.
 *
 *  warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 *  warning On assert from the SoftDevice, the system can only recover on reset.
 *
 *  param[in]   line_num   Line number of the failing ASSERT call.
 *  param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void ADC_IRQHandler(void)
{
    if (nrf_adc_conversion_finished()) {
        uint8_t  adc_result;
        uint16_t batt_lvl_in_milli_volts;
        uint8_t  percentage_batt_lvl;
        uint32_t err_code;

        nrf_adc_conversion_event_clean();

        adc_result = nrf_adc_result_get();

        batt_lvl_in_milli_volts = 
            ADC_RESULT_IN_MILLI_VOLTS(adc_result) + DIODE_FWD_VOLT_DROP_MILLIVOLTS;

        percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);

        err_code = ble_bas_battery_level_update(&m_bas, percentage_batt_lvl);
        
        if ((err_code != NRF_SUCCESS)             &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
        {
            APP_ERROR_HANDLER(err_code);
        }

        //PRINTF("battery: %d%%\n", (int)percentage_batt_lvl);
    }
}

/**@brief Function for configuring ADC to do battery level conversion.
 */
static void adc_configure(void)
{
    uint32_t err_code;
    nrf_adc_config_t adc_config = NRF_ADC_CONFIG_DEFAULT;

    // Configure ADC
    adc_config.reference  = NRF_ADC_CONFIG_REF_VBG;
    adc_config.resolution = NRF_ADC_CONFIG_RES_8BIT;
    adc_config.scaling    = NRF_ADC_CONFIG_SCALING_SUPPLY_ONE_THIRD;
    nrf_adc_configure(&adc_config);

    // Enable ADC interrupt
    nrf_adc_int_enable(ADC_INTENSET_END_Msk);
    err_code = sd_nvic_ClearPendingIRQ(ADC_IRQn);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_EnableIRQ(ADC_IRQn);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    nrf_adc_start();
}

/*
 *  Function for receiving Battery Service events.
 */
void ble_bas_evt_handler(ble_bas_t * p_bas, ble_bas_evt_t * p_evt)
{
    uint32_t err_code;

    switch (p_evt->evt_type) {

        case BLE_BAS_EVT_NOTIFICATION_ENABLED:
            //puts("battatery notification enabled");
            err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_BAS_EVT_NOTIFICATION_DISABLED:
            //puts("battery notification disabled");
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // don't care about other events.
            break;
    }
}

#ifdef USE_BATTERY_SIMULATOR
/*
 *  Function for initializing the battery simulator.
 */
static void battery_sim_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    ble_sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
}
#endif
  
/*
 *  Function for initializing the Battery Service.
 */
static void battery_service_init(void)
{
    uint32_t       err_code;
    ble_bas_init_t bas_init;
 
#ifdef USE_BATTERY_SIMULATOR
    {
        m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
        m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
        m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
        m_battery_sim_cfg.start_at_max = true;

        ble_sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
    }
#endif

    // Initialize Battery Service
    memset(&bas_init, 0, sizeof(bas_init));

    // Set security level for the Battery Service.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = ble_bas_evt_handler;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    puts("Battery Service");
}

/*
 *  Function for initializing the Device Service.
 */
static void device_info_service_init(void)
{
    uint32_t       err_code;
    ble_dis_init_t dis_init;

    // Initialize Device Information Service
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

    puts("Device Info Service");
}

/*
 *  Function for the LEDs initialization.
 *
 *  details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(ADVERTISING_LED_PIN);
    nrf_gpio_cfg_output(CONNECTED_LED_PIN);

    LED_OFF(ADVERTISING_LED_PIN);
    LED_OFF(CONNECTED_LED_PIN);
}

/*
 *  Function for the GAP initialization.
 *
 *  details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    /* Dynamically build-up MAC address and Device Name */
    {
        char device_name [24];
        ble_gap_addr_t mac = { 0 };

        err_code = sd_ble_gap_address_get( &mac );
        APP_ERROR_CHECK(err_code);

        /*  Device name unique-ifier: Use lower 3 bytes of MAC 
         *  as extension to device name. 
         */
        sprintf(device_name, "%s_%02X%02X%02X", DEVICE_NAME,
                mac.addr[2], mac.addr[1], mac.addr[0] );

        PRINTF("device name: %s\n", device_name);

        err_code = sd_ble_gap_device_name_set(&sec_mode,
                                              (const uint8_t *) &device_name,
                                              strlen(device_name));
        APP_ERROR_CHECK(err_code);
    }

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/*
 *  Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/*
 *  Function for handling the Connection Parameters Module.
 *
 *  details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          note: All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 *  param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/*
 *  Function for handling a Connection Parameters error.
 *
 *  param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/*
 *  Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/*
 *  Function for initializing the Advertising functionality.
 *
 *  details  Encodes the required advertising data and passes it to the stack.
 *           Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t       flags;

    puts("advertising_init");

    /* Set flags according to whether a non-zero advertisement (DISCovery) timeout is given */
    flags = (APP_ADV_TIMEOUT_IN_SECONDS == 0) ?
                  BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE :
                       BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    ble_uuid_t scan_uuids[] = {
        {OLED_UUID_SERVICE,                   g_oled_service.uuid_type},
        {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
    };

    /* Build ADVertisement data */
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = flags;

    /* Build SCAN response data */
    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(scan_uuids) / sizeof(scan_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = scan_uuids;

    /* Set ADV and SCAN data */
    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}

/*
 *  Function to start the advertising timer and blink the LED.
 */
static void start_advertising_blinking(void)
{
    uint32_t err_code;

    // Start advertising LED timer
    err_code = app_timer_start(m_advertising_timer_id,
                               ADVERT_BLINK_INTERVAL_LONG,
                               (void*)ADVERT_BLINK_INTERVAL_LONG);
    APP_ERROR_CHECK(err_code);
}

/*
 *  Function to stop the advertising timer and clear the LED.
 */
static void stop_advertising_blinking(void)
{
    uint32_t err_code;

    // Stop advertising timer
    err_code = app_timer_stop(m_advertising_timer_id);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_clear(ADVERTISING_LED_PIN);
}

/*
 *  Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    {
        uint32_t    count;

        // Verify if there is any flash access pending, if yes delay starting 
        // advertising until it's complete.
        err_code = pstorage_access_status_get(&count);
        APP_ERROR_CHECK(err_code);

        if (count != 0) {
            m_memory_access_in_progress = true;
            return;
        }
    }

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);

    start_advertising_blinking();
}

/*
 *  Function
 */
void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

    stop_advertising_blinking();
}

/*
 *  Function for handling the advertising timer timeout.
 *  This function will be to toggle the awaiting-pairing LED.
 *
 *  param[in]   p_context   Pointer used for passing some arbitrary information 
 *                          (context) from the app_start_timer() call to the 
 *                          timeout handler.
 */
static void advertising_timeout_handler(void * last_interval)
{
    uint32_t err_code;
    uint32_t next_interval;

    // Derive new time interval, and set LED accordingly.
    switch ((uint32_t) last_interval) {

        case ADVERT_BLINK_INTERVAL_SHORT:
            next_interval = ADVERT_BLINK_INTERVAL_LONG;
            LED_OFF(ADVERTISING_LED_PIN);
            break;

        default:
            PRINTF("Bad time interval: %d\n", (int) last_interval);
            // intentional fall through (try to recover)

        case ADVERT_BLINK_INTERVAL_LONG:
            next_interval = ADVERT_BLINK_INTERVAL_SHORT;
            LED_ON(ADVERTISING_LED_PIN);
            break;
    }

    // Restart advertising timer with new time interval.
    err_code = app_timer_start(m_advertising_timer_id,
                               next_interval, 
                               (void*)next_interval);
    APP_ERROR_CHECK(err_code); 
}

#ifdef BLE_DFU_APP_SUPPORT
/*
 *  On confirm of Service Changed, start bootloader.
 */
static void service_changed_evt(ble_evt_t * p_ble_evt)
{
    if (p_ble_evt->header.evt_id == BLE_GATTS_EVT_SC_CONFIRM) {
        PUTS("Service Changed confirmed");

        // Starting the bootloader - will cause reset.
        bootloader_start(m_conn_handle);
    }
}

/*
 *  DFU BLE Reset Prepare
 */
static void reset_prepare(void)
{
    PUTS("reset_prepare");

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        // Disconnect from peer.
        uint8_t status = BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION;

        APP_ERROR_CHECK( sd_ble_gap_disconnect(m_conn_handle, status) );
    }
    else {
        // If not connected, then the device will be advertising. 
        // Hence stop the advertising.
        advertising_stop();
    }

    stop_advertising_blinking();

    APP_ERROR_CHECK( ble_conn_params_stop() );
}

/*
 *  Function for initializing the DFU Service.
 */
static void dfu_init(void)
{
    ble_dfu_init_t   dfus_init;

    /* Initialize the Device Firmware Update Service. */
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler    = dfu_app_on_dfu_evt;
    dfus_init.error_handler  = NULL;

    APP_ERROR_CHECK( ble_dfu_init(&m_dfus, &dfus_init) );

    dfu_app_reset_prepare_set(reset_prepare);
}
#endif // BLE_DFU_APP_SUPPORT


/*
 *  Function for the Timer initialization.
 *
 *  details Initializes the timer module.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    err_code = app_timer_create(&m_advertising_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                advertising_timeout_handler);
    APP_ERROR_CHECK(err_code); 

    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

/*
 * 
 */
static uint32_t rounded_div(uint64_t a, uint64_t b)
{
    return ((a + (b/2)) / b);
}

/*
 *  Does APP_TIMER_TICKS() macro as function
 */
uint32_t app_timer_ticks(uint32_t ms)
{
    uint32_t ticks;
    ticks = ((uint32_t) rounded_div((ms) * (uint64_t)APP_TIMER_CLOCK_FREQ,
                                    ((APP_TIMER_PRESCALER) + 1) * 1000));
    return ticks;
}

/*
 *  Function for handling the Application's BLE Stack events.
 *
 *  param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;

    static ble_gap_evt_auth_status_t m_auth_status;
    static ble_gap_enc_key_t         m_enc_key;
    static ble_gap_id_key_t          m_id_key;
    static ble_gap_sign_info_t       m_sign_key;
    static ble_gap_sec_keyset_t      m_keys = {.keys_periph = {&m_enc_key, &m_id_key, &m_sign_key}};


    switch (p_ble_evt->header.evt_id) {

        case BLE_GAP_EVT_CONNECTED:

            puts("on_ble_evt: CONNECTED");

            // Stop advertising timer
            stop_advertising_blinking();

#if defined(INDICATE_CONNECT)
            LED_ON(CONNECTED_LED_PIN);
#endif
            LED_OFF(ADVERTISING_LED_PIN);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:

            puts("on_ble_evt: DISCONNECTED");

#if defined(INDICATE_CONNECT)
            LED_OFF(CONNECTED_LED_PIN);
#endif
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &m_sec_params,
                                                   &m_keys);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            {
                uint32_t flags = BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS | 
                                 BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS;

                err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, flags);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;

        case BLE_GAP_EVT_SEC_INFO_REQUEST:
          {
            bool                    master_id_matches;
            ble_gap_sec_kdist_t *   p_distributed_keys;
            ble_gap_enc_info_t *    p_enc_info;
            ble_gap_irk_t *         p_id_info;
            ble_gap_sign_info_t *   p_sign_info;

            master_id_matches  = memcmp(&p_ble_evt->evt.gap_evt.params.sec_info_request.master_id,
                                        &m_enc_key.master_id,
                                        sizeof(ble_gap_master_id_t)) == 0;

            p_distributed_keys = &m_auth_status.kdist_periph;

            p_enc_info  = (p_distributed_keys->enc  && master_id_matches) ? &m_enc_key.enc_info : NULL;
            p_id_info   = (p_distributed_keys->id   && master_id_matches) ? &m_id_key.id_info   : NULL;
            p_sign_info = (p_distributed_keys->sign && master_id_matches) ? &m_sign_key         : NULL;

            err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, p_id_info, p_sign_info);
            APP_ERROR_CHECK(err_code);
            break;
          }

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING) {
                
                LED_OFF(ADVERTISING_LED_PIN);
                
                // Go to system-off mode (this function will not return; wakeup will cause a reset)  
                puts("system power off");
                APP_ERROR_CHECK( sd_power_system_off() );
            }
            break;

        case BLE_EVT_TX_COMPLETE:
            break;

        default:
            // No implementation needed.
            break;
    }
}

/*
 *  Function for handling the Application's system events.
 */
static void on_sys_evt(uint32_t sys_evt)
{
    switch (sys_evt) {

        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
            if (m_memory_access_in_progress) {
                m_memory_access_in_progress = false;
                advertising_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}

/*
 *  Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 *  details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 *  param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_oled_on_ble_evt(&g_oled_service, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    dm_ble_evt_handler(p_ble_evt);

#ifdef BLE_DFU_APP_SUPPORT
    service_changed_evt(p_ble_evt);
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
#endif

    on_ble_evt(p_ble_evt);
}

/*
 *  Function for dispatching a system event to interested modules.
 *
 *  details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 *  param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
}

/*
 *  Function for initializing services that will be used by the application.
 */
static void services_init(void)
{    
    battery_service_init();
    device_info_service_init();

#ifdef BLE_DFU_APP_SUPPORT
    dfu_init();
#endif

    APP_ERROR_CHECK( ble_oled_init(&g_oled_service) );
}

/*
 *  Function for initializing the BLE stack.
 *
 *  details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ble_gap_addr_t      addr;
    ble_enable_params_t ble_enable_params;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Enable BLE stack 
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    APP_ERROR_CHECK( sd_ble_enable(&ble_enable_params) );

    sd_ble_gap_address_get(&addr);
    sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &addr);

    // Subscribe for BLE events.
    APP_ERROR_CHECK( softdevice_ble_evt_handler_set(ble_evt_dispatch) );

    // Register with the SoftDevice handler module for BLE events.
    APP_ERROR_CHECK( softdevice_sys_evt_handler_set(sys_evt_dispatch) );

    // Get TX buffer count.
    APP_ERROR_CHECK( sd_ble_tx_buffer_count_get(&g_tx_buffer_count_max) );
}

/*
 *  Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/*
 *  Function for initializing the GPIOTE handler module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}

/*
 *
 */
uint32_t service_changed_indicate(void)
{
    uint32_t  err_code;

    err_code = sd_ble_gatts_service_changed(m_conn_handle,
                                            APP_SERVICE_HANDLE_START,
                                            BLE_HANDLE_MAX);
    switch (err_code) {

        case NRF_SUCCESS:
            PUTS("Service Changed indicated"); 
            break;

        case BLE_ERROR_INVALID_CONN_HANDLE:
        case NRF_ERROR_INVALID_STATE:
        case BLE_ERROR_NO_TX_BUFFERS:
        case NRF_ERROR_BUSY:
        case BLE_ERROR_GATTS_SYS_ATTR_MISSING:
            PRINTF("service_changed minor error: %u\n", (unsigned)err_code); 
            break;

        case NRF_ERROR_NOT_SUPPORTED:
            PUTS("service_changed not supported");
            break;

        default:
            APP_ERROR_HANDLER(err_code);
            break;
    }

    return err_code;
}


/*
 *  Function for loading application-specific context after establishing a 
 *  secure connection.
 *
 *  This function will load the application context and check if the ATT table 
 *  is marked as changed. 
 *  If the ATT table is marked as changed, a Service Changed Indication
 *  is sent to the peer if the Service Changed CCCD is set to indicate.
 *
 *  param[in] p_handle The Device Manager handle that identifies the connection
 *                     for which the context should be loaded.
 */
static void app_context_load(dm_handle_t const * p_handle)
{
    uint32_t                 err_code;
    static uint32_t          context_data;
    dm_application_context_t context;

    context.len    = sizeof(context_data);
    context.p_data = (uint8_t *)&context_data;

    err_code = dm_application_context_get(p_handle, &context);
    if (err_code == NRF_SUCCESS) {

        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0) {

            err_code = sd_ble_gatts_service_changed(m_conn_handle, 
                                                    APP_SERVICE_HANDLE_START, 
                                                    BLE_HANDLE_MAX);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)) {

                APP_ERROR_HANDLER(err_code);
            }
        }

        APP_ERROR_CHECK( dm_application_context_delete(p_handle) );
    }
    else if (err_code == DM_NO_APP_CONTEXT) {
        // No context available. Ignore.
    }
    else {
        APP_ERROR_HANDLER(err_code);
    }
}

/*
 *  Function for handling the Device Manager events.
 *  param[in]   p_evt   Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           uint32_t            event_result)
{
    APP_ERROR_CHECK(event_result);

    switch (p_event->event_id) {

        case DM_EVT_LINK_SECURED:
            puts("LINK SECURED event");

#ifdef BLE_DFU_APP_SUPPORT
            app_context_load(p_handle);
#endif
            break;

        default:
            // No implementation needed.
            break;
    }

    return NRF_SUCCESS;
}

/*
 *  Function for the Device Manager initialization.
 */
static void device_manager_init(void)
{
    dm_init_param_t         init_data;
    dm_application_param_t  register_param;

    memset(&init_data, 0, sizeof(init_data));

#if 0
    // Clear all bonded centrals if the Bonds Delete button is pushed.
    init_data.clear_persistent_data = (nrf_gpio_pin_read(TEMP_PAIR_BUTTON_PIN) == 0);
#endif

    APP_ERROR_CHECK( dm_init(&init_data) );

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));
    
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    APP_ERROR_CHECK( dm_register(&m_app_handle, &register_param) );
}

/*
 *  Function for the Power manager.
 */
static void power_manage(void)
{
    APP_ERROR_CHECK( sd_app_evt_wait() );
}

/*
 *  Function for application main entry.
 */
int main(void)
{
    ble_stack_init();
    scheduler_init();

    // Initialize

#if defined(DBGLOG_SUPPORT)
    uart_init();
#endif

    PRINTF("\n*** OLED built: %s %s ***\n\n", __DATE__, __TIME__);

    APP_ERROR_CHECK( pstorage_init() );

    leds_init();
    timers_init();
    gpiote_init();
    ssd130x_init();
    adc_configure();

#if 1 // set to 0 for development/debugging
    PUTS("Starting BLE services");

    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    sec_params_init();

    device_manager_init();

    // Start execution
    advertising_start();
#endif

    // Enter main loop
    for (;;) {
        app_sched_execute();
        power_manage();
    }
}
