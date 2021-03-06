/*
 * Copyright (c) 2017, JoonDong
 * All rights reserved.
 * Copyright holder's blog : http://joondong.tistory.com
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Copyright holder's blog address must bo located under the copyright notice.
 *
 * 3. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 4. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdint.h>
#include <string.h>
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_nds.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "sensorsim.h"
#include "softdevice_handler.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "app_timer_appsh.h"
#include "bsp.h"
#include "nrf_nvic.h"
#include "nrf_delay.h"
#include "nrf_adc.h"
#include "nrf_gpio.h"
#include "bsp_custom_1_btn_ble.h"
#include "peer_manager.h"
#include "fds.h"
#include "fstorage.h"
#include "nrf_ble_gatt.h"
#include "ble_conn_state.h"

#ifdef BLE_DFU_SUPPORT
	#include "ble_dfu.h"
#endif

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                /**< Include the Service Changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT             /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define CENTRAL_LINK_COUNT               0                                /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                      "Noise_Detector"                 /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "JoonDong"            			  /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 300                              /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
// TODO : 약 45초 정도?
#define APP_ADV_TIMEOUT_IN_SECONDS       60                              /**< The advertising time-out in units of seconds. */

#define APP_TIMER_PRESCALER              15                             /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(15000, APP_TIMER_PRESCALER)                             /**< Battery level measurement interval (ms). */
#define MAX_BATTERY_LEVEL                100                              /**< Maximum simulated battery level. */
#define MIN_BATTERY_LEVEL                1                               /**< Minimum simulated battery level. */
#define MAX_BATTERY_VOLTAGE				 209							/* About 4V (accept as 100%) */
#define MIN_BATTERY_VOLTAGE				 MAX_BATTERY_VOLTAGE - 64		/* About 2.77V (accept as 0%)*/
#define BATTERY_VOLTAGE_AVERAGE_COUNT	 4

#define NOISE_DETECTOR_MEAS_INTERVAL     APP_TIMER_TICKS(10, APP_TIMER_PRESCALER)  /**< Noise detector measurement interval (ms). */
#define MIN_NOISE_VALUE                   0                              /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_NOISE_VALUE                   255                              /**< Maximum heart rate as returned by the simulated measurement function. */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(40, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(80, UNIT_1_25_MS) /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                    0                                /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(8000, UNIT_10_MS)  /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   5000                             /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    30000                            /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE             /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                               /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                       /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define OSTIMER_WAIT_FOR_QUEUE           2                                /**< Number of ticks to wait for the timer queue to be ready */

#define APP_FEATURE_NOT_SUPPORTED        BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

static uint16_t  m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
static ble_bas_t m_bas;                                   /**< Structure used to identify the battery service. */
static ble_nds_t m_nds;                                   /**< Structure used to identify the heart rate service. */

#ifdef BLE_DFU_SUPPORT
	static ble_dfu_t m_dfus;							  /**< Structure used to identify the Device Firmware Update service. */
#endif

static nrf_ble_gatt_t m_gatt;                             /**< Structure for gatt module*/

APP_TIMER_DEF(m_battery_timer_id);                        /**< Battery timer. */
APP_TIMER_DEF(m_noise_detector_timer_id);                 /**< Noise Detector timer. */

//#define 	BATTERY_LEVEL_INPUT 	NRF_ADC_CONFIG_INPUT_3 // P0.02
//#define		NOISE_DETECTOR_INPUT	NRF_ADC_CONFIG_INPUT_4 // P0.03
#define 	BATTERY_LEVEL_INPUT 	NRF_ADC_CONFIG_INPUT_4 // P0.03
#define		NOISE_DETECTOR_INPUT	NRF_ADC_CONFIG_INPUT_5 // P0.04

#define 	MAX_DETECTED_NOISE_VALUE_LEN 		20
__volatile 	bool		m_is_battery_level_checked = true;		// This reduce overhead of configuring ADC.

__volatile bool m_is_advertising;

// This pin is connected to gate of P-ch power mosfet.
#define AMPLIFIER_SWITCH_PIN 	8 // p0.08 - on : 0, off : 1
//#define AMPLIFIER_SWITCH_PIN 	4 // p0.04 - on : 0, off : 1

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
	error_info_t* err_info = (error_info_t*)info;

    NRF_LOG_ERROR("err_code = %x \r\n", err_info->err_code);
    NRF_LOG_FINAL_FLUSH();

    // On assert, the system can only recover with a reset.
#ifndef DEBUG
    NVIC_SystemReset();
#else
    app_error_save_and_stop(id, pc, info);
#endif // DEBUG
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
    ret_code_t err_code;

    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    if(err_code == NRF_SUCCESS)
    	m_is_advertising = true;

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling File Data Storage events.
 *
 * @param[in] p_evt  Peer Manager event.
 * @param[in] cmd
 */
static void fds_evt_handler(fds_evt_t const * const p_evt)
{
    if (p_evt->id == FDS_EVT_GC)
    {
        NRF_LOG_DEBUG("GC completed\n");
    }
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.\r\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        	sd_nvic_SystemReset(); // TODO : 이게 최선?
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start();
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

#ifdef BLE_DFU_SUPPORT
/**@brief Function for handling Device Firmware Update events.
 *
 * @param[in] p_dfu  Structure used to identify the Device Firmware Update service.
 * @param[in] p_evt  Device Firmware Update event.
 */
static void ble_dfu_evt_handler(ble_dfu_t * p_dfu, ble_dfu_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case BLE_DFU_EVT_INDICATION_DISABLED:
            NRF_LOG_INFO("Indication for BLE_DFU is disabled\r\n");
            break;

        case BLE_DFU_EVT_INDICATION_ENABLED:
            NRF_LOG_INFO("Indication for BLE_DFU is enabled\r\n");
            break;

        case BLE_DFU_EVT_ENTERING_BOOTLOADER:
            NRF_LOG_INFO("Device is entering bootloader mode!\r\n");
            break;
        default:
            NRF_LOG_INFO("Unknown event from ble_dfu\r\n");
            break;
    }
}
#endif

/**@brief Function for handling Noise Detector events.
 *
 * @param[in] p_nds  Structure used to identify the Noise Detector service.
 * @param[in] p_evt  Noise Detector events.
 */
static uint32_t ble_nds_evt_handler(ble_nds_t * p_nds, ble_nds_evt_t * p_evt) {
	uint32_t err_code = NRF_SUCCESS;

	switch (p_evt->evt_type)
	{
		case BLE_DNV_EVT_NOTIFICATION_DISABLED:
			NRF_LOG_INFO("Indication for BLE_DNV is disabled\r\n");
			break;
		case BLE_DNV_EVT_NOTIFICATION_ENABLED:
			NRF_LOG_INFO("Indication for BLE_DNV is enabled\r\n");
			break;
		case BLE_NDCP_EVT_INDIFICATION_DISABLED:
			NRF_LOG_INFO("Indication for BLE_NDCP is disabled\r\n");
			break;
		case BLE_NDCP_EVT_INDIFICATION_ENABLED:
			NRF_LOG_INFO("Indication for BLE_NDCP is enabled\r\n");
			break;
		case BLE_NDCP_EVT_NOISE_DETECTOR_TIMER_DISABLE_REQUESTED:
		    // Noise detector timer stop
			nrf_gpio_pin_set(AMPLIFIER_SWITCH_PIN);
		    err_code = app_timer_stop(m_noise_detector_timer_id);
		    NRF_LOG_INFO("Noise Detector is disabled\r\n");
			break;
		case BLE_NDCP_EVT_NOISE_DETECTOR_TIMER_ENABLE_REQUESTED:
			// Noise detector timer start
			nrf_gpio_pin_clear(AMPLIFIER_SWITCH_PIN);
		    err_code = app_timer_start(m_noise_detector_timer_id, NOISE_DETECTOR_MEAS_INTERVAL, NULL);
		    NRF_LOG_INFO("Noise Detector is enabled\r\n");
			break;
	}
	return err_code;
}

/* @brief Function for changing adc configuration to detect both battery level and noise detector.
 *
 * @param[in] is_battery_level_checking		if true, setting for battery level.
 */
static void adc_config(bool is_battery_level_checking)
{
	nrf_adc_config_t config;

	if(is_battery_level_checking)
	{
		// setting ADC range from 0 to 1.2V and resolution to 8bits
		config.resolution = ADC_CONFIG_RES_8bit;
		config.scaling = NRF_ADC_CONFIG_SCALING_INPUT_FULL_SCALE;
		config.reference = ADC_CONFIG_REFSEL_VBG;
	}
	else
	{
		// setting ADC range from 0 to 1.8V and resolution to 8bits
		config.resolution = ADC_CONFIG_RES_8bit;
		config.scaling = NRF_ADC_CONFIG_SCALING_INPUT_TWO_THIRDS;
		config.reference = ADC_CONFIG_REFSEL_VBG;
	}
	nrf_adc_configure(&config);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code;

    // Prepare wakeup buttons.
    err_code = bsp_custom_1_btn_ble_setup_sleep_button();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

static uint8_t calculate_battery_level_64_resolution(int32_t battery_voltage, uint32_t* err_code)
{
	*err_code = NRF_SUCCESS;

	battery_voltage -= MIN_BATTERY_VOLTAGE; // 210 - 64 = 146

	if(battery_voltage > 48) // 64 - 49(16) : 100% - 15%
	{
		if(battery_voltage > 63)
			return 100;

		battery_voltage -= 48;
		return 15 + (((battery_voltage << 6) + (battery_voltage << 4) + (battery_voltage << 2) + battery_voltage) >> 4); // 15 + (battery_voltage x 85) ÷ 16
	}
	else if(battery_voltage > 16) // 48 - 17(32) : 15% - 2%
	{
		battery_voltage -= 16;
		return 2 + (((battery_voltage << 3) + (battery_voltage << 2) + battery_voltage) >> 5); // 2 + (battery_voltage x 13) ÷ 32
	}
	else if(battery_voltage > 0) // 16 - 1(16) : 2% - 0%
	{
		return (battery_voltage << 1) >> 4; // (battery_voltage x 2) ÷ 16
	}
	else // battery_voltage == 0
	{
		*err_code = NRF_ERROR_RESOURCES; // It means battery is discharged.
		return 0;
	}
}

/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t 	err_code;
    static uint32_t battery_voltage_buffer[BATTERY_VOLTAGE_AVERAGE_COUNT];
    static uint8_t buffer_index;
    int8_t		battery_level;

    adc_config(true);
    battery_voltage_buffer[buffer_index++] = nrf_adc_convert_single(BATTERY_LEVEL_INPUT) & 0xFF;
    m_is_battery_level_checked = true;
    NRF_LOG_INFO("battery_voltage_buffer[%d] = %d\r\n", buffer_index-1, battery_voltage_buffer[buffer_index-1]);

    if(buffer_index == BATTERY_VOLTAGE_AVERAGE_COUNT)
    {
    	int32_t tmp = 0;

    	buffer_index = 0;

    	for(int i=0; i<BATTERY_VOLTAGE_AVERAGE_COUNT; i++)
    	{
    		tmp += battery_voltage_buffer[i];
    	}

    	// TODO
    	battery_level = calculate_battery_level_64_resolution(tmp >> 2, &err_code); // tmp ÷ 4
    	//battery_level = tmp >> 2; // tmp ÷ 4
    	NRF_LOG_INFO("battery_level = %d\r\n", tmp);

    	if(err_code != NRF_SUCCESS)
    		sleep_mode_enter();

    	err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    	if ((err_code != NRF_SUCCESS) &&
    			(err_code != NRF_ERROR_INVALID_STATE) &&
				(err_code != BLE_ERROR_NO_TX_PACKETS) &&
				(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    	{
    		APP_ERROR_HANDLER(err_code);
    	}
    }
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}

static void detected_noise_value_update()
{
	uint32_t err_code = NRF_SUCCESS;
	static uint8_t detected_value_buffer[MAX_DETECTED_NOISE_VALUE_LEN];

	// configure only when battery level ADC completed.
	if(m_is_battery_level_checked)
	{
		adc_config(!m_is_battery_level_checked);
		m_is_battery_level_checked = false;
	}

	detected_value_buffer[m_nds.cur_index++] = nrf_adc_convert_single(NOISE_DETECTOR_INPUT) & 0xFF;

	if(m_nds.cur_index == m_nds.max_dnv_len)
	{
		m_nds.cur_index = 0;
		// TODO : 문제 없나?
		m_nds.p_dnv_buffer = detected_value_buffer;
		err_code = ble_nds_detected_noise_value_broadcast(&m_nds);
	}

	NRF_LOG_INFO("ble_nds_detected_noise_value_broadcast err_code = %x\r\n", err_code);
	if ((err_code != NRF_SUCCESS) &&
			(err_code != NRF_ERROR_BUSY) &&
			(err_code != NRF_ERROR_INVALID_STATE) &&
			(err_code != BLE_ERROR_NO_TX_PACKETS) &&
			(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
	{
		APP_ERROR_HANDLER(err_code);
	}
}

static void noise_detector_meas_timeout_handler(void *p_context)
{
	UNUSED_PARAMETER(p_context);
	//NRF_LOG_INFO("noise_detector_meas_timeout_handler\r\n");
	detected_noise_value_update();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_noise_detector_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                noise_detector_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    //err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void services_init(void)
{
	uint32_t         err_code;
    ble_nds_init_t   nds_init;
    ble_bas_init_t   bas_init;
    //ble_dis_init_t   dis_init;
#ifdef BLE_DFU_SUPPORT
    ble_dfu_init_t dfus_init;
#endif

	// Initialize Noise Detector Service
    memset(&nds_init, 0, sizeof(nds_init));

    nds_init.evt_handler        = ble_nds_evt_handler;
    nds_init.error_handler		= NULL;
    nds_init.max_dnv_len		= MAX_DETECTED_NOISE_VALUE_LEN;

    // setting GAP attribute security requirement for Detected Noise Value characteristic.
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&nds_init.nds_dnv_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&nds_init.nds_dnv_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&nds_init.nds_dnv_attr_md.write_perm);

    // setting GAP attribute security requirement for Noise Detector Control Point.
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&nds_init.nds_ndcp_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&nds_init.nds_ndcp_attr_md.write_perm);

    err_code = ble_nds_init(&m_nds, &nds_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    nrf_delay_ms(200); // waiting for charging capacitor... fMAX = 6.5Hz
    adc_config(true);
    bas_init.initial_batt_level = calculate_battery_level_64_resolution(nrf_adc_convert_single(BATTERY_LEVEL_INPUT) & 0xFF, &err_code);
    //bas_init.initial_batt_level = nrf_adc_convert_single(BATTERY_LEVEL_INPUT) & 0xFF; // TODO
    m_is_battery_level_checked = true;
    if(err_code != NRF_SUCCESS)
        	sleep_mode_enter();

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    /*memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);*/

#ifdef BLE_DFU_SUPPORT
    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler                               = ble_dfu_evt_handler;
    dfus_init.ctrl_point_security_req_write_perm        = SEC_SIGNED;
    dfus_init.ctrl_point_security_req_cccd_write_perm   = SEC_SIGNED;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);
#endif
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
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
    cp_init.start_on_notify_cccd_handle    = m_nds.dnv_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast Advertising\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected, reason %d\r\n",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_nds_on_ble_evt(&m_nds, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_custom_1_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    nrf_ble_gatt_on_ble_evt(&m_gatt, p_ble_evt);
#ifdef BLE_DFU_SUPPORT
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt); // != ble_dfu_evt_handler
#endif
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;



    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(NRF_BLE_CENTRAL_LINK_COUNT,
                                                    NRF_BLE_PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(NRF_BLE_CENTRAL_LINK_COUNT, NRF_BLE_PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_GATT_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;

    switch (event)
    {
    	case BSP_EVENT_PREPARE_SLEEP:
    		bsp_custom_1_btn_ble_prepare_sleep_mode();
    		break;

        case BSP_EVENT_SLEEP:
        	bsp_custom_1_btn_ble_enter_sleep_mode();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_ADVERTISING_START:
            if (!m_is_advertising)
            	advertising_start();
            break;

        default:
            break;
    }
}

/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = fds_register(fds_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_adv_modes_config_t options;
    static ble_uuid_t  m_adv_uuids[1];
    //m_adv_uuids[0].uuid = BLE_NDS_SERVICE_UUID;
    //m_adv_uuids[0].type = m_nds.uuid_type;
    // occured encode error because of advertising data size.
    //m_adv_uuids[1].uuid = BLE_UUID_BATTERY_SERVICE;
	//m_adv_uuids[1].type = BLE_UUID_TYPE_BLE;
	//m_adv_uuids[2].uuid = BLE_UUID_DEVICE_INFORMATION_SERVICE;
	//m_adv_uuids[2].type = BLE_UUID_TYPE_BLE;
    m_adv_uuids[0].uuid = BLE_UUID_BATTERY_SERVICE;
    m_adv_uuids[0].type = BLE_UUID_TYPE_BLE;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_whitelist_enabled = false;
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(uint32_t pressed_time_ms)
{
    //bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);

    APP_ERROR_CHECK(err_code);

    err_code = bsp_custom_1_btn_ble_init(NULL, pressed_time_ms);
    if(err_code != NRF_SUCCESS)
    	sleep_mode_enter();

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}

/*GATT generic Event handler*/
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t * p_evt)
{
    ble_nds_on_gatt_evt(&m_nds, p_evt);

}

/*GATT Module init*/
void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */
int main(void)
{
	uint32_t err_code;
    //bool     erase_bonds;

    // Initialize.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    timers_init();
    buttons_leds_init(0);
    //scheduler_init();
    nrf_gpio_cfg_output(AMPLIFIER_SWITCH_PIN);
    nrf_gpio_pin_set(AMPLIFIER_SWITCH_PIN);

    ble_stack_init();
    peer_manager_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
    NRF_LOG_INFO("Noise Detector Sensor Start!\r\n");

    // battery_timer start!
    app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    advertising_start();

    // Enter main loop.
    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
	}
}
