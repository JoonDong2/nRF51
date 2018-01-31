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
#ifndef BLE_NDS_H__
#define BLE_NDS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_ble_gatt.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_NDS_BASE_UUID  {{0x50, 0xA, 0x99, 0x2, 0x77, 0x5E, 0xF9, 0xBD, 0x67, 0x49, 0x94, 0x9, 0x00, 0x00, 0x73, 0xF6}}
// This is a 16-bit UUID.
#define BLE_NDS_SERVICE_UUID                 0x8D00          //!< The UUID of the DFU Service.

// These UUIDs are used with the NDS base address to create a 128-bit UUID (bb22594e-b08a-43e7-9ad7-e469xxxxfcf5).
#define BLE_NDS_DETECTED_VALUE_UUID          0x8D01          //!< The UUID of the NDS Detected Value Characteristic
#define BLE_DFU_CP_UUID                		 0x8D02          //!< The UUID of the NDS Control Point Characteristic.

/**@brief Noise Detector Service event type. */
typedef enum
{
	BLE_DNV_EVT_NOTIFICATION_DISABLED,
	BLE_DNV_EVT_NOTIFICATION_ENABLED,
	BLE_NDCP_EVT_INDIFICATION_DISABLED,
	BLE_NDCP_EVT_INDIFICATION_ENABLED,
	BLE_NDCP_EVT_NOISE_DETECTOR_TIMER_DISABLE_REQUESTED,        /**< Noise Detector's ADC and broadcast disabled event. If this event is received, NDCP 'Write Request' was permitted. So have to turn ff the ADC and app_timer at the on ble_nds_on_ble_evt in the main.c >*/
	BLE_NDCP_EVT_NOISE_DETECTOR_TIMER_ENABLE_REQUESTED          /**< Noise Detector's ADC and broadcast enabled event. If this event is received, NDCP 'Write Request' was permitted. So have to turn on the ADC and app_timer at the on ble_nds_on_ble_evt in the main.c >*/
} ble_nds_evt_type_t;

/**@brief Noise Detector Service event. */
typedef struct
{
    ble_nds_evt_type_t evt_type;                        /**< Type of event. */
} ble_nds_evt_t;


// Forward declaration of the ble_nds_t type.
typedef struct 	ble_nds_s 	ble_nds_t;

/**@brief Noise Detector Service event handler type. */
typedef uint32_t 	(*ble_nds_evt_handler_t) 	(ble_nds_t * p_nds, ble_nds_evt_t * p_evt);

// Noise Detector Control Point response values
typedef enum
{
    NDCP_RSP_RESERVED              = 0x00,                                        /**< Reserved for future use. */
	NDCP_RSP_SUCCESS               = 0x01,                                        /**< Success. */
	NDCP_RSP_OP_CODE_NOT_SUPPORTED = 0x02,                                        /**< Op Code not supported. */
	NDCP_RSP_INVALID_PARAMETER     = 0x03,                                        /**< Invalid Parameter. */
	NDCP_RSP_OPERATION_FAILED      = 0x04,                                        /**< Operation Failed. */
	NDCP_RSP_PROC_ALR_IN_PROG      = BLE_GATT_STATUS_ATTERR_CPS_PROC_ALR_IN_PROG, /**< Control point procedure is already in progress. */
	NDCP_RSP_CCCD_CONFIG_IMPROPER  = BLE_GATT_STATUS_ATTERR_CPS_CCCD_CONFIG_ERROR /**< CCCD is improperly configured. */
} ble_ndcp_rsp_code_t;

/** @brief Noise Detector Control Point procedure status */
typedef enum
{
    NDCP_STATE_NO_PROC_IN_PROGRESS,                        /**< No procedure in progress. */
	NDCP_STATE_PROC_IN_PROGRESS,                        /**< Control Point indication is in progress. */
} ble_ndcp_procedure_status_t;

// Noise Detector Control Point Op Code values
typedef enum
{
	NDCP_OP_RESERVED,							/**< There is no opcode. >*/
	NDCP_OP_TURN_OFF_NOISE_DETECTOR_TIMER,   	/**< Turn on noise detector timer. >*/
	NDCP_OP_TURN_ON_NOISE_DETECTOR_TIMER 		/**< Turn off noise detector timer. >*/
} ble_ndcp_op_code_t;

// Noise Detector Control Point Op Code values
typedef enum
{
	NDCP_RESULT_RESERVED,				/**< There is no result. >*/
	NDCP_RESULT_FAILED,   				/**< Requested operation is failed. >*/
	NDCP_RESULT_SUCCEED, 				/**< Requested operation, including indication is succeed. >*/
} ble_ndcp_rusult_t;

typedef struct
{
	ble_ndcp_op_code_t 	opcode;
	ble_ndcp_rusult_t	result;
}ble_ndcp_rsp_t;

/**@brief Noise Detector Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_nds_evt_handler_t        	evt_handler;           /**< Event handler to be called for handling events in the Noise Detector Service. */
    ble_srv_error_handler_t      	error_handler;			/**< Error handler to be called for handling errors in the Noise Detector Service. */
    ble_srv_cccd_security_mode_t 	nds_dnv_attr_md;       /**< Initial security level for noise detector service detected value attribute */
    ble_srv_security_mode_t      	nds_ndcp_attr_md;      /**< Initial security level for noise detector service control point attribute */
    uint8_t* 					 	p_dnv_buffer;			/**< This is poplulated to m_nds.p_dnv_buffer. >*/
    uint8_t                      	max_dnv_len;        				/**< Maximum Detected Noise Values length, adjusted according to the current ATT MTU. */
}ble_nds_init_t;

/**@brief Noise Detector Service structure. This contains various status information for the service. */
struct ble_nds_s
{
    ble_nds_evt_handler_t       	evt_handler;        			/**< Event handler to be called for handling events in the Noise Detector Service. */
    ble_srv_error_handler_t      	error_handler;			 		/**< Error handler to be called for handling errors in the Noise Detector Service. not used.*/
    uint16_t                    	conn_handle;        			/**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint16_t                    	service_handle;     			/**< Handle of Noise Detector Service (as provided by the BLE stack). */
    uint8_t                     	uuid_type;         				/**< UUID type assigned to the Noise Detector Service by the SoftDevice. */
    // associated with DNV char
    ble_gatts_char_handles_t    	dnv_handles;       				/**< Handles related to the Detected Noise Value characteristic. */
    ble_gatts_char_handles_t    	ndcp_handles;       			/**< Handles related to the Noise Detector Control Point characteristic. */
    uint8_t*						p_dnv_buffer;		 			/**< Point to buffer detected values are saved. this space is used at the ADC peripheral. >*/
    uint8_t                     	max_dnv_len;        				/**< Maximum Detected Noise Values length, adjusted according to the current ATT MTU. */
    __volatile uint8_t				cur_index;						/**< The index of buffer. >*/
    bool 					 		is_dnv_notification_permitted;		/**< This is changed true after ADC module detect 20 times. >*/
    // associated with NDCP char
    ble_ndcp_procedure_status_t		ndcp_procedure_status;			/**< This appear the ndcp operation's status. >*/
    ble_ndcp_rsp_t					ndcp_rsp;
    bool 					 		is_ndcp_indication_permitted;	/**< This is changed true when a peer request to write to NDCP characteristic's CCCD. >*/
};

/**@brief Function for initializing the Noise Detector Service.
 *
 * @param[out]  p_nds       Noise Detector structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_hrs_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_nds_init(ble_nds_t * p_nds, const ble_nds_init_t * p_nds_init);

/**@brief Function for handling the GATT module's events.
 *
 * @details Handles all events from the GATT module of interest to the Noise Detector Service.
 *
 * @param[in]   p_nds      	Noise Detector Service structure.
 * @param[in]   p_gatt_evt  Event received from the GATT module.
 */
void ble_nds_on_gatt_evt(ble_nds_t * p_nds, nrf_ble_gatt_evt_t * p_gatt_evt);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Heart Rate Service.
 *
 * @param[in]   p_nds      Noise Detector Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_nds_on_ble_evt(ble_nds_t * p_nds, ble_evt_t * p_ble_evt);

/**@brief Function for broadcasting the detected noise value to a peer.
 *
 * @details Broadcast m_dnv_buffer to a peer.
 *
 * @param[in]   p_nds      Noise Detector Service structure.
 */
uint32_t ble_nds_detected_noise_value_broadcast(ble_nds_t* p_nds);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Noise Detector Service.
 *
 * @param[in]   p_nds      Noise Detector Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_nds_on_ble_evt(ble_nds_t * p_nds, ble_evt_t * p_ble_evt);

#ifdef __cplusplus
}
#endif

#endif // BLE_NDS_H__
