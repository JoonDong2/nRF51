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
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_NDS)
#include "ble_nds.h"
#include <string.h>
#include "ble_l2cap.h"
#include "ble_srv_common.h"

#define NRF_LOG_MODULE_NAME "BLE_NDS"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define OPCODE_LENGTH 1                                                              /**< Length of opcode inside Detected Noise Value packet. */
#define HANDLE_LENGTH 2                                                              /**< Length of handle inside Detected Noise Value packet. */
#define MAX_DETECTED_NOISE_VALUE_LEN      	(GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH) 	 /**< Maximum size of a transmitted Detected Noise Value */
#define NOISE_DETECTOR_CONTROLPOINT_LEN 	2

/**@brief Function for adding Detected Noise Value characteristic.
 *
 * @param[in] p_nds       Noise Detector Service structure.
 * @param[in] p_nds_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t dnv_char_add(ble_nds_t * p_nds, const ble_nds_init_t * p_nds_init)
{
	/**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
	ble_gatts_char_md_t 	char_md;
	ble_gatts_attr_md_t 	cccd_md;
	ble_gatts_attr_t    	attr_char_value;
	ble_uuid_t          	ble_uuid;
	ble_gatts_attr_md_t 	attr_md;

	memset(&cccd_md, 0, sizeof(cccd_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	cccd_md.write_perm = p_nds_init->nds_dnv_attr_md.cccd_write_perm;
	cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.notify = 1;
	char_md.p_char_user_desc  = NULL;
	char_md.p_char_pf         = NULL;
	char_md.p_user_desc_md    = NULL;
	char_md.p_cccd_md         = &cccd_md;
	char_md.p_sccd_md         = NULL;

	ble_uuid.type = p_nds->uuid_type;
	ble_uuid.uuid = BLE_NDS_DETECTED_VALUE_UUID;

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.read_perm  = p_nds_init->nds_dnv_attr_md.read_perm;
	attr_md.write_perm = p_nds_init->nds_dnv_attr_md.write_perm;
	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth    = 0;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = p_nds->max_dnv_len;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = MAX_DETECTED_NOISE_VALUE_LEN;

	return sd_ble_gatts_characteristic_add(p_nds->service_handle,
	                                       &char_md,
	                                       &attr_char_value,
	                                       &p_nds->dnv_handles);
}

/**@brief Function for adding Noise Detector Service Control Point characteristic.
 *
 * @param[in] p_nds       Noise Detector Service structure.
 * @param[in] p_nds_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t nds_cp_char_add(ble_nds_t* p_nds, const ble_nds_init_t* p_nds_init)
{
	/**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;

	uint8_t 			ndcp_init_value[NOISE_DETECTOR_CONTROLPOINT_LEN];

	memset(&cccd_md, 0, sizeof(cccd_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&cccd_md.write_perm);
	cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.write 	= 1;
	char_md.char_props.indicate = 1;
	char_md.p_char_user_desc  	= NULL;
	char_md.p_char_pf         	= NULL;
	char_md.p_user_desc_md    	= NULL;
	char_md.p_cccd_md         	= &cccd_md;
	char_md.p_sccd_md         	= NULL;

	ble_uuid.type = p_nds->uuid_type;
	ble_uuid.uuid = BLE_DFU_CP_UUID;

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.read_perm  = p_nds_init->nds_ndcp_attr_md.read_perm;
	attr_md.write_perm = p_nds_init->nds_ndcp_attr_md.write_perm;
	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth    = 0;
	attr_md.wr_auth    = 1;
	attr_md.vlen       = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));
	ndcp_init_value[0] = (uint8_t)NDCP_OP_RESERVED;
	ndcp_init_value[1] = (uint8_t)NDCP_RESULT_RESERVED;

	attr_char_value.p_uuid    	= &ble_uuid;
	attr_char_value.p_attr_md 	= &attr_md;
	attr_char_value.p_value		= ndcp_init_value;
	attr_char_value.init_len  	= sizeof(ndcp_init_value);
	attr_char_value.init_offs 	= 0;
	attr_char_value.max_len   	= sizeof(ndcp_init_value);

	return sd_ble_gatts_characteristic_add(p_nds->service_handle,
		                                   &char_md,
		                                   &attr_char_value,
		                                   &p_nds->ndcp_handles);
}

/**@brief Function for handling write events to the Detected Noise Value characteristic's descriptor.
 *
 * @param[in]   p_hrs         Noise Detector Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_dnv_cccd_write(ble_nds_t * p_nds, ble_gatts_evt_write_t * p_evt_write)
{
    if (p_evt_write->len == BLE_CCCD_VALUE_LEN)
    {
        // CCCD written, update notification state
        if (p_nds->evt_handler != NULL)
        {
            ble_nds_evt_t evt;
            p_nds->is_dnv_notification_permitted = ble_srv_is_notification_enabled(p_evt_write->data);

            if (p_nds->is_dnv_notification_permitted)
            {
                evt.evt_type = BLE_DNV_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_DNV_EVT_NOTIFICATION_DISABLED;
            }

            p_nds->evt_handler(p_nds, &evt); // in the main.c
        }
    }
}

/**@brief Function for handling write events to the Detected Noise Value characteristic's descriptor.
 *
 * @param[in]   p_hrs         Noise Detector Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_ndcp_cccd_write(ble_nds_t * p_nds, ble_gatts_evt_write_t * p_evt_write)
{
    if (p_evt_write->len == BLE_CCCD_VALUE_LEN)
    {
        // CCCD written, update notification state
        if (p_nds->evt_handler != NULL)
        {
            ble_nds_evt_t evt;
            p_nds->is_ndcp_indication_permitted = ble_srv_is_indication_enabled(p_evt_write->data);

            if (p_nds->is_ndcp_indication_permitted)
            {
                evt.evt_type = BLE_NDCP_EVT_INDIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_NDCP_EVT_INDIFICATION_DISABLED;
            }

            p_nds->evt_handler(p_nds, &evt); // in the main.c
        }
    }
}

uint32_t ble_nds_init(ble_nds_t * p_nds, const ble_nds_init_t * p_nds_init)
{
	uint32_t   err_code;
	ble_uuid_t ble_uuid;
	ble_uuid128_t nds_base_uuid = BLE_NDS_BASE_UUID;

	// Initialize service structure
	p_nds->evt_handler                 	= p_nds_init->evt_handler;
	p_nds->error_handler				= p_nds_init->error_handler;
	p_nds->conn_handle                 	= BLE_CONN_HANDLE_INVALID;
	p_nds->max_dnv_len                 	= p_nds_init->max_dnv_len;
	p_nds->p_dnv_buffer					= NULL;
	p_nds->cur_index					= 0;
	p_nds->is_dnv_notification_permitted  	= false;
	p_nds->is_ndcp_indication_permitted  	= false;
	p_nds->ndcp_rsp.opcode = NDCP_OP_RESERVED;
	p_nds->ndcp_rsp.result = NDCP_RESULT_RESERVED;

	// Add a custom base UUID.
	err_code = sd_ble_uuid_vs_add(&nds_base_uuid, &p_nds->uuid_type);
	VERIFY_SUCCESS(err_code);

	ble_uuid.type = p_nds->uuid_type; // it is already populated above.
	ble_uuid.uuid = BLE_NDS_SERVICE_UUID; // new

	// Add the service.
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
	                                    &ble_uuid,
	                                    &p_nds->service_handle);
	/**@snippet [Adding proprietary Service to S110 SoftDevice] */
	VERIFY_SUCCESS(err_code);

	err_code = dnv_char_add(p_nds, p_nds_init);
	VERIFY_SUCCESS(err_code);

	err_code = nds_cp_char_add(p_nds, p_nds_init);
	VERIFY_SUCCESS(err_code);

	return NRF_SUCCESS;
}

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_hrs       Noise Detector Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_nds_t * p_nds, ble_evt_t * p_ble_evt)
{
	p_nds->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_hrs       Noise Detector Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_nds_t * p_nds, ble_evt_t * p_ble_evt)
{
	ble_nds_evt_t evt = {.evt_type = BLE_NDCP_EVT_NOISE_DETECTOR_TIMER_DISABLE_REQUESTED};
    UNUSED_PARAMETER(p_ble_evt);
    p_nds->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_nds->evt_handler(p_nds, &evt);
}

/* This broadcast the result of a peer's request with an received opcode.
 * @details This is called after process peer's request, so a result is involved to parameters.
 *
 * @param[in]   p_nds       	Noise Detector Service structure.
 * @param[in]   prev_err_code   Result of processing peer's request @reference nrf_error.h
 * */
static void ble_ndcp_result_broadcast(ble_nds_t * p_nds, uint32_t prev_err_code)
{
	uint32_t err_code 		= NRF_SUCCESS;
	ble_gatts_hvx_params_t 	hvx_params;
	uint8_t 				response[NOISE_DETECTOR_CONTROLPOINT_LEN];
	uint16_t				response_len = NOISE_DETECTOR_CONTROLPOINT_LEN;


	if(prev_err_code == NRF_SUCCESS)
	{
		p_nds->ndcp_rsp.result = NDCP_RESULT_SUCCEED;
	}
	else
	{
		p_nds->ndcp_rsp.result = NDCP_RESULT_FAILED;
	}

	response[0] = (uint8_t)p_nds->ndcp_rsp.opcode;
	response[1] = (uint8_t)p_nds->ndcp_rsp.result;

	// Initialize value struct.
	memset(&hvx_params, 0, sizeof(hvx_params));

	hvx_params.handle = p_nds->ndcp_handles.value_handle;
	hvx_params.type   = BLE_GATT_HVX_INDICATION;
	hvx_params.offset = 0;
	hvx_params.p_len  = &response_len;
	hvx_params.p_data = response;

	err_code = sd_ble_gatts_hvx(p_nds->conn_handle, &hvx_params);
	NRF_LOG_INFO("ndcp_rsp.opcode = %x, ndcp_rsp.result = %x\r\n", response[0], response[1]);
	NRF_LOG_INFO("ble_ndcp_result_broadcast :: sd_ble_gatts_hvx err_code = %d\r\n", err_code);

	if(err_code != NRF_SUCCESS) {
		if(p_nds->error_handler != NULL)
			p_nds->error_handler(err_code); // TODO : if error_code is not NRF_SUCCESS, the operation is back to the previous status.
	}

	p_nds->ndcp_rsp.opcode = NDCP_OP_RESERVED;
	p_nds->ndcp_rsp.result = NDCP_RESULT_RESERVED;
}

/**@brief Handle write events to the Location and Navigation Service Control Point characteristic.
 *
 * @param[in]   p_lncp         	Location and Navigation Service structure.
 * @param[in]   p_evt_write   	Write event received from the BLE stack.
 */
static void on_ctrlpt_write(ble_nds_t * p_nds, ble_gatts_evt_write_t const * p_evt_write)
{
    uint32_t      err_code;

    ble_gatts_rw_authorize_reply_params_t write_authorize_reply;
    memset(&write_authorize_reply, 0, sizeof(write_authorize_reply));

    write_authorize_reply.type   = BLE_GATTS_AUTHORIZE_TYPE_WRITE;

    if (p_nds->ndcp_procedure_status == NDCP_STATE_NO_PROC_IN_PROGRESS)
    {
        write_authorize_reply.params.write.update      = 1;
        write_authorize_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;

        // if the op code is navigation control, its cccd must be checked
        if (p_evt_write->len == 0)
        {
        	write_authorize_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_INVALID_PDU;
        }
    }
    else
    {
        write_authorize_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_CPS_PROC_ALR_IN_PROG;
    }


    // reply to the write authorization
    do {
        err_code = sd_ble_gatts_rw_authorize_reply(p_nds->conn_handle, &write_authorize_reply);
        if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_BUSY)
        {
            if (p_nds->error_handler != NULL)
            {
                p_nds->error_handler(err_code);
            }
        }
    } while (err_code == NRF_ERROR_BUSY);


    if (write_authorize_reply.params.write.gatt_status != BLE_GATT_STATUS_SUCCESS)
    {
    	p_nds->ndcp_procedure_status = NDCP_STATE_NO_PROC_IN_PROGRESS;
        return; // If return at here, the request is not handled. SoftDevice sent reject message about a peer's WRITE REQUEST.
        // This reject message is handled at a peer.
    }

    // Start executing the control point write action
    p_nds->ndcp_procedure_status = NDCP_STATE_PROC_IN_PROGRESS;
    if (p_evt_write->len > 0)
    {
    	if(p_nds->evt_handler != NULL)
    	{
    		ble_nds_evt_t evt;
    		p_nds->ndcp_rsp.opcode = (ble_ndcp_op_code_t)p_evt_write->data[0];
    		switch (p_nds->ndcp_rsp.opcode)
        	{
            	case NDCP_OP_TURN_OFF_NOISE_DETECTOR_TIMER:
            		evt.evt_type = BLE_NDCP_EVT_NOISE_DETECTOR_TIMER_DISABLE_REQUESTED;
                	break;

            	case NDCP_OP_TURN_ON_NOISE_DETECTOR_TIMER:
            		evt.evt_type = BLE_NDCP_EVT_NOISE_DETECTOR_TIMER_ENABLE_REQUESTED;
                	break;

                // Unrecognized Op Code
            	default:
            		err_code = NDCP_RSP_OP_CODE_NOT_SUPPORTED;
            		break;
        	}
    		err_code = p_nds->evt_handler(p_nds, &evt);
    	}
    	ble_ndcp_result_broadcast(p_nds, err_code); // include error handler
    }
    p_nds->ndcp_procedure_status = NDCP_STATE_NO_PROC_IN_PROGRESS;
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_nds       Noise Detector Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_nds_t * p_nds, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_nds->dnv_handles.cccd_handle)
    {
        on_dnv_cccd_write(p_nds, p_evt_write);
    }
    else if(p_evt_write->handle == p_nds->ndcp_handles.cccd_handle)
    {
    	on_ndcp_cccd_write(p_nds, p_evt_write);
    }
}

/**@brief Write authorization request event handler.
 *
 * @details The write authorization request event handler is only called when writing to the control point.
 *
 * @param[in]   p_nds       Noise Detector Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_rw_authorize_req(ble_nds_t * p_nds, ble_evt_t * p_ble_evt)
{
	const ble_gatts_evt_rw_authorize_request_t * p_auth_req =
						&p_ble_evt->evt.gatts_evt.params.authorize_request;

	if (
	    (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
	    &&
	    (p_auth_req->request.write.handle == p_nds->ndcp_handles.value_handle)
	    &&
	    (p_auth_req->request.write.op != BLE_GATTS_OP_PREP_WRITE_REQ)
	    &&
	    (p_auth_req->request.write.op != BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
	    &&
	    (p_auth_req->request.write.op != BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)
	   )
	{
	    on_ctrlpt_write(p_nds, &p_auth_req->request.write);
	}
}

void ble_nds_on_ble_evt(ble_nds_t * p_nds, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_nds, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_nds, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_nds, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        	on_rw_authorize_req(p_nds, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/* This broadcast detected noise value only when p_nds->p_dnv_buffer reach to p_nds->max_dnv_len.
 *
 * @param[in]   p_nds       	Noise Detector Service structure.
 * */
uint32_t ble_nds_detected_noise_value_broadcast(ble_nds_t* p_nds)
{
	if (p_nds == NULL)
	{
	    return NRF_ERROR_NULL;
	}

	uint32_t err_code = NRF_SUCCESS;
	ble_gatts_value_t gatts_value;

	// Initialize value struct.
	memset(&gatts_value, 0, sizeof(gatts_value));

	gatts_value.len     = p_nds->max_dnv_len;
	gatts_value.offset  = 0;
	gatts_value.p_value = p_nds->p_dnv_buffer;

	// Update database.
	err_code = sd_ble_gatts_value_set(p_nds->conn_handle,
	                                  p_nds->dnv_handles.value_handle,
	                                  &gatts_value);
	NRF_LOG_INFO("sd_ble_gatts_value_set err_code = %d\r\n", err_code);NRF_LOG_FLUSH();

	if(err_code != NRF_SUCCESS)
		return err_code;

	// Indicate only when m_noise_detector_timer times out 20 times and is_dnv_notification_permitted is changed to true.
	if((p_nds->conn_handle != BLE_CONN_HANDLE_INVALID) && p_nds->is_dnv_notification_permitted == true)
	{
		ble_gatts_hvx_params_t hvx_params;

		memset(&hvx_params, 0, sizeof(hvx_params));

		hvx_params.handle = p_nds->dnv_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = gatts_value.offset;
		hvx_params.p_len  = &gatts_value.len;
		hvx_params.p_data = gatts_value.p_value;

		err_code = sd_ble_gatts_hvx(p_nds->conn_handle, &hvx_params);
		NRF_LOG_INFO("sd_ble_gatts_hvx err_code = %x\r\n", err_code);
		return err_code;
	}
	else
	{
		return NRF_ERROR_INVALID_STATE;
	}

}

//TODO : 이상없나?
void ble_nds_on_gatt_evt(ble_nds_t * p_nds, nrf_ble_gatt_evt_t * p_gatt_evt)
{
    if (p_nds->conn_handle == p_gatt_evt->conn_handle)
    {
    	if(p_nds->max_dnv_len > p_gatt_evt->att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH)
    		p_nds->max_dnv_len = p_gatt_evt->att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}
#endif // NRF_MODULE_ENABLED(BLE_NDS)
