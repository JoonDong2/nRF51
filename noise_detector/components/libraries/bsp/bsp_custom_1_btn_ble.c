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
#include "bsp_custom_1_btn_ble.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ble.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "nrf_power.h"
#include "nrf_soc.h"

#define BTN_ID_WAKEUP             0  /**< ID of button used to wake up the application. */
#define BTN_ID_SLEEP              0  /**< ID of button used to put the application into sleep mode. */
#define BTN_ID_DISCONNECT         0  /**< ID of button used to gracefully terminate a connection on long press. */
#define BTN_ID_ADVERTISE         0  /**< ID of button used to start advertising. */

#define BTN_ACTION_PREPARE_SLEEP        BSP_BUTTON_ACTION_LONG_PUSH    /**< Button action used to put the application into sleep mode. */
#define BTN_ACTION_ENTER_SLEEP        	BSP_BUTTON_ACTION_RELEASE    /**< Button action used to put the application into sleep mode. */
#define BTN_ACTION_DISCONNECT     		BSP_BUTTON_ACTION_PUSH  /**< Button action used to gracefully terminate a connection on long press. */
#define BTN_ACTION_ADVERTSING_START  	BSP_BUTTON_ACTION_PUSH  	 /**< Button action used to start advertising. */

__volatile bool m_is_sleep_mode_prepared;

/**@brief This macro will return from the current function if err_code
 *        is not NRF_SUCCESS.
 */
#define RETURN_ON_ERROR(err_code)  \
do                                 \
{                                  \
    if ((err_code) != NRF_SUCCESS) \
    {                              \
        return err_code;           \
    }                              \
}                                  \
while (0)


/**@brief This macro will return from the current function if err_code
 *        is not NRF_SUCCESS or NRF_ERROR_INVALID_PARAM.
 */
#define RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code)                             \
do                                                                              \
{                                                                               \
    if (((err_code) != NRF_SUCCESS) && ((err_code) != NRF_ERROR_INVALID_PARAM)) \
    {                                                                           \
        return err_code;                                                        \
    }                                                                           \
}                                                                               \
while (0)


/**@brief This macro will return from the current function if err_code
 *        is not NRF_SUCCESS or NRF_ERROR_NOT_SUPPORTED.
 */
#define RETURN_ON_ERROR_NOT_NOT_SUPPORTED(err_code)                             \
do                                                                              \
{                                                                               \
    if (((err_code) != NRF_SUCCESS) && ((err_code) != NRF_ERROR_NOT_SUPPORTED)) \
    {                                                                           \
        return err_code;                                                        \
    }                                                                           \
}                                                                               \
while (0)


/**@brief This macro will call the registered error handler if err_code
 *        is not NRF_SUCCESS and the error handler is not NULL.
 */
#define CALL_HANDLER_ON_ERROR(err_code)                           \
do                                                                \
{                                                                 \
    if (((err_code) != NRF_SUCCESS) && (m_error_handler != NULL)) \
    {                                                             \
        m_error_handler(err_code);                                \
    }                                                             \
}                                                                 \
while (0)


static bsp_custom_1_btn_ble_error_handler_t m_error_handler = NULL; /**< Error handler registered by the user. */
static uint32_t                    m_num_connections = 0;  /**< Number of connections the device is currently in. */

/**@brief Function for configuring the buttons for connection.
 *
 * @retval NRF_SUCCESS  Configured successfully.
 * @return A propagated error code.
 */
static uint32_t connection_buttons_configure()
{
    uint32_t err_code;

    err_code = bsp_event_to_button_action_assign(BTN_ID_SLEEP,
                                                 BTN_ACTION_PREPARE_SLEEP,
												 BSP_EVENT_PREPARE_SLEEP);
    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

    err_code = bsp_event_to_button_action_assign(BTN_ID_SLEEP,
                                                 BTN_ACTION_ENTER_SLEEP,
    											 BSP_EVENT_SLEEP);
        RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

    err_code = bsp_event_to_button_action_assign(BTN_ID_DISCONNECT,
                                                 BTN_ACTION_DISCONNECT,
                                                 BSP_EVENT_DISCONNECT);
    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

    return NRF_SUCCESS;
}


/**@brief Function for configuring the buttons for advertisement.
 *
 * @retval NRF_SUCCESS  Configured successfully.
 * @return A propagated error code.
 */
static uint32_t advertising_buttons_configure()
{
    uint32_t err_code;

    err_code = bsp_event_to_button_action_assign(BTN_ID_SLEEP,
                                                 BTN_ACTION_PREPARE_SLEEP,
												 BSP_EVENT_PREPARE_SLEEP);
    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

    err_code = bsp_event_to_button_action_assign(BTN_ID_SLEEP,
                                                 BTN_ACTION_ENTER_SLEEP,
        										 BSP_EVENT_SLEEP);
    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

    // The action of this event is implemented manually in bsp event handler.
    err_code = bsp_event_to_button_action_assign(BTN_ID_ADVERTISE,
    											 BTN_ACTION_ADVERTSING_START,
												 BSP_EVENT_ADVERTISING_START);
    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

    return NRF_SUCCESS;
}

/**@brief Function for checking the BSP condition valid at startup.
 *
 * @details When a button was used to wake up the device, the button press will not generate an
 *          interrupt. This function reads which button was pressed at startup, and returns the
 *          appropriate BSP event.
 *
 * @param[out] p_startup_event  Where to put the extracted BSP event.
 */
static void startup_condition_check(uint32_t pressed_time_ms)
{
    // React to button states
	uint32_t count_64_ms_unit = pressed_time_ms >> 6; // pressed_time_ms ¡À 64
	uint32_t remainder_ms = pressed_time_ms & 0x3F; // pressed_time_ms % 64

	for(int i=0; i<count_64_ms_unit; i++)
	{
		nrf_delay_ms(64);
		if(!bsp_button_is_pressed(BTN_ID_WAKEUP))
			nrf_power_system_off();
	}

	nrf_delay_ms(remainder_ms);
	if(!bsp_button_is_pressed(BTN_ID_WAKEUP))
		nrf_power_system_off();
}

uint32_t bsp_custom_1_btn_ble_setup_sleep_button(void)
{
    uint32_t err_code = bsp_wakeup_button_enable(BTN_ID_WAKEUP);
    RETURN_ON_ERROR_NOT_NOT_SUPPORTED(err_code);

    return NRF_SUCCESS;
}

void bsp_custom_1_btn_ble_prepare_sleep_mode()
{
	m_is_sleep_mode_prepared = true;
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

void bsp_custom_1_btn_ble_enter_sleep_mode()
{
	if(m_is_sleep_mode_prepared)
		sleep_mode_enter();
}

void bsp_custom_1_btn_ble_on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            if (m_num_connections == 0)
            {
                err_code = connection_buttons_configure();
                CALL_HANDLER_ON_ERROR(err_code);
            }

            m_num_connections++;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_num_connections--;

            if (m_num_connections == 0)
            {
                err_code = advertising_buttons_configure();
                CALL_HANDLER_ON_ERROR(err_code);
            }
            break;

        default:
            break;
    }
}


uint32_t bsp_custom_1_btn_ble_init(bsp_custom_1_btn_ble_error_handler_t error_handler, uint32_t pressed_time_ms)
{
    uint32_t err_code = NRF_SUCCESS;

    m_error_handler = error_handler;

    if(pressed_time_ms != 0)
    {
    	startup_condition_check(pressed_time_ms);
    }

    if(err_code != NRF_SUCCESS)
    	return err_code;

    if (m_num_connections == 0)
    {
        err_code = advertising_buttons_configure();
    }


    return err_code;
}
