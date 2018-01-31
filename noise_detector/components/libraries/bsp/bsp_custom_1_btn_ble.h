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
#ifndef BSP_CUSTOM_1_BTN_BLE_H__
#define BSP_CUSTOM_1_BTN_BLE_H__

#include <stdint.h>
#include "ble.h"
#include "bsp.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief BLE Button Module error handler type. */
typedef void (*bsp_custom_1_btn_ble_error_handler_t) (uint32_t nrf_error);

/**@brief Function for initializing the BLE Button Module.
 *
 * Before calling this function, the BSP module must be initialized with buttons.
 *
 * @param[out] error_handler      Error handler to call in case of internal errors in BLE Button
 *                                Module.
 *
 * @retval NRF_SUCCESS  If initialization was successful. Otherwise, a propagated error code is
 *                      returned.
 */
uint32_t bsp_custom_1_btn_ble_init(bsp_custom_1_btn_ble_error_handler_t error_handler, uint32_t pressed_time_ms);

/**@brief Function for setting up wakeup buttons before going into sleep mode.
 *
 * @retval NRF_SUCCESS  If the buttons were prepared successfully. Otherwise, a propagated error
 *                      code is returned.
 */
uint32_t bsp_custom_1_btn_ble_setup_sleep_button(void);

/**@brief Function for preparing sleep mode.
 * This must be used at BSP_EVENT_PREPARE_SLEEP event in bsp event handler.
 */
void bsp_custom_1_btn_ble_prepare_sleep_mode();

/**@brief Function for entering sleep mode.
 * This must be used at BSP_EVENT_SLEEP event in bsp event handler.
 */
void bsp_custom_1_btn_ble_enter_sleep_mode();

/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to this module.
 *
 * @param[in] p_ble_evt BLE stack event.
 */
void bsp_custom_1_btn_ble_on_ble_evt(ble_evt_t * p_ble_evt);

#define BSP_EVENT_PREPARE_SLEEP		BSP_EVENT_KEY_0

#ifdef __cplusplus
}
#endif

#endif /* BSP_CUSTOM_1_BTN_BLE_H__ */

/** @} */
