/******************************************************************************
  * \attention
  *
  * <h2><center>&copy; COPYRIGHT 2021 STMicroelectronics</center></h2>
  *
  * Licensed under ST MIX MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        www.st.com/mix_myliberty
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/

/*! \file
 *
 *  \author SRA
 *
 *  \brief ST25R95 Analog Configuration Settings
 *
 */

#ifndef ST25R95_ANALOGCONFIG_H
#define ST25R95_ANALOGCONFIG_H

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "rfal_rfst25r95_analogConfig.h"
#include "st25r95_com.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL MACROS
 ******************************************************************************
 */

/*! Macro for Configuration Setting with only one register-mask-value set:
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1] */
#define MODE_ENTRY_1_REG(MODE, R0, M0, V0)              \
    (uint8_t)((MODE) >> 8), (uint8_t)((MODE) & 0xFFU), 1, (uint8_t)((R0) >> 8), (uint8_t)((R0) & 0xFFU), (uint8_t)(M0), (uint8_t)(V0)


/*
 ******************************************************************************
 * GLOBAL DATA TYPES
 ******************************************************************************
 */
const uint8_t rfalAnalogConfigDefaultSettings[] = {
  //****** Default Analog Configuration for Poll NFC-A Tx. ******/
  MODE_ENTRY_1_REG((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                   , ST25R95_REG_ARC_B, 0xf0, 0xD0 /* Modulation Index 0x1X: 10%, 0x2X: 17%, 0x3X: 25%, 0x4X: 30%, 0x5X: 33%, 0x6X: 36%, 0xDX: 95% */
                  )
  //****** Default Analog Configuration for Poll NFC-A Rx. ******/
  , MODE_ENTRY_1_REG((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                     , ST25R95_REG_ARC_B, 0x0f, 0x03 /* Receiver Gain 0: 34dB, 1: 32dB, 3: 27dB, 7: 20dB, F: 8dB */
                    )

  //****** Default Analog Configuration for Poll NFC-B Tx. ******/
  , MODE_ENTRY_1_REG((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                     , ST25R95_REG_ARC_B, 0xf0, 0x30 /* Modulation Index 0x1X: 10%, 0x2X: 17%, 0x3X: 25%, 0x4X: 30%, 0x5X: 33%, 0x6X: 36%, 0xDX: 95% */
                    )
  //****** Default Analog Configuration for Poll NFC-B Rx. ******/
  , MODE_ENTRY_1_REG((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                     , ST25R95_REG_ARC_B, 0x0f, 0x00 /* Receiver Gain 0: 34dB, 1: 32dB, 3: 27dB, 7: 20dB, F: 8dB */
                    )

  //****** Default Analog Configuration for Poll NFC-F Tx. ******/
  , MODE_ENTRY_1_REG((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                     , ST25R95_REG_ARC_B, 0xf0, 0x50 /* Modulation Index 0x1X: 10%, 0x2X: 17%, 0x3X: 25%, 0x4X: 30%, 0x5X: 33%, 0x6X: 36%, 0xDX: 95% */
                    )
  //****** Default Analog Configuration for Poll NFC-F Rx. ******/
  , MODE_ENTRY_1_REG((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                     , ST25R95_REG_ARC_B, 0x0f, 0x00 /* Receiver Gain 0: 34dB, 1: 32dB, 3: 27dB, 7: 20dB, F: 8dB */
                    )

  //****** Default Analog Configuration for Poll NFC-V Tx. ******/
  , MODE_ENTRY_1_REG((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                     , ST25R95_REG_ARC_B, 0xf0, 0x50 /* Modulation Index 0x1X: 10%, 0x2X: 17%, 0x3X: 25%, 0x4X: 30%, 0x5X: 33%, 0x6X: 36%, 0xDX: 95% */
                    )
  //****** Default Analog Configuration for Poll NFC-V Rx. ******/
  , MODE_ENTRY_1_REG((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                     , ST25R95_REG_ARC_B, 0x0f, 0x03 /* Receiver Gain 0: 34dB, 1: 32dB, 3: 27dB, 7: 20dB, F: 8dB */
                    )
#if RFAL_FEATURE_LISTEN_MODE
  //****** Default Analog Configuration for Listen NFC-A Tx. ******/
  , MODE_ENTRY_1_REG((RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                     , ST25R95_REG_ACC_A, 0x0f, 0x07 /* Load modulation index 1: Min, 7: default, F: Max */
                    )
  //****** Default Analog Configuration for Listen NFC-A Rx. ******/
  , MODE_ENTRY_1_REG((RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                     , ST25R95_REG_ACC_A, 0xf0, 0x20 /* Demod sensivity 1: 10%, 2: 100% */
                    )
#endif
};

#endif /* ST25R95_ANALOGCONFIG_H */
