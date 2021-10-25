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
 *  \brief ST25R95 communication declaration file
 *
 */
/*!
 * This driver provides basic abstraction for communication with the ST25R95.
 * It uses the SPI driver for interfacing with the ST25R95.
 *
 *
 * \addtogroup RFAL
 * @{
 *
 * \addtogroup RFAL-HAL
 * \brief RFAL Hardware Abstraction Layer
 * @{
 *
 * \addtogroup ST25R95
 * \brief RFAL ST25R95 Driver
 * @{
 *
 * \addtogroup ST25R95_Com
 * \brief RFAL ST25R95 Communication
 * @{
 *
 */

#ifndef ST25R95_COM_H
#define ST25R95_COM_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/

/* See ST95HF DS §4.1.1 or CR95HF DS §4.2.1 */
#define ST25R95_CONTROL_SEND                             0x00 /*!< Send command to the ST25R95 */
#define ST25R95_CONTROL_RESET                            0x01 /*!< Reset the ST25R95           */
#define ST25R95_CONTROL_READ                             0x02 /*!< Read data from the ST25R95  */
#define ST25R95_CONTROL_POLL                             0x03 /*!< Poll the ST25R95            */

#define ST25R95_CONTROL_POLL_TIMEOUT                      100 /*!< Polling timeout             */
#define ST25R95_CONTROL_POLL_NO_TIMEOUT                     0 /*!< non blocking polling        */

/* See ST95HF DS §5.2 or CR95HF DS §5.2 */
#define ST25R95_COMMAND_IDN                              0x01 /*!< Requests short information about the ST25R95 and its revision.                           */
#define ST25R95_COMMAND_PROTOCOLSELECT                   0x02 /*!< Selects the RF communication protocol and specifies certain protocol-related parameters. */
#define ST25R95_COMMAND_POLLFIED                         0x03 /*!< Returns the current value of the FieldDet flag (used in Card Emulation mode).            */
#define ST25R95_COMMAND_SENDRECV                         0x04 /*!< Sends data using the previously selected protocol and receives the tag response.         */
#define ST25R95_COMMAND_LISTEN                           0x05 /*!< Listens for data using previously selected protocol (used in Card Emulation mode).       */
#define ST25R95_COMMAND_SEND                             0x06 /*!< Sends data using previously selected protocol (used in Card Emulation mode).             */
#define ST25R95_COMMAND_IDLE                             0x07 /*!< Switches the ST25R95 into a low consumption                                              */
#define ST25R95_COMMAND_RDREG                            0x08 /*!< Reads Wake-up event register or the Analog Register Configuration (ARC_B) register       */
#define ST25R95_COMMAND_WRREG                            0x09 /*!< Write register                                                                           */
#define ST25R95_COMMAND_BAUDRATE                         0x0A /*!< Sets the UART baud rate.                                                                 */
#define ST25R95_COMMAND_ACFILTER                         0x0D /*!< Enables or disables the anti-collision filter for ISO/IEC 14443 Type A protocol.         */
#define ST25R95_COMMAND_ECHO                             0x55 /*!< ST25R95 performs a serial interface ECHO command                                         */

#define ST25R95_SPI_DUMMY_BYTE                           0x00 /*!< Dummy byte when nothing to transmit on SPI                                               */

#define ST25R95_CMD_COMMAND_OFFSET                       0x00U /*!< CMD Offset. See CR95HF DS § 4.2.1                        */
#define ST25R95_CMD_RESULT_OFFSET                        0x00U /*!< Resp Code Offset. See CR95HF DS § 4.2.1                  */
#define ST25R95_CMD_LENGTH_OFFSET                        0x01U /*!< LEN Offset. See CR95HF DS § 4.2.1                        */
#define ST25R95_CMD_DATA_OFFSET                          0x02U /*!< DATA[0] Offset. See CR95HF DS § 4.2.1                    */

#define ST25R95_COMMUNICATION_BUFFER_SIZE                (528 + 2) /*!< Max received buffer size                            */
#define ST25R95_COMMUNICATION_UART_WDOGTIMER             (5000U)   /*!< Uart watchdog timer                                 */

#define ST25R95_PROTOCOL_FIELDOFF                        0x00U /*!< ProtocolSelect protocol code: Field OFF                  */
#define ST25R95_PROTOCOL_ISO15693                        0x01U /*!< ProtocolSelect protocol code: ISO15693  (Reader)         */
#define ST25R95_PROTOCOL_ISO14443A                       0x02U /*!< ProtocolSelect protocol code: ISO14443A (Reader)         */
#define ST25R95_PROTOCOL_ISO14443B                       0x03U /*!< ProtocolSelect protocol code: ISO14443B (Reader)         */
#define ST25R95_PROTOCOL_ISO18092                        0x04U /*!< ProtocolSelect protocol code: ISO18092  (Reader)         */
#define ST25R95_PROTOCOL_CE_ISO14443A                    0x05U /*!< ProtocolSelect protocol code: ISO14443  (Card Emulation) */

#define ST25R95_PROTOCOL_MAX                             ST25R95_PROTOCOL_CE_ISO14443A /*!< Max value of protocol field */

#define ST25R95_FWT_MAX                                  0x40A8BC0 /*!< Max FWT supported: 5s */

#define ST25R95_ERRCODE_NONE                             0x00 /*!< no error occurred */
#define ST25R95_ERRCODE_FRAMEOKADDITIONALINFO            0x80 /*!< Frame correctly received (additionally see CRC/Parity information) */
#define ST25R95_ERRCODE_INVALIDCMDLENGHT                 0x82 /*!< Invalid command length */
#define ST25R95_ERRCODE_INVALIDPROTOCOL                  0x83 /*!< Invalid protocol */
#define ST25R95_ERRCODE_COMERROR                         0x86 /*!< Hardware communication error */
#define ST25R95_ERRCODE_FRAMEWAITTIMEOUT                 0x87 /*!< Frame wait time out (no valid reception) */
#define ST25R95_ERRCODE_INVALIDSOF                       0x88 /*!< Invalid SOF */
#define ST25R95_ERRCODE_OVERFLOW                         0x89 /*!< Too many bytes received and data still arriving */
#define ST25R95_ERRCODE_FRAMING                          0x8A /*!< if start bit = 1 or stop bit = 0 */
#define ST25R95_ERRCODE_EGT                              0x8B /*!< EGT time out */
#define ST25R95_ERRCODE_FIELDLENGTH                      0x8C /*!< Valid for ISO/IEC 18092, if Length <3 */
#define ST25R95_ERRCODE_CRC                              0x8D /*!< CRC error, Valid only for ISO/IEC 18092 */
#define ST25R95_ERRCODE_RECEPTIONLOST                    0x8E /*!< When reception is lost without EOF received (or subcarrier was lost) */
#define ST25R95_ERRCODE_NOFIELD                          0x8F /*!< When Listen command detects the absence of external field */
#define ST25R95_ERRCODE_RESULTSRESIDUAL                  0x90 /*!< Residual bits in last byte. Useful for ACK/NAK reception of ISO/IEC 14443 Type A. */
#define ST25R95_ERRCODE_61_SOF                           0x61 /*!< SOF error during the EMD process */
#define ST25R95_ERRCODE_62_CRC                           0x62 /*!< CRC error during the EMD process */
#define ST25R95_ERRCODE_63_SOF_HIGH                      0x63 /*!< SOF error in ISO14443B occurs during high part (duration of 2 to 3 Elementary Unit Time, ETU) */
#define ST25R95_ERRCODE_65_SOF_LOW                       0x65 /*!< SOF error in ISO14443B occurs during low part (duration of 10 to 11 Elementary Unit Time, ETU) */
#define ST25R95_ERRCODE_66_EGT                           0x66 /*!< Extra Guard Time (EGT) error in ISO14443B */
#define ST25R95_ERRCODE_67_TR1TOOLONG                    0x67 /*!< TR1 set by card too long in case of protocol ISO14443B */
#define ST25R95_ERRCODE_68_TR1TOOSHORT                   0x68 /*!< TTR1 set by card too short in case of protocol ISO14443B */

#define ST25R95_REG_ARC_B                                0x6801 /*!< ARC_B register address */
#define ST25R95_REG_ACC_A                                0x6804 /*!< ACC_A register address */
#define ST25R95_REG_TIMERW                               0x3A00 /*!< TIMER register address */

#define ST25R95_IDN_RESPONSE_BUFLEN                              (15 + 2) /*!< IDN response buffer len */
#define ST25R95_PROTOCOLSELECT_RESPONSE_BUFLEN                   (0  + 2) /*!< ProtocolSelect response buffer len */
#define ST25R95_POLLFIELD_RESPONSE_BUFLEN                        (1  + 2) /*!< PollField response buffer len */
#define ST25R95_LISTEN_RESPONSE_BUFLEN                           (0  + 2) /*!< Listen response buffer len */
#define ST25R95_RDREG_RESPONSE_BUFLEN                            (1  + 2) /*!< ReadReg response buffer len */
#define ST25R95_WRREG_RESPONSE_BUFLEN                            (0  + 2) /*!< WriteRead response buffer len */
#define ST25R95_ACFILTER_RESPONSE_BUFLEN                         (1  + 2) /*!< ACFilter response buffer len */
#define ST25R95_IDLE_RESPONSE_BUFLEN                             (1  + 2) /*!< Idle response buffer len */
#define ST25R95_ECHO_RESPONSE_BUFLEN                             (3)      /*!< Echo response buffer len */
#define ST25R95_SEND_RESPONSE_BUFLEN                             (0  + 2) /*!< Send response buffer len */

#define ST25R95_ACSTATE_IDLE                             0x00U /*!< AC Filter state: Idle */
#define ST25R95_ACSTATE_READYA                           0x01U /*!< AC Filter state: ReadyA */
#define ST25R95_ACSTATE_ACTIVE                           0x04U /*!< AC Filter state: Active */
#define ST25R95_ACSTATE_HALT                             0x80U /*!< AC Filter state: Halt */
#define ST25R95_ACSTATE_READYAX                          0x81U /*!< AC Filter state: ReadyA* */
#define ST25R95_ACSTATE_ACTIVEX                          0x84U /*!< AC Filter state: Active* */

#define ST25R95_IS_PROT_ISO15693_CRC_ERR(status)        (((status) & 0x02U) == 0x02U) /*!< Test for CRC flag in SendRcv response additional byte for ISO15693 protocol */
#define ST25R95_IS_PROT_ISO15693_COLLISION_ERR(status)  (((status) & 0x01U) == 0x01U) /*!< Test for Collision flag in SendRcv response additional byte for ISO15693 protocol*/

#define ST25R95_IS_PROT_ISO14443A_COLLISION_ERR(status) (((status) & 0x80U) == 0x80U) /*!< Test for Collision flag in SendRcv response additional byte for ISO14443A protocol */
#define ST25R95_IS_PROT_ISO14443A_CRC_ERR(status)       (((status) & 0x20U) == 0x20U) /*!< Test for CRC       flag in SendRcv response additional byte for ISO14443A protocol */
#define ST25R95_IS_PROT_ISO14443A_PARITY_ERR(status)    (((status) & 0x10U) == 0x10U) /*!< Test for Parity    flag in SendRcv response additional byte for ISO14443A protocol */

#define ST25R95_IS_PROT_ISO14443B_CRC_ERR(status)       (((status) & 0x02U) == 0x02U) /*!< Test for CRC       flag in SendRcv response additional byte for ISO14443B protocol */

#define ST25R95_IS_PROT_ISO18092_CRC_ERR(status)        (((status) & 0x02U) == 0x02U) /*!< Test for CRC       flag in SendRcv response additional byte for ISO18092  protocol */

#define ST25R95_PROTOCOLSELECT_BR_OFFSET                (3U) /*!< Bit Rate offset in ProtocolSelect Command */

#define ST25R95_IDLE_WUPERIOD_OFFSET                    (0x09U) /*!< WUPeriod offset in Idle Command  */
#define ST25R95_IDLE_DACDATAL_OFFSET                    (0x0CU) /*!< DacDataL offset in Idle Command  */
#define ST25R95_IDLE_DACDATAH_OFFSET                    (0x0DU) /*!< DacDataH offset in Idle Command  */
#define ST25R95_DACDATA_MAX                             (0xFCU) /*!< DacData max value (6 bits MSB)   */
#define ST25R95_IDLE_WKUP_TIMEOUT                       (0x01U) /*!< Idle wakeup source: timeout      */
#define ST25R95_IDLE_WKUP_TAGDETECT                     (0x02U) /*!< Idle wakeup source: Tag Detected */


/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/


/*
******************************************************************************
* GLOBAL TYPES
******************************************************************************
*/


/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/


#endif /* ST25R95_COM_H */

/**
  * @}
  *
  * @}
  *
  * @}
  *
  * @}
  */
