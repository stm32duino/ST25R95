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
 *  \brief ST25R95 high level interface
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "rfal_rfst25r95.h"
#include "st25r95.h"
#include "st25r95_com.h"
#include "nfc_utils.h"

/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL CONSTANTS
******************************************************************************
*/

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::st25r95Initialize(void)
{
  uint32_t attempt = 5;
  ReturnCode retCode = ERR_NONE;

  /* First perform the startup sequence */
  st25r95SPI_nIRQ_IN_Pulse();
  /* Reset ST25R95 */
  st25r95SPIResetChip();
  /* If no answer from ECHO command, reset and retry again up to max attempt */
  while ((st25r95SPICommandEcho() != ERR_NONE) && (attempt != 0)) {
    attempt--;
    st25r95SPIResetChip();
  }
  if (attempt == 0) {
    retCode = ERR_SYSTEM;
  }

  return (retCode);
}


/*******************************************************************************/
void RfalRfST25R95Class::st25r95Deinitialize(void)
{
  /* Reset ST25R95 */
  st25r95SPIResetChip();
}


/*******************************************************************************/
void RfalRfST25R95Class::st25r95SPI_nIRQ_IN_Pulse(void)
{
  digitalWrite(irq_in_pin, HIGH);
  delay(1); /* wait t0 */
  digitalWrite(irq_in_pin, LOW);
  delay(1); /* wait t1 */
  digitalWrite(irq_in_pin, HIGH);
  delay(11); /* wait t3: seems more than 10ms needed */
}


/*******************************************************************************/
void RfalRfST25R95Class::st25r95SPIResetChip(void)
{
  digitalWrite(cs_pin, HIGH);
  delay(1);
  dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
  digitalWrite(cs_pin, LOW);
  /* Send Reset Control byte over SPI */
  st25r95SPISendReceiveByte(ST25R95_CONTROL_RESET);
  delay(1);
  digitalWrite(cs_pin, HIGH);
  dev_spi->endTransaction();
  delay(3);
  st25r95SPI_nIRQ_IN_Pulse();
}


/*******************************************************************************/
ReturnCode RfalRfST25R95Class::st25r95FieldOn(uint32_t protocol)
{
  if (protocol == ST25R95_PROTOCOL_FIELDOFF) {
    protocol = ST25R95_PROTOCOL_ISO15693;
  }
  return (st25r95ProtocolSelect(protocol));
}

ReturnCode RfalRfST25R95Class::st25r95FieldOff(void)
{
  return (st25r95ProtocolSelect(ST25R95_PROTOCOL_FIELDOFF));
}
