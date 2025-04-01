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
 *  \brief Implementation of ST25R95 communication.
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "rfal_rfst25r95.h"
#include "st25r95_com.h"
#include "st25r95.h"
#include "nfc_utils.h"


/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/

/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/

static uint8_t st25r95CommandIDN[] = {ST25R95_COMMAND_IDN, 0x00};

static uint8_t ProtocolSelectCommandFieldOff[]     = {0x02, 0x02, 0x00, 0x00};
static uint8_t ProtocolSelectCommandISO15693[]     = {0x02, 0x02, 0x01, 0x0D};
static uint8_t ProtocolSelectCommandISO14443A[]    = {0x02, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t ProtocolSelectCommandISO14443B[]    = {0x02, 0x05, 0x03, 0x01, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t ProtocolSelectCommandISO18092[]     = {0x02, 0x05, 0x04, 0x51, 0x1F, 0x06, 0x00, 0x00}; /* WA: keep len=5 & do not use DD */
static uint8_t ProtocolSelectCommandCEISO14443A[]  = {0x02, 0x02, 0x12, 0x0A};

static uint8_t *ProtocolSelectCommands[6] = {
  ProtocolSelectCommandFieldOff,
  ProtocolSelectCommandISO15693,
  ProtocolSelectCommandISO14443A,
  ProtocolSelectCommandISO14443B,
  ProtocolSelectCommandISO18092,
  ProtocolSelectCommandCEISO14443A,
};

static uint8_t WrRegAnalogRegConfigISO15693[]   = {0x09, 0x04, 0x68, 0x01, 0x01, 0x53};
static uint8_t WrRegAnalogRegConfigISO14443A[]  = {0x09, 0x04, 0x68, 0x01, 0x01, 0xD3};
static uint8_t WrRegAnalogRegConfigISO14443B[]  = {0x09, 0x04, 0x68, 0x01, 0x01, 0x30};
static uint8_t WrRegAnalogRegConfigISO18092[]   = {0x09, 0x04, 0x68, 0x01, 0x01, 0x50};
static uint8_t WrRegAnalogRegConfigCEISO1443A[] = {0x09, 0x04, 0x68, 0x01, 0x04, 0x27};
static uint8_t *WrRegAnalogRegConfigs[6] = {
  NULL,
  WrRegAnalogRegConfigISO15693,
  WrRegAnalogRegConfigISO14443A,
  WrRegAnalogRegConfigISO14443B,
  WrRegAnalogRegConfigISO18092,
  WrRegAnalogRegConfigCEISO1443A
};

static uint8_t WrRegEnableAutoDetectFilter[] = {0x09, 0x04, 0x0A, 0x01, 0x02, 0xA1};
static uint8_t WrRegTimerWindowValue[]       = {0x09, 0x04, 0x3A, 0x00, 0x58, 0x04};

static uint8_t Calibrate[] = {ST25R95_COMMAND_IDLE, 0x0E, 0x03, 0xA1, 0x00, 0xB8, 0x01, 0x18, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x3F, 0x01};
static uint8_t WrRegAnalogRegConfigIndex[]  = {0x09, 0x03, 0x68, 0x00, 0x01};
static uint8_t RdRegAnalogRegConfig[]       = {0x08, 0x03, 0x69, 0x01, 0x00};

static uint8_t EchoCommand[1] = {ST25R95_COMMAND_ECHO};
static uint8_t Idle[] = {ST25R95_COMMAND_IDLE, 0x0E, 0x0A, 0x21, 0x00, 0x38, 0x01, 0x18, 0x00, 0x20, 0x60, 0x60, 0x74, 0x84, 0x3F, 0x00};

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
bool RfalRfST25R95Class::st25r95CheckChipID(void)
{
  bool retCode = false;
  uint8_t respBuffer[ST25R95_IDN_RESPONSE_BUFLEN];

  if (st25r95SPISendCommandTypeAndLen(st25r95CommandIDN, respBuffer, ST25R95_IDN_RESPONSE_BUFLEN) == ERR_NONE) {
    if (respBuffer[ST25R95_CMD_LENGTH_OFFSET] != 0) {
      retCode = (strcmp((const char *)&respBuffer[ST25R95_CMD_DATA_OFFSET], "NFC FS2JAST4") == 0);
    }
  }
  return (retCode);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::st25r95ProtocolSelect(uint8_t protocol)
{
  ReturnCode retCode;
  uint8_t respBuffer[MAX(ST25R95_PROTOCOLSELECT_RESPONSE_BUFLEN, ST25R95_WRREG_RESPONSE_BUFLEN)];

  retCode = st25r95SPISendCommandTypeAndLen(ProtocolSelectCommands[protocol], respBuffer, ST25R95_PROTOCOLSELECT_RESPONSE_BUFLEN);
  if ((retCode == ERR_NONE) && (respBuffer[ST25R95_CMD_RESULT_OFFSET] != ST25R95_ERRCODE_NONE)) {
    retCode = ERR_PARAM;
  }

  /* Adjust ARC_B or ACC_A register */
  if ((protocol != ST25R95_PROTOCOL_FIELDOFF)) {
    st25r95SPISendCommandTypeAndLen(WrRegAnalogRegConfigs[protocol], respBuffer, ST25R95_WRREG_RESPONSE_BUFLEN);
  }

  if (protocol == ST25R95_PROTOCOL_ISO18092) {
    st25r95SPISendCommandTypeAndLen(WrRegEnableAutoDetectFilter, respBuffer, ST25R95_WRREG_RESPONSE_BUFLEN);
  }
  if (protocol == ST25R95_PROTOCOL_ISO14443A) {
    st25r95SPISendCommandTypeAndLen(WrRegTimerWindowValue, respBuffer, ST25R95_WRREG_RESPONSE_BUFLEN);
  }
#if RFAL_FEATURE_LISTEN_MODE
  if (protocol == ST25R95_PROTOCOL_CE_ISO14443A) {
    st25r95SPIRxCtx.inListen = false;
  }
#endif /* RFAL_FEATURE_LISTEN_MODE */
  return (retCode);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::st25r95SetBitRate(uint8_t protocol, rfalBitRate txBR, rfalBitRate rxBR)
{
  uint8_t *conf;
  if ((protocol == ST25R95_PROTOCOL_FIELDOFF) || (protocol > ST25R95_PROTOCOL_MAX)) {
    return ERR_PARAM;
  }
  conf = &ProtocolSelectCommands[protocol][ST25R95_PROTOCOLSELECT_BR_OFFSET];
  *conf &= 0x0F;

  switch (protocol) {
    case (ST25R95_PROTOCOL_ISO15693):
      switch (rxBR) {
        case (RFAL_BR_26p48):
          break;
        case (RFAL_BR_52p97):
          *conf |= 0x10;
          break;
        default:
          return (ERR_NOT_IMPLEMENTED);
      }
      break;
    case (ST25R95_PROTOCOL_ISO14443A):
      switch (txBR) {
        case (RFAL_BR_106):
          break;
        case (RFAL_BR_212):
          *conf |= 0x40;
          break;
        case (RFAL_BR_424):
          *conf |= 0x80;
          break;
        default:
          return (ERR_NOT_IMPLEMENTED);
      }
      switch (rxBR) {
        case (RFAL_BR_106):
          break;
        case (RFAL_BR_212):
          *conf |= 0x10;
          break;
        case (RFAL_BR_424):
          *conf |= 0x20;
          break;
        default:
          return (ERR_NOT_IMPLEMENTED);
      }
      break;
    case (ST25R95_PROTOCOL_ISO14443B):
      switch (txBR) {
        case (RFAL_BR_106):
          break;
        case (RFAL_BR_212):
          *conf |= 0x40;
          break;
        case (RFAL_BR_424):
          *conf |= 0x80;
          break;
        case (RFAL_BR_848):
          *conf |= 0xC0;
          break;
        default:
          return (ERR_NOT_IMPLEMENTED);
      }
      switch (rxBR) {
        case (RFAL_BR_106):
          break;
        case (RFAL_BR_212):
          *conf |= 0x10;
          break;
        case (RFAL_BR_424):
          *conf |= 0x20;
          break;
        case (RFAL_BR_848):
          *conf |= 0x30;
          break;
        default:
          return (ERR_NOT_IMPLEMENTED);
      }
      break;
    case (ST25R95_PROTOCOL_ISO18092):
      switch (txBR) {
        case (RFAL_BR_212):
          *conf |= 0x40;
          break;
        case (RFAL_BR_424):
          *conf |= 0x80;
          break;
        default:
          return (ERR_NOT_IMPLEMENTED);
      }
      switch (rxBR) {
        case (RFAL_BR_212):
          *conf |= 0x10;
          break;
        case (RFAL_BR_424):
          *conf |= 0x20;
          break;
        default:
          return (ERR_NOT_IMPLEMENTED);
      }
      break;
    case (ST25R95_PROTOCOL_CE_ISO14443A):
      switch (txBR) {
        case (RFAL_BR_106):
          break;
        default:
          return (ERR_NOT_IMPLEMENTED);
      }
      switch (rxBR) {
        case (RFAL_BR_106):
          break;
        default:
          return (ERR_NOT_IMPLEMENTED);
      }
      break;
    default:
      return (ERR_NOT_IMPLEMENTED);
  }
  return (ERR_NONE);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::st25r95SetFWT(uint8_t protocol, uint32_t fwt)
{
  uint8_t PP;
  uint32_t MM;
  uint32_t DD;
  uint32_t FWT;

  FWT = MIN(fwt, ST25R95_FWT_MAX);     /* Limit the FWT to the max supported */
  fwt = FWT;
  PP = 0;

  if (protocol == ST25R95_PROTOCOL_ISO18092) {
    /* Workaround for ST25R95_PROTOCOL_ISO18092:
     * DD parameters seems to overwritten by MM by the ROM code.
     * So this parameter should not be used (i.e ProtocolSelect Len should be 5)
     */
    DD = 0; /* Should not be used in protocolSelect */
    while (FWT > ((128U + 1U) * (128U) * 32U)) {
      PP++;
      FWT /= 2U;
    }
    MM = FWT / (128U * 32U);
  } else {
    while (FWT > ((128 + 1) * (255) * 32)) {
      PP++;
      FWT /= 2;
    }
    do {
      if (FWT > ((64 + 1) * (255) * 32)) {
        MM = 128UL;
        break;
      }
      if (FWT > ((32 + 1) * (255) * 32)) {
        MM = 64UL;
        break;
      }
      if (FWT > ((16 + 1) * (255) * 32)) {
        MM = 32UL;
        break;
      }
      if (FWT > ((8 + 1) * (255) * 32)) {
        MM = 16UL;
        break;
      }
      if (FWT > ((4 + 1) * (255) * 32)) {
        MM = 8UL;
        break;
      }
      if (FWT > ((2 + 1) * (255) * 32)) {
        MM = 4UL;
        break;
      }
      if (FWT > ((1 + 1) * (255) * 32)) {
        MM = 2UL;
        break;
      }
      if (FWT > ((0 + 1) * (255) * 32)) {
        MM = 1UL;
        break;
      }
      MM = 0UL;
    } while (0);

    DD = (((FWT + 31UL) / 32UL) + MM) / (MM + 1UL);
    DD = (DD > 128) ? DD - 128UL : 0;
  }

  switch (protocol) {
    case ST25R95_PROTOCOL_ISO14443A:
      if (
        (ProtocolSelectCommandISO14443A[4] != PP) ||
        (ProtocolSelectCommandISO14443A[5] != MM) ||
        (ProtocolSelectCommandISO14443A[6] != DD)) {
        ProtocolSelectCommandISO14443A[4] = PP;
        ProtocolSelectCommandISO14443A[5] = (uint8_t)MM;
        ProtocolSelectCommandISO14443A[6] = (uint8_t)DD;

        return (st25r95ProtocolSelect(protocol));
      }
      break;

    case ST25R95_PROTOCOL_ISO14443B:
      if (
        (ProtocolSelectCommandISO14443B[4] != PP) ||
        (ProtocolSelectCommandISO14443B[5] != MM) ||
        (ProtocolSelectCommandISO14443B[6] != DD)) {
        ProtocolSelectCommandISO14443B[4] = PP;
        ProtocolSelectCommandISO14443B[5] = (uint8_t)MM;
        ProtocolSelectCommandISO14443B[6] = (uint8_t)DD;

        return (st25r95ProtocolSelect(protocol));
      }
      break;

    case ST25R95_PROTOCOL_ISO18092:
      if (
        (ProtocolSelectCommandISO18092[5] != PP) ||
        (ProtocolSelectCommandISO18092[6] != MM) ||
        (ProtocolSelectCommandISO18092[7] != DD)) {
        ProtocolSelectCommandISO18092[5] = PP;
        ProtocolSelectCommandISO18092[6] = (uint8_t)MM;
        ProtocolSelectCommandISO18092[7] = (uint8_t)DD;

        return (st25r95ProtocolSelect(protocol));
      }
      break;

    default:
      break;
  }
  return (ERR_NONE);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::st25r95SetSlotCounter(uint8_t slots)
{
  if ((ProtocolSelectCommandISO18092[4] & 0xF) != slots) {
    ProtocolSelectCommandISO18092[4] &= 0xF0;
    ProtocolSelectCommandISO18092[4] |= (slots & 0xF);
    return (st25r95ProtocolSelect(ST25R95_PROTOCOL_ISO18092));
  }
  return (ERR_NONE);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::st25r95WriteReg(uint8_t protocol, uint16_t reg, uint8_t value)
{
  ReturnCode retCode;
  uint8_t respBuffer[ST25R95_WRREG_RESPONSE_BUFLEN];

  switch (reg) {
    case (ST25R95_REG_ARC_B):
      if ((protocol == ST25R95_PROTOCOL_ISO15693)  ||
          (protocol == ST25R95_PROTOCOL_ISO14443A) ||
          (protocol == ST25R95_PROTOCOL_ISO14443B) ||
          (protocol == ST25R95_PROTOCOL_ISO18092)) {
        WrRegAnalogRegConfigs[protocol][5] = value;
        st25r95SPISendCommandTypeAndLen(WrRegAnalogRegConfigs[protocol], respBuffer, ST25R95_WRREG_RESPONSE_BUFLEN);
        retCode = (respBuffer[ST25R95_CMD_RESULT_OFFSET] == 0) ? ERR_NONE : ERR_PARAM;
      } else {
        retCode = ERR_WRONG_STATE;
      }
      break;

    case (ST25R95_REG_ACC_A):
      if (protocol == ST25R95_PROTOCOL_CE_ISO14443A) {
        WrRegAnalogRegConfigs[protocol][5] = value;
        st25r95SPISendCommandTypeAndLen(WrRegAnalogRegConfigs[protocol], respBuffer, ST25R95_WRREG_RESPONSE_BUFLEN);
        retCode = (respBuffer[ST25R95_CMD_RESULT_OFFSET] == 0) ? ERR_NONE : ERR_PARAM;
      } else {
        retCode = ERR_WRONG_STATE;
      }
      break;
    default:
      retCode = ERR_PARAM;
      break;
  }

  return (retCode);
}

/*******************************************************************************/
uint8_t RfalRfST25R95Class::st25r95CalibrateTagDetector(void)
{
  const uint8_t steps[6] = {0x80U, 0x40U, 0x20U, 0x10U, 0x08U, 0x4U};
  uint8_t       respBuffer[ST25R95_IDLE_RESPONSE_BUFLEN];
  uint8_t       i;

  /* 8 steps dichotomy implementation as per AN3433 */

  /* Check that wake up detection is tag detect (0x02) when DacDataH is Min Dac value 0x00 */
  Calibrate[ST25R95_IDLE_DACDATAH_OFFSET] = 0x00U;
  st25r95SPISendCommandTypeAndLen(Calibrate, respBuffer, ST25R95_IDLE_RESPONSE_BUFLEN);
  if ((respBuffer[ST25R95_CMD_RESULT_OFFSET] != ST25R95_ERRCODE_NONE) || (respBuffer[ST25R95_CMD_LENGTH_OFFSET] != 0x01) || (respBuffer[ST25R95_CMD_DATA_OFFSET] != ST25R95_IDLE_WKUP_TAGDETECT)) {
    return (0xFFU);
  }
  /* Check that wake up detection is timeout (0x01) when DacDataH is Max Dac value 0xFC */
  Calibrate[ST25R95_IDLE_DACDATAH_OFFSET] = ST25R95_DACDATA_MAX;
  st25r95SPISendCommandTypeAndLen(Calibrate, respBuffer, ST25R95_IDLE_RESPONSE_BUFLEN);
  if ((respBuffer[ST25R95_CMD_RESULT_OFFSET] != ST25R95_ERRCODE_NONE) || (respBuffer[ST25R95_CMD_LENGTH_OFFSET] != 0x01) || (respBuffer[ST25R95_CMD_DATA_OFFSET] != ST25R95_IDLE_WKUP_TIMEOUT)) {
    return (0xFFU);
  }

  for (i = 0; i < 6; i++) {
    switch (respBuffer[ST25R95_CMD_DATA_OFFSET]) {
      case ST25R95_IDLE_WKUP_TIMEOUT:
        Calibrate[ST25R95_IDLE_DACDATAH_OFFSET] -= steps[i];
        break;
      case ST25R95_IDLE_WKUP_TAGDETECT:
        Calibrate[ST25R95_IDLE_DACDATAH_OFFSET] += steps[i];
        break;
      default:
        return ERR_SYSTEM;
        /*NOTREACHED*/
        break;
    }
    respBuffer[ST25R95_CMD_DATA_OFFSET] = 0x00U;
    st25r95SPISendCommandTypeAndLen(Calibrate, respBuffer, ST25R95_IDLE_RESPONSE_BUFLEN);
  }
  if (respBuffer[2U] == ST25R95_IDLE_WKUP_TIMEOUT) {
    Calibrate[ST25R95_IDLE_DACDATAH_OFFSET] -= 0x04U;
  }
  return (Calibrate[ST25R95_IDLE_DACDATAH_OFFSET]);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::st25r95ReadReg(uint16_t reg, uint8_t *value)
{
  ReturnCode retCode = ERR_NONE;
  uint8_t respBuffer[ST25R95_RDREG_RESPONSE_BUFLEN];

  switch (reg) {
    case (ST25R95_REG_ARC_B):
      WrRegAnalogRegConfigIndex[4U] = 0x01U;
      break;

    case (ST25R95_REG_ACC_A):
      WrRegAnalogRegConfigIndex[4U] = 0x04U;
      break;

    default:
      retCode = ERR_PARAM;
      break;
  }
  if (retCode == ERR_NONE) {
    st25r95SPISendCommandTypeAndLen(WrRegAnalogRegConfigIndex, respBuffer, ST25R95_RDREG_RESPONSE_BUFLEN);
    if (respBuffer[ST25R95_CMD_RESULT_OFFSET] == ST25R95_ERRCODE_NONE) {
      st25r95SPISendCommandTypeAndLen(RdRegAnalogRegConfig, respBuffer, ST25R95_RDREG_RESPONSE_BUFLEN);
      if (respBuffer[ST25R95_CMD_RESULT_OFFSET] == ST25R95_ERRCODE_NONE) {
        *value = respBuffer[2];
      } else {
        retCode = ERR_PARAM;
      }
    } else {
      retCode = ERR_PARAM;
    }
  }

  return (retCode);

}

/*******************************************************************************/
void RfalRfST25R95Class::st25r95SPIRxTx(uint8_t *txData, uint8_t *rxData, uint16_t length)
{
  uint8_t txByte = 0;
  uint8_t rxByte;
  uint16_t len = length;

  while (len != 0) {
    if (txData != NULL) {
      txByte = txData[length - len];
    }

    rxByte = dev_spi->transfer(txByte);

    if (rxData != NULL) {
      rxData[length - len] = rxByte;
    }

    len--;
  }
}

/*******************************************************************************/
uint8_t RfalRfST25R95Class::st25r95SPISendReceiveByte(uint8_t data)
{
  uint8_t received_byte;

  st25r95SPIRxTx(&data, &received_byte, 1);
  return (received_byte);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::st25r95SPIPollRead(uint32_t timeout)
{
  uint32_t timer;
  ReturnCode retCode = ERR_NONE;

  timer = timerCalculateTimer(timeout);
  while (digitalRead(irq_out_pin) == HIGH && (timeout != 0) && !timerIsExpired(timer)) {;}

  if (digitalRead(irq_out_pin) == HIGH) {
    retCode = ERR_TIMEOUT;
  }

  return (retCode);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::st25r95SPIPollSend(void)
{
  ReturnCode retCode = ERR_NONE;
  uint8_t response;

  dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
  digitalWrite(cs_pin, LOW);
  st25r95SPISendReceiveByte(ST25R95_CONTROL_POLL);
  response = st25r95SPISendReceiveByte(ST25R95_CONTROL_POLL);
  digitalWrite(cs_pin, HIGH);
  dev_spi->endTransaction();

  if (!ST25R95_POLL_DATA_CAN_BE_SEND(response)) {
    retCode = ERR_TIMEOUT;
  }
  return (retCode);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::st25r95SPISendCommandTypeAndLen(uint8_t *cmd, uint8_t *resp, uint16_t respBuffLen)
{
  ReturnCode retCode = ERR_NONE;
  uint32_t len;

  if (respBuffLen < 2) {
    retCode = ERR_NOMEM;
  } else {
    resp[ST25R95_CMD_RESULT_OFFSET] = ST25R95_ERRCODE_COMERROR;
    resp[ST25R95_CMD_LENGTH_OFFSET] = 0x00;

    /* 1 - Send the  command */
    dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_pin, LOW);
    st25r95SPISendReceiveByte(ST25R95_CONTROL_SEND);
    st25r95SPIRxTx(cmd, NULL, cmd[ST25R95_CMD_LENGTH_OFFSET] + 2);
    digitalWrite(cs_pin, HIGH);
    dev_spi->endTransaction();

    /* 2 - Poll the ST25R95 until it is ready to transmit */
    retCode = st25r95SPIPollRead(ST25R95_CONTROL_POLL_TIMEOUT);

    if (retCode == ERR_NONE) {
      dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
      digitalWrite(cs_pin, LOW);
      st25r95SPISendReceiveByte(ST25R95_CONTROL_READ);
      resp[ST25R95_CMD_RESULT_OFFSET] = st25r95SPISendReceiveByte(ST25R95_SPI_DUMMY_BYTE);
      resp[ST25R95_CMD_LENGTH_OFFSET] = st25r95SPISendReceiveByte(resp[ST25R95_CMD_RESULT_OFFSET]);
      len = resp[ST25R95_CMD_LENGTH_OFFSET];
      /* compute len according to CR95HF DS § 4.4 */
      if ((resp[ST25R95_CMD_RESULT_OFFSET] & 0x8F) == 0x80) {
        len |= (((uint32_t)resp[ST25R95_CMD_RESULT_OFFSET]) & 0x60U) << 3U;
      }
      /* read the len-bytes frame */
      if (respBuffLen >= (len + 2)) {
        if (len != 0) {
          st25r95SPIRxTx(NULL, &resp[ST25R95_CMD_DATA_OFFSET], len);
        }
      } else {
        st25r95SPIRxTx(NULL, NULL, ST25R95_COMMUNICATION_BUFFER_SIZE);
        retCode = ERR_NOMEM;
      }
      digitalWrite(cs_pin, HIGH);
      dev_spi->endTransaction();
    } else {
      dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
      digitalWrite(cs_pin, LOW);
      st25r95SPIRxTx(NULL, NULL, ST25R95_COMMUNICATION_BUFFER_SIZE);
      digitalWrite(cs_pin, HIGH);
      dev_spi->endTransaction();
      retCode = ERR_SYSTEM;
    }
  }
  return (retCode);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::st25r95SPICommandEcho(void)
{
  ReturnCode retCode = ERR_NONE;
  uint8_t respBuffer[ST25R95_ECHO_RESPONSE_BUFLEN];

  /* 0 - Poll the ST25R95 to make sure data can be send */
  /* Used only in cas of ECHO Command as this command is sent just after the ST25R95 reset */
  retCode = st25r95SPIPollSend();

  if (retCode == ERR_NONE) {
    /* 1 - Send the echo command */
    dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_pin, LOW);
    st25r95SPISendReceiveByte(ST25R95_CONTROL_SEND);
    st25r95SPISendReceiveByte(EchoCommand[0]);
    digitalWrite(cs_pin, HIGH);
    dev_spi->endTransaction();

    /* 2 - Poll the ST25R95 until it is ready to transmit */
    retCode = st25r95SPIPollRead(ST25R95_CONTROL_POLL_TIMEOUT);

    /* 3 - Read echo response */
    if (retCode == ERR_NONE) {
      dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
      digitalWrite(cs_pin, LOW);
      st25r95SPISendReceiveByte(ST25R95_CONTROL_READ);
      respBuffer[ST25R95_CMD_RESULT_OFFSET] = st25r95SPISendReceiveByte(ST25R95_SPI_DUMMY_BYTE);
      /* Read 2 additional bytes. See  ST95HF DS §5.7 :
       * The ECHO command (0x55) allows exiting Listening mode.
       * In response to the ECHO command, the ST25R95 sends 0x55 + 0x8500 (error code of the Listening state cancelled by the MCU).
       */
      respBuffer[1] = st25r95SPISendReceiveByte(ST25R95_SPI_DUMMY_BYTE);
      respBuffer[2] = st25r95SPISendReceiveByte(ST25R95_SPI_DUMMY_BYTE);
      digitalWrite(cs_pin, HIGH);
      dev_spi->endTransaction();

      if (respBuffer[ST25R95_CMD_RESULT_OFFSET] != ST25R95_COMMAND_ECHO) {
        st25r95SPIRxTx(NULL, NULL, ST25R95_COMMUNICATION_BUFFER_SIZE);
        retCode = ERR_SYSTEM;
      }
    }
  }
#if RFAL_FEATURE_LISTEN_MODE
  st25r95SPIRxCtx.inListen = false;
#endif /* RFAL_FEATURE_LISTEN_MODE */
  return (retCode);
}

/*******************************************************************************/
void RfalRfST25R95Class::st25r95SPISendData(uint8_t *buf, uint8_t bufLen, uint8_t protocol, uint32_t flags)
{
  uint8_t len;

  dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
  digitalWrite(cs_pin, LOW);
  st25r95SPISendReceiveByte(ST25R95_CONTROL_SEND);
  if (protocol == ST25R95_PROTOCOL_CE_ISO14443A) {
    /* Card Emulation mode */
    st25r95SPISendReceiveByte(ST25R95_COMMAND_SEND);
  } else {
    st25r95SPISendReceiveByte(ST25R95_COMMAND_SENDRECV);
  }
  /* add transmission Flag Len in case of 14443A */
  len = ((protocol == ST25R95_PROTOCOL_ISO14443A) || (protocol == ST25R95_PROTOCOL_CE_ISO14443A)) ? bufLen + 1 : bufLen;
  /* add SoD len in case of ISO14443A + NFCIP1 */
  len += ((protocol == ST25R95_PROTOCOL_ISO14443A) && ((flags & RFAL_TXRX_FLAGS_NFCIP1_ON) == RFAL_TXRX_FLAGS_NFCIP1_ON)) ? 2 : 0;
  st25r95SPISendReceiveByte(len);
  if ((protocol == ST25R95_PROTOCOL_ISO14443A) && ((flags & RFAL_TXRX_FLAGS_NFCIP1_ON) == RFAL_TXRX_FLAGS_NFCIP1_ON)) {
    st25r95SPISendReceiveByte(0xF0U);
    st25r95SPISendReceiveByte(bufLen + 1); /* DP 2.0 17.4.1.3 The SoD SHALL contain a length byte LEN at the position shown in Figure 43 with a value equal to n+1, where n indicates the number of bytes the payload consists of.*/
  }
  st25r95SPIRxTx(buf, NULL, bufLen);

}

/*******************************************************************************/
void RfalRfST25R95Class::st25r95SPISendTransmitFlag(uint8_t protocol, uint8_t transmitFlag)
{
  if ((protocol == ST25R95_PROTOCOL_ISO14443A) || (protocol == ST25R95_PROTOCOL_CE_ISO14443A)) {
    /* send transmission Flag */
    st25r95SPISendReceiveByte(transmitFlag);
  }

  digitalWrite(cs_pin, HIGH);
  dev_spi->endTransaction();
}

/*******************************************************************************/
void RfalRfST25R95Class::st25r95SPIPrepareRx(uint8_t protocol, uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *rxRcvdLen, uint32_t flags, uint8_t *additionalRespBytes)
{
  st25r95SPIRxCtx.protocol            = protocol;
  st25r95SPIRxCtx.rxBuf               = rxBuf;
  st25r95SPIRxCtx.rxBufLen            = rxBufLen;
  st25r95SPIRxCtx.rxRcvdLen           = rxRcvdLen;
  st25r95SPIRxCtx.rmvCRC              = ((flags & RFAL_TXRX_FLAGS_CRC_RX_KEEP) != RFAL_TXRX_FLAGS_CRC_RX_KEEP);
  st25r95SPIRxCtx.NFCIP1              = ((protocol == ST25R95_PROTOCOL_ISO14443A) && ((flags & RFAL_TXRX_FLAGS_NFCIP1_ON) == RFAL_TXRX_FLAGS_NFCIP1_ON));
  st25r95SPIRxCtx.additionalRespBytes = additionalRespBytes;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::st25r95SPICompleteRx(void)
{
  uint8_t Result;
  uint16_t len;
  uint16_t rcvdLen;
  rfalBitRate rxBr;
  ReturnCode retCode = ERR_NONE;
  uint16_t additionalRespBytesNb = 1;

  dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
  digitalWrite(cs_pin, LOW);
  st25r95SPISendReceiveByte(ST25R95_CONTROL_READ);
  Result = st25r95SPISendReceiveByte(ST25R95_SPI_DUMMY_BYTE);
  len = st25r95SPISendReceiveByte(ST25R95_SPI_DUMMY_BYTE);

  /* compute len according to CR95HF DS § 4.4 */
  if ((Result & 0x8F) == 0x80) {
    len |= (((uint32_t)Result) & 0x60) << 3;
    Result &= 0x9F;
  }
  rcvdLen = 0;

  switch (Result) {
    case ST25R95_ERRCODE_NONE:
    case ST25R95_ERRCODE_FRAMEOKADDITIONALINFO:
    case ST25R95_ERRCODE_RESULTSRESIDUAL:
      break;
    case ST25R95_ERRCODE_COMERROR:
      retCode = ERR_INTERNAL;
      break;
    case ST25R95_ERRCODE_FRAMEWAITTIMEOUT:
      retCode = ERR_TIMEOUT;
      break;
    case ST25R95_ERRCODE_OVERFLOW:
      retCode = ERR_HW_OVERRUN;
      break;
    case ST25R95_ERRCODE_INVALIDSOF:
    case ST25R95_ERRCODE_RECEPTIONLOST:
    case ST25R95_ERRCODE_FRAMING:
    case ST25R95_ERRCODE_EGT:
    case ST25R95_ERRCODE_61_SOF:
    case ST25R95_ERRCODE_63_SOF_HIGH:
    case ST25R95_ERRCODE_65_SOF_LOW:
    case ST25R95_ERRCODE_66_EGT:
    case ST25R95_ERRCODE_67_TR1TOOLONG:
    case ST25R95_ERRCODE_68_TR1TOOSHORT:
      retCode = ERR_FRAMING;
      break;
    case ST25R95_ERRCODE_62_CRC:
      retCode = ERR_CRC;
      break;
    case ST25R95_ERRCODE_NOFIELD:
      retCode = ERR_LINK_LOSS;
      break;
    default:
      retCode = ERR_SYSTEM;
      break;
  }

  if ((retCode != ERR_NONE) && (len != 0)) {
    st25r95SPIRxTx(NULL, NULL, ST25R95_COMMUNICATION_BUFFER_SIZE);
    len = 0;
  }


  /* In ISO14443A 106kbps 2 additional bytes of collision information are provided */
  rfalGetBitRate(NULL, &rxBr);
  if ((st25r95SPIRxCtx.protocol == ST25R95_PROTOCOL_ISO14443A) && (rxBr == RFAL_BR_106)) {
    additionalRespBytesNb += 2;
  }


  /* read the frame */
  do {
    if (len == 0) {
      additionalRespBytesNb = 0;
      break;
    }
    if (len < additionalRespBytesNb) {
      /* Flush ST25R95 fifo */
      st25r95SPIRxTx(NULL, NULL, ST25R95_COMMUNICATION_BUFFER_SIZE);
      retCode = ERR_SYSTEM;
      break;
    }
    len -= additionalRespBytesNb;
    if ((Result == ST25R95_ERRCODE_RESULTSRESIDUAL) && (st25r95SPIRxCtx.protocol == ST25R95_PROTOCOL_ISO14443A)) {
      st25r95SPIRxCtx.rmvCRC = false;
    }
    if ((st25r95SPIRxCtx.rmvCRC) && (st25r95SPIRxCtx.protocol != ST25R95_PROTOCOL_ISO18092)) {
      if (len < 2) {
        /* Flush ST25R95 fifo */
        st25r95SPIRxTx(NULL, NULL, ST25R95_COMMUNICATION_BUFFER_SIZE);
        additionalRespBytesNb = 0;
        retCode = ERR_SYSTEM;
        break;
      }
      len -= 2;
    }
    if ((st25r95SPIRxCtx.NFCIP1) && (len >= 1)) {
      st25r95SPIRxTx(NULL, st25r95SPIRxCtx.NFCIP1_SoD, 1);
      len -= 1;
    }
    if ((len > st25r95SPIRxCtx.rxBufLen) ||
        ((st25r95SPIRxCtx.protocol == ST25R95_PROTOCOL_ISO18092) && ((len + 1U) > st25r95SPIRxCtx.rxBufLen)) || /* Need one extra byte room to prepend Len byte in rxBuf in case of Felica */
        ((!st25r95SPIRxCtx.rmvCRC) && (st25r95SPIRxCtx.protocol == ST25R95_PROTOCOL_ISO18092) && ((len + 3U) > st25r95SPIRxCtx.rxBufLen))) { /* same + 2 extra bytes room to append CRC */
      /* Flush ST25R95 fifo */
      st25r95SPIRxTx(NULL, NULL, ST25R95_COMMUNICATION_BUFFER_SIZE);
      additionalRespBytesNb = 0;
      retCode = ERR_NOMEM;
      break;
    }
    rcvdLen = len;
    if (len != 0) {
      if (st25r95SPIRxCtx.protocol == ST25R95_PROTOCOL_ISO18092) {
        st25r95SPIRxTx(NULL, &st25r95SPIRxCtx.rxBuf[RFAL_NFCF_LENGTH_LEN], len);
        rcvdLen += RFAL_NFCF_LENGTH_LEN;
        len += RFAL_NFCF_LENGTH_LEN;
        st25r95SPIRxCtx.rxBuf[0] = (uint8_t)(rcvdLen & 0xFFU);
      } else {
        st25r95SPIRxTx(NULL, st25r95SPIRxCtx.rxBuf, len);
      }
    }
    if ((st25r95SPIRxCtx.rmvCRC) && (st25r95SPIRxCtx.protocol != ST25R95_PROTOCOL_ISO18092)) {
      st25r95SPIRxTx(NULL, st25r95SPIRxCtx.BufCRC, 2);
    }
    st25r95SPIRxTx(NULL, st25r95SPIRxCtx.additionalRespBytes, additionalRespBytesNb);

    /* check collision and CRC error */
    switch (st25r95SPIRxCtx.protocol) {
      case (ST25R95_PROTOCOL_ISO15693):
        retCode = ST25R95_IS_PROT_ISO15693_COLLISION_ERR(st25r95SPIRxCtx.additionalRespBytes[0]) ? ERR_RF_COLLISION : (ST25R95_IS_PROT_ISO15693_CRC_ERR(st25r95SPIRxCtx.additionalRespBytes[0]) ? ERR_CRC : retCode);
        break;
      case (ST25R95_PROTOCOL_ISO14443A):
        retCode = (Result == ST25R95_ERRCODE_RESULTSRESIDUAL) ? ((ReturnCode)(ERR_INCOMPLETE_BYTE + ((st25r95SPIRxCtx.additionalRespBytes[0] & 0xFU) % 8U))) : (ST25R95_IS_PROT_ISO14443A_COLLISION_ERR(st25r95SPIRxCtx.additionalRespBytes[0]) ? ERR_RF_COLLISION : (ST25R95_IS_PROT_ISO14443A_PARITY_ERR(st25r95SPIRxCtx.additionalRespBytes[0]) ? ERR_PAR : (ST25R95_IS_PROT_ISO14443A_CRC_ERR(st25r95SPIRxCtx.additionalRespBytes[0]) ? ERR_CRC : retCode)));
        break;
      case (ST25R95_PROTOCOL_ISO14443B):
        if (ST25R95_IS_PROT_ISO14443B_CRC_ERR(st25r95SPIRxCtx.additionalRespBytes[0])) {
          retCode = ERR_CRC;
        }
        break;
      case (ST25R95_PROTOCOL_ISO18092):
        if (ST25R95_IS_PROT_ISO18092_CRC_ERR(st25r95SPIRxCtx.additionalRespBytes[0])) {
          retCode = ERR_CRC;
        }
        break;
      default:
        break;
    }
  } while (0);

  digitalWrite(cs_pin, HIGH);
  dev_spi->endTransaction();

  if ((!st25r95SPIRxCtx.rmvCRC) && (st25r95SPIRxCtx.protocol == ST25R95_PROTOCOL_ISO18092) && (rcvdLen == len)) {
    /* increase room for CRC*/
    st25r95SPIRxCtx.rxBuf[rcvdLen++] = 0x00;
    st25r95SPIRxCtx.rxBuf[rcvdLen++] = 0x00;
  }

  /* update *rxRcvdLen if not null pointer */
  if (st25r95SPIRxCtx.rxRcvdLen != NULL) {
    (*st25r95SPIRxCtx.rxRcvdLen) = rcvdLen;
  }
#if RFAL_FEATURE_LISTEN_MODE
  if (st25r95SPIRxCtx.protocol == ST25R95_PROTOCOL_CE_ISO14443A) {
    st25r95SPIRxCtx.inListen = false;
    st25r95SPIGetLmState(); /* store lmState */
  }
#endif /* RFAL_FEATURE_LISTEN_MODE */
  st25r95SPIRxCtx.retCode = retCode;
  return (retCode);
}

/*******************************************************************************/

bool RfalRfST25R95Class::st25r95SPIIsTransmitCompleted(void)
{
  return (true);
}

bool RfalRfST25R95Class::st25r95SPIIsInListen(void)
{
  return (st25r95SPIRxCtx.inListen);
}

/*******************************************************************************/
void RfalRfST25R95Class::st25r95SPIIdle(uint8_t dacDataL, uint8_t dacDataH, uint8_t WUPeriod)
{
  Idle[ST25R95_IDLE_WUPERIOD_OFFSET] = WUPeriod;
  Idle[ST25R95_IDLE_DACDATAL_OFFSET] = dacDataL;
  Idle[ST25R95_IDLE_DACDATAH_OFFSET] = dacDataH;
  dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
  digitalWrite(cs_pin, LOW);
  st25r95SPISendReceiveByte(ST25R95_CONTROL_SEND);
  st25r95SPIRxTx(Idle, NULL, Idle[ST25R95_CMD_LENGTH_OFFSET] + 2);
  digitalWrite(cs_pin, HIGH);
  dev_spi->endTransaction();
}

/*******************************************************************************/
void RfalRfST25R95Class::st25r95SPIGetIdleResponse(void)
{
  uint8_t respBuffer[ST25R95_IDLE_RESPONSE_BUFLEN];

  dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
  digitalWrite(cs_pin, LOW);
  st25r95SPISendReceiveByte(ST25R95_CONTROL_READ);
  respBuffer[ST25R95_CMD_RESULT_OFFSET] = st25r95SPISendReceiveByte(ST25R95_SPI_DUMMY_BYTE);
  respBuffer[ST25R95_CMD_LENGTH_OFFSET] = st25r95SPISendReceiveByte(respBuffer[ST25R95_CMD_RESULT_OFFSET]);
  if ((sizeof(respBuffer)) >= (respBuffer[ST25R95_CMD_LENGTH_OFFSET] + 2U)) {
    if (respBuffer[ST25R95_CMD_LENGTH_OFFSET] != 0) {
      st25r95SPIRxTx(NULL, &respBuffer[ST25R95_CMD_DATA_OFFSET], respBuffer[ST25R95_CMD_LENGTH_OFFSET]);
    }
  } else {
    st25r95SPIRxTx(NULL, NULL, ST25R95_COMMUNICATION_BUFFER_SIZE);
  }
  digitalWrite(cs_pin, HIGH);
  dev_spi->endTransaction();
}

/*******************************************************************************/
void RfalRfST25R95Class::st25r95SPIKillIdle(void)
{
  ReturnCode retCode = ERR_NONE;

  st25r95SPI_nIRQ_IN_Pulse();
  /* Poll the ST25R95 until it is ready to transmit */
  retCode = st25r95SPIPollRead(ST25R95_CONTROL_POLL_TIMEOUT);

  if (retCode == ERR_NONE) {
    st25r95SPIGetIdleResponse();
  }

}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/

