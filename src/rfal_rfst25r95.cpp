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
 *  \brief RF Abstraction Layer (RFAL)
 *
 *  RFAL implementation for ST25R95
 */


/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "rfal_rfst25r95.h"

/*******************************************************************************/
RfalRfST25R95Class::RfalRfST25R95Class(SPIClass *spi, int cs_pin, int irq_in_pin, int irq_out_pin, int interface_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), irq_in_pin(irq_in_pin), irq_out_pin(irq_out_pin), interface_pin(interface_pin), spi_speed(spi_speed)
{
  memset(&gRFAL, 0, sizeof(rfal));
  memset(&gRfalAnalogConfigMgmt, 0, sizeof(rfalAnalogConfigMgmt));
  memset(&iso15693PhyConfig, 0, sizeof(iso15693PhyConfig_t));
  memset(&st25r95SPIRxCtx, 0, sizeof(st25r95SPIRxContext));
  timerStopwatchTick = 0;
}


/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalInitialize(void)
{
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);

  pinMode(irq_in_pin, OUTPUT);
  digitalWrite(irq_in_pin, LOW);

  pinMode(irq_out_pin, INPUT);

  /* If interface pin is passed in the constructor, put it HIGH to enable SPI mode */
  if (interface_pin) {
    pinMode(interface_pin, OUTPUT);
    digitalWrite(interface_pin, HIGH);
  }

  rfalAnalogConfigInitialize();              /* Initialize RFAL's Analog Configs */

  /* Ensure that no previous operation is still ongoing */
  if (rfalChipIsBusy()) {
    return ERR_REQUEST;
  }

  /* Initialize chip */
  if (st25r95Initialize() != ERR_NONE) {
    return (ERR_SYSTEM);
  }

  /* Check expected chip: ST25R95 */
  if (!st25r95CheckChipID()) {
    return ERR_HW_MISMATCH;
  }

  /*******************************************************************************/
  /* Debug purposes */
  /*LogSetLevel(LOG_MODULE_DEFAULT, LOG_LEVEL_INFO); !!!!!!!!!!!!!!! */

  /*******************************************************************************/
  gRFAL.state              = RFAL_STATE_INIT;
  gRFAL.mode               = RFAL_MODE_NONE;
  gRFAL.protocol           = ST25R95_PROTOCOL_FIELDOFF;
  gRFAL.field              = false;

  /* Disable all timings */
  gRFAL.timings.FDTListen  = RFAL_TIMING_NONE;
  gRFAL.timings.FDTPoll    = RFAL_TIMING_NONE;
  gRFAL.timings.GT         = RFAL_TIMING_NONE;

  gRFAL.tmr.GT             = RFAL_TIMING_NONE;
  gRFAL.tmr.FDTPoll        = RFAL_TIMING_NONE;

  gRFAL.callbacks.preTxRx  = NULL;
  gRFAL.callbacks.postTxRx = NULL;

  /* Initialize Wake-Up Mode */
  gRFAL.wum.state = RFAL_WUM_STATE_NOT_INIT;
  gRFAL.wum.CalTagDet = ST25R95_TAGDETECT_DEF_CALIBRATION;

  if (gRFAL.wum.CalTagDet == 0xFFU) {
    return ERR_SYSTEM;
  }
  return ERR_NONE;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalCalibrate(void)
{
  return (ERR_NONE);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalAdjustRegulators(uint16_t *result)
{
  NO_WARNING(result);
  return (ERR_NONE);
}

/*******************************************************************************/
void RfalRfST25R95Class::rfalSetUpperLayerCallback(rfalUpperLayerCallback pFunc)
{
  NO_WARNING(pFunc);
  return;
}

/*******************************************************************************/
void RfalRfST25R95Class::rfalSetPreTxRxCallback(rfalPreTxRxCallback pFunc)
{
  gRFAL.callbacks.preTxRx = pFunc;
}


/*******************************************************************************/
void RfalRfST25R95Class::rfalSetPostTxRxCallback(rfalPostTxRxCallback pFunc)
{
  gRFAL.callbacks.postTxRx = pFunc;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalDeinitialize(void)
{
  /* Ensure that no previous operation is still ongoing */
  if (rfalChipIsBusy()) {
    return ERR_REQUEST;
  }

  /* Deinitialize chip */
  st25r95Deinitialize();

  gRFAL.state = RFAL_STATE_IDLE;
  return ERR_NONE;
}

/*******************************************************************************/
void RfalRfST25R95Class::rfalSetObsvMode(uint8_t txMode, uint8_t rxMode)
{
  NO_WARNING(txMode);
  NO_WARNING(rxMode);
  return;
}


/*******************************************************************************/
void RfalRfST25R95Class::rfalGetObsvMode(uint8_t *txMode, uint8_t *rxMode)
{
  NO_WARNING(txMode);
  NO_WARNING(rxMode);
  return;
}

/*******************************************************************************/
void RfalRfST25R95Class::rfalDisableObsvMode(void)
{
  return;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalSetMode(rfalMode mode, rfalBitRate txBR, rfalBitRate rxBR)
{
  /* Check if RFAL is not initialized */
  if (gRFAL.state == RFAL_STATE_IDLE) {
    return ERR_WRONG_STATE;
  }

  /* Check allowed bit rate value */
  if ((txBR == RFAL_BR_KEEP) || (rxBR == RFAL_BR_KEEP)) {
    return ERR_PARAM;
  }

  /*******************************************************************************/
  /* Ensure that no previous operation is still ongoing */
  if (rfalChipIsBusy()) {
    return ERR_REQUEST;
  }


  switch (mode) {
    /*******************************************************************************/
    case RFAL_MODE_POLL_NFCA:
    case RFAL_MODE_POLL_NFCA_T1T:
      gRFAL.protocol = ST25R95_PROTOCOL_ISO14443A;
      gRFAL.NfcaSplitFrame = false;
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX));
      break;
    case RFAL_MODE_POLL_NFCB:
      gRFAL.protocol = ST25R95_PROTOCOL_ISO14443B;
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX));
      break;
    case RFAL_MODE_POLL_NFCF:
      gRFAL.protocol = ST25R95_PROTOCOL_ISO18092;
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX));
      break;
    case RFAL_MODE_POLL_NFCV:
      gRFAL.protocol = ST25R95_PROTOCOL_ISO15693;
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX));
      break;
    case RFAL_MODE_LISTEN_NFCA:
      return ERR_NOTSUPP;
      /*NOTREACHED*/
      break;
    /*******************************************************************************/
    case RFAL_MODE_POLL_B_PRIME:
    case RFAL_MODE_POLL_B_CTS:
    case RFAL_MODE_POLL_PICOPASS:
    case RFAL_MODE_POLL_ACTIVE_P2P:
    case RFAL_MODE_LISTEN_ACTIVE_P2P:
    case RFAL_MODE_LISTEN_NFCB:
    case RFAL_MODE_LISTEN_NFCF:
      return ERR_NOTSUPP;
      /*NOTREACHED*/
      break;

    /*******************************************************************************/
    default:
      return ERR_NOT_IMPLEMENTED;
  }

  /* Set state as STATE_MODE_SET only if not initialized yet (PSL) */
  gRFAL.state = ((gRFAL.state < RFAL_STATE_MODE_SET) ? RFAL_STATE_MODE_SET : gRFAL.state);
  gRFAL.mode  = mode;

  /* Apply the given bit rate and mode */
  return (rfalSetBitRate(txBR, rxBR));
}

/*******************************************************************************/
rfalMode RfalRfST25R95Class::rfalGetMode(void)
{
  return gRFAL.mode;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalSetBitRate(rfalBitRate txBR, rfalBitRate rxBR)
{
  ReturnCode retCode = ERR_NONE;

  /* Check if RFAL is not initialized */
  if (gRFAL.state == RFAL_STATE_IDLE) {
    return ERR_WRONG_STATE;
  }

  /*******************************************************************************/
  /* Ensure that no previous operation is still ongoing */
  if (rfalChipIsBusy()) {
    return ERR_REQUEST;
  }


  /* Store the new Bit Rates */
  gRFAL.txBR = ((txBR == RFAL_BR_KEEP) ? gRFAL.txBR : txBR);
  gRFAL.rxBR = ((rxBR == RFAL_BR_KEEP) ? gRFAL.rxBR : rxBR);

  retCode = st25r95SetBitRate(gRFAL.protocol, txBR, rxBR);
  if ((retCode == ERR_NONE) && (gRFAL.protocol != ST25R95_PROTOCOL_FIELDOFF)) {
    /* If field on, update bitrate value through ProtocolSelect */
    retCode = st25r95ProtocolSelect(gRFAL.protocol);
  }

  return (retCode);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalGetBitRate(rfalBitRate *txBR, rfalBitRate *rxBR)
{
  if ((gRFAL.state == RFAL_STATE_IDLE) || (gRFAL.mode == RFAL_MODE_NONE)) {
    return ERR_WRONG_STATE;
  }

  if (txBR != NULL) {
    *txBR = gRFAL.txBR;
  }

  if (rxBR != NULL) {
    *rxBR = gRFAL.rxBR;
  }

  return ERR_NONE;
}

/*******************************************************************************/
void RfalRfST25R95Class::rfalSetErrorHandling(rfalEHandling eHandling)
{
  NO_WARNING(eHandling);
  return;
}

/*******************************************************************************/
rfalEHandling RfalRfST25R95Class::rfalGetErrorHandling(void)
{
  return RFAL_ERRORHANDLING_NONE;
}

/*******************************************************************************/
void RfalRfST25R95Class::rfalSetFDTPoll(uint32_t FDTPoll)
{
  gRFAL.timings.FDTPoll = MIN(FDTPoll, RFAL_ST25R95_GPT_MAX_1FC);
}

/*******************************************************************************/
uint32_t RfalRfST25R95Class::rfalGetFDTPoll(void)
{
  return gRFAL.timings.FDTPoll;
}

/*******************************************************************************/
void RfalRfST25R95Class::rfalSetFDTListen(uint32_t FDTListen)
{
  gRFAL.timings.FDTListen = MIN(FDTListen, RFAL_ST25R95_MRT_MAX_1FC);
}

/*******************************************************************************/
uint32_t RfalRfST25R95Class::rfalGetFDTListen(void)
{
  return gRFAL.timings.FDTListen;
}

/*******************************************************************************/
void RfalRfST25R95Class::rfalSetGT(uint32_t GT)
{
  gRFAL.timings.GT = MIN(GT, RFAL_ST25R95_GT_MAX_1FC);
}

/*******************************************************************************/
uint32_t RfalRfST25R95Class::rfalGetGT(void)
{
  return gRFAL.timings.GT;
}

/*******************************************************************************/
bool RfalRfST25R95Class::rfalIsGTExpired(void)
{
  if (gRFAL.tmr.GT != RFAL_TIMING_NONE) {
    if (!rfalTimerisExpired(gRFAL.tmr.GT)) {
      return false;
    }
  }
  return true;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalFieldOnAndStartGT(void)
{
  ReturnCode ret;

  /* Check if RFAL has been initialized  */
  if ((gRFAL.state < RFAL_STATE_INIT)) {
    return ERR_WRONG_STATE;
  }

  /*******************************************************************************/
  /* Ensure that no previous operation is still ongoing */
  if (rfalChipIsBusy()) {
    return ERR_REQUEST;
  }


  ret = ERR_NONE;

  /*******************************************************************************/
  /* Turn field On if not already On */
  if (!gRFAL.field) {
    ret = st25r95FieldOn(gRFAL.protocol);
    gRFAL.field = true;
  }

  /*******************************************************************************/
  /* Start GT timer in case the GT value is set */
  if ((gRFAL.timings.GT != RFAL_TIMING_NONE)) {
    /* Ensure that a SW timer doesn't have a lower value then the minimum  */
    rfalTimerStart(gRFAL.tmr.GT, rfalConv1fcToMs(MAX((gRFAL.timings.GT), RFAL_ST25R95_GT_MIN_1FC)));
  }

  return ret;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalFieldOff(void)
{
  /* Ensure that no previous operation is still ongoing */
  if (rfalChipIsBusy()) {
    return ERR_REQUEST;
  }

  rfalWakeUpModeStop();
  gRFAL.field = false;
  gRFAL.protocol = ST25R95_PROTOCOL_FIELDOFF;
  return (st25r95FieldOff());
}



/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalStartTransceive(const rfalTransceiveContext *ctx)
{
  /* Ensure that RFAL is already Initialized and the mode has been set */
  if ((gRFAL.state >= RFAL_STATE_MODE_SET)) {
    /*******************************************************************************/
    /* Ensure that no previous operation is still ongoing */
    if (rfalChipIsBusy()) {
      return ERR_REQUEST;
    }

    gRFAL.TxRx.ctx = *ctx;

    /*******************************************************************************/
    if (rfalIsModePassiveComm(gRFAL.mode)) { /* Passive Comms */
      if ((gRFAL.TxRx.ctx.fwt != RFAL_FWT_NONE) && (gRFAL.TxRx.ctx.fwt != 0)) {
        st25r95SetFWT(gRFAL.protocol, gRFAL.TxRx.ctx.fwt);
      } else {
        /* Since ST25R95 does not support, use max FWT available */
        st25r95SetFWT(gRFAL.protocol, ST25R95_FWT_MAX);
      }
    }

    gRFAL.state       = RFAL_STATE_TXRX;
    gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_IDLE;
    gRFAL.TxRx.status = ERR_BUSY;
    gRFAL.Lm.dataFlag = false;

    /*******************************************************************************/
    if ((RFAL_MODE_POLL_NFCV == gRFAL.mode) || (RFAL_MODE_POLL_PICOPASS == gRFAL.mode)) {
      /* In NFCV a TxRx with a valid txBuf and txBufSize==0 indicates to send an EOF */
      /* Skip logic below that would go directly into receive                        */
      if (gRFAL.TxRx.ctx.txBuf != NULL) {
        return  ERR_NONE;
      }
    }


    /*******************************************************************************/
    /* Check if the Transceive start performing Tx or goes directly to Rx          */
    if ((gRFAL.TxRx.ctx.txBuf == NULL) || (gRFAL.TxRx.ctx.txBufLen == 0)) {
      return ERR_NOT_IMPLEMENTED;
    }

    return ERR_NONE;
  }

  return ERR_WRONG_STATE;
}


/*******************************************************************************/
bool RfalRfST25R95Class::rfalIsTransceiveInTx(void)
{
  return ((gRFAL.TxRx.state >= RFAL_TXRX_STATE_TX_IDLE) && (gRFAL.TxRx.state < RFAL_TXRX_STATE_RX_IDLE));
}


/*******************************************************************************/
bool RfalRfST25R95Class::rfalIsTransceiveInRx(void)
{
  return (gRFAL.TxRx.state >= RFAL_TXRX_STATE_RX_IDLE);
}


/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalTransceiveBlockingTx(uint8_t *txBuf, uint16_t txBufLen, uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *actLen, uint32_t flags, uint32_t fwt)
{
  ReturnCode               ret;
  rfalTransceiveContext    ctx;

  rfalCreateByteFlagsTxRxContext(ctx, txBuf, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt);
  EXIT_ON_ERR(ret, rfalStartTransceive(&ctx));


  return rfalTransceiveRunBlockingTx();
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalTransceiveBlockingRx(void)
{
  ReturnCode ret;

  do {
    rfalWorker();
  } while (((ret = rfalGetTransceiveStatus()) == ERR_BUSY) && rfalIsTransceiveInRx());

  return ret;
}


/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalTransceiveBlockingTxRx(uint8_t *txBuf, uint16_t txBufLen, uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *actLen, uint32_t flags, uint32_t fwt)
{
  ReturnCode ret;

  EXIT_ON_ERR(ret, rfalTransceiveBlockingTx(txBuf, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt));
  ret = rfalTransceiveBlockingRx();

  /* Convert received bits to bytes */
  if (actLen != NULL) {
    *actLen = rfalConvBitsToBytes(*actLen);
  }

  return ret;
}


/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalRunTransceiveWorker(void)
{
  if (gRFAL.state == RFAL_STATE_TXRX) {
    /* Run Tx or Rx state machines */
    if (rfalIsTransceiveInTx()) {
      rfalTransceiveTx();
      return rfalGetTransceiveStatus();
    } else if (rfalIsTransceiveInRx()) {
      rfalTransceiveRx();
      return rfalGetTransceiveStatus();
    }
  }
  return ERR_WRONG_STATE;
}

/*******************************************************************************/
rfalTransceiveState RfalRfST25R95Class::rfalGetTransceiveState(void)
{
  return gRFAL.TxRx.state;
}

ReturnCode RfalRfST25R95Class::rfalGetTransceiveStatus(void)
{
  return ((gRFAL.TxRx.state == RFAL_TXRX_STATE_IDLE) ? gRFAL.TxRx.status : ERR_BUSY);
}


/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalGetTransceiveRSSI(uint16_t *rssi)
{
  NO_WARNING(rssi);

  return ERR_NOTSUPP;
}


/*******************************************************************************/
void RfalRfST25R95Class::rfalWorker(void)
{
  switch (gRFAL.state) {
    case RFAL_STATE_TXRX:
      rfalRunTransceiveWorker();
      break;
    case RFAL_STATE_WUM:
      rfalRunWakeUpModeWorker();
      break;
    /* Nothing to be done */
    default:
      break;
  }
}

/*******************************************************************************/
void RfalRfST25R95Class::rfalTransceiveTx(void)
{
  uint8_t transmitFlag = 0;

  if (gRFAL.TxRx.state != gRFAL.TxRx.lastState) {
    /* rfalLogD("%s: lastSt: %d curSt: %d \r\n", __FUNCTION__, gRFAL.TxRx.lastState, gRFAL.TxRx.state);*/
    gRFAL.TxRx.lastState = gRFAL.TxRx.state;
  }

  switch (gRFAL.TxRx.state) {
    /*******************************************************************************/
    case RFAL_TXRX_STATE_TX_IDLE:
      /* Nothing to do */
      gRFAL.TxRx.state = RFAL_TXRX_STATE_TX_WAIT_GT ;
    /* fall through */

    /*******************************************************************************/
    case RFAL_TXRX_STATE_TX_WAIT_GT:
      /* Wait for GT and FDT Poll */

      if (!rfalIsGTExpired() || !rfalTimerisExpired(gRFAL.tmr.FDTPoll)) {
        break;
      }
      gRFAL.tmr.GT = RFAL_TIMING_NONE;
      gRFAL.TxRx.state = RFAL_TXRX_STATE_TX_TRANSMIT;
    /* fall through */

    /*******************************************************************************/
    case RFAL_TXRX_STATE_TX_TRANSMIT:
      /*******************************************************************************/
      /* Execute Pre Transceive Callback                                             */
      /*******************************************************************************/
      if (gRFAL.callbacks.preTxRx != NULL) {
        gRFAL.callbacks.preTxRx();
      }
      /*******************************************************************************/
      /* Prepare Rx                                                                  */
      /*******************************************************************************/
      st25r95SPIPrepareRx(
        gRFAL.protocol,
        gRFAL.TxRx.ctx.rxBuf,
        rfalConvBitsToBytes(gRFAL.TxRx.ctx.rxBufLen),
        gRFAL.TxRx.ctx.rxRcvdLen,
        gRFAL.TxRx.ctx.flags,
        gRFAL.RxInformationBytes
      );
      /*******************************************************************************/
      /* Send the data                                                               */
      /*******************************************************************************/
      st25r95SPISendData(gRFAL.TxRx.ctx.txBuf, rfalConvBitsToBytes(gRFAL.TxRx.ctx.txBufLen), gRFAL.protocol, gRFAL.TxRx.ctx.flags);

      /* Start FDTPoll SW timer */
      rfalTimerStart(gRFAL.tmr.FDTPoll, (RFAL_ST25R95_SW_TMR_MIN_1MS + rfalConv1fcToMs(gRFAL.timings.FDTPoll)));

      gRFAL.TxRx.state = RFAL_TXRX_STATE_TX_WAIT_TXE;
    /* fall through */

    /*******************************************************************************/
    case RFAL_TXRX_STATE_TX_WAIT_TXE:
      if (!st25r95SPIIsTransmitCompleted()) {
        break;
      }
      transmitFlag = gRFAL.TxRx.ctx.txBufLen % 8;
      if (transmitFlag == 0) {
        transmitFlag = 0x8;
      }
      if (!(gRFAL.TxRx.ctx.flags & RFAL_TXRX_FLAGS_CRC_TX_MANUAL)) {
        transmitFlag |= RFAL_ST25R95_ISO14443A_APPENDCRC;
      }
      if (gRFAL.NfcaSplitFrame) {
        transmitFlag |= RFAL_ST25R95_ISO14443A_SPLITFRAME;
      }
      if (gRFAL.mode == RFAL_MODE_POLL_NFCA_T1T) {
        transmitFlag |= RFAL_ST25R95_ISO14443A_TOPAZFORMAT;
      }
      st25r95SPISendTransmitFlag(gRFAL.protocol, transmitFlag);
      gRFAL.TxRx.state = RFAL_TXRX_STATE_TX_DONE;
    /* fall through */

    /*******************************************************************************/
    case RFAL_TXRX_STATE_TX_DONE:
      /* If no rxBuf is provided do not wait/expect Rx */
      if (gRFAL.TxRx.ctx.rxBuf == NULL) {
        /* Clean up Transceive */
        //rfalCleanupTransceive();
        gRFAL.TxRx.status = ERR_NONE;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_IDLE;
        break;
      }
      /* Goto Rx */
      gRFAL.TxRx.state  =  RFAL_TXRX_STATE_RX_IDLE;
      break;

    /*******************************************************************************/
    case RFAL_TXRX_STATE_TX_FAIL:
      /* Error should be assigned by previous state */
      if (gRFAL.TxRx.status == ERR_BUSY) {
        gRFAL.TxRx.status = ERR_SYSTEM;
      }
      gRFAL.TxRx.state = RFAL_TXRX_STATE_IDLE;
      break;

    /*******************************************************************************/
    default:
      gRFAL.TxRx.status = ERR_SYSTEM;
      gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
      break;
  }
}

/*******************************************************************************/
void RfalRfST25R95Class::rfalTransceiveRx(void)
{
  ReturnCode retCode;

  if (gRFAL.TxRx.state != gRFAL.TxRx.lastState) {
    /*rfalLogD("%s: lastSt: %d curSt: %d \r\n", __FUNCTION__, gRFAL.TxRx.lastState, gRFAL.TxRx.state);*/
    gRFAL.TxRx.lastState = gRFAL.TxRx.state;
  }

  switch (gRFAL.TxRx.state) {
    /*******************************************************************************/
    case RFAL_TXRX_STATE_RX_IDLE:

      /* Clear rx counters */
      if (gRFAL.TxRx.ctx.rxRcvdLen)  {
        *gRFAL.TxRx.ctx.rxRcvdLen = 0;
      }

      gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_WAIT_RXE;
    /* fall through */

    /*******************************************************************************/
    case RFAL_TXRX_STATE_RX_WAIT_RXE:
      if (st25r95SPIPollRead(ST25R95_CONTROL_POLL_NO_TIMEOUT) == ERR_TIMEOUT) {
        break;
      }
      gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_READ_DATA;
    /* fall through */

    /*******************************************************************************/
    case RFAL_TXRX_STATE_RX_READ_DATA:
      retCode = st25r95SPICompleteRx();
      /* Re-Start FDTPoll SW timer */
      rfalTimerStart(gRFAL.tmr.FDTPoll, (RFAL_ST25R95_SW_TMR_MIN_1MS + rfalConv1fcToMs(gRFAL.timings.FDTPoll)));

      if (gRFAL.TxRx.ctx.rxRcvdLen != NULL) {
        (*gRFAL.TxRx.ctx.rxRcvdLen) = rfalConvBytesToBits(*gRFAL.TxRx.ctx.rxRcvdLen);

        /*******************************************************************************/
        /* In case of Incomplete byte append the residual bits                         */
        /*******************************************************************************/
        if ((retCode >= ERR_INCOMPLETE_BYTE_01) && (retCode <= ERR_INCOMPLETE_BYTE_07)) {
          (*gRFAL.TxRx.ctx.rxRcvdLen) += (retCode - ERR_INCOMPLETE_BYTE);

          if ((*gRFAL.TxRx.ctx.rxRcvdLen) > 0) {
            (*gRFAL.TxRx.ctx.rxRcvdLen) -= RFAL_BITS_IN_BYTE;
          }

          retCode = ERR_INCOMPLETE_BYTE;
        }
      }


      /*******************************************************************************/
      /* Execute Post Transceive Callback                                            */
      /*******************************************************************************/
      if (gRFAL.callbacks.postTxRx != NULL) {
        gRFAL.callbacks.postTxRx();
      }

      if (retCode != ERR_NONE) {
        gRFAL.TxRx.status = retCode;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
        break;
      }
      gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_DONE;
    /* fall through */

    /*******************************************************************************/
    case RFAL_TXRX_STATE_RX_DONE:
      gRFAL.TxRx.status = ERR_NONE;
      gRFAL.TxRx.state  = RFAL_TXRX_STATE_IDLE;
      break;


    /*******************************************************************************/
    case RFAL_TXRX_STATE_RX_FAIL:
      gRFAL.TxRx.state = RFAL_TXRX_STATE_IDLE;
      break;

    /*******************************************************************************/
    default:
      gRFAL.TxRx.status = ERR_SYSTEM;
      gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
      break;
  }
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalTransceiveRunBlockingTx(void)
{
  ReturnCode ret;

  do {
    rfalWorker();
  } while (((ret = rfalGetTransceiveStatus()) == ERR_BUSY) && rfalIsTransceiveInTx());

  if (rfalIsTransceiveInRx()) {
    return ERR_NONE;
  }

  return ret;
}

/*******************************************************************************/
bool RfalRfST25R95Class::rfalChipIsBusy(void)
{
  /* ST25R95 cannot be interrupted while an operation is ongoing */

  /* Check whether a Transceive operation is still running */
  if ((gRFAL.state == RFAL_STATE_TXRX) && (gRFAL.TxRx.state > RFAL_TXRX_STATE_TX_IDLE)) {
    return (true);
  }

  return (false);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalISO14443ATransceiveShortFrame(rfal14443AShortFrameCmd txCmd, uint8_t *rxBuf, uint8_t rxBufLen, uint16_t *rxRcvdLen, uint32_t fwt)
{
  ReturnCode ret;
  rfalTransceiveContext ctx;
  uint8_t st95hShortFrameBuffer;
  /* Check if RFAL is properly initialized */
  if ((gRFAL.state < RFAL_STATE_MODE_SET) || ((gRFAL.mode != RFAL_MODE_POLL_NFCA) && (gRFAL.mode != RFAL_MODE_POLL_NFCA_T1T)) || !gRFAL.field) {
    return ERR_WRONG_STATE;
  }

  /* Check for valid parameters */
  if ((rxBuf == NULL) || (rxRcvdLen == NULL) || (fwt == RFAL_FWT_NONE)) {
    return ERR_PARAM;
  }

  gRFAL.NfcaSplitFrame = false;
  /*******************************************************************************/
  /* Update the short frame buffer with the REQA or WUPA command                 */
  st95hShortFrameBuffer =  txCmd;


  ctx.flags     = (RFAL_TXRX_FLAGS_CRC_TX_MANUAL | RFAL_TXRX_FLAGS_CRC_RX_KEEP);
  ctx.txBuf     = &st95hShortFrameBuffer;
  ctx.txBufLen  = 7;
  ctx.rxBuf     = rxBuf;
  ctx.rxBufLen  = rxBufLen;
  ctx.rxRcvdLen = rxRcvdLen;
  ctx.fwt       = fwt;

  rfalStartTransceive(&ctx);

  /*******************************************************************************/
  /* Run Transceive blocking */
  ret = rfalTransceiveRunBlockingTx();
  if (ret == ERR_NONE) {
    ret = rfalTransceiveBlockingRx();
  }

  /* ST25R95 has no means to disable CRC check, discard CRC errors */
  if (ret == ERR_CRC) {
    ret = ERR_NONE;
  }

  return ret;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalISO14443ATransceiveAnticollisionFrame(uint8_t *buf, uint8_t *bytesToSend, uint8_t *bitsToSend, uint16_t *rxLength, uint32_t fwt)
{
  ReturnCode            ret;
  rfalTransceiveContext ctx;
  uint8_t               collByte;

  /* Check if RFAL is properly initialized */
  if ((gRFAL.state < RFAL_STATE_MODE_SET) || (gRFAL.mode != RFAL_MODE_POLL_NFCA)) {
    return ERR_WRONG_STATE;
  }

  /* Check for valid parameters */
  if ((buf == NULL) || (bytesToSend == NULL) || (bitsToSend == NULL) || (rxLength == NULL)) {
    return ERR_PARAM;
  }

  gRFAL.NfcaSplitFrame = true;

  /*******************************************************************************/
  /* Prepare for Transceive                                                      */
  ctx.flags     = (RFAL_TXRX_FLAGS_CRC_TX_MANUAL | RFAL_TXRX_FLAGS_CRC_RX_KEEP);
  ctx.txBuf     = buf;
  ctx.txBufLen  = (rfalConvBytesToBits(*bytesToSend) + *bitsToSend);
  ctx.rxBuf     = (buf + (*bytesToSend));
  ctx.rxBufLen  = rfalConvBytesToBits(RFAL_ISO14443A_SDD_RES_LEN);
  ctx.rxRcvdLen = rxLength;
  ctx.fwt       = fwt;

  EXIT_ON_ERR(ret, rfalStartTransceive(&ctx));

  /*******************************************************************************/
  collByte = 0;

  /* save the collision byte */
  if ((*bitsToSend) > 0) {
    buf[(*bytesToSend)] <<= (RFAL_BITS_IN_BYTE - (*bitsToSend));
    buf[(*bytesToSend)] >>= (RFAL_BITS_IN_BYTE - (*bitsToSend));
    collByte = buf[(*bytesToSend)];
  }

  /*******************************************************************************/
  /* Run Transceive blocking */
  ret = rfalTransceiveRunBlockingTx();
  if (ret == ERR_NONE) {
    ret = rfalTransceiveBlockingRx();
    /* ignore CRC error */
    if (ret == ERR_CRC) {
      ret = ERR_NONE;
    }

    /*******************************************************************************/
    if ((*bitsToSend) > 0) {
      buf[(*bytesToSend)] >>= (*bitsToSend);
      buf[(*bytesToSend)] <<= (*bitsToSend);
      buf[(*bytesToSend)] |= collByte;
    }

    if ((ERR_RF_COLLISION == ret)) {
      (*rxLength) = rfalConvBytesToBits(gRFAL.RxInformationBytes[1]);
      (*bytesToSend) = (gRFAL.RxInformationBytes[1] + (*bytesToSend)) & 0xF;
      (*bitsToSend)  = gRFAL.RxInformationBytes[2] & 0x7;
    }
  }
  gRFAL.NfcaSplitFrame = false;
  return ret;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalISO15693TransceiveAnticollisionFrame(uint8_t *txBuf, uint8_t txBufLen, uint8_t *rxBuf, uint8_t rxBufLen, uint16_t *actLen)
{
  ReturnCode            ret;
  rfalTransceiveContext ctx;

  /* Check if RFAL is properly initialized */
  if ((gRFAL.state < RFAL_STATE_MODE_SET) || (gRFAL.mode != RFAL_MODE_POLL_NFCV)) {
    return ERR_WRONG_STATE;
  }

  /*******************************************************************************/
  /* Prepare for Transceive  */
  ctx.flags     = (((txBufLen == 0) ? RFAL_TXRX_FLAGS_CRC_TX_MANUAL : RFAL_TXRX_FLAGS_CRC_TX_AUTO) | RFAL_TXRX_FLAGS_CRC_RX_KEEP | RFAL_TXRX_FLAGS_AGC_OFF | ((txBufLen == 0) ? RFAL_TXRX_FLAGS_NFCV_FLAG_MANUAL : RFAL_TXRX_FLAGS_NFCV_FLAG_AUTO)); /* Disable Automatic Gain Control (AGC) for better detection of collision */
  ctx.txBuf     = txBuf;
  ctx.txBufLen  = rfalConvBytesToBits(txBufLen);
  ctx.rxBuf     = rxBuf;
  ctx.rxBufLen  = rfalConvBytesToBits(rxBufLen);
  ctx.rxRcvdLen = actLen;
  ctx.fwt       = RFAL_FWT_NONE;

  EXIT_ON_ERR(ret, rfalStartTransceive(&ctx));

  /*******************************************************************************/
  /* Run Transceive blocking */
  ret = rfalTransceiveRunBlockingTx();
  if (ret == ERR_NONE) {
    ret = rfalTransceiveBlockingRx();
  }

  return ret;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalISO15693TransceiveEOFAnticollision(uint8_t *rxBuf, uint8_t rxBufLen, uint16_t *actLen)
{
  uint8_t dummy;

  return rfalISO15693TransceiveAnticollisionFrame(&dummy, 0, rxBuf, rxBufLen, actLen);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalISO15693TransceiveEOF(uint8_t *rxBuf, uint8_t rxBufLen, uint16_t *actLen)
{
  ReturnCode ret;
  uint8_t    dummy;

  /* Check if RFAL is properly initialized */
  if ((gRFAL.state < RFAL_STATE_MODE_SET) || (gRFAL.mode != RFAL_MODE_POLL_NFCV)) {
    return ERR_WRONG_STATE;
  }

  /*******************************************************************************/
  /* Run Transceive blocking */
  ret = rfalTransceiveBlockingTxRx(&dummy,
                                   0,
                                   rxBuf,
                                   rxBufLen,
                                   actLen,
                                   ((uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL | (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP | (uint32_t)RFAL_TXRX_FLAGS_AGC_ON),
                                   rfalConv64fcTo1fc(RFAL_FWT_NONE));
  return ret;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalFeliCaPoll(rfalFeliCaPollSlots slots, uint16_t sysCode, uint8_t reqCode, rfalFeliCaPollRes *pollResList, uint8_t pollResListSize, uint8_t *devicesDetected, uint8_t *collisionsDetected)
{
  ReturnCode        ret;
  uint8_t           frame[RFAL_FELICA_POLL_REQ_LEN - RFAL_FELICA_LEN_LEN];  // LEN is added by ST25R95 automatically
  uint16_t          actLen;
  uint8_t           frameIdx;
  uint8_t           devDetected;
  uint8_t           colDetected;

  /* Check if RFAL is properly initialized */
  if ((gRFAL.state < RFAL_STATE_MODE_SET) || (gRFAL.mode != RFAL_MODE_POLL_NFCF)) {
    return ERR_WRONG_STATE;
  }

  frameIdx    = 0;
  colDetected = 0;
  devDetected = 0;

  /*******************************************************************************/
  /* Compute SENSF_REQ frame */
  frame[frameIdx++] =  FELICA_CMD_POLLING;       /* CMD: SENF_REQ                       */
  frame[frameIdx++] = (uint8_t)(sysCode >> 8);   /* System Code (SC)                    */
  frame[frameIdx++] = (uint8_t)(sysCode & 0xFF); /* System Code (SC)                    */
  frame[frameIdx++] = reqCode;                   /* Communication Parameter Request (RC)*/
  frame[frameIdx++] = (uint8_t)slots;            /* TimeSlot (TSN)                      */

  st25r95SetSlotCounter((uint8_t)slots);
  /*******************************************************************************/
  /* NRT should not stop on reception - Use EMVCo mode to run NRT in nrt_emv     *
   * ERRORHANDLING_EMVCO has no special handling for NFC-F mode                  */
  rfalSetErrorHandling(RFAL_ERRORHANDLING_EMVCO);

  /*******************************************************************************/
  /* Run transceive blocking,
   * Calculate Total Response Time in(64/fc):
   *                       512 PICC process time + (n * 256 Time Slot duration)  */
  ret = rfalTransceiveBlockingTx(
          frame,
          frameIdx,
          ((uint8_t *)gRFAL.nfcfData.pollResponses),
          RFAL_FELICA_POLL_RES_LEN,
          &actLen,
          RFAL_TXRX_FLAGS_CRC_RX_REMV,
          rfalConv64fcTo1fc(RFAL_FELICA_POLL_DELAY_TIME + (RFAL_FELICA_POLL_SLOT_TIME * ((uint32_t)slots + 1U))));

  /*******************************************************************************/
  /* If Tx OK, Wait for first response                                           */
  if (ret == ERR_NONE) {
    ret = rfalTransceiveBlockingRx();
    if (ret != ERR_TIMEOUT) {
      /* If the reception was OK, new device found */
      if (ret == ERR_NONE) {
        devDetected++;
      }
      /* If the reception was not OK, mark as collision */
      else {
        colDetected++;
      }
    }
  }
  st25r95SetSlotCounter((uint8_t)RFAL_FELICA_1_SLOT);
  /*******************************************************************************/
  /* Restore NRT to normal mode - back to previous error handling */


  /*******************************************************************************/
  /* Assign output parameters if requested                                       */

  if ((pollResList != NULL) && (pollResListSize > 0) && (devDetected > 0)) {
    ST_MEMCPY(pollResList, gRFAL.nfcfData.pollResponses, (RFAL_FELICA_POLL_RES_LEN * MIN(pollResListSize, devDetected)));
  }

  if (devicesDetected != NULL) {
    *devicesDetected = devDetected;
  }

  if (collisionsDetected != NULL) {
    *collisionsDetected = colDetected;
  }

  return ((colDetected || devDetected) ? ERR_NONE : ret);
}

/*****************************************************************************
 *  Listen Mode                                                              *
 *****************************************************************************/

/*******************************************************************************/
bool RfalRfST25R95Class::rfalIsExtFieldOn(void)
{
  return (false);
}


/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalListenStart(uint32_t lmMask, const rfalLmConfPA *confA, const rfalLmConfPB *confB, const rfalLmConfPF *confF, uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *rxLen)
{
  (void)lmMask;
  (void)confA;
  (void)confB;
  (void)confF;
  (void)rxBuf;
  (void)rxBufLen;
  (void)rxLen;
  return ERR_NOTSUPP;
}


/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalListenSleepStart(rfalLmState sleepSt, uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *rxLen)
{
  (void)sleepSt;
  (void)rxBuf;
  (void)rxBufLen;
  (void)rxLen;
  return ERR_NOTSUPP;
}


/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalListenStop(void)
{
  return ERR_NOTSUPP;
}


/*******************************************************************************/
rfalLmState RfalRfST25R95Class::rfalListenGetState(bool *dataFlag, rfalBitRate *lastBR)
{
  (void)dataFlag;
  (void)lastBR;
  return RFAL_LM_STATE_NOT_INIT;
}


/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalListenSetState(rfalLmState newSt)
{
  (void)newSt;
  return ERR_NOTSUPP;
}


/*******************************************************************************
 *  Wake-Up Mode                                                               *
 *******************************************************************************/

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalWakeUpModeStart(const rfalWakeUpConfig *config)
{
  /* Check if RFAL is not initialized */
  if (gRFAL.state == RFAL_STATE_IDLE) {
    return ERR_WRONG_STATE;
  }

  if (config == NULL) {
    gRFAL.wum.cfg.period           = RFAL_WUM_PERIOD_300MS;
    gRFAL.wum.cfg.irqTout          = false;
    gRFAL.wum.cfg.swTagDetect      = false;

    gRFAL.wum.cfg.indAmp.enabled   = true;
    gRFAL.wum.cfg.indPha.enabled   = false;
    gRFAL.wum.cfg.cap.enabled      = false;
    gRFAL.wum.cfg.indAmp.delta     = 8U;
    gRFAL.wum.cfg.indAmp.reference = RFAL_WUM_REFERENCE_AUTO;
  } else {
    gRFAL.wum.cfg = *config;
  }

  /* Check for valid configuration */
  if (gRFAL.wum.cfg.cap.enabled || gRFAL.wum.cfg.indPha.enabled  || gRFAL.wum.cfg.swTagDetect || !gRFAL.wum.cfg.indAmp.enabled) {
    return ERR_PARAM;
  }

  if (gRFAL.wum.cfg.indAmp.reference == RFAL_WUM_REFERENCE_AUTO) {
    gRFAL.wum.cfg.indAmp.reference = gRFAL.wum.CalTagDet;
  }
  if ((gRFAL.wum.cfg.indAmp.delta > gRFAL.wum.cfg.indAmp.reference) || ((((uint32_t)gRFAL.wum.cfg.indAmp.delta) + ((uint32_t)gRFAL.wum.cfg.indAmp.reference)) > 0xFCUL)) {
    return ERR_PARAM;
  }

  /* Use a fixed period of ~300 ms */
  st25r95SPIIdle(gRFAL.wum.cfg.indAmp.reference - gRFAL.wum.cfg.indAmp.delta, gRFAL.wum.cfg.indAmp.reference + gRFAL.wum.cfg.indAmp.delta, RFAL_ST25R95_IDLE_DEFAULT_WUPERIOD);
  gRFAL.state     = RFAL_STATE_WUM;
  gRFAL.wum.state = RFAL_WUM_STATE_ENABLED;
  return ERR_NONE;
}

/*******************************************************************************/
bool RfalRfST25R95Class::rfalWakeUpModeHasWoke(void)
{
  return (gRFAL.wum.state >= RFAL_WUM_STATE_ENABLED_WOKE);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalWakeUpModeStop(void)
{
  /* Ensure that no previous operation is still ongoing */
  if (rfalChipIsBusy()) {
    return ERR_REQUEST;
  }


  if (gRFAL.wum.state == RFAL_WUM_STATE_NOT_INIT) {
    return ERR_WRONG_STATE;
  }

  gRFAL.wum.state = RFAL_WUM_STATE_NOT_INIT;
  st25r95SPIKillIdle();
  st25r95SPICommandEcho();
  return ERR_NONE;
}


/*******************************************************************************/
void RfalRfST25R95Class::rfalRunWakeUpModeWorker(void)
{
  if (gRFAL.state != RFAL_STATE_WUM) {
    return;
  }

  switch (gRFAL.wum.state) {
    case RFAL_WUM_STATE_ENABLED:
    case RFAL_WUM_STATE_ENABLED_WOKE:
      if (st25r95SPIPollRead(ST25R95_CONTROL_POLL_NO_TIMEOUT) != ERR_TIMEOUT) {
        st25r95SPIGetIdleResponse();
        gRFAL.wum.state = RFAL_WUM_STATE_ENABLED_WOKE;
      }

    default:
      break;
  }
}


/*******************************************************************************
 *  RF Chip                                                                    *
 *******************************************************************************/

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalChipWriteReg(uint16_t reg, const uint8_t *values, uint8_t len)
{
  ReturnCode retCode;

  /* Ensure that no previous operation is still ongoing */
  if (rfalChipIsBusy()) {
    return ERR_REQUEST;
  }


  if (len != 1) {
    retCode = ERR_PARAM;
  } else {
    retCode = st25r95WriteReg(gRFAL.protocol, reg, values[0]);
  }
  return (retCode);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalChipReadReg(uint16_t reg, uint8_t *values, uint8_t len)
{
  ReturnCode retCode;

  /* Ensure that no previous operation is still ongoing */
  if (rfalChipIsBusy()) {
    return ERR_REQUEST;
  }


  if (len != 1) {
    retCode = ERR_PARAM;
  } else {
    retCode = st25r95ReadReg(reg, values);
  }
  return (retCode);
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalChipExecCmd(uint16_t cmd)
{
  NO_WARNING(cmd);

  return ERR_NOTSUPP;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalChipWriteTestReg(uint16_t reg, uint8_t value)
{
  NO_WARNING(reg);
  NO_WARNING(value);

  return ERR_NOTSUPP;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalChipReadTestReg(uint16_t reg, uint8_t *value)
{
  NO_WARNING(reg);
  NO_WARNING(value);

  return ERR_NOTSUPP;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalChipChangeRegBits(uint16_t reg, uint8_t valueMask, uint8_t value)
{
  ReturnCode retCode;
  uint8_t tmp;

  retCode = st25r95ReadReg(reg, &tmp);

  if (retCode == ERR_NONE) {
    /* mask out the bits we don't want to change */
    tmp &= (uint8_t)(~((uint32_t)valueMask));
    /* set the new value */
    tmp |= (value & valueMask);
    retCode = st25r95WriteReg(gRFAL.protocol, reg, tmp);
  }

  return retCode;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalChipChangeTestRegBits(uint16_t reg, uint8_t valueMask, uint8_t value)
{
  NO_WARNING(reg);
  NO_WARNING(valueMask);
  NO_WARNING(value);

  return ERR_NOTSUPP;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalChipSetRFO(uint8_t rfo)
{
  NO_WARNING(rfo);

  return ERR_NOTSUPP;
}


/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalChipGetRFO(uint8_t *result)
{
  NO_WARNING(result);

  return ERR_NOTSUPP;
}


/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalChipMeasureAmplitude(uint8_t *result)
{
  NO_WARNING(result);

  return ERR_NOTSUPP;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalChipMeasurePhase(uint8_t *result)
{
  NO_WARNING(result);

  return ERR_NOTSUPP;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalChipMeasureCapacitance(uint8_t *result)
{
  NO_WARNING(result);

  return ERR_NOTSUPP;
}

/*******************************************************************************/
ReturnCode RfalRfST25R95Class::rfalChipMeasurePowerSupply(uint8_t param, uint8_t *result)
{
  NO_WARNING(param);
  NO_WARNING(result);

  return ERR_NOTSUPP;
}
