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
#ifndef RFAL_RFST25R95_H
#define RFAL_RFST25R95_H


/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "SPI.h"
#include "rfal_rf.h"
#include "rfal_nfc.h"
#include "st_errno.h"
#include "nfc_utils.h"
#include "st25r95.h"
#include "st25r95_com.h"
#include "rfal_rfst25r95_analogConfig.h"
#include "rfal_rfst25r95_iso15693_2.h"

/*
 ******************************************************************************
 * ENABLE SWITCHES
 ******************************************************************************
 */

/*
******************************************************************************
* GLOBAL TYPES
******************************************************************************
*/

/*! Struct that holds all involved on a Transceive including the context passed by the caller     */
typedef struct {
  rfalTransceiveState     state;       /*!< Current transceive state                            */
  rfalTransceiveState     lastState;   /*!< Last transceive state (debug purposes)              */
  ReturnCode              status;      /*!< Current status/error of the transceive              */

  rfalTransceiveContext   ctx;         /*!< The transceive context given by the caller          */
} rfalTxRx;

/*! Struct that holds all context for the Listen Mode                                             */
typedef struct {
  uint8_t                *rxBuf;       /*!< Location to store incoming data in Listen Mode      */
  uint16_t                rxBufLen;    /*!< Length of rxBuf                                     */
  uint16_t               *rxLen;       /*!< Pointer to write the data length placed into rxBuf  */
  bool                    dataFlag;    /*!< Listen Mode current Data Flag                       */
} rfalLm;

/*! Struct that holds all context for the Wake-Up Mode                                             */
typedef struct {
  rfalWumState            state;       /*!< Current Wake-Up Mode state                           */
  rfalWakeUpConfig        cfg;         /*!< Current Wake-Up Mode context                         */
  uint8_t                 CalTagDet;   /*!< Tag Detection calibration value                      */
} rfalWum;

typedef struct {
  uint32_t                GT;          /*!< GT in 1/fc                  */
  uint32_t                FDTListen;   /*!< FDTListen in 1/fc           */
  uint32_t                FDTPoll;     /*!< FDTPoll in 1/fc             */
} rfalTimings;

/*! Struct that holds the software timers                                 */
typedef struct {
  uint32_t                GT;          /*!< RFAL's GT timer             */
  uint32_t                FDTPoll;     /*!< RFAL's FST Poll timer       */
} rfalTimers;

/*! Struct that holds the RFAL's callbacks                                */
typedef struct {
  rfalPreTxRxCallback     preTxRx;     /*!< RFAL's Pre TxRx callback    */
  rfalPostTxRxCallback    postTxRx;    /*!< RFAL's Post TxRx callback   */
} rfalCallbacks;

/*! Struct that holds NFC-F data - Used only inside rfalFelicaPoll() (static to avoid adding it into stack) */
typedef struct {
  rfalFeliCaPollRes pollResponses[RFAL_FELICA_POLL_MAX_SLOTS];   /* FeliCa Poll response container for 16 slots */
} rfalNfcfWorkingData;

typedef struct {
  rfalState             state;     /*!< RFAL's current state                            */
  rfalMode              mode;      /*!< RFAL's current mode                             */
  rfalBitRate           txBR;      /*!< RFAL's current Tx Bit Rate                      */
  rfalBitRate           rxBR;      /*!< RFAL's current Rx Bit Rate                      */
  bool                  field;     /*!< Current field state (On / Off)                  */

  rfalTimings           timings;   /*!< RFAL's timing setting                           */
  rfalTxRx              TxRx;      /*!< RFAL's transceive management                    */
  rfalLm                Lm;        /*!< RFAL's listen mode management                   */
  rfalWum               wum;       /*!< RFAL's Wake-Up mode management                  */

  rfalTimers            tmr;       /*!< RFAL's Software timers                          */
  rfalCallbacks         callbacks; /*!< RFAL's callbacks                                */

  uint8_t               protocol;  /*!< ProtocolSelect protocol                         */
  uint8_t               RxInformationBytes[3]; /*!< ST25R95 additional information bytes*/
  rfalNfcfWorkingData   nfcfData; /*!< RFAL's working data when supporting NFC-F        */
  bool                  NfcaSplitFrame;
} rfal;

/*! Felica's command set */
typedef enum {
  FELICA_CMD_POLLING                  = 0x00, /*!< Felica Poll/REQC command (aka SENSF_REQ) to identify a card    */
  FELICA_CMD_POLLING_RES              = 0x01, /*!< Felica Poll/REQC command (aka SENSF_RES) response              */
  FELICA_CMD_REQUEST_SERVICE          = 0x02, /*!< verify the existence of Area and Service                       */
  FELICA_CMD_REQUEST_RESPONSE         = 0x04, /*!< verify the existence of a card                                 */
  FELICA_CMD_READ_WITHOUT_ENCRYPTION  = 0x06, /*!< read Block Data from a Service that requires no authentication */
  FELICA_CMD_WRITE_WITHOUT_ENCRYPTION = 0x08, /*!< write Block Data to a Service that requires no authentication  */
  FELICA_CMD_REQUEST_SYSTEM_CODE      = 0x0c, /*!< acquire the System Code registered to a card                   */
  FELICA_CMD_AUTHENTICATION1          = 0x10, /*!< authenticate a card                                            */
  FELICA_CMD_AUTHENTICATION2          = 0x12, /*!< allow a card to authenticate a Reader/Writer                   */
  FELICA_CMD_READ                     = 0x14, /*!< read Block Data from a Service that requires authentication    */
  FELICA_CMD_WRITE                    = 0x16, /*!< write Block Data to a Service that requires authentication     */
} t_rfalFeliCaCmd;

/*! Struct for Analog Config Look Up Table Update */
typedef struct {
  const uint8_t *currentAnalogConfigTbl; /*!< Reference to start of current Analog Configuration      */
  uint16_t configTblSize;          /*!< Total size of Analog Configuration                      */
  bool    ready;                  /*!< Indicate if Look Up Table is complete and ready for use */
} rfalAnalogConfigMgmt;

/*! SPI transceive context definition */
typedef struct {
  bool rmvCRC;                         /*!< Remove CRC flag                 */
  bool inListen;                       /*!< inListen flags                  */
  bool NFCIP1;                         /*!< NFCIP1 flags                    */
  rfalLmState LmState;                 /*!< LmState                         */
  uint16_t rxBufLen;                   /*!< rxBufLen                        */
  uint16_t *rxRcvdLen;                 /*!< rxRcvdLen                       */
  uint8_t *rxBuf;                      /*!< rxBuf                           */
  uint8_t *additionalRespBytes;        /*!< additionalRespBytes             */
  ReturnCode retCode;                  /*!< retCode                         */
  uint8_t BufCRC[2];                   /*!< BufCRC                          */
  uint8_t NFCIP1_SoD[1];               /*!< NFCIP1_SoD                      */
  uint8_t protocol;                    /*!< protocol                        */

} st25r95SPIRxContext;

/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/

#define RFAL_ST25R95_GPT_MAX_1FC         rfalConv8fcTo1fc(0xFFFF)                     /*!< Max GPT steps in 1fc (0xFFFF steps of 8/fc    => 0xFFFF * 590ns  = 38,7ms)      */
#define RFAL_ST25R95_NRT_MAX_1FC         rfalConv4096fcTo1fc(0xFFFF)                  /*!< Max NRT steps in 1fc (0xFFFF steps of 4096/fc => 0xFFFF * 302us  = 19.8s)       */
#define RFAL_ST25R95_NRT_DISABLED        0                                            /*!< NRT Disabled: All 0 No-response timer is not started, wait forever              */
#define RFAL_ST25R95_MRT_MAX_1FC         rfalConv64fcTo1fc(0x00FF)                    /*!< Max MRT steps in 1fc (0x00FF steps of 64/fc   => 0x00FF * 4.72us = 1.2ms)       */
#define RFAL_ST25R95_MRT_MIN_1FC         rfalConv64fcTo1fc(0x0004)                    /*!< Min MRT steps in 1fc (0<=mrt<=4 ; 4 (64/fc)  => 0x0004 * 4.72us = 18.88us)      */
#define RFAL_ST25R95_GT_MAX_1FC          rfalConvMsTo1fc(5000)                        /*!< Max GT value allowed in 1/fc                                                    */
#define RFAL_ST25R95_GT_MIN_1FC          rfalConvMsTo1fc(RFAL_ST25R95_SW_TMR_MIN_1MS) /*!< Min GT value allowed in 1/fc                                                    */
#define RFAL_ST25R95_SW_TMR_MIN_1MS      1

#define RFAL_FELICA_POLL_DELAY_TIME     512                                           /*!<  FeliCa Poll Processing time is 2.417 ms ~512*64/fc Digital 1.1 A4              */
#define RFAL_FELICA_POLL_SLOT_TIME      256                                           /*!<  FeliCa Poll Time Slot duration is 1.208 ms ~256*64/fc Digital 1.1 A4           */

#define RFAL_ISO14443A_SDD_RES_LEN      5                                             /*!< SDD_RES | Anticollision (UID CLn) length  -  rfalNfcaSddRes                     */

#define RFAL_ST25R95_ISO14443A_APPENDCRC                                             0x20U /*!< Transmission flags bit 5: Append CRC        */
#define RFAL_ST25R95_ISO14443A_SPLITFRAME                                            0x40U /*!< Transmission flags bit 6: SplitFrame        */
#define RFAL_ST25R95_ISO14443A_TOPAZFORMAT                                           0x80U /*!< Transmission flags bit 7: Topaz send format */

#define RFAL_ST25R95_IDLE_DEFAULT_WUPERIOD                                           0x24U /*!< Fixed WU Period to reach ~300 ms timeout with Max Sleep = 0 */

#define ST25R95_TAGDETECT_DEF_CALIBRATION 0x7C             /*!< Tag Detection Calibration default value                    */

/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/
#define rfalTimerStart(timer, time_ms)         (timer) = timerCalculateTimer((uint16_t)(time_ms)) /*!< Configures and starts the RTOX timer          */
#define rfalTimerisExpired(timer)              timerIsExpired(timer)          /*!< Checks if timer has expired                   */


class RfalRfST25R95Class : public RfalRfClass {
  public:

    /*
    ******************************************************************************
    * RFAL RF FUNCTION PROTOTYPES
    ******************************************************************************
    */

    RfalRfST25R95Class(SPIClass *spi, int cs_pin, int irq_in_pin, int irq_out_pin, int interface_pin = -1, uint32_t spi_speed = 2000000);
    ReturnCode rfalInitialize(void);
    ReturnCode rfalCalibrate(void);
    ReturnCode rfalAdjustRegulators(uint16_t *result);
    void rfalSetUpperLayerCallback(rfalUpperLayerCallback pFunc);
    void rfalSetPreTxRxCallback(rfalPreTxRxCallback pFunc);
    void rfalSetPostTxRxCallback(rfalPostTxRxCallback pFunc);
    ReturnCode rfalDeinitialize(void);
    ReturnCode rfalSetMode(rfalMode mode, rfalBitRate txBR, rfalBitRate rxBR);
    rfalMode rfalGetMode(void);
    ReturnCode rfalSetBitRate(rfalBitRate txBR, rfalBitRate rxBR);
    ReturnCode rfalGetBitRate(rfalBitRate *txBR, rfalBitRate *rxBR);
    void rfalSetErrorHandling(rfalEHandling eHandling);
    rfalEHandling rfalGetErrorHandling(void);
    void rfalSetObsvMode(uint8_t txMode, uint8_t rxMode);
    void rfalGetObsvMode(uint8_t *txMode, uint8_t *rxMode);
    void rfalDisableObsvMode(void);
    void rfalSetFDTPoll(uint32_t FDTPoll);
    uint32_t rfalGetFDTPoll(void);
    void rfalSetFDTListen(uint32_t FDTListen);
    uint32_t rfalGetFDTListen(void);
    uint32_t rfalGetGT(void);
    void rfalSetGT(uint32_t GT);
    bool rfalIsGTExpired(void);
    ReturnCode rfalFieldOnAndStartGT(void);
    ReturnCode rfalFieldOff(void);
    ReturnCode rfalStartTransceive(const rfalTransceiveContext *ctx);
    rfalTransceiveState rfalGetTransceiveState(void);
    ReturnCode rfalGetTransceiveStatus(void);
    bool rfalIsTransceiveInTx(void);
    bool rfalIsTransceiveInRx(void);
    ReturnCode rfalGetTransceiveRSSI(uint16_t *rssi);
    void rfalWorker(void);
    ReturnCode rfalISO14443ATransceiveShortFrame(rfal14443AShortFrameCmd txCmd, uint8_t *rxBuf, uint8_t rxBufLen, uint16_t *rxRcvdLen, uint32_t fwt);
    ReturnCode rfalISO14443ATransceiveAnticollisionFrame(uint8_t *buf, uint8_t *bytesToSend, uint8_t *bitsToSend, uint16_t *rxLength, uint32_t fwt);
    ReturnCode rfalFeliCaPoll(rfalFeliCaPollSlots slots, uint16_t sysCode, uint8_t reqCode, rfalFeliCaPollRes *pollResList, uint8_t pollResListSize, uint8_t *devicesDetected, uint8_t *collisionsDetected);
    ReturnCode rfalISO15693TransceiveAnticollisionFrame(uint8_t *txBuf, uint8_t txBufLen, uint8_t *rxBuf, uint8_t rxBufLen, uint16_t *actLen);
    ReturnCode rfalISO15693TransceiveEOFAnticollision(uint8_t *rxBuf, uint8_t rxBufLen, uint16_t *actLen);
    ReturnCode rfalISO15693TransceiveEOF(uint8_t *rxBuf, uint8_t rxBufLen, uint16_t *actLen);
    ReturnCode rfalTransceiveBlockingTx(uint8_t *txBuf, uint16_t txBufLen, uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *actLen, uint32_t flags, uint32_t fwt);
    ReturnCode rfalTransceiveBlockingRx(void);
    ReturnCode rfalTransceiveBlockingTxRx(uint8_t *txBuf, uint16_t txBufLen, uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *actLen, uint32_t flags, uint32_t fwt);
    bool rfalIsExtFieldOn(void);
    ReturnCode rfalListenStart(uint32_t lmMask, const rfalLmConfPA *confA, const rfalLmConfPB *confB, const rfalLmConfPF *confF, uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *rxLen);
    ReturnCode rfalListenSleepStart(rfalLmState sleepSt, uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *rxLen);
    ReturnCode rfalListenStop(void);
    rfalLmState rfalListenGetState(bool *dataFlag, rfalBitRate *lastBR);
    ReturnCode rfalListenSetState(rfalLmState newSt);
    ReturnCode rfalWakeUpModeStart(const rfalWakeUpConfig *config);
    bool rfalWakeUpModeHasWoke(void);
    ReturnCode rfalWakeUpModeStop(void);


    /*
    ******************************************************************************
    * RFAL ANALOG CONFIG FUNCTION PROTOTYPES
    ******************************************************************************
    */


    /*!
     *****************************************************************************
     * \brief Initialize the Analog Configuration
     *
     * Reset the Analog Configuration LUT pointer to reference to default settings.
     *
     *****************************************************************************
     */
    void rfalAnalogConfigInitialize(void);


    /*!
     *****************************************************************************
     * \brief Indicate if the current Analog Configuration Table is complete and ready to be used.
     *
     * \return true if current Analog Configuration Table is complete and ready to be used.
     * \return false if current Analog Configuration Table is incomplete
     *
     *****************************************************************************
     */
    bool rfalAnalogConfigIsReady(void);

    /*!
     *****************************************************************************
     * \brief  Write the whole Analog Configuration table in raw format
     *
     * Writes the Analog Configuration and Look Up Table with the given raw table
     *
     * NOTE: Function does not check the validity of the given Table contents
     *
     * \param[in]  configTbl:     location of config Table to be loaded
     * \param[in]  configTblSize: size of the config Table to be loaded
     *
     * \return ERR_NONE    : if setting is updated
     * \return ERR_PARAM   : if configTbl is invalid
     * \return ERR_NOMEM   : if the given Table is bigger exceeds the max size
     * \return ERR_REQUEST : if the update Configuration Id is disabled
     *
     *****************************************************************************
     */
    ReturnCode rfalAnalogConfigListWriteRaw(const uint8_t *configTbl, uint16_t configTblSize);

    /*!
     *****************************************************************************
     * \brief  Write the Analog Configuration table with new analog settings.
     *
     * Writes the Analog Configuration and Look Up Table with the new list of register-mask-value
     * and Configuration ID respectively.
     *
     * NOTE: Function does not check for the validity of the Register Address.
     *
     * \param[in]  more: 0x00 indicates it is last Configuration ID settings;
     *                   0x01 indicates more Configuration ID setting(s) are coming.
     * \param[in]  *config: reference to the configuration list of current Configuration ID.
     *
     * \return ERR_PARAM   : if Configuration ID or parameter is invalid
     * \return ERR_NOMEM   : if LUT is full
     * \return ERR_REQUEST : if the update Configuration Id is disabled
     * \return ERR_NONE    : if setting is updated
     *
     *****************************************************************************
     */
    ReturnCode rfalAnalogConfigListWrite(uint8_t more, const rfalAnalogConfig *config);

    /*!
     *****************************************************************************
     * \brief  Read the whole Analog Configuration table in raw format
     *
     * Reads the whole Analog Configuration Table in raw format
     *
     * \param[out]   tblBuf: location to the buffer to place the Config Table
     * \param[in]    tblBufLen: length of the buffer to place the Config Table
     * \param[out]   configTblSize: Config Table size
     *
     * \return ERR_PARAM : if configTbl or configTblSize is invalid
     * \return ERR_NOMEM : if configTblSize is not enough for the whole table
     * \return ERR_NONE  : if read is successful
     *
     *****************************************************************************
     */
    ReturnCode rfalAnalogConfigListReadRaw(uint8_t *tblBuf, uint16_t tblBufLen, uint16_t *configTblSize);

    /*!
     *****************************************************************************
     * \brief  Read the Analog Configuration table.
     *
     * Read the Analog Configuration Table
     *
     * \param[in]     configOffset: offset to the next Configuration ID in the List Table to be read.
     * \param[out]    more: 0x00 indicates it is last Configuration ID settings;
     *                      0x01 indicates more Configuration ID setting(s) are coming.
     * \param[out]    config: configuration id, number of configuration sets and register-mask-value sets
     * \param[in]     numConfig: the remaining configuration settings space available;
     *
     * \return ERR_NOMEM : if number of Configuration for respective Configuration ID is greater the the remaining configuration setting space available
     * \return ERR_NONE  : if read is successful
     *
     *****************************************************************************
     */
    ReturnCode rfalAnalogConfigListRead(rfalAnalogConfigOffset *configOffset, uint8_t *more, rfalAnalogConfig *config, rfalAnalogConfigNum numConfig);

    /*!
     *****************************************************************************
     * \brief  Set the Analog settings of indicated Configuration ID.
     *
     * Update the chip with indicated analog settings of indicated Configuration ID.
     *
     * \param[in]  configId: configuration ID
     *
     * \return ERR_PARAM if Configuration ID is invalid
     * \return ERR_INTERNAL if error updating setting to chip
     * \return ERR_NONE if new settings is applied to chip
     *
     *****************************************************************************
     */
    ReturnCode rfalSetAnalogConfig(rfalAnalogConfigId configId);


    /*
    ******************************************************************************
    * RFAL RF CHIP FUNCTION PROTOTYPES
    ******************************************************************************
    */


    /*!
     *****************************************************************************
     * \brief Writes a register on the RF Chip
     *
     * Checks if the given register is valid and if so, writes the value(s)
     * on the RF Chip register
     *
     * \param[in] reg: register address to be written, or the first if len > 1
     * \param[in] values: pointer with content to be written on the register(s)
     * \param[in] len: number of consecutive registers to be written
     *
     *
     * \return ERR_PARAM    : Invalid register or bad request
     * \return ERR_NOTSUPP  : Feature not supported
     * \return ERR_NONE     : Write done with no error
     *****************************************************************************
     */
    ReturnCode rfalChipWriteReg(uint16_t reg, const uint8_t *values, uint8_t len);

    /*!
     *****************************************************************************
     * \brief Reads a register on the RF Chip
     *
     * Checks if the given register is valid and if so, reads the value(s)
     * of the RF Chip register(s)
     *
     * \param[in]  reg: register address to be read, or the first if len > 1
     * \param[out] values: pointer where the register(s) read content will be placed
     * \param[in]  len: number of consecutive registers to be read
     *
     * \return ERR_PARAM    : Invalid register or bad request
     * \return ERR_NOTSUPP  : Feature not supported
     * \return ERR_NONE     : Read done with no error
     *****************************************************************************
     */
    ReturnCode rfalChipReadReg(uint16_t reg, uint8_t *values, uint8_t len);

    /*!
     *****************************************************************************
     * \brief Change a register on the RF Chip
     *
     * Change the value of the register bits on the RF Chip Test set in the valueMask.
     *
     * \param[in] reg: register address to be modified
     * \param[in] valueMask: mask value of the register bits to be changed
     * \param[in] value: register value to be set
     *
     * \return ERR_PARAM    : Invalid register or bad request
     * \return ERR_NOTSUPP  : Feature not supported
     * \return ERR_OK       : Change done with no error
     *****************************************************************************
     */
    ReturnCode rfalChipChangeRegBits(uint16_t reg, uint8_t valueMask, uint8_t value);

    /*!
     *****************************************************************************
     * \brief Writes a Test register on the RF Chip
     *
     * Writes the value on the RF Chip Test register
     *
     * \param[in] reg: register address to be written
     * \param[in] value: value to be written on the register
     *
     *
     * \return ERR_PARAM    : Invalid register or bad request
     * \return ERR_NOTSUPP  : Feature not supported
     * \return ERR_NONE     : Write done with no error
     *****************************************************************************
     */
    ReturnCode rfalChipWriteTestReg(uint16_t reg, uint8_t value);

    /*!
     *****************************************************************************
     * \brief Reads a Test register on the RF Chip
     *
     * Reads the value of the RF Chip Test register
     *
     * \param[in]  reg: register address to be read
     * \param[out] value: pointer where the register content will be placed
     *
     * \return ERR_PARAM    :Invalid register or bad request
     * \return ERR_NOTSUPP  : Feature not supported
     * \return ERR_NONE     : Read done with no error
     *****************************************************************************
     */
    ReturnCode rfalChipReadTestReg(uint16_t reg, uint8_t *value);

    /*!
     *****************************************************************************
     * \brief Change a Test register on the RF Chip
     *
     * Change the value of the register bits on the RF Chip Test set in the valueMask.
     *
     * \param[in] reg: test register address to be modified
     * \param[in] valueMask: mask value of the register bits to be changed
     * \param[in] value: register value to be set
     *
     * \return ERR_PARAM     : Invalid register or bad request
     * \return ERR_NOTSUPP   : Feature not supported
     * \return ERR_OK        : Change done with no error
     *****************************************************************************
     */
    ReturnCode rfalChipChangeTestRegBits(uint16_t reg, uint8_t valueMask, uint8_t value);

    /*!
     *****************************************************************************
     * \brief Execute command on the RF Chip
     *
     * Checks if the given command is valid and if so, executes it on
     * the RF Chip
     *
     * \param[in] cmd: direct command to be executed
     *
     * \return ERR_PARAM     : Invalid command or bad request
     * \return  ERR_NOTSUPP  : Feature not supported
     * \return ERR_NONE      : Direct command executed with no error
     *****************************************************************************
     */
    ReturnCode rfalChipExecCmd(uint16_t cmd);

    /*!
     *****************************************************************************
     * \brief  Set RFO
     *
     * Sets the RFO value to be used when the field is on (unmodulated/active)
     *
     * \param[in] rfo : the RFO value to be used
     *
     * \return  ERR_IO           : Internal error
     * \return  ERR_NOTSUPP      : Feature not supported
     * \return  ERR_NONE         : No error
     *****************************************************************************
     */
    ReturnCode rfalChipSetRFO(uint8_t rfo);


    /*!
     *****************************************************************************
     * \brief  Get RFO
     *
     * Gets the RFO value used used when the field is on (unmodulated/active)
     *
     * \param[out] result : the current RFO value
     *
     * \return  ERR_IO           : Internal error
     * \return  ERR_NOTSUPP      : Feature not supported
     * \return  ERR_NONE         : No error
     *****************************************************************************
     */
    ReturnCode rfalChipGetRFO(uint8_t *result);


    /*!
     *****************************************************************************
     * \brief  Measure Amplitude
     *
     * Measures the RF Amplitude
     *
     * \param[out] result : result of RF measurement
     *
     * \return  ERR_IO           : Internal error
     * \return  ERR_NOTSUPP      : Feature not supported
     * \return  ERR_NONE         : No error
     *****************************************************************************
     */
    ReturnCode rfalChipMeasureAmplitude(uint8_t *result);


    /*!
     *****************************************************************************
     * \brief  Measure Phase
     *
     * Measures the Phase
     *
     * \param[out] result : result of Phase measurement
     *
     * \return  ERR_IO           : Internal error
     * \return  ERR_NOTSUPP      : Feature not supported
     * \return  ERR_NONE         : No error
     *****************************************************************************
     */
    ReturnCode rfalChipMeasurePhase(uint8_t *result);


    /*!
     *****************************************************************************
     * \brief  Measure Capacitance
     *
     * Measures the Capacitance
     *
     * \param[out] result : result of Capacitance measurement
     *
     * \return  ERR_IO           : Internal error
     * \return  ERR_NOTSUPP      : Feature not supported
     * \return  ERR_NONE         : No error
     *****************************************************************************
     */
    ReturnCode rfalChipMeasureCapacitance(uint8_t *result);


    /*!
     *****************************************************************************
     * \brief  Measure Power Supply
     *
     * Measures the Power Supply
     *
     * \param[in]   param : measurement parameter (chip specific)
     * \param[out] result : result of the measurement
     *
     * \return  ERR_IO           : Internal error
     * \return  ERR_NOTSUPP      : Feature not supported
     * \return  ERR_NONE         : No error
     *****************************************************************************
     */
    ReturnCode rfalChipMeasurePowerSupply(uint8_t param, uint8_t *result);


    /*
    ******************************************************************************
    * RFAL CRC FUNCTION PROTOTYPES
    ******************************************************************************
    */


    /*!
     *****************************************************************************
     *  \brief  Calculate CRC according to CCITT standard.
     *
     *  This function takes \a length bytes from \a buf and calculates the CRC
     *  for this data. The result is returned.
     *  \note This implementation calculates the CRC with LSB first, i.e. all
     *  bytes are "read" from right to left.
     *
     *  \param[in] preloadValue : Initial value of CRC calculation.
     *  \param[in] buf : buffer to calculate the CRC for.
     *  \param[in] length : size of the buffer.
     *
     *  \return 16 bit long crc value.
     *
     *****************************************************************************
     */
    uint16_t rfalCrcCalculateCcitt(uint16_t preloadValue, const uint8_t *buf, uint16_t length);


    /*
    ******************************************************************************
    * RFAL ISO 15693_2 FUNCTION PROTOTYPES
    ******************************************************************************
    */


    /*!
     *****************************************************************************
     *  \brief  Initialize the ISO15693 phy
     *
     *  \param[in] config : ISO15693 phy related configuration (See #iso15693PhyConfig_t)
     *  \param[out] needed_stream_config : return a pointer to the stream config
     *              needed for this iso15693 config. To be used for configure RF chip.
     *
     *  \return ERR_IO : Error during communication.
     *  \return ERR_NONE : No error.
     *
     *****************************************************************************
     */
    ReturnCode iso15693PhyConfigure(const iso15693PhyConfig_t *config,
                                    const struct iso15693StreamConfig **needed_stream_config);

    /*!
     *****************************************************************************
     *  \brief  Return current phy configuration
     *
     *  This function returns current Phy configuration previously
     *  set by #iso15693PhyConfigure
     *
     *  \param[out] config : ISO15693 phy configuration.
     *
     *  \return ERR_NONE : No error.
     *
     *****************************************************************************
     */
    ReturnCode iso15693PhyGetConfiguration(iso15693PhyConfig_t *config);

    /*!
     *****************************************************************************
     *  \brief  Code an ISO15693 compatible frame
     *
     *  This function takes \a length bytes from \a buffer, perform proper
     *  encoding and sends out the frame to the ST25R391x.
     *
     *  \param[in] buffer : data to send, modified to adapt flags.
     *  \param[in] length : number of bytes to send.
     *  \param[in] sendCrc : If set to true, CRC is appended to the frame
     *  \param[in] sendFlags: If set to true, flag field is sent according to
     *                        ISO15693.
     *  \param[in] picopassMode :  If set to true, the coding will be according to Picopass
     *  \param[out] subbit_total_length : Return the complete bytes which need to
     *                                   be send for the current coding
     *  \param[in,out] offset : Set to 0 for first transfer, function will update it to
                  point to next byte to be coded
     *  \param[out] outbuf : buffer where the function will store the coded subbit stream
     *  \param[out] outBufSize : the size of the output buffer
     *  \param[out] actOutBufSize : the amount of data stored into the buffer at this call
     *
     *  \return ERR_IO : Error during communication.
     *  \return ERR_AGAIN : Data was not coded all the way. Call function again with a new/emptied buffer
     *  \return ERR_NO_MEM : In case outBuf is not big enough. Needs to have at
                 least 5 bytes for 1of4 coding and 65 bytes for 1of256 coding
     *  \return ERR_NONE : No error.
     *
     *****************************************************************************
     */
    ReturnCode iso15693VCDCode(uint8_t *buffer, uint16_t length, bool sendCrc, bool sendFlags, bool picopassMode,
                               uint16_t *subbit_total_length, uint16_t *offset,
                               uint8_t *outbuf, uint16_t outBufSize, uint16_t *actOutBufSize);


    /*!
     *****************************************************************************
     *  \brief  Receive an ISO15693 compatible frame
     *
     *  This function receives an ISO15693 frame from the ST25R391x, decodes the frame
     *  and writes the raw data to \a buffer.
     *  \note Buffer needs to be big enough to hold CRC also (+2 bytes)
     *
     *  \param[in] inBuf : buffer with the hamming coded stream to be decoded
     *  \param[in] inBufLen : number of bytes to decode (=length of buffer).
     *  \param[out] outBuf : buffer where received data shall be written to.
     *  \param[in] outBufLen : Length of output buffer, should be approx twice the size of inBuf
     *  \param[out] outBufPos : The number of decoded bytes. Could be used in
     *                          extended implementation to allow multiple calls
     *  \param[out] bitsBeforeCol : in case of ERR_COLLISION this value holds the
     *   number of bits in the current byte where the collision happened.
     *  \param[in] ignoreBits : number of bits in the beginning where collisions will be ignored
     *  \param[in] picopassMode :  if set to true, the decoding will be according to Picopass
     *
     *  \return ERR_COLLISION : collision occurred, data incorrect
     *  \return ERR_CRC : CRC error, data incorrect
     *  \return ERR_TIMEOUT : timeout waiting for data.
     *  \return ERR_NONE : No error.
     *
     *****************************************************************************
     */
    ReturnCode iso15693VICCDecode(const uint8_t *inBuf,
                                  uint16_t inBufLen,
                                  uint8_t *outBuf,
                                  uint16_t outBufLen,
                                  uint16_t *outBufPos,
                                  uint16_t *bitsBeforeCol,
                                  uint16_t ignoreBits,
                                  bool picopassMode);


    /*
    ******************************************************************************
    * RFAL ST25R95 FUNCTION PROTOTYPES
    ******************************************************************************
    */

    /*!
     *****************************************************************************
     *  \brief  Initialise ST25R95 driver
     *
     *  This function initialises the ST25R95 driver.
     *
     *  \return ERR_NONE    : Operation successful
     *  \return Other       : Init failed
     *
     *****************************************************************************
     */
    ReturnCode st25r95Initialize(void);

    /*!
     *****************************************************************************
     *  \brief  Deinitialize ST25R95 driver
     *
     *  Calling this function deinitializes the ST25R95 driver.
     *
     *****************************************************************************
     */
    void st25r95Deinitialize(void);

    /*!
     *****************************************************************************
     *  \brief  Generates an IRQ_IN pulse (SPI intefrace)
     *
     *  Generates an IRQ_IN pulse (see CR95HF DS §3.2).
     *
     *
     *****************************************************************************
     */
    void st25r95SPI_nIRQ_IN_Pulse(void);

    /*!
     *****************************************************************************
     *  \brief  Check Identity
     *
     *  Checks if the chip ID is as expected.
     *
     *  \return  true when IC type is as expected
     *
     *****************************************************************************
     */
    bool st25r95CheckChipID(void);

    /*!
     *****************************************************************************
     *  \brief  Turns field on
     *
     *  This function turns field On
     *
     *  \param[in]   protocol: ST25R95_PROTOCOL_xxx protocol value.
     *
     *  \return ERR_NONE    : Operation successful
     *
     *****************************************************************************
     */
    ReturnCode st25r95FieldOn(uint32_t protocol);

    /*!
     *****************************************************************************
     *  \brief  Turns field off
     *
     *  This function turns field Off
     *
     *  \return ERR_NONE    : Operation successful
     *
     *****************************************************************************
     */
    ReturnCode st25r95FieldOff(void);


    /*
    ******************************************************************************
    * RFAL ST25R95 COM FUNCTION PROTOTYPES
    ******************************************************************************
    */


    /*!
     *****************************************************************************
     *  \brief  Resets the ST25R95 (SPI interface)
     *
     *  This function is used to reset the ST25R95 as per CR95HF DS § 4.2.1 .
     *
     *
     *****************************************************************************
     */
    void st25r95SPIResetChip(void);

    /*!
     *****************************************************************************
     *  \brief  Sends a command to ST25R95 and reads the response (SPI Interface)
     *
     *  This function is used to send a \a cmd and read the \a resp. The \a cmd command
     *  must follow the CMD LEN format (i.e. any command except echo)
     *
     *  \param[in]   cmd: Command as per CR95HF DS §5.
     *  \param[out]  resp: buffer for the response.
     *  \param[in]   respBuffLen: response buffer length
     *
     *  \return ERR_NONE    : Operation successful
     *  \return ERR_TIMEOUT : Timeout
     *
     *****************************************************************************
     */
    ReturnCode st25r95SPISendCommandTypeAndLen(uint8_t *cmd, uint8_t *resp, uint16_t respBuffLen);


    /*!
     *****************************************************************************
     *  \brief  Sends a echo command to ST25R95 (SPI Interface)
     *
     *  This function is used to send an echo command
     *
     *  \return ERR_NONE    : Operation successful (echo response received)
     *  \return ERR_TIMEOUT : Timeout
     *
     *****************************************************************************
     */
    ReturnCode st25r95SPICommandEcho(void);


    /*!
     *****************************************************************************
     *  \brief  Sends one byte over SPI and returns the byte received
     *
     *  This function is used to send one byte over SPI and  returns the byte received
     *
     *  \param[in]   data: byte to be sent over SPI
     *
     *  \return ERR_NONE    : Operation successful (echo response received)
     *  \return ERR_TIMEOUT : Timeout
     *
     *****************************************************************************
     */
    uint8_t st25r95SPISendReceiveByte(uint8_t data);

    /*!
     *****************************************************************************
     *  \brief  Sends ProtocolSelect command
     *
     *  This function is used to send ST25R95 ProtocolSelect command
     *
     *  \param[in]   protocol: value of the protocol
     *
     *  \return ERR_NONE    : Operation successful
     *  \return ERR_TIMEOUT : Timeout
     *  \return ERR_PARAM   : ProtocolSelect failure
     *
     *****************************************************************************
     */
    ReturnCode st25r95ProtocolSelect(uint8_t protocol);

    /*!
     *****************************************************************************
     *  \brief  Set bit rates
     *
     *  This function is used to set Tx and Rx bit rates
     *
     *  \param[in]   protocol: value of the protocol
     *  \param[in]   txBR: tx bit rate
     *  \param[in]   rxBR: rx bit rate
     *
     *  \return ERR_NONE            : Operation successful
     *  \return ERR_NOT_IMPLEMENTED : unsupported bit rate value
     *
     *****************************************************************************
     */
    ReturnCode st25r95SetBitRate(uint8_t protocol, rfalBitRate txBR, rfalBitRate rxBR);

    /*!
     *****************************************************************************
     *  \brief  Sends data (SPI interface)
     *
     *  This function is used to send data
     *
     *  \param[in]   buf     : data to be sent
     *  \param[in]   bufLen  : buffer length
     *  \param[in]   protocol: protocol index
     *  \param[in]   flags   : TransceiveFlags indication special handling
     *
     *
     *****************************************************************************
     */
    void st25r95SPISendData(uint8_t *buf, uint8_t bufLen, uint8_t protocol, uint32_t flags);

    /*!
     *****************************************************************************
     *  \brief  Prepares Rx (SPI Interface)
     *
     *  This function is used to send data
     *
     *   \warning This method should be called only if polling is successful
     *
     *  \param[in]   protocol           : protocol index
     *  \param[in]   rxBuf              : buffer to place the response
     *  \param[in]   rxBufLen           : length of rxBuf
     *  \param[in]   rxRcvdLen          : received length
     *  \param[in]   flags              : TransceiveFlags indication special handling
     *  \param[in]   additionalRespBytes: additional response bytes
     *
     *****************************************************************************
     */
    void st25r95SPIPrepareRx(uint8_t protocol, uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *rxRcvdLen, uint32_t flags, uint8_t *additionalRespBytes);

    /*!
     *****************************************************************************
     *  \brief  Polls the ST25R95 for incoming data (SPI Interface)
     *
     *  This function is used to poll the ST25R95 for incoming data
     *
     *  \param[in]   timeout: timeout value
     *
     *  \return ERR_NONE    : Data ready to be read
     *  \return ERR_TIMEOUT : No data available
     *
     *****************************************************************************
     */
    ReturnCode st25r95SPIPollRead(uint32_t timeout);

    /*!
     *****************************************************************************
     *  \brief  Polls the ST25R95 for sending (SPI Interface)
     *
     *  This function is used to poll the ST25R95 before sending data
     *
     *  \return ERR_NONE    : ST25R95 ready to receive data
     *  \return ERR_TIMEOUT : ST25R95 not ready to receive data
     *
     *****************************************************************************
     */
    ReturnCode st25r95SPIPollSend(void);

    /*!
     *****************************************************************************
     *  \brief  set FWT
     *
     *  This function is used to set the FWT for the next frames
     *
     *  \param[in]   protocol: protocol index
     *  \param[in]   fwt: frame waiting time
     *
     *  \return ERR_NONE    : Operation successful
     *
     *****************************************************************************
     */
    ReturnCode st25r95SetFWT(uint8_t protocol, uint32_t fwt);

    /*!
     *****************************************************************************
     *  \brief  set slot counter
     *
     *  This function is used to set the slot counter (ISO18092)
     *
     *  \param[in]   slots: number of slots
     *
     *  \return ERR_NONE    : Operation successful
     *
     *****************************************************************************
     */
    ReturnCode st25r95SetSlotCounter(uint8_t slots);

    /*!
     *****************************************************************************
     *  \brief  Write Register
     *
     *  This function is used to write ST25R95 ARC_B register
     *
     *  \param[in]   protocol: protocol index
     *  \param[in]   reg: register (currently only ST25R95_REG_ARC_B register is defined)
     *  \param[in]   value: register value
     *
     *  \return ERR_NONE    : Operation successful
     *
     *****************************************************************************
     */
    ReturnCode st25r95WriteReg(uint8_t protocol, uint16_t reg, uint8_t value);

    /*!
     *****************************************************************************
     *  \brief  Read Register
     *
     *  This function is used to read ST25R95 ARC_B register
     *
     *  \param[in]   reg: register (currently only ST25R95_REG_ARC_B register is defined)
     *  \param[out]  value: register value
     *
     *  \return ERR_NONE    : Operation successful
     *
     *****************************************************************************
     */
    ReturnCode st25r95ReadReg(uint16_t reg, uint8_t *value);

    /*!
     *****************************************************************************
     *  \brief End of transmit test function (SPI mode)
     *
     *  This function is used to test whether the current transmit is finished or not.
     *
     *  \return true    : Transmit completed
     *  \return false   : Transmit not completed
     *
     *****************************************************************************
     */
    bool st25r95SPIIsTransmitCompleted(void);

    /*!
     *****************************************************************************
     *  \brief Listen mode test function (CE mode)
     *
     *  This function is used to test whether the ST25R95 is currently in Listen mode.
     *
     *  \return true    : Listen mode on going
     *  \return false   : Not in Listen mode
     *
     *****************************************************************************
     */
    bool st25r95SPIIsInListen(void);

    /*!
     *****************************************************************************
     *  \brief  Send Idle command (SPI mode)
     *
     *  This function is used to send the Idle command.
     *
     *  \param[in]   dacDataL: lower comparator value
     *  \param[in]   dacDataH: higher comparator value
     *  \param[in]   WUPeriod: time between 2 detections
     *
     *****************************************************************************
     */
    void st25r95SPIIdle(uint8_t dacDataL, uint8_t dacDataH, uint8_t WUPeriod);

    /*!
     *****************************************************************************
     *  \brief  Flush the Idle response (SPI mode)
     *
     *  This function is used to flush the Idle response.
     *
     *****************************************************************************
     */
    void st25r95SPIGetIdleResponse(void);

    /*!
     *****************************************************************************
     *  \brief  Send transmit flag (SPI mode)
     *
     *  This function is used to send the transmit flag during the SendRecv command.
     *
     *  \param[in]   protocol: value of the protocol
     *  \param[in]   transmitFlag: transmission flags
     *
     *****************************************************************************
     */
    void st25r95SPISendTransmitFlag(uint8_t protocol, uint8_t transmitFlag);

    /*!
     *****************************************************************************
     *  \brief  Process end of Rx (SPI mode)
     *
     *  This function is used to conclude the Rx request and returns the status of the Rx.
     *
     *  \return ERR_NONE    : Operation successful
     *
     *****************************************************************************
     */
    ReturnCode st25r95SPICompleteRx(void);

    /*!
     *****************************************************************************
     *  \brief  return to ready mode (SPI mode)
     *
     *  This function is used to exit from Idle mode and return to Ready mode.
     *
     *****************************************************************************
     */
    void st25r95SPIKillIdle(void);

    /*!
     *****************************************************************************
     *  \brief Calibrate Tag Detector
     *
     *  This function returns the Tag Detector DAC calibration value.
     *
     * \return uint8_t Any : calibration value
     *
     *****************************************************************************
     */
    uint8_t st25r95CalibrateTagDetector(void);

    /*!
     *****************************************************************************
     *  \brief SPI transceive function
     *
     *  This function is used for SPI communication.
     *
     *  \param[in]   txData: Tx Data
     *  \param[out]  rxData: Rx Data
     *  \param[in]   length: Tx/Rx Data buffer len
     *
     *****************************************************************************
     */
    void st25r95SPIRxTx(uint8_t *txData, uint8_t *rxData, uint16_t length);

    /*
    ******************************************************************************
    * RFAL ST25R95 TIMER FUNCTION PROTOTYPES
    ******************************************************************************
    */


    /*!
     *****************************************************************************
     * \brief  Calculate Timer
     *
     * This method calculates when the timer will be expired given the amount
     * time in milliseconds /a tOut.
     * Once the timer has been calculated it will then be used to check when
     * it expires.
     *
     * \see timersIsExpired
     *
     * \param[in]  time : time/duration in Milliseconds for the timer
     *
     * \return u32 : The new timer calculated based on the given time
     *****************************************************************************
     */
    uint32_t timerCalculateTimer(uint16_t time);


    /*!
     *****************************************************************************
     * \brief  Checks if a Timer is Expired
     *
     * This method checks if a timer has already expired.
     * Based on the given timer previously calculated it checks if this timer
     * has already elapsed
     *
     * \see timersCalculateTimer
     *
     * \param[in]  timer : the timer to check
     *
     * \return true  : timer has already expired
     * \return false : timer is still running
     *****************************************************************************
     */
    bool timerIsExpired(uint32_t timer);


    /*!
     *****************************************************************************
     * \brief  Performs a Delay
     *
     * This method performs a delay for the given amount of time in Milliseconds
     *
     * \param[in]  time : time/duration in Milliseconds of the delay
     *
     *****************************************************************************
     */
    void timerDelay(uint16_t time);


    /*!
     *****************************************************************************
     * \brief  Stopwatch start
     *
     * This method initiates the stopwatch to later measure the time in ms
     *
     *****************************************************************************
     */
    void timerStopwatchStart(void);


    /*!
     *****************************************************************************
     * \brief  Stopwatch Measure
     *
     * This method returns the elapsed time in ms since the stopwatch was initiated
     *
     * \return The time in ms since the stopwatch was started
     *****************************************************************************
     */
    uint32_t timerStopwatchMeasure(void);

  protected:

    void rfalTransceiveTx(void);
    void rfalTransceiveRx(void);
    ReturnCode rfalTransceiveRunBlockingTx(void);
    bool rfalChipIsBusy(void);
    ReturnCode rfalRunTransceiveWorker(void);
    void rfalRunWakeUpModeWorker(void);

    rfalAnalogConfigNum rfalAnalogConfigSearch(rfalAnalogConfigId configId, uint16_t *configOffset);
    uint16_t rfalCrcUpdateCcitt(uint16_t crcSeed, uint8_t dataByte);

    SPIClass *dev_spi;
    int cs_pin;
    int irq_in_pin;
    int irq_out_pin;
    int interface_pin;
    uint32_t spi_speed;

    rfal gRFAL;              /*!< RFAL module instance               */
    rfalAnalogConfigMgmt gRfalAnalogConfigMgmt;  /*!< Analog Configuration LUT management */
    iso15693PhyConfig_t iso15693PhyConfig; /*!< current phy configuration */
    st25r95SPIRxContext st25r95SPIRxCtx; /*!< Context for SPI transceive     */
    uint32_t timerStopwatchTick;
};

#ifdef __cplusplus
extern "C" {
#endif
ReturnCode iso15693PhyVCDCode1Of4(const uint8_t data, uint8_t *outbuffer, uint16_t maxOutBufLen, uint16_t *outBufLen);
ReturnCode iso15693PhyVCDCode1Of256(const uint8_t data, uint8_t *outbuffer, uint16_t maxOutBufLen, uint16_t *outBufLen);
#ifdef __cplusplus
}
#endif

#endif /* RFAL_RFST25R95_H */
