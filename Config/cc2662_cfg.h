/*********************************************************************************************************************
 *  COPYRIGHT
 *  ------------------------------------------------------------------------------------------------------------------
 *  \verbatim
 *
 *                 TEXAS INSTRUMENTS INCORPORATED PROPRIETARY INFORMATION
 *
 *                 Property of Texas Instruments, Unauthorized reproduction and/or distribution
 *                 is strictly prohibited.  This product  is  protected  under  copyright  law
 *                 and  trade  secret law as an  unpublished work.
 *                 (C) Copyright 2021 Texas Instruments Inc.  All rights reserved.
 *
 *  \endverbatim
 *  ------------------------------------------------------------------------------------------------------------------
 *  FILE DESCRIPTION
 *  ------------------------------------------------------------------------------------------------------------------
 *  File:       cc2662_cfg.h
 *  Project:    TIBMS
 *  Module:     CONFIG
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for WBMS interface
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05May2022    SEM                0000000000000    Initial version
 * 01.01.00       22June2023   SEM                0000000000000    Command Aggregation Implementation
 *                                                                 Performance Improvements
 * 02.00.00       31Oct2023    SEM                0000000000000    Code Optimization for B8.0 Release
 * 02.01.00       09Nov2023    MF                 0000000000000    Added OAD and New WD Cmd for 8.1 Release
 *********************************************************************************************************************/

#ifndef CC2662_CFG_H
#define CC2662_CFG_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************************************************************
 * Standard Header Files
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Other Header Files
 *********************************************************************************************************************/

#include "tibms_bmi.h"

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

#define CC2662_CFG_MAJOR_VERSION                   (0x02u)
#define CC2662_CFG_MINOR_VERSION                   (0x02u)
#define CC2662_CFG_PATCH_VERSION                   (0x00u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

#define CC2662_IFACES	                            (1u)

#define CC2662_MANUAL_NETWORK_START                 STD_ON          /** Manual Network Start **/
#define CC2662_CMD_AGGREGATION                      STD_ON          /** Command Aggretagion ON **/
#define CC2662_MAX_RETRY_DISCARD                    STD_ON          /** Remove from Re-Trying the Packets **/
#define CC2662_MAC_QUE_ENABLED                      STD_ON          /** MAC QUE for packet optimizations **/

#define CC2662_TX_CMD_MAX_LEN                       (16)
#define CC2662_CMD_QUEUE_LEN_CHAIN_0                (63u)
#define CC2662_EXEC_QUEUE_LEN_CHAIN_0               (31u)
#define CC2662_READ_QUEUE_LEN_CHAIN_0               (7u)

#define CC2662_MAC_UID_SIZE                         (0x08u)
#define NPI_MAX_MACTABLE_IDX                        (0x01u)         /** 1x128 = 128 -> supports 14 devices MAC settings **/

//Manual Network
#define CC2662_USER_CFG_T_SLOT_UL_TIME              (70)    //(103)//70
#define CC2662_USER_CFG_T_SLOT_DL_TIME              (250)   //(153)//250//
#define CC2662_USER_CFG_MAX_PKT_TX_RETRIES          (5U)
#define CC2662_USER_CFG_KEEP_ALIVE_INT              (16U)
#define CC2662_USER_CFG_T_SLOT_UL2DL_TIME           (0)
#define CC2662_NUM_OF_SKIP_ALIV_INT                 (1)
#define CC2662_USER_CFG_DENYLIST                    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
#define CC2662_START_NW_TIMEOUT                     (16*1000000) //16secs
#define CC2662_DUALNW_TIMEOUT                       (16*1000000) //16Secs

#define CC2662_MANUAL_PERS_STARTUP_TIME             (300u)
#define CC2662_MANUAL_VOL_STARTUP_TIME              (20000u)
#define CC2662_MAX_RETRY                            (3u)

#define CC2662_BQDRIVER_INIT_DELAY                  (100u)

#define USER_CFG_wUID                               {0x012345678, 0x12457865 }
/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef struct Cc2662_MACCfgType_Tag
{
    uint8 zDevUniqueID[CC2662_MAC_UID_SIZE];
    uint8 eSlotN;
}
Cc2662_MACCfgType;

typedef struct Cc2662_DfuMACCfgType_Tag
{
    uint8 zDfuUniqueID[CC2662_MAC_UID_SIZE];
    uint8 eDfuSlotN;
}
Cc2662_DfuMACCfgType;

typedef struct Cc2662_ReqDataType_Tag
{
    const ServiceCfgType *pServiceReq;
    uint8 *pRxData;

    uint8 zTxCmd[CC2662_TX_CMD_MAX_LEN];
    uint16 wReqDataLen;
    uint8 uBlockSiz;
    uint8 uStatus;

    uint8 uCmd;
    uint8 uCmdTyp;
    uint8 uCcReqTyp;
    uint8 uCmdInfo;
}
Cc2662_ReqDataType;

typedef struct Cc2662_NwCfgType_Tag
{
    const Cc2662_DfuMACCfgType **pDfuMacTbl;
    const Cc2662_MACCfgType **pMacTbl;
    uint16 wDfuMacCfgSiz;
    uint16 wMacCfgSiz;
    uint16 wNwPktLength;

    uint16 wWbmsNwId;
    uint16 wDownlinkTime;

    uint8 uNwOpMode;
	uint8 uNwJoinMode;
	uint8 uUplinkTime;
	uint8 uMaxRetries;

	uint8 uKeepAliveInt;
	uint8 uUltoDlTime;
	uint8 uNumofSkipAlivInt;
	uint8 uDenylist[5];
	uint8 uDualWmMode;
	uint8 uDualWmType;

	uint8 uProtoSByte;
	
	uint32 DevDisc_timeout;
    uint32 RePair_timeout;
    uint32 DualNwStart_timeout;

	uint64 wUID[2];

    uint8  wdStorageCmd;
    uint8  wdStorageMode;
    uint32 wdBackOffTime;
    uint32 wdScanTimeOut;
    uint8 uRsvd[3];

}
Cc2662_NwCfgType;

typedef struct Cc2662_ConfigType_Tag
{
    const Basic_ConfigType *pBswCfg;
	const Cc2662_NwCfgType *pNwCfg;

    Cc2662_ReqDataType *pCmdDataMemCfg;
    uint8 *pNpiTpBufMemCfg;

    void *pCmdQueFreeMemCfg;
    uint16 wCmdQueSizeCfg;
    uint16 wCmdQueCntCfg;

    void *(*pCmdQueExecMemCfg)[(CC2662_EXEC_QUEUE_LEN_CHAIN_0 + 1)];
    uint16 wExecQueSizeCfg;
    uint16 wExecQueCntCfg;

    void *pReadQueExecMemCfg;
    uint16 wReadQueSizeCfg;
    uint16 wReadQueCntCfg;

    uint16 wBqNpiTpBufLenCfg;
    uint8 uCmdIfaceCfg;
    uint8 uRsvd[3];
}
Cc2662_ConfigType;

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

extern CONST(Comif_ConfigType, TIBMSCFG_CONST) zBmiComifCfgWbmsCfg_0;

/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /*CC2662_CFG_H*/

/*********************************************************************************************************************
 * End of File: cc2662_cfg.h
 *********************************************************************************************************************/
