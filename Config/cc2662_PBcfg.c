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
 *  File:       cc2662_PBcfg_0.c
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

/*********************************************************************************************************************
 * Standard Header Files
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Other Header Files
 *********************************************************************************************************************/

#include "tibms_api.h"
#include "tibms_comif.h"

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

/**	Major Software Config C Version number */
#define CC2662_CFG_C_MAJOR_VERSION             (0x02u)

/**	Minor Software Config C Version number */
#define CC2662_CFG_C_MINOR_VERSION             (0x02u)

/** Software Patch Config C Version number */
#define CC2662_CFG_C_PATCH_VERSION             (0x00u)

#if ((CC2662_CFG_C_MAJOR_VERSION != CC2662_CFG_MAJOR_VERSION) || \
     (CC2662_CFG_C_MINOR_VERSION != CC2662_CFG_MINOR_VERSION) || \
	 (CC2662_CFG_C_PATCH_VERSION != CC2662_CFG_PATCH_VERSION))
#error "tibms: Config version numbers of cc2662_PBcfg_0.c and cc2662_cfg.h are inconsistent!"
#endif

/*********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

#define CC2662_PBCFG_0_START_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

static Cc2662_ReqDataType zReqDataMem_0[CC2662_CMD_QUEUE_LEN_CHAIN_0];
static uint8 zNpiTpBufMem_0[TIBMS_MAX_AFES_IN_IFACE_2*256U];
static void *pCmdQueFreeMem_0[(CC2662_CMD_QUEUE_LEN_CHAIN_0 + 1U)];
static void *pCmdQueExecMem_0[COMIF_CMD_PRIO_MAX][(CC2662_EXEC_QUEUE_LEN_CHAIN_0 + 1U)];
static void *pReadQueExecMem_0[(CC2662_READ_QUEUE_LEN_CHAIN_0 + 1U)];

#define CC2662_PBCFG_0_STOP_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 * Local CONST Object Definitions
 *********************************************************************************************************************/

#define CC2662_PBCFG_0_START_SEC_CONST
#include "Cdd_MemMap.h"

CONST(Cc2662_MACCfgType, TIBMSCFG_CONST) zMACInfoDev_0 = { { 0xDD, 0x6A, 0xCE, 0x4D, 0x71, 0xD8, 0xFF, 0xFF },  0 };         // MAC ID of Device 0
CONST(Cc2662_MACCfgType, TIBMSCFG_CONST) zMACInfoDev_1 = { { 0xD3, 0x51, 0x5B, 0x9B, 0xA8, 0xFC, 0xFF, 0xFF },  1 };         // MAC ID of Device 1
CONST(Cc2662_MACCfgType, TIBMSCFG_CONST) zMACInfoDev_2 = { { 0xF3, 0x6A, 0xCE, 0x4D, 0x71, 0xD8, 0xFF, 0xFF },  2 };         // MAC ID of Device 2
CONST(Cc2662_MACCfgType, TIBMSCFG_CONST) zMACInfoDev_3 = { { 0xF3, 0x6A, 0xCE, 0x4D, 0x71, 0xD8, 0xFF, 0xFF },  3 };         // MAC ID of Device 3
CONST(Cc2662_MACCfgType, TIBMSCFG_CONST) zMACInfoDev_4 = { { 0xF3, 0x6A, 0xCE, 0x4D, 0x71, 0xD8, 0xFF, 0xFF },  4 };         // MAC ID of Device 4
CONST(Cc2662_MACCfgType, TIBMSCFG_CONST) zMACInfoDev_5 = { { 0xF3, 0x6A, 0xCE, 0x4D, 0x71, 0xD8, 0xFF, 0xFF },  5 };         // MAC ID of Device 5
CONST(Cc2662_MACCfgType, TIBMSCFG_CONST) zMACInfoDev_6 = { { 0xF3, 0x6A, 0xCE, 0x4D, 0x71, 0xD8, 0xFF, 0xFF },  6 };         // MAC ID of Device 6
CONST(Cc2662_MACCfgType, TIBMSCFG_CONST) zMACInfoDev_7 = { { 0xF3, 0x6A, 0xCE, 0x4D, 0x71, 0xD8, 0xFF, 0xFF },  7 };         // MAC ID of Device 7
CONST(Cc2662_MACCfgType, TIBMSCFG_CONST) zMACInfoDev_8 = { { 0xF3, 0x6A, 0xCE, 0x4D, 0x71, 0xD8, 0xFF, 0xFF },  8 };         // MAC ID of Device 8
CONST(Cc2662_MACCfgType, TIBMSCFG_CONST) zMACInfoDev_9 = { { 0xF3, 0x6A, 0xCE, 0x4D, 0x71, 0xD8, 0xFF, 0xFF },  9 };         // MAC ID of Device 9
CONST(Cc2662_MACCfgType, TIBMSCFG_CONST) zMACInfoDev_10 = { { 0xF3, 0x6A, 0xCE, 0x4D, 0x71, 0xD8, 0xFF, 0xFF },  10 };        // MAC ID of Device 10
CONST(Cc2662_MACCfgType, TIBMSCFG_CONST) zMACInfoDev_11 = { { 0xF3, 0x6A, 0xCE, 0x4D, 0x71, 0xD8, 0xFF, 0xFF },  11 };        // MAC ID of Device 11

CONST(Cc2662_DfuMACCfgType, TIBMSCFG_CONST) zMACInfoDfu_0 = { { 0xC5, 0x6A, 0xCE, 0x4D, 0x71, 0xD8, 0xFF, 0xFF },  0 };        // DFU MAC ID of Device 0
CONST(Cc2662_DfuMACCfgType, TIBMSCFG_CONST) zMACInfoDfu_1 = { { 0x18, 0x6B, 0xCE, 0x4D, 0x71, 0xD8, 0xFF, 0xFF },  1 };        // DFU MAC ID of Device 1

CONST(Cc2662_MACCfgType, TIBMSCFG_CONST) *zDevMACInfo_0[] =
{
     &zMACInfoDev_0,                                            /* MAC ID of Device 0 */
     &zMACInfoDev_1,                                            /* MAC ID of Device 1 */
     &zMACInfoDev_2,                                            /* MAC ID of Device 2 */
     &zMACInfoDev_3,                                            /* MAC ID of Device 3 */
     &zMACInfoDev_4,                                            /* MAC ID of Device 4 */
     &zMACInfoDev_5,                                            /* MAC ID of Device 5 */
     &zMACInfoDev_6,                                            /* MAC ID of Device 6 */
     &zMACInfoDev_7,                                            /* MAC ID of Device 7 */
     &zMACInfoDev_8,                                            /* MAC ID of Device 8 */
     &zMACInfoDev_9,                                            /* MAC ID of Device 9 */
     &zMACInfoDev_10,                                           /* MAC ID of Device 10 */
     &zMACInfoDev_11                                            /* MAC ID of Device 11 */
};

CONST(Cc2662_DfuMACCfgType, TIBMSCFG_CONST) *zDfuMACInfo_0[] =
{
    &zMACInfoDfu_0,                                             /* Primary DFU   */
    &zMACInfoDfu_1                                              /* Secondary DFU */
};

CONST(Cc2662_NwCfgType, TIBMSCFG_CONST) zCc2662_NwConfig_0 =
{
    &zDfuMACInfo_0[0],                                          /* MAC ID Table         */
    &zDevMACInfo_0[0],                                          /* MAC ID Table         */
    (sizeof(zDfuMACInfo_0))/sizeof(*zDfuMACInfo_0),             /* MAC Table Size       */
    (sizeof(zDevMACInfo_0))/sizeof(*zDevMACInfo_0),             /* MAC Table Size       */

#if(WBMS_SPI_ENABLE == 1)
    128,                                                        /* uNwPktLength         */
#else
    256,
#endif

    0xDDDD,                                                     /* wWbmsNwId            */
    CC2662_USER_CFG_T_SLOT_DL_TIME,                             /* wDownlinkTime        */
    CC2662_MAC_NwkOpMode_Volatile,                              /* uNwOpMode            */
    CC2662_SELECTIVE,                                           /* uNwJoinMode          */
    CC2662_USER_CFG_T_SLOT_UL_TIME,                             /* uUplinkTime          */
    CC2662_USER_CFG_MAX_PKT_TX_RETRIES,                         /* uMaxRetries          */
    CC2662_USER_CFG_KEEP_ALIVE_INT,                             /* uKeepAliveInt        */
    CC2662_USER_CFG_T_SLOT_UL2DL_TIME,                          /* uUltoDlTime          */
    CC2662_NUM_OF_SKIP_ALIV_INT,                                /* uNumofSkipAlivInt    */
    CC2662_USER_CFG_DENYLIST,                                   /* uDenylist            */
	1,                                                          /* uDualWmMode          */
	0,                                                          /* uDualWmType          */
#if(WBMS_SPI_ENABLE == 1)
    COMIF_SPI,                                                  /* uProtoSByte          */
#else
    COMIF_UART,
#endif
	2000,                                                        /* DevDisc_timeout     */
    2000,                                                        /* RePair_timeout      */
    2000,                                                        /* DualNwStart_timeout */
	USER_CFG_wUID,                                               /* wUID                */

    0x10U,                                                       /* wdStorageCmd        */
    1U,                                                          /* wdStorageMode       */
    100,                                                         /* wdBackOffTime       */
    3000000,                                                     /* wdScanTimeOut       */
	{ 0u, 0u, 0u }                                               /* uRsvd               */
};

STATIC CONST(Cc2662_ConfigType, TIBMSCFG_CONST) zCc2662Cfg_0 =
{
    &zBmiBswCfg_2,                                              /* pBswCfg              */
	&zCc2662_NwConfig_0,				                        /* pWbmsNwCfg           */

    zReqDataMem_0,                                              /* pCmdDataMemCfg       */
    zNpiTpBufMem_0,                                             /* pNpiTpBufMemCfg      */

    pCmdQueFreeMem_0,                                           /* pCmdQueFreeMemCfg    */
    sizeof(Cc2662_ReqDataType),                                 /* wCmdQueSizeCfg       */
    CC2662_CMD_QUEUE_LEN_CHAIN_0,                               /* wCmdQueCntCfg        */

    pCmdQueExecMem_0,                                           /* pCmdQueExecMemCfg    */
    sizeof(Cc2662_ReqDataType),                                 /* wExecQueSizeCfg      */
    CC2662_EXEC_QUEUE_LEN_CHAIN_0,                              /* wCmdQueCntCfg        */

    pReadQueExecMem_0,                                          /* pReadQueExecMemCfg   */
    sizeof(void *),                                             /* wReadQueSizeCfg      */
    CC2662_READ_QUEUE_LEN_CHAIN_0,                              /* wReadQueCntCfg       */

    (TIBMS_MAX_AFES_IN_IFACE_2*256U),                            /* wBqNpiTpBufLenCfg    */
    0u,                                                         /* uCmdIfaceCfg         */
    { 0u, 0u, 0u }                                              /* uRsvd                */
};

STATIC CONST(Comif_OperationType, TIBMSCFG_CONST) zCc2662OpsCfg =
{
    Cc2662_Init,
    Cc2662_HandleTransfer,
    Cc2662_HandleRecieve,
    Cc2662_Notify,
    Cc2662_Control,
    Cc2662_Deinit
};

/*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

CONST(Comif_ConfigType, TIBMSCFG_CONST) zBmiComifCfgWbmsCfg_0 =
{
    (void*) &zCc2662Cfg_0,                                      /* pComChipCfg      */
    &zCc2662OpsCfg,                                             /* pComifOpsCfg     */
    zBmiComCfg_2,                                               /* pCommCfg         */

    2u,                                                         /* uComIfaceCfg     */
    COMIF_WIRELESS,                                             /* eComifTypeCfg    */
    COMIF_CC2662_CHIP,                                          /* eComifChipCfg    */

#if(WBMS_SPI_ENABLE == 1)
    COMIF_SPI,                                                  /* ePeripCfg        */
#else
    COMIF_UART,
#endif

    0u,                                                         /* uNfaultPinCfg    */
    { 0u, 0u, 0u }                                              /* uRsvd            */
};

#define CC2662_PBCFG_0_STOP_SEC_CONST
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * Local Function Prototypes
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Functions Definition
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * External Functions Definition
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * End of File: cc2662_PBcfg_0.c
 *********************************************************************************************************************/
