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
 *  File:       bq79600_PBcfg_0.c
 *  Project:    TIBMS
 *  Module:     COMIF
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Post Build configuration for BQ79600
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05May2022    SEM                0000000000000    Initial version
 *
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
#define BQ79600_CFG_C_MAJOR_VERSION             (0x01u)

/**	Minor Software Config C Version number */
#define BQ79600_CFG_C_MINOR_VERSION             (0x00u)

/** Software Patch Config C Version number */
#define BQ79600_CFG_C_PATCH_VERSION             (0x01u)

#if ((BQ79600_CFG_C_MAJOR_VERSION != BQ79600_CFG_MAJOR_VERSION) || \
     (BQ79600_CFG_C_MINOR_VERSION != BQ79600_CFG_MINOR_VERSION) || \
	 (BQ79600_CFG_C_PATCH_VERSION != BQ79600_CFG_PATCH_VERSION))
#error "tibms: Config version numbers of bq79600_PBcfg_0.c and bq79600_cfg.h are inconsistent!"
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

#define BQ79600_PBCFG_0_START_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

static Bq79600_ReqDataType zReqDataMem_0[BQ79600_CMD_QUEUE_LEN_CHAIN_0 + 1];
static void *pCmdQueFreeMem_0[(BQ79600_CMD_QUEUE_LEN_CHAIN_0 + 1)];
static void *pCmdQueExecMem_0[COMIF_CMD_PRIO_MAX][(BQ79600_CMD_QUEUE_LEN_CHAIN_0 + 1)];
static void *pReadQueExecMem_0[(BQ79600_READ_QUEUE_LEN_CHAIN_0 + 1)];

#define BQ79600_PBCFG_0_STOP_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 *  Local CONST Object Definitions
 *********************************************************************************************************************/

#define BQ79600_PBCFG_0_START_SEC_CONST
#include "Cdd_MemMap.h"

STATIC CONST(Bq79600_ConfigType, TIBMSCFG_CONST) zBq79600Cfg_0 =
{
    &zBmiBswCfg_0,                      /* pBswCfg              */

    zReqDataMem_0,                      /* pCmdDataMemCfg       */
    pCmdQueFreeMem_0,                   /* pCmdQueFreeMemCfg    */
    sizeof(Bq79600_ReqDataType),        /* wCmdQueSizeCfg       */
    BQ79600_CMD_QUEUE_LEN_CHAIN_0,      /* wCmdQueCntCfg        */

    pCmdQueExecMem_0,                   /* pCmdQueExecMemCfg    */
    sizeof(Bq79600_ReqDataType),        /* wExecQueSizeCfg      */
    BQ79600_EXEC_QUEUE_LEN_CHAIN_0,     /* wCmdQueCntCfg        */

    pReadQueExecMem_0,                  /* pReadQueExecMemCfg   */
    sizeof(void *),                     /* wReadQueSizeCfg      */
    BQ79600_READ_QUEUE_LEN_CHAIN_0,     /* wReadQueCntCfg       */
	
	#if(BMI_SPI_ENABLE == 1)
    COMIF_SPI,                                                  /* uPeripcfg          */
#else
    COMIF_UART,
#endif

    0u                                  /* uCmdIfaceCfg         */
};

/* PMI Bridge configuration*/
STATIC CONST(Bq79600_ConfigType, TIBMSCFG_CONST) zBq79600Cfg_1 =
{
    &zPmiBswCfg_0,                      /* pBswCfg              */

    zReqDataMem_0,                      /* pCmdDataMemCfg       */
    pCmdQueFreeMem_0,                   /* pCmdQueFreeMemCfg    */
    sizeof(Bq79600_ReqDataType),        /* wCmdQueSizeCfg       */
    BQ79600_CMD_QUEUE_LEN_CHAIN_0,      /* wCmdQueCntCfg        */

    pCmdQueExecMem_0,                   /* pCmdQueExecMemCfg    */
    sizeof(Bq79600_ReqDataType),        /* wExecQueSizeCfg      */
    BQ79600_EXEC_QUEUE_LEN_CHAIN_0,     /* wCmdQueCntCfg        */

    pReadQueExecMem_0,                  /* pReadQueExecMemCfg   */
    sizeof(void *),                     /* wReadQueSizeCfg      */
    BQ79600_READ_QUEUE_LEN_CHAIN_0,     /* wReadQueCntCfg       */

    0u                                  /* uCmdIfaceCfg         */
};

STATIC CONST(Comif_OperationType, TIBMSCFG_CONST) zBq79600OpsCfg_0 =
{
    Bq79600_Init,
    Bq79600_HandleTransfer,
    Bq79600_HandleRecieve,
    Bq79600_Notify,
    Bq79600_Control,
    Bq79600_Deinit
};

/*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

CONST(Comif_ConfigType, TIBMSCFG_CONST) zBmiComifCfgWiredCfg_0 =
{
    (const void*) &zBq79600Cfg_0,               /* pComChipCfg      */
    &zBq79600OpsCfg_0,                          /* pComifOpsCfg     */
    zBmiComCfg_0,                               /* pCommCfg         */

    0u,                                         /* uComIfaceCfg     */
    COMIF_WIRED,                                /* eComifTypeCfg    */
    COMIF_BQ79600_CHIP,                         /* eComifChipCfg    */
#if(BMI_SPI_ENABLE == 1)
    COMIF_SPI,                                                  /* uPeripcfg          */
#else
    COMIF_UART,
#endif
    0u,                                         /* uNfaultPinCfg    */
    0u,                                         /* uManualModeCfg   */
    0u,                                         /* uStartupCmdCfg   */
    0u                                          /* uRsvd            */
};

/* PMI Bridge configuration*/
CONST(Comif_ConfigType, TIBMSCFG_CONST) zPmiComifCfgWiredBridgeSpiCfg =
{
    (const void*) &zBq79600Cfg_1,               /* pComChipCfg      */
    &zBq79600OpsCfg_0,                          /* pComifOpsCfg     */
    zPmiComCfg_0,                               /* pCommCfg         */

    0u,                                         /* uComIfaceCfg     */
    COMIF_WIRED,                                /* eComifTypeCfg    */
    COMIF_BQ79600_CHIP,                         /* eComifChipCfg    */
    COMIF_SPI,                                  /* ePeripCfg        */
    0u,                                         /* uNfaultPinCfg    */
    0u,                                         /* uManualModeCfg   */
    0u,                                         /* uStartupCmdCfg   */
    0u                                          /* uRsvd            */
};

#define BQ79600_PBCFG_0_STOP_SEC_CONST
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
 * End of File: bq79600_PBcfg.c
 *********************************************************************************************************************/
