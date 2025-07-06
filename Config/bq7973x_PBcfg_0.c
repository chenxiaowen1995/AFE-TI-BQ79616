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
 *  File:       bq7973x_PBcfg_0.c
 *  Project:    TIBMS
 *  Module:     PMI
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Post Build configuration for PMI
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
#include "tibms_pmi.h"

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

/**	Major Software Config C Version number */
#define BQ7973X_CFG_C_MAJOR_VERSION             (0x01u)

/**	Minor Software Config C Version number */
#define BQ7973X_CFG_C_MINOR_VERSION             (0x00u)

/** Software Patch Config C Version number */
#define BQ7973X_CFG_C_PATCH_VERSION             (0x00u)

#if ((BQ7973X_CFG_C_MAJOR_VERSION != BQ7973X_CFG_MAJOR_VERSION) || \
     (BQ7973X_CFG_C_MINOR_VERSION != BQ7973X_CFG_MINOR_VERSION) || \
	 (BQ7973X_CFG_C_PATCH_VERSION != BQ7973X_CFG_PATCH_VERSION))
#error "tibms: Config version numbers of bq7973x_PBcfg_0.c and bq7973x_cfg.h are inconsistent!"
#endif


/*********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

#define BQ7973X_CMD_QUEUE_LEN_CHAIN_0           (127u)
#define BQ7973X_READ_QUEUE_LEN_CHAIN_0          (20u)
static Bq7973x_ReqDataType zReqDataMem_0[BQ7973X_CMD_QUEUE_LEN_CHAIN_0 + 1];
static void *pCmdQueFreeMem_0[(BQ7973X_CMD_QUEUE_LEN_CHAIN_0 + 1)];
static void *pCmdQueExecMem_0[(BQ7973X_CMD_QUEUE_LEN_CHAIN_0 + 1)];
static void *pReadQueExecMem_0[(BQ7973X_READ_QUEUE_LEN_CHAIN_0 + 1)];


#define BQ7973X_DIAG_CMD_QUEUE_LEN_CHAIN_0      (70u)
static Bq7973x_DiagReqType zPmiDiagReqDataMem_0[BQ7973X_DIAG_CMD_QUEUE_LEN_CHAIN_0 + 1];
static void *pPmiDiagCmdQueFreeMem_0[(BQ7973X_DIAG_CMD_QUEUE_LEN_CHAIN_0 + 1)];
static void *pPmiDiagCmdQueExecMem_0[(BQ7973X_DIAG_CMD_QUEUE_LEN_CHAIN_0 + 1)];
/*********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Local CONST Object Definitions
 *********************************************************************************************************************/

/** BQ7973X Communication Configuration **/

#define BQ7973X_PBCFG_0_START_SEC_CONST
#include "Cdd_MemMap.h"
STATIC CONST(Bq7973x_ComConfigType, TIBMSCFG_CONST) zBq7973xComCfg_0 =
{
    &zPmiBswCfg_0,                                      /* pBswCfg              */
    zReqDataMem_0,                                      /* pCmdDataMemCfg       */

    pCmdQueFreeMem_0,                                   /* pCmdQueFreeMemCfg    */
    pCmdQueExecMem_0,                                   /* pCmdQueExecMemCfg    */
    pReadQueExecMem_0,                                  /* pReadQueExecMemCfg   */

    sizeof(Bq7973x_ReqDataType),                        /* wCmdQueSizeCfg       */
    BQ7973X_CMD_QUEUE_LEN_CHAIN_0,                      /* wCmdQueCntCfg        */

    sizeof(void *),                                     /* wReadQueSizeCfg      */
    BQ7973X_READ_QUEUE_LEN_CHAIN_0,                     /* wReadQueCntCfg       */

    0u                                                  /* uCmdIfaceCfg         */
};
/** Bq7973X Functional Configuration **/
STATIC CONST(Bq7973x_RegsType, TIBMSCFG_CONST) rcBq7973xRegsCfg =
{
    {
        {
            BQ7973X_DEV_CONF1_POR_VAL,
            BQ7973X_DEV_CONF2_POR_VAL
        },  /* uDev_Conf */
        BQ7973X_COMM_CONF_POR_VAL,  /* uComm_Conf */
        BQ7973X_ADC_CONF_POR_VAL,  /* uAdc_Conf */
        {
            BQ7973X_FAULT_MSK1_POR_VAL,
            BQ7973X_FAULT_MSK2_POR_VAL,
            BQ7973X_FAULT_MSK3_POR_VAL
        },  /* uFault_Msk */
        {
            BQ7973X_CS_ADC1_CAL1_POR_VAL,
            BQ7973X_CS_ADC1_CAL2_POR_VAL
        },  /* uCS_Adc1_Cal */
        {
            0x12u, 0x12u, 0x12u, 0x12u, 0x12u, 0x12u, 0x12u, 0x12u
        },  /* uGpio_Conf */
        {
            BQ7973X_CS_ADC2_CAL1_POR_VAL,
            BQ7973X_CS_ADC2_CAL2_POR_VAL
        },  /* uCS_Adc2_Cal */
        {
            0u, 1u, 0u, 1u, 0u, 1u, 0u, 1u
        },  /* uOC_Conf */
        BQ7973X_MPSI_CONF_POR_VAL,  /* uMspi_Conf */
    },
    {
        1u,  /* uControl2 */
        {
            6u,
            5u,
            BQ7973X_ADC_CTRL3_POR_VAL,
            BQ7973X_ADC_CTRL4_POR_VAL
        },  /* uAdc_Ctrl */
        {
            BQ7973X_DIAG_ADC_CTRL1_POR_VAL,
            BQ7973X_DIAG_ADC_CTRL2_POR_VAL
        },  /* uDiag_Adc_Ctrl */
        {
            BQ7973X_DIAG_MISC_CTRL1_POR_VAL,
            BQ7973X_DIAG_MISC_CTRL2_POR_VAL
        },  /* uDiag_Misc_Ctrl */
        {
            BQ7973X_DIAG_OC_CTRL1_POR_VAL,
            BQ7973X_DIAG_OC_CTRL2_POR_VAL
        },  /* uDiag_OC_Ctrl */
        {
            BQ7973X_FAULT_RST1_POR_VAL,
            BQ7973X_FAULT_RST2_POR_VAL,
            BQ7973X_FAULT_RST3_POR_VAL
        },  /* uFault_RST */
        {
            BQ7973X_OC_CTRL1_POR_VAL,
            BQ7973X_OC_CTRL2_POR_VAL
        },  /* uOC_Ctrl */
        BQ7973X_I2C_WR_DATA_POR_VAL,  /* uI2C_WR_Data */
        BQ7973X_I2C_CTRL_POR_VAL,  /* uI2C_Ctrl */
        BQ7973X_MPSI_CTRL_POR_VAL,  /* uMspi_Ctrl */
        {
            BQ7973X_SPI_TX4_POR_VAL,
            BQ7973X_SPI_TX3_POR_VAL,
            BQ7973X_SPI_TX2_POR_VAL,
            BQ7973X_SPI_TX1_POR_VAL
        },  /* uMspi_Tx */
        BQ7973X_MSPI_EXE_POR_VAL,  /* uMspi_Exe */
        3u,  /* uCC_Ctrl */
        BQ7973X_SW_CTRL_POR_VAL,  /* uSW_Ctrl */
        {
            BQ7973X_GPIO_CTRL1_POR_VAL,
            BQ7973X_GPIO_CTRL2_POR_VAL
        },  /* uGpio_Ctrl */
        BQ7973X_GPIO_PWM_CTRL_POR_VAL,  /* uGPIO_PWM_CTRL */
    }
};

STATIC CONST(Bq7973x_ConfigType, TIBMSCFG_CONST) zBq7973xCfg =
{
    &rcBq7973xRegsCfg,      /* register configurations */
    zPmiDiagReqDataMem_0,                  /* pDiagReqDataMemCfg */
        pPmiDiagCmdQueFreeMem_0,               /* pDiagReqDataMemCfg */
        pPmiDiagCmdQueExecMem_0,               /* pDiagCmdQueFreeMemCfg */

        sizeof(Bq7973x_DiagReqType),        /* uDiagCmdQueSizeCfg */
        BQ7973X_DIAG_CMD_QUEUE_LEN_CHAIN_0, /* uDiagCmdQueCntCfg */
};

STATIC CONST(Pmi_OperationType, TIBMSCFG_CONST) zBq7973xOpsCfg =
{
    Bq7973x_Init,
    NULL,
    NULL,
    Bq7973x_ReceiveProc,
    Bq7973x_DataDecode,
    Bq7973x_ServiceProc,
    NULL,
    Bq7973x_ControlProc,
    Bq7973x_DeInit
};

/*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

CONST(Pmi_ConfigType, TIBMSCFG_CONST) zPmiCfgDaisyChain =
{
    (void *) &zBq7973xCfg,                      /* pPmcCfg */
    &zPmiBswCfg_0,                              /* pBswCfg */
    &zBq7973xOpsCfg,                            /* pPmcOpsCfg */
    zPmiFuncCfg_1,                              /* pFuncCfg */
    zPmiDiagCfg_1,                              /* pDiagCfg */
    NULL,                                       /* pCommCfg */

    PMI_BQ79731_CHIP,                           /* ePmiChipCfg    */
    0u                                          /* uPmiIfaceCfg  */
};

#define BQ7973X_PBCFG_0_STOP_SEC_CONST
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
 * End of File: bq7973x_PBcfg.c
 *********************************************************************************************************************/
