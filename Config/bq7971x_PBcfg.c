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
 *  File:       bq7971x_PBcfg.c
 *  Project:    TIBMS
 *  Module:     BMI
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Post Build configuration for BQ7971x
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
#include "tibms_bmi.h"

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

/**	Major Software Config C Version number */
#define BQ7971X_CFG_C_MAJOR_VERSION             (0x01u)

/**	Minor Software Config C Version number */
#define BQ7971X_CFG_C_MINOR_VERSION             (0x00u)

/** Software Patch Config C Version number */
#define BQ7971X_CFG_C_PATCH_VERSION             (0x00u)

#if ((BQ7971X_CFG_C_MAJOR_VERSION != BQ7971X_CFG_MAJOR_VERSION) || \
     (BQ7971X_CFG_C_MINOR_VERSION != BQ7971X_CFG_MINOR_VERSION) || \
	 (BQ7971X_CFG_C_PATCH_VERSION != BQ7971X_CFG_PATCH_VERSION))
#error "tibms: Config version numbers of bq7971x_PBcfg_0.c and bq7971x_cfg.h are inconsistent!"
#endif

/*********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

#define BQ7971X_DIAG_CMD_QUEUE_LEN_CHAIN_0      (50u)
#define BQ7971X_DIAG_CMD_QUEUE_LEN_CHAIN_1      (50u)

/*********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

 /*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

#define BQ7971X_PBCFG_0_START_SEC_CONST
#include "Cdd_MemMap.h"

static Bq7971x_DiagReqType zDiagReqDataMem_0[BQ7971X_DIAG_CMD_QUEUE_LEN_CHAIN_0 + 1U];
void *pDiagCmdQueFreeMem_0[(BQ7971X_DIAG_CMD_QUEUE_LEN_CHAIN_0 + 1U)];
void *pDiagCmdQueExecMem_0[(BQ7971X_DIAG_CMD_QUEUE_LEN_CHAIN_0 + 1U)];

static Bq7971x_DiagReqType zDiagReqDataMem_1[BQ7971X_DIAG_CMD_QUEUE_LEN_CHAIN_1 + 1U];
void *pDiagCmdQueFreeMem_1[(BQ7971X_DIAG_CMD_QUEUE_LEN_CHAIN_1 + 1U)];
void *pDiagCmdQueExecMem_1[(BQ7971X_DIAG_CMD_QUEUE_LEN_CHAIN_1 + 1U)];

STATIC CONST(Bq7971x_DevCfgType, TIBMSCFG_CONST) zDevRegsCfg_0[TIBMS_MAX_BMICS_IN_IFACE_0] =
{
    {
        0x3FFFF, 0x07F, 0u,                                                         /** Cells/GPIO/BB configuration in BMIC **/
        { 0x2Cu, (BQ7971X_CELL_NUM_ACT - 6U) },                                                           /** Cell configuraiton in BMIC **/
        { 0x0u, 0x9u, 0x9u, 0x9u, 0x9u, 0x01u },                               /** GPIO Configurataion **/
        { 0x00u, 0x00u, 0x40u },                                                    /** BB Configuraiton **/
        { 0x00u, 0x00u }
    },
    {
        0x3FFFF, 0x07F, 0u,
        { 0x2Cu, (BQ7971X_CELL_NUM_ACT - 6U) },
        { 0x0u, 0x9u, 0x9u, 0x9u, 0x9u, 0x01u }, 
        { 0x00u, 0x00u, 0x40u },
        { 0x00u, 0x00u }
    },
    {
        0x3FFFF, 0x7FC, 0u,
        { 0x2Cu, (BQ7971X_CELL_NUM_ACT - 6U) },
        { 0x00u, 0x12u, 0x00u, 0x00u, 0x12u, 0x02u },
        { 0x00u, 0x00u, 0x40u },
        { 0x00u, 0x00u }
    },
    {
        0x3FFFF, 0x7FC, 0u,
        { 0x2Cu, (BQ7971X_CELL_NUM_ACT - 6U) },
        { 0x00u, 0x12u, 0x00u, 0x00u, 0x12u, 0x02u },
        { 0x00u, 0x00u, 0x40u },
        { 0x00u, 0x00u }
    },
    {
        0x3FFFF, 0x7FC, 0u,
        { 0x2Cu, (BQ7971X_CELL_NUM_ACT - 6U) },
        { 0x00u, 0x12u, 0x00u, 0x00u, 0x12u, 0x02u },
        { 0x00u, 0x00u, 0x40u },
        { 0x00u, 0x00u }
    },
    {
        0x3FFFF, 0x7FC, 0u,
        { 0x2Cu, (BQ7971X_CELL_NUM_ACT - 6U) },
        { 0x00u, 0x12u, 0x00u, 0x00u, 0x12u, 0x02u },
        { 0x00u, 0x00u, 0x40u },
        { 0x00u, 0x00u }
    },
    {
        0x3FFFF, 0x7FC, 0u,
        { 0x2Cu, (BQ7971X_CELL_NUM_ACT - 6U) },
        { 0x00u, 0x12u, 0x00u, 0x00u, 0x12u, 0x02u },
        { 0x00u, 0x00u, 0x40u },
        { 0x00u, 0x00u }
    },
    {
        0x3FFFF, 0x7FC, 0u,
        { 0x2Cu, (BQ7971X_CELL_NUM_ACT - 6U) },
        { 0x00u, 0x12u, 0x00u, 0x00u, 0x12u, 0x02u },
        { 0x00u, 0x00u, 0x40u },
        { 0x00u, 0x00u }
    },
    {
        0x3FFFF, 0x7FC, 0u,
        { 0x2Cu, (BQ7971X_CELL_NUM_ACT - 6U) },
        { 0x00u, 0x12u, 0x00u, 0x00u, 0x12u, 0x02u },
        { 0x00u, 0x00u, 0x40u },
        { 0x00u, 0x00u }
    },
    {
        0x3FFFF, 0x7FC, 0u,
        { 0x2Cu, (BQ7971X_CELL_NUM_ACT - 6U) },
        { 0x00u, 0x12u, 0x00u, 0x00u, 0x12u, 0x02u },
        { 0x00u, 0x00u, 0x40u },
        { 0x00u, 0x00u }
    },
    {
        0x3FFFF, 0x7FC, 0u,
        { 0x2Cu, (BQ7971X_CELL_NUM_ACT - 6U) },
        { 0x00u, 0x12u, 0x00u, 0x00u, 0x12u, 0x02u },
        { 0x00u, 0x00u, 0x40u },
        { 0x00u, 0x00u }
    },
    {
        0x3FFFF, 0x7FC, 0u,
        { 0x2Cu, (BQ7971X_CELL_NUM_ACT - 6U) },
        { 0x00u, 0x12u, 0x00u, 0x00u, 0x12u, 0x02u },
        { 0x00u, 0x00u, 0x40u },
        { 0x00u, 0x00u }
    }
};

STATIC CONST(Bq7971x_RegsType, TIBMSCFG_CONST) rcBq7971xRegsCfg_0 =
{
    /* NVM_Regs */
    {
        {
            BQ7971X_DEV_CONF1_POR_VAL,                              /* uDev_Conf */
            BQ7971X_CELL_NUM_ACT - 6U
        },
        BQ7971X_COMM_CONF_POR_VAL,                                  /* uComm_Conf */
        {
            BQ7971X_BBVC_POSN1_POR_VAL,                             /* uBBVC_Posn */
            BQ7971X_BBVC_POSN2_POR_VAL,
            0x40u
        },
        BQ7971X_ADC_CONF_POR_VAL,                                   /* uAdc_Conf */
        {
            BQ7971X_OV_THRESH_POR_VAL,                              /* uOVUV_Thresh */
            BQ7971X_UV_THRESH_POR_VAL
        },
        BQ7971X_OTUT_THRESH_POR_VAL,                                /* uOTUT_Thresh */
        {
            BQ7971X_UV_DISABLE1_POR_VAL,                            /* uUV_Disable */
            BQ7971X_UV_DISABLE2_POR_VAL,
            0x40u
        },
        {
            BQ7971X_FAULT_MSK1_POR_VAL,                             /* uFault_Msk */
            BQ7971X_FAULT_MSK2_POR_VAL
        },
        {
            BQ7971X_CS_ADC_CAL1_POR_VAL,                            /* uCS_Adc_Cal */
            BQ7971X_CS_ADC_CAL2_POR_VAL
        },
        {
            BQ7971X_OC_CONF1_POR_VAL,                               /* uOC_Conf */
            BQ7971X_OC_CONF2_POR_VAL
        },
        BQ7971X_IDDQ_CONF_POR_VAL,                                  /* uIddq_Conf */
        {
            //2,1   4,3    6,5    8,7    10,9   0,11
            0x09u, 0x09u, 0x09u, 0x09u, 0x09u, 0x01u                /* uGpio_Conf */
        },
        {
            BQ7971X_VCELL_OFFSET1_POR_VAL,                          /* uVcell_Offset */
            BQ7971X_VCELL_OFFSET2_POR_VAL,
            BQ7971X_VCELL_OFFSET3_POR_VAL,
            BQ7971X_VCELL_OFFSET4_POR_VAL,
            BQ7971X_VCELL_OFFSET5_POR_VAL,
            BQ7971X_VCELL_OFFSET6_POR_VAL,
            BQ7971X_VCELL_OFFSET7_POR_VAL,
            BQ7971X_VCELL_OFFSET8_POR_VAL,
            BQ7971X_VCELL_OFFSET9_POR_VAL
        },
    },
    /* Ctrl_Regs */
    {
        {
            BQ7971X_CONTROL1_POR_VAL,                               /* uControl */
            BQ7971X_CONTROL2_POR_VAL
        },
        {
            BQ7971X_ADC_CTRL1_POR_VAL,                              /* uAdc_Ctrl */
            0x2Du,
            BQ7971X_ADC_CTRL3_POR_VAL,
            BQ7971X_ADC_CTRL4_POR_VAL
        },
        {
            0x1Fu,   /* VCELL_THR 32 mV. */                         /* uDiag_Adc_Ctrl */
            0x07u,   /* GPIO_THR 8 mV (absolute) or 0.200% (ratiometric). */
            BQ7971X_DIAG_ADC_CTRL3_POR_VAL
        },
        {
            BQ7971X_DIAG_MISC_CTRL1_POR_VAL,                        /* uDiag_Misc_Ctrl */
            BQ7971X_DIAG_MISC_CTRL2_POR_VAL,
        },
        {
            BQ7971X_CB_CELL18_CTRL_POR_VAL,                         /* uCB_Cell_Ctrl */
            BQ7971X_CB_CELL17_CTRL_POR_VAL,
            BQ7971X_CB_CELL16_CTRL_POR_VAL,
            BQ7971X_CB_CELL15_CTRL_POR_VAL,
            BQ7971X_CB_CELL14_CTRL_POR_VAL,
            BQ7971X_CB_CELL13_CTRL_POR_VAL,
            BQ7971X_CB_CELL12_CTRL_POR_VAL,
            BQ7971X_CB_CELL11_CTRL_POR_VAL,
            BQ7971X_CB_CELL10_CTRL_POR_VAL,
            BQ7971X_CB_CELL9_CTRL_POR_VAL,
            BQ7971X_CB_CELL8_CTRL_POR_VAL,
            BQ7971X_CB_CELL7_CTRL_POR_VAL,
            BQ7971X_CB_CELL6_CTRL_POR_VAL,
            BQ7971X_CB_CELL5_CTRL_POR_VAL,
            BQ7971X_CB_CELL4_CTRL_POR_VAL,
            BQ7971X_CB_CELL3_CTRL_POR_VAL,
            BQ7971X_CB_CELL2_CTRL_POR_VAL,
            BQ7971X_CB_CELL1_CTRL_POR_VAL
        },
        BQ7971X_VCBDONE_THRESH_POR_VAL,                             /* uVCBDone_Thresh */
        BQ7971X_OTCB_THRESH_POR_VAL,                                /* uOTCB_Thresh */
        {
            BQ7971X_OVUV_CTRL1_POR_VAL,                             /* uOVUV_Ctrl */
            3u
        },
        {
            BQ7971X_OTUT_CTRL1_POR_VAL,                             /* uOTUT_Ctrl */
            3u
        },
        {
            BQ7971X_BAL_CTRL1_POR_VAL,                              /* uBal_Ctrl */
            BQ7971X_BAL_CTRL2_POR_VAL,
            BQ7971X_BAL_CTRL3_POR_VAL
        },
        {
            BQ7971X_FAULT_RST1_POR_VAL,                             /* uFault_RST */
            BQ7971X_FAULT_RST2_POR_VAL
        },
        BQ7971X_OC_CTRL_POR_VAL,                                    /* uOC_Ctrl */
        BQ7971X_I2C_WR_DATA_POR_VAL,                                /* uI2C_WR_Data */
        BQ7971X_I2C_CTRL_POR_VAL,                                   /* uI2C_Ctrl */
        BQ7971X_SPI_CONF_POR_VAL,                                   /* uSpi_Conf */
        {
            BQ7971X_SPI_TX3_POR_VAL,                                /* uSpi_TX */
            BQ7971X_SPI_TX2_POR_VAL,
            BQ7971X_SPI_TX1_POR_VAL
        },
        BQ7971X_SPI_EXE_POR_VAL                                     /* uSpi_Exe */
    },

    zDevRegsCfg_0,                                                  /* pDevRegCfg */
    FALSE,                                                          /* uCellCfgPerDevInit - Neede only if per BMIC configurtaiton */
    FALSE,                                                          /* uGpioCfgPerDevInit - Neede only if per BMIC configurtaiton */
};

STATIC CONST(Bq7971x_ConfigType, TIBMSCFG_CONST) zBq7971xCfg_0 =
{
    &rcBq7971xRegsCfg_0,                /* Register Configuration */

    zDiagReqDataMem_0,                  /* pDiagReqDataMemCfg */
    pDiagCmdQueFreeMem_0,               /* pDiagReqDataMemCfg */
    pDiagCmdQueExecMem_0,               /* pDiagCmdQueFreeMemCfg */

    sizeof(Bq7971x_DiagReqType),        /* uDiagCmdQueSizeCfg */
    BQ7971X_DIAG_CMD_QUEUE_LEN_CHAIN_0, /* uDiagCmdQueCntCfg */
};

STATIC CONST(Bq7971x_ConfigType, TIBMSCFG_CONST) zBq7971xCfg_1 =
{
    &rcBq7971xRegsCfg_0,                /* Register Configuration */

    zDiagReqDataMem_1,                  /* pDiagReqDataMemCfg */
    pDiagCmdQueFreeMem_1,               /* pDiagReqDataMemCfg */
    pDiagCmdQueExecMem_1,               /* pDiagCmdQueFreeMemCfg */

    sizeof(Bq7971x_DiagReqType),        /* uDiagCmdQueSizeCfg */
    BQ7971X_DIAG_CMD_QUEUE_LEN_CHAIN_1, /* uDiagCmdQueCntCfg */
};


STATIC CONST(Bmi_OperationType, TIBMSCFG_CONST) zBq7971xOpsCfg =
{
    Bq7971x_Init,
    Bq7971x_TxHandlerProc,
    Bq7971x_RxHandlerProc,
    Bq7971x_ReceiveProc,
    Bq7971x_DataDecode,
    Bq7971x_ServiceProc,
    Bq7971x_NotifyProc,
    Bq7971x_ControlProc,
    Bq7971x_DeInit
};

CONST(Bmi_ConfigType, TIBMSCFG_CONST) zBmiCfg_0 =
{
    (void *) &zBq7971xCfg_0,                        /* pBmcCfg      */
    &zBmiBswCfg_0,                                  /* pBswCfg      */
    &zBq7971xOpsCfg,                                /* pBmcOpsCfg   */
    zBmiFuncCfg_0,                                  /* pFuncCfg     */
    zBmiCellBalCfg_0,                               /* pCellBalCfg  */
    zBmiDiagCfg_0,                                  /* pDiagCfg     */

    BMI_BQ79718_CHIP                                /* eBmiChip     */
};

CONST(Bmi_ConfigType, TIBMSCFG_CONST) zBmiCfg_1 =
{
    (void *) &zBq7971xCfg_1,                        /* pBmcCfg      */
    &zBmiBswCfg_2,                                  /* pBswCfg      */
    &zBq7971xOpsCfg,                                /* pBmcOpsCfg   */
    zBmiFuncCfg_2,                                  /* pFuncCfg     */
    zBmiCellBalCfg_2,                               /* pCellBalCfg  */
    zBmiDiagCfg_2,                                  /* pDiagCfg     */

    BMI_BQ79718_CHIP                                /* eBmiChip     */
};

#define BQ7971X_PBCFG_0_STOP_SEC_CONST
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Local CONST Object Definitions
 *********************************************************************************************************************/

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
 * End of File: bq7971x_PBcfg_0.c
 *********************************************************************************************************************/
