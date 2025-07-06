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
 *  File:       bq7971x_cfg.h
 *  Project:    TIBMS
 *  Module:     BMI
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  BQ7971x pre-compile configuration
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

#ifndef BQ7971X_CFG_H
#define BQ7971X_CFG_H

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

#include "bq7971x_regs.h"

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

#define BQ7971X_CFG_MAJOR_VERSION               (0x01u)
#define BQ7971X_CFG_MINOR_VERSION               (0x00u)
#define BQ7971X_CFG_PATCH_VERSION               (0x00u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

#define BQ7971X_IFACES		                    (2u)
#define BQ7971X_SILICON_REV		                (BQ7971X_SILICON_REV_B1)

#define BQ7971X_CELL_NUM_MAX                    (18u)
#define BQ7971X_GPIO_NUM_MAX                    (11u)

#define BQ7971X_CELL_NUM_ACT                    (BMIAPP_NUM_OF_CELLS)
#define BQ7971X_GPIO_NUM_ACT                    (BMIAPP_NUM_OF_GPIOS)
#define BQ7971X_AFE_NUM_MAX                     (2u)
#define BQ7971X_GPIO_USED_MASK                  (5u)

#define BQ7971X_DIE_TEMP_NUM                    (2u)

#define BQ7971X_DIAG_MAX_RETRY                  (3u)
#define BQ7971X_DIAG_STAT_RETRY                 (1u)
#define BQ7971X_DIAG_INIT_RETRY                 (3u)

#define BQ7971X_GPIO_PLAU_DB_MAX                (5u)

#define BQ7971X_CELL_VOLTAGE_REG_OFFSET         (BQ7971X_VCELL18_HI_OFFSET + \
                                                ((BQ7971X_CELL_NUM_MAX - BQ7971X_CELL_NUM_ACT) * READ_2BYTE))

/** Set the timer for cell balancing
 * 0x01 = 30s
 * 0x02 = 60s
 * 0x03 = 300s
 * 0x04 = 600s
 * 0x05 to 0x0F = range from 20min to 120min in 10min steps
 * 0x10 to 0x1F = range from 150min to 600min in 30min steps
 */
#define BQ7971X_CELL_CB_TIME_DEFAULT            (1u)

#define BQ7971X_WAKEUP_PING_DELAY               (4u)   	/* 3.5ms 	*/
#define BQ7971X_WAKEUP_TONE_DELAY               (10u)  	/* 10ms 	*/
#define BQ7971X_HW_RESET_DELAY                  (45u)  	/* 45ms 	*/
#define BQ7971X_SLP2ACT_TONE_DELAY              (1u)    /* 500us 	*/
#define BQ7971X_GOTO_SLEEP_DELAY                (1u)    /* 100us 	*/
#define BQ7971X_SOFT_RESET_DELAY                (1u)   	/* 1ms 		*/
#define BQ7971X_GOTO_SHUTDOWN_DELAY             (5u)   	/* 5ms 		*/

#define BQ7971X_VAL_MAXVOLREGVALUE              (0xFFFEu)
#define BQ7971X_VAL_INVALDVOLVALUE              (0xFFFFu)

#define BQ7971X_CELLVCCBSHRT_THRESHOLD          (5000u)
#define BQ7971X_CABLE_RES_AB_THRESHOLD          (3000u)

#define BQ7971X_MAXVCELLPLAUVALUE               (48000u)
#define BQ7971X_MIDVCELLPLAUVALUE               (25000u)
#define BQ7971X_MINVCELLPLAUVALUE               (2000u)
#define BQ7971X_VCELL_THRESHHOLD                (10000u)    /* 1V */

#define BQ7971X_GPIOOPENWIRE_THRESHOLD          (10000u)    /* 1V */
#define BQ7971X_GPIOADJSHORT_THRESHOLD          (1000u)     /* 100mV */

#define BQ7971X_MAXGPIOPLAUVALUE                (40000u)   /* 4V - SEM CSU Board */
#define BQ7971X_MINGPIOPLAUVALUE                (5000u)    /* 500mv - SEM CSU Board */
#define BQ7971X_GPIOOW_PULLDET                  (20000u)   /* 2V difference to decide pull down or pull up */

#define BQ7971X_MAXBGPLAUVALUE                  (13000u)
#define BQ7971X_MINBGPLAUVALUE                  (11000u)
#define BQ7971X_MAXREFCAPPLAUVALUE              (40250u)
#define BQ7971X_MINREFCAPPLAUVALUE              (39750u)

#define BQ7971X_MAXDIETEMPPLAUVALUE             (1200u)     /* 120(degress) */
#define BQ7971X_MINDIETEMPPLAUVALUE             (65186u)    /* -35(degress) */
#define BQ7971X_TEMPDIFFPLAUVALUE               (100u)      /* 10 degrees Celsius */
#define BQ7971X_DIETEMPSTUCKVALUE               (3u)        /* 3 FDTI cycles */

#define BQ7971X_LSB_STANDARD                    (10u)
#define BQ7971X_STEP_CENTI_NTC                  (400u)

#define BQ7971X_VCCBSHRT_DELAY                  (1u)        /* Wait 1.1ms for an accurate ADC result in the result registers. */
#define BQ7971X_ACOMP_DONE_DELAY                (9u)
#define BQ7971X_DCOMP_DONE_DELAY                (100u)

#define BQ7971X_GPIO_CONF_DELAY                 (6u)        /* If (GP_DR==1) wait at least 2.8ms; else wait at least 5.5ms */

#define BQ7971X_SMOVDAC_ERR                     (1000u)
#define BQ7971X_SMUVDAC_ERR                     (1000u)
#define BQ7971X_OV_THR_RES                      (2700u)
#define BQ7971X_OV_THR_STEP                     (25u)
#define BQ7971X_OV_THR_REG_VAL1                 (0x0Du)
#define BQ7971X_OV_THR_REG_VAL2                 (0x40u)
#define BQ7971X_OV_THR_REG_VAL3                 (0x51u)
#define BQ7971X_OV_THR_REG_VAL4                 (0x80u)
#define BQ7971X_OV_THR_REG_VAL5                 (0xA6u)
#define BQ7971X_OV_THR_VAL1                     (3600u)
#define BQ7971X_OV_THR_VAL2                     (4175u)
#define BQ7971X_UV_THR_RES                      (1200u)
#define BQ7971X_UV_THR_STEP                     (50u)
#define BQ7971X_UV_THR_REG_VAL                  (0x32u)
#define BQ7971X_UV_THR_VAL                      (3700u)
#define BQ7971X_OVUV_CTRL2_OVUV_THR_LOCK        (4u)
#define BQ7971X_OVUV_CTRL2_OVUV_RR              (2u)
#define BQ7971X_OVUV_CTRL1_OV_THR               (0x00u)
#define BQ7971X_OVUV_CTRL1_UV_THR               (0x40u)

#define BQ7971X_SMOTDAC_ERR                     (400u)
#define BQ7971X_SMUTDAC_ERR                     (400u)
#define BQ7971X_OT_THR_RES                      (10u)
#define BQ7971X_OT_THR_STEP                     (1u)
#define BQ7971X_OT_THR_REG_VAL                  (0x15u)
#define BQ7971X_OT_THR_VAL                      (30u)
#define BQ7971X_UT_THR_RES                      (72u)
#define BQ7971X_UT_THR_STEP                     (2u)
#define BQ7971X_OTUT_CTRL2_OTUT_THR_LOCK        (4u)
#define BQ7971X_OTUT_CTRL2_OTUT_RR              (2u)
#define BQ7971X_OTUT_CTRL1_OT_THR               (0u)
#define BQ7971X_OTUT_CTRL1_UT_THR               (0x20u)

#define BQ7971X_DIAG_CBFET_DELAY                (20u)
#define BQ7971X_ACOMPINJ_DONE_DELAY             (17u)
#define BQ7971X_DCOMPINJ_DONE_DELAY             (100u)
#define BQ7971X_DCOMPINJ_RETRY_DELAY            (5u)
#define BQ7971X_DRDY_BIST_PWR_DELAY             (4u)
#define BQ7971X_DIAG_D1D2_DELAY                 (6u)

#define BQ7971X_INIT_RETRY_DELAY                (5u)
#define BQ7971X_INIT_DONE_DELAY                 (100u)
#define BQ7971X_ADC_GO_DELAY                    (6u)    /* GPADC => 3.3 ms if GP_DR==1, 5.5ms if GP_DR==0 */


/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef struct Bq7971x_RegsPerDevConfType_Tag
{
    uint32 uNCellsCfg;
    uint16 uNGpioCfg;
    uint16 uNBBCfg;

    uint8 uDev_Conf[BQ7971X_DEV_CONF_REG_NUM];
    uint8 uGpio_Conf[BQ7971X_GPIO_CONF_REG_NUM];
    uint8 uBBVC_Posn[3u];
    uint8 uRsvd[2];
}
Bq7971x_DevCfgType;

typedef struct Bq7971x_NVMRegs_TypeTag
{
    uint8 uDev_Conf[2u];    /* 0x002 - 0x003 */
    uint8 uComm_Conf;       /* 0x004 */
    uint8 uBBVC_Posn[3u];   /* 0x005 - 0x007 Whenever user select and set a channel in BBVC_POSN*, user shall select the same channel setting to 1 in UV_DISABLE* */
    uint8 uAdc_Conf;        /* 0x008 */
    uint8 uOVUV_Thresh[2u]; /* 0x009 - 0x00A */
    uint8 uOTUT_Thresh;     /* 0x00B */
    uint8 uUV_Disable[3u];  /* 0x00C - 0x00E */
    uint8 uFault_Msk[2u];   /* 0x00F - 0x010 */
    uint8 uCS_Adc_Cal[2u];  /* 0x011 - 0x012 */
    uint8 uOC_Conf[2u];     /* 0x013 - 0x014 */
    uint8 uIddq_Conf;       /* 0x01D */
    uint8 uGpio_Conf[BQ7971X_GPIO_CONF_REG_NUM];   /* 0x01E - 0x023 */
    uint8 uVcell_Offset[9u];/* 0x024 - 0x02C */
}
Bq7971x_NVMRegsType;

typedef struct Bq7971x_CtrlRegsType_Tag
{
    uint8 uControl[2u];       /* 0x309 - 0x30A */
    uint8 uAdc_Ctrl[BQ7971X_ADC_CTRL_REG_NUM];      /* 0x310 - 0x313 */
    uint8 uDiag_Adc_Ctrl[3u]; /* 0x314 - 0x316 */
    uint8 uDiag_Misc_Ctrl[2u];/* 0x317 - 0x318 */
    uint8 uCB_Cell_Ctrl[18u]; /* 0x320 - 0x331 */
    uint8 uVCBDone_Thresh;    /* 0x332 */
    uint8 uOTCB_Thresh;       /* 0x333 */
    uint8 uOVUV_Ctrl[2u];     /* 0x334 - 0x335 */
    uint8 uOTUT_Ctrl[2u];     /* 0x336 - 0x337 */
    uint8 uBal_Ctrl[BQ7971X_BAL_CTRL_REG_NUM];      /* 0x338 - 0x339 */
    uint8 uFault_RST[2u];     /* 0x340 - 0x341 */
    uint8 uOC_Ctrl;           /* 0x360 */
    uint8 uI2C_WR_Data;       /* 0x370 */
    uint8 uI2C_Ctrl;          /* 0x371 */
    uint8 uSpi_Conf;          /* 0x380 */
    uint8 uSpi_TX[3u];        /* 0x381 - 0x383 */
    uint8 uSpi_Exe;           /* 0x384 */
}
Bq7971x_CtrlRegsType;

typedef struct Bq7971x_RegsType_Tag
{
    Bq7971x_NVMRegsType zRegNVM;
    Bq7971x_CtrlRegsType zRegCtrl;

    const Bq7971x_DevCfgType *pDevRegCfg;
    uint16 uCellCfgPerDevInit;
    uint16 uGpioCfgPerDevInit;
}
Bq7971x_RegsType;

typedef struct Bq7971x_DiagReqType_Tag
{
    void *pUserData;
    const ServiceCfgType *pDiagSvcCfg;
    uint8 uDiagReqId;
    uint8 uDiagStep;
    uint8 uDiagStat;
    uint8 uDiagRetry;
}
Bq7971x_DiagReqType;

typedef struct Bq7971x_ConfigType_Tag
{
    const Bq7971x_RegsType *pBqRegsCfg;

    Bq7971x_DiagReqType *pDiagReqDataMemCfg;
    void *pDiagCmdQueFreeMemCfg;
    void *pDiagCmdQueExecMemCfg;

    uint16 uDiagCmdQueSizeCfg;
    uint16 uDiagCmdQueCntCfg;
}
Bq7971x_ConfigType;

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

extern CONST(Bmi_ConfigType, TIBMSCFG_CONST) zBmiCfg_0;
extern CONST(ServiceCfgType, TIBMSCFG_CONST) zBmiFuncCfg_0[];
extern CONST(ServiceCfgType, TIBMSCFG_CONST) zBmiCellBalCfg_0[];
extern CONST(ServiceCfgType, TIBMSCFG_CONST) zBmiDiagCfg_0[];
extern CONST(ServiceCfgType, TIBMSCFG_CONST) zBmiComCfg_0[];
extern CONST(Basic_ConfigType, TIBMSCFG_CONST) zBmiBswCfg_0;

extern CONST(Bmi_ConfigType, TIBMSCFG_CONST) zBmiCfg_1;
extern CONST(ServiceCfgType, TIBMSCFG_CONST) zBmiFuncCfg_2[];
extern CONST(ServiceCfgType, TIBMSCFG_CONST) zBmiCellBalCfg_2[];
extern CONST(ServiceCfgType, TIBMSCFG_CONST) zBmiDiagCfg_2[];
extern CONST(ServiceCfgType, TIBMSCFG_CONST) zBmiComCfg_2[];
extern CONST(Basic_ConfigType, TIBMSCFG_CONST) zBmiBswCfg_2;

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

#endif /*BQ7971X_CFG_H*/

/*********************************************************************************************************************
 * End of File: bq7971x_cfg.h
 *********************************************************************************************************************/
