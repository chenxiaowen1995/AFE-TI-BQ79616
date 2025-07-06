/*********************************************************************************************************************
 *  COPYRIGHT
 *  ------------------------------------------------------------------------------------------------------------------
 *  \verbatim
 *
 *   TEXAS INSTRUMENTS INCORPORATED PROPRIETARY INFORMATION
 *
 *   Property of Texas Instruments, Unauthorized reproduction and/or distribution
 *   is strictly prohibited.  This product  is  protected  under  copyright  law
 *   and  trade  secret law as an  unpublished work.
 *   (C) Copyright 2021 Texas Instruments Inc.  All rights reserved.
 *
 *  \endverbatim
 *  ------------------------------------------------------------------------------------------------------------------
 *  FILE DESCRIPTION
 *  ------------------------------------------------------------------------------------------------------------------
 *  File:       bq7973x_diag.h
 *  Project:    TIBMS
 *  Module:     DIAG
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for PMI DIAG interface
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       9Oct         Ashraf & Yehia    0000000000000    Initial version
 *
 *********************************************************************************************************************/

#ifndef BQ7973X_DIAG_H
#define BQ7973X_DIAG_H

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

#include "tibms_utils.h"
#include "bq7973x_cfg.h"
#include "bq7973x_regs.h"
/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

#define BQ7973X_DIAG_MAJOR_VERSION (0x01u)
#define BQ7973X_DIAG_MINOR_VERSION (0x00u)
#define BQ7973X_DIAG_PATCH_VERSION (0x00u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

#define PMI_SM_REG_READ_1BYTE                                   (0u)
#define PMI_SM_REG_READ_2BYTE                                   (1u)
#define PMI_SM_REG_READ_3BYTE                                   (2u)
#define PMI_SM_REG_READ_4BYTE                                   (3u)
#define PMI_SM_REG_READ_5BYTE                                   (4u)

#define PMI_DIAG_DEV_STAT                                       (PMI_SM_REG_READ_2BYTE)
#define PMI_DIAG_DIAG_STAT                                      (PMI_SM_REG_READ_2BYTE)
#define PMI_DIAG_DEV_DIAG_STAT                                  (PMI_SM_REG_READ_5BYTE)
#define PMI_DIAG_ADC_DATA_READY                                 (PMI_SM_REG_READ_1BYTE)
#define PMI_DIAG_DIAG_D1D2                                      (PMI_SM_REG_READ_2BYTE)
#define PMI_DIAG_GPIO_STAT                                      (PMI_SM_REG_READ_2BYTE)
#define PMI_DIAG_FAULT_ADC_MISC                                 (PMI_SM_REG_READ_1BYTE)


/* values taken from BMI files*/
#define BQ7973X_VAL_INVALDVOLVALUE           (0xFFFFu)

#define BQ7973X_UV_THR_RES                      (1200u)
#define BQ7973X_UV_THR_STEP                     (50u)
#define BQ7973X_UV_THR_REG_VAL                  (0x32u)
#define BQ7973X_UV_THR_VAL                      (3700u)

#define BQ7973X_OT_THR_RES                      (10u)
#define BQ7973X_OT_THR_STEP                     (1u)
#define BQ7973X_OT_THR_REG_VAL                  (0x15u)
#define BQ7973X_OT_THR_VAL                      (30u)

#define BQ7973X_UT_THR_RES                      (72u)
#define BQ7973X_UT_THR_STEP                     (2u)

#define BQ7973X_STEP_CENTI_NTC                  (400u)

#define BQ7973X_OTUT_THRESH_OFFSET                      (0x00Bu)

#define BQ7973X_OTUT_THRESH_UT_THR_POS                  (0x05u)
#define BQ7973X_OTUT_THRESH_UT_THR_MSK                  (0xe0u)

#define BQ7973X_OTUT_THRESH_OT_THR_POS                  (0x00u)
#define BQ7973X_OTUT_THRESH_OT_THR_MSK                  (0x1Fu)
#define BQ7973X_LSB_STANDARD                    (10u)
#define BQ7973X_OV_THR_REG_VAL1                 (0x0Du)
#define BQ7973X_OV_THR_REG_VAL2                 (0x40u)
#define BQ7973X_OV_THR_REG_VAL3                 (0x51u)
#define BQ7973X_OV_THR_REG_VAL4                 (0x80u)
#define BQ7973X_OV_THR_REG_VAL5                 (0xA6u)
#define BQ7973X_OV_THR_RES                      (2700u)
#define BQ7973X_OV_THR_STEP                     (25u)
#define BQ7973X_OV_THR_VAL1                     (3600u)
#define BQ7973X_OV_THR_VAL2                     (4175u)


/**********************************************************/
/*  9.1 */
#define BQ7973X_MARGIN_MODE_M1RD             (4u)
#define BQ7973X_FAULT_RST2_COMM              (0x1Fu)

#define BQ7973X_READ_1BYTE               (1u)
#define BQ7973X_READ_2BYTE               (2u)
#define BQ7973X_READ_3BYTE               (3u)
#define BQ7973X_READ_4BYTE               (4u)

#define BQ7973X_WRITE_1BYTE                  (1u)
#define BQ7973X_WRITE_2BYTE                  (2u)
#define BQ7973X_WRITE_3BYTE                  (3u)
#define BQ7973X_WRITE_4BYTE                  (4u)


#define BQ7973X_OC_UNLOCKCODE1               (0xCBu)
#define BQ7973X_OC_UNLOCKCODE2               (0xF6u)
#define BQ7973X_FAULT_OC_MSK                 (0x6Cu) /* all fault flag [OCC1], [OCD1], [OCC2] and [OCD2] be set */


#define BQ7973X_VF_NUM_MAX                   (2u)

#define BQ7973X_DIO_NFAULT_PIN               DioChannel_N2HET1_22
#define BQ7973X_MAXDIETEMPPLAUVALUE          (1200u)    /* 120(degress) */
#define BQ7973X_MINDIETEMPPLAUVALUE          (0xFEA2u)  /* -35(degress) */
#define BQ7973X_INVALDTEMPVALUE              (0x8000u)   /* Invalid temperature value  */
#define BQ7973X_VFPLAU_CHECKTIMES            (1u)
#define BQ7973X_GPADC_DELAY                  /*(8192u)*/ (9u)
#define BQ7973X_DIAG_D1D2_DELAY              (9u)
#define BQ7973X_INSUL_DET_DELAY              (1u)
#define BQ7973X_DCOMP_DONE_DELAY             (36u)
#define BQ7973X_ACOMP_DONE_DELAY             (9u)
#define BQ7973X_VF_OPEN_DELAY                (1u)   /* depend on the VF plausibility input voltage limits and */
#define BQ7973X_OCPLAU_CHECKTIMES            (1u)
#define BQ7973X_DIAGNOERROR                  (0u)
#define BQ7973X_DIAGERROR                    (1u)
#define BQ7973X_DIAG_D1D2_SEL_BG             (4u)
#define BQ7973X_DIAG_D1D2_SEL_AVDD           (6u)
#define BQ7973X_DIAG_D1D2_SEL_CS1VREF        (6u)
#define BQ7973X_DIETEMP_CHECKTIMES           (1u)
#define BQ7973X_NFAULT_DIAG_RST1             (0x83u)
#define BQ7973X_NFAULT_DIAG_RST2             (0xFFu)
#define BQ7973X_DIETEMPSTUCKVALUE            (3u)        /* 3 FDTI cycles */

#define BQ7973X_GPOPENWIRE_THRESHOLD         (10000u)

#define BQ7973X_GPIOSETADCONLY               (0x02u)
#define BQ7973X_GPIOSETOUTLOW                (0x05u)     /* the capacitance of the external resistor network */


#define BQ7973X_GPIO_CONF1_6_REG_NUM         (6u)
#define BQ7973X_GPIO_CONF7_8_REG_NUM         (2u)


#define BQ7973X_GPIO_CONF1_GPIO1_FLT_EN_MSK  (0x80u)
#define BQ7973X_GPIO_CONF2_CS_RDY_EN_MSK     (0x40u)

#define BQ7973X_GPIO_NUM_ACT                 (15u)
#define BQ7973X_OC_NUM_ACT                   (2u)
#define BQ7973X_CS_NUM_ACT                   (2u)


#define BQ7973X_MAXOCPLAUVALUE               (0x3852u)  /* 275mv */
#define BQ7973X_MINOCPLAUVALUE               (0xC7AEu) /* -275mv */
#define BQ7973X_VF_NUM_ACT                                (2u)
#define BQ7973X_GP_ADC_NUM                                (2u)
#define BQ7973X_INVALDOCVALUE                (0x8000u)
#define BQ7973X_MAXSWMONVALUE                (33000u)
#define BQ7973X_MINSWMONVALUE                (29000u)
#define BQ7973X_SW_DIAG_DELAY                (10u)
#define BQ7973X_SW_DIAG_SW_CTRL1             (0x22u)  /* SW1,SW3 output is high, SW2,SW4 output is off */
#define BQ7973X_SW_DIAG_SW_CTRL2             (0x44u) /* SW1,SW3 output is off, SW2,SW4 output is high */
#define BQ7973X_COMM_H2L                     (0u)
#define BQ7973X_COMM_L2H                     (0x80u)


#define BQ7973X_OC_THRH                      (0x3u)  /* Set the desired over current threshold 137.5mV in registers OC*_THR* */
#define BQ7973X_OC_THRL                      (0x85u) /* and the deglitch filter time 32us is set by [OC*_DEG]. */
#define BQ7973X_DIAG_OCC1_CTRL1              (0x47u) /* Set OCC1 above the OC threshold(-275mV) and The OC1 over */
#define BQ7973X_DIAG_OCC1_CTRL2              (0xB3u) /* current detection mode is enabled */
#define BQ7973X_DIAG_OCD1_CTRL1              (0x6u)  /* Set OCD1 below the OC threshold(30mV) and The OC1 over */
#define BQ7973X_DIAG_OCD1_CTRL2              (0x33u) /* current detection mode is enabled */
#define BQ7973X_DIAG_OCC2_CTRL1              (0x79u) /* Set OCC2 below the OC threshold(-30mV) and The OC2 over */
#define BQ7973X_DIAG_OCC2_CTRL2              (0xD7u) /* current detection mode is enabled */
#define BQ7973X_DIAG_OCD2_CTRL1              (0x38u) /* Set OCD2 above the OC threshold(275mV) and The OC2 over */
#define BQ7973X_DIAG_OCD2_CTRL2              (0x57u) /* current detection mode is enabled */
#define BQ7973X_VAL_INVALDVOLVALUE           (0xFFFFu)
#define BQ7973X_INSUL_DET_SW_CTRL1           (6u)   /* SW1 output is high, SW2 output is low */
#define BQ7973X_INSUL_DET_SW_CTRL2           (0xAu) /* SW1 output is high, SW2 output is high */

#define BQ7973X_CSPLAU_CHECKTIMES (1u)

#define BQ7973X_GPPLAU_CHECKTIMES (1u)

#define BQ7973X_INVALDCSVALUE (0x800000u)


#define BQ7973X_MAXVFPLAUVALUE               (45000u)
#define BQ7973X_MINVFPLAUVALUE               (5000u)
#define BQ7973X_MAXCSPLAUVALUE               (2750000u)  /* 275mv */
#define BQ7973X_MINCSPLAUVALUE               (0xD609D0u) /* -275mv */

#define BQ7973X_SEC_OTP_UNLOCKCODE           (0x6Fu)
#define BQ7973X_OTP_DONE_DELAY               (20u)
#define BQ7973X_OTP_UNLOCK_STATCHECK         (0xF0u)
#define BQ7973X_GPIOSETNUM                   (3u)
#define BQ7973X_GPIOSETPULLUP                (0x06u)
#define BQ7973X_GPIOSETPULLDOWN              (0x07u)
#define BQ7973X_FIR_OTP_UNLOCKCODE           (0xBCu)
#define BQ7973X_MAXGPPLAUVALUE               (45000u)
#define BQ7973X_MINGPPLAUVALUE               (5000u)
#define BQ7973X_DIE_TEMP_NUM                 (2u)
#define BQ7973X_INVALDDIETEMPVALUE           (0x8000u)
#define BQ7973X_GPADJSHORT_THRESHOLD         (1000u)
#define BQ7973X_OCPWMPERCENT                 (100u)
#define BQ7973X_NOOCFAULTPWM                 (25u)  /* there is no over current fault present the OC1 and OC2 output*/
                                                    /* signals toggle low for 75% and high for 25% of the time. */
#define BQ7973X_OCFAULTPWM                   (75u) /* An over current fault is present then the OC1 and OC2 output*/
                                                    /* signals toggle low for 25% and high for 75% of the time. */
#define BQ7973X_ICU_OC1CHANNEL               ((uint8) 4u) /* Connect OC1 to MUC pin N2HET1[16]. */
#define BQ7973X_ICU_OC2CHANNEL               ((uint8) 5u) /* Connect OC2 to MUC pin N2HET1[18]. */
/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef enum BQ7973x_ChannelStatusType_Yag
{
    BQ7973X_NO_CHANNEL_OPEN = 0u,
    BQ7973X_EVEN_CHANNEL_OPEN,
    BQ7973X_ODD_CHANNEL_OPEN
}
Bq7973x_ChannelStatusType;

typedef enum Bq7973x_DiagStepsType_Tag
{
    DIAG_xSTEP_READY,
    DIAG_xSTEP_STATE1,
    DIAG_xSTEP_STATE2,
    DIAG_xSTEP_STATE3,
    DIAG_xSTEP_STATE4,
    DIAG_xSTEP_STATE5,
    DIAG_xSTEP_STATE6,
    DIAG_xSTEP_STATE7,
    DIAG_xSTEP_STATE8,
    DIAG_xSTEP_STATE9,
    DIAG_xSTEP_STATE10,
    DIAG_xSTEP_STATE11,
    DIAG_xSTEP_STATE12,
    DIAG_xSTEP_STATE13,
    DIAG_xSTEP_FINISHED
}
Bq7973x_DiagStepsType;

typedef enum BQ7973x_DiagProcStateType_Tag
{
    PMI_DIAG_IDLE = 0x1,
    PMI_DIAG_TRANSIENT = 0x2,
    PMI_DIAG_RUN = 0x3,

    PMI_DIAG_COMPLETE = 0x4,
    PMI_DIAG_ERROR = 0x5,
    PMI_DIAG_RETRY_ERROR = 0x6,
    PMI_DIAG_ABORT = 0x7
}
Bq7973x_DiagProcStateType;

typedef struct BQ7973x_DiagType_Tag
{
    uint32 qGuardStart;

    const Bq7973x_ConfigType *pPmcCfg;
    const ServiceCfgType *pDiagCfg;
    const ServiceCfgType *pFuncCfg;
    const Basic_ConfigType *pBswCfg;
    const Bq7973x_ManagerType *pPmcMgr;
    const Comif_ManagerType *pComifCtx;

    ctPeQueueType zDiagCmdQueFree;
    ctPeQueueType zDiagCmdQueExec;

    DiagPmiResultType *pDiagResult;
    DiagPmiStatusType *pDiagStatus;

    uint16 wOvThreshVal;
    uint16 wUvThreshVal;
    uint16 wOtThreshVal;
    uint16 wUtThreshVal;
    uint8 uCheckTimesNum ; /* to be init with zero*/
    uint8 uBmcIface;
    uint8 uNAFEs;
    uint8 sDevId; /* to be init with zero or 1 in the Diag_init func*/
    uint8 uNPmics; 
    uint8 uPmicIface;
    uint8 uDioNFaultPin;
    uint8 eComType;
    uint8 uProcState;
    uint8 uDiagRestartReq;

    uint8 uDiagInitState;
    uint8 uInit;

    uint32 qGuardEnd;
}
Bq7973x_DiagType;

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

#define BQ7973X_DIAG_START_SEC_CODE
#include "Cdd_MemMap.h"

FUNC(uint16, BQ7973x_diag_CODE) BQ7973x_DiagAbsDiffTemp(uint16 wTemp1, uint16 wTemp2);
FUNC(uint16, BQ7973x_diag_CODE) BQ7973x_DiagAbsDiffVoltage(uint16 wVolt1, uint16 wVolt2);

FUNC(uint8, BQ7973x_diag_CODE) Bq7973x_DiagServiceReq(Bq7973x_DiagType *pDiagMgr, uint8 uDiagReqId, void *pUserData);
FUNC(uint8, BQ7973x_diag_CODE) Bq7973x_DiagDataProc(Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pService);

FUNC(uint8, bq7973x_diag_CODE) BQ7973x_GetDiagInitState(Bq7973x_DiagType *pDiagMgr);
FUNC(uint8, bq7973x_diag_CODE) bq7973x_diagDeInit(uint8 uPmicIface);
FUNC(void, BQ7973x_diag_CODE) Bq7973x_DiagInitComplete(uint8 uIface, uint8 uServiceId, uint8 uSubId,
                                                       uint8 uDiagStat, void *pData);

FUNC(uint8, BQ7973x_diag_CODE) Bq7973x_DiagFdti(Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
                                                Bq7973x_DiagReqType *pDiagReq);

FUNC(uint8, BQ7973x_diag_CODE) Bq7973x_DiagMpfdi(Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
                                                 Bq7973x_DiagReqType *pDiagReq);

FUNC(void*, BQ7973x_diag_CODE) BQ7973x_DiagInit(const Bq7973x_ManagerType *pPmicMgrCtx, const Pmi_ConfigType *pPmiCfg,
                                                const Comif_ManagerType *pComifCtx, uint8 uPmicIface, uint8 uNAFEsCfg,
                                                uint8 uNfaultPinCfg, uint8 eComType, uint8 uNBmicsCfg, uint8 uSDevId);
FUNC(uint8, bq7973x_CODE) BQ7973X_OCCtrl1Init(Bq7973x_DiagType *pPmicMgr ,const ServiceCfgType *pSvcCfg);

/*
FUNC(void*, BQ7973x_diag_CODE) BQ7973x_DiagInit(const Bq7973x_ManagerType *pBmcMgrCtx, const Bmi_ConfigType *pPMICfg,
                                                const Comif_ManagerType *pComifCtx, uint8 uBmcIface, uint8 uNAFEsCfg,
                                                uint8 uNfaultPinCfg, uint8 eComType);
*/

#define BQ7973X_DIAG_STOP_SEC_CODE
#include "Cdd_MemMap.h"

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

#endif /*BQ7973X_DIAG_H*/

/*********************************************************************************************************************
 * End of File: BQ7973x_diag.h
 *********************************************************************************************************************/
