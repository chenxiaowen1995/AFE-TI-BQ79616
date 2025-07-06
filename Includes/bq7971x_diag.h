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
 *  File:       bq7971x_diag.h
 *  Project:    TIBMS
 *  Module:     DIAG
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for BMI DIAG interface
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       24Aug2022    SEM    0000000000000    Initial version
 *
 *********************************************************************************************************************/

#ifndef BQ7971X_DIAG_H
#define BQ7971X_DIAG_H

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

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

#define BQ7971X_DIAG_MAJOR_VERSION (0x01u)
#define BQ7971X_DIAG_MINOR_VERSION (0x00u)
#define BQ7971X_DIAG_PATCH_VERSION (0x00u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

#define BMI_SM_REG_READ_1BYTE                                   (0u)
#define BMI_SM_REG_READ_2BYTE                                   (1u)
#define BMI_SM_REG_READ_3BYTE                                   (2u)
#define BMI_SM_REG_READ_4BYTE                                   (3u)
#define BMI_SM_REG_READ_5BYTE                                   (4u)

#define BMI_DIAG_DEV_STAT                                       (BMI_SM_REG_READ_2BYTE)
#define BMI_DIAG_DIAG_STAT                                      (BMI_SM_REG_READ_2BYTE)
#define BMI_DIAG_DEV_DIAG_STAT                                  (BMI_SM_REG_READ_5BYTE)
#define BMI_DIAG_ADC_DATA_READY                                 (BMI_SM_REG_READ_1BYTE)
#define BMI_DIAG_DIAG_D1D2                                      (BMI_SM_REG_READ_2BYTE)
#define BMI_DIAG_GPIO_STAT                                      (BMI_SM_REG_READ_2BYTE)
#define BMI_DIAG_FAULT_ADC_MISC                                 (BMI_SM_REG_READ_1BYTE)

#define BQ7971X_ADC_FREEZE_MSK                             (0xFEu)
#define BQ7971X_ADC_UNFREEZE_MSK                           (0x7Eu)
#define BQ7971X_NFAULT_DIAG_RST1                           (0x83u)
#define BQ7971X_NFAULT_DIAG_RST2                           (0xFFu)
#define BQ7971X_DEBUG_TX_DIS                               (0x0Bu)
#define BQ7971X_DEBUG_CTRL_UNLOCKCODE                      (0xA5u)
#define BQ79600_FAULT_RST_RST_UART_SPI_MSK                 (0x40u)
#define BQ79600_FAULT_RST_OFFSET                           (0x2030u)
#define BQ79600_DEBUG_UART_FRAME_OFFSET                    (0x2303u)
#define BQ7971X_FACT_CRC_DONE_DELAY                        (8u)
#define BQ7971X_FAULT_RST2_COMM                            (0x1Fu)
#define BQ7971X_SPI_TEST_CODE                              (0xFFu)
#define BQ7971X_MAX_ADC_DLY                                (1020u)
#define BQ7971X_UNIT_ADC_DLY                               (4u)
/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef enum Bq7971x_ChannelStatusType_Yag
{
    BQ7971X_NO_CHANNEL_OPEN = 0u,
    BQ7971X_EVEN_CHANNEL_OPEN,
    BQ7971X_ODD_CHANNEL_OPEN
}
Bq7971x_ChannelStatusType;

typedef enum Bq7971x_DiagStepsType_Tag
{
    DIAG_STEP_READY,
    DIAG_STEP_STATE1,
    DIAG_STEP_STATE2,
    DIAG_STEP_STATE3,
    DIAG_STEP_STATE4,
    DIAG_STEP_STATE5,
    DIAG_STEP_FINISHED,
}
Bq7971x_DiagStepsType;

typedef enum Bq7971x_DiagProcStateType_Tag
{
    BMI_DIAG_IDLE = 0x1,
    BMI_DIAG_TRANSIENT = 0x2,
    BMI_DIAG_RUN = 0x3,

    BMI_DIAG_COMPLETE = 0x4,
    BMI_DIAG_ERROR = 0x5,
    BMI_DIAG_RETRY_ERROR = 0x6,
    BMI_DIAG_ABORT = 0x7
}
Bq7971x_DiagProcStateType;

typedef struct Bq7971x_DiagType_Tag
{
    uint32 qGuardStart;

    const Bq7971x_ConfigType *pBmicCfg;
    const ServiceCfgType *pDiagCfg;
    const ServiceCfgType *pFuncCfg;
    const Basic_ConfigType *pBswCfg;
    const Bq7971x_ManagerType *pBmcMgr;
    const Comif_ManagerType *pComifCtx;

    ctPeQueueType zDiagCmdQueFree;
    ctPeQueueType zDiagCmdQueExec;

    DiagResultType *pDiagResult;
    DiagStatusType *pDiagStatus;

    uint16 wOvThreshVal;
    uint16 wUvThreshVal;
    uint16 wOtThreshVal;
    uint16 wUtThreshVal;

    uint8 uBmicIface;
    uint8 uNAFEs;
    uint8 uNBmics;
    uint8 sDevId;

    uint8 uDioNFaultPin;
    uint8 eComType;
    uint8 uProcState;
    uint8 uDiagRestartReq;

    uint8 uDiagInitState;
    uint8 uInit;
    uint8 uRsvd[2];

    uint32 qGuardEnd;
}
Bq7971x_DiagType;

typedef struct Bq7971x_CellBalType_Tag
{
    uint16 uCbStatus;
    uint16 uCbSrcCtrl;
}
Bq7971x_CellBalType;



struct Bq7971x_ManagerType_Tag
{
    uint32 qGuardStart;

    const Bq7971x_ConfigType *pBmicCfg;
    const ServiceCfgType *pFuncCfg;
    const ServiceCfgType *pCellBalCfg;
    const Basic_ConfigType *pBswCfg;
    const Comif_ManagerType *pComifCtx;

    Bq7971x_DiagType *pBmicDiag;
    Bq7971x_CellBalType *pCbCtrl;

    uint32 qApplRes;

    uint8 uBmicIface;
    uint8 eCommDir;
    uint8 eComType;
    uint8 eDrvState;

    uint8 nCurrNoOfDevs;
    uint8 uNAFEs;
    uint8 uNBmics;
    uint8 uNDevices;

    uint8 uSDevId;
    uint8 uIsPmicInIf;
    uint8 uNfaultPin;
    uint8 uInit;

    uint32 qGuardEnd;
};

typedef enum BQ7971X_D1_SEL_Enum_Tag
{
    BQ7971X_DIAG_D1_NO_SEL = 0u,
    BQ7971X_DIAG_D1_OV_UV_CBDONE_DAC = 1u,
    BQ7971X_DIAG_D1_BG1 = 2u,
    BQ7971X_DIAG_D1_AVDD = 3u,
    BQ7971X_DIAG_D1_OCC_MAIN_DAC = 4u,
    BQ7971X_DIAG_D1_OCD_MAIN_DAC = 5u,
} BQ7971X_DIAG_D1_SEL_Enum_Type;

typedef enum BQ7971X_D2_SEL_Enum_Tag
{
    BQ7971X_DIAG_D2_NO_SEL = 0u,
    BQ7971X_DIAG_D2_OT_UT_OTCB_DAC = 1u,
    BQ7971X_DIAG_D2_BG2 = 2u,
    BQ7971X_DIAG_D2_OCC_RDNT_DAC = 4u,
    BQ7971X_DIAG_D2_OCD_RDNT_DAC = 5u,
} BQ7971X_DIAG_D2_SEL_Enum_Type;

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

#define BQ7971X_DIAG_START_SEC_CODE
#include "Cdd_MemMap.h"

FUNC(uint16, bq7971x_diag_CODE) DiagAbsDiffTemp(uint16 wTemp1, uint16 wTemp2);
FUNC(uint16, bq7971x_diag_CODE) DiagAbsDiffVoltage(uint16 wVolt1, uint16 wVolt2);

FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagServiceReq(Bq7971x_DiagType *pDiagMgr, uint8 uDiagReqId, void *pUserData , uint8 uChannel);
FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagDataProc(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pService);

FUNC(uint8, bq7971x_diag_CODE) Bq7971x_GetDiagInitState(Bq7971x_DiagType *pDiagMgr);
FUNC(void, bq7971x_diag_CODE) Bq7971x_DiagInitComplete(uint8 uIface, uint8 uServiceId, uint8 uSubId,
                                                          uint8 uStatus, void *pData);

FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagFdti(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
                                                Bq7971x_DiagReqType *pDiagReq ,uint8 uChannel);

FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagMpfdi(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
                                                 Bq7971x_DiagReqType *pDiagReq, uint8 uChannel);

FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagFdti_2(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
                                                Bq7971x_DiagReqType *pDiagReq, uint8 uChannel);

FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagMpfdi_2(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
                                                 Bq7971x_DiagReqType *pDiagReq,uint8 uChannel);
FUNC(void*, bq7971x_diag_CODE) Bq7971x_DiagInit(const Bq7971x_ManagerType *pBmicMgrCtx, const Bmi_ConfigType *pBmiCfg,
                                                const Comif_ManagerType *pComifCtx, uint8 uBmicIface, uint8 uNAFEsCfg,
                                                uint8 uNfaultPinCfg, uint8 eComType, uint8 uNBmicsCfg, uint8 uSDevId);

FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagDeInit(uint8 uBmicIface);
#define BQ7971X_DIAG_STOP_SEC_CODE
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

#endif /*BQ7971X_DIAG_H*/

/*********************************************************************************************************************
 * End of File: bq7971x_diag.h
 *********************************************************************************************************************/
