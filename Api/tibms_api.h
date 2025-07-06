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
 *  File:       tibms_api.h
 *  Project:    TIBMS
 *  Module:     TIBMS
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  API's for the TI BMS functionalities
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05May2022    SEM                0000000000000    Initial version
 * 01.01.00       04Aug2023    SEM                0000000000000    Pack monitor API Updates
 *
 *********************************************************************************************************************/

#ifndef TIBMS_API_H
#define TIBMS_API_H

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

#include "tibms_cfg.h"
#include "tibms_diag.h"
#include "tibms_com.h"
#include "tibms_version.h"

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

#define TIBMS_API_MAJOR_VERSION             (0x01u)
#define TIBMS_API_MINOR_VERSION             (0x01u)
#define TIBMS_API_PATCH_VERSION             (0x00u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

#define TIBMS_VENDOR_ID                     ((uint16) 44u)
#define TIBMS_MODULE_ID                     ((uint16) 255u)
#define TIBMS_INSTANCE_ID                   ((uint8)  00u)

#define BMI_STATUS_ERROR_STATE              (118u)
#define BMI_STATUS_INIT_PROGRESS            (119u)
#define BMI_STATUS_AUTOADDR_SUCCESS         (120u)
#define BMI_STATUS_CFGINIT_SUCCESS          (121u)
#define BMI_STATUS_STARTUP_SUCCESS          (122u)
#define BMI_STATUS_DIAG_INIT_PROGRESS       (123u)
#define BMI_STATUS_DIAG_INIT_COMPLETED      (124u)
#define BMI_STATUS_INIT_SUCCESS             (125u)
#define BMI_STATUS_NORMAL_STATE             (126u)

#define PMI_STATUS_ERROR_STATE              (118u)
#define PMI_STATUS_INIT_PROGRESS            (119u)
#define PMI_STATUS_AUTOADDR_SUCCESS         (120u)
#define PMI_STATUS_CFGINIT_SUCCESS          (121u)
#define PMI_STATUS_STARTUP_SUCCESS          (122u)
#define PMI_STATUS_DIAG_INIT_PROGRESS       (123u)
#define PMI_STATUS_DIAG_INIT_COMPLETED      (124u)
#define PMI_STATUS_INIT_SUCCESS             (125u)
#define PMI_STATUS_NORMAL_STATE             (126u)

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef uint8 Bmi_uCbTime[TIBMS_NO_OF_BMI_IFACES][TIBMS_BMIC_CELLS_IFACE_0];
/*typedef struct Bmi_CellBalType_Tag
{
    //uint32 uCbEnable:1;
    uint8 uCbTime[TIBMS_NO_OF_BMI_IFACES][TIBMS_BMIC_CELLS_IFACE_0];
   // uint32 uRsvd:24;
}
Bmi_CellBalType;*/

typedef enum Bmi_CellBalServiceType_Tag
{
    BMI_CB_GET_BALSTAT,
    BMI_CB_GET_BALSWSTAT,
    BMI_CB_GET_BALDONESTAT,
    BMI_CB_GET_BALTIME,
    BMI_CB_RESP_END,

    BMI_CB_SET_VCBDONETH,
    BMI_CB_SET_OTCBTH,
    BMI_CB_SET_BALDUTY,
    BMI_CB_SET_CBTIME,
    BMI_CB_SET_FLTSTOP_EN,
    BMI_CB_SET_BAL_ACT,
    BMI_CB_SET_AUTO_BAL,
    BMI_CB_START_BALCTRL,
    BMI_CB_PAUSE_BALCTRL,
    BMI_CB_UNPAUSE_BALCTRL,
    BMI_CB_REQ_END
}
Bmi_CellBalServiceType;

typedef enum Bmi_WakeupReqType_Tag
{
    BMI_SHUT2WAKEUP = 0x30u,
    BMI_SLP2WAKEUP = 0x31u
}
Bmi_WakeupReqType;

typedef enum Bmi_ResetReqType_Tag
{
    BMI_NO_RESET_REQ = 0x00u,
    BMI_RESET_REQ = 0x01u
}
Bmi_ResetReqType;

typedef enum ComifNotifyType_Tag
{
    TRANSFER_REQUEST_NOTIFY = 0x10u,
    TRANSFER_COMPLETE_NOTIFY = 0x11u,
    TRANSFER_TIMEOUT_NOTIFY = 0x12u,
    TRANSFER_DELAY_NOTIFY = 0x13u
}
ComifNotifyType;

typedef enum ResultNotifyType_Tag
{
    INIT_NOTIFY_BLOCKING = 0x20u,
    INIT_NOTIFY_CALLBACK = 0x21,
    INIT_NOTIFY_POLLING = 0x22
}
ResultNotifyType;

typedef enum DiasyChainDirType_Tag
{
    DIR_COMH2L = 0x00u,
    DIR_COML2H = 0x80,
    DIR_DIRECT = 0x01
}
DaisyChainDirType;

typedef enum DaisyChainPmicDevIdType_Tag
{
    PM_DEVID_BEGIN = 0x1,
    PM_DEVID_END = 0xF
}
DaisyChainPmicDevIdType;

typedef enum BmiIfaceType_Tag
{
    BMI_IFACE_0 = 0u,
    BMI_IFACE_1 = 1u,
    BMI_IFACE_2 = 2u,
    BMI_IFACE_3 = 3u,
    BMI_IFACE_4 = 4u,
}
BmiIfaceType;

typedef enum PmiIfaceType_Tag
{
    PMI_IFACE_0 = 0u,
    PMI_IFACE_1 = 1u,
    PMI_IFACE_2 = 2u,
    PMI_IFACE_3 = 3u,
    PMI_IFACE_4 = 4u,
}
PmiIfaceType;

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

/** Pack Monitor Exposed Functionalities **/

FUNC(uint8, tibms_pmi_CODE) Pmi_GetCfVoltageData(uint8 uPmiIface, void *pBuffer, uint16 wLength);
FUNC(uint8, tibms_pmi_CODE) Pmi_GetVfVoltageData(uint8 uPmiIface, void *pBuffer, uint16 wLength);
FUNC(uint8, tibms_pmi_CODE) Pmi_GetPackCurrentData(uint8 uPmiIface, void *pBuffer, uint16 wLength);

FUNC(uint8, tibms_pmi_CODE) Pmi_SetOCOutput(uint8 uPmiIface,uint8 uOCCtrl2);
FUNC(uint8, tibms_pmi_CODE) Pmi_EnterCommDebug(uint8 uPmiIface);
FUNC(uint8, tibms_pmi_CODE) Pmi_SetADCDelay(uint8 uPmiIface,uint16 xTimeUs);
FUNC(uint8, tibms_pmi_CODE) Pmi_DisADCFreeze(uint8 uPmiIface);
FUNC(uint8, tibms_pmi_CODE) Pmi_EnADCFreeze(uint8 uPmiIface);
FUNC(uint8, tibms_pmi_CODE) Pmi_MSPIConf(uint8 uPmiIface);
FUNC(uint8, tibms_pmi_CODE) Pmi_SetSPICtrl(uint8 uPmiIface,uint8 uSPICtrl);
FUNC(uint8, tibms_pmi_CODE) Pmi_SetSPIExe(uint8 uPmiIface,uint8 uSPIExe);
FUNC(uint8, tibms_pmi_CODE) Pmi_WriteSPIData(uint8 uPmiIface,uint8 *uSPIData);
FUNC(uint8, tibms_pmi_CODE) Pmi_EnableI2C(uint8 uPmiIface);
FUNC(uint8, tibms_pmi_CODE) Pmi_SetI2CData(uint8 uPmiIface,uint8 uI2CData);
FUNC(uint8, tibms_pmi_CODE) Pmi_SetI2CCtrl(uint8 uPmiIface, uint8 CtrlCmd);

FUNC(uint8, tibms_pmi_CODE) Pmi_GetI2CData(uint8 uPmiIface);
FUNC(uint8, tibms_pmi_CODE) Pmi_GetCCOvf(uint8 uPmiIface);
FUNC(uint8, tibms_pmi_CODE) Pmi_GetCP(uint8 uPmiIface);
FUNC(uint8, tibms_pmi_CODE) Pmi_GetCcAcc_Cnt(uint8 uPmiIface);
FUNC(uint8, tibms_pmi_CODE) Pmi_CcClear(uint8 uPmiIface);
FUNC(uint8, tibms_pmi_CODE) Pmi_GetVGpio(uint8 uPmiIface);
FUNC(uint8, tibms_pmi_CODE) Pmi_GetVf(uint8 uPmiIface);
FUNC(uint8, tibms_pmi_CODE) Pmi_GetCurrent(uint8 uPmiIface);
FUNC(uint8, tibms_pmi_CODE) Pmi_ResetRequest(uint8 uPmiIface, uint8 uResetTyp);
FUNC(uint8, tibms_pmi_CODE) Pmi_HandleReqTransfer(uint8 uPmiIface);
FUNC(uint8, tibms_pmi_CODE) Pmi_HandleRecvComplete(uint8 uPmiIface);
FUNC(uint8, tibms_pmi_CODE) Pmi_ProcessRxData(const ServiceCfgType *pSvcCfg, const uint8 *pRxData, uint8 uRespStatus);
FUNC(uint8, tibms_pmi_CODE) Pmi_DecodeServiceData(uint8 uPmiIface, uint8 uService, void *pBuffer, uint16 wLength);
FUNC(uint8, tibms_pmi_CODE) Pmi_ProcessNotify(uint8 uPmiIface, uint8 uType);
FUNC(uint8, tibms_pmi_CODE) Pmi_GetDriverState(uint8 uPmiIface);
FUNC(uint8, tibms_pmi_CODE) Pmi_ConfigInit(uint8 uPmiIface, uint8 eNotify);
FUNC(uint8, tibms_pmi_CODE) Pmi_StartupInit(uint8 uPmiIface, uint8 eNotify);
FUNC(uint8, tibms_pmi_CODE) Pmi_Init(const Bms_ConfigType *pConfigSet_PB, Comif_ManagerType *pComifMgr, uint8 uWupReq);
FUNC(uint8, tibms_pmi_CODE) Pmi_Deinit(uint8 uPmiIface);

/** Battery Monitor Exposed Functionalities **/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagStatus(uint8 uBmiIfaceIn, void *pDiagStat, uint16 wLength);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagResult(uint8 uBmiIfaceIn, void *pDiagResult, uint16 wLength);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetGpioData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetVGpio(uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetVcellData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetVCell(uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_ResetRequest(uint8 uBmiIface, uint8 uResetTyp);
FUNC(uint8, tibms_bmi_CODE) Bmi_ShutdownRequest(uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_SoftReset(uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_HWReset(uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_DevSleep(uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_ShutTowake(uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_SleepToWake(uint8 uBmiIfaceIn);
FUNC(uint8, tibms_bmi_CODE) Bmi_HandleReqTransfer(uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_HandleRecvComplete(uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_DecodeServiceData(uint8 uBmiIface, uint8 uService, void *pBuffer, uint16 wLength);
FUNC(uint8, tibms_bmi_CODE) Bmi_ProcessNotify(uint8 uBmiIface, uint8 uType);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagState(uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetDriverState(uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_StartupInit(uint8 uBmiIface, uint8 eNotify);
FUNC(uint8, tibms_bmi_CODE) Bmi_Init(const Bms_ConfigType *pConfigSet_PB, uint8 uWupReq);
FUNC(uint8, tibms_bmi_CODE) Bmi_Deinit(uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetVersionInfo(uint8 uBmiIface, VersionInfoType *pBmifVersion);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagRdnt(uint8 uBmiIfaceIn);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagMain(uint8 uBmiIfaceIn);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetVBat(uint8 uBmiIfaceIn);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetVBatData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetVRefCap(uint8 uBmiIfaceIn);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetVCapRefData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength);
FUNC(uint8, tibms_bmi_CODE) Bmi_SetDiagAdcCtrl1(uint8 uBmiIfaceIn, uint8 uAdcCtr1);
FUNC(uint8, tibms_bmi_CODE) Bmi_SetDiagAdcCtrl2(uint8 uBmiIfaceIn, uint8 uAdcCtr2);
FUNC(uint8, tibms_bmi_CODE) Bmi_SetDiagAdcCtrl3(uint8 uBmiIfaceIn, uint8 uAdcCtr3);

/************************* 8.2 ************************************/
FUNC(uint8, tibms_bmi_CODE) Bmi_SetCommConf  (uint8 uBmiIface,uint8 uNodeId );
FUNC(uint8, tibms_bmi_CODE) Bmi_SetBBConf    (uint8 uBmiIface, uint8 uDevId);
FUNC(uint8, tibms_bmi_CODE) Bmi_SetEnableSPI (uint8 uBmiIface, uint8 uDevId);
FUNC(uint8, tibms_bmi_CODE) Bmi_SetSPIConf   (uint8 uBmiIface, uint8 uDevId);
FUNC(uint8, tibms_bmi_CODE) Bmi_SetSPIExe    (uint8 uBmiIface, uint8 uDevId, uint8 uSPIExe);
FUNC(uint8, tibms_bmi_CODE) Bmi_WriteSPIData (uint8 uBmiIface, uint8 uDevId, const uint8 *uSPIData);
FUNC(uint8, tibms_bmi_CODE) Bmi_ReadSPIData  (uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_SetEnableI2C (uint8 uBmiIface , uint8 uDevId);
FUNC(uint8, tibms_bmi_CODE) Bmi_SetI2CCtrl   (uint8 uBmiIface, uint8 uDevId, uint8 uI2CCtrl);
FUNC(uint8, tibms_bmi_CODE) Bmi_SetI2CData   (uint8 uBmiIface, uint8 uDevId, uint8 uI2CData);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetI2CData   (uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetI2CFault  (uint8 uBmiIface);

FUNC(uint8, tibms_bmi_CODE) Bmi_GpioOpenSetGpioDown(uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetBalTimeData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetBalStatData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetBalSWStatData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetBalDoneStatData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength);

FUNC(uint8, tibms_bmi_CODE) Bmi_GetVcellActSum(uint8 uBmiIfaceIn);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetDieTemp(uint8 uBmiIfaceIn);
FUNC(uint8, tibms_bmi_CODE) BMI_GpioOpenSetGpioConf(uint8 uBmiIface, uint8 devId, uint8 gpio_conf_index, Gpio_val lower_gpio_val, Gpio_val higher_gpio_val );
FUNC(uint8, tibms_bmi_CODE) Bmi_EnADCFreeze(uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_DisADCFreeze(uint8 uBmiIface);
FUNC(uint8, tibms_bmi_CODE) Bmi_SetADCDelay(uint8 uBmiIface,  uint16 xTimeUs , uint8 uDevId);

FUNC(uint8, tibms_bmi_CODE) Bmi_GetVcellActSumData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetDieTempData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagMainData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagRdntData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength);
FUNC(uint8, tibms_bmi_CODE) Bmi_Get_I2CData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength);
FUNC(uint8, tibms_bmi_CODE) Bmi_GetI2CFaultData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength);
/*********************************************************************************************************************
 * Exported Inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/

#define Bmi_GetBalStat(uIf)           Bmi_CellBalCtrl(uIf, BMI_CB_GET_BALSTAT, NULL)
#define Bmi_GetBalSWStat(uIf)         Bmi_CellBalCtrl(uIf, BMI_CB_GET_BALSWSTAT, NULL)
#define Bmi_GetBalDoneStat(uIf)       Bmi_CellBalCtrl(uIf, BMI_CB_GET_BALDONESTAT, NULL)
#define Bmi_GetBalTime(uIf, uCbCh)    Bmi_CellBalCtrl(uIf, BMI_CB_GET_BALTIME, (void *) uCbCh)
#define Bmi_SetCbThresh(uIf, uTh)     Bmi_CellBalCtrl(uIf, BMI_CB_SET_VCBDONETH, (void *) uTh)
#define Bmi_SetOtCbThresh(uIf, uTh)   Bmi_CellBalCtrl(uIf, BMI_CB_SET_OTCBTH, (void *) uTh)
#define Bmi_SetCbBalDuty(uIf, uDuty)  Bmi_CellBalCtrl(uIf, BMI_CB_SET_BALDUTY, (void *) uDuty)
#define Bmi_SetCbTime(uIf, pCbCtrl)   Bmi_CellBalCtrl(uIf, BMI_CB_SET_CBTIME, (void *) pCbCtrl)
#define Bmi_StartBalCtrl(uIf, isAutoBal_u8)         Bmi_CellBalCtrl(uIf, BMI_CB_START_BALCTRL, isAutoBal_u8)
#define Bmi_PauseBalCtrl(uIf)         Bmi_CellBalCtrl(uIf, BMI_CB_PAUSE_BALCTRL, NULL)

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /*TIBMS_API_H*/

/*********************************************************************************************************************
 * End of File: tibms_api.h
 *********************************************************************************************************************/
