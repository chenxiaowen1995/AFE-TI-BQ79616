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
 *  File:       bq7971x.h
 *  Project:    TIBMS
 *  Module:     BMI
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for BQ7971x interface
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

#ifndef BQ7971X_H
#define BQ7971X_H

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

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

#define BQ7971X_SW_MAJOR_VERSION                            (0x01u)
#define BQ7971X_SW_MINOR_VERSION                            (0x00u)
#define BQ7971X_SW_PATCH_VERSION                            (0x00u)

/** Texas Instruments Vendor ID*/
#define BQ7971X_VENDOR_ID                                   (44u)

/** BQ7971x Driver Module ID       */
#define BQ7971X_MODULE_ID                                   (255u)

/** BQ7971x Driver Instance ID */
#define BQ7971X_INSTANCE_ID                                 (11u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

#define BQ7971X_GET_DATA_PTR(pSvcCfg, uNAFEs, uDevIdx)      (&(pSvcCfg->pRawData[(pSvcCfg->uDataOffset + \
                                                             (pSvcCfg->uBlockLen * (uNAFEs - uDevIdx - 1u)))]))

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

#define BQ7971X_GPIOSETNUM                                  (0x03u)
#define BQ7971X_GPIOSETADCONLY                              (0x02u)
#define BQ7971X_GPIOSETOUTLOW                               (0x05u)
#define BQ7971X_GPIOSETPULLUP                               (0x06u)     /* GPIO as ADC input (absolute) and weak pull-up. */
#define BQ7971X_GPIOSETPULLDOWN                             (0x07u)     /* GPIO as ADC input (absolute) and weak pull-down. */

#define BQ7971X_INVALDVOLVALUE                              (0xFFFFu)
#define BQ7971X_INVALDTEMPVALUE                             (0x8000u)   /* Invalid temperature value  */

#define BQ7971X_CB_APP_CTRL                                 (0x00u)
#define BQ7971X_CB_DIAG_CTRL                                (0x20u)

#define BQ7971X_SILICON_REV_A0                              (0x01u)
#define BQ7971X_SILICON_REV_B0                              (0x02u)
#define BQ7971X_SILICON_REV_B1                              (0x03u)
#define BQ7971X_SILICON_REV_C1                              (0x04u)
#define BQ7971X_SILICON_REV_D1                              (0x05u)

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef enum Bq7971x_CBReqSrcType_Tag
{
    BQ7971X_CB_APP_NONE,
    BQ7971X_CB_APP_PAUSE,
    BQ7971X_CB_APP_UNPAUSE,
    BQ7971X_CB_APP_RUN,

    BQ7971X_CB_DIAG_PAUSE,
    BQ7971X_CB_DIAG_UNPAUSE,
    BQ7971X_CB_DIAG_RUN
}
Bq7971x_CBReqSrcType;

typedef struct Bq7971x_ManagerType_Tag Bq7971x_ManagerType;

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

#define BQ7971X_START_SEC_CODE
#include "Cdd_MemMap.h"

FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeVcell(const Bq7971x_ManagerType *pBmcCtx, const ServiceCfgType *pSvcCfg,
                                              uint16 *pOutVcellVolt);
FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeGpio(const Bq7971x_ManagerType *pBmcCtx, const ServiceCfgType *pSvcCfg,
                                             uint16 *pOutGpioVolt);

FUNC(uint8, bq7971x_CODE) Bq7971x_StartBalCtrl(const Bq7971x_ManagerType *pMgr, uint32 qRg, uint8 uReq, uint8 uCSrc, uint8 isAutoBal_u8);
FUNC(uint8, bq7971x_CODE) Bq7971x_PauseBalCtrl(const Bq7971x_ManagerType *pMgr, uint32 qRg, uint8 uReq, uint8 uCSrc);
FUNC(uint8, bq7971x_CODE) Bq7971x_UnpauseBalCtrl(const Bq7971x_ManagerType *pMgr, uint32 qRg, uint8 uReq, uint8 uCSrc);
FUNC(uint8, bq7971x_CODE) Bq7971x_GetBalCtrlStatus(const Bq7971x_ManagerType *pBmcMgr);
FUNC(uint8, bq7971x_CODE) Bq7971x_SetCbTime(const Bq7971x_ManagerType *pBmicMgr, const Bmi_uCbTime * uCbTime );
FUNC(uint8, bq7971x_CODE) Bq7971x_SetDiagAdcCtrl1(const Bq7971x_ManagerType *pBmcMgr, uint8 uAdcCtrl);
FUNC(uint8, bq7971x_CODE) Bq7971x_SetDiagAdcCtrl2(const Bq7971x_ManagerType *pBmcMgr, uint8 uAdcCtr2);
FUNC(uint8, bq7971x_CODE) Bq7971x_SetDiagAdcCtrl3(const Bq7971x_ManagerType *pBmcMgr, uint8 uAdcCtr3);
FUNC(uint8, bq7971x_CODE) Bq7971x_DevConfInit(Bq7971x_ManagerType *pBmcMgr);
FUNC(uint8, bq7971x_CODE) Bq7971x_Control2Init(Bq7971x_ManagerType *pBmcMgr);
FUNC(uint8, bq7971x_CODE) Bq7971x_GpioConfInit(const Bq7971x_ManagerType *pBmcMgr);
FUNC(uint8, bq7971x_CODE) Bq7971x_AdcCtrlInit(Bq7971x_ManagerType *pBmcMgr);
FUNC(uint8, bq7971x_CODE) Bq7971x_OvUvThreshInit(Bq7971x_ManagerType *pBmcMgr);
FUNC(uint8, bq7971x_CODE) Bq7971x_OtUtThreshInit(Bq7971x_ManagerType *pBmcMgr);

FUNC(uint8, bq7971x_CODE) Bq7971x_ControlProc(void *pBmcCtx, uint8 uCmd, void *pData, uint16 wLength);
FUNC(uint8, bq7971x_CODE) Bq7971x_ServiceProc(void *pBmcCtx, uint8 uServiceTyp, uint8 uCmd, uint8 uNodeId, void *pData);
FUNC(uint8, bq7971x_CODE) Bq7971x_TxHandlerProc(void *pBmcCtx);
FUNC(uint8, bq7971x_CODE) Bq7971x_RxHandlerProc(void *pBmcCtx);
FUNC(uint8, bq7971x_CODE) Bq7971x_NotifyProc(void *pBmcCtx, uint8 uType);
FUNC(uint8, bq7971x_CODE) Bq7971x_ReceiveProc(void *, const ServiceCfgType *, const uint8 *, uint8);
FUNC(uint8, bq7971x_CODE) Bq7971x_DataDecode(void *pBmcCtx, uint8 uService,uint8 uSubId, void *pBuffer, uint16 wLength);
FUNC(void*, bq7971x_CODE) Bq7971x_Init(const InterfaceCfgType *pIfaceCfg, Comif_ManagerType *pComifCtx, uint8 uNfaultCfg , uint8 uStartUp, uint8 uWupReq);

FUNC(uint8, bq7971x_CODE) Bq7971x_DeInit(void *pBmcCtx);

                                /*  8.1 */
STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_DevPowerManager(Bq7971x_ManagerType *pBmicMgr, uint8 uState);
STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_GetVRefCap(Bq7971x_ManagerType *pBmicMgr,const  ServiceCfgType *pSvcCfg);
STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_GetVBat(Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg);
STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_GetDiagRdnt(Bq7971x_ManagerType *pBmicMgr,const  ServiceCfgType *pSvcCfg);
STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_GetDiagMain(Bq7971x_ManagerType *pBmicMgr,const  ServiceCfgType *pSvcCfg);
                                    /* 8.2 */
FUNC(uint8, bq7971x_CODE) BQ7971X_SetCommConf( Bq7971x_ManagerType *pBmicMgr , uint8 uDevId);
FUNC(uint8, bq7971x_CODE) BQ7971X_SetBBConf( Bq7971x_ManagerType *pBmicMgr , uint8 uDevId);
FUNC(uint8, bq7971x_CODE) BQ7971X_SetEnableSPI(const Bq7971x_ManagerType *pBmicMgr , uint8 uDevId);
FUNC(uint8, bq7971x_CODE) BQ7971X_SetSPIConf(const Bq7971x_ManagerType *pBmicMgr , uint8 uDevId);
FUNC(uint8, bq7971x_CODE) BQ7971X_SetSPIExe(const Bq7971x_ManagerType *pBmicMgr , uint8 uDevId, uint8 uSPIExe);
FUNC(uint8, bq7971x_CODE) BQ7971X_WriteSPIData(const Bq7971x_ManagerType *pBmicMgr , uint8 uDevId, const uint8 *uSPIData);
FUNC(uint8, bq7971x_CODE) BQ7971X_ReadSPIData( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg);
FUNC(uint8, bq7971x_CODE) BQ7971X_SetEnableI2C( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg , uint8 uDevId);
FUNC(uint8, bq7971x_CODE) BQ7971X_SetI2CCtrl( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg , uint8 uDevId, uint8 uI2CCtrl);
FUNC(uint8, bq7971x_CODE) BQ7971X_SetI2CData( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg , uint8 uDevId, uint8 uI2CData);
FUNC(uint8, bq7971x_CODE) BQ7971X_GetI2CData( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg);
FUNC(uint8, bq7971x_CODE) BQ7971X_GetI2CFault( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg);
FUNC(uint8, bq7971x_CODE) BQ7971X_GpioOpenSetGpioDown( Bq7971x_ManagerType *pBmicMgr);
STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_GetVcellActSum(Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg);
STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_GetDieTemp(Bq7971x_ManagerType *pBmicMgr,const  ServiceCfgType *pSvcCfg);
FUNC(uint8, bq7971x_CODE) BQ7971X_GpioAdjSetGpioConf( Bq7971x_ManagerType *pBmicMgr, Gpio_conf *g_conf,const ServiceCfgType *pSvcCfg);
FUNC(uint8, bq7971x_CODE) BQ7971X_EnADCFreeze( Bq7971x_ManagerType *pBmicMgr);
FUNC(uint8, bq7971x_CODE) BQ7971X_DisADCFreeze( Bq7971x_ManagerType *pBmicMgr);
FUNC(uint8, bq7971x_CODE) BQ7971X_SetADCDelay( Bq7971x_ManagerType *pBmicMgr  ,  uint16 xTimeUs , uint8 uDevId);
FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeVcellActSum(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg,
                                              uint16 *pOutVcellActSum);
FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeDieTemp(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg,
                                              uint32 *pOutDieTemp);
FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeDiagMain(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg,
                                              uint32 *pOutDiagMain);
FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeDiagRdnt(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg,
                                              uint32 *pOutDiagRdnt);
FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeI2CData(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg,
                                              uint16 *pOutI2CData);
FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeI2CFault(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg,
                                              uint16 *pOutI2CFault);
#define BQ7971X_STOP_SEC_CODE
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

#include "bq7971x_cfg.h"
#include "bq7971x_diag.h"

#endif /*BQ7971X_H*/

/*********************************************************************************************************************
 * End of File: bq7971x.h
 *********************************************************************************************************************/
