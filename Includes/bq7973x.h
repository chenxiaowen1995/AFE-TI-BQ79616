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
 *  File:       bq7973x.h
 *  Project:    TIBMS
 *  Module:     PMI
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for BMI interface
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

#ifndef BQ7973X_H
#define BQ7973X_H

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

#define BQ7973X_SW_MAJOR_VERSION                (0x01u)
#define BQ7973X_SW_MINOR_VERSION                (0x00u)
#define BQ7973X_SW_PATCH_VERSION                (0x00u)

/** Texas Instruments Vendor ID*/
#define BQ7973X_VENDOR_ID                       (44u)

/** BQ7973X Driver Module ID       */
#define BQ7973X_MODULE_ID                       (255u)

/** BQ7973X Driver Instance ID */
#define BQ7973X_INSTANCE_ID                     (21u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef struct Bq7973x_ManagerType_Tag Bq7973x_ManagerType;

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

#define BQ7973X_START_SEC_CODE
#include "Cdd_MemMap.h"

FUNC(uint8, bq7973x_CODE) Bq7973x_SetOCCtrl1(Bq7973x_ManagerType *pPmcMgr, uint8 uOcCtrl);
FUNC(uint8, bq7973x_CODE) Bq7973x_SetOCConf(Bq7973x_ManagerType *pPmcMgr,  const uint8 *pCfgSet);
FUNC(uint8, bq7973x_CODE) Bq7973x_Control2Init(Bq7973x_ManagerType *);
FUNC(uint8, bq7973x_CODE) Bq7973x_GpioConfInit(Bq7973x_ManagerType *pPmcMgr, const uint8 *pCfgSet);
FUNC(uint8, bq7973x_CODE) Bq7973x_AdcCtrlInit(Bq7973x_ManagerType *pPmcMgr);
FUNC(uint8, bq7973x_CODE) Bq7973x_CCCtrlInit(Bq7973x_ManagerType *pPmcMgr);
FUNC(uint8, bq7973x_CODE) Bq7973x_SetSWInit(Bq7973x_ManagerType *pPmcMgr, uint8 uSwConf);
FUNC(uint8, bq7973x_CODE) Bq7973x_DiagProc(void *pBmcCtx, uint8 uCmd);
FUNC(uint8, bq7973x_CODE) Bq7973x_ControlProc(void *pBmcCtx, uint8 uCmd, void *pData);
FUNC(uint8, bq7973x_CODE) Bq7973x_ServiceProc(void *pBmcCtx, uint8 uServiceTyp, uint8 uCmd, void *pServiceData);
FUNC(uint8, bq7973x_CODE) Bq7973x_TxHandlerProc(void *pBmcCtx);
FUNC(uint8, bq7973x_CODE) Bq7973x_RxHandlerProc(void *pBmcCtx);
FUNC(uint8, bq7973x_CODE) Bq7973x_NotifyProc(void *pBmcCtx, uint8 uType);
FUNC(uint8, bq7973x_CODE) Bq7973x_ReceiveProc(void *pPmcCtx, const ServiceCfgType *pSvcCfg, const uint8 *pRxData,
                                              uint8 uStatus);
FUNC(uint8, bq7973x_CODE) Bq7973x_DataDecode(void *pPmcCtx, uint8 uService, void *pBuffer, uint16 wLength);
FUNC(void*, bq7973x_CODE) Bq7973x_Init(const InterfaceCfgType *iface, Comif_ManagerType *comif, uint8 uWkup);
FUNC(uint8, bq7973x_CODE) Bq7973x_DeInit(void *pBmcCtx);
FUNC(uint8, bq7973x_CODE) Bq7973x_Notify(void *pComifCtx, uint8 uNotifyType);
FUNC(uint8, bq7973x_CODE) Bq7973x_HandleRecieve(void *pComifCtx);
FUNC(uint8, bq7973x_CODE) Bq7973x_HandleTransfer(void *pComifCtx);
FUNC(uint8, bq7973x_CODE) Bq7973x_Control(void *pComifCtx, uint8 uCmd, void *pData);
FUNC(void*, bq7973x_CODE) Bq7973x_ComInit(const void *pComCfgCtx, Emem_StasticsType *pEmemStatsCfg,
                                          const ServiceCfgType *pCommCfg);
FUNC(uint8, bq7973x_CODE) Bq7973x_ComDeinit(void *pComifCtx);

//#define BQ7973x_GET_DATA_PTR(pSvcCfg, uNAFEs, uDevIdx)      (&(pSvcCfg->pRawData[(pSvcCfg->uDataOffset + \
//                                                             (pSvcCfg->uBlockLen * (uNAFEs - uDevIdx - 1u)))]))

#define BQ7973x_GET_DATA_PTR(pSvcCfg,uNAFEs,uDevIdx) (&(pSvcCfg->pRawData[(pSvcCfg->uDataOffset + (pSvcCfg->uBlockLen * (uNAFEs - uDevIdx - 1u)))]))

                                                             
#define BQ7973X_STOP_SEC_CODE
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

#include "bq7973x_cfg.h"

#endif /*BQ7973X_H*/

/*********************************************************************************************************************
 * End of File: bq7973x.h
 *********************************************************************************************************************/
