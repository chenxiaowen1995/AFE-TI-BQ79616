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
 *  File:       bq79600.h
 *  Project:    TIBMS
 *  Module:     COMIF
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
 * 01.00.00       05May2022    SEM                  0000000000000    Initial version
 *
 *********************************************************************************************************************/

#ifndef BQ79600_H
#define BQ79600_H

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

#define BQ79600_SW_MAJOR_VERSION                (0x01u)
#define BQ79600_SW_MINOR_VERSION                (0x00u)
#define BQ79600_SW_PATCH_VERSION                (0x01u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

/** Texas Instruments Vendor ID*/
#define BQ79600_VENDOR_ID                       (44u)

/** BQ79600 Driver Module ID       */
#define BQ79600_MODULE_ID                       (255u)

/** BQ79600 Driver Instance ID */
#define BQ79600_INSTANCE_ID                     (31u)

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

#define BQ79600_START_SEC_CODE
#include "Cdd_MemMap.h"

FUNC(uint8, BQ79600_CODE) Bq79600_HandleTransfer(void *pComifCtx);
FUNC(uint8, BQ79600_CODE) Bq79600_HandleRecieve(void *pComifCtx);
FUNC(uint8, bq79600_CODE) Bq79600_Notify(void *pComifCtx, uint8 uNotifyType);
FUNC(uint8, bq79600_CODE) Bq79600_Control(void *pComifCtx, uint8 uCmd, void *pData);
FUNC(void*, bq79600_CODE) Bq79600_Init(const void *pComCfgCtx, Emem_StasticsType *pEmemStatsCfg,
                                       const ServiceCfgType *pCommCfg);
FUNC(uint8, bq79600_CODE) Bq79600_Deinit(void *pComifCtx);

#define BQ79600_STOP_SEC_CODE
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

#include "bq79600_cfg.h"

#endif /*BQ79600_H*/

/*********************************************************************************************************************
 * End of File: bq79600.h
 *********************************************************************************************************************/
