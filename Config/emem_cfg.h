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
 *  File:       emem_cfg.h
 *  Project:    TIBMS
 *  Module:     CONFIG
 *  Generator:  Code generation tool (if any)
 *
 *  Description: Configuration for Error Memory
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05May2022    SEM                  0000000000000    Initial version
 * 01.01.00       31Oct2023    SEM                  0000000000000    Updates on Emem Statistics
 *
 *********************************************************************************************************************/

#ifndef ERROR_MEMORY_CFG_H
#define ERROR_MEMORY_CFG_H

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

/**	Major Software Config Version number */
#define EMEM_CFG_MAJOR_VERSION                                  (0x01u)

/**	Minor Software Config Version number */
#define EMEM_CFG_MINOR_VERSION                                  (0x00u)

/** Software Patch Config Version number */
#define EMEM_CFG_PATCH_VERSION                                  (0x00u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

#define TIMBS_BMI_EMEM_ENABLE                                   (STD_ON)
#define TIMBS_PMI_EMEM_ENABLE                                   (STD_ON)
#define TIMBS_COMIF_EMEM_ENABLE                                 (STD_ON)

#define ERRMEM_STATE_INIT                                       (0xBE)

#define EMEM_ERRORSTATUS2USERSTATUS(uError, cond)               (cond)

#define EMEM_PARAM1_SET_U8(p1, p2, p3, p4)                      ((p1 << (8*3)) | (p2 << (8*2)) | (p3 << (8*1)) | (p1 << (8*0)))
#define EMEM_PARAM1_GET_U8(data)                                ((data >> (8*3)) & 0xFF)
#define EMEM_PARAM2_GET_U8(data)                                ((data >> (8*2)) & 0xFF)
#define EMEM_PARAM3_GET_U8(data)                                ((data >> (8*1)) & 0xFF)
#define EMEM_PARAM4_GET_U8(data)                                ((data >> (8*0)) & 0xFF)

#define EMEM_PARAM1_SET_U16(p1, p2)                             ((p1 << (16*1)) | (p2 << (16*0)))
#define EMEM_PARAM1_GET_U16(data)                               ((data >> (16u * 1)) & 0xFFFF)
#define EMEM_PARAM2_GET_U16(data)                               ((data >> (16u * 0)) & 0xFFFF)

#define EMEM_PARAM1_SET_U32(p1)                                 ((p1 >> (32u * 0)) & 0xFFFFFFFF)
#define EMEM_PARAM1_GET_U32(data)                               ((param_1 >> (32u * 0)) & 0xFFFFFFFF)

#if(TIMBS_COMIF_EMEM_ENABLE == STD_ON)
    #define EMEM_UPDATE_STATICS(pStatics, param)                (pStatics->param++)
    #define EMEM_UPDATE_STATICS_DATA(pStatics, param, qData)    (pStatics->param = qData)
    #define EMEM_UPDATE_STATICS_DATAI(pStatics, param, qData)   (pStatics->param += qData)
#else
    #define EMEM_UPDATE_STATICS(pStatics, param)
    #define EMEM_UPDATE_STATICS_DATA(pStatics, param, qData)
    #define EMEM_UPDATE_STATICS_DATAI(pStatics, param, qData)
#endif

#define EMEM_BMI_LOG_MAX                                        (8u)
#define EMEM_PMI_LOG_MAX                                        (8u)
#define EMEM_COMIF_LOG_MAX                                      (8u)
#define EMEM_COMIF_STATS_MAX                                    (TIBMS_NO_OF_COMIF_IFACES)

#define EMEM_ERROR_HOOK_ENABLED                                 (STD_ON)

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef enum Emem_ModuleType_Tag
{
    EMEM_MODULE_BMI,
    EMEM_MODULE_PMI,
    EMEM_MODULE_COMIF,
    EMEM_MODULE_MAX
}
Emem_ModuleType;

struct Emem_StasticsType_Tag
{
    uint32 qNTxReqs;
    uint32 qNRxResps;
    uint32 qNCmdAggr;

    uint16 qNReTxs;
    uint16 qNReTxsRetry;
    uint16 qNProtReTxs;
    uint16 qNFCSFails;

    uint16 qNCrcFails;
    uint16 qNComRec;
    uint16 qNReqQueFail;
    uint16 qQueMaxUsage;
};

typedef struct Emem_EntryType_Tag
{
    uint8 eModuleId;
    uint8 eServiceId;
    uint8 uErrorId;
    uint8 nError;

    uint32 qParam1;
    uint32 qParam2;
}
Emem_EntryType;

typedef struct Emem_ManagerType_Tag
{
    uint32 qEmemRes;

    Emem_EntryType *pEmemEntry;
    uint32 (*resource_req)(uint32 qResId, uint32 qReq);
    void (*report_error)(uint16 ModuleId, uint8 InstanceId, uint8 ApiId, uint8 ErrorId);

    uint8 uEmemIdx;
    uint8 uEmemSiz;
    uint8 uEmemState;
    uint8 uEmemOv;
}
Emem_ManagerType;

typedef struct Emem_ConfigType_Tag
{
    uint32 qEmemRes;
    uint32 (*resource_req)(uint32 qResId, uint32 qReq);
    void (*report_error)(uint16 ModuleId, uint8 InstanceId, uint8 ApiId, uint8 ErrorId);

    Emem_EntryType *pEmemEntry;
    Emem_StasticsType *pEmemStats;

    uint16 wEmemEntrySiz;
    uint16 wEmemStaticsSiz;
}
Emem_ConfigType;

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

#define TIBMS_CFG_START_SEC_CONST
#include "Cdd_MemMap.h"

extern CONST(Emem_ConfigType, TIBMSCFG_CONST) zBmiEmemConfig;
extern CONST(Emem_ConfigType, TIBMSCFG_CONST) zPmiEmemConfig;
extern CONST(Emem_ConfigType, TIBMSCFG_CONST) zComifEmemConfig;

#define TIBMS_CFG_STOP_SEC_CONST
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /**ERROR_MEMORY_CFG_H**/

/*********************************************************************************************************************
 * End of File: errMem_cfg.h
 *********************************************************************************************************************/
