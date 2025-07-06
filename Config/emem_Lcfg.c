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
 *  File:       emem_Lcfg.c
 *  Project:    TIBMS
 *  Module:     CFG
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Linktime configuration for TI BMS interface 0
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

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

/**	Major Software Config C Version number */
#define EMEM_LCFG_C_MAJOR_VERSION             (0x01u)

/**	Minor Software Config C Version number */
#define EMEM_LCFG_C_MINOR_VERSION             (0x00u)

/** Software Patch Config C Version number */
#define EMEM_LCFG_C_PATCH_VERSION             (0x00u)

#if (  (EMEM_CFG_MAJOR_VERSION != EMEM_LCFG_C_MAJOR_VERSION) \
    || (EMEM_CFG_MINOR_VERSION != EMEM_LCFG_C_MINOR_VERSION) \
	|| (EMEM_CFG_PATCH_VERSION != EMEM_LCFG_C_PATCH_VERSION))
#error "tibms: Config version numbers of emem_Lcfg.c and emem_cfg.h are inconsistent!"
#endif

/*********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

#define TIBMS_LCFG_0_START_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

#define TIBMS_LCFG_0_STOP_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

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

/**********************************************************************************************************************
 * uint32 EmemResourceRequest(uint32 qReq, uint32 qResId)
 *********************************************************************************************************************/
/*! \brief          This function is Transfer the Tx Data
 *
 *  \param[in]       qReq: Request type
 *  \param[in]       qResId: Resource to set
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            None
 *  \post           None
 *  \return         uint32
 *  \retval         E_OK: Successfull return if the transfer request is accepeted
 *                  E_NOT_OK: Failed for inserting transfer request
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint32, bsw_CODE) EmemResourceRequest(uint32 qResId, uint32 qReq)
{
    uint32 qRet = E_NOT_OK;

    switch(qReq)
    {
        case GET_RESOURCE:
        {
            GetResource(qResId);
            qRet = E_OK;
            break;
        }
        case RELEASE_RESOURCE:
        {
            ReleaseResource(qResId);
            qRet = E_OK;
            break;
        }
        default:
        {
            break;
        }
    }

    return qRet;
}

/**********************************************************************************************************************
 * uint32 EmemResourceRequest(uint32 qReq, uint32 qResId)
 *********************************************************************************************************************/
/*! \brief          This function is Transfer the Tx Data
 *
 *  \param[in]       qReq: Request type
 *  \param[in]       qResId: Resource to set
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            None
 *  \post           None
 *  \return         uint32
 *  \retval         E_OK: Successfull return if the transfer request is accepeted
 *                  E_NOT_OK: Failed for inserting transfer request
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(void, bsw_CODE) EmemReportError(uint16 ModuleId, uint8 InstanceId, uint8 ApiId, uint8 ErrorId)
{
#if(EMEM_ERROR_HOOK_ENABLED == STD_ON )
    configASSERT(0);
#endif
}

/*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

#if (TIMBS_BMI_EMEM_ENABLE == STD_ON)

#define TIBMS_LCFG_0_START_SEC_VAR_NOINIT_UNSPECIFIED 
#include "Cdd_MemMap.h"

Emem_EntryType zBmiEmemEntry[EMEM_BMI_LOG_MAX];
Emem_EntryType zPmiEmemEntry[EMEM_PMI_LOG_MAX];
Emem_EntryType zComifEmemEntry[EMEM_COMIF_LOG_MAX];
Emem_StasticsType zComifEmemStats[TIBMS_NO_OF_COMIF_IFACES];
#define TIBMS_LCFG_0_STOP_SEC_VAR_NOINIT_UNSPECIFIED 
#include "Cdd_MemMap.h"


#define TIBMS_LCFG_0_START_SEC_CONST
#include "Cdd_MemMap.h"


CONST(Emem_ConfigType, TIBMSCFG_CONST) zBmiEmemConfig =
{
    ResID_BmiEmemLog,
    EmemResourceRequest,
    EmemReportError,

    zBmiEmemEntry,
    NULL,

    EMEM_BMI_LOG_MAX,
    0u
};

#define TIBMS_LCFG_0_STOP_SEC_CONST
#include "Cdd_MemMap.h"
#endif

#if (TIMBS_PMI_EMEM_ENABLE == STD_ON)

#define TIBMS_LCFG_0_START_SEC_CONST
#include "Cdd_MemMap.h"


CONST(Emem_ConfigType, TIBMSCFG_CONST) zPmiEmemConfig =
{
    ResID_PmiEmemLog,
    EmemResourceRequest,
    EmemReportError,

    zPmiEmemEntry,
    NULL,

    EMEM_BMI_LOG_MAX,
    0u
};

#define TIBMS_LCFG_0_STOP_SEC_CONST
#include "Cdd_MemMap.h"

#endif

#if (TIMBS_COMIF_EMEM_ENABLE == STD_ON)

#define TIBMS_LCFG_0_START_SEC_CONST
#include "Cdd_MemMap.h"


CONST(Emem_ConfigType, TIBMSCFG_CONST) zComifEmemConfig =
{
    ResID_ComifEmemLog,
    EmemResourceRequest,
    EmemReportError,

    zComifEmemEntry,
    zComifEmemStats,

    EMEM_COMIF_LOG_MAX,
    EMEM_COMIF_STATS_MAX
};

#define TIBMS_LCFG_0_STOP_SEC_CONST
#include "Cdd_MemMap.h"

#endif

/*********************************************************************************************************************
 * External Functions Definition
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * End of File: emem_Lcfg.c
 *********************************************************************************************************************/
