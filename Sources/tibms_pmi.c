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
 *  File:       tibms_pmi.c
 *  Project:    TIBMS
 *  Module:     PMI
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for PMI interface
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       13July2022    SEM                 0000000000000    Initial version
 * 01.01.00       04Aug2023     SEM                 0000000000000    Pack monitor updates for including daisychain
 *                                                                   Pack monitor updates for read data
 *
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Standard Header Files
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Other Header Files
 *********************************************************************************************************************/

#include "tibms_api.h"
#include "tibms_pmi.h"
#include "tibms_comif.h"

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

/**	Major Software Config C Version number */
#define PMI_SW_C_MAJOR_VERSION             (0x01u)

/**	Minor Software Config C Version number */
#define PMI_SW_C_MINOR_VERSION             (0x01u)

/** Software Patch Config C Version number */
#define PMI_SW_C_PATCH_VERSION             (0x00u)

#if ((PMI_SW_C_MAJOR_VERSION != TIBMS_CFG_MAJOR_VERSION) || \
     (PMI_SW_C_MINOR_VERSION != TIBMS_CFG_MINOR_VERSION) || \
	 (PMI_SW_C_PATCH_VERSION != TIBMS_CFG_PATCH_VERSION))
#error "tibms: Config version numbers of tibms_pmi.c and tibms_cfg.h are inconsistent!"
#endif

#if ((PMI_SW_MAJOR_VERSION != PMI_SW_C_MAJOR_VERSION) || \
     (PMI_SW_MINOR_VERSION != PMI_SW_C_MINOR_VERSION) || \
	 (PMI_SW_PATCH_VERSION != PMI_SW_C_PATCH_VERSION))
#error "tibms: Config version numbers of tibms_pmi.c and tibms_pmi.h are inconsistent!"
#endif

/*********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

#define PMI_GUARD_START                         (0xBBA1AB1BU)
#define PMI_GUARD_END                           (0xBAFECEEBU)
#define PMI_INIT                                (0xCBU)

#define PMI_INTEGRITY_CHECK(pPmiMgr)            (((pPmiMgr) != NULL) && \
                                                 (PMI_GUARD_START == (pPmiMgr)->qGuardStart) && \
                                                 (PMI_GUARD_END == (pPmiMgr)->qGuardEnd) && \
                                                 (PMI_INIT == (pPmiMgr)->uInit))

/*********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

typedef struct Pmi_ManagerType_Tag
{
    uint32 qGuardStart;

    const InterfaceCfgType *pIfaceCfg;
    const Pmi_OperationType *pPmcOps;
    void *pPmicMgr;
    Comif_ManagerType *pComifMgrCtx;

    uint8 uInit;
    uint8 uWupReq;
    uint8 uResetReq;
    uint8 uPmiIface;

    uint32 qGuardEnd;
}
Pmi_ManagerType;

/*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

#define TIBMS_PMI_START_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

static Pmi_ManagerType zPmiManager[TIBMS_NO_OF_PMI_IFACES];

#if(TIMBS_PMI_EMEM_ENABLE == STD_ON)
static Emem_ManagerType zPmiEmemMgr;
#endif

#define TIBMS_PMI_STOP_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 *  Local CONST Object Definitions
 *********************************************************************************************************************/

/*********************************************************************************************************************
 *  Local Function Prototypes
 *********************************************************************************************************************/

/*********************************************************************************************************************
 *  Local Inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 *  Local Functions Definition
 *********************************************************************************************************************/

/*********************************************************************************************************************
 *  External Functions Definition
 *********************************************************************************************************************/

#define TIBMS_PMI_START_SEC_CODE
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_GetCurrent(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          Get pack current
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_GetCurrent(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_PACK_CURRENT, 0,(void *) NULL);


        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_GetVf(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          Get pack current
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_GetVf(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_VF_VOLT,0, (void *) NULL);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_GetCP(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_GetCP(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_CP_VOLT,0,(void *) NULL);
        }
    }

    return uRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_DiagServiceRequest(uint8 uPmiIfaceIn, uint8 uCmd, void *pData)
 *********************************************************************************************************************/
/*! \brief          Diagnostic Service Request
 *
 *  \param[in]      uPmiIfaceIn: corresponding Pmi interface
 *  \param[in]      uCmd: Request Command
 *  \param[inout]   pData: Data Request
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_DiagServiceRequest(uint8 uPmiIfaceIn, uint8 uCmd, void *pData)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIfaceIn];
    if(PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_DIAGNOSTICS, uCmd, pData);
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_GetCcAcc_Cnt(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_GetCcAcc_Cnt(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_CC_ACC_CNT,0, (void *) NULL);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_CcClear(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_CcClear(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_CC_CLEAR,0, (void *) NULL);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_GetI2CData(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_GetI2CData(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_GET_I2C_DATA,0, (void *) NULL);
        }
    }

    return uRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_GetCCOvf(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          This function is used to read the indicates a coulomb counter overflow fault.
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_GetCCOvf(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr,PMI_SERVICE_GET_CCOVF,0, (void *) NULL);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_SetI2CCtrl(uint8 uPmiIface, uint8 CtrlCmd)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_SetI2CCtrl(uint8 uPmiIface, uint8 CtrlCmd)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_SET_I2C_CTRL,CtrlCmd, (void *) NULL);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_SetI2CData(uint8 uPmiIface,uint8 uI2CData)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_SetI2CData(uint8 uPmiIface,uint8 uI2CData)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_SET_I2C_DATA,uI2CData, (void *) NULL);
        }
    }

    return uRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_EnableI2C(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_EnableI2C(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_ENABLE_I2C,0, (void *) NULL);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_WriteSPIData(uint8 uPmiIface,uint8 *uSPIData)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_WriteSPIData(uint8 uPmiIface,uint8 *uSPIData)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_WRITE_SPI_DATA,0, uSPIData);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_SetSPIExe(uint8 uPmiIface,uint8 uSPIExe)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_SetSPIExe(uint8 uPmiIface,uint8 uSPIExe)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_SET_SPI_EXE,uSPIExe, (void *) NULL);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_SetSPICtrl(uint8 uPmiIface,uint8 uSPICtrl)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_SetSPICtrl(uint8 uPmiIface,uint8 uSPICtrl)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_SET_SPI_CTRL,uSPICtrl, (void *) NULL);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_MSPIConf(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_MSPIConf(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_MSPI_CONF,0, (void *) NULL);
        }
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_EnADCFreeze(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_EnADCFreeze(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_EN_ADC_FREEZE,0, (void *) NULL);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_DisADCFreeze(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_DisADCFreeze(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_DIS_ADC_FREEZE,0, (void *) NULL);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_SetADCDelay(uint8 uPmiIface,uint16 xTimeUs)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_SetADCDelay(uint8 uPmiIface,uint16 xTimeUs)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_SET_ADC_DELAY,0u , &xTimeUs);
        }
    }

    return uRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_EnterCommDebug(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_EnterCommDebug(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_ENTER_COMM_DEBUG,0u , (void *) NULL);
        }
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_SetOCOutput(uint8 uPmiIface,uint8 uOCCtrl2)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_SetOCOutput(uint8 uPmiIface,uint8 uOCCtrl2)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_SET_OC_OUTPUT,uOCCtrl2 , (void *) NULL);
        }
    }

    return uRet;
}









/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_GetVGpio(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          Get CP Voltage
 *
 *  \param[in]      uPmiIface: corresponding pmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_GetVGpio(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_VGPIO, 0,(void *) NULL);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_ResetRequest(uint8 uPmiIface, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing the reset request
 *
 *  \param[in]      uPmiIface: corresponding Network Iface
 *  \param[in]      uResetReq: Request Type (soft , hard , Sleep2Wakeup )
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_ResetRequest(uint8 uPmiIface, uint8 uResetTyp)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if(PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_RESET, uResetTyp,(void *) NULL);         
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_HandleReqTransfer(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          Handle will be called once driver has data to send
 *
 *  \param[in]      uPmiIface: corresponding Network Iface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_HandleReqTransfer(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if(PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->handle_tx != NULL)
        {
            uRet = pPmiMgr->pPmcOps->handle_tx(pPmiMgr->pPmicMgr);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_HandleRecvComplete(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          Handle will be called once driver successfully recieve the complete packet
 *
 *  \param[in]      uPmiIface: corresponding Network Iface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_HandleRecvComplete(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if(PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->handle_rx != NULL)
        {
            uRet = pPmiMgr->pPmcOps->handle_rx(pPmiMgr->pPmicMgr);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_ProcessRxData(const ServiceDataType *, uint8 *, uint8 , uint16 )
 *********************************************************************************************************************/
/*! \brief          process the Rx Data recieved from the devices
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      pSvcCtx: Pointer to Service Data
 *  \param[in]      pRxData: Pointer to Response Data
 *  \param[in]      uRespStatus: status of the transfer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           This calls the applicatin layer callback function to process the recieved data
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_ProcessRxData(const ServiceCfgType *pSvcCfg, const uint8 *pRxData, uint8 uRespStatus)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[pSvcCfg->uIface];
    if (PMI_INTEGRITY_CHECK(pPmiMgr) && (pSvcCfg != NULL))
    {
        if(pPmiMgr->pPmcOps->process_rx != NULL)
        {
            uRet = pPmiMgr->pPmcOps->process_rx(pPmiMgr->pPmicMgr, pSvcCfg, pRxData, uRespStatus);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_GetPackCurrentData(uint8 uPmiIface, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the Rx Data recieved from the devices for Pack Current
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      uPmiIfaceIn: corresponding PMI interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           This calls the applicatin layer callback function to process the recieved data
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_GetPackCurrentData(uint8 uPmiIface, void *pBuffer, uint16 wLength)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if(PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        uRet = (uint8)PMI_INVALID_DECODE_BUF;
        if((pBuffer != NULL) && (wLength > 0U))
        {
            if(pPmiMgr->pPmcOps->decode_rx != NULL)
            {
                uRet = pPmiMgr->pPmcOps->decode_rx(pPmiMgr->pPmicMgr, PMI_SERVICE_PACK_CURRENT, pBuffer, wLength);
            }
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_GetVfVoltageData(uint8 uPmiIface, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the Rx Data recieved from the devices for VF Voltage
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      uPmiIfaceIn: corresponding PMI interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           This calls the applicatin layer callback function to process the recieved data
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_GetVfVoltageData(uint8 uPmiIface, void *pBuffer, uint16 wLength)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if(PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        uRet = (uint8)PMI_INVALID_DECODE_BUF;
        if((pBuffer != NULL) && (wLength > 0U))
        {
            if(pPmiMgr->pPmcOps->decode_rx != NULL)
            {
                uRet = pPmiMgr->pPmcOps->decode_rx(pPmiMgr->pPmicMgr, PMI_SERVICE_VF_VOLT, pBuffer, wLength);
            }
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_GetCfVoltageData(uint8 uPmiIface, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the Rx Data recieved from the devices for VF Voltage
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      uPmiIfaceIn: corresponding PMI interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           This calls the applicatin layer callback function to process the recieved data
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_GetCfVoltageData(uint8 uPmiIface, void *pBuffer, uint16 wLength)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if(PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        uRet = (uint8)PMI_INVALID_DECODE_BUF;
        if((pBuffer != NULL) && (wLength > 0U))
        {
            if(pPmiMgr->pPmcOps->decode_rx != NULL)
            {
                uRet = pPmiMgr->pPmcOps->decode_rx(pPmiMgr->pPmicMgr, PMI_SERVICE_CP_VOLT, pBuffer, wLength);
            }
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_GetCfVoltageData(uint8 uPmiIface, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the Rx Data recieved from the devices for VF Voltage
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      uPmiIfaceIn: corresponding PMI interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           This calls the applicatin layer callback function to process the recieved data
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_GetCcAccCntData(uint8 uPmiIface, void *pBuffer, uint16 wLength)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if(PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        uRet = (uint8)PMI_INVALID_DECODE_BUF;
        if((pBuffer!= NULL) && (wLength > 0U))
        {
            if(pPmiMgr->pPmcOps->decode_rx != NULL)
            {
                uRet = pPmiMgr->pPmcOps->decode_rx(pPmiMgr->pPmicMgr, PMI_SERVICE_CC_ACC_CNT, pBuffer, wLength);
            }
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_ProcessNotify(uint8 uPmiIface, uint8 uType)
 *********************************************************************************************************************/
/*! \brief          Process the notification recieved from the external interface
 *
 *  \param[in]      uPmiIface: corresponding Network Iface
 *  \param[in]      uType: Notification information
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_ProcessNotify(uint8 uPmiIface, uint8 uType)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if(PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->notify != NULL)
        {
            uRet = pPmiMgr->pPmcOps->notify(pPmiMgr->pPmicMgr, uType);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_GetDriverState(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          Initialization during startup
 *
 *  \param[in]      uPmiIface
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           Pmi_StartupInit will be completed after PMIC returns successfull and transfer to into Normal mode
 *  \return         returns the eDriver State
 *  \retval
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_GetDriverState(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if(PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->ioctl != NULL)
        {
            uRet = pPmiMgr->pPmcOps->ioctl(pPmiMgr->pPmicMgr, PMI_GET_DRIVER_STATE, NULL);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_ConfigInit(uint8 uPmiIface, uint8 eNotify)
 *********************************************************************************************************************/
/*! \brief          Initialization during startup
 *
 *  \param[in]      uPmiIface
 *  \param[in]      eNotify - Notification Method after completing the Init
 *                  INIT_NOTIFY_BLOCKING
 *                  INIT_NOTIFY_CALLBACK
 *                  INIT_NOTIFY_POLLING
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           Pmi_StartupInit will be completed after PMIC returns successfull and transfer to into Normal mode
 *  \return         status of the initialization
 *  \retval         PMI_OK: Successful Reset Request Init
 *                  PMI_NOT_OK: Failed to initiate startup
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_ConfigInit(uint8 uPmiIface, uint8 eNotify)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if(PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_CONFIG, 0, (void *) &eNotify);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_StartupInit(uint8 uPmiIface, uint8 eNotify)
 *********************************************************************************************************************/
/*! \brief          Initialization during startup
 *
 *  \param[in]      uPmiIface
 *  \param[in]      eNotify - Notification Method after completing the Init
 *                  INIT_NOTIFY_BLOCKING
 *                  INIT_NOTIFY_CALLBACK
 *                  INIT_NOTIFY_POLLING
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           Pmi_StartupInit will be completed after PMIC returns successfull and transfer to into Normal mode
 *  \return         status of the initialization
 *  \retval         PMI_OK: Successful Reset Request Init
 *                  PMI_NOT_OK: Failed to initiate startup
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_StartupInit(uint8 uPmiIface, uint8 eNotify)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if(PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmiMgr->pPmcOps->service != NULL)
        {
            uRet = pPmiMgr->pPmcOps->service(pPmiMgr->pPmicMgr, PMI_SERVICE_STARTUP, 0, (void *) &eNotify);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_SetErrorDetails(uint8 uSvcId, uint8 uErrorId, uint32 qParam1, uint32 qParam2)
 *********************************************************************************************************************/
/*! \brief          This function is Set the Error Memory Details
 *
 *
 *  \param[in]       uSvcId
 *  \param[in]       uErrorId
 *  \param[in]       param_1
 *  \param[in]       param_2
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *
 *  \return         Set the Error Memory Information
 *  \retval         PMI_OK: Successful
 *                  PMI_NOT_OK: Failed
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_SetErrorDetails(uint8 uSvcId, uint8 uErrorId, uint32 qParam1, uint32 qParam2)
{
#if(TIMBS_PMI_EMEM_ENABLE == STD_ON)
    Emem_EntryType *pEmemEntry;
    uint8 uErrIdx;
    uint8 uEMIdOcpd;

    if((uErrorId > PMI_OK) && (uErrorId < PMI_INIT_PROGRESS))
    {
        if(ERRMEM_STATE_INIT == zPmiEmemMgr.uEmemState)
        {
            uEMIdOcpd = zPmiEmemMgr.uEmemSiz;

            zPmiEmemMgr.resource_req(zPmiEmemMgr.qEmemRes, GET_RESOURCE);
            for(uErrIdx = 0; uErrIdx < zPmiEmemMgr.uEmemSiz; uErrIdx++)
            {
                if((uErrorId == zPmiEmemMgr.pEmemEntry[uErrIdx].uErrorId) &&
                   (uSvcId == zPmiEmemMgr.pEmemEntry[uErrIdx].eServiceId))
                {
                    uEMIdOcpd = uErrIdx;
                }
            }

            if(zPmiEmemMgr.uEmemSiz == uEMIdOcpd)
            {
                pEmemEntry = &zPmiEmemMgr.pEmemEntry[zPmiEmemMgr.uEmemIdx++];
                if(pEmemEntry != NULL)
                {
                    pEmemEntry->eServiceId = uSvcId;
                    pEmemEntry->uErrorId = uErrorId;
                    pEmemEntry->qParam1 = qParam1;
                    pEmemEntry->qParam2 = qParam2;
                }
            }
            else
            {
                pEmemEntry = &zPmiEmemMgr.pEmemEntry[uEMIdOcpd];
                if(pEmemEntry != NULL)
                {
                    pEmemEntry->nError++;
                }
            }

            if(zPmiEmemMgr.uEmemIdx == zPmiEmemMgr.uEmemSiz)
            {
                zPmiEmemMgr.uEmemIdx = 0;
            }

            zPmiEmemMgr.resource_req(zPmiEmemMgr.qEmemRes, RELEASE_RESOURCE);
        }
    }
#endif

     return EMEM_ERRORSTATUS2USERSTATUS(uErrorId, ((uErrorId > PMI_OK) && (uErrorId < PMI_INIT_PROGRESS)));
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_ErrMemoryInit(void)
 *********************************************************************************************************************/
/*! \brief          Error Memory Init
 *
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *
 *  \return         status of version information
 *  \retval         PMI_OK: Successful
 *                  PMI_NOT_OK: Failed
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_ErrMemoryInit(void)
{
    uint8 uRet = (uint8)PMI_NOT_OK;

#if(TIMBS_PMI_EMEM_ENABLE == STD_ON)
    if(zPmiEmemConfig.pEmemEntry != NULL)
    {
        zPmiEmemMgr.resource_req = zPmiEmemConfig.resource_req;
        zPmiEmemMgr.qEmemRes = zPmiEmemConfig.qEmemRes;

        zPmiEmemMgr.pEmemEntry = zPmiEmemConfig.pEmemEntry;
        zPmiEmemMgr.uEmemIdx = 0u;
        zPmiEmemMgr.uEmemSiz = zPmiEmemConfig.wEmemEntrySiz;

        uRet = (uint8)PMI_OK;
    }
#else
    uRet = PMI_OK;
#endif

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_Init(uint8 uCfgSet, uint8 uWupReq)
 *********************************************************************************************************************/
/*! \brief          This function is used to init the Pmi core for all chains in the config set
 *
 *  \param[in]      uCfgSet
 *  \param[in]      uWupReq
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         status of the initialization
 *  \retval         PMI_OK: Successful init
 *                  PMI_NOT_OK: Failed to init
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_Init(const Bms_ConfigType *pConfigSet_PB, Comif_ManagerType *pComifMgr, uint8 uWupReq)
{
    Pmi_ManagerType *pPmiMgr;
    const Pmi_ConfigType *pPmiCfg;
    uint8 uRet = (uint8)PMI_INIT_UNINIT;

    do
    {
        #if(TIMBS_PMI_EMEM_ENABLE == STD_ON)
        uRet = Pmi_ErrMemoryInit();
        if(PMI_NOT_OK == uRet)
        {
            uRet = (uint8)PMI_INVALID_EMEM_CFG;
            break;
        }
        #endif

        if((NULL == pConfigSet_PB) ||
           (TIBMS_CFG_START_SIG != pConfigSet_PB->qStartSig) ||
           (TIBMS_CFG_STOP_SIG != pConfigSet_PB->qEndSig))
        {
            uRet = (uint8)PMI_ILLEGAL_CONFIG;
            break;
        }

        pPmiCfg = pConfigSet_PB->pIfaceCfg->pPmiCfg;
        if(pPmiCfg->uPmiIfaceCfg >= TIBMS_NO_OF_PMI_IFACES)
        {
            uRet = (uint8)PMI_INVALID_CONFIG;
            break;
        }

        pPmiMgr = &zPmiManager[pPmiCfg->uPmiIfaceCfg];
        if((pPmiMgr != NULL) && (PMI_INIT != pPmiMgr->uInit))
        {
            memset(pPmiMgr, 0, sizeof(Pmi_ManagerType));

            pPmiMgr->uPmiIface = (uint8)pPmiCfg->uPmiIfaceCfg;
            pPmiMgr->uWupReq = uWupReq;
            pPmiMgr->pIfaceCfg = pConfigSet_PB->pIfaceCfg;

            if(NULL == pPmiMgr->pIfaceCfg)
            {
                uRet = (uint8)PMI_INVALID_CHAIN_CFG;
                break;
            }

            pPmiMgr->pPmcOps = pPmiMgr->pIfaceCfg->pPmiCfg->pPmcOpsCfg;
            if(NULL == pPmiMgr->pPmcOps)
            {
                uRet = (uint8)PMI_INVALID_OPS_CFG;
                break;
            }

            if(NULL == pComifMgr)
            {
                pPmiMgr->pComifMgrCtx = Comif_Init(pPmiMgr->pIfaceCfg->pComIfCfg, NULL); // TBD 
                if(NULL == pPmiMgr->pComifMgrCtx)
                {
                    uRet = (uint8)PMI_COMIF_INIT_FAILED;
                    break;
                }
            }
            else
            {
                pPmiMgr->pComifMgrCtx = pComifMgr;
            }

            pPmiMgr->qGuardStart = PMI_GUARD_START;
            pPmiMgr->qGuardEnd = PMI_GUARD_END;
            pPmiMgr->uInit = PMI_INIT;

//#if BMS_CONFIG_IFACE_ENABLE == STD_ON
            pPmiMgr->pPmicMgr = pPmiMgr->pPmcOps->init(pPmiMgr->pIfaceCfg, pPmiMgr->pComifMgrCtx, uWupReq);
            if(NULL == pPmiMgr->pPmicMgr)
            {
                uRet = (uint8)PMI_PMC_INIT_FAILED;
                break;
            }
//#endif

        }
        else
        {
            uRet = (uint8)PMI_ALREADY_INIT;
            break;
        }

        uRet = (uint8)PMI_INIT_SUCCESS;
	}
    while(0);

    return Pmi_SetErrorDetails(PMI_SVCID_INIT, uRet, 0u, 0u);
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_pmi_CODE) Pmi_Deinit(uint8 uPmiIface)
 *********************************************************************************************************************/
/*! \brief          This function is used to De-init the Pmi core which is already initilized
 *
 *
 *  \param[in]      uPmiIface
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            Already Initialized by Pmi_Init
 *  \post
 *
 *  \return         status of the de-initialization
 *  \retval         PMI_OK: Successful De-init
 *                  PMI_NOT_OK: Failed to De-init
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_Deinit(uint8 uPmiIface)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if(PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        // TBD:
    	uRet = (uint8)PMI_OK;
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, tibms_pmi_CODE) Pmi_GetVersionInfo(uint8 uPmiIface, PMI_VersionInfoType *pPmifVersion)
 *********************************************************************************************************************/
/*! \brief          This function is used to De-init the PMI core which is already initilized
 *
 *
 *  \param[in]       uPmiIface
 *  \param[out]      pPmifVersion
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *
 *  \return         status of version information
 *  \retval         PMI_OK: Successful
 *                  PMI_NOT_OK: Failed
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_pmi_CODE) Pmi_GetVersionInfo(uint8 uPmiIface, VersionInfoType *pPmifVersion)
{
    Pmi_ManagerType *pPmiMgr;
    uint8 uRet = (uint8)PMI_NOT_OK;

    pPmiMgr = &zPmiManager[uPmiIface];
    if(PMI_INTEGRITY_CHECK(pPmiMgr))
    {
        if(pPmifVersion != NULL)
        {
            uRet = pPmiMgr->pPmcOps->ioctl(pPmiMgr->pPmicMgr, PMI_GET_VERSION, pPmifVersion);
            if(PMI_OK == uRet)
            {
                pPmifVersion->zFwVersion.vendorID = TIBMS_VENDOR_ID;
                pPmifVersion->zFwVersion.moduleID = TIBMS_MODULE_ID;

                pPmifVersion->zFwVersion.sw_major_version = TIBMS_API_MAJOR_VERSION;
                pPmifVersion->zFwVersion.sw_minor_version = TIBMS_API_MINOR_VERSION;
                pPmifVersion->zFwVersion.sw_patch_version = TIBMS_API_PATCH_VERSION;

                pPmifVersion->zPmiVersion.vendorID = PMI_VENDOR_ID;
                pPmifVersion->zPmiVersion.moduleID = PMI_MODULE_ID;
                pPmifVersion->zPmiVersion.sw_major_version = PMI_SW_MAJOR_VERSION;
                pPmifVersion->zPmiVersion.sw_minor_version = PMI_SW_MINOR_VERSION;
                pPmifVersion->zPmiVersion.sw_patch_version = PMI_SW_PATCH_VERSION;

                uRet = (uint8)PMI_OK;
            }
        }
    }

    return uRet;
}


#define TIBMS_PMI_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * End of File: tibms_pmi.c
 *********************************************************************************************************************/
