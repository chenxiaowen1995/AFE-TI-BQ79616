/**********************************************************************************************************************
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
 *  File:       bq7973x_funcs.c
 *  Project:    TIBMS
 *  Module:     PMI
 *  Generator:  Code generation tool (ifany)
 *
 *  Description:  Exposed functionalities forPMI interface
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

/**********************************************************************************************************************
 * Standard Header Files
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Other Header Files
 *********************************************************************************************************************/

#include "tibms_api.h"
#include "tibms_comif.h"
#include "tibms_pmi.h"
#include "bq7973x_diag.h"
/**********************************************************************************************************************
 * Version Check (ifrequired)
 *********************************************************************************************************************/

/**	Major Software Config C Version number */
#define BQ7973X_FUNC_C_MAJOR_VERSION             (0x01u)

/**	Minor Software Config C Version number */
#define BQ7973X_FUNC_C_MINOR_VERSION             (0x00u)

/** Software Patch Config C Version number */
#define BQ7973X_FUNC_C_PATCH_VERSION             (0x00u)

#if ((BQ7973X_FUNC_C_MAJOR_VERSION != BQ7973X_CFG_MAJOR_VERSION) || \
     (BQ7973X_FUNC_C_MINOR_VERSION != BQ7973X_CFG_MINOR_VERSION) || \
	 (BQ7973X_FUNC_C_PATCH_VERSION != BQ7973X_CFG_PATCH_VERSION))
#error "tibms: Config version numbers of bq7973x_funcs.c and bq7973x_cfg.h are inconsistent!"
#endif

#if ((BQ7973X_SW_MAJOR_VERSION != BQ7973X_FUNC_C_MAJOR_VERSION) || \
     (BQ7973X_SW_MINOR_VERSION != BQ7973X_FUNC_C_MINOR_VERSION) || \
	 (BQ7973X_SW_PATCH_VERSION != BQ7973X_FUNC_C_PATCH_VERSION))
#error "tibms: Config version numbers of bq7973x_funcs.c and bq7971x.h are inconsistent!"
#endif

#if ((BQ7973X_REGS_MAJOR_VERSION != BQ7973X_FUNC_C_MAJOR_VERSION) || \
     (BQ7973X_REGS_MINOR_VERSION != BQ7973X_FUNC_C_MINOR_VERSION) || \
	 (BQ7973X_REGS_PATCH_VERSION != BQ7973X_FUNC_C_PATCH_VERSION))
#error "tibms: Config version numbers of bq7973x_funcs.c and bq7973x_regs.h are inconsistent!"
#endif

/**********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/
#define BQ7973X_OC_UNLOCKCODE1               (0xCBu)
#define BQ7973X_OC_UNLOCKCODE2               (0xF6u)
#define BQ7973X_GUARD_START                             (0xCCA1AB1FU)
#define BQ7973X_GUARD_END                               (0xCAFECEEEU)
#define BQ7973X_INIT                                    (0xCAU)

#define BQ7973X_INTEGRITY_CHECK(pPmicMgr)                ((NULL != (pPmicMgr)) && \
                                                         (BQ7973X_GUARD_START == pPmicMgr->qGuardStart) && \
                                                         (BQ7973X_GUARD_END == pPmicMgr->qGuardEnd) && \
                                                         (BQ7973X_INIT == pPmicMgr->uInit))

#define BQ7973X_DEBUG_CTRL_UNLOCKCODE        (0xA5u)
#define BQ7973X_DIAG_D1D2_SEL_AVDD           (6u)
#define BQ7973X_SLP2ACT_TONE_DELAY           (1u)   /* 1000us */
#define BQ7973X_ADC_FREEZE_MSK               (0xFEu)
#define BQ7973X_ADC_UNFREEZE_MSK             (0x7Eu)
#define BQ7973X_MAX_ADC_DLY                  (1020u)
#define BQ7973X_UNIT_ADC_DLY                 (4u)

#define BQ7973X_ADC_FREEZE_MSK               (0xFEu)
#define BQ7973X_ADC_UNFREEZE_MSK             (0x7Eu)
/**********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

typedef enum DriverStateType_Tag
{
    BQ7973X_STATE_UINIT,
    BQ7973X_STATE_WAKEUP,
    BQ7973X_STATE_INIT,
    BQ7973X_STATE_NORMAL,
    BQ7973X_STATE_DIAG,
    BQ7973X_STATE_ERROR,
}
DriverStateType;

struct Bq7973x_ManagerType_Tag
{
    uint32 qGuardStart;

    const Bq7973x_ConfigType *pPmcConfig;
    const ServiceCfgType *pFuncCfg;
    const Basic_ConfigType *pBswCfg;
    Comif_ManagerType *pComifCtx;

    Bq7973x_DiagType *pPmicDiag;
    uint32 qApplRes;

    uint8 eDrvState;
    uint8 uInit;
    uint8 uWupReq;
    uint8 uPmicIface;
	uint8 uNAFEs;

    uint8 uIsPmicInIface;
    uint8 eCommDir;
    uint8 uPmicDevId;
    uint8 uRsvd;

/* Diag_Update_Start */
    uint8 uSDevId;
    uint8 uNPmics;
    uint8 eComType ; 
/* Diag_Update_END */
    uint32 qGuardEnd;
};

/**********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

#define BQ7973X_FUNCS_START_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

static Bq7973x_ManagerType zBq7973xMgr[BQ7973X_IFACES];
#define BQ7973X_FUNCS_STOP_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

/***********************************************************************************************************************
 *  Local CONST Object Definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  Local Function Prototypes
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7971x_CODE) BQ7973X_DevPowerManager(Bq7973x_ManagerType *pPmicMgr, uint8 uResetTyp);
STATIC FUNC(uint8, bq7973x_CODE) BQ7973X_GetVRrfCap(Bq7973x_ManagerType *pPmicMgr,const ServiceCfgType *pSvcCfg );
FUNC(uint8, bq7973x_CODE) BQ7973X_OCCtrl1Init(Bq7973x_DiagType *pPmicMgr ,const ServiceCfgType *pSvcCfg);
/**********************************************************************************************************************
 *  Local Inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  Local Functions Definition
*********************************************************************************************************************/

#define BQ7973X_FUNCS_START_SEC_CODE
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 * Function name: Bq7973x_ConfigInit
 *********************************************************************************************************************/
/** \brief This function is used to
 *
 * Extended function description
 *
 * \param[in]
 * \param[in]
 * \param[in]
 *
 * \pre Preconditions
 * \post Postconditions
 * \return Bq7973x_ManagerType
 * \retval NULL iferror
 *         Valid pointer on success
 *
 *********************************************************************************************************************/
extern Bq7973x_DiagType zBq7973xDiagMgr[BQ7973X_IFACES];
STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_ConfigInit(Bq7973x_ManagerType *pPmicMgr)
{
    uint8 uRet;

    do
    {
        uRet = Bq7973x_Control2Init(pPmicMgr);
        if((uint8)PMI_OK != uRet)
        {
            uRet = (uint8)PMI_PMC_CTRL2_FAILED;
            break;
        }

        uRet = Bq7973x_GpioConfInit(pPmicMgr, &uRet);          // Temp, need to modify the argument
        if((uint8)PMI_OK != uRet)
        {
            uRet = (uint8)PMI_PMC_GPIOCFG_FAILED;
            break;
        }

        uRet = Bq7973x_SetOCConf(pPmicMgr, &uRet);           // Temp, need to modify the argument
        if((uint8)PMI_OK != uRet)
        {
            uRet = (uint8)PMI_PMC_SETOC_FAILED;
            break;
        }

        uRet = Bq7973x_AdcCtrlInit(pPmicMgr);
        if((uint8)PMI_OK != uRet)
        {
            uRet = (uint8)PMI_PMC_ADCCTRL_FAILED;
            break;
        }

        uRet = Bq7973x_CCCtrlInit(pPmicMgr);
        if((uint8)PMI_OK != uRet)
        {
            uRet = (uint8)PMI_PMC_CCCTRL_FAILED;
            break;
        }

        uRet = Bq7973x_SetSWInit(pPmicMgr, 0u);
        if((uint8)PMI_OK != uRet)
        {
            uRet = (uint8)PMI_PMC_SETSW_FAILED;
            break;
        }

        uRet = Bq7973x_SetOCCtrl1(pPmicMgr, 0xB);
        if((uint8)PMI_OK != uRet)
        {
            uRet = (uint8)PMI_PMC_OCCTRL1_FAILED;
            break;
        }
        //uRet = Bq7973x_DiagServiceReq(pPmicMgr->pPmicDiag, PMI_MPFDI_STARTUPINIT, NULL);
        //if(PMI_OK != uRet)
        //{
        //    uRet = BMI_BMC_DIAG_STARTUP_ERROR;
        //    break;
        //}
        // TBD: Diagnostic Init Call with Custom CRC  update.
        pPmicMgr->eDrvState = (uint8)BQ7973X_STATE_NORMAL;
        pPmicMgr->pPmicDiag = &zBq7973xDiagMgr[0u];
        pPmicMgr->pPmicDiag->uDiagInitState = PMI_DIAG_INIT_COMPLETED;
        uRet = (uint8)PMI_PMC_CFGINIT_SUCCESS;
    }
    while(0);

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_CFGDATA_INIT, uRet, pPmicMgr->uPmicIface, 0);
}

/**********************************************************************************************************************
 * Function name: Bq7973x_StartupInit
 *********************************************************************************************************************/
/** \brief This function is used to
 *
 * Extended function description
 *
 * \param[in]
 * \param[in]
 * \param[in]
 *
 * \pre Preconditions
 * \post Postconditions
 * \return Bq7973x_ManagerType
 * \retval NULL iferror
 *         Valid pointer on success
 *
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_StartupInit(Bq7973x_ManagerType *pPmicMgr)
{
    uint8 uRet = (uint8)PMI_INIT_PROGRESS;
    uint8 uCmd;

    do
    {
        if((uint8)BQ7973X_STATE_WAKEUP == pPmicMgr->eDrvState)
        {
            uRet = Comiface_WakePing(pPmicMgr->pComifCtx, BQ7973X_WAKEUP_PING_DELAY);
            if((uint8)COMIF_OK != uRet)
            {
                uRet = (uint8)PMI_PMC_WAKEUP_CMD_FAILED;
                break;
            }

            if((uint8)DIR_DIRECT == pPmicMgr->eCommDir)
            {
                uRet = Bq7973x_ConfigInit(pPmicMgr);
                if((uint8)COMIF_OK != uRet)
                {
                    uRet = (uint8)PMI_PMC_CONF_INIT_FAILED;
                    break;
                }
            }
            else
            {
                // TBD: Need to implement power state manager according to the wakeup
                uCmd = (pPmicMgr->eCommDir | BQ7973X_CONTROL1_SEND_WAKE_MSK);
                uRet = Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_CONTROL1_OFFSET,
                                       &uCmd, WRITE_1BYTE);
                if((uint8)COMIF_OK != uRet)
                {
                    uRet = (uint8)PMI_PMC_WAKEUP_STACK_FAILED;
                    break;
                }
                uRet = Comif_SetDelay(pPmicMgr->pComifCtx, BQ7973X_WAKEUP_TONE_DELAY);
            }

            uRet = (uint8)PMI_PMC_STARTUP_SUCCESS;
        }

        if((uint8)BQ7973X_STATE_NORMAL == pPmicMgr->eDrvState)
        {
            uRet = (uint8)PMI_PMC_NORMAL_STATE;
        }
    }
    while(0);

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_STARTUP_INIT, uRet, pPmicMgr->uPmicIface, pPmicMgr->eDrvState);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_GetCurrent(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function is used to get the current value from the 73x device
 *
 *  \param[in]      pPmicMgr: Manager
 *  \param[in]      pSvcCfg: SVC context for the reading the data
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_GetCurrent(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uIface = 0, eDrvState =0;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        eDrvState = pPmicMgr->eDrvState;
        uIface = pPmicMgr->uPmicIface;

        uRet = Comif_SingleRead(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_CURRENT1_HI_OFFSET,
                               pSvcCfg, (BQ7973X_CURRENT_REG_NUM * READ_3BYTE));
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_GETPCURRENT, uRet, uIface, eDrvState);
}


/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_GetCurrent(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function is used to get the read VF voltage from the 73x device
 *
 *  \param[in]      pPmicMgr: Manager
 *  \param[in]      pSvcCfg: SVC context for the reading the data
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_GetVf(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uIface = 0, eDrvState =0;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        eDrvState = pPmicMgr->eDrvState;
        uIface = pPmicMgr->uPmicIface;

        uRet = Comif_SingleRead(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_VF2_HI_OFFSET,
                               pSvcCfg, (BQ7973X_VF_REG_NUM * READ_2BYTE));
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_GETVF_VOLT, uRet, uIface, eDrvState);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_GetCpVolt(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function is used to get the read VF voltage from the 73x device
 *
 *  \param[in]      pPmicMgr: Manager
 *  \param[in]      pSvcCfg: SVC context for the reading the data
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_GetCpVolt(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uIface = 0, eDrvState =0;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        eDrvState = pPmicMgr->eDrvState;
        uIface = pPmicMgr->uPmicIface;

        uRet = Comif_SingleRead(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_CP_HI_OFFSET,
                               pSvcCfg, READ_2BYTE);
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_GETCP_VOLT, uRet, uIface, eDrvState);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_GetCcAccCnt(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function is used to get the read VF voltage from the 73x device
 *
 *  \param[in]      pPmicMgr: Manager
 *  \param[in]      pSvcCfg: SVC context for the reading the data
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_GetCcAccCnt(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uIface = 0, eDrvState =0;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        eDrvState = pPmicMgr->eDrvState;
        uIface = pPmicMgr->uPmicIface;

        uRet = Comif_SingleRead(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_CC_ACC_HI_OFFSET,
                               pSvcCfg, BQ7973X_CC_REG_NUM);
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_GETCC_ACC_CNT, uRet, uIface, eDrvState);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_CCClear(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function is used to get the read VF voltage from the 73x device
 *
 *  \param[in]      pPmicMgr: Manager
 *  \param[in]      pSvcCfg: SVC context for the reading the data
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_CCClear(Bq7973x_ManagerType *pPmicMgr)
{
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uIface = 0, eDrvState =0;
    uint8 uCmd = BQ7973X_CC_CTRL_CC_CLR_GO_MSK;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        eDrvState = pPmicMgr->eDrvState;
        uIface = pPmicMgr->uPmicIface;

        uRet = Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_CC_CTRL_OFFSET,
                               &uCmd, WRITE_1BYTE);
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_CC_CLEAR, uRet, uIface, eDrvState);
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7973x_CODE) Bq7973x_GetVGpio(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function is used to get the read VF voltage from the 73x device
 *
 *  \param[in]      pPmicMgr: Manager
 *  \param[in]      pSvcCfg: SVC context for the reading the data
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_GetVGpio(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uIface = 0, eDrvState =0;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        eDrvState = pPmicMgr->eDrvState;
        uIface = pPmicMgr->uPmicIface;

        uRet = Comif_SingleRead(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_GPIO1_HI_OFFSET,
                               pSvcCfg, (READ_2BYTE * BQ7973X_GPIO_REG_NUM));
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_GET_VGPIO, uRet, uIface, eDrvState);
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7973x_CODE) Bq7973x_GetDieTemp(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function is used to get the read Die Temparatur from the 73x device
 *
 *  \param[in]      pPmicMgr: Manager
 *  \param[in]      pSvcCfg: SVC context for the reading the data
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_GetDieTemp(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uIface = 0, eDrvState =0;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        eDrvState = pPmicMgr->eDrvState;
        uIface = pPmicMgr->uPmicIface;

        uRet = Comif_SingleRead(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_DIETEMP1_HI_OFFSET,
                               pSvcCfg, READ_4BYTE);
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_GET_DIETEMP, uRet, uIface, eDrvState);
}

/**********************************************************************************************************************
 *  External Functions Definition
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_Control2Init(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          This function is used to initialize CONTROL2 Register
 *
 *  \param[in]      pPmicMgr: Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7973x_CODE) Bq7973x_Control2Init(Bq7973x_ManagerType *pPmicMgr)
{
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uIface = 0, eDrvState =0;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        eDrvState = pPmicMgr->eDrvState;
        uIface = pPmicMgr->uPmicIface;

        /*initialize CONTROL2 Register*/
        uRet = Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_CONTROL2_OFFSET,
                               &pPmicMgr->pPmcConfig->p_rcBq7973xRegsCfg->zRegCtrl.uControl2, WRITE_1BYTE);
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_CTRL2_CFG_INIT, uRet, uIface, eDrvState);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_GpioConfInit(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          This function is used to initialize GPIO_CONFx
 *
 *  \param[in]      pPmicMgr: Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7973x_CODE) Bq7973x_GpioConfInit(Bq7973x_ManagerType *pPmicMgr, const uint8 *pCfgSet)
{
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uIface = 0, eDrvState =0;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        eDrvState = pPmicMgr->eDrvState;
        uIface = pPmicMgr->uPmicIface;
        if(NULL != pCfgSet)
        {
            /*initialize GPIO_CONFx*/
            uRet = Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_GPIO_CONF1_OFFSET,
                                     &pPmicMgr->pPmcConfig->p_rcBq7973xRegsCfg->zRegNVM.uGpio_Conf[0],
                                     BQ7973X_GPIO_CONF1_6_REG_NUM);

            if((uint8)COMIF_OK == uRet)
            {
                uRet = Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_GPIO_CONF7_OFFSET,
                      &pPmicMgr->pPmcConfig->p_rcBq7973xRegsCfg->zRegNVM.uGpio_Conf[BQ7973X_GPIO_CONF1_6_REG_NUM - 1u],
                                     BQ7973X_GPIO_CONF7_8_REG_NUM);
            }
        }
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_GPIO_CFG_INIT, uRet, uIface, eDrvState);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_SetOCConf(Bq7973x_ManagerType *pPmicMgr)
 *********************************************************************************************************************/
/*! \brief          Function Set the desired over current threshold in registers OC*_THR* and the deglitch filter time is set by [OC*_DEG].
 *
 *  \param[in]      pPmicMgr: Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7973x_CODE) Bq7973x_SetOCConf(Bq7973x_ManagerType *pPmicMgr,  const uint8 *pCfgSet)
{
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uIface = 0, eDrvState =0;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        eDrvState = pPmicMgr->eDrvState;
        uIface = pPmicMgr->uPmicIface;
        if(NULL != pCfgSet)
        {
            /*initialize GPIO_CONFx*/
            uRet = Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_OC_CONF1_OFFSET,
                                     pPmicMgr->pPmcConfig->p_rcBq7973xRegsCfg->zRegNVM.uOC_Conf,
                                     BQ7973X_GPIO_CONF1_6_REG_NUM);

            if((uint8)COMIF_OK == uRet)
            {
                uRet = Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_OC_CONF7_OFFSET,
                                         &pPmicMgr->pPmcConfig->p_rcBq7973xRegsCfg->zRegNVM.uOC_Conf[6u],
                                         BQ7973X_GPIO_CONF7_8_REG_NUM);
            }
        }
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_OC_CFG_INIT, uRet, uIface, eDrvState);
}


/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_AdcCtrlInit(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          This function is used to initialize ADC_CTRLx
 *
 *  \param[in]      pPmicMgr: Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7973x_CODE) Bq7973x_AdcCtrlInit(Bq7973x_ManagerType *pPmicMgr)
{
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uIface = 0, eDrvState =0;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        eDrvState = pPmicMgr->eDrvState;
        uIface = pPmicMgr->uPmicIface;

        /*initialize ADC_CTRLx */
        uRet = Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_ADC_CTRL1_OFFSET,
                                 pPmicMgr->pPmcConfig->p_rcBq7973xRegsCfg->zRegCtrl.uAdc_Ctrl,
                                 BQ7973X_ADC_CTRL_REG_NUM);
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_ADC_CFG_INIT, uRet, uIface, eDrvState);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_AdcCtrlInit(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function is used to initialize CC_CTRL
 *
 *  \param[in]      pPmicMgr: Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7973x_CODE) Bq7973x_CCCtrlInit(Bq7973x_ManagerType *pPmicMgr)
{
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uIface = 0, eDrvState =0;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        eDrvState = pPmicMgr->eDrvState;
        uIface = pPmicMgr->uPmicIface;

        /*initialize CC Control*/
        uRet = Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_CC_CTRL_OFFSET,
                                     &pPmicMgr->pPmcConfig->p_rcBq7973xRegsCfg->zRegCtrl.uCC_Ctrl,
                                     WRITE_1BYTE);
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_CC_CFG_INIT, uRet, uIface, eDrvState);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_SetSWInit(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function Init SW
 *
 *  \param[in]      pPmicMgr: Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7973x_CODE) Bq7973x_SetSWInit(Bq7973x_ManagerType *pPmicMgr, uint8 uSwConf)
{
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uIface = 0, eDrvState =0;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        eDrvState = pPmicMgr->eDrvState;
        uIface = pPmicMgr->uPmicIface;

        /*initialize CC Control*/
        uRet = Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_SW_CTRL_OFFSET,
                                     &uSwConf,
                                     BQ7973X_SW_REG_NUM);
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_CC_CFG_INIT, uRet, uIface, eDrvState);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_SetOCCtrl1(Bq7973x_ManagerType *pPmicMgr)
 *********************************************************************************************************************/
/*! \brief          Function Init Set (OC Control)  over current condition to set PWM pattern on OC pins.
 *
 *  \param[in]      pPmicMgr: Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7973x_CODE) Bq7973x_SetOCCtrl1(Bq7973x_ManagerType *pPmicMgr, uint8 uOcCtrl)
{
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uIface = 0, eDrvState =0;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        eDrvState = pPmicMgr->eDrvState;
        uIface = pPmicMgr->uPmicIface;

        uRet = Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_OC_CTRL1_OFFSET,
                                     &uOcCtrl, WRITE_1BYTE);
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_OC_CTRL1_INIT, uRet, uIface, eDrvState);
}
/*********************************************************************************************************************/
/*! \brief This function is used to perform soft reset of the stack device.
*
 *  \param[in]      pPmicMgr: Manager
*
*  \reentrant      TRUE
*  \synchronous    FALSE
*  \pre
*  \post
*  \return         uint8
*  \retval         BMI_OK
*                  BMI_NOT_OK
*  \trace
*********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_CODE) BQ7973X_DevPowerManager(Bq7973x_ManagerType *pPmicMgr, uint8 uResetTyp)
{
    uint8 uRet = (uint8)PMI_OK;
    uint8 uCmd;
	uint8 uDevIdx =0u;

    switch (uResetTyp)
    {

        case (uint8)PMI_SVCID_PMC_SET_SLP2WAKEUP:
        {
            uCmd = (pPmicMgr->eCommDir | BQ7973X_CONTROL1_SEND_SLPTOACT_MSK);
		    uRet |= Comif_SingleWrite(pPmicMgr->pComifCtx, 0, BQ7973X_CONTROL1_OFFSET, &uCmd, WRITE_1BYTE);

		    uRet |= Comif_SetDelay(pPmicMgr->pComifCtx, BQ7973X_SLP2ACT_TONE_DELAY);

            break;
        }

        case (uint8)PMI_SVCID_PMC_SET_SOFT_RESET:
        {
		    uCmd = (pPmicMgr->eCommDir | BQ7973X_CONTROL1_SOFT_RESET_MSK);
		    uRet |= Comif_StackWrite(pPmicMgr->pComifCtx, FALSE, COMIF_PRIO_EXCL_CELL(BQ7973X_CONTROL1_OFFSET, COMIF_LOCK),
		    							&uCmd, WRITE_1BYTE);
		    uRet |= Comif_SingleWrite(pPmicMgr->pComifCtx, 0, COMIF_PRIO_EXCL_CELL(BQ7973X_CONTROL1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
		    uRet |= Comif_SetDelay(pPmicMgr->pComifCtx, BQ7973X_SOFT_RESET_DELAY);
            break;
        }
        case (uint8)PMI_SVCID_PMC_SET_HW_RESET:
        {
		    /* perform hard reset of the stack device. */
		    uCmd = (pPmicMgr->eCommDir  | BQ7973X_CONTROL1_SEND_SD_HW_RST_MSK);
		    for (uDevIdx = 0u; uDevIdx < pPmicMgr->uNAFEs; uDevIdx ++)
		    {
		    	uRet |= Comif_SingleWrite(pPmicMgr->pComifCtx, (uDevIdx+1), COMIF_PRIO_EXCL_CELL(BQ7973X_CONTROL1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
		    	uRet |= Comif_SetDelay(pPmicMgr->pComifCtx, BQ7973X_SOFT_RESET_DELAY);
		    }
		    uRet |= Comif_SingleWrite(pPmicMgr->pComifCtx, 0, COMIF_PRIO_EXCL_CELL(BQ7973X_CONTROL1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
		    uRet |= Comif_SetDelay(pPmicMgr->pComifCtx, BQ7973X_SOFT_RESET_DELAY);
		    /* perform hard reset of the bridge device. */
		    uCmd = (pPmicMgr->eCommDir  | BQ7971X_CONTROL1_GOTO_SHUTDOWN_MSK);
		    uRet |= Comif_SingleWrite(pPmicMgr->pComifCtx, 0, COMIF_PRIO_EXCL_CELL(BQ7973X_CONTROL1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
		    uRet |= Comif_SetDelay(pPmicMgr->pComifCtx, BQ7973X_HW_RESET_DELAY);
            break;
        }
        default:
        {
            uRet = (uint8)PMI_NOT_OK;
            break;
        }
    }
    return Pmi_SetErrorDetails(uResetTyp, uRet, pPmicMgr->uPmicIface, pPmicMgr->eDrvState);
}
/**********************************************************************************************************************
 * Function name: Bq7973x_ControlProc
 *********************************************************************************************************************/
/** \brief This function is used to
 *
 * Extended function description
 *
 * \param[in]
 * \param[in]
 * \param[in]
 *
 * \pre Preconditions
 * \post Postconditions
 * \return Bq7973x_ManagerType
 * \retval NULL iferror
 *         Valid pointer on success
 *
 *********************************************************************************************************************/

FUNC(uint8, bq79600_CODE) Bq7973x_ControlProc(void *pPmcCtx, uint8 uCmd, void *pData)
{
    Bq7973x_ManagerType *pPmicMgr = (Bq7973x_ManagerType *) pPmcCtx;
    uint8 uRet = (uint8)PMI_NOT_OK;
    VersionInfoType *pPmcVer;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        switch(uCmd)
        {
            case (uint8)PMI_GET_DRIVER_STATE:
            {
                uRet = (uint8)PMI_INIT_PROGRESS;
                if((uint8)BQ7973X_STATE_NORMAL == pPmicMgr->eDrvState)
                {
                    uRet = (uint8)PMI_PMC_NORMAL_STATE;
                }
                break;
            }
            case (uint8)PMI_GET_DIAG_STATE:
            {
                break;
            }
            case (uint8)PMI_GET_VERSION:
            {
                pPmcVer = (VersionInfoType *) pData;
                if(NULL != pPmcVer)
                {
                    pPmcVer->zPmcVersion.vendorID = BQ7973X_VENDOR_ID;
                    pPmcVer->zPmcVersion.moduleID = BQ7973X_MODULE_ID;
                    pPmcVer->zPmcVersion.sw_major_version = BQ7973X_SW_MAJOR_VERSION;
                    pPmcVer->zPmcVersion.sw_minor_version = BQ7973X_SW_MINOR_VERSION;
                    pPmcVer->zPmcVersion.sw_patch_version = BQ7973X_SW_PATCH_VERSION;

                    uRet = Comif_GetVersion(pPmicMgr->pComifCtx, pPmcVer);
                }
                break;
            }
            default:
            {
                uRet = (uint8)PMI_PMC_INVALID_CTRL_CMD;
                break;
            }
        }
    }

    return uRet;
}


/**********************************************************************************************************************
 * Function name: Bq7973x_TxHandlerProc
 *********************************************************************************************************************/
/** \brief This function is used to
 *
 * Extended function description
 *
 * \param[in]
 * \param[in]
 * \param[in]
 *
 * \pre Preconditions
 * \post Postconditions
 * \return Bq7973x_ManagerType
 * \retval NULL iferror
 *         Valid pointer on success
 *
 *********************************************************************************************************************/

FUNC(uint8, bq7973x_CODE) Bq7973x_TxHandlerProc(void *pPmcCtx)
{
    Bq7973x_ManagerType *pPmicMgr = (Bq7973x_ManagerType *)  pPmcCtx;
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uPmicIface = TIBMS_INVALID_IFACE;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        uPmicIface = pPmicMgr->uPmicIface;
        uRet = Comif_HandleTransfer(pPmicMgr->pComifCtx);
    }

    return Pmi_SetErrorDetails(PMI_SVCID_HANDLE_TX, uRet, uPmicIface, 0u);
}

/**********************************************************************************************************************
 * Function name: Bq7973x_RxHandlerProc
 *********************************************************************************************************************/
/** \brief This function is used to
 *
 * Extended function description
 *
 * \param[in]
 *
 * \pre Preconditions
 * \post Postconditions
 * \return uint8
 * \retval
 *
 *
 *********************************************************************************************************************/

FUNC(uint8, bq7973x_CODE) Bq7973x_RxHandlerProc(void *pPmcCtx)
{
    Bq7973x_ManagerType *pPmicMgr = (Bq7973x_ManagerType *)  pPmcCtx;
    uint8 uRet =(uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uPmicIface = TIBMS_INVALID_IFACE;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        uPmicIface = pPmicMgr->uPmicIface;
        uRet = Comif_HandleRecieve(pPmicMgr->pComifCtx);
    }

    return Pmi_SetErrorDetails(PMI_SVCID_HANDLE_RX, uRet, uPmicIface, 0);
}

/**********************************************************************************************************************
 * Function name: Bq7973x_NotifyProc
 *********************************************************************************************************************/
/** \brief This function is used to
 *
 * Extended function description
 *
 * \param[in]
 * \param[in]
 *
 * \pre Preconditions
 * \post Postconditions
 * \return uint8
 * \retval
 *
 *
 *********************************************************************************************************************/

FUNC(uint8, bq7973x_CODE) Bq7973x_NotifyProc(void *pPmcCtx, uint8 uType)
{
    Bq7973x_ManagerType *pPmicMgr = (Bq7973x_ManagerType *)  pPmcCtx;
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uPmicIface = TIBMS_INVALID_IFACE;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        uPmicIface = pPmicMgr->uPmicIface;
        uRet = Comif_HandleNotify(pPmicMgr->pComifCtx, uType);
    }

    return Pmi_SetErrorDetails(PMI_SVCID_NOTIFICATION, uRet, uPmicIface, uType);
}

/**********************************************************************************************************************
 * uint8 Bq7971x_DecodeVcell(Bq7971x_ManagerType *pBmcCtx, const ServiceCfgType *pSvcCfg, uint16 *pOutVcellVolt)
 *********************************************************************************************************************/
/** \brief This function is used to Decode the Cell Voltage Buffer
 *
 *  \param[in]      pBmcCtx: Manager
 *  \param[in]      uType: Type of the Notify
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *
 *  \return         uint8
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7971x_CODE) Bq7973x_DecodePackCurrent(const Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg,
                                                    float *pOutPackCurrent)
{
    uint8 *pData;
    uint32 qPackCurrent;
    uint8 uRet = (uint8)PMI_NOT_OK;

    if(((uint8)RESP_VALID == pSvcCfg->pSvcStat->uRespStatus) && (NULL != pOutPackCurrent))
    {
        pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
        pData = (uint8 *) &pSvcCfg->pRawData[pSvcCfg->uDataOffset];
        if(NULL != pData)
        {
            qPackCurrent = (pData[0] << 16u | pData[1u] << 8u | pData[2]);
            if(qPackCurrent < BQ7973X_INVALDVOLCSVALUE)
            {
                pOutPackCurrent[0] = (qPackCurrent * BQ7973X_LSB_CS);
            }
            else if(qPackCurrent > BQ7973X_INVALDVOLCSVALUE)
            {
                pOutPackCurrent[0] = ((-((sint32)(((~qPackCurrent + 1U) & BQ7973X_CLIPVOLCSVALUE)))) * BQ7973X_LSB_CS);
            }
            else
            {
                pOutPackCurrent[0] = BQ7973X_INVALDVOLCSVALUE;
            }

            qPackCurrent = (pData[3] << 16u | pData[4u] << 8u | pData[5]);
            if(qPackCurrent <  BQ7973X_INVALDVOLCSVALUE)
            {
                pOutPackCurrent[1] = (qPackCurrent * BQ7973X_LSB_CS);
            }
            else if(qPackCurrent > BQ7973X_INVALDVOLCSVALUE)
            {
                pOutPackCurrent[1] = ((-((sint32)(((~qPackCurrent + 1) & BQ7973X_CLIPVOLCSVALUE)))) * BQ7973X_LSB_CS);
            }
            else
            {
                pOutPackCurrent[1] = BQ7973X_INVALDVOLCSVALUE;
            }
        }
        pSvcCfg->pSvcStat->uInfo = (uint8)RESP_DECODED;
        uRet = (uint8)PMI_OK;
        pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);
    }

    return uRet;
}

/**********************************************************************************************************************
 * uint8 Bq7971x_DecodeVcell(Bq7971x_ManagerType *pBmcCtx, const ServiceCfgType *pSvcCfg, uint16 *pOutVcellVolt)
 *********************************************************************************************************************/
/** \brief This function is used to Decode the Cell Voltage Buffer
 *
 *  \param[in]      pBmcCtx: Manager
 *  \param[in]      uType: Type of the Notify
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *
 *  \return         uint8
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7971x_CODE) Bq7973x_DecodeVfVolt(const Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg,
                                               uint16 *pOutVfVolt)
{
    uint8 *pData;
    uint8 uRet = (uint8)PMI_NOT_OK;

    if(((uint8)RESP_VALID == pSvcCfg->pSvcStat->uRespStatus) && (NULL != pOutVfVolt))
    {
        pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
        pData = (uint8 *) &pSvcCfg->pRawData[pSvcCfg->uDataOffset];
        if(NULL != pData)
        {
            pOutVfVolt[0u] = ((pData[0] << 8u) | (pData[1u]));
            pOutVfVolt[1u] = ((pData[2] << 8u) | (pData[3u]));
        }
        pSvcCfg->pSvcStat->uInfo = (uint8)RESP_DECODED;
        uRet = (uint8)PMI_OK;
        pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);
    }

    return uRet;
}

/**********************************************************************************************************************
 * uint8 Bq7971x_DecodeVcell(Bq7971x_ManagerType *pBmcCtx, const ServiceCfgType *pSvcCfg, uint16 *pOutVcellVolt)
 *********************************************************************************************************************/
/** \brief This function is used to Decode the Cell Voltage Buffer
 *
 *  \param[in]      pBmcCtx: Manager
 *  \param[in]      uType: Type of the Notify
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *
 *  \return         uint8
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7971x_CODE) Bq7973x_DecodeCfVolt(const Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg,
                                               uint16 *pOutCfVolt)
{
    uint8 *pData;
    uint8 uRet = (uint8)PMI_NOT_OK;

    if(((uint8)RESP_VALID == pSvcCfg->pSvcStat->uRespStatus) && (NULL != pOutCfVolt))
    {
        pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
        pData = (uint8 *) &pSvcCfg->pRawData[pSvcCfg->uDataOffset];
        if(NULL != pData)
        {
            pOutCfVolt[0u] = ((pData[0] << 8u) | (pData[1u]));
        }
        pSvcCfg->pSvcStat->uInfo = (uint8)RESP_DECODED;
        uRet = (uint8)PMI_OK;
        pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);
    }

    return uRet;
}

/**********************************************************************************************************************
 * uint8 Bq7971x_DecodeVcell(Bq7971x_ManagerType *pBmcCtx, const ServiceCfgType *pSvcCfg, uint16 *pOutVcellVolt)
 *********************************************************************************************************************/
/** \brief This function is used to Decode the Cell Voltage Buffer
 *
 *  \param[in]      pBmcCtx: Manager
 *  \param[in]      uType: Type of the Notify
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *
 *  \return         uint8
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7971x_CODE) Bq7973x_DecodeCcAccCnt(const Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg,
                                                 uint16 *pOutCcAccCnt)
{
    uint8 *pData;
    uint8 uRet = (uint8)PMI_NOT_OK;

    if(((uint8)RESP_VALID == pSvcCfg->pSvcStat->uRespStatus) && (NULL != pOutCcAccCnt))
    {
        pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
        pData = (uint8 *) &pSvcCfg->pRawData[pSvcCfg->uDataOffset];
        if(NULL != pData)
        {
            pOutCcAccCnt[0u] = (uint8) pData[0u];
            pOutCcAccCnt[1u] = (uint8) pData[1u];
            pOutCcAccCnt[2u] = (uint8) pData[2u];
            pOutCcAccCnt[3u] = (uint8) pData[3u];
            pOutCcAccCnt[4u] = (uint8) pData[4u];
        }
        pSvcCfg->pSvcStat->uInfo = (uint8)RESP_DECODED;
        uRet = (uint8)PMI_OK;
        pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);
    }

    return uRet;
}

/**********************************************************************************************************************
 * Function name: Bq7973x_ReceiveProc
 *********************************************************************************************************************/
/** \brief This function is used to
 *
 * Extended function description
 *
 * \param[in]
 * \param[in]
 *
 * \pre Preconditions
 * \post Postconditions
 * \return uint8
 * \retval
 *
 *
 *********************************************************************************************************************/

FUNC(uint8, bq7973x_CODE) Bq7973x_ReceiveProc(void *pPmicCtx, const ServiceCfgType *pSvcCfg, const uint8 *pRxData,
                                              uint8 uStatus)
{
    Bq7973x_ManagerType *pPmicMgr = (Bq7973x_ManagerType *)  pPmicCtx;
    uint32 qRespTime;
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uServiceId = 0;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        uRet = (uint8)PMI_PMC_INVALID_RX_PROC;
        if(NULL != pSvcCfg)
        {
            uServiceId = pSvcCfg->uServiceId;
            pSvcCfg->pSvcStat->uRespStatus = uStatus;
            switch(pSvcCfg->uServiceId)
            {
                case (uint8)PMI_SERVICE_PACK_CURRENT:
                {
                    uRet = Bq7973x_DecodePackCurrent(pPmicMgr, pSvcCfg, (float *) pSvcCfg->pDecodeBuf0);
                    break;
                }
                case (uint8)PMI_SERVICE_VF_VOLT:
                {
                    uRet = Bq7973x_DecodeVfVolt(pPmicMgr, pSvcCfg, (uint16 *) pSvcCfg->pDecodeBuf0);
                    break;
                }
                case (uint8)PMI_SERVICE_CP_VOLT:
                {
                    uRet = Bq7973x_DecodeCfVolt(pPmicMgr, pSvcCfg, (uint16 *) pSvcCfg->pDecodeBuf0);
                    break;
                }
                case (uint8)PMI_SERVICE_CC_ACC_CNT:
                {
                    uRet = Bq7973x_DecodeCcAccCnt(pPmicMgr, pSvcCfg, (uint16 *) pSvcCfg->pDecodeBuf0);
                    break;
                }
                case (uint8)PMI_SERVICE_DIAGNOSTICS:
                {
                    uRet = Bq7973x_DiagDataProc(pPmicMgr->pPmicDiag, pSvcCfg);
                    break;
                }
                case (uint8)PMI_SERVICE_CONFIG:
                {
                    uRet =  (uint8)PMI_OK;
                    break;
                }
                default:
                {
                    uRet = (uint8)PMI_PMC_INVALID_SVC_CMD;
                    break;
                }
            }

            pSvcCfg->pSvcStat->nRespMsgs++;
            pPmicMgr->pBswCfg->timestamp_req(&pSvcCfg->pSvcStat->cReqTimeMs, (uint32 *)&qRespTime);
            qRespTime = (qRespTime - pSvcCfg->pSvcStat->cReqTimeMs);
            if(pSvcCfg->pSvcStat->cRespMaxTimeMs < qRespTime)
            {
                pSvcCfg->pSvcStat->cRespMaxTimeMs = qRespTime;
            }

            if((uint8)RESP_INVALID == uStatus)
            {
                pSvcCfg->pSvcStat->uFailure++;
            }
        }
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_RX_DATA_PROC, uRet, uServiceId, 0);
}

/**********************************************************************************************************************
 * Function name: Bq7973x_DataDecode
 *********************************************************************************************************************/
/** \brief This function is used to
 *
 * Extended function description
 *
 * \param[in]
 * \param[in]
 *
 * \pre Preconditions
 * \post Postconditions
 * \return uint8
 * \retval
 *
 *
 *********************************************************************************************************************/

FUNC(uint8, bq7973x_CODE) Bq7973x_DataDecode(void *pPmcCtx, uint8 uService, void *pBuffer, uint16 wLength)
{
    Bq7973x_ManagerType *pPmicMgr = (Bq7973x_ManagerType *)  pPmcCtx;
    const ServiceCfgType *pFuncSvcCfg;
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        uRet = (uint8)PMI_PMC_DECODE_RX_BUSY;
        switch(uService)
        {
            case (uint8)PMI_SERVICE_DIETEMP:
            {
                if(NULL != pBuffer)
                {
                    uRet = (uint8)PMI_PMC_DECODE_DIETEMP_PREV_DATA;
                    pFuncSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_DIETEMP];
                    if((NULL != pFuncSvcCfg) && ((uint8)RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        pFuncSvcCfg->pSvcStat->uInfo = (uint8)RESP_USED;
                        if((wLength > 0U) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            (void)memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, wLength);
                            pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);
                            uRet = (uint8)PMI_OK;
                        }
                    }
                }
                break;
            }


            case (uint8)PMI_SERVICE_VGPIO:
            {
                if(NULL != pBuffer)
                {
                    uRet = (uint8)PMI_PMC_DECODE_GPIO_PREV_DATA;
                    pFuncSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_VGPIO];
                    if((NULL != pFuncSvcCfg) && ((uint8)RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        pFuncSvcCfg->pSvcStat->uInfo = (uint8)RESP_USED;
                        if((wLength > 0U) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            (void)memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, wLength);
                            pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);
                            uRet = (uint8)PMI_OK;
                        }
                    }
                }
                break;
            }

            case (uint8)PMI_SERVICE_CC_ACC_CNT:
            {
                if(NULL != pBuffer)
                {
                    uRet = (uint8)PMI_PMC_DECODE_CCACCCNT_PREV_DATA;
                    pFuncSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_CC_ACC_CNT];
                    if((NULL != pFuncSvcCfg) && ((uint8)RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        pFuncSvcCfg->pSvcStat->uInfo = (uint8)RESP_USED;
                        if((wLength > 0U) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            (void)memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, wLength);
                            pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);
                            uRet = (uint8)PMI_OK;
                        }
                    }
                }
                break;
            }


            case (uint8)PMI_SERVICE_CP_VOLT:
            {
                if(NULL != pBuffer)
                {
                    uRet = (uint8)PMI_PMC_DECODE_CP_PREV_DATA;
                    pFuncSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_CP_VOLT];
                    if((NULL != pFuncSvcCfg) && ((uint8)RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        pFuncSvcCfg->pSvcStat->uInfo = (uint8)RESP_USED;
                        if((wLength > 0U) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            (void)memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, wLength);
                            pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);
                            uRet = (uint8)PMI_OK;
                        }
                    }
                }
                break;
            }

            case (uint8)PMI_SERVICE_VF_VOLT:
            {
                if(NULL != pBuffer)
                {
                    uRet = (uint8)PMI_PMC_DECODE_VF_PREV_DATA;
                    pFuncSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_VF_VOLT];
                    if((NULL != pFuncSvcCfg) && ((uint8)RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        pFuncSvcCfg->pSvcStat->uInfo = (uint8)RESP_USED;
                        if((wLength > 0U) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            (void)memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, wLength);
                            pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);
                            uRet = (uint8)PMI_OK;
                        }
                    }
                }
                break;
            }

           case (uint8)PMI_SERVICE_PACK_CURRENT:
            {
                if(NULL != pBuffer)
                {
                    uRet = (uint8)PMI_PMC_DECODE_PC_PREV_DATA;
                    pFuncSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_PACK_CURRENT];
                    if((NULL != pFuncSvcCfg) && ((uint8)RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        pFuncSvcCfg->pSvcStat->uInfo = (uint8)RESP_USED;
                        if((wLength > 0U) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            (void)memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, wLength);
                            pPmicMgr->pBswCfg->resource_req(pPmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);
                            uRet = (uint8)PMI_OK;
                        }
                    }
                }
                break;
            }

            default:
            {
                break;
            }
        }
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_RX_DECODE, uRet, uService, 0);
}

/**********************************************************************************************************************
 *  FUNC(void*, bq7973x_CODE) Bq7973x_Init(const ChainCfgType *pIfaceCfg, Comif_ManagerType *pComifCtx, uint8 uWupReq)
 *********************************************************************************************************************/
/*! \brief          This function is used to init the Bq7973x core for corresponding chains in the config set
 *
 *  \param[in]      pIfaceCfg - Configurations for the chain
 *  \param[in]      pComifCtx - Communication Interface context
 *  \param[in]      uWupReq - Wakeup Request
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         void *
 *  \retval         Pointer to Manager
 *  \trace
 *********************************************************************************************************************/

FUNC(void*, bq7973x_CODE) Bq7973x_Init(const InterfaceCfgType *pIfaceCfg, Comif_ManagerType *pComifCtx, uint8 uWupReq)
{
    Bq7973x_ManagerType *pPmicMgr;
    Bq7973x_ManagerType *pBq7973xCtxRet = NULL;
    const Pmi_ConfigType *pPmiCfg;
    uint8 uRet;

    do
    {
        if(NULL == pIfaceCfg)
        {
            uRet = (uint8)PMI_PMC_ILLEGALCTX;
            break;
        }

        pPmiCfg = pIfaceCfg->pPmiCfg;
        if((NULL == pPmiCfg) ||
           (NULL == pPmiCfg->pPmcOpsCfg) ||
           (NULL == pPmiCfg->pFuncCfg))
        {
            uRet = (uint8)PMI_PMC_INVALID_CFG;
            break;
        }

        if(NULL == pComifCtx)
        {
            uRet = (uint8)PMI_PMC_INVALID_COMIFMGR;
            break;
        }

        if(pPmiCfg->uPmiIfaceCfg >= BQ7973X_IFACES)
        {
            uRet = (uint8)PMI_PMC_INVALID_IFACECFG;
            break;
        }


        pPmicMgr = &zBq7973xMgr[pPmiCfg->uPmiIfaceCfg];
        if(NULL == pPmicMgr)
        {
            uRet = (uint8)PMI_PMC_INVALID_MGR;
            break;
        }

        if(BQ7973X_INIT == pPmicMgr->uInit)
        {
            uRet = (uint8)PMI_PMC_ALREADY_INUSE;
            break;
        }

        (void)memset(pPmicMgr, 0, sizeof(Bq7973x_ManagerType));

        pPmicMgr->pPmcConfig = (Bq7973x_ConfigType *) pPmiCfg->pPmicCfg;
        pPmicMgr->pFuncCfg = pIfaceCfg->pPmiCfg->pFuncCfg;

        pPmicMgr->pComifCtx = pComifCtx;
        pPmicMgr->uWupReq = uWupReq;

        pPmicMgr->pBswCfg = pPmiCfg->pBswCfg;
        if(NULL == pPmicMgr->pBswCfg)
        {
            uRet = (uint8)PMI_PMC_INVALID_IFACECFG;
            break;
        }

        pPmicMgr->qApplRes = pPmicMgr->pBswCfg->qApplRes;
        pPmicMgr->eCommDir = pIfaceCfg->eReqComDirCfg;
        pPmicMgr->eDrvState = (uint8)BQ7973X_STATE_WAKEUP;
        pPmicMgr->uPmicIface = pPmiCfg->uPmiIfaceCfg;
        pPmicMgr->uIsPmicInIface = pIfaceCfg->uIsPmicInIfCfg;
        pPmicMgr->uPmicDevId = pIfaceCfg->uPmicDevIdCfg;
        /* Diag_Update_Start */
        uint8 uNfaultCfg = 0 ; // passed as argument in BMI init but not PMI
        pPmicMgr->uNAFEs = (pIfaceCfg->uNPmicsCfg );                                        /** Total Number of AFE's in the system configuration **/
        pPmicMgr->uNPmics = pIfaceCfg->uNPmicsCfg;                                          /** Total Number of PMIC AFE's in the system configuration **/
        pPmicMgr->uSDevId = 0;                                                                 /** DevID zero is for the  PMIchip **/
#if 1
        pPmicMgr->pPmicDiag = BQ7973x_DiagInit(pPmicMgr, pPmiCfg, pComifCtx, pPmicMgr->uPmicIface,
                                             pPmicMgr->uNAFEs, uNfaultCfg, pPmicMgr->eComType,
                                             pPmicMgr->uNPmics, pPmicMgr->uSDevId);
        if(NULL == pPmicMgr->pPmicDiag)
        {
            uRet = PMI_PMC_DIAG_INIT_FAILED;
            break;
        }
#endif
        /* Diag_Update_END */
        pPmicMgr->qGuardStart = BQ7973X_GUARD_START;
        pPmicMgr->qGuardEnd = BQ7973X_GUARD_END;

        pPmicMgr->uInit = BQ7973X_INIT;

        pBq7973xCtxRet = pPmicMgr;
        uRet = (uint8)PMI_OK;
    }
    while(0);

    (void)Pmi_SetErrorDetails(PMI_SVCID_PMC_INIT, uRet, uWupReq, 0u);

    return pBq7973xCtxRet;
}

/**********************************************************************************************************************
 * Function name: Bq7973x_DeInit
 *********************************************************************************************************************/
/** \brief This function is used to
 *
 * Extended function description
 *
 * \param[in]
 *
 * \pre Preconditions
 * \post Postconditions
 * \return Bq7973x_ManagerType
 * \retval NULL iferror
 *         Valid pointer on success
 *
 *********************************************************************************************************************/

FUNC(uint8, bq7973x_CODE) Bq7973x_DeInit(void *pPmcCtx)
{
    Bq7973x_ManagerType *pPmicMgr = (Bq7973x_ManagerType *)  pPmcCtx;
    uint8 uRet = (uint8)PMI_PMC_INTEGRITY_FAILED;
    uint8 uPmicIface = TIBMS_INVALID_IFACE;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        uPmicIface = pPmicMgr->uPmicIface;
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_DEINIT, uRet, uPmicIface, 0);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) BQ7973X_GetVRrfCap(Bq7973x_ManagerType *pPmicMgr,const ServiceCfgType *pSvcCfg uint8 uDevId)
 *********************************************************************************************************************/
/*! \brief          Function is used to get the read VF voltage from the 73x device
 *
 *  \param[in]      pPmicMgr: Manager
 *  \param[in]      pSvcCfg
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \uRet           PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7973x_CODE) BQ7973X_GetVRrfCap(Bq7973x_ManagerType *pPmicMgr,const ServiceCfgType *pSvcCfg )
{
    uint8 uRet = (uint8)PMI_OK;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {

        uRet |= Comif_SingleRead(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_REF_CAP_HI_OFFSET,
                                 pSvcCfg, READ_2BYTE );
    }
    return uRet;
}


/**********************************************************************************************************************
 *  FUNC(uint8, bq7971x_CODE) BQ7973X_OCCtrl1Init(Bq7973x_ManagerType *pPmicMgr ,const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function is used to get the read VF voltage from the 73x device
 *
 *  \param[in]      pPmicMgr: Manager
 *  \param[in]      pSvcCfg
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Pmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \uRet           PMI_OK: Successful completion
 *                  PMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

 FUNC(uint8, bq7973x_CODE) BQ7973X_OCCtrl1Init( Bq7973x_DiagType * pPmicMgr,const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = PMI_OK;
    uint8 uDevIdx=0;

    for(uDevIdx =pPmicMgr->sDevId; uDevIdx < pPmicMgr->uNAFEs; uDevIdx++)
    {
        uRet |= Comif_SingleWrite(pPmicMgr->pComifCtx,uDevIdx, BQ7973X_OC_CTRL1_OFFSET, pPmicMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uOC_Ctrl , WRITE_1BYTE); 
    }
    uRet |= Comif_StackRead(pPmicMgr->pComifCtx, pPmicMgr->uNAFEs, BQ7973X_OC_CTRL1_OFFSET,pSvcCfg, (READ_1BYTE));
    return uRet;
}


/**********************************************************************************************************************/
// TBC the below functions shall be called somewhere !
/**********************************************************************************************************************/


/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_SetOCOutput( Bq7973x_ManagerType *pPmicMgr,  const uint8 *pCfgSet, uint8 uOCCtrl2 )
 *********************************************************************************************************************/
/** \brief This function is used to directly control OC Pin.
 *
 * Extended function description
 *
 * \param[in] uActDevID: device ID active in the current communication direction
 * \param[in] uOCCtrl2: OC_CTRL2 register value (select which OC pin to output)
 * \param[out] TD_ConfigPtr: Pointer to a transaction descriptor instance
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
Std_ReturnType BQ7973X_SetOCOutput( Bq7973x_ManagerType *pPmicMgr, uint8 uOCCtrl2 )
{
    uint8 uRet = PMI_PMC_INTEGRITY_FAILED;
    uint8 uCmd;

    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {

        uCmd = BQ7973X_OC_UNLOCKCODE1;
        uRet = Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_OC_UNLOCK1_OFFSET,
                                 &uCmd , WRITE_1BYTE);
        uCmd = BQ7973X_OC_UNLOCKCODE2;
        uRet |= Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_OC_UNLOCK2_OFFSET,
                                     &uCmd, WRITE_1BYTE);
        uCmd = uOCCtrl2 & (BQ7973X_OC_CTRL2_OC1_FLT_GO_MSK | BQ7973X_OC_CTRL2_OC2_FLT_GO_MSK);       
        uRet |= Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_OC_CTRL2_OFFSET,
                                         &uCmd, WRITE_1BYTE);                

    }
    return Pmi_SetErrorDetails(PMI_SVCID_PMC_SET_OC_OUT, uRet, pPmicMgr->uPmicIface, pPmicMgr->eDrvState);
}

/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_EnterCommDebug(Bq7973x_ManagerType *pPmicMgr)
 *********************************************************************************************************************/
/** \brief This function is used to enter communication debug mode.
 *
 * Extended function description
 *
 * \param[in] uActDevID: device ID active in the current communication direction
 * \param[out] TD_ConfigPtr: Pointer to a transaction descriptor instance
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
Std_ReturnType BQ7973X_EnterCommDebug(Bq7973x_ManagerType *pPmicMgr)
{
    Std_ReturnType uRet;
    uint8 uCmd;
    uCmd = BQ7973X_DEBUG_CTRL_UNLOCKCODE;
    uRet = Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_DEBUG_CTRL_UNLOCK_OFFSET,
                                &uCmd , WRITE_1BYTE);
    return Pmi_SetErrorDetails(PMI_SVCID_PMC_ENTR_COMM_DEBUG, uRet, pPmicMgr->uPmicIface, pPmicMgr->eDrvState);
}         

/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_EnADCFreeze(Bq7973x_ManagerType *pPmicMgr ,const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/** \brief API to perform enable all ADC output results freeze mode.
 *
 * Extended function description
 *
 * \param[in] uActDevID: device ID active in the current communication direction
 * \param[out] TD_ConfigPtr: Pointer to a transaction descriptor instance
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnTypef
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
Std_ReturnType BQ7973X_EnADCFreeze(Bq7973x_ManagerType *pPmicMgr)
{
    Std_ReturnType RetVal = E_OK;
    uint8 uCmd;

    uCmd = ((pPmicMgr->pPmcConfig->p_rcBq7973xRegsCfg->zRegCtrl.uAdc_Ctrl[1u] | BQ7973X_ADC_CTRL2_FREEZE_EN_MSK) & BQ7973X_ADC_FREEZE_MSK);
    RetVal |= Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId,BQ7973X_ADC_CTRL2_OFFSET,&uCmd,WRITE_1BYTE);

    //BQ7973X_uAdcCtrlRegData[1u] = uCmd;
    return RetVal;
}

/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_DisADCFreeze(Bq7973x_ManagerType *pPmicMgr ,const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/** \brief API to perform disable all ADC output results freeze mode.
 *
 * Extended function description
 *
 * \param[in] uActDevID: device ID active in the current communication direction
 * \param[out] TD_ConfigPtr: Pointer to a transaction descriptor instance
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
Std_ReturnType BQ7973X_DisADCFreeze(Bq7973x_ManagerType *pPmicMgr )
{
    Std_ReturnType RetVal = E_OK;
    uint8 uCmd;

    uCmd = pPmicMgr->pPmcConfig->p_rcBq7973xRegsCfg->zRegCtrl.uAdc_Ctrl[1u] & BQ7973X_ADC_UNFREEZE_MSK;
    RetVal |= Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId,BQ7973X_ADC_CTRL2_OFFSET,&uCmd,WRITE_1BYTE);

    //BQ7973X_uAdcCtrlRegData[1u] = uCmd;
    return RetVal;
}

/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_SetADCDelay(uint8 uActDevID, uint16 xTimeUs, Com_TD_Config_Struct_Type *TD_ConfigPtr)
 *********************************************************************************************************************/
/** \brief API to delay for this setting time before being enabled to start the conversion.
 *
 * Extended function description
 *
 * \param[in] uActDevID: device ID active in the current communication direction
 * \param[in] xTimeUs: Physical time(unit / us)
 * \param[out] TD_ConfigPtr: Pointer to a transaction descriptor instance
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
Std_ReturnType BQ7973X_SetADCDelay(Bq7973x_ManagerType *pPmicMgr,uint16 *pTimeUs)
{
    Std_ReturnType RetVal= E_OK;
    uint8 uCmd;
    uint16 xTimeUs =(uint16) *pTimeUs ; 
    if(xTimeUs <= BQ7973X_MAX_ADC_DLY)
    {
        uCmd = (uint8)(xTimeUs / BQ7973X_UNIT_ADC_DLY);
    }
    else
    {
        uCmd = BQ7973X_MAX_ADC_DLY / BQ7973X_UNIT_ADC_DLY;
    }
    RetVal |= Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId,BQ7973X_ADC_CONF_OFFSET,&uCmd,WRITE_1BYTE);
    return RetVal;
}



/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_SetEnableSPI(Bq7973x_ManagerType *pPmicMgr ,const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/** \brief This function is used to enable MSPI.
 *
 * Extended function description
 *
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_CODE)BQ7973X_MSPIConf(Bq7973x_ManagerType *pPmicMgr ,const ServiceCfgType *pSvcCfg)
{
    Std_ReturnType RetVal = E_OK;
    uint8 uCmd;
    uCmd = pPmicMgr->pPmcConfig->p_rcBq7973xRegsCfg->zRegNVM.uMspi_Conf;

    RetVal |= Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_MPSI_CONF_OFFSET,&uCmd , WRITE_1BYTE);

    //BQ7973X_uMspiConfRegData[0u] = uCmd; 
    //RetVal |= ComIface_SingleRd(uDevId, BQ7973X_MPSI_CONF_OFFSET, BQ7973X_WRITE_1BYTE, &BQ7973X_uMspiConfRegCheck, TD_ConfigPtr);
    RetVal |= Comif_SingleRead(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_MPSI_CONF_OFFSET,pSvcCfg, WRITE_1BYTE);

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_MSPI_CONF, RetVal, pPmicMgr->uPmicIface, pPmicMgr->eDrvState);
}

/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_SetSPICtrl(uint8 uSPICtrl ,Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/** \brief This function is used to write to SPI_CONF register to configure SPI communication.
 *
 * Extended function description
 *
 * \param[in] uSPICtrl: SPI_CTRL register value.
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_CODE) BQ7973X_SetSPICtrl(Bq7973x_ManagerType *pPmicMgr,uint8 uSPICtrl)
{
    Std_ReturnType RetVal = E_OK;
    uint8 uCmd = uSPICtrl;
    RetVal |= Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_MPSI_CTRL_OFFSET,&uCmd , WRITE_1BYTE);

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_SET_SPI_CTR, RetVal, pPmicMgr->uPmicIface, pPmicMgr->eDrvState);
}


/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_SetSPIExe( uint8 uSPIExe, Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/** \brief This function is used to write to SPI_EXE register.
 *
 * Extended function description
 *
 * \param[in] uSPIExe: SPI_EXE register value.
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
Std_ReturnType BQ7973X_SetSPIExe( Bq7973x_ManagerType *pPmicMgr,uint8 uSPIExe)
{
    Std_ReturnType RetVal = E_OK;
    uint8 uCmd = uSPIExe;
    RetVal |= Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_MSPI_EXE_OFFSET,&uCmd , WRITE_1BYTE);
    return RetVal;
}

/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_WriteSPIData(uint8 *uSPIData, Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/** \brief This function is used to Write to external SPI slave.
 *
 * Extended function description
 *
 * \param[in] uSPIData: Pointer to data to be used to write to SPI slave device.
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
Std_ReturnType BQ7973X_WriteSPIData( Bq7973x_ManagerType *pPmicMgr,uint8 *uSPIData)
{
    Std_ReturnType RetVal = E_OK;
    uint8 ucmd;
    RetVal |= Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_SPI_TX4_OFFSET,uSPIData , WRITE_4BYTE);
    ucmd = BQ7973X_MSPI_EXE_MSPI_GO_MSK;
    RetVal |= Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_MSPI_EXE_OFFSET,&ucmd , WRITE_1BYTE);
    ucmd = BQ7973X_MSPI_EXE_POR_VAL;
    RetVal |= Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_MSPI_EXE_OFFSET,&ucmd , WRITE_1BYTE);

    return RetVal;
}


/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_GetCCOvf(uint8 uActDevID, Com_RspDataCheck_Struct_Type * RspDataCheck, \
 *                                                  Com_TD_Config_Struct_Type *TD_ConfigPtr)
 *********************************************************************************************************************/
/** \brief This function is used to read the indicates a coulomb counter overflow fault.
 *
 * Extended function description
 *
 * \param[in] uActDevID: device ID active in the current communication direction
 * \param[out] RspDataCheck: Pointer to a variable to store FAULT_ADC_MISC register value
 * \param[out] TD_ConfigPtr: Pointer to a transaction descriptor instance
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
Std_ReturnType BQ7973X_GetCCOvf(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
{
    Std_ReturnType RetVal = E_OK;
    RetVal |= Comif_SingleRead(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId ,BQ7973X_FAULT_ADC_MISC_OFFSET,pSvcCfg, READ_1BYTE);
    return RetVal;
}

/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_EnableI2C(uint8 uDevId, Com_TD_Config_Struct_Type *TD_ConfigPtr)
 *********************************************************************************************************************/
/** \brief This function is used to enable I2C
 *
 * Extended function description
 *  \param[in]      pPmicMgr: Manager
 *  \param[in]      pSvcCfg - service config
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
Std_ReturnType BQ7973X_EnableI2C(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
{
    Std_ReturnType RetVal = E_OK;
    uint8 uCmd;

    uCmd = pPmicMgr->pPmcConfig->p_rcBq7973xRegsCfg->zRegCtrl.uControl2 | BQ7973X_CONTROL2_I2C_EN_MSK;
    RetVal |= Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_CONTROL2_OFFSET,&uCmd , WRITE_1BYTE);
    //BQ7973X_uControl2RegData[0u] = uCmd & (~BQ7973X_CONTROL2_PROG_GO_MSK);
    RetVal |= Comif_SingleRead(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_CONTROL2_OFFSET,pSvcCfg, WRITE_1BYTE);

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_ENABLE_I2C, RetVal, pPmicMgr->uPmicIface, pPmicMgr->eDrvState);
}

/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_SetI2CCtrl ( uint8 uI2CCtrl, Bq7973x_ManagerType *pPmicMgr)
 *********************************************************************************************************************/
/** \brief This function is used to set I2C_CTRL register.
 *
 * Extended function description
 *
 * \param[in]      pPmicMgr: Manager
 * \param[in]      pSvcCfg - service config
 * \param[in] uI2CCtrl: I2C_CTRL register value.
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
Std_ReturnType BQ7973X_SetI2CCtrl( Bq7973x_ManagerType *pPmicMgr,uint8 uI2CCtrl)
{

    Std_ReturnType RetVal = E_OK;
    uint8 uCmd;
    uCmd = uI2CCtrl & ((1u << BQ7973X_I2C_CTRL_SEND_POS) - 1u);
    RetVal |= Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_I2C_CTRL_OFFSET ,&uCmd, WRITE_1BYTE);

    //BQ7973X_uI2cCtrlRegData[0u] = uCmd & (~BQ7973X_I2C_CTRL_I2C_GO_MSK); 
    return Pmi_SetErrorDetails(PMI_SVCID_PMC_SET_I2C_CTRL, RetVal, pPmicMgr->uPmicIface, pPmicMgr->eDrvState);
}

/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_SetI2CData(uint8 uDevId, uint8 uI2CData, Com_TD_Config_Struct_Type *TD_ConfigPtr)
 *********************************************************************************************************************/
/** \brief This function is used to set I2C_WR_DATA register.
 *
 * Extended function description
 *
 * \param[in]      pPmicMgr: Manager
 * \param[in]      pSvcCfg - service config
 * \param[in] uI2CData: I2C_WR_DATA register value.
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
Std_ReturnType BQ7973X_SetI2CData( Bq7973x_ManagerType *pPmicMgr,uint8 uI2CData)
{

    Std_ReturnType RetVal = E_OK;
    uint8 uCmd;
    uCmd = uI2CData & BQ7973X_I2C_WR_DATA_DATA_MSK;
    RetVal |= Comif_SingleWrite(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId, BQ7973X_I2C_WR_DATA_OFFSET, &uCmd, WRITE_1BYTE);
    //BQ7973X_uI2cWrDataRegData[0u] = uCmd;
    return Pmi_SetErrorDetails(PMI_SVCID_PMC_SET_I2C_DATA, RetVal, pPmicMgr->uPmicIface, pPmicMgr->eDrvState);
}

/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_GetI2CData(uint8 uDevId, Com_RspDataCheck_Struct_Type * RspDataCheck, Com_TD_Config_Struct_Type *TD_ConfigPtr)
 *********************************************************************************************************************/
/** \brief This function is used to get I2C_WR_DATA register value.
 *
 * Extended function description
 *
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
Std_ReturnType BQ7973X_GetI2CData( Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
{

    Std_ReturnType RetVal = E_OK;
    RetVal |= Comif_SingleRead(pPmicMgr->pComifCtx, pPmicMgr->uPmicDevId ,BQ7973X_I2C_RD_DATA_OFFSET,pSvcCfg, READ_1BYTE);

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_GET_I2C_DATA, RetVal, pPmicMgr->uPmicIface, pPmicMgr->eDrvState);
}






/**********************************************************************************************************************
 * Function name: Bq7973x_ServiceProc
 *********************************************************************************************************************/
/** \brief This function is used to
 *
 * Extended function description
 *
 * \param[in]
 * \param[in]
 * \param[in]
 *
 * \pre Preconditions
 * \post Postconditions
 * \return Bq7973x_ManagerType
 * \retval NULL iferror
 *         Valid pointer on success
 *
 *********************************************************************************************************************/
FUNC(uint8, bq7973x_CODE) Bq7973x_ServiceProc(void *pPmcCtx, uint8 uServiceTyp, uint8 uCmd, void *pData)
{
    Bq7973x_ManagerType *pPmicMgr = (Bq7973x_ManagerType *)  pPmcCtx;
    const ServiceCfgType *pSvcCfg = NULL;
    uint8 uRet = PMI_PMC_INTEGRITY_FAILED;
    uint8 eData = 0;

           
    if(BQ7973X_INTEGRITY_CHECK(pPmicMgr))
    {
        switch(uServiceTyp)
        {
            case PMI_SERVICE_DIAGNOSTICS:
            {
				pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_DIAGNOSTICS];
				if(pSvcCfg)
				{
					uRet = Bq7973x_DiagServiceReq(pPmicMgr->pPmicDiag, uCmd, pData);
				}
			
                break;
            }
            case PMI_SERVICE_CC_CLEAR:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_CC_CLEAR];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = Bq7973x_CCClear(pPmicMgr);
                }
                break;
            }
            case PMI_SERVICE_GET_I2C_DATA:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_GET_I2C_DATA];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = BQ7973X_GetI2CData(pPmicMgr,pSvcCfg);
                }
                break;
            }
            case PMI_SERVICE_GET_CCOVF:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_GET_CCOVF];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = BQ7973X_GetCCOvf(pPmicMgr,pSvcCfg);
                }
                break;
            }
            case PMI_SERVICE_SET_I2C_DATA:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_SET_I2C_DATA];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = BQ7973X_SetI2CData( pPmicMgr,uCmd);
                }
                break;
            }
            case PMI_SERVICE_SET_I2C_CTRL:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_SET_I2C_CTRL];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = BQ7973X_SetI2CCtrl( pPmicMgr,uCmd);
                }
                break;
            }
            case PMI_SERVICE_ENABLE_I2C:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_ENABLE_I2C];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = BQ7973X_EnableI2C(pPmicMgr,pSvcCfg);
                }
                break;
            }
            case PMI_SERVICE_WRITE_SPI_DATA:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_WRITE_SPI_DATA];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = BQ7973X_WriteSPIData(pPmicMgr,pData);
                }
                break;
            }
            case PMI_SERVICE_SET_SPI_EXE:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_SET_SPI_EXE];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = BQ7973X_SetSPIExe(pPmicMgr,uCmd);
                }
                break;
            }
            case PMI_SERVICE_SET_SPI_CTRL:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_SET_SPI_CTRL];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = BQ7973X_SetSPICtrl(pPmicMgr,uCmd);
                }
                break;
            }
            case PMI_SERVICE_MSPI_CONF:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_MSPI_CONF];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = BQ7973X_MSPIConf(pPmicMgr,pSvcCfg);
                }
                break;
            }
            case PMI_SERVICE_EN_ADC_FREEZE:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_EN_ADC_FREEZE];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = BQ7973X_EnADCFreeze(pPmicMgr);
                }
                break;
            }
            case PMI_SERVICE_DIS_ADC_FREEZE:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_DIS_ADC_FREEZE];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = BQ7973X_DisADCFreeze(pPmicMgr);
                }
                break;
            }
            case PMI_SERVICE_SET_ADC_DELAY:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_SET_ADC_DELAY];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = BQ7973X_SetADCDelay(pPmicMgr,pData);
                }
                break;
            }
            case PMI_SERVICE_ENTER_COMM_DEBUG:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_ENTER_COMM_DEBUG];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = BQ7973X_EnterCommDebug(pPmicMgr);
                }
                break;
            }
            case PMI_SERVICE_SET_OC_OUTPUT:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_SET_OC_OUTPUT];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = BQ7973X_SetOCOutput(pPmicMgr,uCmd);
                }
                break;
            }

            case PMI_SERVICE_DIETEMP:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_DIETEMP];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = Bq7973x_GetDieTemp(pPmicMgr, pSvcCfg);
                }
                break;
            }
            case PMI_SERVICE_VGPIO:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_VGPIO];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    uRet = Bq7973x_GetVGpio(pPmicMgr, pSvcCfg);
                }
                break;
            }
            case PMI_SERVICE_CC_ACC_CNT:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_CC_ACC_CNT];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    pSvcCfg->pSvcStat->uRespStatus = RESP_BUSY;
                    uRet = Bq7973x_GetCcAccCnt(pPmicMgr, pSvcCfg);
                }
                break;
            }
            case PMI_SERVICE_CP_VOLT:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_CP_VOLT];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    pSvcCfg->pSvcStat->uRespStatus = RESP_BUSY;
                    uRet = Bq7973x_GetCpVolt(pPmicMgr, pSvcCfg);
                }
                break;
            }
            case PMI_SERVICE_VF_VOLT:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_VF_VOLT];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    pSvcCfg->pSvcStat->uRespStatus = RESP_BUSY;
                    uRet = Bq7973x_GetVf(pPmicMgr, pSvcCfg);
                }
                break;
            }
            case PMI_SERVICE_PACK_CURRENT:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_PACK_CURRENT];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    pSvcCfg->pSvcStat->uRespStatus = RESP_BUSY;
                    uRet = Bq7973x_GetCurrent(pPmicMgr, pSvcCfg);
                }
                break;
            }
			case PMI_SERVICE_RESET:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_RESET];
                if(pSvcCfg&& (pSvcCfg->pSvcStat))
                {
                    uRet = BQ7973X_DevPowerManager(pPmicMgr, uCmd);
                }
                break;
            }
			case PMI_SERVICE_GET_VRF_CAP:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_GET_VRF_CAP];
                if(pSvcCfg&& (pSvcCfg->pSvcStat))
                {
                    uRet = BQ7973X_GetVRrfCap(pPmicMgr, pSvcCfg);
                }
                break;
            }
            case PMI_SERVICE_CONFIG:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_CONFIG];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    pSvcCfg->pSvcStat->uRespStatus = RESP_BUSY;
                    if(pData)
                    {
                        uRet = Bq7973x_ConfigInit(pPmicMgr);
                    }
                }
                break;
            }
            case PMI_SERVICE_STARTUP:
            {
                pSvcCfg = &pPmicMgr->pFuncCfg[PMI_SERVICE_STARTUP];
                if((pSvcCfg) && (pSvcCfg->pSvcStat))
                {
                    pSvcCfg->pSvcStat->uRespStatus = RESP_BUSY;
                    if(pData)
                    {
                        uRet = Bq7973x_StartupInit(pPmicMgr);
                    }
                }
                break;
            }
            default:
            {
                uRet = PMI_PMC_INVALID_SVC_CMD;
                break;
            }
        }

        if(PMI_OK == uRet)
        {
            // GetCounterValue(OsCounter, &pSvcCfg->pSvcStat->cReqTimeMs);
            pSvcCfg->pSvcStat->nReqMsgs++;
        }
    }

    return Pmi_SetErrorDetails(PMI_SVCID_PMC_SERVICE_PROC, uRet, 0, eData);
}


#define BQ7973X_FUNCS_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 *  End of File: bq7973x_funcs.c
 *********************************************************************************************************************/
