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
 *  File:       bq7971x_diag.c
 *  Project:    TIBMS
 *  Module:     DIAG
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for Bq7971x diagnostics
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       24Aug2022    SEM                  0000000000000    Initial version
 * 01.00.01       20March2023  SEM                  0000000000000    Initial version
 *
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Standard Header Files
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Other Header Files
 *********************************************************************************************************************/

#include "tibms_api.h"
#include "tibms_comif.h"
#include "tibms_bmi.h"
#include "tibms_utils.h"
#include "bq7971x_diag.h"

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

/**	Major Software Config C Version number */
#define BQ7971X_DIAG_C_MAJOR_VERSION                    (0x01u)

/**	Minor Software Config C Version number */
#define BQ7971X_DIAG_C_MINOR_VERSION                    (0x00u)

/** Software Patch Config C Version number */
#define BQ7971X_DIAG_C_PATCH_VERSION                    (0x00u)

#if ((BQ7971X_DIAG_C_MAJOR_VERSION != BQ7971X_CFG_MAJOR_VERSION) || \
     (BQ7971X_DIAG_C_MINOR_VERSION != BQ7971X_CFG_MINOR_VERSION) || \
	 (BQ7971X_DIAG_C_PATCH_VERSION != BQ7971X_CFG_PATCH_VERSION))
#error "tibms: Config version numbers of bq7971x_diag.c and bq7971x_cfg.h are inconsistent!"
#endif

#if ((BQ7971X_DIAG_MAJOR_VERSION != BQ7971X_DIAG_C_MAJOR_VERSION) || \
     (BQ7971X_DIAG_MINOR_VERSION != BQ7971X_DIAG_C_MINOR_VERSION) || \
	 (BQ7971X_DIAG_PATCH_VERSION != BQ7971X_DIAG_C_PATCH_VERSION))
#error "tibms: Config version numbers of bq7971x_diag.c and bq7971x_diag.h are inconsistent!"
#endif

#if ((BQ7971X_REGS_MAJOR_VERSION != BQ7971X_DIAG_C_MAJOR_VERSION) || \
     (BQ7971X_REGS_MINOR_VERSION != BQ7971X_DIAG_C_MINOR_VERSION) || \
	 (BQ7971X_REGS_PATCH_VERSION != BQ7971X_DIAG_C_PATCH_VERSION))
#error "tibms: Config version numbers of bq7971x_diag.c and bq7971x_regs.h are inconsistent!"
#endif

/*********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

#define BQ7971X_DIAG_GUARD_START                        (0xDDA1AB1FU)
#define BQ7971X_DIAG_GUARD_END                          (0xDAFEDEEEU)
#define BQ7971X_DIAG_INIT                               (0xDAU)

#define DIAG_INTEGRITY_CHECK(pDiagMgr)                  ((NULL != (pDiagMgr)) && \
                                                        (BQ7971X_DIAG_GUARD_START == pDiagMgr->qGuardStart) && \
                                                        (BQ7971X_DIAG_GUARD_END == pDiagMgr->qGuardEnd) && \
                                                        (BQ7971X_DIAG_INIT == pDiagMgr->uInit))

/*********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

 /*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

#define BQ7971X_DIAG_START_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

static Bq7971x_DiagType zBq7971xDiagMgr[BQ7971X_IFACES];
static DiagStatusType zBmiDiagStatus[BQ7971X_IFACES];

#define BQ7971X_DIAG_STOP_SEC_VAR_NOINIT_UNSPECIFIED
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

#define BQ7971_DIAG_START_SEC_CODE
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 * uint8 Bq7971x_DiagHandler(Bq7971x_DiagType *pDiagMgr)
 *********************************************************************************************************************/
/*! \brief          Diagnostics Handler State Machine
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pSvcCfg - Diag Service Context
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagHandler(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg , uint8 uChannel)
{
    Bq7971x_DiagReqType *pDiagReq;
    uint32 qElapsed;
    uint8 uRet = (uint8)BMI_NOT_OK;
    uint8 uDiagComplete = FALSE;

    pDiagReq = (Bq7971x_DiagReqType *)ctPeQueueGetData(&pDiagMgr->zDiagCmdQueExec);
    if(NULL != pDiagReq)
    {
        if((NULL != pDiagReq->pDiagSvcCfg) && (NULL != pDiagReq->pDiagSvcCfg->pSvcStat))
        {
            if((uint8)DIAG_STEP_READY == pDiagReq->uDiagStep)
            {
                pDiagReq->pDiagSvcCfg->pSvcStat->nReqMsgs++;
                pDiagMgr->pBswCfg->timestamp_req(&pDiagReq->pDiagSvcCfg->pSvcStat->cReqTimeMs, NULL);
            }

            if(pDiagReq->uDiagReqId >= (uint8)BMI_SM_FDTI_CYCLE_INIT)
            {
                uRet = Bq7971x_DiagFdti(pDiagMgr, pSvcCfg, pDiagReq ,uChannel);
                uRet = Bq7971x_DiagFdti_2(pDiagMgr, pSvcCfg, pDiagReq, uChannel);
            }
            else
            {
                uRet = Bq7971x_DiagMpfdi(pDiagMgr, pSvcCfg, pDiagReq,uChannel);
                uRet = Bq7971x_DiagMpfdi_2(pDiagMgr, pSvcCfg, pDiagReq,uChannel);
            }

            pDiagReq->pDiagSvcCfg->pSvcStat->uRespStatus = pDiagReq->uDiagStat;
            pDiagReq->pDiagSvcCfg->pSvcStat->uInfo = pDiagReq->uDiagStep;
            pDiagMgr->uProcState = pDiagReq->uDiagStat;

            if(pDiagReq->uDiagStat >= (uint8)BMI_DIAG_COMPLETE)
            {
                uDiagComplete = TRUE;
                if(pDiagReq->uDiagStat > (uint8)BMI_DIAG_COMPLETE)
                {
                    pDiagReq->pDiagSvcCfg->pSvcStat->uFailure++;
                    if((uint8)BMI_DIAG_ABORT == pDiagReq->uDiagStat)
                    {
                        // Call USER Notification with Error
                    }
                }
            }

            if(0U != uDiagComplete)
            {
                pDiagMgr->pBswCfg->timestamp_req(&pDiagReq->pDiagSvcCfg->pSvcStat->cReqTimeMs, (uint32 *)&qElapsed);
                pDiagReq->pDiagSvcCfg->pSvcStat->cRespMaxTimeMs =(uint8)
                                                     max(qElapsed, pDiagReq->pDiagSvcCfg->pSvcStat->cRespMaxTimeMs);

                pDiagReq->pDiagSvcCfg->pSvcStat->nRespMsgs++;
                if(NULL != pDiagReq->pDiagSvcCfg->cbApplSvc)
                {
                    pDiagReq->pDiagSvcCfg->cbApplSvc(pDiagReq->pDiagSvcCfg->uIface,
                                                     pDiagReq->pDiagSvcCfg->uServiceId,
                                                     pDiagReq->pDiagSvcCfg->uSubId,
                                                     pDiagReq->uDiagStat,
                                                     NULL);
                }
            }
        }
        else
        {
            pDiagMgr->uProcState = (uint8)BMI_DIAG_ERROR;
            uRet = (uint8)BMI_BMC_DIAG_QUE_INVALID_REQ;
        }
    }
    else
    {
        pDiagMgr->uProcState = (uint8)BMI_DIAG_IDLE;
        uRet = (uint8)BMI_OK;
    }

    return uRet;
}

/**********************************************************************************************************************
 * uint8 Bq7971x_DiagScheduleNext(Bq7971x_DiagType *pDiagMgr)
 *********************************************************************************************************************/
/*! \brief          Diagnostics Cleanup of Last run and trigger Next
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *
 *  \reentrant      FALSE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagScheduleNext(Bq7971x_DiagType *pDiagMgr)
{
    Bq7971x_DiagReqType *pDiagReq;
    uint8 uRet = (uint8)BMI_NOT_OK;

    if((uint8)BMI_DIAG_ABORT == pDiagMgr->uProcState)
    {
         // Call USER Notification with Error details
    }

    do
    {
        pDiagReq = (Bq7971x_DiagReqType *)ctPeQueueRemoveData(&pDiagMgr->zDiagCmdQueExec);
        if(NULL != pDiagReq)
        {
            (void)ctPeQueueAddData(&pDiagMgr->zDiagCmdQueFree, pDiagReq);
        }

        uRet = Bq7971x_DiagHandler(pDiagMgr, NULL, 0U);
    }
    while(pDiagMgr->uProcState >= (uint8)BMI_DIAG_COMPLETE);

    return uRet;
}

/*********************************************************************************************************************
 *  External Functions Definition
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  FUNC(uint16, bq7971x_diag_CODE) DiagAbsDiffVoltage(uint16 wVolt1, uint16 wVolt2)
 *********************************************************************************************************************/
/*! \brief          Testing Absolute  Voltage difference
 *
 *  \param[in]      wVolt1 - Voltage Value 1
 *  \param[in]      wVolt2 - Voltage Value 2
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint16
 *  \retval         Diffrence in the voltage is returned
 *  \trace
 *********************************************************************************************************************/

FUNC(uint16, bq7971x_diag_CODE) DiagAbsDiffVoltage(uint16 wVolt1, uint16 wVolt2)
{
    uint16 wDiffVol = BQ7971X_VAL_INVALDVOLVALUE; /* the vol1/2 is 0xFFFF, invalid value */

    /* VOL1 < 65534, The vol1 is positive */
    if(wVolt1 < BQ7971X_VAL_INVALDVOLVALUE)
    {
        /* VOL2 < 32768, The vol2 is positive */
        if (wVolt2 < BQ7971X_VAL_INVALDVOLVALUE)
        {
            /* vol1 less than vol2 */
            if (wVolt1 <= wVolt2)
            {
                wDiffVol = wVolt2 - wVolt1;
            }
            else
            {
                wDiffVol = wVolt1 - wVolt2;
            }
        }
    }

    return wDiffVol;
}

/**********************************************************************************************************************
 *  FUNC(uint16, bq7971x_diag_CODE) DiagAbsDiffTemp(uint16 wVolt1, uint16 wVolt2)
 *********************************************************************************************************************/
/*! \brief          Testing Absolute Tempearature difference
 *
 *  \param[in]      uint16 wTemp1: first temperature register value
 *  \param[in]      uint16 wTemp2: second temperature register value
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint16
 *  \retval         the diff temperature of input temperature1 and 2
 *  \trace
 *********************************************************************************************************************/

FUNC(uint16, bq7971x_diag_CODE) DiagAbsDiffTemp(uint16 wTemp1, uint16 wTemp2)
{
    uint16 wDiffTemp;

    /* wTemp1 < 32768, The wTemp1 is positive */
    if (wTemp1 < BQ7971X_INVALDTEMPVALUE)
    {
        /* wTemp2 < 32768, The wTemp2 is positive */
        if (wTemp2 < BQ7971X_INVALDTEMPVALUE)
        {
            /* wTemp1 less than wTemp2 */
            if (wTemp1 <= wTemp2)
            {
                wDiffTemp = wTemp2 - wTemp1;
            }
            else
            {
                wDiffTemp = wTemp1 - wTemp2;
            }
        }
        /* wTemp2 > 32768, The wTemp2 is negative */
        else if (wTemp2 > BQ7971X_INVALDTEMPVALUE)
        {
            wDiffTemp = ((BQ7971X_INVALDVOLVALUE - wTemp2) + 1u) + wTemp1;
        }
        else
        {
            /* the wTemp2 is 0x8000, invalid value */
            wDiffTemp = BQ7971X_INVALDVOLVALUE;
        }
    }
    /* wTemp1 > 32768, The wTemp1 is negative */
    else if (wTemp1 > BQ7971X_INVALDTEMPVALUE)
    {
        /* wTemp2 < 32768, The wTemp2 is positive */
        if (wTemp2 < BQ7971X_INVALDTEMPVALUE)
        {
            wDiffTemp = ((BQ7971X_INVALDVOLVALUE - wTemp1) + 1u) + wTemp2;
        }
        /* wTemp2 > 32768, The wTemp2 is negative */
        else if (wTemp2 > BQ7971X_INVALDTEMPVALUE)
        {
            /* wTemp1 less than wTemp2 */
            if (wTemp1 <= wTemp2)
            {
                wDiffTemp = wTemp2 - wTemp1;
            }
            else
            {
                wDiffTemp = wTemp1 - wTemp2;
            }
        }
        else
        {
            /* the wTemp2 is 0x8000, invalid value */
            wDiffTemp = BQ7971X_INVALDVOLVALUE;
        }
    }
    else
    {
        /* the wTemp1 is 0x8000, invalid value */
        wDiffTemp = BQ7971X_INVALDVOLVALUE;
    }

    return wDiffTemp;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagDataProc(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          This function is used to Process the Diag read Requests, timeout request, and read request
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pSvcCfg - Diagnostic read context
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval         Pointer to Manager
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagDataProc(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = (uint8)BMI_BMC_DIAG_INTEGRITY_FAILED;

    if((DIAG_INTEGRITY_CHECK(pDiagMgr)) && (NULL != pSvcCfg))
    {
        uRet = Bq7971x_DiagHandler(pDiagMgr, pSvcCfg, 0U);
        if(pDiagMgr->uProcState >= (uint8) BMI_DIAG_COMPLETE)
        {
            uRet = Bq7971x_DiagScheduleNext(pDiagMgr);
        }
        uRet = Bmi_SetErrorDetails(BMI_SVCID_BMC_DIAG_PROC, uRet, pSvcCfg->uSubId, pDiagMgr->uProcState);
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagServiceReq(Bq7971x_DiagType *pDiagMgr, uint8 uReqType)
 *********************************************************************************************************************/
/*! \brief          This function is used to init the Bq7971x core for corresponding chains in the config set
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      uDiagReqId - Diagnostic Request
 *  \param[in]      pData - Customer Data
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval         Pointer to Manager
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagServiceReq(Bq7971x_DiagType *pDiagMgr, uint8 uDiagReqId, void *pData , uint8 uChannel)
{
    uint8 uRet = (uint8)BMI_BMC_DIAG_INTEGRITY_FAILED;
    uint8 uDiagStat = (uint8)BMI_DIAG_ABORT;
    Bq7971x_DiagReqType *pDiagReq;

    uRet = (uint8)BMI_BMC_DIAG_INVALID_CMD;
    if(uDiagReqId < (uint8)BMI_DIAGSERVICE_END)
    {
        uRet = (uint8)BMI_BMC_DIAG_QUE_FAILED;
        pDiagReq = (Bq7971x_DiagReqType *) ctPeQueueRemoveData(&pDiagMgr->zDiagCmdQueFree);
        if(NULL != pDiagReq)
        {
            pDiagReq->pUserData = pData;
            pDiagReq->uDiagReqId = uDiagReqId;
            pDiagReq->uDiagStep = (uint8)DIAG_STEP_READY;
            pDiagReq->uDiagRetry = 0u;
            pDiagReq->uDiagStat = (uint8)BMI_DIAG_IDLE;
            pDiagReq->pDiagSvcCfg = &pDiagMgr->pDiagCfg[uDiagReqId];
            if(uDiagReqId == pDiagReq->pDiagSvcCfg->uSubId)
            {
                uRet = ctPeQueueAddData(&pDiagMgr->zDiagCmdQueExec, pDiagReq);
                if((uint8)QUEUE_E_OK == uRet)
                {
                    uDiagStat = (uint8)BMI_DIAG_RUN;
                    if((uint8)BMI_DIAG_IDLE == pDiagMgr->uProcState)
                    {
                        pDiagMgr->uProcState = (uint8) BMI_DIAG_TRANSIENT;
                        uRet = Bq7971x_DiagHandler(pDiagMgr, pDiagReq->pDiagSvcCfg , uChannel);
                        if(pDiagMgr->uProcState >= (uint8) BMI_DIAG_COMPLETE)
                        {
                            uRet = Bq7971x_DiagScheduleNext(pDiagMgr);
                        }
                        uDiagStat = pDiagMgr->uProcState;
                    }
                }
            }
            else
            {
                (void)ctPeQueueAddData(&pDiagMgr->zDiagCmdQueFree, pDiagReq);
                uDiagStat = (uint8)BMI_DIAG_ERROR;
                uRet = (uint8)BMI_BMC_DIAG_SUBID_INVALID;
            }
        }
        else
        {
           /* If you are here, then probably the Queue size needs to be increased */
           uRet = (uint8)BMI_BMC_DIAG_QUE_FAILED;
        }

    }

    uRet = Bmi_SetErrorDetails(BMI_SVCID_BMC_DIAG_SERVICE, uRet, uDiagStat, uDiagReqId);

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(void, bq7971x_diag_CODE) Bq7971x_DiagInitComplete(const ServiceCfgType *pDiagCfg, uint8 uComInitState)
 *********************************************************************************************************************/
/*! \brief          Callback function for COM INIT status
 *
 *  \param[in]      uIface - Driver Interface
 *  \param[in]      uServiceId - Service ID
 *  \param[in]      uSubId - SubId
 *  \param[in]      uDiagStat - Status
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         void
 *  \retval         None
 *  \trace
 *********************************************************************************************************************/

FUNC(void, bq7971x_diag_CODE) Bq7971x_DiagInitComplete(uint8 uIface, uint8 uServiceId, uint8 uSubId,
                                                       uint8 uDiagStat, void *pData)
{
    Bq7971x_DiagType *pDiagMgr;
    uint8 *pData0 = (uint8 *) pData;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pDiagMgr = &zBq7971xDiagMgr[uIface];
    if(NULL != pDiagMgr)
    {
        if((uint8)BMI_DIAG_COMPLETE == uDiagStat)
        {
            pDiagMgr->uDiagInitState = (uint8)BMI_DIAG_INIT_COMPLETED;
            if(pData0 != NULL)
            {
            	*pData0 = (uint8)uSubId;
            }
            uRet = (uint8)BMI_OK;
        }
        (void)Bmi_SetErrorDetails(BMI_SVCID_BMC_DIAG_STARTUP, uRet, uDiagStat, uIface);
    }
}

/**********************************************************************************************************************
 *  FUNC(uint16, bq7971x_diag_CODE) Bq7971x_GetDiagInitState(Bq7971x_DiagType *pDiagMgr)
 *********************************************************************************************************************/
/*! \brief          Get the Diag Init Status
 *
 *  \param[in]      pDiagMgr
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval         Current Status of Diagnostic
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7971x_diag_CODE) Bq7971x_GetDiagInitState(Bq7971x_DiagType *pDiagMgr)
{
    uint8 uRet = (uint8)BMI_BMC_DIAG_INTEGRITY_FAILED;

    if(DIAG_INTEGRITY_CHECK(pDiagMgr))
    {
        uRet = pDiagMgr->uDiagInitState;
    }

    return uRet;
}

/**********************************************************************************************************************
 * void* Bq7971x_DiagInit(const Bq7971x_ManagerType *pBmicMgrCtx, const Bmi_ConfigType *pBmiCfg,
                          const Comif_ManagerType *pComifCtx, uint8 uBmicIface, uint8 uNAFEsCfg)
 *********************************************************************************************************************/
/*! \brief          This function is used to init the Bq7971x core for corresponding chains in the config set
 *
 *  \param[in]      pBmicMgrCtx - Battery Monitor Chip context
 *  \param[in]      pBmiCfg - Configurations for the Bmi Interface
 *  \param[in]      pComifCtx - Communication Interface context
 *  \param[in]      uBmicIface - Network Iface Number
 *  \param[in]      uNAFEsCfg - Total Number of AFE's
 *  \param[in]      uNfaultPinCfg - NFault PIN
 *  \param[in]      eComType - Communication type
 *  \param[in]      uNBmicsCfg - Total Number of BMIC AFE's
 *  \param[in]      sDevIdCfg - Start ID for BMIC
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         void *
 *  \retval         Pointer to Manager
 *  \trace
 *********************************************************************************************************************/

FUNC(void*, bq7971x_diag_CODE) Bq7971x_DiagInit(const Bq7971x_ManagerType *pBmicMgrCtx, const Bmi_ConfigType *pBmiCfg,
                                                const Comif_ManagerType *pComifCtx, uint8 uBmicIface, uint8 uNAFEsCfg,
                                                uint8 uNfaultPinCfg, uint8 eComType, uint8 uNBmicsCfg, uint8 sDevIdCfg)
{
    Bq7971x_DiagType *pDiagMgr;
    Bq7971x_DiagType *pBq7971xDiagRet = NULL;
    uint16 wTemp;
    uint8 uRet;

    do
    {
        if(uBmicIface >= BQ7971X_IFACES)
        {
            uRet =  (uint8)BMI_BMC_INVALID_IFACECFG;
            break;
        }

        pDiagMgr = &zBq7971xDiagMgr[uBmicIface];
        if(NULL == pDiagMgr)
        {
            uRet =  (uint8)BMI_BMC_INVALID_DIAG_MGR;
            break;
        }

        if(BQ7971X_DIAG_INIT == pDiagMgr->uInit)
        {
            uRet =  (uint8)BMI_BMC_ALREADY_INUSE;
            break;
        }

        (void)memset(pDiagMgr, 0, sizeof(Bq7971x_DiagType));
        if((NULL == pBmiCfg) ||
           (NULL == pBmiCfg->pBmicCfg) ||
           (NULL == pBmiCfg->pFuncCfg) ||
           (NULL == pBmiCfg->pDiagCfg))
        {
            uRet =  (uint8)BMI_BMC_INVALID_CFG;
            break;
        }

        pDiagMgr->pBmicCfg = (Bq7971x_ConfigType *) pBmiCfg->pBmicCfg;
        pDiagMgr->pFuncCfg = pBmiCfg->pFuncCfg;
        pDiagMgr->pDiagCfg = pBmiCfg->pDiagCfg;

        pDiagMgr->pDiagResult = (DiagResultType*) pDiagMgr->pFuncCfg[BMI_SERVICE_DIAGNOSTICS].pDecodeBuf0;
        if(NULL == pDiagMgr->pDiagResult)
        {
            uRet =  (uint8)BMI_BMC_INVALID_CFG;
            break;
        }

        pDiagMgr->pDiagStatus = (DiagStatusType*) &zBmiDiagStatus[uBmicIface];
        if(NULL == pDiagMgr->pDiagStatus)
        {
            uRet =  (uint8)BMI_BMC_INVALID_CFG;
            break;
        }

        (void)memset(pDiagMgr->pDiagStatus, 0x0, sizeof(DiagStatusType));
        pDiagMgr->pBswCfg = pBmiCfg->pBswCfg;
        if(NULL == pDiagMgr->pBswCfg)
        {
            uRet =  (uint8)BMI_BMC_INVALID_BSWCFG;
            break;
        }

        if((NULL == pDiagMgr->pBmicCfg->pDiagCmdQueFreeMemCfg) ||
           (NULL == pDiagMgr->pBmicCfg->pDiagReqDataMemCfg) || (!pDiagMgr->pBmicCfg->uDiagCmdQueCntCfg))
        {
            uRet =  (uint8)BMI_BMC_INVALID_QUECFG;
            break;
        }

        uRet = ctPeQueueCreate(&pDiagMgr->zDiagCmdQueFree, pDiagMgr->pBswCfg->qDiagRes, pDiagMgr->pBswCfg,
                               pDiagMgr->pBmicCfg->pDiagCmdQueFreeMemCfg, pDiagMgr->pBmicCfg->uDiagCmdQueSizeCfg,
                               pDiagMgr->pBmicCfg->uDiagCmdQueCntCfg);
        if((uint8)QUEUE_E_OK != uRet)
        {
            uRet =  (uint8)BMI_BMC_INVALID_QUECFG;
            break;
        }

        for (wTemp = 0; wTemp < pDiagMgr->pBmicCfg->uDiagCmdQueCntCfg; wTemp++)
        {
            uRet = ctPeQueueAddData(&pDiagMgr->zDiagCmdQueFree,
                                    (void*)&pDiagMgr->pBmicCfg->pDiagReqDataMemCfg[wTemp]);
            if((uint8)QUEUE_E_OK != uRet)
            {
                uRet =  (uint8)BMI_BMC_INVALID_QUECFG;
                break;
            }
        }

        if((uint8)BMI_BMC_INVALID_QUECFG == uRet)
        {
            break;
        }

        if(NULL == pDiagMgr->pBmicCfg->pDiagCmdQueExecMemCfg)
        {
            uRet =  (uint8)BMI_BMC_INVALID_QUECFG;
            break;
        }

        uRet = ctPeQueueCreate(&pDiagMgr->zDiagCmdQueExec, pDiagMgr->pBswCfg->qDiagRes, pDiagMgr->pBswCfg,
                                pDiagMgr->pBmicCfg->pDiagCmdQueExecMemCfg, pDiagMgr->pBmicCfg->uDiagCmdQueSizeCfg,
                                pDiagMgr->pBmicCfg->uDiagCmdQueCntCfg);
        if((uint8)QUEUE_E_OK != uRet)
        {
            uRet =  (uint8)BMI_BMC_INVALID_QUECFG;
            break;
        }

        if(NULL == pComifCtx)
        {
            uRet =  (uint8)BMI_BMC_INVALID_COMIFMGR;
            break;
        }

        pDiagMgr->pComifCtx = pComifCtx;
        if(NULL == pBmicMgrCtx)
        {
            uRet =  (uint8)BMI_BMC_DIAG_INVALID_BMCMGR;
            break;
        }
        pDiagMgr->pBmcMgr = pBmicMgrCtx;
        pDiagMgr->uBmicIface = uBmicIface;
        pDiagMgr->uNAFEs = uNAFEsCfg;
        pDiagMgr->uNBmics = uNBmicsCfg;
        pDiagMgr->sDevId = sDevIdCfg;
        pDiagMgr->uDioNFaultPin = uNfaultPinCfg;
        pDiagMgr->eComType = eComType;
        pDiagMgr->uDiagRestartReq = FALSE;

        pDiagMgr->uProcState =  (uint8)BMI_DIAG_IDLE;
        pDiagMgr->uDiagInitState =  (uint8)BMI_DIAG_INIT_PROGRESS;

        wTemp = pDiagMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uOVUV_Thresh[0u];
        if(wTemp < BQ7971X_OV_THR_REG_VAL1)
        {
            pDiagMgr->wOvThreshVal = (((wTemp * BQ7971X_OV_THR_STEP) + BQ7971X_OV_THR_RES) * BQ7971X_LSB_STANDARD);
        }
        else if((wTemp >= BQ7971X_OV_THR_REG_VAL2) && (wTemp < BQ7971X_OV_THR_REG_VAL3))
        {
            pDiagMgr->wOvThreshVal = ((((wTemp - BQ7971X_OV_THR_REG_VAL2) * BQ7971X_OV_THR_STEP) +
                                         BQ7971X_OV_THR_VAL1) * BQ7971X_LSB_STANDARD);
        }
        else if((wTemp >= BQ7971X_OV_THR_REG_VAL4) && (wTemp < BQ7971X_OV_THR_REG_VAL5))
        {
            pDiagMgr->wOvThreshVal = ((((wTemp - BQ7971X_OV_THR_REG_VAL4) * BQ7971X_OV_THR_STEP) +
                                         BQ7971X_OV_THR_VAL2) * BQ7971X_LSB_STANDARD);
        }
        else
        {
            pDiagMgr->wOvThreshVal = BQ7971X_OV_THR_RES * BQ7971X_LSB_STANDARD;
        }

        wTemp = pDiagMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uOVUV_Thresh[1u];
        if(wTemp <= BQ7971X_UV_THR_REG_VAL)
        {
            pDiagMgr->wUvThreshVal = ((wTemp * BQ7971X_UV_THR_STEP) + BQ7971X_UV_THR_RES) * BQ7971X_LSB_STANDARD;
        }
        else
        {
            pDiagMgr->wUvThreshVal = BQ7971X_UV_THR_VAL * BQ7971X_LSB_STANDARD;
        }

        wTemp = (pDiagMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uOTUT_Thresh & BQ7971X_OTUT_THRESH_OT_THR_MSK);
        if(wTemp <= BQ7971X_OT_THR_REG_VAL)
        {
            pDiagMgr->wOtThreshVal = ((wTemp + BQ7971X_OT_THR_RES) * BQ7971X_STEP_CENTI_NTC);
        }
        else
        {
            pDiagMgr->wOtThreshVal = (BQ7971X_OT_THR_VAL * BQ7971X_STEP_CENTI_NTC);
        }

        pDiagMgr->wUtThreshVal = ((((pDiagMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uOTUT_Thresh >>
                                    BQ7971X_OTUT_THRESH_UT_THR_POS) * BQ7971X_UT_THR_STEP) +
                                    BQ7971X_UT_THR_RES) * BQ7971X_STEP_CENTI_NTC);

        pDiagMgr->qGuardStart = BQ7971X_DIAG_GUARD_START;
        pDiagMgr->qGuardEnd = BQ7971X_DIAG_GUARD_END;
        pDiagMgr->uInit = BQ7971X_DIAG_INIT;

        pBq7971xDiagRet = pDiagMgr;
        uRet =  (uint8)BMI_OK;
    }
    while(NULL != 0);

    (void)Bmi_SetErrorDetails(BMI_SVCID_BMC_DIAG_INIT, uRet, uBmicIface, uNAFEsCfg);

    return pBq7971xDiagRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagDeInit(uint8 uBmicIface)
 *********************************************************************************************************************/
/*! \brief          This function is used to init the Bq7971x core for corresponding chains in the config set
 *
 *  \param[in]      uBmicIface  - Network Iface Number

 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval         Pointer to Manager
 *  \trace
 *********************************************************************************************************************/
FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagDeInit(uint8 uBmicIface)
{
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;
    Bq7971x_DiagType *pBmicDiagMgr = &zBq7971xDiagMgr[uBmicIface];
    
    if(DIAG_INTEGRITY_CHECK(pBmicDiagMgr))
    {
        (void)memset(&zBq7971xDiagMgr[uBmicIface],0, sizeof(Bq7971x_DiagType));
        uRet = Bmi_SetErrorDetails(BMI_SVCID_BMC_DEINIT, BMI_OK, pBmicDiagMgr->uBmicIface, 0u);
    }
    return uRet;
}

#define BQ7971_DIAG_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * End of File: bq7971x_diag.c
 *********************************************************************************************************************/
