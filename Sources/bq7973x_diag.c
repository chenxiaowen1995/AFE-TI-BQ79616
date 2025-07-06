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
 *  File:       bq7973x_diag.c
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
#include "tibms_pmi.h"
#include "tibms_utils.h"
#include "bq7973x_diag.h"

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/



/*********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

#define bq7973x_diag_GUARD_START                        (0xDDA1AB1FU)
#define bq7973x_diag_GUARD_END                          (0xDAFEDEEEU)
#define bq7973x_diag_INIT                               (0xDAu)

#define DIAG_INTEGRITY_CHECK(pDiagMgr)                  ((NULL != (pDiagMgr)) && \
                                                        (bq7973x_diag_GUARD_START == pDiagMgr->qGuardStart) && \
                                                        (bq7973x_diag_GUARD_END == pDiagMgr->qGuardEnd) && \
                                                        (bq7973x_diag_INIT == pDiagMgr->uInit))

/*********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

 /*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

#define bq7973x_diag_START_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

 Bq7973x_DiagType zBq7973xDiagMgr[BQ7973X_IFACES];
static DiagStatusType zPmiDiagStatus[BQ7973X_IFACES];

#define bq7973x_diag_STOP_SEC_VAR_NOINIT_UNSPECIFIED
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
 * uint8 bq7973x_diagHandler(Bq7973x_DiagType *pDiagMgr)
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

STATIC FUNC(uint8, bq7973x_diag_CODE) bq7973x_diagHandler(Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg)
{
    Bq7973x_DiagReqType *pDiagReq;
    uint32 qElapsed;
    uint8 uRet = (uint8)BMI_NOT_OK;
    uint8 uDiagComplete = (uint8)FALSE;

    pDiagReq = (Bq7973x_DiagReqType*) ctPeQueueGetData(&pDiagMgr->zDiagCmdQueExec);
    if(pDiagReq != NULL )
    {
        if(((pDiagReq->pDiagSvcCfg)!= NULL) && ((pDiagReq->pDiagSvcCfg->pSvcStat)!= NULL))
        {
            if((uint8)DIAG_STEP_READY == pDiagReq->uDiagStep)
            {
                pDiagReq->pDiagSvcCfg->pSvcStat->nReqMsgs++;
                pDiagMgr->pBswCfg->timestamp_req(&pDiagReq->pDiagSvcCfg->pSvcStat->cReqTimeMs, NULL);
            }
            /*check on the type of the diag request based on the macro PMI_FDTI_CYCLE_INIT */
            if(pDiagReq->uDiagReqId >= PMI_FDTI_CYCLE_INIT)
            {
                uRet = Bq7973x_DiagFdti(pDiagMgr, pSvcCfg, pDiagReq);
            }
            else
            {
                uRet = Bq7973x_DiagMpfdi(pDiagMgr, pSvcCfg, pDiagReq);
            }

            pDiagReq->pDiagSvcCfg->pSvcStat->uRespStatus = pDiagReq->uDiagStat;
            pDiagReq->pDiagSvcCfg->pSvcStat->uInfo = pDiagReq->uDiagStep;
            pDiagMgr->uProcState = pDiagReq->uDiagStat;

            if(pDiagReq->uDiagStat >= (uint8)PMI_DIAG_COMPLETE)
            {
                uDiagComplete = (uint8)TRUE;
                if(pDiagReq->uDiagStat > (uint8)PMI_DIAG_COMPLETE)
                {
                    pDiagReq->pDiagSvcCfg->pSvcStat->uFailure++;
                    if((uint8)PMI_DIAG_ABORT == pDiagReq->uDiagStat)
                    {
                        // Call USER Notification with Error
                    }
                }
            }

            if(uDiagComplete)
            {
                pDiagMgr->pBswCfg->timestamp_req(&pDiagReq->pDiagSvcCfg->pSvcStat->cReqTimeMs, (uint32 *)&qElapsed);
                pDiagReq->pDiagSvcCfg->pSvcStat->cRespMaxTimeMs =
                                                     (uint8) max(qElapsed, pDiagReq->pDiagSvcCfg->pSvcStat->cRespMaxTimeMs);

                pDiagReq->pDiagSvcCfg->pSvcStat->nRespMsgs++;
                if((pDiagReq->pDiagSvcCfg->cbApplSvc) != NULL)
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
            pDiagMgr->uProcState = (uint8)PMI_DIAG_ERROR;
            uRet = (uint8)PMI_PMC_DIAG_QUE_INVALID_REQ;
        }
    }
    else
    {
        pDiagMgr->uProcState = (uint8)PMI_DIAG_IDLE;
        uRet = (uint8)PMI_OK;
    }

    return uRet;
}

/**********************************************************************************************************************
 * uint8 bq7973x_diagScheduleNext(Bq7973x_DiagType *pDiagMgr)
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

STATIC FUNC(uint8, bq7973x_diag_CODE) bq7973x_diagScheduleNext(Bq7973x_DiagType *pDiagMgr)
{
    Bq7973x_DiagReqType *pDiagReq;
    uint8 uRet = (uint8)BMI_NOT_OK;

    if((uint8)PMI_DIAG_ABORT == pDiagMgr->uProcState)
    {
         // Call USER Notification with Error details
    }

    do
    {
        pDiagReq =(Bq7973x_DiagReqType *) ctPeQueueRemoveData(&pDiagMgr->zDiagCmdQueExec);
        if(pDiagReq != NULL)
        {
           (void) ctPeQueueAddData(&pDiagMgr->zDiagCmdQueFree, pDiagReq);
        }

        uRet = bq7973x_diagHandler(pDiagMgr, NULL);
    }
    while(pDiagMgr->uProcState >= (uint8)PMI_DIAG_COMPLETE);

    return uRet;
}

/*********************************************************************************************************************
 *  External Functions Definition
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  FUNC(uint16, bq7973x_diag_CODE) BQ7973x_DiagAbsDiffVoltage(uint16 wVolt1, uint16 wVolt2)
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

FUNC(uint16, bq7973x_diag_CODE) BQ7973x_DiagAbsDiffVoltage(uint16 wVolt1, uint16 wVolt2)
{
    uint16 wDiffVol = BQ7973X_VAL_INVALDVOLVALUE; /* the vol1/2 is 0xFFFF, invalid value */

    /* VOL1 < 65534, The vol1 is positive */
    if(wVolt1 < BQ7973X_VAL_INVALDVOLVALUE)
    {
        /* VOL2 < 32768, The vol2 is positive */
        if (wVolt2 < BQ7973X_VAL_INVALDVOLVALUE)
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
 *  FUNC(uint16, bq7973x_diag_CODE) BQ7973x_DiagAbsDiffTemp(uint16 wVolt1, uint16 wVolt2)
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

FUNC(uint16, bq7973x_diag_CODE) BQ7973x_DiagAbsDiffTemp(uint16 wTemp1, uint16 wTemp2)
{
    uint16 wDiffTemp;

    /* wTemp1 < 32768, The wTemp1 is positive */
    if (wTemp1 < BQ7973X_INVALDTEMPVALUE)
    {
        /* wTemp2 < 32768, The wTemp2 is positive */
        if (wTemp2 < BQ7973X_INVALDTEMPVALUE)
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
        else if (wTemp2 > BQ7973X_INVALDTEMPVALUE)
        {
            wDiffTemp = ((BQ7973X_INVALDVOLVALUE - wTemp2) + 1u) + wTemp1;
        }
        else
        {
            /* the wTemp2 is 0x8000, invalid value */
            wDiffTemp = BQ7973X_INVALDVOLVALUE;
        }
    }
    /* wTemp1 > 32768, The wTemp1 is negative */
    else if (wTemp1 > BQ7973X_INVALDTEMPVALUE)
    {
        /* wTemp2 < 32768, The wTemp2 is positive */
        if (wTemp2 < BQ7973X_INVALDTEMPVALUE)
        {
            wDiffTemp = ((BQ7973X_INVALDVOLVALUE - wTemp1) + 1u) + wTemp2;
        }
        /* wTemp2 > 32768, The wTemp2 is negative */
        else if (wTemp2 > BQ7973X_INVALDTEMPVALUE)
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
            wDiffTemp = BQ7973X_INVALDVOLVALUE;
        }
    }
    else
    {
        /* the wTemp1 is 0x8000, invalid value */
        wDiffTemp = BQ7973X_INVALDVOLVALUE;
    }

    return wDiffTemp;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_diag_CODE) Bq7973x_DiagDataProc(Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg)
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

FUNC(uint8, bq7973x_diag_CODE) Bq7973x_DiagDataProc(Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg)
{
    uint8 uRet =(uint8) PMI_PMC_DIAG_INTEGRITY_FAILED;

    if((DIAG_INTEGRITY_CHECK(pDiagMgr)) && (pSvcCfg != NULL))
    {
        uRet = bq7973x_diagHandler(pDiagMgr, pSvcCfg);
        if(pDiagMgr->uProcState >= (uint8)PMI_DIAG_COMPLETE)
        {
            uRet = bq7973x_diagScheduleNext(pDiagMgr);
        }
        uRet = Pmi_SetErrorDetails(PMI_SVCID_PMC_DIAG_PROC, uRet, pSvcCfg->uSubId, pDiagMgr->uProcState);
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_diag_CODE) Bq7973x_DiagServiceReq(Bq7973x_DiagType *pDiagMgr, uint8 uDiagReqId, void *pData)
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

FUNC(uint8, bq7973x_diag_CODE) Bq7973x_DiagServiceReq(Bq7973x_DiagType *pDiagMgr, uint8 uDiagReqId, void *pData)
{
    uint8 uRet = (uint8)PMI_PMC_DIAG_INTEGRITY_FAILED;
    uint8 uDiagStat = (uint8)PMI_DIAG_ABORT;
    Bq7973x_DiagReqType *pDiagReq;

    uRet = (uint8)PMI_PMC_DIAG_INVALID_CMD;
    if(uDiagReqId < (uint8)PMI_DIAGSERVICE_END)
    {
        uRet = (uint8)PMI_PMC_DIAG_QUE_FAILED;
        pDiagReq = (Bq7973x_DiagReqType *) ctPeQueueRemoveData(&pDiagMgr->zDiagCmdQueFree);
        if(pDiagReq != NULL)
        {
            pDiagReq->pUserData = pData;
            pDiagReq->uDiagReqId = uDiagReqId;
            pDiagReq->uDiagStep = (uint8)DIAG_STEP_READY;
            pDiagReq->uDiagRetry = 0u;
            pDiagReq->uDiagStat = (uint8)PMI_DIAG_IDLE;
            pDiagReq->pDiagSvcCfg = &pDiagMgr->pDiagCfg[uDiagReqId];
            if(uDiagReqId == pDiagReq->pDiagSvcCfg->uSubId)
            {
                uRet = ctPeQueueAddData(&pDiagMgr->zDiagCmdQueExec, pDiagReq);
                if((uint8)QUEUE_E_OK == uRet)
                {
                    uDiagStat = (uint8)PMI_DIAG_RUN;
                    if((uint8)PMI_DIAG_IDLE == pDiagMgr->uProcState)
                    {
                        pDiagMgr->uProcState = (uint8)PMI_DIAG_TRANSIENT;
                        uRet = bq7973x_diagHandler(pDiagMgr, pDiagReq->pDiagSvcCfg);
                        if(pDiagMgr->uProcState >= (uint8)PMI_DIAG_COMPLETE)
                        {
                            uRet = bq7973x_diagScheduleNext(pDiagMgr);
                        }
                        uDiagStat = pDiagMgr->uProcState;
                    }
                }
            }
            else
            {
                (void)ctPeQueueAddData(&pDiagMgr->zDiagCmdQueFree, pDiagReq);
                uDiagStat = (uint8)PMI_DIAG_ERROR;
                uRet = (uint8)PMI_PMC_DIAG_SUBID_INVALID;
            }
        }
        else
        {
           /* If you are here, then probably the Queue size needs to be increased */
           uRet = (uint8)BMI_BMC_DIAG_QUE_FAILED;
        }
    }

    uRet = Pmi_SetErrorDetails(PMI_SVCID_PMC_DIAG_SERVICE, uRet, uDiagStat, uDiagReqId);

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(void, bq7973x_diag_CODE) Bq7973x_DiagInitComplete(uint8 uIface, uint8 uServiceId, uint8 uSubId,
                                                       uint8 uDiagStat, void *pData)
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
FUNC(void, BQ7973x_diag_CODE) Bq7973x_DiagInitComplete(uint8 uIface, uint8 uServiceId, uint8 uSubId,
                                                       uint8 uDiagStat, void *pData)
{
    Bq7973x_DiagType *pDiagMgr;
    uint8 *pData0 = (uint8 *) pData;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pDiagMgr = &zBq7973xDiagMgr[uIface];
    if(pDiagMgr != NULL)
    {
        if((uint8)PMI_DIAG_COMPLETE == uDiagStat)
        {
            pDiagMgr->uDiagInitState = (uint8)PMI_DIAG_INIT_COMPLETED;
            if(pData0 != NULL)
            {
            	pData0[0] = uSubId;
            }
            uRet = (uint8)PMI_OK;
        }
        (void)  Pmi_SetErrorDetails(PMI_SVCID_PMC_DIAGSTARTUP_INIT, uRet, uDiagStat, uIface);
    }
}

/**********************************************************************************************************************
 *  FUNC(uint16, bq7973x_diag_CODE) BQ7973X_GetDiagInitState(Bq7973x_DiagType *pDiagMgr)
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

FUNC(uint8, bq7973x_diag_CODE) BQ7973X_GetDiagInitState(Bq7973x_DiagType *pDiagMgr)
{
    uint8 uRet = (uint8)PMI_PMC_DIAG_INTEGRITY_FAILED;

    if(DIAG_INTEGRITY_CHECK(pDiagMgr))
    {
        uRet = pDiagMgr->uDiagInitState;
    }

    return uRet;
}

/**********************************************************************************************************************
 * void* BQ7973x_DiagInit(const BQ7973X_ManagerType *pPmicMgrCtx, const Pmi_ConfigType *pPmiCfg,
                          const Comif_ManagerType *pComifCtx, uint8 uPmicIface, uint8 uNAFEsCfg)
 *********************************************************************************************************************/
/*! \brief          This function is used to init the Bq7971x core for corresponding chains in the config set
 *
 *  \param[in]      pPmicMgrCtx - Battery Monitor Chip context
 *  \param[in]      pPmiCfg - Configurations for the Pmi Interface
 *  \param[in]      pComifCtx - Communication Interface context
 *  \param[in]      uPmicIface - Network Iface Number
 *  \param[in]      uNAFEsCfg - Total Number of AFE's
 *  \param[in]      uNfaultPinCfg - NFault PIN
 *  \param[in]      eComType - Communication type
 *  \param[in]      uNPmicsCfg - Total Number of PMIC AFE's
 *  \param[in]      sDevIdCfg - Start ID for PMIC
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         void *
 *  \retval         Pointer to Manager
 *  \trace
 *********************************************************************************************************************/

FUNC(void*, bq7973x_diag_CODE) BQ7973x_DiagInit(const Bq7973x_ManagerType *pPmicMgrCtx, const Pmi_ConfigType *pPmiCfg,
                                                const Comif_ManagerType *pComifCtx, uint8 uPmicIface, uint8 uNAFEsCfg,
                                                uint8 uNfaultPinCfg, uint8 eComType, uint8 uNPmicsCfg, uint8 sDevIdCfg)
{
    Bq7973x_DiagType *pDiagMgr;
    Bq7973x_DiagType *pBq7971xDiagRet = NULL;
    uint16 wTemp;
    uint8 uRet;

    do
    {
        if(uPmicIface >= BQ7973X_IFACES)
        {
            uRet = (uint8)PMI_PMC_INVALID_IFACECFG;
            break;
        }

        pDiagMgr = &zBq7973xDiagMgr[uPmicIface];
        if(NULL == pDiagMgr)
        {
            uRet = (uint8)PMI_PMC_INVALID_DIAG_MGR;
            break;
        }

        if(bq7973x_diag_INIT == pDiagMgr->uInit)
        {
            uRet = (uint8)PMI_PMC_ALREADY_INUSE;
            break;
        }

       (void) memset(pDiagMgr, 0, sizeof(Bq7973x_DiagType));
        if((NULL == pPmiCfg) ||
           (NULL == pPmiCfg->pPmicCfg) ||
           (NULL == pPmiCfg->pFuncCfg) ||
           (NULL == pPmiCfg->pDiagCfg))
        {
            uRet = (uint8)PMI_PMC_INVALID_CFG;
            break;
        }
        pDiagMgr->pPmcCfg = (Bq7973x_ConfigType *) pPmiCfg->pPmicCfg;
        pDiagMgr->pFuncCfg = pPmiCfg->pFuncCfg;
        pDiagMgr->pDiagCfg = pPmiCfg->pDiagCfg;

        pDiagMgr->pDiagResult =(DiagPmiResultType*) pDiagMgr->pFuncCfg[PMI_SERVICE_DIAGNOSTICS].pDecodeBuf0;
        if(NULL == pDiagMgr->pDiagResult)
        {
            uRet = (uint8)PMI_PMC_INVALID_CFG;
            break;
        }

        pDiagMgr->pDiagStatus = (DiagPmiStatusType*)  &zPmiDiagStatus[uPmicIface];
        if(NULL == pDiagMgr->pDiagStatus)
        {
            uRet = (uint8)PMI_PMC_INVALID_CFG;
            break;
        }

        (void) memset(pDiagMgr->pDiagStatus, 0x0, sizeof(DiagStatusType));
        pDiagMgr->pBswCfg = pPmiCfg->pBswCfg;
        if(NULL == pDiagMgr->pBswCfg)
        {
            uRet = (uint8)PMI_PMC_INVALID_BSWCFG;
            break;
        }

        if((NULL == pDiagMgr->pPmcCfg->pDiagCmdQueFreeMemCfg) ||
           (NULL == pDiagMgr->pPmcCfg->pDiagReqDataMemCfg) || (!pDiagMgr->pPmcCfg->uDiagCmdQueCntCfg))
        {
            uRet = (uint8)PMI_PMC_INVALID_QUECFG;
            break;
        }

        uRet = ctPeQueueCreate(&pDiagMgr->zDiagCmdQueFree, pDiagMgr->pBswCfg->qDiagRes, pDiagMgr->pBswCfg,
                               pDiagMgr->pPmcCfg->pDiagCmdQueFreeMemCfg, pDiagMgr->pPmcCfg->uDiagCmdQueSizeCfg,
                               pDiagMgr->pPmcCfg->uDiagCmdQueCntCfg);
        if(QUEUE_E_OK != uRet)
        {
            uRet = (uint8)PMI_PMC_INVALID_QUECFG;
            break;
        }

        for (wTemp = 0; wTemp < pDiagMgr->pPmcCfg->uDiagCmdQueCntCfg; wTemp++)
        {
            uRet = ctPeQueueAddData(&pDiagMgr->zDiagCmdQueFree,
                                    (void*)&pDiagMgr->pPmcCfg->pDiagReqDataMemCfg[wTemp]);
            if((uint8)QUEUE_E_OK != uRet)
            {
                uRet = (uint8)PMI_PMC_INVALID_QUECFG;
                break;
            }
        }

        if((uint8)PMI_PMC_INVALID_QUECFG == uRet)
        {
            break;
        }

        if(NULL == pDiagMgr->pPmcCfg->pDiagCmdQueExecMemCfg)
        {
            uRet = (uint8)PMI_PMC_INVALID_QUECFG;
            break;
        }

        uRet = ctPeQueueCreate(&pDiagMgr->zDiagCmdQueExec, pDiagMgr->pBswCfg->qDiagRes, pDiagMgr->pBswCfg,
                                pDiagMgr->pPmcCfg->pDiagCmdQueExecMemCfg, pDiagMgr->pPmcCfg->uDiagCmdQueSizeCfg,
                                pDiagMgr->pPmcCfg->uDiagCmdQueCntCfg);
        if((uint8)QUEUE_E_OK != uRet)
        {
            uRet = (uint8)PMI_PMC_INVALID_QUECFG;
            break;
        }

        if(NULL == pComifCtx)
        {
            uRet = (uint8)PMI_PMC_INVALID_COMIFMGR;
            break;
        }

        pDiagMgr->pComifCtx = pComifCtx;
        if(NULL == pPmicMgrCtx)
        {
            uRet = (uint8)PMI_PMC_DIAG_INVALID_BMCMGR;
            break;
        }
        pDiagMgr->pPmcMgr = pPmicMgrCtx;
        pDiagMgr->uPmicIface = uPmicIface;
        pDiagMgr->uNAFEs = uNAFEsCfg;
        pDiagMgr->uNPmics = uNPmicsCfg;
        pDiagMgr->sDevId = sDevIdCfg;
        pDiagMgr->uDioNFaultPin = uNfaultPinCfg;
        pDiagMgr->eComType = eComType;
        pDiagMgr->uDiagRestartReq = (uint8)FALSE;

        pDiagMgr->uProcState = (uint8)PMI_DIAG_IDLE;
        pDiagMgr->uDiagInitState = (uint8)PMI_DIAG_INIT_PROGRESS;

        wTemp = pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uOVUV_Thresh[0u];
        if(wTemp < BQ7973X_OV_THR_REG_VAL1)
        {
            pDiagMgr->wOvThreshVal = (((wTemp * BQ7973X_OV_THR_STEP) + BQ7973X_OV_THR_RES) * BQ7973X_LSB_STANDARD);
        }
        else if((wTemp >= BQ7973X_OV_THR_REG_VAL2) && (wTemp < BQ7973X_OV_THR_REG_VAL3))
        {
            pDiagMgr->wOvThreshVal = ((((wTemp - BQ7973X_OV_THR_REG_VAL2) * BQ7973X_OV_THR_STEP) +
                                         BQ7973X_OV_THR_VAL1) * BQ7973X_LSB_STANDARD);
        }
        else if((wTemp >= BQ7973X_OV_THR_REG_VAL4) && (wTemp < BQ7973X_OV_THR_REG_VAL5))
        {
            pDiagMgr->wOvThreshVal = ((((wTemp - BQ7973X_OV_THR_REG_VAL4) * BQ7973X_OV_THR_STEP) +
                                         BQ7973X_OV_THR_VAL2) * BQ7973X_LSB_STANDARD);
        }
        else
        {
            pDiagMgr->wOvThreshVal = BQ7973X_OV_THR_RES * BQ7973X_LSB_STANDARD;
        }

        wTemp = pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uOVUV_Thresh[1u];
        if(wTemp <= BQ7973X_UV_THR_REG_VAL)
        {
            pDiagMgr->wUvThreshVal = ((wTemp * BQ7973X_UV_THR_STEP) + BQ7973X_UV_THR_RES) * BQ7973X_LSB_STANDARD;
        }
        else
        {
            pDiagMgr->wUvThreshVal = BQ7973X_UV_THR_VAL * BQ7973X_LSB_STANDARD;
        }

        wTemp = (uint16) (pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uOTUT_Thresh & BQ7973X_OTUT_THRESH_OT_THR_MSK);
        if(wTemp <= BQ7973X_OT_THR_REG_VAL)
        {
            pDiagMgr->wOtThreshVal = ((wTemp + BQ7973X_OT_THR_RES) * BQ7973X_STEP_CENTI_NTC);
        }
        else
        {
            pDiagMgr->wOtThreshVal = (BQ7973X_OT_THR_VAL * BQ7973X_STEP_CENTI_NTC);
        }

        pDiagMgr->wUtThreshVal = (uint16) ((((pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uOTUT_Thresh >>
                                    BQ7973X_OTUT_THRESH_UT_THR_POS) * BQ7973X_UT_THR_STEP) +
                                    BQ7973X_UT_THR_RES) * BQ7973X_STEP_CENTI_NTC);

        pDiagMgr->qGuardStart = bq7973x_diag_GUARD_START;
        pDiagMgr->qGuardEnd = bq7973x_diag_GUARD_END;
        pDiagMgr->uInit = bq7973x_diag_INIT;

        pBq7971xDiagRet = pDiagMgr;
        uRet = PMI_OK;
    }
    while(0);

    (void) Pmi_SetErrorDetails(PMI_SVCID_PMC_DIAG_INIT, uRet, uPmicIface, uNAFEsCfg);

    return (void*)pBq7971xDiagRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7973x_diag_CODE) bq7973x_diagDeInit(uint8 uPmicIface)
 *********************************************************************************************************************/
/*! \brief          This function is used to init the Bq7971x core for corresponding chains in the config set
 *
 *  \param[in]      uPmicIface  - Network Iface Number

 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval         Pointer to Manager
 *  \trace
 *********************************************************************************************************************/
FUNC(uint8, bq7973x_diag_CODE) bq7973x_diagDeInit(uint8 uPmicIface)
{
    uint8 uRet = (uint8) PMI_PMC_INTEGRITY_FAILED;
    Bq7973x_DiagType *pPmicDiagMgr = &zBq7973xDiagMgr[uPmicIface];
    
    if(DIAG_INTEGRITY_CHECK(pPmicDiagMgr))
    {
       (void) memset(&zBq7973xDiagMgr[uPmicIface],0, sizeof(Bq7973x_DiagType));
        uRet = Pmi_SetErrorDetails(PMI_SVCID_PMC_DIAG_DEINIT, PMI_OK, pPmicDiagMgr->uPmicIface, 0u);
    }
    return uRet;
}

#define BQ7971_DIAG_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * End of File: bq7973x_diag.c
 *********************************************************************************************************************/
