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
 *  File:       bq7971x_diagmpfdi.c
 *  Project:    TIBMS
 *  Module:     DIAG
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for Diagnostic MPFDI functionalities
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       24Aug2022    SEM                  0000000000000    Initial version
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
#include "bq7971x_diag.h"

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

/**	Major Software Config C Version number */
#define BQ7971X_DIAGSTRUP_C_MAJOR_VERSION             (0x01u)

/**	Minor Software Config C Version number */
#define BQ7971X_DIAGSTRUP_C_MINOR_VERSION             (0x00u)

/** Software Patch Config C Version number */
#define BQ7971X_DIAGSTRUP_C_PATCH_VERSION             (0x00u)

#if ((BQ7971X_DIAGSTRUP_C_MAJOR_VERSION != BQ7971X_CFG_MAJOR_VERSION) || \
     (BQ7971X_DIAGSTRUP_C_MINOR_VERSION != BQ7971X_CFG_MINOR_VERSION) || \
	 (BQ7971X_DIAGSTRUP_C_PATCH_VERSION != BQ7971X_CFG_PATCH_VERSION))
#error "tibms: Config version numbers of bq7971x_diagstartup.c and bq7971x_cfg.h are inconsistent!"
#endif

#if ((BQ7971X_DIAG_MAJOR_VERSION != BQ7971X_DIAGSTRUP_C_MAJOR_VERSION) || \
     (BQ7971X_DIAG_MINOR_VERSION != BQ7971X_DIAGSTRUP_C_MINOR_VERSION) || \
	 (BQ7971X_DIAG_PATCH_VERSION != BQ7971X_DIAGSTRUP_C_PATCH_VERSION))
#error "tibms: Config version numbers of bq7971x_diagstartup.c and bq7971x_diag.h are inconsistent!"
#endif

#if ((BQ7971X_REGS_MAJOR_VERSION != BQ7971X_DIAGSTRUP_C_MAJOR_VERSION) || \
     (BQ7971X_REGS_MINOR_VERSION != BQ7971X_DIAGSTRUP_C_MINOR_VERSION) || \
	 (BQ7971X_REGS_PATCH_VERSION != BQ7971X_DIAGSTRUP_C_PATCH_VERSION))
#error "tibms: Config version numbers of bq7971x_diagstartup.c and bq7971x_regs.h are inconsistent!"
#endif

/*********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

#define DIAG_DX_BGX_DAC_MASK                            (0x4U)
#define DIAG_D1_OV_UV_CBDONE_DAC_MASK                   (0x2U)
#define DIAG_D2_OT_UT_OTCB_DAC_MASK                     (0x2U)

/*********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

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

#define BQ7971X_DIAGSTARTUP_START_SEC_CODE
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 * uint8 DiagStartupInit(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Diagnostic Startup Init
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pSvcCfg - Diagnostic Service
 *  \param[in]      pDiagReq - Current Diagnostic Request
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

STATIC FUNC(uint8, bq7971x_diag_CODE) DiagStartupInit(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uCmd[2u];
    uint8 uDevIdx;
    uint8 uRegStat;
    uint8 uRet;
    uint8 uLoop = pDiagMgr->sDevId;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_MPFDI_STARTUPINIT == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sInit = (uint8)DIAG_STAT_ERROR;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_STARTUP = DIAGNOERROR;
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx+1U), BQ7971X_CUST_CRC_HI_OFFSET,
                                              &pData[0], WRITE_2BYTE);
                    if((uint8)BMI_OK != uRet)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_STARTUP = DIAGERROR;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    /* Step6: Reset CUST CRC FAULT after updating the CUST CRC. */
                    uCmd[0] = (BQ7971X_FAULT_RST2_RST_OTP_CRC_MSK | BQ7971X_FAULT_RST2_RST_OTP_DATA_MSK);
                    uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) FALSE, BQ7971X_FAULT_RST2_OFFSET, uCmd, WRITE_1BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sInit = (uint8)DIAG_STAT_NOERROR;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE3:
        {
            if((uint8)BMI_SM_FDTI_VC_OPN_DET == pSvcCfg->uSubId)
            {
                /* Step 5: Save Vcell Value for first time. */
                uRet = Bq7971x_DecodeVcell(pDiagMgr->pBmcMgr, pSvcCfg,
                                           (uint16 *) pDiagMgr->pDiagCfg[BMI_SM_FDTI_VC_OPN_DET].pDecodeBuf1);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step6: Read the CUST CRC FAULT for updating the CUST CRC. */
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_CUST_CRC_RSLT_HI_OFFSET,
                                           &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_STARTUPINIT], READ_2BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE2:
        {
            if(BMI_DIAG_DEV_DIAG_STAT == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);

                    uRegStat = ((pData[2u] & BQ7971X_ADC_DATA_RDY_DRDY_GPADC_MSK) |
                                (pData[2u] & BQ7971X_ADC_DATA_RDY_DRDY_VCELLADC_MSK) |
                                (pData[1u] & BQ7971X_DIAG_STAT2_DRDY_DIG_MSK));

                    if(0u == uRegStat)
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    /* Step 5: Save Vcell Value for first time. */
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_CELL((BQ7971X_VCELL18_HI_OFFSET +
                                           ((BQ7971X_CELL_NUM_MAX - BQ7971X_CELL_NUM_ACT) * READ_2BYTE))),
                                           &pDiagMgr->pDiagCfg[BMI_SM_FDTI_VC_OPN_DET], (BQ7971X_CELL_NUM_ACT * READ_2BYTE));
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE3;
                        pDiagReq->uDiagStat =(uint8) BMI_DIAG_RUN;
                    }
                }
                else
                {
                    pDiagReq->uDiagRetry++;
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sInit = (uint8)DIAG_STAT_DRDY_ERROR;

                    /* Step4.3 Delay before the retry */
                    (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_INIT_RETRY_DELAY);

                    /* Step4.4: Retry to Make sure the diagnostic is running. */
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_STAT1_OFFSET,
                                           &pDiagMgr->pDiagCfg[BMI_DIAG_DEV_DIAG_STAT], READ_5BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        if(pDiagReq->uDiagRetry >= BQ7971X_DIAG_INIT_RETRY)
                        {
                            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sInit = (uint8)DIAG_STAT_ABORT;
                            pDiagReq->uDiagStat =(uint8) BMI_DIAG_ABORT; // Stopping Diag Init, and Notifying the Application with ABORT CALL
                        }
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE1:
        {
            if(BMI_DIAG_DEV_STAT == pSvcCfg->uSubId)
            {
                uRet =(uint8) BMI_OK;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    uRegStat = ((pData[0u] & BQ7971X_DEV_STAT1_GPADC_RUN_MSK) |
                                (pData[0u] & BQ7971X_DEV_STAT1_VCELLADC_RUN_MSK));
                    if(0u == uRegStat)
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    /* Step4: Enable the device ADC digital path diagnostic.. */
                    uCmd[0u] = BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_MSK;
                    uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_DIAG_ADC_CTRL3_OFFSET, uCmd, WRITE_1BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        /* Step4.1: Wait for the comparison to be done. */
                        (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_INIT_DONE_DELAY);

                        /* Step4.2: Check the Status of the Digital path diagnostics */
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_STAT1_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_DEV_DIAG_STAT], READ_5BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE2;
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
                else
                {
                    pDiagReq->uDiagRetry++;
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sInit = (uint8)DIAG_STAT_RUN_ERROR;

                    /* Step3.3 Delay before the retry */
                    (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_INIT_RETRY_DELAY);

                    /* Step3.4: Retry to Make sure the diagnostic is running. */
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DEV_STAT1_OFFSET,
                                           &pDiagMgr->pDiagCfg[BMI_DIAG_DEV_STAT], READ_2BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        if(pDiagReq->uDiagRetry >= BQ7971X_DIAG_INIT_RETRY)
                        {
                            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sInit = (uint8)DIAG_STAT_ABORT;
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_ABORT;   // Stopping Diag Init, and Notifying the Application with ABORT CALL
                        }
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_READY:
        {
            // Starting the Diagnostic Init
            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sInit = (uint8)DIAG_STAT_PENDING;

            /* Step1: Reset the FAULT_ADC_* registers. */
            uCmd[0u] = BQ7971X_FAULT_RST2_RST_ADC_MSK;
            uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_FAULT_RST2_OFFSET, uCmd, WRITE_1BYTE);
            if((uint8)BMI_OK == uRet)
            {
                /* Step2: Configure analog compare filter and set VCELL diagnostic threshold */
                uCmd[0u] = pDiagMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uDiag_Adc_Ctrl[0u];
                uCmd[1u] = pDiagMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uDiag_Adc_Ctrl[1u];
                uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_DIAG_ADC_CTRL1_OFFSET, uCmd, WRITE_2BYTE);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step3: Ensure the ADC runs in continuous mode. */
                    uCmd[0u] = pDiagMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uAdc_Ctrl[0u];
                    uCmd[1u] = pDiagMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uAdc_Ctrl[1u] | BQ7971X_ADC_CTRL2_ADC_GO_MSK;
                    uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_ADC_CTRL1_OFFSET, uCmd, WRITE_2BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        /* Step3.1: Wait for ADC conversion to complete */
                        /* TBD: Need to check if this delay is required.
                                 Software is not waiting for the DRDY, and DRDY checked in the last step */
                        (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_ADC_GO_DELAY);

                        /* Step3.2: Make sure the diagnostic is running. This is Retry Safe*/
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DEV_STAT1_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_DEV_STAT], READ_2BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE1;
                            pDiagReq->uDiagStat =(uint8) BMI_DIAG_RUN;
                        }
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

    if(pDiagReq->uDiagStat >= (uint8)BMI_DIAG_ERROR)
    {
        uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;
        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sInit = (uint8)DIAG_STAT_ERROR;
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_DIAG_STARTUP, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uint8 DiagBg1Bg2(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of NFAULT Detection
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval         BMI_OK
 *                  BMI_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_diag_CODE) DiagBg1Bg2(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                 Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint16 wVoltage;
    uint8 uRegStat;
    uint8 uDevIdx;
    uint8 uCmd;
    uint8 uRet;
    uint8 uLoop = pDiagMgr->sDevId;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_MPFDI_BGX_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sBG1 = (uint8)DIAG_STAT_NOERROR;
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sBG2 = (uint8)DIAG_STAT_NOERROR;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_BG1_DIAG = DIAGNOERROR;
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_BG2_DIAG = DIAGNOERROR;

                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    wVoltage = (((uint16) pData[0u] << 8u) | (pData[1u] << 0u));
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zBG1Voltage = wVoltage;
                    if((wVoltage > BQ7971X_MAXBGPLAUVALUE) &&  (wVoltage < BQ7971X_MINBGPLAUVALUE))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_BG1_DIAG = DIAGERROR;
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sBG1 = (uint8)DIAG_STAT_ERROR;

                    }

                    wVoltage = (((uint16) pData[2u] << 8u) | (pData[3u] << 0u));
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zBG2Voltage = wVoltage;
                    if((wVoltage > BQ7971X_MAXBGPLAUVALUE) &&  (wVoltage < BQ7971X_MINBGPLAUVALUE))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_BG2_DIAG = DIAGERROR;
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sBG2 = (uint8)DIAG_STAT_ERROR;
                    }
                }

                uCmd = (pDiagMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uAdc_Ctrl[3u] | BQ7971X_ADC_CTRL4_DIAG_MEAS_GO_MSK);
                uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_ADC_CTRL4_OFFSET, &uCmd, WRITE_1BYTE);
                if((uint8)BMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE1:
        {
            if(BMI_DIAG_ADC_DATA_READY == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    uRegStat = (pData[0] & BQ7971X_ADC_DATA_RDY_DRDY_DIAG_D1D2_MSK);
                    if(0u == uRegStat)
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_D1_HI_OFFSET,
                                           &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_BGX_DIAG], READ_4BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                    }
                }
                else
                {
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sBG1 = (uint8)DIAG_STAT_DRDY_ERROR;
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sBG2 = (uint8)DIAG_STAT_DRDY_ERROR;

                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RETRY_ERROR;
                    if(pDiagReq->uDiagRetry < BQ7971X_DIAG_MAX_RETRY)
                    {
                        pDiagReq->uDiagRetry++;
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_ADC_DATA_RDY_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_ADC_DATA_READY], READ_1BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_READY:
        {
            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sBG1 = (uint8)DIAG_STAT_PENDING;
            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sBG2 = (uint8)DIAG_STAT_PENDING;

            /* Step1: Set D1 converts BG1, D2 converts BG2 */
            uCmd = (DIAG_DX_BGX_DAC_MASK | BQ7971X_ADC_CTRL4_DIAG_MEAS_GO_MSK);
            uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_ADC_CTRL4_OFFSET, &uCmd, WRITE_1BYTE);
            if((uint8)BMI_OK == uRet)
            {
                 /* Step1.1: Wait for Diagnostic to complete */
                (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_DIAG_D1D2_DELAY);

                /* Step1.2: Read the BGx Diagnostic Status */
                uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_ADC_DATA_RDY_OFFSET,
                                       &pDiagMgr->pDiagCfg[BMI_DIAG_ADC_DATA_READY], READ_1BYTE);
                if((uint8)BMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE1;
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                }
            }
            break;
        }
        default:
        {
            break;
        }
    }

    if(pDiagReq->uDiagStat >= (uint8)BMI_DIAG_ERROR)
    {
        uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;

        /* Execution Error either COM Layer or State machine - Data is not useful */
        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sBG1 = (uint8)DIAG_STAT_EXEC_ERROR;
        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sBG2 = (uint8)DIAG_STAT_EXEC_ERROR;

        uCmd = (pDiagMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uAdc_Ctrl[3u] | BQ7971X_ADC_CTRL4_DIAG_MEAS_GO_MSK);
        (void)Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_ADC_CTRL4_OFFSET, &uCmd, WRITE_1BYTE);
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_DIAG_BGX, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uint8 RefCapDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of Refcap Voltage diagnostic
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval         BMI_OK
 *                  BMI_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_diag_CODE) RefCapDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                 Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint16 wVoltage;
    uint8 uDevIdx;
    uint8 uRet;
    uint8 uLoop = pDiagMgr->sDevId;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_MPFDI_REFCAP_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sRefCap = (uint8)DIAG_STAT_NOERROR;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_REFCAP_DIAG = DIAGNOERROR;

                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    wVoltage = (((uint16)pData[0u] << 8u) | (pData[1u] << 0u));
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zRefCapVoltage = wVoltage;
                    if((wVoltage > BQ7971X_MAXREFCAPPLAUVALUE) && (wVoltage < BQ7971X_MINREFCAPPLAUVALUE))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_REFCAP_DIAG = DIAGERROR;
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sRefCap = (uint8)DIAG_STAT_ERROR;
                    }
                }
                pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                uRet = (uint8)BMI_OK;
            }
            break;
        }
        case (uint8)DIAG_STEP_READY:
        {
            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sRefCap = (uint8)DIAG_STAT_PENDING;
            uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_REF_CAP_HI_OFFSET,
                                   &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_REFCAP_DIAG], READ_2BYTE);
            if((uint8)BMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
            }
            break;
        }
        default:
        {
            break;
        }
    }

    if(pDiagReq->uDiagStat >= (uint8)BMI_DIAG_ERROR)
    {
        uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;
        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sRefCap = (uint8)DIAG_STAT_EXEC_ERROR;
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_REFCAP_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uint8 PwrBistDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pDiagCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of NFAULT Detection
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pDiagCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval         BMI_OK
 *                  BMI_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_diag_CODE) PwrBistDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                  Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx;
    uint8 uCmd;
    uint8 uRet;
    uint8 uLoop = pDiagMgr->sDevId;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_MPFDI_PWR_BIST == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sPwrBist = (uint8)DIAG_STAT_NOERROR;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_PWR_BIST = DIAGNOERROR;
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                     if(0u != (pData[0] & BQ7971X_FAULT_PWR1_PWRBIST_FAIL_MSK))
                     {
                        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_PWR_BIST = DIAGERROR;
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sPwrBist = (uint8)DIAG_STAT_ERROR;
                     }
                }
                pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                uRet = (uint8)BMI_OK;
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE1:
        {
            if(BMI_DIAG_DIAG_STAT == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == (pData[0] & BQ7971X_DIAG_STAT1_DRDY_PWRBIST_MSK))
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    /* Step5: Check the power supply diagnosis result. */
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_FAULT_PWR1_OFFSET,
                                           &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_PWR_BIST], READ_2BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        /* Step6: Write the original Dev Conf value. */
                        uCmd = pDiagMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uDev_Conf[0];
                        uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_DEV_CONF1_OFFSET, &uCmd, WRITE_1BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
                else
                {
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sPwrBist = (uint8)DIAG_STAT_DRDY_ERROR;
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RETRY_ERROR;
                    if(pDiagReq->uDiagRetry < BQ7971X_DIAG_MAX_RETRY)
                    {
                        pDiagReq->uDiagRetry++;
                        /* Step3.3: Re-Check whether the pwoer supply BIST is complete. */
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_STAT1_OFFSET,
                                           &pDiagMgr->pDiagCfg[BMI_DIAG_DIAG_STAT], READ_2BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_READY:
        {
            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sPwrBist = (uint8)DIAG_STAT_PENDING;

            /* Step1: Disable NFAULT Pin. */
            uCmd = (uint8)(pDiagMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uDev_Conf[0] & (~BQ7971X_DEV_CONF1_NFAULT_EN_MSK));
            uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_DEV_CONF1_OFFSET, &uCmd, WRITE_1BYTE);
            if((uint8)BMI_OK == uRet)
            {
                /* Step2: Reset the FAULT_PWR* registers. */
                uCmd = BQ7971X_FAULT_RST1_RST_PWR_MSK;
                uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_FAULT_RST1_OFFSET, &uCmd, WRITE_1BYTE);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step3: Start power supply BIST diagnostic. */
                    uCmd = BQ7971X_DIAG_MISC_CTRL2_PWRBIST_GO_MSK;
                    uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_DIAG_MISC_CTRL2_OFFSET,
                                            &uCmd, WRITE_1BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                         /* Step3.1: Wait power supply BIST diagnostic. */
                        (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_DRDY_BIST_PWR_DELAY);

                        /* Step3.2: Check whether the pwoer supply BIST is complete. */
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_STAT1_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_DIAG_STAT], READ_2BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE1;
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
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

    if(pDiagReq->uDiagStat >= (uint8)BMI_DIAG_ERROR)
    {
        uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;
        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sPwrBist = (uint8)DIAG_STAT_EXEC_ERROR;

        uCmd = pDiagMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uDev_Conf[0];
        (void)Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_DEV_CONF1_OFFSET, &uCmd, WRITE_1BYTE);
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_PWRBIST_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uint8 AcompFltinjResult(const Bq7971x_DiagType *, const ServiceCfgType *)
 *********************************************************************************************************************/
/*! \brief          This function is used to analog comparison fault injection diagnostic.
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pDiagCfg - Diagnostic context
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval         BMI_OK
 *                  BMI_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_diag_CODE) AcompFltinjResult(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                        uint8 uCheckFault)
{
    uint8 *pData;
    uint32 qFaultStat;
    uint32 qFault;
    uint32 qNDefCfg;
    uint8 uDevIdx;
    uint8 uRet = (uint8)BMI_OK;
    uint8 uLoop = pDiagMgr->sDevId;

    for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
    {
        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_ACOMP_FLTINJ_GPIO = DIAGNOERROR;
        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_ACOMP_FLTINJ_VCELL = DIAGNOERROR;

        pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
        // VCELL Fault Inject Status Update
        qNDefCfg = pDiagMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNCellsCfg;
        qFaultStat = (uCheckFault > 0) ? qNDefCfg:0u;
        qFault = (uint32) ((pData[2] << 16u) | (pData[3] << 8u) | (pData[4] << 0u));

        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zVcellACompFltInj = (qFault & qNDefCfg);
        if(qFaultStat != pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zVcellACompFltInj)
        {
            pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_ACOMP_FLTINJ_VCELL = DIAGERROR;
            uRet = (uint8)BMI_NOT_OK;
        }
        // GPIO Fault Injection Status Update
        qNDefCfg = pDiagMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNGpioCfg;
        qFaultStat = (uCheckFault > 0) ? qNDefCfg:0u;
        qFault = (uint32) ((pData[0] << 8u) | (pData[1] << 0u));
        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zGpioAcompFltInj = (qFault & qNDefCfg);
        if(qFaultStat != pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zGpioAcompFltInj)
        {
            pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_ACOMP_FLTINJ_GPIO = DIAGERROR;
            uRet = (uint8)BMI_NOT_OK;
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * uint8 AcompFltinjDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          This function is used to analog comparison fault injection diagnostic.
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pDiagCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval         BMI_OK
 *                  BMI_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_diag_CODE) AcompFltinjDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uRegStat;
    uint8 uDevIdx;
    uint8 uCmd;
    uint8 uRet;
    uint8 uLoop = pDiagMgr->sDevId;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_MPFDI_ACOMP_FLTINJ == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sVcellACompStat = (uint8)DIAG_STAT_ERROR;
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sGpioACompStat = (uint8)DIAG_STAT_ERROR;

                /* Step9: Update the Diag Status */
                uRet = AcompFltinjResult(pDiagMgr, pSvcCfg, (uint8) FALSE);
                if((uint8)BMI_OK == uRet)
                {
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sVcellACompStat = (uint8)DIAG_STAT_NOERROR;
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sGpioACompStat = (uint8)DIAG_STAT_NOERROR;
                }
                pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE3:
        {
            if((uint8)BMI_SM_MPFDI_ACOMP_FLTINJ == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sVcellAFltInjStat = (uint8)DIAG_STAT_ERROR;
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sGpioAFltInjStat = (uint8)DIAG_STAT_ERROR;

                /* Step5: Update the Diag Status */
                uRet = AcompFltinjResult(pDiagMgr, pSvcCfg, (uint8) TRUE);
                if((uint8)BMI_OK == uRet)
                {
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sGpioAFltInjStat = (uint8)DIAG_STAT_NOERROR;
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sVcellAFltInjStat = (uint8)DIAG_STAT_NOERROR;
                }

                /* Step6: Stop ANA_BOTH and digital fault injection by DIAG_ADC_CTRL3[ACOMP_MPFLT_INJ] = 0b0. */
                uCmd = (BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_MSK | BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_GO_MSK);
                uRet = Bq7971x_SetDiagAdcCtrl3(pDiagMgr->pBmcMgr, uCmd);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step7: It takes 133* tslot to finish the diagnose of ACOMP. */
                    (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_ACOMPINJ_DONE_DELAY);

                    /* Step8: Reset the FAULT_ADC_* registers. */
                    uCmd = BQ7971X_FAULT_RST2_RST_ADC_MSK;
                    uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_FAULT_RST2_OFFSET, &uCmd, WRITE_1BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_FAULT_ADC_GPIO1_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_ACOMP_FLTINJ],
                                               (BQ7971X_FAULT_ADC_VCELL_NUM+BQ7971X_FAULT_ADC_GPIO_NUM));
                        if((uint8)BMI_OK == uRet)
                        {
                            /* Step9: Remove CBCtrl Pause request from Diag */
                            uRet = Bq7971x_UnpauseBalCtrl(pDiagMgr->pBmcMgr, BQ7971X_BAL_CTRL2_OFFSET,
                                                          BQ7971X_CB_DIAG_UNPAUSE, BQ7971X_CB_APP_CTRL);
                            if((uint8)BMI_OK == uRet)
                            {
                                pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
                                pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                            }
                        }
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE2:
        {
            if(BMI_DIAG_DIAG_STAT == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(pData != NULL)
                    {
                        uRegStat = (pData[1u] & (BQ7971X_DIAG_STAT2_DRDY_ANA_VCELL_MSK |
                                                 BQ7971X_DIAG_STAT2_DRDY_ANA_GPIO_MSK));
                        if(0u == uRegStat)
                        {
                            uRet = (uint8)BMI_NOT_OK;
                        }
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_FAULT_ADC_GPIO1_OFFSET,
                                           &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_ACOMP_FLTINJ],
                                           (BQ7971X_FAULT_ADC_VCELL_NUM+BQ7971X_FAULT_ADC_GPIO_NUM));
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE3;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                    }
                }
                else
                {
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sVcellAFltInjStat = (uint8)DIAG_STAT_DRDY_ERROR;
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sGpioAFltInjStat = (uint8)DIAG_STAT_DRDY_ERROR;
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RETRY_ERROR;
                    if(pDiagReq->uDiagRetry < BQ7971X_DIAG_MAX_RETRY)
                    {
                        pDiagReq->uDiagRetry++;
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_STAT1_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_DIAG_STAT], READ_2BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE1:
        {
            if(BMI_DIAG_DEV_STAT == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    uRegStat = (pData[0u] & (BQ7971X_DEV_STAT1_DIAG_ANA_RUN_MSK));
                    if(0u == uRegStat)
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    /* Step4: Set DIAG_ADC_CTRL3[ACOMP_MPFLT_INJ] = 0b1. */
                    uCmd = (BQ7971X_DIAG_ADC_CTRL3_ACOMP_MPFLT_INJ_MSK |
                            BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_MSK | BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_MODE_BOTH);
                    uRet = Bq7971x_SetDiagAdcCtrl3(pDiagMgr->pBmcMgr, uCmd);
                    if((uint8)BMI_OK == uRet)
                    {
                        /* Step4.1: It takes 133* tslot to finish the diagnose of ACOMP. */
                        (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_ACOMPINJ_DONE_DELAY);

                        /* Step5: Checkif Diagnostics Completed */
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_STAT1_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_DIAG_STAT], READ_2BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE2;
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
                else
                {
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sVcellAFltInjStat = (uint8)DIAG_STAT_RUN_ERROR;
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sGpioAFltInjStat = (uint8)DIAG_STAT_RUN_ERROR;

                    /* Step3.1: Retry incase of not running */
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RETRY_ERROR;
                    if(pDiagReq->uDiagRetry < BQ7971X_DIAG_MAX_RETRY)
                    {
                        pDiagReq->uDiagRetry++;
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DEV_STAT1_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_DEV_STAT], READ_2BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_READY:
        {
            /* Step0: Pause the BalCtrl If running. This is required inorder to make sure that no faults due to CB */
            uRet = Bq7971x_PauseBalCtrl(pDiagMgr->pBmcMgr, BQ7971X_BAL_CTRL2_OFFSET, BQ7971X_CB_DIAG_PAUSE,
                                        BQ7971X_CB_DIAG_CTRL);
            if((uint8)BMI_OK == uRet)
            {
                /* Step1: Reset the FAULT_ADC_* registers. */
                uCmd = BQ7971X_FAULT_RST2_RST_ADC_MSK;
                uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_FAULT_RST2_OFFSET, &uCmd, WRITE_1BYTE);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step2: Select Both cell voltage measurement and GPIO measurement analog path diagnostic
                              and Enable the device ADC digital path diagnostic. */
                    uCmd = (BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_MSK | BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_MODE_BOTH |
                            BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_GO_MSK);
                    uRet = Bq7971x_SetDiagAdcCtrl3(pDiagMgr->pBmcMgr, uCmd);
                    if((uint8)BMI_OK == uRet)
                    {
                        /* Step3: Checkif Diagnostics is running */
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DEV_STAT1_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_DEV_STAT], READ_2BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE1;
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
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

    if(pDiagReq->uDiagStat >= (uint8)BMI_DIAG_ERROR)
    {
        uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;

        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sVcellAFltInjStat = (uint8)DIAG_STAT_EXEC_ERROR;
        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sGpioAFltInjStat = (uint8)DIAG_STAT_EXEC_ERROR;

        /* Step ERROR: Stop digital fault injection by DIAG_ADC_CTRL3[ACOMP_MPFLT_INJ] = 0b0. Unpause BalCtrl*/
        uCmd = (BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_MSK | BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_GO_MSK);
        (void)Bq7971x_SetDiagAdcCtrl3(pDiagMgr->pBmcMgr, uCmd);
        (void)Bq7971x_UnpauseBalCtrl(pDiagMgr->pBmcMgr, BQ7971X_BAL_CTRL2_OFFSET, BQ7971X_CB_DIAG_UNPAUSE,
                               BQ7971X_CB_APP_CTRL);
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_ACOMP_FLTINJ, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uiDcompFltinjDiagnt8 DcompFltinjDiag(const Bq7971x_DiagType *, const ServiceCfgType *, Bq7971x_DiagReqType *)
 *********************************************************************************************************************/
/*! \brief          This function is used to digital comparison fault injection diagnostic.
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pDiagCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval         BMI_OK
 *                  BMI_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_diag_CODE) DcompFltinjDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint32 qCheckFault;
    uint8 uDevIdx;
    uint8 uCmd;
    uint8 uRet;
    uint8 uLoop = pDiagMgr->sDevId;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_MPFDI_DCOMP_FLTINJ == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sDCompStat = (uint8)DIAG_STAT_NOERROR;
                for(uDevIdx = pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_DCOMP_FLTINJ = DIAGNOERROR;

                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    qCheckFault = (((uint32) pData[0u] << 16u) | ((uint16) pData[1u] << 8u) | (pData[2u] << 0u));
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zDcompFltInj = qCheckFault;
                    if(qCheckFault != 0u)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_DCOMP_FLTINJ = DIAGERROR;
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sDCompFltInjStat =(uint8) DIAG_STAT_ERROR;
                    }
                }

                pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                uRet = (uint8)BMI_OK;
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE3:
        {
            if((uint8)BMI_SM_MPFDI_DCOMP_FLTINJ == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sDCompFltInjStat = (uint8)DIAG_STAT_NOERROR;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_DCOMP_FLTINJ = DIAGNOERROR;

                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    qCheckFault =  (((uint32)pData[0u] << 16u) | ((uint16) pData[1u] << 8u) | (pData[2u] << 0u));
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zDcompFltInj = qCheckFault;
                    if(0u == qCheckFault)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_DCOMP_FLTINJ = DIAGERROR;
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sDCompFltInjStat = (uint8)DIAG_STAT_ERROR;
                    }
                }

                /* Step5: Stop digital fault injection by Set DIAG_ADC_CTRL3[DCOMP_MPFLT_INJ] = 0b0. */
                uCmd = BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_MSK;
                uRet = Bq7971x_SetDiagAdcCtrl3(pDiagMgr->pBmcMgr, uCmd);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step6: Delay to complete the Diagnostics */
                    (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_DCOMP_DONE_DELAY);

                    /* Step8: Reset the FAULT_ADC_* registers. */
                    uCmd = BQ7971X_FAULT_RST2_RST_ADC_MSK;
                    uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_FAULT_RST2_OFFSET, &uCmd, WRITE_1BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        /* Step9: Check the faults are cleared. */
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_FAULT_ADC_DIG1_OFFSET,
                                            &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_DCOMP_FLTINJ], READ_3BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE2:
        {
            if(BMI_DIAG_DIAG_STAT == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == (pData[1u] & BQ7971X_DIAG_STAT2_DRDY_DIG_MSK))
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    /* Step3: Read DIG Fault Value */
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_FAULT_ADC_DIG1_OFFSET,
                                           &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_DCOMP_FLTINJ], READ_3BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE3;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                    }
                }
                else
                {
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sDCompFltInjStat = (uint8)DIAG_STAT_DRDY_ERROR;

                    /* Step2.3: Retry delay DIG Diagnostics */
                    (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_DCOMPINJ_RETRY_DELAY);
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RETRY_ERROR;
                    if(pDiagReq->uDiagRetry < BQ7971X_DIAG_MAX_RETRY)
                    {
                        pDiagReq->uDiagRetry++;
                        /* Step2.4: Read the DIG Diagnostics status */
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_STAT1_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_DIAG_STAT], READ_2BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE1:
        {
            if(BMI_DIAG_DEV_STAT == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == (pData[1u] & BQ7971X_DEV_STAT2_DIAG_DIG_RUN_MSK))
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    /* Step2: Set DIAG_ADC_CTRL3[ACOMP_MPFLT_INJ] = 0b1. */
                    uCmd = (BQ7971X_DIAG_ADC_CTRL3_DCOMP_MPFLT_INJ_MSK | BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_MSK);
                    uRet = Bq7971x_SetDiagAdcCtrl3(pDiagMgr->pBmcMgr, uCmd);
                    if((uint8)BMI_OK == uRet)
                    {
                        /* Step2.1: Delay to complete the Diagnostics */
                        (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_DCOMP_DONE_DELAY);

                        /* Step2.2: Checkif Diagnostics is running and drdy */
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_STAT1_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_DIAG_STAT], READ_2BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE2;
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
                else
                {
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sDCompFltInjStat = (uint8)DIAG_STAT_RUN_ERROR;

                    /* Step1.2: Retry delay DIG Diagnostics */
                    (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_DCOMPINJ_RETRY_DELAY);
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RETRY_ERROR;
                    if(pDiagReq->uDiagRetry < BQ7971X_DIAG_MAX_RETRY)
                    {
                        pDiagReq->uDiagRetry++;
                        /* Step1.3: Read the DIG Diagnostics status */
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DEV_STAT1_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_DEV_STAT], READ_2BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_READY:
        {
            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sDCompFltInjStat = (uint8)DIAG_STAT_PENDING;

            /* Step1: Reset the FAULT_ADC_* registers. */
            uCmd = BQ7971X_FAULT_RST2_RST_ADC_MSK;
            uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_FAULT_RST2_OFFSET, &uCmd, WRITE_1BYTE);
            if((uint8)BMI_OK == uRet)
            {
                /* Step1.1: Checkif DIG Diagnostics is running*/
                uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DEV_STAT1_OFFSET,
                                       &pDiagMgr->pDiagCfg[BMI_DIAG_DEV_STAT], READ_2BYTE);
                if((uint8)BMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE1;
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                }
            }
            break;
        }
        default:
        {
            break;
        }
    }

    if(pDiagReq->uDiagStat >= (uint8)BMI_DIAG_ERROR)
    {
        uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;
        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sDCompFltInjStat = (uint8)DIAG_STAT_EXEC_ERROR;

        /* Step ERROR: reset Fault Injection incase of Error */
        uCmd = BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_MSK;
        (void)Bq7971x_SetDiagAdcCtrl3(pDiagMgr->pBmcMgr, uCmd);
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_DCOMP_FLTINJ, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uint8 CbfetDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Cell balance CBFET diagnostic function can be used to verify the operation of the
 *                  cell balancing function.
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pSvcCfg - Diagnostic Request Service
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_diag_CODE) CbfetDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint32 qResult;
    uint8 uRet;
    uint8 uCmd;
    uint8 uDevIdx;
    uint8 uLoop = pDiagMgr->sDevId;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_MPFDI_CBFET_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sCbFet = (uint8)DIAG_STAT_NOERROR;

                /* Step4: Check FAULT_CB_FETOW 1-3 [CBFAIL*]* registers for the diagnostic result. */
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    qResult = ((uint32) ((pData[0u] << 16u) | (pData[1u] << 8u) | (pData[2u] << 0u)) &
                               (pDiagMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNCellsCfg));

                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zCBFETDiagResult = qResult;
                    if(pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zCBFETDiagResult != 0u)
                    {
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sCbFet = DIAG_STAT_ERROR;
                    }

                    /* TBD: When FAULT_CB_FETOW* bit is set and VC/CB comparison passes, then it indicates a CB FET open failure. */
                    // pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_CB_FAULT =
                                                 // (qResult & pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_VCELL_ACOMP);
                    // pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_CBFET_OPN =
                                                // (qResult & ~pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_VCELL_ACOMP);
                }
                pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                uRet = (uint8)BMI_OK;
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE1:
        {
            if(BMI_DIAG_DIAG_STAT == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == (pData[1] & BQ7971X_DIAG_STAT2_DRDY_CBFETOW_MSK))
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    /* Step4: Check FAULT_CB_FETOW 1-3 [CBFAIL*]* registers for the diagnostic result. */
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_FAULT_CB_FETOW1_OFFSET,
                                           &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_CBFET_DIAG], READ_3BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                    }
                }
                else
                {
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sCbFet = (uint8)DIAG_STAT_DRDY_ERROR;
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RETRY_ERROR;
                    if(pDiagReq->uDiagRetry < BQ7971X_DIAG_MAX_RETRY)
                    {
                        pDiagReq->uDiagRetry++;
                        // 1ms delay is required, which is by deault get from scheduling
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_STAT1_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_DIAG_STAT], READ_2BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_READY:
        {
            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sCbFet = (uint8)DIAG_STAT_PENDING;

            /* Step1: Start CB FET/CB OW diagnostic. */
            uCmd = BQ7971X_DIAG_MISC_CTRL2_DIAG_CBFETOW_GO_MSK;
            uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_DIAG_MISC_CTRL2_OFFSET, &uCmd, WRITE_1BYTE);
            if((uint8)BMI_OK == uRet)
            {
                /* Step2: Wait for this diagnostic is done. */
                (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_DIAG_CBFET_DELAY);

                /* Step3: DRDY status update. */
                uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_STAT1_OFFSET,
                                       &pDiagMgr->pDiagCfg[BMI_DIAG_DIAG_STAT], READ_2BYTE);
                if((uint8)BMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE1;
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                }
            }
            break;
        }
        default:
        {
            break;
        }
    }

    if(pDiagReq->uDiagStat >= (uint8)BMI_DIAG_ERROR)
    {
        uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;
        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sCbFet = (uint8)DIAG_STAT_EXEC_ERROR;
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_DIAG_CBFET, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uint8 OvUvDacDiag(const Bq7971x_DiagType *, const ServiceCfgType *, Bq7971x_DiagReqType *, uint8)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of OV UV Diagnostic
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 *  \param[in]      uUvOvReqCmd - Command
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval         BMI_OK
 *                  BMI_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_diag_CODE) OvUvDacDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                  Bq7971x_DiagReqType *pDiagReq, uint8 uUvOvReqCmd)
{
    uint8 *pData;
    uint16 wFaultVolt;
    uint8 uCmd[2];
    uint8 uDevIdx;
    uint8 uRet;
    uint8 uLoop = pDiagMgr->sDevId;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if(BMI_DIAG_DIAG_D1D2 == pSvcCfg->uSubId)
            {
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    wFaultVolt = (((uint16) pData[0u] << 8u) | (pData[1u] << 0u));
                    if(BQ7971X_OVUV_CTRL1_OV_THR == uUvOvReqCmd)
                    {
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sOvStat = (uint8)DIAG_STAT_NOERROR;
                        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_OV_DAC = DIAGNOERROR;
                        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zOvVoltage = wFaultVolt;
                        if((wFaultVolt > (pDiagMgr->wOvThreshVal + BQ7971X_SMOVDAC_ERR)) ||
                           (wFaultVolt < (pDiagMgr->wOvThreshVal - BQ7971X_SMOVDAC_ERR)))
                        {
                            pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_OV_DAC = DIAGERROR;
                            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sOvStat = (uint8)DIAG_STAT_ERROR;
                        }
                    }
                    else
                    {
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sUvStat = (uint8)DIAG_STAT_NOERROR;
                        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_UV_DAC = DIAGNOERROR;
                        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zUvVoltage = wFaultVolt;
                        if((wFaultVolt > (pDiagMgr->wUvThreshVal + BQ7971X_SMUVDAC_ERR)) ||
                           (wFaultVolt < (pDiagMgr->wUvThreshVal - BQ7971X_SMUVDAC_ERR)))
                        {
                            pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_UV_DAC = DIAGERROR;
                            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sUvStat = (uint8)DIAG_STAT_ERROR;
                        }
                    }
                }

                pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                uRet = (uint8)BMI_OK;
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE1:
        {
            if(BMI_DIAG_ADC_DATA_READY == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == (pData[0] & BQ7971X_ADC_DATA_RDY_DRDY_DIAG_D1D2_MSK))
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_D1_HI_OFFSET,
                                           &pDiagMgr->pDiagCfg[BMI_DIAG_DIAG_D1D2], READ_2BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                    }
                }
                else
                {
                    if(BQ7971X_OVUV_CTRL1_OV_THR == uUvOvReqCmd)
                    {
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sOvStat = (uint8)DIAG_STAT_DRDY_ERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sUvStat = (uint8)DIAG_STAT_DRDY_ERROR;
                    }

                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RETRY_ERROR;
                    if(pDiagReq->uDiagRetry < BQ7971X_DIAG_MAX_RETRY)
                    {
                        pDiagReq->uDiagRetry++;
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_ADC_DATA_RDY_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_ADC_DATA_READY], READ_1BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_READY:
        {
            if(BQ7971X_OVUV_CTRL1_OV_THR == uUvOvReqCmd)
            {
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sOvStat = (uint8)DIAG_STAT_PENDING;
            }
            else
            {
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sUvStat = (uint8)DIAG_STAT_PENDING;
            }

            /* Step1: [OVUV_THR_LOCK] = 0b00 to select OV threshold. */
            /* Step2: set [OVUV_THR_LOCK], enable OVUV. */
            uCmd[0] = uUvOvReqCmd;
            uCmd[1] = BQ7971X_OVUV_CTRL2_OVUV_THR_LOCK | BQ7971X_OVUV_CTRL2_OVUV_GO_MSK;
            uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_OVUV_CTRL1_OFFSET, uCmd, WRITE_2BYTE);
            if((uint8)BMI_OK == uRet)
            {
                /* Step3: [DIAG_D1D2_SEL] =0b001, then [DIAG_MEAS_GO] =0b1. */
                uCmd[0] = (DIAG_D1_OV_UV_CBDONE_DAC_MASK | BQ7971X_ADC_CTRL4_DIAG_MEAS_GO_MSK);
                uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_ADC_CTRL4_OFFSET, uCmd, WRITE_1BYTE);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step4: Delay for completing the Diag*/
                    (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_DIAG_D1D2_DELAY);

                    /* Step5: Reading DRDY Status */
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_ADC_DATA_RDY_OFFSET,
                                          &pDiagMgr->pDiagCfg[BMI_DIAG_ADC_DATA_READY], READ_1BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE1;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
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

    if(pDiagReq->uDiagStat >= (uint8)BMI_DIAG_ERROR)
    {
        uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;
        if(BQ7971X_OVUV_CTRL1_OV_THR == uUvOvReqCmd)
        {
            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sOvStat = (uint8)DIAG_STAT_EXEC_ERROR;
        }
        else
        {
            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sUvStat = (uint8)DIAG_STAT_EXEC_ERROR;
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_DIAG_OVUV_DAC, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_diag_CODE) OtUtDacDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of OT UT Diagnostic
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 *  \param[in]      uOtUtCmd - Diag Command
 *  \param[in]      uOtUtUpdate - Update for Ot/UT
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval         BMI_OK
 *                  BMI_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_diag_CODE) OtUtDacDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                  Bq7971x_DiagReqType *pDiagReq, uint8 uOtUtReqCmd)
{
    uint8 *pData;
    uint16 wFaultVolt;
    uint8 uCmd[2];
    uint8 uRet;
    uint8 uDevIdx;
    uint8 uLoop = pDiagMgr->sDevId;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if(BMI_DIAG_DIAG_D1D2 == pSvcCfg->uSubId)
            {
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    wFaultVolt = (((uint16) pData[0u] << 8u) | (pData[1u] << 0u));
                    if(BQ7971X_OTUT_CTRL1_OT_THR == uOtUtReqCmd)
                    {
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sOtStat = (uint8)DIAG_STAT_NOERROR;
                        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_OT_DAC = DIAGNOERROR;
                        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zOtVoltage = wFaultVolt;
                        if((wFaultVolt > (pDiagMgr->wOtThreshVal + BQ7971X_SMOTDAC_ERR)) ||
                           (wFaultVolt < (pDiagMgr->wOtThreshVal - BQ7971X_SMOTDAC_ERR)))
                        {
                            pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_OT_DAC = DIAGERROR;
                            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sOtStat = (uint8)DIAG_STAT_ERROR;
                        }
                    }
                    else
                    {
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sUtStat = (uint8)DIAG_STAT_NOERROR;
                        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_UT_DAC = DIAGNOERROR;
                        pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zUtVoltage = wFaultVolt;
                        if((wFaultVolt > (pDiagMgr->wUtThreshVal + BQ7971X_SMUTDAC_ERR)) ||
                           (wFaultVolt < (pDiagMgr->wUtThreshVal - BQ7971X_SMUTDAC_ERR)))
                        {
                            pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_UT_DAC = DIAGERROR;
                            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sUtStat = (uint8)DIAG_STAT_ERROR;
                        }
                    }
                }
                pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                uRet = (uint8)BMI_OK;
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE1:
        {
            if(BMI_DIAG_ADC_DATA_READY == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == (pData[0] & BQ7971X_ADC_DATA_RDY_DRDY_DIAG_D1D2_MSK))
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_D2_HI_OFFSET,
                                           &pDiagMgr->pDiagCfg[BMI_DIAG_DIAG_D1D2], READ_2BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                    }
                }
                else
                {
                    if(BQ7971X_OTUT_CTRL1_OT_THR == uOtUtReqCmd)
                    {
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sOtStat = (uint8)DIAG_STAT_DRDY_ERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sUtStat = (uint8)DIAG_STAT_DRDY_ERROR;
                    }
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RETRY_ERROR;
                    if(pDiagReq->uDiagRetry < BQ7971X_DIAG_MAX_RETRY)
                    {
                        pDiagReq->uDiagRetry++;
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_ADC_DATA_RDY_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_ADC_DATA_READY], READ_1BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_READY:
        {
            if(BQ7971X_OTUT_CTRL1_OT_THR == uOtUtReqCmd)
            {
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sOtStat = (uint8)DIAG_STAT_PENDING;
            }
            else
            {
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sUtStat = (uint8)DIAG_STAT_PENDING;
            }

            /* Step1: [OTUT_THR_LOCK] = 0b00 to select OT threshold. */
            /* Step2: set [OTUT_THR_LOCK], enable OTUT. */
            uCmd[0] = uOtUtReqCmd;
            uCmd[1] = BQ7971X_OTUT_CTRL2_OTUT_THR_LOCK | BQ7971X_OTUT_CTRL2_OTUT_GO_MSK;
            uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_OTUT_CTRL1_OFFSET, uCmd, WRITE_2BYTE);
            if((uint8)BMI_OK == uRet)
            {
                /* Step3: [DIAG_D1D2_SEL] =0b001, then [DIAG_MEAS_GO] =0b1. */
                uCmd[0] = (DIAG_D2_OT_UT_OTCB_DAC_MASK | BQ7971X_ADC_CTRL4_DIAG_MEAS_GO_MSK);
                uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_ADC_CTRL4_OFFSET, uCmd, WRITE_1BYTE);
                if((uint8)BMI_OK == uRet)
                {
                    (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_DIAG_D1D2_DELAY);
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_ADC_DATA_RDY_OFFSET,
                                          &pDiagMgr->pDiagCfg[BMI_DIAG_ADC_DATA_READY], READ_1BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE1;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
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

    if(pDiagReq->uDiagStat >= (uint8)BMI_DIAG_ERROR)
    {
        uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;
        if(BQ7971X_OTUT_CTRL1_OT_THR == uOtUtReqCmd)
        {
            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sOtStat = (uint8)DIAG_STAT_EXEC_ERROR;
        }
        else
        {
            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sUtStat = (uint8)DIAG_STAT_EXEC_ERROR;
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_OTUT_DAC_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uint8 CbOwFltinjDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          This function is used to analog comparison fault injection diagnostic.
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pDiagCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval         BMI_OK
 *                  BMI_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_diag_CODE) CbOwFltinjDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint32 qCheckFault;
    uint8 uDevIdx;
    uint8 uCmd;
    uint8 uRet;
    uint8 uLoop = pDiagMgr->sDevId;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_CBOW_FLTINJ == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sCbOwFltInjStat = (uint8)DIAG_STAT_NOERROR;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    if(0u == pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_CBOW_FLTINJ != 0u)
                    {
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sCbOwFltInjStat = DIAG_STAT_ERROR;
                    }

                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    qCheckFault =  (((uint32)pData[0] << 16u) | ((uint16)pData[1] << 8u) | (pData[2] << 0u));
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_CBOW_FLTINJ = (qCheckFault  &
                                                    pDiagMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNCellsCfg);

                    if(((uint8)DIAG_STAT_NOERROR ==  pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sCbOwFltInjStat) &&
                       (pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_CBOW_FLTINJ))
                    {
                        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sCbOwFltInjStat = (uint8)DIAG_STAT_ERROR;
                    }

                    /* Step7: Reset the FAULT_CB_FETOW* registers. */
                    uCmd = BQ7971X_FAULT_RST1_RST_CB_MSK;
                    uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_FAULT_RST1_OFFSET, &uCmd, WRITE_1BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE2:
        {
            if((uint8)BMI_SM_CBOW_FLTINJ == pSvcCfg->uSubId)
            {
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    qCheckFault =  (((uint32) pData[0] << 16u) | ((uint16) pData[1] << 8u) | (pData[2] << 0u));
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_CBOW_FLTINJ = (qCheckFault  &
                                                    pDiagMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNCellsCfg);
                }

                /* Step5: Clear the FAULT_CB_FETOW* registers. */
                uCmd = BQ7971X_DIAG_MISC_CTRL2_DIAG_CBFETOW_GO_MSK;
                uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_DIAG_MISC_CTRL2_OFFSET, &uCmd, WRITE_1BYTE);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step6: Wait for this diagnostic is done. */
                    (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_DIAG_CBFET_DELAY);

                    /* Step3: Check FAULT_CB_FETOW 1-3 [CBFAIL*]* registers for the diagnostic result. */
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_FAULT_CB_FETOW1_OFFSET,
                                           &pDiagMgr->pDiagCfg[BMI_SM_CBOW_FLTINJ], READ_3BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        uRet = Bq7971x_UnpauseBalCtrl(pDiagMgr->pBmcMgr, BQ7971X_BAL_CTRL2_OFFSET,
                                                      BQ7971X_CB_DIAG_UNPAUSE, BQ7971X_CB_APP_CTRL);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE1:
        {
            if(BMI_DIAG_DIAG_STAT == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == (pData[1u] & BQ7971X_DIAG_STAT2_DRDY_CBFETOW_MSK))
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    /* Step3: Check FAULT_CB_FETOW 1-3 [CBFAIL*]* registers for the diagnostic result. */
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_FAULT_CB_FETOW1_OFFSET,
                                           &pDiagMgr->pDiagCfg[BMI_SM_CBOW_FLTINJ], READ_3BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE2;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                    }
                }
                else
                {
                    pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sCbOwFltInjStat = (uint8)DIAG_STAT_DRDY_ERROR;
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RETRY_ERROR;
                    if(pDiagReq->uDiagRetry < BQ7971X_DIAG_MAX_RETRY)
                    {
                        pDiagReq->uDiagRetry++;
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_STAT1_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_DIAG_STAT], READ_2BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_READY:
        {
            pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sCbOwFltInjStat = (uint8)DIAG_STAT_ERROR;

            /* Step0: Pause the BalCtrl If running. This is required inorder to make sure that no faults due to CB */
            uRet = Bq7971x_PauseBalCtrl(pDiagMgr->pBmcMgr, BQ7971X_BAL_CTRL2_OFFSET, BQ7971X_CB_DIAG_PAUSE,
                                        BQ7971X_CB_DIAG_CTRL);
            if((uint8)BMI_OK == uRet)
            {
                /* Step1: Inject fault condition to the diagnostic path and start CB FET/CB OW diagnostic. */
                uCmd = BQ7971X_DIAG_MISC_CTRL2_CBFETOW_MPFLT_INJ_MSK | BQ7971X_DIAG_MISC_CTRL2_DIAG_CBFETOW_GO_MSK;
                uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_DIAG_MISC_CTRL2_OFFSET, &uCmd, WRITE_1BYTE);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step2: Wait for this diagnostic is done. */
                    (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_DIAG_CBFET_DELAY);

                    /* Step3: Checkif Diagnostics is running */
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_STAT1_OFFSET,
                                           &pDiagMgr->pDiagCfg[BMI_DIAG_DIAG_STAT], READ_2BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE1;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
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

    if(pDiagReq->uDiagStat >= (uint8)BMI_DIAG_ERROR)
    {
        uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;
        pDiagMgr->pDiagStatus->zMpfdtiStat.zMpfdtiOut.sCbOwFltInjStat =  (uint8)DIAG_STAT_EXEC_ERROR;

        uCmd = BQ7971X_DIAG_MISC_CTRL2_DIAG_CBFETOW_GO_MSK;
        (void)Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_DIAG_MISC_CTRL2_OFFSET, &uCmd, WRITE_1BYTE);
        (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_DIAG_CBFET_DELAY);
        (void)Bq7971x_UnpauseBalCtrl(pDiagMgr->pBmcMgr, BQ7971X_BAL_CTRL2_OFFSET, BQ7971X_CB_DIAG_UNPAUSE,
                               BQ7971X_CB_APP_CTRL);
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_CBOW_FLTINJ, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/*********************************************************************************************************************
 *  External Functions Definition
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagMpfdi(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
 *                                                   Bq7971x_DiagReqDataType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          MPFDI Diagnostics request handler state machine
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pSvcReqCfg - Diagnostic Request Service
 *  \param[in]      pDiagReq - Current Diag Request
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagMpfdi(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
                                                 Bq7971x_DiagReqType *pDiagReq ,uint8 uChannel)
{
    uint8 uRet;

    switch(pDiagReq->uDiagReqId)
    {
        case (uint8)BMI_SM_CBOW_FLTINJ:
        {
            uRet = CbOwFltinjDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)BMI_SM_MPFDI_UT_DAC:
        {
            uRet = OtUtDacDiag(pDiagMgr, pSvcReqCfg, pDiagReq, BQ7971X_OTUT_CTRL1_UT_THR);
            break;
        }
        case (uint8)BMI_SM_MPFDI_OT_DAC:
        {
            uRet = OtUtDacDiag(pDiagMgr, pSvcReqCfg, pDiagReq, BQ7971X_OTUT_CTRL1_OT_THR);
            break;
        }
        case (uint8)BMI_SM_MPFDI_UV_DAC:
        {
            uRet = OvUvDacDiag(pDiagMgr, pSvcReqCfg, pDiagReq, BQ7971X_OVUV_CTRL1_UV_THR);
            break;
        }
        case (uint8)BMI_SM_MPFDI_OV_DAC:
        {
            uRet = OvUvDacDiag(pDiagMgr, pSvcReqCfg, pDiagReq, BQ7971X_OVUV_CTRL1_OV_THR);
            break;
        }
        case (uint8)BMI_SM_MPFDI_CBFET_DIAG:
        {
            uRet = CbfetDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)BMI_SM_MPFDI_DCOMP_FLTINJ:
        {
            uRet = DcompFltinjDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)BMI_SM_MPFDI_ACOMP_FLTINJ:
        {
            uRet = AcompFltinjDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)BMI_SM_MPFDI_PWR_BIST:
        {
            uRet = PwrBistDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)BMI_SM_MPFDI_REFCAP_DIAG:
        {
            uRet = RefCapDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)BMI_SM_MPFDI_BGX_DIAG:
        {
            uRet = DiagBg1Bg2(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)BMI_SM_MPFDI_STARTUPINIT:
        {
            uRet = DiagStartupInit(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        default:
        {
            uRet = (uint8)BMI_BMC_DIAG_INVALID_CMD;
            break;
        }
    }

    return uRet;
}

#define BQ7971X_DIAGSTARTUP_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * End of File: bq7971x_diagmpfdi.c
 *********************************************************************************************************************/
