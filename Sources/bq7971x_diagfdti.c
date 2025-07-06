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
 *  File:       bq7971x_diagfdti.c
 *  Project:    TIBMS
 *  Module:     DIAG
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for BMI DIAG interface
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       24Aug2022    SEM                  0000000000000    Initial version
 * 01.01.00       26May2023    SEM                  0000000000000    Updated new steps for diagnostics
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
#define BQ7971X_DIAGFUNC_C_MAJOR_VERSION                    (0x01u)

/**	Minor Software Config C Version number */
#define BQ7971X_DIAGFUNC_C_MINOR_VERSION                    (0x00u)

/** Software Patch Config C Version number */
#define BQ7971X_DIAGFUNC_C_PATCH_VERSION                    (0x00u)

#if ((BQ7971X_DIAGFUNC_C_MAJOR_VERSION != BQ7971X_CFG_MAJOR_VERSION) || \
     (BQ7971X_DIAGFUNC_C_MINOR_VERSION != BQ7971X_CFG_MINOR_VERSION) || \
	 (BQ7971X_DIAGFUNC_C_PATCH_VERSION != BQ7971X_CFG_PATCH_VERSION))
#error "tibms: Config version numbers of bq7971x_diagfuncs.c and bq7971x_cfg.h are inconsistent!"
#endif

#if ((BQ7971X_DIAG_MAJOR_VERSION != BQ7971X_DIAGFUNC_C_MAJOR_VERSION) || \
     (BQ7971X_DIAG_MINOR_VERSION != BQ7971X_DIAGFUNC_C_MINOR_VERSION) || \
	 (BQ7971X_DIAG_PATCH_VERSION != BQ7971X_DIAGFUNC_C_PATCH_VERSION))
#error "tibms: Config version numbers of bq7971x_diagfuncs.c and bq7971x_diag.h are inconsistent!"
#endif

#if ((BQ7971X_REGS_MAJOR_VERSION != BQ7971X_DIAGFUNC_C_MAJOR_VERSION) || \
     (BQ7971X_REGS_MINOR_VERSION != BQ7971X_DIAGFUNC_C_MINOR_VERSION) || \
	 (BQ7971X_REGS_PATCH_VERSION != BQ7971X_DIAGFUNC_C_PATCH_VERSION))
#error "tibms: Config version numbers of bq7971x_diagfuncs.c and bq7971x_regs.h are inconsistent!"
#endif

/*********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

#define CB_EVEN_CELLS                                               (0u)
#define CB_ODD_CELLS                                                (1u)
#define DIAG_CELL_CB_TIME_DEFAULT                                   (1u)

/*********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

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

#define BQ7971X_DIAGFUNCS_START_SEC_CODE
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_diag_CODE) VCellPlauDiag(const Bq7971x_DiagType *pDiagMgr, uint16 *pVcellData)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of VC Open Detection State Machine
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pVcellData - VCell Data
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

STATIC FUNC(uint8, bq7971x_diag_CODE) VCellPlauDiag(const Bq7971x_DiagType *pDiagMgr, uint16 *pVcellData)
{
    DiagResultType *pDiagOut;
    uint16 wVCellVolt;
    uint8 uDevIdx;
    uint8 uVcellIdx;

    for(uDevIdx = pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
    {
        pDiagOut = &pDiagMgr->pDiagResult[uDevIdx];
        if(pDiagOut != NULL)
        {
            pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sVcPlau = (uint8)DIAG_STAT_NOERROR;
            for(uVcellIdx = 0u; uVcellIdx < BQ7971X_CELL_NUM_ACT; uVcellIdx++)
            {
                if(TIBMS_BitTest(pDiagMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNCellsCfg, uVcellIdx) != 0u)
                {
                    wVCellVolt = (uint16) pVcellData[(uDevIdx * BQ7971X_CELL_NUM_ACT) + uVcellIdx];
                    if((wVCellVolt < BQ7971X_MAXVCELLPLAUVALUE) && (wVCellVolt > BQ7971X_MINVCELLPLAUVALUE))
                    {
                        /* CELL Voltage Plausibility valid  */
                        TIBMS_BitClear(pDiagOut->zFdtiResult.zSM_VCELL_PLAU, uVcellIdx);
                    }
                    else
                    {
                        TIBMS_BitSet(pDiagOut->zFdtiResult.zSM_VCELL_PLAU, uVcellIdx);
                        pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sVcPlau = (uint8)DIAG_STAT_ERROR;
                    }
                }
            }
        }
    }

    return (uint8)BMI_OK;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_diag_CODE) GpioPlauDiag(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pGpioSvc)
 *********************************************************************************************************************/
/*! \brief          GPIO voltage Plausibility Check
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pGpioVolt
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval         BMI_OK
 *                  BMI_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_diag_CODE) GpioPlauDiag(const Bq7971x_DiagType *pDiagMgr, uint16 *pGpioVoltage)
{
    DiagResultType *pDiagOut;
    uint16 wGpioVoltage;
    uint8 uDevIdx;
    uint8 uGpioIdx;

    for(uDevIdx = pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
    {
        pDiagOut = &pDiagMgr->pDiagResult[uDevIdx];
        pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sGpioPlau = (uint8)DIAG_STAT_NOERROR;
        for(uGpioIdx = 0u; uGpioIdx < BQ7971X_GPIO_NUM_ACT; uGpioIdx++)
        {
            if(TIBMS_BitTest(pDiagMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNGpioCfg, uGpioIdx) != 0u)
            {
                wGpioVoltage = (uint16) pGpioVoltage[(uDevIdx * BQ7971X_GPIO_NUM_ACT) + uGpioIdx];
                if((wGpioVoltage < BQ7971X_MAXGPIOPLAUVALUE) && (wGpioVoltage > BQ7971X_MINGPIOPLAUVALUE))
                {
                    /* GPIO Voltage Plausibility valid  */
                    TIBMS_BitClear(pDiagOut->zFdtiResult.zSM_GPIO_PLAU, uGpioIdx);
                    if(wGpioVoltage > BQ7971X_GPIOOW_PULLDET)
                    {
                        /* In case a GPIO reads > 2.0V (results from step2), enable a weak pull-down */
                        TIBMS_BitSet(pDiagOut->zFdtiResult.zGpioPullSel, uGpioIdx);
                    }
                    else
                    {
                        /* in case a GPIO reads <2.0V, enable a weak pull-up */
                        TIBMS_BitClear(pDiagOut->zFdtiResult.zGpioPullSel, uGpioIdx);
                    }
                }
                else
                {
                    TIBMS_BitSet(pDiagOut->zFdtiResult.zSM_GPIO_PLAU, uGpioIdx);
                    pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sGpioPlau = (uint8)DIAG_STAT_ERROR;
                }
            }
        }
    }

    return (uint8)BMI_OK;
}
/**********************************************************************************************************************
 * uint8 DiagFdtiStatus(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Diagnostics FDTI Status check
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
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

STATIC FUNC(uint8, bq7971x_diag_CODE) DiagFdtiStatus(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uRunStatus;
    uint8 uDrdyStaus;
    uint8 uRet;
    uint8 uDevIdx;
    uint8 uLoop;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_FDTI_CYCLE_INIT == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);

                    uRunStatus = ((pData[3u] & (BQ7971X_DEV_STAT1_GPADC_RUN_MSK|BQ7971X_DEV_STAT1_VCELLADC_RUN_MSK)) |
                                  (pData[4u] & BQ7971X_DEV_STAT2_DIAG_DIG_RUN_MSK));
                    uDrdyStaus = ((pData[2u] & (BQ7971X_ADC_DATA_RDY_DRDY_GPADC_MSK |
                                   BQ7971X_ADC_DATA_RDY_DRDY_VCELLADC_MSK)) |
                                  (pData[1u] & BQ7971X_DIAG_STAT2_DRDY_DIG_MSK));

                    if((0u == uRunStatus) || (0u == uDrdyStaus))
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                    pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDiagStat = (uint8)DIAG_STAT_NOERROR;
                }
                else
                {
                    pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDiagStat = (uint8)DIAG_STAT_DRDY_ERROR;
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RETRY_ERROR;
                    pDiagReq->uDiagRetry++;
                    if(BQ7971X_DIAG_STAT_RETRY >= pDiagReq->uDiagRetry)
                    {
                        pDiagReq->uDiagRetry++;
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_STAT1_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_SM_FDTI_CYCLE_INIT], READ_5BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                    else
                    {
                        // TBD: Restart the Diagnostics, Report Error to be corrected outside FDTI
                        pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDiagStat = (uint8)DIAG_STAT_ABORT;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_ABORT;
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_READY:
        {
            /* Save Read Cell Voltage Value. The value is already decoded during the last main Cell read */
            pDiagMgr->pBswCfg->resource_req(pDiagMgr->pBswCfg->qApplRes, GET_RESOURCE);

            if((NULL != (pDiagMgr->pFuncCfg[BMI_SERVICE_DIAGNOSTICS].pDecodeBuf1)) && (NULL != pDiagMgr->pDiagResult))
            {
                (void)memcpy(pDiagMgr->pFuncCfg[BMI_SERVICE_DIAGNOSTICS].pDecodeBuf1, pDiagMgr->pDiagResult,
                       (sizeof(DiagResultType) * pDiagMgr->uNAFEs));
            }

            if((NULL != (pDiagMgr->pFuncCfg[BMI_SERVICE_DIAGNOSTICS].pDecodeBuf2)) && (NULL != pDiagMgr->pDiagStatus))
            {
                (void)memcpy(pDiagMgr->pFuncCfg[BMI_SERVICE_DIAGNOSTICS].pDecodeBuf2, pDiagMgr->pDiagStatus,
                       sizeof(DiagStatusType));
            }
			
			if(NULL != pDiagMgr->pFuncCfg[BMI_SERVICE_CELL_VOLT].pDecodeBuf0) 
			{
				(void)memcpy(pDiagMgr->pDiagCfg[BMI_SM_FDTI_VC_OPN_DET].pDecodeBuf0,
                   pDiagMgr->pFuncCfg[BMI_SERVICE_CELL_VOLT].pDecodeBuf0,
                   (pDiagMgr->pFuncCfg[BMI_SERVICE_CELL_VOLT].uDecodeLen * pDiagMgr->uNAFEs));
			}
			
			if(NULL != pDiagMgr->pFuncCfg[BMI_SERVICE_GPIO_READ].pDecodeBuf0)
			{
            /* Save Read GPIO Voltage Value. The value is already decoded during the last main GPIO read */
					(void)memcpy(pDiagMgr->pDiagCfg[BMI_SM_FDTI_GPIO_DIAG].pDecodeBuf0,
                   		pDiagMgr->pFuncCfg[BMI_SERVICE_GPIO_READ].pDecodeBuf0,
                   		(pDiagMgr->pFuncCfg[BMI_SERVICE_GPIO_READ].uDecodeLen * pDiagMgr->uNAFEs));
			} 
            pDiagMgr->pBswCfg->resource_req(pDiagMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

            pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDiagStat = (uint8)DIAG_STAT_PENDING;

            /* Cell PlauDiag. */
            (void)VCellPlauDiag(pDiagMgr, (uint16 *) pDiagMgr->pDiagCfg[BMI_SM_FDTI_VC_OPN_DET].pDecodeBuf0);

            /* GPIO PlauDiag. */
            (void)GpioPlauDiag(pDiagMgr, (uint16 *) pDiagMgr->pDiagCfg[BMI_SM_FDTI_GPIO_DIAG].pDecodeBuf0);

            uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_STAT1_OFFSET,
                                   &pDiagMgr->pDiagCfg[BMI_SM_FDTI_CYCLE_INIT], READ_5BYTE);
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
        pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDiagStat = (uint8)DIAG_STAT_EXEC_ERROR;
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_DIAG_STATUS, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) GpioConfInit(const Bq7971x_ManagerType *pBmicMgr)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_CODE) GpioConfInit(const Bq7971x_DiagType *pDiagMgr)
{
    uint8 uRet = (uint8)BMI_BMC_GPIOCFG_FAILED;
    uint8 uDevIdx;
    uint8 uLoop;
    //initialize GPIO CONF Register
    if((pDiagMgr->pBmicCfg->pBqRegsCfg->uGpioCfgPerDevInit) != 0u)
    {
        uLoop = pDiagMgr->sDevId;
        for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
        {
            uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx+1u),
                                      COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO_CONF1_OFFSET, COMIF_LOCK),
                            &pDiagMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx-pDiagMgr->sDevId].uGpio_Conf[0],
                                      BQ7971X_GPIO_CONF_REG_NUM);
        }
    }
    else
    {
        uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,
                                COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO_CONF1_OFFSET, COMIF_LOCK),
                                &pDiagMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uGpio_Conf[0],
                                BQ7971X_GPIO_CONF_REG_NUM);
    }


    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_diag_CODE) GpioOpnWrConfig(const Bq7971x_DiagType *pDiagMgr)
 *********************************************************************************************************************/
/*! \brief          GPIO Open Wire config, here the configuration is done according to the PULL UP/DOWN settings
 *
 *  \param[in]      pDiagMgr - Diagnostic context
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

STATIC FUNC(uint8, bq7971x_diag_CODE) GpioOpnWrConfig(const Bq7971x_DiagType *pDiagMgr)
{
    uint8 uRet = (uint8)BMI_OK;
    uint8 uCmd[BQ7971X_GPIO_CONF_REG_NUM];
    uint8 uDevIdx;
    uint8 uGpioIdx;
    uint8 uLoop = pDiagMgr->sDevId;

    for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
    {
        (void)memset(uCmd, 0, (sizeof(uint8) * BQ7971X_GPIO_CONF_REG_NUM));
        for(uGpioIdx = 0u; uGpioIdx < BQ7971X_GPIO_NUM_ACT; uGpioIdx++)
        {
            if(TIBMS_BitTest(pDiagMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNGpioCfg, uGpioIdx) != 0u)
            {
                if( TIBMS_BitTest(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zGpioPullSel, uGpioIdx) != 0u)
                {
                    uCmd[uGpioIdx/2U] |= (BQ7971X_GPIOSETPULLDOWN << ((uGpioIdx & 0x1U) * BQ7971X_GPIOSETNUM));
                }
                else
                {
                    uCmd[uGpioIdx/2U] |= (BQ7971X_GPIOSETPULLUP << ((uGpioIdx & 0x1U) * BQ7971X_GPIOSETNUM));
                }
            }
        }

        uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx,(uDevIdx+1u),
                                  COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO_CONF1_OFFSET, COMIF_LOCK),
                                  uCmd, BQ7971X_GPIO_CONF_REG_NUM);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_diag_CODE) GpioAdjShrtConfig(Bq7971x_DiagType *pDiagMgr)
 *********************************************************************************************************************/
/*! \brief          GPIO Adjshort Config setting ODD numbers low
 *
 *  \param[in]      pDiagMgr - Diagnostic context
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

STATIC FUNC(uint8, bq7971x_diag_CODE) GpioAdjShrtConfig(const Bq7971x_DiagType *pDiagMgr)
{
    uint8 uRet = (uint8)BMI_OK;
    uint8 uCmd[BQ7971X_GPIO_CONF_REG_NUM];
    uint8 uDevIdx;
    uint8 uGpioCfg;
    uint8 uGpioIdx;
    uint8 uLoop = pDiagMgr->sDevId;
    for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
    {
        (void)memset(uCmd, 0, (sizeof(uint8) * BQ7971X_GPIO_CONF_REG_NUM));
        for(uGpioIdx = 0u; uGpioIdx < BQ7971X_GPIO_NUM_ACT; uGpioIdx++)
        {
            if( TIBMS_BitTest(pDiagMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNGpioCfg, uGpioIdx) != 0u)
            {
                uGpioCfg = ((uGpioIdx & 0x1U) > 0u)? BQ7971X_GPIOSETADCONLY:BQ7971X_GPIOSETOUTLOW;
                uCmd[uGpioIdx/2U] |= (uGpioCfg << ((uGpioIdx & 0x1U) * BQ7971X_GPIOSETNUM));
            }
        }

        uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx,(uDevIdx+1U),
                                  COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO_CONF1_OFFSET, COMIF_LOCK),
                                  uCmd, BQ7971X_GPIO_CONF_REG_NUM);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_diag_CODE) CheckGpioOpenWire(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pGpioSvc)
 *********************************************************************************************************************/
/*! \brief          Evaluates Gpio status sets the Open Wire status
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pSvcCfg - Gpio read service context
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

STATIC FUNC(uint8, bq7971x_diag_CODE) GpioOpnWrUpdate(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg)
{
    uint16 *pGpioDataA;
    uint16 *pGpioDataB;
    uint16 wAbsVolt;
    uint8 uRet = (uint8)BMI_NOT_OK;
    uint8 uDevIdx;
    uint8 uGpioIdx;
    uint8 uDecIdx;
    uint8 uLoop;

    uRet = Bq7971x_DecodeGpio(pDiagMgr->pBmcMgr, pSvcCfg, (uint16 *) pSvcCfg->pDecodeBuf1);
    if((uint8)BMI_OK == uRet)
    {
        pGpioDataA = (uint16 *) pSvcCfg->pDecodeBuf0;
        pGpioDataB = (uint16 *) pSvcCfg->pDecodeBuf1;
        if((pGpioDataA != NULL) && (pGpioDataB != NULL))
        {
            pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sGpioOpnWr = (uint8)DIAG_STAT_NOERROR;
             uLoop = pDiagMgr->sDevId;
            for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {
                for(uGpioIdx = 0u; uGpioIdx < BQ7971X_GPIO_NUM_MAX; uGpioIdx++)
                {
                    if(((TIBMS_BitTest(pDiagMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNGpioCfg, uGpioIdx) != 0u)) &&
                       (!(TIBMS_BitTest(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_GPIO_PLAU, uGpioIdx)!= 0u)))
                    {
                        uDecIdx = ((uDevIdx * BQ7971X_GPIO_NUM_MAX) + uGpioIdx);

                        /* if (ABS Value A - Value B) > 1.0V, GPIO open wire error */
                        wAbsVolt = DiagAbsDiffVoltage(pGpioDataA[uDecIdx], pGpioDataB[uDecIdx]);
                        if(wAbsVolt > BQ7971X_GPIOOPENWIRE_THRESHOLD)
                        {
                            TIBMS_BitSet(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_GPIO_OPNWR, uGpioIdx);
                            pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sGpioOpnWr = (uint8)DIAG_STAT_ERROR;
                        }
                        else
                        {
                            TIBMS_BitClear(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_GPIO_OPNWR, uGpioIdx);
                        }
                    }
                    else
                    {
                        /* Plausibility Check Failed or Not configured as Input ADC,
                           So Only Plausibility error, No Open Wire error */
                    }
                }
            }
            uRet = (uint8)BMI_OK;
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_diag_CODE) GpioAdjShrtUpdate(Bq7971x_DiagType *pDiagMgr, uint8 *pDiagCfg)
 *********************************************************************************************************************/
/*! \brief          Evaluates the gpio status from current Gpio read and prev saved read
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pDiagCfg - for setting the GPIO CONFIG
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

STATIC FUNC(uint8, bq7971x_diag_CODE) GpioAdjShrtUpdate(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg)
{
    uint16 *pGpioData1;
    uint16 *pGpioData2;
    uint16 wAbsVolt;
    uint8 uDecIdx;
    uint8 uDevIdx;
    uint8 uGpioIdx;
    uint8 uRet = (uint8)BMI_NOT_OK;
    uint8 uLoop;

    uRet = Bq7971x_DecodeGpio(pDiagMgr->pBmcMgr, pSvcCfg, (uint16 *) pSvcCfg->pDecodeBuf1);
    if((uint8)BMI_OK == uRet)
    {
        pGpioData1 = (uint16 *) pSvcCfg->pDecodeBuf0;
        pGpioData2 = (uint16 *) pSvcCfg->pDecodeBuf1;
        if((pGpioData1 != NULL) && (pGpioData2 != NULL))
        {
            pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sGpioAdjShrt = (uint8)DIAG_STAT_NOERROR;
            uLoop = pDiagMgr->sDevId;
            for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {
                uGpioIdx = 1u;
                while(uGpioIdx < BQ7971X_GPIO_NUM_MAX)        // 0th Index is GPIO1, 1st Index GPIO2 - EVEN GPIO
                {
                    if(TIBMS_BitTest(pDiagMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNGpioCfg, uGpioIdx) != 0u)
                    {
                        uDecIdx = ((uDevIdx * BQ7971X_GPIO_NUM_MAX) + uGpioIdx);
                        wAbsVolt = DiagAbsDiffVoltage(pGpioData1[uDecIdx], pGpioData2[uDecIdx]);
                        if(wAbsVolt > BQ7971X_GPIOADJSHORT_THRESHOLD)
                        {
                            if((!(TIBMS_BitTest(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_GPIO_OPNWR, uGpioIdx)!= 0u)) &&
                               (!(TIBMS_BitTest(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_GPIO_PLAU, uGpioIdx) != 0u)))
                            {
                                TIBMS_BitSet(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_GPIO_ADJSHRT, uGpioIdx);
                                pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sGpioAdjShrt = (uint8)DIAG_STAT_ERROR;
                            }
                        }
                        else
                        {
                            TIBMS_BitClear(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_GPIO_ADJSHRT, uGpioIdx);
                        }
                    }
                    uGpioIdx+=2U;
                }
            }
            uRet = (uint8)BMI_OK;
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * uint8 GpioDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of GPIO Open and Adjshort Detection State Machine
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
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

STATIC FUNC(uint8, bq7971x_diag_CODE) GpioDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                               Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint16 wGpioStat;
    uint8 uRet;
    uint8 uDevIdx;
    uint8 uGpioIdx;
    uint8 uLoop;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_FDTI_GPIO_DIAG == pSvcCfg->uSubId)
            {
                /* Step4.5: Update GPIO AdjShort Wire Status  */
                uRet = GpioAdjShrtUpdate(pDiagMgr, pSvcCfg);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step5: Update Original GPIO Configuration,
                       incase of error in the Step 4.5 the error handling will update the original config  */
                    uRet = GpioConfInit(pDiagMgr);
                    if((uint8)BMI_OK == uRet)
                    {
                        /* Step5.1: Delay GPIO Configuration  */
                        (void)Comif_SetDelay(pDiagMgr->pComifCtx, COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO_CONF_DELAY, COMIF_UNLOCK));
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE2:
        {
            if(BMI_DIAG_GPIO_STAT == pSvcCfg->uSubId)
            {
                 /* Step4.4: Update GPIO AdjShort Wire Status  */
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    wGpioStat = (uint16) ((pData[1u] << 8u) | pData[0u]);
                    uGpioIdx = 0u;
                    while(uGpioIdx < BQ7971X_GPIO_NUM_MAX)                // 0th Index is GPIO1, ODD GPIO
                    {
                        if(!(TIBMS_BitTest(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_GPIO_PLAU, uGpioIdx)!= 0u))
                        {
                            if(TIBMS_BitTest(wGpioStat, uGpioIdx) != 0u)
                            {
                                 TIBMS_BitSet(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_GPIO_ADJSHRT, uGpioIdx);
                            }
                        }
                        uGpioIdx+=2U;
                    }
                }
                pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                uRet = (uint8)BMI_OK;
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE1:
        {
            if((uint8)BMI_SM_FDTI_GPIO_DIAG == pSvcCfg->uSubId)
            {
                /* Step3.3: Update GPIO Open Wire Status  */
                uRet = GpioOpnWrUpdate(pDiagMgr, pSvcCfg);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step4: AdjShort Configuration . */
                    uRet = GpioAdjShrtConfig(pDiagMgr);
                    if((uint8)BMI_OK == uRet)
                    {
                        /* Step4.1: Delay for Setting AdjShort Configuration . Delay will make sure it is re-try safe for WBMS */
                        (void)Comif_SetDelay(pDiagMgr->pComifCtx, COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO_CONF_DELAY, COMIF_LOCK));

                        /* Step4.2: GPIO STAT Register. Clubbing this reads for better timing, the digital output error is very rare*/
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                               COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO_STAT1_OFFSET, COMIF_LOCK),
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_GPIO_STAT], READ_2BYTE);

                        /* Step4.3: Read GPIO voltage value for checking AdjShort. */
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                               COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO1_HI_OFFSET, COMIF_LOCK),
                                               &pDiagMgr->pDiagCfg[BMI_SM_FDTI_GPIO_DIAG],
                                               (BQ7971X_GPIO_NUM_MAX * READ_2BYTE));
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE2;
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_READY:
        {
            pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sGpioOpnWr = (uint8)DIAG_STAT_PENDING;
            pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sGpioAdjShrt = (uint8)DIAG_STAT_PENDING;

            /* Step1 and 2: Covered in Status Check : checking the ADC Status and Plausibility check */
            /* Step3: Write GPIO configure ADC & Weak Pull-Up Or Down. */
            uRet = GpioOpnWrConfig(pDiagMgr);
            if((uint8)BMI_OK == uRet)
            {
                /* Step3.1 : Delay for GPIO configure ADC & Weak Pull-Up Or Down. Delay will make sure it is re-try safe for WBMS*/
                (void)Comif_SetDelay(pDiagMgr->pComifCtx, COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO_CONF_DELAY, COMIF_LOCK));

                /* Step3.2: Read GPIO Voltage after configuring the GPIO for checking GPIO Open Wire */
                uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                       COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO1_HI_OFFSET, COMIF_LOCK),
                                       &pDiagMgr->pDiagCfg[BMI_SM_FDTI_GPIO_DIAG],
                                       (BQ7971X_GPIO_NUM_MAX * READ_2BYTE));
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
        pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sGpioOpnWr = (uint8)DIAG_STAT_EXEC_ERROR;
        pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sGpioAdjShrt = (uint8)DIAG_STAT_EXEC_ERROR;

       /* Update Original GPIO Configuration incase of ERROR */
        if((uint8)BMI_OK == GpioConfInit(pDiagMgr))
        {
            /* Delay GPIO Configuration to make sure that Normal GPIO read has proper results */
            (void)Comif_SetDelay(pDiagMgr->pComifCtx, COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO_CONF_DELAY, COMIF_UNLOCK));
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_GPIO_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uint8 GpioOpnWireDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of GPIO Open Wire Detection State Machine
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
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

STATIC FUNC(uint8, bq7971x_diag_CODE) GpioOpnWireDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq)
{
    uint8 uRet;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_FDTI_GPIO_OPNWR == pSvcCfg->uSubId)
            {
                /* Step3.3: Update GPIO Open Wire Status  */
                uRet = GpioOpnWrUpdate(pDiagMgr, pSvcCfg);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step4: Update Original GPIO Configuration
                       incase of error in the Step 3.3 the error handling will update the original config  */
                    uRet = GpioConfInit(pDiagMgr);
                    if((uint8)BMI_OK == uRet)
                    {
                        /* Step4.1: Delay GPIO Configuration  */
                        (void)Comif_SetDelay(pDiagMgr->pComifCtx, COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO_CONF_DELAY, COMIF_UNLOCK));
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                    }
                }
            }
            break;
        }
        case DIAG_STEP_READY:
        {
            pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sGpioOpnWr = (uint8)DIAG_STAT_PENDING;

            /* Step1 and 2: Covered in Status Check : checking the ADC Status and Plausibility check */
            /* Step3: Write GPIO configure ADC & Weak Pull-Up Or Down. */
            uRet = GpioOpnWrConfig(pDiagMgr);
            if((uint8)BMI_OK == uRet)
            {
                /* Step3.1 : Delay for GPIO configure ADC & Weak Pull-Up Or Down. */
                (void)Comif_SetDelay(pDiagMgr->pComifCtx, COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO_CONF_DELAY, COMIF_LOCK));

                /* Step3.2: Read GPIO Voltage after configuring the GPIO for checking GPIO Open Wire */
                uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                       COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO1_HI_OFFSET, COMIF_LOCK),
                                       &pDiagMgr->pDiagCfg[BMI_SM_FDTI_GPIO_OPNWR],
                                       (BQ7971X_GPIO_NUM_MAX * READ_2BYTE));
                if((uint8)BMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
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

    if(pDiagReq->uDiagStat >= BMI_DIAG_ERROR)
    {
        uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;
        pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sGpioOpnWr = (uint8)DIAG_STAT_EXEC_ERROR;

       /* Update Original GPIO Configuration incase of ERROR */
        if((uint8)BMI_OK == GpioConfInit(pDiagMgr))
        {
            /* Delay GPIO Configuration to make sure that Normal GPIO read has proper results */
            (void)Comif_SetDelay(pDiagMgr->pComifCtx, COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO_CONF_DELAY, COMIF_UNLOCK));
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_GPIO_OPNWR, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uint8 GpioDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of GPIO Open and Adjshort Detection State Machine
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
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

STATIC FUNC(uint8, bq7971x_diag_CODE) GpioAdjShrtDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint16 wGpioStat;
    uint8 uRet;
    uint8 uDevIdx;
    uint8 uGpioIdx;
    uint8 uLoop;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_FDTI_GPIO_ADJSHRT == pSvcCfg->uSubId)
            {
                /* Step3.3.1: Update GPIO AdjShort Wire Status  */
                uRet = GpioAdjShrtUpdate(pDiagMgr, pSvcCfg);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step4: Update Original GPIO Configuration
                       incase of error in the Step 3.3.1 the error handling will update the original config  */
                    uRet = GpioConfInit(pDiagMgr);
                    if((uint8)BMI_OK == uRet)
                    {
                        /* Step4.1: Delay GPIO Configuration  */
                        (void)Comif_SetDelay(pDiagMgr->pComifCtx, COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO_CONF_DELAY, COMIF_UNLOCK));
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                    }
                }
            }
            break;
        }
        case DIAG_STEP_STATE1:
        {
            if(BMI_DIAG_GPIO_STAT == pSvcCfg->uSubId)
            {
                 /* Step3.2.1: Update GPIO AdjShort Wire Status  */
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    wGpioStat = (uint16) ((pData[1u] << 8u) | pData[0u]);
                    uGpioIdx = 1u;
                    while(uGpioIdx < BQ7971X_GPIO_NUM_MAX)
                    {
                        if(!(TIBMS_BitTest(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_GPIO_PLAU, uGpioIdx) !=0u))
                        {
                            if(TIBMS_BitTest(wGpioStat, uGpioIdx) != 0u)
                            {
                                 TIBMS_BitSet(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_GPIO_ADJSHRT, uGpioIdx);
                            }
                        }
                        uGpioIdx+=2U;
                    }
                }
                pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                uRet = (uint8)BMI_OK;
            }
            break;
        }
        case DIAG_STEP_READY:
        {
            pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sGpioAdjShrt = (uint8)DIAG_STAT_PENDING;

            /* Step1 and 2: Covered in Status Check : checking the ADC Status and Plausibility check */
            /* Step3: AdjShort Configuration . */
            uRet = GpioAdjShrtConfig(pDiagMgr);
            if((uint8)BMI_OK == uRet)
            {
                /* Step3.1: Delay for Setting AdjShort Configuration . Delay will make sure it is re-try safe for WBMS */
                (void)Comif_SetDelay(pDiagMgr->pComifCtx, COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO_CONF_DELAY, COMIF_LOCK));

                /* Step3.2: GPIO STAT Register. Clubbing this reads for better timing, the digital output error is very rare*/
                uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                       COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO_STAT1_OFFSET, COMIF_LOCK),
                                       &pDiagMgr->pDiagCfg[BMI_DIAG_GPIO_STAT], READ_2BYTE);

                /* Step3.3: Read GPIO voltage value for checking AdjShort. */
                uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                       COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO1_HI_OFFSET, COMIF_LOCK),
                                       &pDiagMgr->pDiagCfg[BMI_SM_FDTI_GPIO_ADJSHRT],
                                       (BQ7971X_GPIO_NUM_MAX * READ_2BYTE));
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

    if(pDiagReq->uDiagStat >= BMI_DIAG_ERROR)
    {
        uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;
        pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sGpioAdjShrt = (uint8)DIAG_STAT_EXEC_ERROR;

       /* Update Original GPIO Configuration incase of ERROR */
        if((uint8)BMI_OK == GpioConfInit(pDiagMgr))
        {
            /* Delay GPIO Configuration to make sure that Normal GPIO read has proper results */
            (void)Comif_SetDelay(pDiagMgr->pComifCtx, COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO_CONF_DELAY, COMIF_UNLOCK));
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_GPIO_ADJSHRT, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uint8 VCellOpenDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of VC Open Detection State Machine
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
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

STATIC FUNC(uint8, bq7971x_diag_CODE) VCellOpenDiag(const Bq7971x_DiagType *pDiagMgr,
                                                    Bq7971x_DiagReqType *pDiagReq)
{
    uint16 *pVcellData;
    uint16 *pPrevVcellData;
    uint16 wAbsVolt;
    uint8 uRet;
    uint8 uDevIdx;
    uint8 uCellIdx;
    uint8 uDecIdx;
    uRet = BMI_OK;
    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    uint8 uLoop;
    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:

        {
            uRet = (uint8)BMI_OK;
            pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sVcOpen = (uint8)DIAG_STAT_ERROR;

            pVcellData = (uint16 *) pDiagMgr->pDiagCfg[BMI_SM_FDTI_VC_OPN_DET].pDecodeBuf0;
            pPrevVcellData = (uint16 *) pDiagMgr->pDiagCfg[BMI_SM_FDTI_VC_OPN_DET].pDecodeBuf1;
            uLoop = pDiagMgr->sDevId;
            for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {
                for(uCellIdx = 0u; uCellIdx < BQ7971X_CELL_NUM_ACT; uCellIdx++)
                {
                    if((TIBMS_BitTest(pDiagMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNCellsCfg, uCellIdx) != 0u) &&
                       (!(TIBMS_BitTest(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_VCELL_PLAU, uCellIdx) != 0u)))
                    {
                        uDecIdx = ((uDevIdx * BQ7971X_CELL_NUM_ACT) + uCellIdx);
                        wAbsVolt = DiagAbsDiffVoltage(pVcellData[uDecIdx], pPrevVcellData[uDecIdx]);
                        if(wAbsVolt > BQ7971X_VCELL_THRESHHOLD)
                        {
                            TIBMS_BitSet(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_VC_OPN_DET, uCellIdx);
                            uRet = (uint8)BMI_NOT_OK;
                        }
                        else
                        {
                            TIBMS_BitClear(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_VC_OPN_DET, uCellIdx);
                        }
                        pPrevVcellData[uDecIdx] = pVcellData[uDecIdx];
                    }
                    else
                    {
                        /* Plausibility Check Failed or Not configured as Input ADC,
                           So Only Plausibility error, No Open Wire error */
                    }
                }
            }

            if((uint8)BMI_OK == uRet)
            {
                pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sVcOpen = (uint8)DIAG_STAT_NOERROR;
            }

            pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
            pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
            uRet = (uint8)BMI_OK;
            break;
        }
        case DIAG_STEP_READY:
        {
              uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL( (BQ7971X_VCELL18_HI_OFFSET + ((BQ7971X_CELL_NUM_MAX - BQ7971X_CELL_NUM_ACT) * 2u)), COMIF_LOCK),
                                    &pDiagMgr->pDiagCfg[BMI_SM_FDTI_VC_OPN_DET],(BQ7971X_CELL_NUM_ACT * READ_1BYTE));
              if(BMI_OK == uRet)
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

    if(pDiagReq->uDiagStat >= BMI_DIAG_ERROR)
    {
        uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;
        pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sVcOpen = (uint8)DIAG_STAT_EXEC_ERROR;
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_VC_OPN_DET, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_diag_CODE) VCCBShrtCellBalCfg(Bq7971x_DiagType *pDiagMgr, uint8 uSCellIdx)
 *********************************************************************************************************************/
/*! \brief          Setting EVEN CB
 *
 *  \param[in]      pDiagMgr - Diagnostic context
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

STATIC FUNC(uint8, bq7971x_diag_CODE) VCCBShrtCellBalCfg(const Bq7971x_DiagType *pDiagMgr, uint8 uSCellIdx)
{
    uint8 uCbCtrl[BQ7971X_CELL_NUM_ACT];
    uint8 uDevIdx;
    uint8 uCellIdx;
    uint8 uRet = (uint8)BMI_NOT_OK;
    uint8 uLoop = pDiagMgr->sDevId;
    for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
    {
        (void)memset(uCbCtrl, 0, BQ7971X_CELL_NUM_ACT);
        uCellIdx = uSCellIdx;
        while(uCellIdx < BQ7971X_CELL_NUM_ACT)
        {
            if(TIBMS_BitTest(pDiagMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNCellsCfg, uCellIdx) != 0u)
            {
                uCbCtrl[(BQ7971X_CELL_NUM_ACT - 1u - uCellIdx)] = DIAG_CELL_CB_TIME_DEFAULT;
            }
            uCellIdx += 2u;
        }


        uRet = Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx + 1u),
                                 COMIF_PRIO_EXCL_CELL(BQ7971X_CB_CELL18_CTRL_OFFSET, COMIF_LOCK),
                                 &uCbCtrl[0], 8u);
        if((uint8)BMI_OK == uRet)
        {
            uRet = Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx + 1u),
                                     COMIF_PRIO_EXCL_CELL(BQ7971X_CB_CELL10_CTRL_OFFSET, COMIF_LOCK),
                                     &uCbCtrl[8], 8u);
            if((uint8)BMI_OK == uRet)
            {
                uRet = Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx + 1u),
                                         COMIF_PRIO_EXCL_CELL(BQ7971X_CB_CELL2_CTRL_OFFSET, COMIF_LOCK),
                                         &uCbCtrl[BQ7971X_CELL_NUM_ACT-1], 2u);
            }
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7971x_diag_CODE) VCCBShrtUpdate(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pVCell)
 *********************************************************************************************************************/
/*! \brief          Update the Status Result to DiagResults buffer
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pDiagCfg - VCELL read service context
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
STATIC FUNC(uint8, bq7971x_diag_CODE) VCCBShrtUpdate(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     uint8 uSCellIdx, uint8 uJwi)
{
    uint16 *pVCellData1;
    uint16 *pVCellData2;
    uint16 wAbsVolt;
    uint8 uRet = (uint8)BMI_NOT_OK;
    uint8 uDevIdx;
    uint8 uCellIdx;
    uint8 uLoop;

    pVCellData1 = (uint16 *) pSvcCfg->pDecodeBuf0;
    pVCellData2 = (uint16 *) pSvcCfg->pDecodeBuf1;
    if((pVCellData1 != NULL) && (pVCellData2 != NULL))
    {
        uRet = Bq7971x_DecodeVcell(pDiagMgr->pBmcMgr, pSvcCfg, pVCellData2);
        if(BMI_OK == uRet)
        {
            uLoop = pDiagMgr->sDevId;
            for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {
                uCellIdx = uSCellIdx;
                while (uCellIdx < BQ7971X_CELL_NUM_ACT)
                {
                    wAbsVolt = DiagAbsDiffVoltage(pVCellData1[(uDevIdx*BQ7971X_CELL_NUM_ACT)+uCellIdx],
                                                  pVCellData2[(uDevIdx*BQ7971X_CELL_NUM_ACT)+uCellIdx]);

                    if((wAbsVolt > BQ7971X_CELLVCCBSHRT_THRESHOLD))
                    {
                        TIBMS_BitSet(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_VCCB_SHRT, uCellIdx);
                        TIBMS_BitClear(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_CABLE_RES_AB, uCellIdx);
                        uRet = (uint8)BMI_NOT_OK;
                    }
                    else
                    {
                        if(wAbsVolt > BQ7971X_CABLE_RES_AB_THRESHOLD)
                        {
                            TIBMS_BitSet(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_VCCB_SHRT, uCellIdx);
                        }
                        else
                        {
                            TIBMS_BitClear(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_CABLE_RES_AB, uCellIdx);
                        }
                        TIBMS_BitClear(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_VCCB_SHRT, uCellIdx);
                    }
                    uCellIdx += uJwi;
                }
            }
        }
    }
    
    return uRet;
}

/**********************************************************************************************************************
 * uint8 VCellOpenDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of VC Open Detection State Machine
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
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
STATIC FUNC(uint8, bq7971x_diag_CODE) VCCBShortDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                    Bq7971x_DiagReqType *pDiagReq)
{
    uint8 uRet;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_FDTI_VCCB_SHRT == pSvcCfg->uSubId)
            {
                /* Step 4.3: Compare and update the VCCB Short Diag for EVEN and ODD cells, indx starts at 0, increments 1 */
                uRet = VCCBShrtUpdate(pDiagMgr, pSvcCfg, 0u, 1u);
                if((uint8)BMI_NOT_OK == uRet)
                {
                    pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sVccbShrt = (uint8)DIAG_STAT_ERROR;
                }

                /* Step 7: Pause Cell Balance */
                uRet = Bq7971x_PauseBalCtrl(pDiagMgr->pBmcMgr,
                                            COMIF_PRIO_EXCL_CELL(BQ7971X_BAL_CTRL2_OFFSET, COMIF_UNLOCK),
                                            BQ7971X_CB_DIAG_PAUSE, BQ7971X_CB_APP_CTRL);
                if((uint8)BMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                }
            }
            break;
        }
        case DIAG_STEP_STATE2:
        {
            if((uint8)BMI_SM_FDTI_VCCB_SHRT == pSvcCfg->uSubId)
            {
                /* Step 5.3: compare and update VCCB Shrt Diag status. */
                uRet = VCCBShrtUpdate(pDiagMgr, pSvcCfg, CB_EVEN_CELLS, 2u);
                if((uint8)BMI_NOT_OK == uRet)
                {
                    pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sVccbShrt = (uint8)DIAG_STAT_ERROR;
                }

                /* Step 6: Turn off cell balancing */
                uRet = VCCBShrtCellBalCfg(pDiagMgr, BQ7971X_CELL_NUM_ACT);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step 6.1: Pause cell balancing */
                    uRet = Bq7971x_PauseBalCtrl(pDiagMgr->pBmcMgr,
                                                COMIF_PRIO_EXCL_CELL(BQ7971X_BAL_CTRL2_OFFSET, COMIF_UNLOCK),
                                                BQ7971X_CB_DIAG_PAUSE, BQ7971X_CB_APP_CTRL);
                    if((uint8)BMI_OK == uRet)
                    {
                        /* Step 6.2: Wait for DLPF filter to settle - Wait at least 0.5ms or more */
                        (void)Comif_SetDelay(pDiagMgr->pComifCtx, COMIF_PRIO_EXCL_CELL(BQ7971X_VCCBSHRT_DELAY, COMIF_UNLOCK)); // TBD: To be checked if delay is really needed
                        pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sVccbShrt = (uint8)DIAG_STAT_NOERROR;
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                    }
                }
            }
            break;
        }
        case DIAG_STEP_STATE1:
        {
            if((uint8)BMI_SM_FDTI_VCCB_SHRT == pSvcCfg->uSubId)
            {
                uRet = VCCBShrtUpdate(pDiagMgr, pSvcCfg, CB_ODD_CELLS, 2u);
                if((uint8)BMI_NOT_OK == uRet)
                {
                    pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sVccbShrt = (uint8)DIAG_STAT_ERROR;
                }

                /* Step 5: Select channels to enable CB FET. E.g. all Even channels only */
                uRet = VCCBShrtCellBalCfg(pDiagMgr, CB_EVEN_CELLS);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step 5.1: Wait for DLPF filter to settle - Wait at least 0.5ms or more */
                    (void)Comif_SetDelay(pDiagMgr->pComifCtx, COMIF_PRIO_EXCL_CELL(BQ7971X_VCCBSHRT_DELAY, COMIF_LOCK));  // TBD: To be checked if delay is really needed

                    /* Step 5.2: Save Vcell Value for compare. */
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                           COMIF_PRIO_EXCL_CELL((BQ7971X_VCELL18_HI_OFFSET +
                                           ((BQ7971X_CELL_NUM_MAX - BQ7971X_CELL_NUM_ACT) * READ_2BYTE)), COMIF_LOCK),
                                           &pDiagMgr->pDiagCfg[BMI_SM_FDTI_VCCB_SHRT],
                                           (BQ7971X_CELL_NUM_ACT * READ_2BYTE));
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE2;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                    }
                }
            }
            break;
        }
        case DIAG_STEP_READY:
        {
            pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sVccbShrt = (uint8)DIAG_STAT_NOERROR;

            /* Step 1/2: Store the Read Vcell values */   // TBD: Need to check this during Keyon Diag
            uRet = Bq7971x_GetBalCtrlStatus(pDiagMgr->pBmcMgr);
            if((uint8)BQ7971X_CB_APP_NONE == uRet)
            {
                /* Step 3: Select channels to enable CB FET. E.g. all odd channels only */
                uRet = VCCBShrtCellBalCfg(pDiagMgr, CB_ODD_CELLS);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step 3.1: Start cell balancing */
                    uRet = Bq7971x_StartBalCtrl(pDiagMgr->pBmcMgr,
                                                COMIF_PRIO_EXCL_CELL(BQ7971X_BAL_CTRL2_OFFSET, COMIF_LOCK),
                                                BQ7971X_CB_DIAG_RUN, BQ7971X_CB_DIAG_CTRL, 0x1u);
                    if((uint8)BMI_OK == uRet)
                    {
                        /* Step 3.2: Wait for DLPF filter to settle - Wait at least 0.5ms or more */
                        uRet = Comif_SetDelay(pDiagMgr->pComifCtx, COMIF_PRIO_EXCL_CELL(BQ7971X_VCCBSHRT_DELAY, COMIF_LOCK)); // TBD: To be checked if delay is really needed
                        if((uint8)BMI_OK == uRet)
                        {
                            /* Step3.3: Save Vcell Value for compare. */
                            uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                                   COMIF_PRIO_EXCL_CELL((BQ7971X_VCELL18_HI_OFFSET +
                                                   ((BQ7971X_CELL_NUM_MAX - BQ7971X_CELL_NUM_ACT) * READ_2BYTE)), COMIF_LOCK),
                                                   &pDiagMgr->pDiagCfg[BMI_SM_FDTI_VCCB_SHRT],
                                                   (BQ7971X_CELL_NUM_ACT * READ_2BYTE));
                            if((uint8)BMI_OK == uRet)
                            {
                                pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE1;
                                pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                            }
                        }
                    }
                }
            }
            else
            {
                /* Step 4: Unpause cell balancing */
                uRet = Bq7971x_UnpauseBalCtrl(pDiagMgr->pBmcMgr,
                                              COMIF_PRIO_EXCL_CELL(BQ7971X_BAL_CTRL2_OFFSET, COMIF_LOCK),
                                              BQ7971X_CB_DIAG_UNPAUSE, BQ7971X_CB_DIAG_CTRL);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step 4.1: Wait for DLPF filter to settle - Wait at least 0.5ms or more */
                    uRet = Comif_SetDelay(pDiagMgr->pComifCtx, COMIF_PRIO_EXCL_CELL(BQ7971X_VCCBSHRT_DELAY, COMIF_LOCK));
                    if((uint8)BMI_OK == uRet)
                    {
                        /* Step4.2: Save Vcell Value for compare. */
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                               COMIF_PRIO_EXCL_CELL((BQ7971X_VCELL18_HI_OFFSET +
                                               ((BQ7971X_CELL_NUM_MAX - BQ7971X_CELL_NUM_ACT) * READ_2BYTE)),
                                                COMIF_LOCK), &pDiagMgr->pDiagCfg[BMI_SM_FDTI_VCCB_SHRT],
                                               (BQ7971X_CELL_NUM_ACT * READ_2BYTE));
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
        default:
        {
            break;
        }
    }

    if(pDiagReq->uDiagStat >= BMI_DIAG_ERROR)
    {
        uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;
        pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sVccbShrt = (uint8)DIAG_STAT_EXEC_ERROR;
        (void)Bq7971x_PauseBalCtrl(pDiagMgr->pBmcMgr, COMIF_PRIO_EXCL_CELL(BQ7971X_BAL_CTRL2_OFFSET, COMIF_UNLOCK),
                             BQ7971X_CB_DIAG_PAUSE, BQ7971X_CB_APP_CTRL);
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_VCCB_SHRT_DET, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uint8 AcompDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of VC Open Detection State Machine
 *
 *  \param[in]      pDiagMgr - Diagnostic context
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
STATIC FUNC(uint8, bq7971x_diag_CODE) AcompDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint32 qFault;
    uint8 uRet;
    uint8 uDevIdx;
    uint8 uCmd;
    uint8 uLoop;
    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_FDTI_VCELLGPIO_ACOMP == pSvcCfg->uSubId)
            {
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    // VCELL Fault Inject Status Update
                    qFault = ((uint32)((pData[2] << 16u) | (pData[3] << 8u) | (pData[4] << 0u)) &
                                pDiagMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNCellsCfg);

                    pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_ACOMP_VCELL = qFault;
                    // GPIO Fault Injection Status Update
                    qFault = (uint32) ((uint16)((pData[0] << 8u) | (pData[1] << 0u)) &
                              pDiagMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNGpioCfg);
                    pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_ACOMP_GPIO = qFault;
                }
                pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                uRet = (uint8)BMI_OK;
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE4:
        {
            if((uint8)BMI_SM_FDTI_FAULT_SUMMARY == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_FAULT_SUMMARY = (uint8) pData[0u];
                    qFault = (pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_FAULT_SUMMARY &
                              BQ7971X_FAULT_SUMMARY_FAULT_ADC_CB_MSK);
                    if(BQ7971X_FAULT_SUMMARY_FAULT_ADC_CB_MSK == qFault)
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_NOT_OK == uRet)
                {
                    pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sACompDiag = (uint8)DIAG_STAT_ERROR;
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                           COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_ADC_GPIO1_OFFSET, COMIF_LOCK),
                                           &pDiagMgr->pDiagCfg[BMI_SM_FDTI_VCELLGPIO_ACOMP],
                                           (BQ7971X_FAULT_ADC_VCELL_NUM+BQ7971X_FAULT_ADC_GPIO_NUM));
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                    }
                }
                else
                {
                    pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sACompDiag = (uint8)DIAG_STAT_NOERROR;
                    pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                }

                uCmd = (BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_MSK | BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_GO_MSK);
                uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,
                                        COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_ADC_CTRL3_OFFSET, COMIF_LOCK),
                                        &uCmd, WRITE_1BYTE);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step9: Remove CBCtrl Pause request from Diag */
                    uRet = Bq7971x_UnpauseBalCtrl(pDiagMgr->pBmcMgr,
                                                  COMIF_PRIO_EXCL_CELL(BQ7971X_BAL_CTRL2_OFFSET, COMIF_UNLOCK),
                                                  BQ7971X_CB_DIAG_UNPAUSE, BQ7971X_CB_APP_CTRL);
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE3:
        {
            if(BMI_DIAG_DIAG_STAT == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == (pData[1u] & (BQ7971X_DIAG_STAT2_DRDY_ANA_VCELL_MSK |
                                           BQ7971X_DIAG_STAT2_DRDY_ANA_GPIO_MSK)))
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                           COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_SUMMARY_OFFSET, COMIF_LOCK),
                                           &pDiagMgr->pDiagCfg[BMI_SM_FDTI_FAULT_SUMMARY], READ_1BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE4;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                    }
                }
                else
                {
                    pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sACompDiag = (uint8)DIAG_STAT_DRDY_ERROR;
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RETRY_ERROR;
                    pDiagReq->uDiagRetry++;
                    if(pDiagReq->uDiagRetry < BQ7971X_DIAG_MAX_RETRY)
                    {
                        /* Step5: Checkif Diagnostics Completed */
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                               COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_STAT1_OFFSET, COMIF_LOCK),
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
        case (uint8)DIAG_STEP_STATE2:
        {
            if(BMI_DIAG_FAULT_ADC_MISC == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(1u == (pData[0u] & (BQ7971X_FAULT_ADC_MISC_DIAG_ANA_ABORT_MSK)))
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    /* Step3: Retry - the diagnostic is running check. */
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                           COMIF_PRIO_EXCL_CELL(BQ7971X_DEV_STAT1_OFFSET, COMIF_LOCK),
                                           &pDiagMgr->pDiagCfg[BMI_DIAG_DEV_STAT], READ_2BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE1; // Re-run Step1
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                    }
                }
                else
                {
                    pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sACompDiag = (uint8)DIAG_STAT_ABORT;
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ABORT;
                }
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE1:
        {
            if(BMI_DIAG_DEV_STAT == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == (pData[0u] & (BQ7971X_DEV_STAT1_DIAG_ANA_RUN_MSK)))
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    /* Step5: Checkif Diagnostics Completed */
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                           COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_STAT1_OFFSET, COMIF_LOCK),
                                           &pDiagMgr->pDiagCfg[BMI_DIAG_DIAG_STAT], READ_2BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE3;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                    }
                }
                else
                {
                    pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sACompDiag = (uint8)DIAG_STAT_RUN_ERROR;
                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RETRY_ERROR;
                    pDiagReq->uDiagRetry++;
                    if(pDiagReq->uDiagRetry < BQ7971X_DIAG_MAX_RETRY)
                    {
                        /* Step3: Re-check if diagnostic is running. */
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                               COMIF_PRIO_EXCL_CELL(BQ7971X_DEV_STAT1_OFFSET, COMIF_LOCK),
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_DEV_STAT], READ_2BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                    else
                    {
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                               COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_ADC_MISC_OFFSET, COMIF_LOCK),
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_FAULT_ADC_MISC], READ_1BYTE);
                        if((uint8)BMI_OK == uRet)
                        {
                            pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE2;
                            pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                        }
                    }
                }
            }
            break;
        }
        case DIAG_STEP_READY:
        {
            pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sACompDiag = (uint8)DIAG_STAT_PENDING;

            /* Step 1: Pause Cell balancing if running */
            uRet = Bq7971x_PauseBalCtrl(pDiagMgr->pBmcMgr, COMIF_PRIO_EXCL_CELL(BQ7971X_BAL_CTRL2_OFFSET, COMIF_LOCK),
                                        BQ7971X_CB_DIAG_PAUSE, BQ7971X_CB_DIAG_CTRL);
            if((uint8)BMI_OK == uRet)
            {
                /* Step 1.1: Done part of Startup Init Configure analog compare filter and set VCELL diagnostic threshold */
                /* Step 2: Select Both cell voltage measurement and GPIO measurement analog path diagnostic
                    and Enable the device ADC digital path diagnostic. */
                uCmd = (BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_MSK | BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_MODE_BOTH |
                       BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_GO_MSK);

                uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,
                                        COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_ADC_CTRL3_OFFSET, COMIF_LOCK),
                                        &uCmd, WRITE_1BYTE);
                if((uint8)BMI_OK == uRet)
                {
                    /* Step3: Make sure the diagnostic is running. */
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                           COMIF_PRIO_EXCL_CELL(BQ7971X_DEV_STAT1_OFFSET, COMIF_LOCK),
                                           &pDiagMgr->pDiagCfg[BMI_DIAG_DEV_STAT], READ_2BYTE);

                    if((uint8)BMI_OK == uRet)
                    {
                        /* Step3.1: Wait for the comparison to be done. */
                        (void)Comif_SetDelay(pDiagMgr->pComifCtx, COMIF_PRIO_EXCL_CELL(BQ7971X_ACOMP_DONE_DELAY, COMIF_LOCK));

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
        pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sACompDiag = (uint8)DIAG_STAT_EXEC_ERROR;

        uCmd = (BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_MSK | BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_GO_MSK);
        (void)Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_ADC_CTRL3_OFFSET, COMIF_UNLOCK),
                         &uCmd, WRITE_1BYTE);
        (void)Bq7971x_UnpauseBalCtrl(pDiagMgr->pBmcMgr, COMIF_PRIO_EXCL_CELL(BQ7971X_BAL_CTRL2_OFFSET, COMIF_UNLOCK),
                               BQ7971X_CB_DIAG_UNPAUSE, BQ7971X_CB_APP_CTRL);

    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_VCELLGPIO_ACOMP, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uint8 DcompDiag(const Bq7971x_DiagType *, const ServiceCfgType *, Bq7971x_DiagReqType *)
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

STATIC FUNC(uint8, bq7971x_diag_CODE) DcompDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx;
    uint8 uRet;
    uint8 uLoop;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_FDTI_DCOMP_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDCompDiag = (uint8)DIAG_STAT_NOERROR;
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_DCOMP = ((pData[0u] << 16u) |
                                                                              (pData[1u] << 8u) | (pData[2u] << 0u));
                    if(pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_DCOMP != 0u)
                    {
                        pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDCompDiag = DIAG_STAT_ERROR;
                    }
                }
                pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                uRet = (uint8)BMI_OK;
            }
            break;
        }
        case (uint8)DIAG_STEP_STATE1:
        {
            if(BMI_DIAG_DEV_DIAG_STAT == pSvcCfg->uSubId)
            {
                uRet = (uint8)BMI_OK;
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if((0u == (pData[3u] & BQ7971X_DEV_STAT2_DIAG_DIG_RUN_MSK)) ||
                       (0u == (pData[1u] & BQ7971X_DIAG_STAT2_DRDY_DIG_MSK)))
                    {
                        uRet = (uint8)BMI_NOT_OK;
                    }
                }

                if((uint8)BMI_OK == uRet)
                {
                    /* Step3: Read DIG Fault Value */
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_FAULT_ADC_DIG1_OFFSET,
                                           &pDiagMgr->pDiagCfg[BMI_SM_FDTI_DCOMP_DIAG], READ_3BYTE);
                    if((uint8)BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = (uint8)DIAG_STEP_FINISHED;
                        pDiagReq->uDiagStat = (uint8)BMI_DIAG_RUN;
                    }
                }
                else
                {
                    pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDCompDiag = (uint8)DIAG_STAT_RUN_ERROR;

                    /* Step2: DRDY check Retry delay */
                    (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_DCOMPINJ_RETRY_DELAY);

                    pDiagReq->uDiagStat = (uint8)BMI_DIAG_RETRY_ERROR;
                    pDiagReq->uDiagRetry++;
                    if(pDiagReq->uDiagRetry < BQ7971X_DIAG_MAX_RETRY)
                    {
                        /* Step2: DRDY check Retry */
                        uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_STAT1_OFFSET,
                                               &pDiagMgr->pDiagCfg[BMI_DIAG_DEV_DIAG_STAT], READ_5BYTE);
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
            pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDCompDiag = (uint8)DIAG_STAT_PENDING;

            /* Step1: Check if DIG Diagnostics is running and DRDY*/
            uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIAG_STAT1_OFFSET,
                                   &pDiagMgr->pDiagCfg[BMI_DIAG_DEV_DIAG_STAT], READ_5BYTE);
            if((uint8)BMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_STEP_STATE1;
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
        pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDCompDiag = (uint8)DIAG_STAT_EXEC_ERROR;
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_DCOMP_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uint8 FaultSummary(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of VC Open Detection State Machine
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
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

STATIC FUNC(uint8, bq7971x_diag_CODE) FaultSummary(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                   Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;
    uint8 uDevIdx;
    uint8 uLoop;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_FDTI_FAULT_SUMMARY == pSvcCfg->uSubId)
            {
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.zSM_FAULT_SUMMARY = (uint8) pData[0u];
                }

                pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                uRet = (uint8)BMI_OK;
            }
            break;
        }
        case (uint8)DIAG_STEP_READY:
        {
            uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_FAULT_SUMMARY_OFFSET,
                                   &pDiagMgr->pDiagCfg[BMI_SM_FDTI_FAULT_SUMMARY], READ_1BYTE);
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

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_FAULT_SUMMARY, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}
/**********************************************************************************************************************
 * uint8 DieTempDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of VC Open Detection State Machine
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
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

STATIC FUNC(uint8, bq7971x_diag_CODE) DieTempDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                  Bq7971x_DiagReqType *pDiagReq)
{
    DiagResultType *pResult;
    uint8 *pData;
    uint16 wTempDiff;
    uint8 uRet;
    uint8 uDevIdx;
    uint8 uLoop;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if((uint8)BMI_SM_FDTI_DIETMEP_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDieTemp = (uint8)DIAG_STAT_NOERROR;
                pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDieTempDiff = (uint8)DIAG_STAT_NOERROR;
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    uRet = (uint8)BMI_OK;
                    pResult = &pDiagMgr->pDiagResult[uDevIdx];

                    pResult->zFdtiResult.zSM_DIE_TEMP1 = DIAGNOERROR;
                    pResult->zFdtiResult.zSM_DIE_TEMP2 = DIAGNOERROR;
                    pResult->zFdtiResult.zSM_DIE_TEMP_DIFF = DIAGNOERROR;

                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    pResult->zFdtiResult.zDieTemp1 = (uint16) ((pData[0u] << 8) | (pData[1u]));
                    if((BQ7971X_INVALDTEMPVALUE == pResult->zFdtiResult.zDieTemp1) ||
                       ((pResult->zFdtiResult.zDieTemp1 > BQ7971X_MAXDIETEMPPLAUVALUE) &&
                        (pResult->zFdtiResult.zDieTemp1 < BQ7971X_MINDIETEMPPLAUVALUE)))
                    {
                        pResult->zFdtiResult.zSM_DIE_TEMP1 = DIAGERROR;
                        uRet = (uint8)BMI_NOT_OK;
                    }

                    pResult->zFdtiResult.zDieTemp2 = (uint16) ((pData[2u] << 8) | (pData[3u]));
                    if((BQ7971X_INVALDTEMPVALUE == pResult->zFdtiResult.zDieTemp2) ||
                       ((pResult->zFdtiResult.zDieTemp2 > BQ7971X_MAXDIETEMPPLAUVALUE) &&
                       (pResult->zFdtiResult.zDieTemp2 < BQ7971X_MINDIETEMPPLAUVALUE)))
                    {
                        pResult->zFdtiResult.zSM_DIE_TEMP2 = DIAGERROR;
                        uRet = (uint8)BMI_NOT_OK;
                    }

                    if((uint8)BMI_OK == uRet)
                    {
                        wTempDiff = DiagAbsDiffTemp(pResult->zFdtiResult.zDieTemp1, pResult->zFdtiResult.zDieTemp2);
                        if(wTempDiff > BQ7971X_TEMPDIFFPLAUVALUE)
                        {
                            pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDieTempDiff = (uint8)DIAG_STAT_ERROR;
                            pResult->zFdtiResult.zSM_DIE_TEMP_DIFF = DIAGERROR;
                        }
                        else
                        {
                            if(pResult->zFdtiResult.zDieTempPrev1 == pResult->zFdtiResult.zDieTemp1)
                            {
                                pResult->zFdtiResult.zDieTemp1Stuck++;
                                if(pResult->zFdtiResult.zDieTemp1Stuck > BQ7971X_DIETEMPSTUCKVALUE)
                                {
                                    pResult->zFdtiResult.zSM_DIE_TEMP1 = DIAGERROR;
                                    pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDieTemp = (uint8)DIAG_STAT_ERROR;
                                }
                            }
                            else
                            {
                                pResult->zFdtiResult.zDieTemp1Stuck = 0;
                            }

                            if(pResult->zFdtiResult.zDieTempPrev2 == pResult->zFdtiResult.zDieTemp2)
                            {
                                pResult->zFdtiResult.zDieTemp2Stuck++;
                                if(pResult->zFdtiResult.zDieTemp2Stuck > BQ7971X_DIETEMPSTUCKVALUE)
                                {
                                    pResult->zFdtiResult.zSM_DIE_TEMP2 = (uint8)DIAGERROR;
                                    pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDieTemp = (uint8)DIAG_STAT_ERROR;
                                }
                            }
                            else
                            {
                                pResult->zFdtiResult.zDieTemp2Stuck = 0u;
                            }
                        }
                    }

                    pResult->zFdtiResult.zDieTempPrev1 = pResult->zFdtiResult.zDieTemp1;
                    pResult->zFdtiResult.zDieTempPrev2 = pResult->zFdtiResult.zDieTemp2;
                }
                pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
                uRet = (uint8)BMI_OK;
            }
            break;
        }
        case (uint8)DIAG_STEP_READY:
        {
            pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDieTemp = (uint8)DIAG_STAT_PENDING;
            pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDieTempDiff =(uint8) DIAG_STAT_PENDING;

            uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, BQ7971X_DIETEMP1_HI_OFFSET,
                                   &pDiagMgr->pDiagCfg[BMI_SM_FDTI_DIETMEP_DIAG], READ_4BYTE);
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
        pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDieTemp = (uint8)DIAG_STAT_EXEC_ERROR;
        pDiagMgr->pDiagStatus->zFdtiStat.zFdtiOut.sDieTempDiff = (uint8)DIAG_STAT_EXEC_ERROR;
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_DIE_TEMP, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uint8 BQ7971X_GetDiagD1(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of VC Open Detection State Machine
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
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


STATIC FUNC(uint8, bq7971x_diag_CODE) BQ7971X_GetDiagD1(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq,BQ7971X_DIAG_D1_SEL_Enum_Type uDiagD1Sel)
{
    uint8 *pData;
    uint8 uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;
    uint8 uDevIdx;
    uint8 uCmd;
    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;

    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
            if(BMI_SM_GET_DIAG_D1 == pSvcCfg->uSubId)
            {
                 for (uDevIdx = 0u; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if ((pData[1] & BQ7971X_ADC_DATA_RDY_DRDY_DIAG_D1D2_MSK) == 0u)
                    {
                        pDiagMgr->pDiagResult->zFdtiResult.Comb.SM_DRDY_DIAGD1D2 = DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zFdtiResult.Comb.SM_DRDY_DIAGD1D2 = DIAGNOERROR;
                    }
               }
                uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_D1_HI_OFFSET, COMIF_LOCK) ,
                                           &pDiagMgr->pDiagCfg[BMI_SM_GET_DIAG_D1], READ_2BYTE);
               if (uRet == (uint8)BMI_OK)
               {
                   pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
               }
            }
            break;
        }
        case (uint8)DIAG_STEP_READY:
        {
            uCmd = (((uint8)uDiagD1Sel << BQ7971X_ADC_CTRL4_DIAG_D1D2_SEL_POS) | BQ7971X_ADC_CTRL4_DIAG_MEAS_GO_MSK);

             uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,
                                        COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_CTRL4_OFFSET, COMIF_UNLOCK),
                                        &uCmd, WRITE_1BYTE);
            uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_DIAG_D1D2_DELAY);
            uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_DATA_RDY_OFFSET, COMIF_LOCK) ,
                                           &pDiagMgr->pDiagCfg[BMI_SM_GET_DIAG_D1], READ_1BYTE);
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

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_GET_DIAG_D1, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uint8 BQ7971X_GetDiagD2(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of VC Open Detection State Machine
 *
 *  \param[in]      pDiagMgr - Diagnostic context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
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

STATIC FUNC(uint8, bq7971x_diag_CODE) BQ7971X_GetDiagD2(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq,BQ7971X_DIAG_D2_SEL_Enum_Type uDiagD2Sel)
{
    uint8 *pData;
    uint8 uRet = (uint8)BMI_BMC_DIAG_EXEC_ERROR;
    uint8 uDevIdx;
    uint8 uCmd;

    pDiagReq->uDiagStat = (uint8)BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_STEP_FINISHED:
        {
                 for (uDevIdx = 0u; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if ((pData[1] & BQ7971X_ADC_DATA_RDY_DRDY_DIAG_D1D2_MSK) == 0u)
                    {
                        pDiagMgr->pDiagResult->zFdtiResult.Comb.SM_DRDY_DIAGD1D2 = DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zFdtiResult.Comb.SM_DRDY_DIAGD1D2 = DIAGNOERROR;
                    }
               }
                uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_D2_HI_OFFSET, COMIF_LOCK) ,
                                           &pDiagMgr->pDiagCfg[BMI_SM_GET_DIAG_D2], READ_2BYTE);
               if (uRet == (uint8)BMI_OK)
               {
                   pDiagReq->uDiagStat = (uint8)BMI_DIAG_COMPLETE;
               }

            break;
        }
        case (uint8)DIAG_STEP_READY:
        {
            uCmd = (((uint8)uDiagD2Sel << BQ7971X_ADC_CTRL4_DIAG_D1D2_SEL_POS) | BQ7971X_ADC_CTRL4_DIAG_MEAS_GO_MSK);

             uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,
                                        COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_CTRL4_OFFSET, COMIF_UNLOCK),
                                        &uCmd, WRITE_1BYTE);
            uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_DIAG_D1D2_DELAY);
            uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_DATA_RDY_OFFSET, COMIF_LOCK) ,
                                           &pDiagMgr->pDiagCfg[BMI_SM_GET_DIAG_D2], READ_1BYTE);
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

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_GET_DIAG_D2, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}
/*********************************************************************************************************************
 *  External Functions Definition
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagFdti(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
 *                                                   Bq7971x_DiagReqDataType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief          FDI Diagnostics request handler state machine
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pSvcReqCfg - Diagnostic Request Service
 *  \param[in]      pDiagReq - Current Diag Request
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         uint8
 *  \retval
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagFdti(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
                                               Bq7971x_DiagReqType *pDiagReq, uint8 uChannel)
{
    uint8 uRet;

    switch(pDiagReq->uDiagReqId)
    {
        case (uint8)BMI_SM_FDTI_DIETMEP_DIAG:
        {
            uRet = DieTempDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)BMI_SM_FDTI_FAULT_SUMMARY:
        {
            uRet = FaultSummary(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)BMI_SM_FDTI_DCOMP_DIAG:
        {
            uRet = DcompDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)BMI_SM_FDTI_VCELLGPIO_ACOMP:
        {
            uRet = AcompDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)BMI_SM_FDTI_VCCB_SHRT:
        {
            uRet = VCCBShortDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)BMI_SM_FDTI_VC_OPN_DET:
        {
            uRet = VCellOpenDiag(pDiagMgr,pDiagReq);
            break;
        }
        case (uint8)BMI_SM_FDTI_GPIO_ADJSHRT:
        {
            uRet = GpioAdjShrtDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)BMI_SM_FDTI_GPIO_OPNWR:
        {
            uRet = GpioOpnWireDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)BMI_SM_FDTI_GPIO_DIAG:
        {
            uRet = GpioDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)BMI_SM_GET_DIAG_D1:
        {
            uRet = BQ7971X_GetDiagD1(pDiagMgr, pSvcReqCfg, pDiagReq,BQ7971X_DIAG_D1_NO_SEL);
            break;
        }
        case (uint8)BMI_SM_GET_DIAG_D2:
        {
            uRet = BQ7971X_GetDiagD2(pDiagMgr, pSvcReqCfg, pDiagReq,BQ7971X_DIAG_D2_NO_SEL);
            break;
        }
        case (uint8)BMI_SM_FDTI_CYCLE_INIT:
        {
            uRet = DiagFdtiStatus(pDiagMgr, pSvcReqCfg, pDiagReq);
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

#define BQ7971X_DIAGFUNCS_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * End of File: bq7971x_diagfdti.c
 *********************************************************************************************************************/
