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
 *  File:       bq7971x_diagmpfdi_2.c
 *  Project:    TIBMS
 *  Module:     DIAG
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for Diagnostic MPFDI functionalities
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  Ahmed Ashraf
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       24Dec2023    Ahmed Ashraf                  0000000000000    Initial version
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
#include "bq7971x.h"


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

#define DIAG_DX_BGX_DAC_MASK                            (0x4)
#define DIAG_D1_OV_UV_CBDONE_DAC_MASK                   (0x2)
#define DIAG_D2_OT_UT_OTCB_DAC_MASK                     (0x2)


#define BQ7971X_FIR_OTP_UNLOCKCODE           (0xBCu)
#define BQ7971X_SEC_OTP_UNLOCKCODE           (0x6Fu)
#define BQ7971X_OTP_UNLOCK_STATCHECK         (0xEEu)
#define BQ7971X_OTP_DONE_DELAY               (200u)
#define BQ7971X_GPIO_CONF_RATIO              (9u)
#define BQ7971X_GPIOOPEN_DET_DELAY           (6u)
#define BQ7971X_OVUVSC_DONE_DELAY            (2u)
#define BQ7971X_OTUTRR_DONE_DELAY            (5u)


static uint8  BQ7971X_uFault_Ov[BQ7971X_IFACES + 1u][3u];                  /* 0x540 */
static uint8  BQ7971X_uFault_Uv[BQ7971X_IFACES + 1u][3u];                  /* 0x543 */
static uint8  BQ7971X_uFault_Ot[BQ7971X_IFACES + 1u][2u];                  /* 0x547 */
static uint8  BQ7971X_uFault_Ut[BQ7971X_IFACES + 1u][2u];                  /* 0x54A */
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


/*********************************************************************************************************************
 *  External Functions Definition
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * uint8 OTPProgram(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
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

STATIC FUNC(uint8, bq7971x_diag_CODE) OTPProgram(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                 Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx;
    uint8 uCmd;
    uint8 uRet;
    uint8 uLoop;
     
    pDiagReq->uDiagStat = BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
            if(BMI_SM_MPFDI_OTP_PROGRAM == pSvcCfg->uSubId)
            {  
               uLoop = pDiagMgr->sDevId;
               for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (((pData[0]  & BQ7971X_OTP_STAT_DONE_MSK)) == 0u)
                    {
                        uRet = (uint8) BMI_NOT_OK;
                    }
                    pData[1] = pData[0]  >> BQ7971X_OTP_STAT_PROGERR_POS ;
                    if (pData[0] != 0u)
                    {
                        uRet = (uint8) BMI_NOT_OK;
                    }
               }
               /* Step7: Issue a digital reset to reload the registers with the updated OTP values */
                uCmd = (pDiagMgr->pBmcMgr->eCommDir  | BQ7971X_CONTROL1_SOFT_RESET_MSK);
                uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) FALSE,
                                        COMIF_PRIO_EXCL_CELL(BQ7971X_CONTROL1_OFFSET, COMIF_UNLOCK),
                                        &uCmd, WRITE_1BYTE);
                uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_SOFT_RESET_DELAY);

                uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) FALSE,
                                                        COMIF_PRIO_EXCL_CELL(BQ7971X_CONTROL1_OFFSET, COMIF_UNLOCK),
                                                        &uCmd, WRITE_1BYTE);
                uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_SOFT_RESET_DELAY);

               // uRet = Bq7971x_StartupInit(pDiagMgr, pSvcCfg);

                if((uint8) BMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
                }
            }
            break;
        }
        case DIAG_STEP_STATE1:
        {
            if(BMI_SM_MPFDI_OTP_PROGRAM == pSvcCfg->uSubId)
            {
                /* Step3: Check and ensure that [LOADED], [UV_OVOK], [TRY] bits are all clear of error all are 0's */
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (((pData[0]  & BQ7971X_OTP_STAT_UNLOCK_MSK) == 0u))
                    {
                        pDiagMgr->pDiagResult->zFdtiResult.Comb.SM_OTP_UNLCK = DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zFdtiResult.Comb.SM_OTP_UNLCK = DIAGNOERROR;
                    }
                    if (((pData[0]  & BQ7971X_OTP_UNLOCK_STATCHECK)) != 0u)
                    {
                        uRet = (uint8) BMI_NOT_OK;
                    }
               }
               /* Step4: Start the OTP programming */
                uCmd = BQ7971X_CONTROL2_PROG_GO_MSK;
                uRet = Comif_StackWrite(pDiagMgr->pComifCtx,(uint8) TRUE,
                                        COMIF_PRIO_EXCL_CELL(BQ7971X_CONTROL2_OFFSET, COMIF_UNLOCK),
                                        &uCmd, WRITE_1BYTE);
                /* Step5: Start the OTP programming */
                uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_OTP_DONE_DELAY);
                /* Step6: Check to ensure there is no error during OTP programming */
                uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_OTP_STAT_OFFSET, COMIF_LOCK) ,
                                           &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_OTP_PROGRAM], READ_1BYTE);

                    if((uint8) BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = DIAG_STEP_FINISHED;
                        pDiagReq->uDiagStat = BMI_DIAG_RUN;
                    }
            }
            else
            {
               uRet = (uint8) BMI_NOT_OK;
            }

            break;
        }
        case DIAG_STEP_READY:
        {
            /* Step1: Unlock the OTP programming */
            uCmd = BQ7971X_FIR_OTP_UNLOCKCODE;
            uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,
                                        COMIF_PRIO_EXCL_CELL(BQ7971X_OTP_PROG_UNLOCK1_OFFSET, COMIF_UNLOCK),
                                        &uCmd, WRITE_1BYTE);
            uCmd = BQ7971X_SEC_OTP_UNLOCKCODE;
            uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,
                                        COMIF_PRIO_EXCL_CELL(BQ7971X_OTP_PROG_UNLOCK2_OFFSET, COMIF_UNLOCK),
                                        &uCmd, WRITE_1BYTE);
             /* Step2: Read to confirm OTP_STAT[UNLOCK] = 1 */
             uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_OTP_STAT_OFFSET, COMIF_LOCK) ,
                                           &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_OTP_PROGRAM], READ_1BYTE);

                if((uint8) BMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = DIAG_STEP_STATE1;
                    pDiagReq->uDiagStat = BMI_DIAG_RUN;
                }

            break;
        }
        default:
        {
            break;
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_OTP_PROGRAM, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}



/**********************************************************************************************************************
 * uint8 OtUtSCDiag(const Bq7971x_DiagType *, const ServiceCfgType *, Bq7971x_DiagReqType *, uint8)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of OV UV Diagnostic
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 *  \param[in]      uUTOTReqCmd - Command
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

STATIC FUNC(uint8, bq7971x_diag_CODE) OtUtSCDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                  Bq7971x_DiagReqType *pDiagReq,uint8 uChannel)
{
    uint8 *pData;
    uint8 uCmd;
    uint8 uDevIdx;
    uint8 uRet= BMI_OK;
    uint8 uLoop;

    uint8   uGpioConf[BQ7971X_GPIO_CONF_REG_NUM] = {BQ7971X_GPIO_CONF_RATIO, BQ7971X_GPIO_CONF_RATIO,
                    BQ7971X_GPIO_CONF_RATIO, BQ7971X_GPIO_CONF_RATIO, BQ7971X_GPIO_CONF_RATIO, BQ7971X_GPIO_CONF_RATIO};


    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
            if(BMI_SM_MPFDI_OTUT_SC_DIAG == pSvcCfg->uSubId)
            { 
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    pData [0] = (uint8) pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_OT_SC;
                    pData [0] = (uint8) pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_UT_SC;
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_OT_SC = (((uint16)BQ7971X_uFault_Ot[uDevIdx][0u]) << 8u) | ((uint16)BQ7971X_uFault_Ot[uDevIdx][1u]);
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_UT_SC  = (((uint16)BQ7971X_uFault_Ut[uDevIdx][0u]) << 8u) | ((uint16)BQ7971X_uFault_Ut[uDevIdx][1u]);
                }
                /* Step9: Do not run the OTUT comparator. */
                uCmd = BQ7971X_OTUT_CTRL2_OTUT_GO_MSK;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_OTUT_CTRL2_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                /* Step10: Reset the FAULT_UT* and FAULT_OT* registers. */
                uCmd = BQ7971X_FAULT_RST1_RST_UT_MSK | BQ7971X_FAULT_RST1_RST_OT_MSK;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);

                 /* Step11: Resume GPIO configure. */
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {   
                    uCmd = pDiagMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uGpio_Conf[uDevIdx];
                    uRet = Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx+1u), COMIF_PRIO_EXCL_CELL(BQ7971X_GPIO_CONF1_OFFSET, COMIF_LOCK),
                                            &uCmd, BQ7971X_GPIO_CONF_REG_NUM);
                }
                 uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_GPIOOPEN_DET_DELAY);

                if((uint8) BMI_OK == uRet)
                  {
                       pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
                  }
            }
            break;
        }
        case DIAG_STEP_STATE1:
        {
            if(BMI_SM_MPFDI_OTUT_SC_DIAG == pSvcCfg->uSubId)
            {

                /* Step6: Select UT threshold, configure a particular single channel. */
                uCmd = (uChannel & BQ7971X_OTUT_CTRL1_OTUT_LOCK_MSK) | BQ7971X_OTUT_CTRL1_UT_THR;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_OTUT_CTRL1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                /* Step7: Lock the OTUT comparator to a single channel, enable OTUT, wait for completion. */
                uCmd = BQ7971X_OTUT_CTRL2_OTUT_THR_LOCK | BQ7971X_OTUT_CTRL2_OTUT_GO_MSK;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_OTUT_CTRL2_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                /* Step8: Check UT detection single channel result. */
                 uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_UT1_OFFSET, COMIF_LOCK) ,
                                               &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_OTUT_SC_DIAG], READ_2BYTE);

                   if((uint8) BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = DIAG_STEP_FINISHED;
                        pDiagReq->uDiagStat = BMI_DIAG_RUN;
                    }

            }
            break;
        }
        case DIAG_STEP_READY:
        {   
            uLoop = pDiagMgr->sDevId;
            for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {
                uGpioConf[0u] = (pDiagMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uGpio_Conf[0] & BQ7971X_GPIO_CONF1_SPI_EN_MSK) | BQ7971X_GPIO_CONF_RATIO;
                uRet = Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx+1u), COMIF_PRIO_EXCL_CELL(BQ7971X_GPIO_CONF1_OFFSET, COMIF_LOCK),
                                            uGpioConf, BQ7971X_GPIO_CONF_REG_NUM);
            }
            uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_GPIOOPEN_DET_DELAY);
            /* Step2: Reset the FAULT_UT* and FAULT_OT* registers. */
            uCmd = BQ7971X_FAULT_RST1_RST_UT_MSK | BQ7971X_FAULT_RST1_RST_OT_MSK;
            uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
            /* Step3: Select OT threshold, configure a particular single channel. */
            uCmd = uChannel & BQ7971X_OTUT_CTRL1_OTUT_LOCK_MSK;
            uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_OTUT_CTRL1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
            /* Step4: Lock the OTUT comparator to a single channel, enable OTUT, wait for completion. */
            uCmd = BQ7971X_OTUT_CTRL2_OTUT_THR_LOCK | BQ7971X_OTUT_CTRL2_OTUT_GO_MSK;
            uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_OTUT_CTRL2_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
            /* Step5: Check OT detection single channel result. */
             uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_OTP_OFFSET, COMIF_LOCK) ,
                                               &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_OTUT_SC_DIAG], READ_1BYTE);
                    if((uint8) BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = DIAG_STEP_STATE1;
                        pDiagReq->uDiagStat = BMI_DIAG_RUN;
                    }

            break;
        }
        default:
        {
            break;
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_DIAG_OTUT_SC, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}




/**********************************************************************************************************************
 * uint8 OvUvSCDiag(const Bq7971x_DiagType *, const ServiceCfgType *, Bq7971x_DiagReqType *, uint8)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of OV UV Diagnostic
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 *  \param[in]      uUTOTReqCmd - Command
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

STATIC FUNC(uint8, bq7971x_diag_CODE) OvUvSCDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                  Bq7971x_DiagReqType *pDiagReq,    uint8 uChannel)
{
    uint8 *pData;

    uint8 uCmd;
    uint8 uDevIdx;
    uint8 uRet = (uint8) BMI_OK;
    uint8 uLoop;

    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
            if(BMI_SM_MPFDI_OVUV_SC_DIAG == pSvcCfg->uSubId)
            {
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    pData [0] = (uint8) pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_OV_SC;
                    pData [0] = (uint8) pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_UV_SC;
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_OV_SC =  (((uint32)BQ7971X_uFault_Ov[uDevIdx][0u]) << 16u)
                    | (((uint32)BQ7971X_uFault_Ov[uDevIdx ][1u]) << 8u) \
                    | ((uint32)BQ7971X_uFault_Ov[uDevIdx ][2u]);
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_UV_SC  =  (((uint32)BQ7971X_uFault_Uv[uDevIdx][0u]) << 16u)
                    | (((uint32)BQ7971X_uFault_Uv[uDevIdx ][1u]) << 8u)
                    | ((uint32)BQ7971X_uFault_Uv[uDevIdx ][2u]);
                }
                /* Step8: Do not run the OVUV comparator. */
                uCmd = BQ7971X_OVUV_CTRL2_OVUV_GO_MSK;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_OVUV_CTRL2_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                /* Step9: Reset the FAULT_UV* and FAULT_OV* registers. */
                uCmd = BQ7971X_FAULT_RST1_RST_UV_MSK | BQ7971X_FAULT_RST1_RST_OV_MSK;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);

                if((uint8) BMI_OK == uRet)
                  {
                       pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
                  }
            }
            break;
        }
        case DIAG_STEP_STATE1:
        {
            if(BMI_SM_MPFDI_OVUV_SC_DIAG == pSvcCfg->uSubId)
            {
                /* Step5: Select UV threshold, configure a particular single channel. */
                uCmd = (uChannel & BQ7971X_OVUV_CTRL1_OVUV_LOCK_MSK) | BQ7971X_OVUV_CTRL1_UV_THR;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_OVUV_CTRL1_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);

                /* Step6: Lock the OVUV comparator to a single channel, enable OVUV, wait for completion. */
                uCmd = BQ7971X_OVUV_CTRL2_OVUV_THR_LOCK | BQ7971X_OVUV_CTRL2_OVUV_GO_MSK;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_OVUV_CTRL2_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
                uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_OVUVSC_DONE_DELAY);
                /* Step7: Check UV detection single channel result. */
                 uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_UV1_OFFSET, COMIF_LOCK) ,
                                               &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_OVUV_SC_DIAG], READ_3BYTE);
                   if((uint8) BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = DIAG_STEP_FINISHED;
                        pDiagReq->uDiagStat = BMI_DIAG_RUN;
                    }

            }
            break;
        }
        case DIAG_STEP_READY:
        {

            /* Step1: Reset the FAULT_UV* and FAULT_OV* registers. */
            uCmd = BQ7971X_FAULT_RST1_RST_UV_MSK | BQ7971X_FAULT_RST1_RST_OV_MSK;
            uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST1_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
            /* Step2: Select OV threshold, configure a particular single channel. */
            uCmd = uChannel & BQ7971X_OVUV_CTRL1_OVUV_LOCK_MSK;
            uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_OVUV_CTRL1_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
            /* Step3: Lock the OVUV comparator to a single channel, enable OVUV, wait for completion. */
            uCmd = BQ7971X_OVUV_CTRL2_OVUV_THR_LOCK | BQ7971X_OVUV_CTRL2_OVUV_GO_MSK;
            uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_OVUV_CTRL2_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
            uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_OVUVSC_DONE_DELAY);
            /* Step4: Check OV detection single channel result. */
            uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_OV1_OFFSET, COMIF_LOCK) ,
                                               &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_OVUV_SC_DIAG], READ_3BYTE);


                    if((uint8) BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = DIAG_STEP_STATE1;
                        pDiagReq->uDiagStat = BMI_DIAG_RUN;
                    }


            break;
        }
        default:
        {
            break;
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_DIAG_OVUV_SC, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}



/**********************************************************************************************************************
 * uint8 OtUtRRDiag(const Bq7971x_DiagType *, const ServiceCfgType *, Bq7971x_DiagReqType *, uint8)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of OV UV Diagnostic
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 *  \param[in]      uUTOTReqCmd - Command
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

STATIC FUNC(uint8, bq7971x_diag_CODE) OtUtRRDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                  Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uCmd;
    uint8 uDevIdx;
    uint8 uRet= BMI_OK;
    uint8 uLoop; 

    uint8   uGpioConf[BQ7971X_GPIO_CONF_REG_NUM] = {BQ7971X_GPIO_CONF_RATIO, BQ7971X_GPIO_CONF_RATIO,
                    BQ7971X_GPIO_CONF_RATIO, BQ7971X_GPIO_CONF_RATIO, BQ7971X_GPIO_CONF_RATIO, BQ7971X_GPIO_CONF_RATIO};


    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
            if(BMI_SM_MPFDI_OTUT_RR_DIAG == pSvcCfg->uSubId)
            {
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    pData [0] = (uint8) pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_OT_RR;
                    pData [0] = (uint8) pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_UT_RR;
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_OT_RR = (((uint16)BQ7971X_uFault_Ot[uDevIdx][0u]) << 8u)
                                                                                | ((uint16)BQ7971X_uFault_Ot[uDevIdx][1u]);
                    pDiagMgr->pDiagResult[uDevIdx].zMpfdtiResult.zSM_UT_RR  = (((uint16)BQ7971X_uFault_Ut[uDevIdx ][0u]) << 8u)
                                                                                | ((uint16)BQ7971X_uFault_Ut[uDevIdx ][1u]);
                }
                  /* Step5: Do not run the OTUT comparator. */
                uCmd = BQ7971X_OTUT_CTRL2_OTUT_GO_MSK;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_OTUT_CTRL2_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);

                /* Step6: Reset the FAULT_UT* and FAULT_OT* registers. */
                uCmd = BQ7971X_FAULT_RST1_RST_UT_MSK | BQ7971X_FAULT_RST1_RST_OT_MSK;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);

                 /* Step7: Resume GPIO configure. */

                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    uCmd = pDiagMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uGpio_Conf[uDevIdx];
                    uRet = Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx+1u), COMIF_PRIO_EXCL_CELL(BQ7971X_GPIO_CONF1_OFFSET, COMIF_LOCK),
                                            &uCmd, BQ7971X_GPIO_CONF_REG_NUM);
                }
                 uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_GPIOOPEN_DET_DELAY);

                if((uint8) BMI_OK == uRet)
                  {
                       pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
                  }
            }
            break;
        }
        case DIAG_STEP_STATE1:
        {
            if(BMI_SM_MPFDI_OTUT_RR_DIAG == pSvcCfg->uSubId)
            {

                 uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_UT1_OFFSET, COMIF_LOCK) ,
                                               &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_OTUT_RR_DIAG], READ_2BYTE);

                   if((uint8) BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = DIAG_STEP_FINISHED;
                        pDiagReq->uDiagStat = BMI_DIAG_RUN;
                    }

            }
            break;
        }
        case DIAG_STEP_READY:
        {
            uLoop = pDiagMgr->sDevId;
            for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {
                uGpioConf[0u] = (pDiagMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uGpio_Conf[0] & BQ7971X_GPIO_CONF1_SPI_EN_MSK) | BQ7971X_GPIO_CONF_RATIO;
                uRet = Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx+1u), COMIF_PRIO_EXCL_CELL(BQ7971X_GPIO_CONF1_OFFSET, COMIF_LOCK),
                                            uGpioConf, BQ7971X_GPIO_CONF_REG_NUM);
            }
            uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_GPIOOPEN_DET_DELAY);
            /* Step2: Reset the FAULT_UT* and FAULT_OT* registers. */
            uCmd = BQ7971X_FAULT_RST1_RST_UT_MSK | BQ7971X_FAULT_RST1_RST_OT_MSK;
            uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);

            /* Step3: Run the OTUT round robin with all the active GPIOs, enable OTUT, wait for completion. */
            uCmd = BQ7971X_OTUT_CTRL2_OTUT_RR | BQ7971X_OTUT_CTRL2_OTUT_GO_MSK;
            uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_OTUT_CTRL2_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
            uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_OTUTRR_DONE_DELAY);

            /* Step4: Check OTUT detection round robin result. */
             uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_OT1_OFFSET, COMIF_LOCK) ,
                                               &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_OTUT_RR_DIAG], READ_2BYTE);
                    if((uint8) BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = DIAG_STEP_STATE1;
                        pDiagReq->uDiagStat = BMI_DIAG_RUN;
                    }

            break;
        }
        default:
        {
            break;
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_DIAG_OTUT_RR, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/**********************************************************************************************************************
 * uint8 SleepFaultDiag(const Bq7971x_DiagType *, const ServiceCfgType *, Bq7971x_DiagReqType *, uint8)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of OV UV Diagnostic
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 *  \param[in]      uUTOTReqCmd - Command
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

STATIC FUNC(uint8, bq7971x_diag_CODE) SleepFaultDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                  Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx;
    uint8 uRet= BMI_OK;
    uint8 uLoop;


    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
            if(BMI_SM_MPFDI_SLEEP_FAULT_DIAG == pSvcCfg->uSubId)
            {   
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, (uDevIdx));
                    if(0u == (pData[0u]& BQ7971X_FAULT_COMM_HB_FAIL_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_SLEEP_HB = DIAGNOERROR;
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_SLEEP_FSTHB = DIAGNOERROR;

                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_SLEEP_HB = DIAGERROR;
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_SLEEP_FSTHB = DIAGERROR;
                    }
                }


                pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;

            }
            break;
        }

        case DIAG_STEP_READY:
        {
            if ((uint8)STD_HIGH == Dio_ReadChannel(BQ7971X_DIO_NFAULT_PIN))
            {
                pDiagMgr->pDiagResult[0].zFdtiResult.Comb.SM_SLEEP_FAULT = DIAGNOERROR;
            }
            else
            {
                pDiagMgr->pDiagResult[0].zFdtiResult.Comb.SM_SLEEP_FAULT = DIAGERROR;
            }
            uRet = Comif_StackRead(pDiagMgr->pComifCtx,  pDiagMgr->uNAFEs, COMIF_PRIO_GPIO(BQ7971X_FAULT_COMM_OFFSET),
                                           &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_SLEEP_FAULT_DIAG],READ_1BYTE);
            if((uint8) BMI_OK == uRet)
                    {
                        pDiagReq->uDiagStep = DIAG_STEP_FINISHED;
                        pDiagReq->uDiagStat = BMI_DIAG_RUN;
                    }
            break;
        }
        default:
        {
            break;
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_DIAG_SLEEP_FAULT, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/**********************************************************************************************************************
 * uint8 HWResetDiag(const Bq7971x_DiagType *, const ServiceCfgType *, Bq7971x_DiagReqType *, uint8)
 *********************************************************************************************************************/
/*! \brief          Diagnostics of OV UV Diagnostic
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pSvcCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 *  \param[in]      uUTOTReqCmd - Command
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

STATIC FUNC(uint8, bq7971x_diag_CODE) HWResetDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                  Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uRet= BMI_OK;
    uint16 xReg;
    uint8 BQ7971X_uHW_RST_Diag = 0 ;
    uint8 uCmd;
    uint8 uDevIdx;


    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
            if(BMI_SM_MPFDI_SLEEP_FAULT_DIAG == pSvcCfg->uSubId)
            {       
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs,0u);
                    if(NULL == (pData[0u]))
                    {
                        pDiagMgr->pDiagResult[0].zFdtiResult.Comb.SM_HW_RST  = DIAGERROR;

                    }
                    else
                    {
                        pDiagMgr->pDiagResult[0].zFdtiResult.Comb.SM_HW_RST = DIAGNOERROR;
                    }



                pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;

            }
            break;
        }

        case DIAG_STEP_READY:
        {
             if(BQ7971X_uHW_RST_Diag > 0u)
            {
                uRet = (uint8) BMI_OK ;
            }
            else
            {
                 BQ7971X_uHW_RST_Diag = 1u;
                 /************ HW reset **************/
                 {
                     /* perform hard reset of the stack device. */
                     uCmd = (pDiagMgr->pBmcMgr->eCommDir  | BQ7971X_CONTROL1_SEND_SD_HW_RST_MSK);
                     for (uDevIdx = 0u; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                     {
                         uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx+1u), COMIF_PRIO_EXCL_CELL(BQ7971X_CONTROL1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                         uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_SOFT_RESET_DELAY);
                     }
                     uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, 0u, COMIF_PRIO_EXCL_CELL(BQ7971X_CONTROL1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                     uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_SOFT_RESET_DELAY);
                     /* perform hard reset of the bridge device. */
                     uCmd = (pDiagMgr->pBmcMgr->eCommDir | BQ7971X_CONTROL1_GOTO_SHUTDOWN_MSK);
                     uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, 0u, COMIF_PRIO_EXCL_CELL(BQ7971X_CONTROL1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                     uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_HW_RESET_DELAY);
                 }

                 xReg = (pDiagMgr->pBmcMgr->eCommDir == DIR_COMH2L) ? BQ7971X_DIR0_ADDR_OFFSET : BQ7971X_DIR1_ADDR_OFFSET;
                uRet = Comif_StackRead(pDiagMgr->pComifCtx, 0u, COMIF_PRIO_GPIO(xReg),
                               &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_HW_RESET_DIAG],READ_1BYTE);
                if((uint8) BMI_OK == uRet)
                {
                        pDiagReq->uDiagStep = DIAG_STEP_FINISHED;
                        pDiagReq->uDiagStat = BMI_DIAG_RUN;
                }
            }
            break;
        }
        default:
        {
            break;
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_DIAG_HW_RESET, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}



/**********************************************************************************************************************
 * uint8 DevAddrDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
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
STATIC FUNC(uint8, bq7971x_diag_CODE) DevAddrDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uRet;
    uint8 uBufIndex = 1u;;
    uint16 xReg;


    pDiagReq->uDiagStat = BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
            if(BMI_SM_MPFDI_DEV_ADDR_DIAG == pSvcCfg->uSubId)
            {
                 /* Traverse all device */
                while(uBufIndex <= (pDiagMgr->uNAFEs))
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs,uBufIndex);
                    if(uBufIndex == pData[0])
                    {
                        pDiagMgr->pDiagResult[0].zFdtiResult.Comb.SM_DEV_ADDR = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[0].zFdtiResult.Comb.SM_DEV_ADDR = DIAGERROR;
                    }

                    uBufIndex++;
                }
                pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
                uRet = (uint8) BMI_OK;
            }
            break;
        }
        case DIAG_STEP_STATE1:
        {
            if(BMI_SM_MPFDI_DEV_ADDR_DIAG == pSvcCfg->uSubId)
            {
                xReg = (pDiagMgr->pBmcMgr->eCommDir == DIR_COMH2L) ? BQ7971X_DIR0_ADDR_OFFSET : BQ7971X_DIR1_ADDR_OFFSET;

                uRet = Comif_SingleRead(pDiagMgr->pComifCtx, 0u, COMIF_PRIO_GPIO(xReg),
                                              &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_DEV_ADDR_DIAG], READ_1BYTE);
               if((uint8) BMI_OK == uRet)
               {
                   pDiagReq->uDiagStep = DIAG_STEP_FINISHED;
                   pDiagReq->uDiagStat = BMI_DIAG_RUN;
               }


            }
            break;
        }
        case DIAG_STEP_READY:
        {
            xReg = (pDiagMgr->pBmcMgr->eCommDir == DIR_COMH2L) ? BQ7971X_DIR0_ADDR_OFFSET : BQ7971X_DIR1_ADDR_OFFSET;

            uRet = Comif_SingleRead(pDiagMgr->pComifCtx, 0u, COMIF_PRIO_GPIO(xReg),
                                     &pDiagMgr->pDiagCfg[BMI_SM_MPFDI_DEV_ADDR_DIAG], READ_1BYTE);

            if((uint8) BMI_OK == uRet)
            {
                pDiagReq->uDiagStep = DIAG_STEP_STATE1;
                pDiagReq->uDiagStat = BMI_DIAG_RUN;
            }
            break;
        }
        default:
        {
            break;
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_DIAG_DEV_ADDR, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/**********************************************************************************************************************
 *  FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagMpfdi_2(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
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

FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagMpfdi_2(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
                                                 Bq7971x_DiagReqType *pDiagReq, uint8 uChannel)
{
    uint8 uRet;

    switch(pDiagReq->uDiagReqId)
    {
        case BMI_SM_MPFDI_OTP_PROGRAM:
        {
           uRet = OTPProgram(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_MPFDI_OTUT_SC_DIAG:
        {
           uRet = OtUtSCDiag(pDiagMgr, pSvcReqCfg, pDiagReq,uChannel);
            break;
        }
        case BMI_SM_MPFDI_OVUV_SC_DIAG:
        {
           uRet = OvUvSCDiag(pDiagMgr, pSvcReqCfg, pDiagReq,  uChannel);
            break;
        }
        case BMI_SM_MPFDI_OTUT_RR_DIAG:
        {
           uRet = OtUtRRDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_MPFDI_SLEEP_FAULT_DIAG:
        {
           uRet = SleepFaultDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_MPFDI_HW_RESET_DIAG:
        {
           uRet = HWResetDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_MPFDI_DEV_ADDR_DIAG:
        {
           uRet = DevAddrDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_MPFDI_STARTUPINIT:
        {
            uRet = E_OK;
            break;
        }
        default:
        {
            uRet = BMI_BMC_DIAG_INVALID_CMD;
            break;
        }
    }

    return uRet;
}

#define BQ7971X_DIAGSTARTUP_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * End of File: bq7971x_diagmpfdi_2.c
 *********************************************************************************************************************/
