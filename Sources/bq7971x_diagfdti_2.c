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
 *  File:       bq7971x_diagfdti_2.c
 *  Project:    TIBMS
 *  Module:     DIAG
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for BMI DIAG interface
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  Ahmed Ashraf
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       24Dec2023    Ahmed Ashraf         0000000000000    Initial version
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
#include "Dio.h"

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

/** Major Software Config C Version number */
#define BQ7971X_DIAGFUNC_C_MAJOR_VERSION                    (0x01u)

/** Minor Software Config C Version number */
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

#define CB_EVEN_CELLS                                      (0u)
#define CB_ODD_CELLS                                       (1u)
#define DIAG_CELL_CB_TIME_DEFAULT                          (1u)


#define BQ7971X_MAXPLAUVALUE                               (48000u)
#define BQ7971X_MIDPLAUVALUE                               (25000u)
#define BQ7971X_MINPLAUVALUE                               (2000u)
#define BQ7971X_CBPLAU_CHECKTIMES                          (1u)
#define BQ7971X_BBMAXPLAUVALUE                             (4000u)    /* 400mV */
#define BQ7971X_BBMINPLAUVALUE                             (1500u)    /* 150mV */
#define BQ7971X_VCELLADC_DONE_DELAY                        (9u)
#define BQ7971X_GPIOADC_DONE_DELAY                         (6u)
#define BQ7971X_VCPLAU_CHECKTIMES                          (1u)
#define BQ7971X_BBPLAU_CHECKTIMES                          (1u)
#define BQ7971X_BB_PIN_LOC_RES                             (7u)
/*
static uint8  BQ7971X_uCBPlauChannel = 0;
uint8  BQ7971X_uPlauCheckTimesNum =  0;*/
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


 static uint8  BQ7971X_uPlauCheckTimesNum = 0u;
 static uint8  BQ7971X_uCBPlauChannel = 0u;

 static uint8  BQ7971X_uDebug_Uart[BQ7971X_IFACES + 1u];                    /* 0x781 */
 static uint8  BQ7971X_uDebug_Comh[BQ7971X_IFACES + 1u][3u];                /* 0x782 */
 static uint8  BQ7971X_uDebug_Coml[BQ7971X_IFACES + 1u][3u];                /* 0x785 */
 static uint8  BQ7971X_uFault_Summary[BQ7971X_IFACES + 1u];                 /* 0x532 */
 static uint8  BQ7971X_uCust_Crc_Rslt[BQ7971X_IFACES + 1u][2u];             /* 0x50C */
/*********************************************************************************************************************
 *  External Functions Definition
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * uint8 DiagAbortDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
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

STATIC FUNC(uint8, bq7971x_diag_CODE) DiagAbortDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                    Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uRet = (uint8) BMI_BMC_DIAG_EXEC_ERROR;
    uint8 uDevIdx;


    pDiagReq->uDiagStat = BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
                 for (uDevIdx = pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if ((pData[0] & BQ7971X_FAULT_ADC_MISC_DIAG_ANA_ABORT_MSK) == 0u)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_DIAG_ABORT = DIAGERROR;
                        uRet = BMI_NOT_OK;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_DIAG_ABORT = DIAGNOERROR;
                        uRet = BMI_OK;
                    }
               }

               if (uRet == BMI_OK)
               {
                   pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
               }

            break;
        }
        case DIAG_STEP_READY:
        {
            uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_ADC_MISC_OFFSET, COMIF_LOCK) ,
                                           &pDiagMgr->pDiagCfg[BMI_DIAG_ABORT_DIAG], READ_1BYTE);
            if(BMI_OK == uRet)
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

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_DIAG_ABORT, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/*********************************************************************************************************************/
/*!\brief This function is used to check about FAULT_ADC_MISC diagnosis.
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

STATIC FUNC(uint8, bq7971x_diag_CODE) FaultAdcMiscDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                    Bq7971x_DiagReqType *pDiagReq)
{
  uint8 *pData;
  uint8 uDevIdx;
  uint8 uCmd;
  uint8 uRet = BMI_OK;
  uint8 uLoop;
  pDiagReq->uDiagStat = BMI_DIAG_ERROR;



  switch(pDiagReq->uDiagStep)
  {
     case DIAG_STEP_FINISHED:
      {
          if(BMI_SM_FAULT_ADC_MISC == pSvcCfg->uSubId)
          {
           uLoop = uDevIdx = pDiagMgr->sDevId;
           for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
           {
               pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
               pData[0] &=BQ7971X_FAULT_ADC_MISC_ADC_PFAIL_MSK;
               if( 0u == pData[0])
               {
                   uRet = BMI_OK;
               }
               else
               {
                   uRet = BMI_NOT_OK;
               }
               pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
               pData[0] &=BQ7971X_FAULT_ADC_MISC_DIAG_MEAS_PFAIL_MSK;
               if( 0u ==  pData[0])
               {
                   uRet = BMI_OK;
               }
               else
               {
                   uRet = BMI_NOT_OK;
               }
               pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
               pData[0] &=BQ7971X_FAULT_ADC_MISC_DIAG_ANA_PFAIL_MSK;
               if( 0u == pData[0])
               {
                   uRet = BMI_OK;
               }
               else
               {
                   uRet = BMI_NOT_OK;
               }
           }
            uCmd = BQ7971X_FAULT_RST2_RST_ADC_MSK;
            uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST2_OFFSET, COMIF_UNLOCK) , &uCmd, WRITE_1BYTE);
          if(BMI_OK == uRet)
          {
               pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
          }
          }
          break;
      }
      case DIAG_STEP_READY:
      {    
           uLoop = uDevIdx = pDiagMgr->sDevId;
           for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
              {
                   if (pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_MAIN_PFAIL > 0u)
                   {
                       uCmd = (pDiagMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uAdc_Ctrl[1u] |BQ7971X_ADC_CTRL2_ADC_GO_MSK) ;
                       uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx+1u), COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_CTRL2_OFFSET, COMIF_LOCK),
                                            &uCmd, WRITE_1BYTE);
                   }
                   if (pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_DIAG_PFAIL > 0u)
                   {
                       uCmd = (pDiagMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uAdc_Ctrl[3u] |BQ7971X_ADC_CTRL4_DIAG_MEAS_GO_MSK) ;
                       uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx+1u), COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_CTRL4_OFFSET, COMIF_LOCK),
                                            &uCmd, WRITE_1BYTE);
                   }

                   if (pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_ANA_PFAIL > 0u)
                   {
                       uCmd = BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_MSK | BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_GO_MSK;
                       uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,
                                      COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_ADC_CTRL3_OFFSET, COMIF_LOCK),
                                      &uCmd, WRITE_1BYTE);
                   }
              }

          uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_ADC_MISC_OFFSET, COMIF_LOCK) ,
                                 &pDiagMgr->pDiagCfg[BMI_SM_FAULT_ADC_MISC], READ_1BYTE);
           if(BMI_OK == uRet)
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
  return Bmi_SetErrorDetails(BMI_SVCID_BMC_FAULT_ADC_MISC, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************/
/*! \brief Check the sending and receiving functions of SPI.
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

STATIC FUNC(uint8, bq7971x_diag_CODE) SPIFaultDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7971x_DiagReqType *pDiagReq)
{
   const uint8 uSPIData[3u] = {BQ7971X_SPI_TEST_CODE, BQ7971X_SPI_TEST_CODE, BQ7971X_SPI_TEST_CODE};
   uint8 *pData;
   uint8 uDevId;
   uint8 uRxIndex;
   uint8 uCmd;
   uint8 uRet = BMI_OK;
   uint8   uDiagResult = DIAGNOERROR;
   uint8 uLoop;
   pDiagReq->uDiagStat = BMI_DIAG_ERROR;



   switch(pDiagReq->uDiagStep)
   {
      case DIAG_STEP_FINISHED:
       {
           if(BMI_SM_SPI_FAULT_DIAG == pSvcCfg->uSubId)
           {
                uLoop = pDiagMgr->sDevId;
                for(uDevId = uLoop; uDevId < pDiagMgr->uNAFEs; uDevId++)
                {
                   for(uRxIndex = 0; uRxIndex < 3u; uRxIndex++)
                   {
                       pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, ((uDevId * 3u) +uRxIndex));
                       if(BQ7971X_SPI_TEST_CODE !=  pData[0u])
                       {
                           uDiagResult |= (uint8)(DIAGERROR << uRxIndex);
                       }
                       pDiagMgr->pDiagResult[uDevId].zFdtiResult.Comb.SPI_FAULT_DIAG = uDiagResult;
                   }
                }
               /* Step6: Resume SPI configuration. */
               uCmd = BQ7971X_DIAG_MISC_CTRL1_POR_VAL;
               uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,
                                       COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_MISC_CTRL1_OFFSET, COMIF_LOCK),
                                       &uCmd, WRITE_1BYTE);
                for(uDevId = 0; uDevId <= pDiagMgr->uNAFEs; uDevId++)
               {
                   uCmd = pDiagMgr->pBmicCfg->pBqRegsCfg[uDevId].zRegCtrl.uSpi_Conf;
                   uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,
                                       COMIF_PRIO_EXCL_CELL(BQ7971X_SPI_CONF_OFFSET, COMIF_LOCK),
                                       &uCmd, WRITE_1BYTE);

                   uCmd = (pDiagMgr->pBmicCfg->pBqRegsCfg[uDevId].zRegNVM.uGpio_Conf[0]) ;
                   uRet = Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevId+1u), COMIF_PRIO_EXCL_CELL(BQ7971X_SPI_EXE_OFFSET, COMIF_UNLOCK),
                                               &uCmd, WRITE_1BYTE);
               }

                  if(BMI_OK == uRet)
                  {
                       pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
                  }
           }

           break;
       }
       case DIAG_STEP_STATE1:
       {
           if(BMI_SM_SPI_FAULT_DIAG == pSvcCfg->uSubId)
           {
               for(uDevId = 1; uDevId <= pDiagMgr->uNAFEs; uDevId++)
               {
                   uCmd = BQ7971X_SPI_EXE_SS_CTRL_MSK & (BQ7971X_SPI_EXE_SS_CTRL_MSK | BQ7971X_SPI_EXE_SPI_GO_MSK);
                   uRet = Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevId+1u), COMIF_PRIO_EXCL_CELL(BQ7971X_SPI_EXE_OFFSET, COMIF_LOCK),
                                               &uCmd, WRITE_1BYTE);
               }
            /*   uCmd = pDiagMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uGpio_Conf[0];
               uRet = Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx+1),COMIF_PRIO_EXCL_CELL(BQ7971X_GPIO_CONF1_OFFSET, COMIF_LOCK) ,
                                             &pData[0], WRITE_1BYTE);*/

                /* Add read commands to call callback functions. */
               uRet = Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs , COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_MISC_CTRL1_OFFSET, COMIF_LOCK),
                                        &pDiagMgr->pDiagCfg[BMI_SM_SPI_FAULT_DIAG], READ_2BYTE);
               if(BMI_OK == uRet)
              {
                   pDiagReq->uDiagStep = DIAG_STEP_FINISHED;
                   pDiagReq->uDiagStat = BMI_DIAG_RUN;
              }
           }
           break;
       }

       case DIAG_STEP_READY:
       {
            for(uDevId = 0; uDevId < pDiagMgr->uNAFEs; uDevId++)
               {
               /* Step1: Enable SPI. */
               uCmd = (pDiagMgr->pBmicCfg->pBqRegsCfg[uDevId].zRegNVM.uGpio_Conf[0]  | BQ7971X_GPIO_CONF1_SPI_EN_MSK);
               uRet = Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevId+1u),COMIF_PRIO_EXCL_CELL(BQ7971X_GPIO_CONF1_OFFSET, COMIF_LOCK) ,
                                             &uCmd, WRITE_1BYTE);


               /* Step2: Sets the number of SPI bits to 24. */
               uCmd = (pDiagMgr->pBmicCfg->pBqRegsCfg[uDevId].zRegCtrl.uSpi_Conf & (BQ7971X_SPI_CONF_CPOL_MSK | BQ7971X_SPI_CONF_CPHA_MSK));
               uRet = Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevId+1u), BQ7971X_SPI_CONF_OFFSET,
                                             &uCmd, WRITE_1BYTE);

               /* Step3: Write to SPI_TX*. */
                   uRet = Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevId+1u), COMIF_PRIO_EXCL_CELL(BQ7971X_SPI_TX3_OFFSET, COMIF_LOCK),
                                             uSPIData, WRITE_3BYTE);


                   uCmd = BQ7971X_SPI_EXE_SPI_GO_MSK;
                   uRet = Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevId+1u),COMIF_PRIO_EXCL_CELL(BQ7971X_SPI_EXE_OFFSET, COMIF_LOCK) ,
                                             &uCmd, WRITE_1BYTE);

                   uCmd = BQ7971X_SPI_EXE_SS_CTRL_MSK;
                   uRet = Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevId+1u), COMIF_PRIO_EXCL_CELL(BQ7971X_SPI_EXE_OFFSET, COMIF_LOCK),
                                             &uCmd, WRITE_1BYTE);

               }

           /* Step4: Enables the SPI loopback function to verify SPI funcationality. */
           uCmd = BQ7971X_DIAG_MISC_CTRL1_SPI_LOOPBACK_MSK;
           uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,
                                       COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_MISC_CTRL1_OFFSET, COMIF_LOCK),
                                       &uCmd, WRITE_1BYTE);

            /* Step5: Read from SPI_RX*. */

            for(uDevId = 1; uDevId <= pDiagMgr->uNAFEs; uDevId++)
           {
                uCmd = BQ7971X_SPI_EXE_SPI_GO_MSK & (BQ7971X_SPI_EXE_SS_CTRL_MSK | BQ7971X_SPI_EXE_SPI_GO_MSK);
                uRet = Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevId+1u), COMIF_PRIO_EXCL_CELL(BQ7971X_SPI_EXE_OFFSET, COMIF_LOCK),
                                             &uCmd, WRITE_1BYTE);
           }
           uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL(BQ7971X_SPI_RX3_OFFSET, COMIF_LOCK),
                                  &pDiagMgr->pDiagCfg[BMI_SM_SPI_FAULT_DIAG], READ_3BYTE);
            if(BMI_OK == uRet)
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
   return Bmi_SetErrorDetails(BMI_SVCID_BMC_SPI_FAULT_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/**********************************************************************************************************************
 * uint8 FactTmDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
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

STATIC FUNC(uint8, bq7971x_diag_CODE) FactTmDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uRet = BMI_OK;
    uint8 uDevIdx;


    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
            if(BMI_SM_FACR_TM == pSvcCfg->uSubId)
            {
                 for (uDevIdx = 0u; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (pData[0]  != 0u)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_FACT_TM = DIAGERROR;
                        uRet |= (uint8) BMI_NOT_OK;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_FACT_TM = DIAGNOERROR;
                        uRet |= (uint8) BMI_OK;
                    }
               }
            }
               if (uRet == BMI_OK)
               {
                   pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
               }

            break;
        }
        case DIAG_STEP_READY:
        {
            uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_FACT_TM_REG, COMIF_LOCK) ,
                                           &pDiagMgr->pDiagCfg[BMI_SM_FACR_TM], READ_1BYTE);
            if(BMI_OK == uRet)
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

    return Bmi_SetErrorDetails(BMI_SVCID_FACR_TM, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/**********************************************************************************************************************
 * uint8 _OtpErrDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
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

STATIC FUNC(uint8, bq7971x_diag_CODE) OtpErrDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uRet = BMI_OK;
    uint8 uDevIdx;
    uint8 uCmd;
    uint8 uLoop;

    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
            if(BMI_SM_FAULT_OTP_ERR_DIAG == pSvcCfg->uSubId)
            {    
                 uLoop = pDiagMgr->sDevId;
                 for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (0u == (pData[0] & BQ7971X_FAULT_OTP_GBLOVERR_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_OTP_OVERR = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_OTP_OVERR = DIAGERROR;
                    }
                    if (0u == (pData[0] & BQ7971X_FAULT_OTP_LOADERR_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_OTP_LOADERR = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_OTP_LOADERR = DIAGERROR;
                    }
                    if (0u == (pData[0] & BQ7971X_FAULT_OTP_FACT_CRC_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_FACT_CRC = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_FACT_CRC = DIAGERROR;
                    }
                    if (0u == (pData[0] & BQ7971X_FAULT_OTP_CUST_CRC_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_CUST_CRC = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_CUST_CRC = DIAGERROR;
                    }
               }
            uCmd = BQ7971X_FAULT_RST2_RST_OTP_CRC_MSK | BQ7971X_FAULT_RST2_RST_OTP_DATA_MSK;
            uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,
                                       COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST2_OFFSET, COMIF_UNLOCK),
                                       &uCmd, WRITE_1BYTE);
               if (uRet == BMI_OK)
               {
                   pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
               }
            }
            break;
        }
        case DIAG_STEP_READY:
        {
            uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_OTP_OFFSET, COMIF_LOCK) ,
                                           &pDiagMgr->pDiagCfg[BMI_SM_FAULT_OTP_ERR_DIAG], READ_1BYTE);
            if(BMI_OK == uRet)
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

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_OTP_ERR_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/**********************************************************************************************************************
 * uint8 OtpStatDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
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

STATIC FUNC(uint8, bq7971x_diag_CODE) OtpStatDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uRet = BMI_OK;
    uint8 uDevIdx;
    uint8 uLoop;


    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
            if(BMI_SM_OTP_STATE_DIAG == pSvcCfg->uSubId)
            {
                 uLoop = pDiagMgr->sDevId;
                 for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);

                    if (0u == (pData[0] & BQ7971X_OTP_STAT_PROGERR_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_OTP_PERR = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_OTP_PERR = DIAGERROR;
                    }
                    if (0u == (pData[0] & BQ7971X_OTP_STAT_UV_OVERR_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_OTP_UVOVERROR = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_OTP_UVOVERROR = DIAGERROR;
                    }
                    if (0u == (pData[0] & BQ7971X_OTP_STAT_SUV_SOVERR_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_OTP_SUV_SOVERROR = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_OTP_SUV_SOVERROR = DIAGERROR;
                    }
                    if (0u == (pData[0] & BQ7971X_OTP_STAT_UNLOCK_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_OTP_UNLCKSTAT = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_OTP_UNLCKSTAT = DIAGERROR;
                    }
                    if (0u == (pData[0] & BQ7971X_OTP_STAT_UNLOCK_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_OTP_UNLCK = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_OTP_UNLCK = DIAGERROR;
                    }
                }

                pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
            }
            break;
        }
        case DIAG_STEP_READY:
        {
            uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_OTP_STAT_OFFSET, COMIF_LOCK) ,
                                           &pDiagMgr->pDiagCfg[BMI_SM_OTP_STATE_DIAG], READ_1BYTE);
            if(BMI_OK == uRet)
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

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_OTP_STATE_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/**********************************************************************************************************************
 * uint8 FactCrcDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
 /* \brief This function is used to factory CRC diagnostic.
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

STATIC FUNC(uint8, bq7971x_diag_CODE) FactCrcDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq)
{
    uint8 uDevIdx;
    uint8 uCmd;
    uint8 uRet = BMI_OK;
    uint8 *pData;
    uint8 uLoop;

    pDiagReq->uDiagStat = BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {

        case DIAG_STEP_FINISHED:
        {
            if(BMI_SM_FACTORY_CRC_DIAG == pSvcCfg->uSubId)
            {
                uRet = BMI_OK;
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);

                    if(0u == (pData[0] & BQ7971X_FAULT_OTP_FACT_CRC_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_FACT_CRCDIAG = DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_FACT_CRCDIAG = DIAGNOERROR;
                    }
                }

                /* Step3: Remove Flipping the expected CRC value. */
                uCmd = BQ7971X_DIAG_MISC_CTRL1_POR_VAL;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_MISC_CTRL1_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
                /* Step4: Reset the [FACT_CRC] registers. */
                uCmd = BQ7971X_FAULT_RST2_RST_OTP_CRC_MSK;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST2_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                if (uRet == BMI_OK)
                {
                     pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
                }
            }

            break;
        }
        case DIAG_STEP_READY:
        {
            /* Step1: Flip the expected CRC value. */
            uCmd = BQ7971X_DIAG_MISC_CTRL1_FLIP_FACT_CRC_MSK;
            uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_MISC_CTRL1_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
            uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_FACT_CRC_DONE_DELAY);
            /* Step2: Check factory CRC diagnostic result. */
            uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_OTP_OFFSET, COMIF_LOCK) ,
                                               &pDiagMgr->pDiagCfg[BMI_SM_FACTORY_CRC_DIAG], READ_1BYTE);
            if(BMI_OK == uRet)
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

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_FACTORY_CRC_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/**********************************************************************************************************************
 * uint8 FactCrcStatDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief Reads the [FACT_CRC_DONE] status bit to determine the CRC test cycle is complete.
 *
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7971x_diag_CODE) CrcStatDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx;
    uint8 uRet = BMI_OK;
    uint8 uLoop;
    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
            if(BMI_SM_CRC_STATE_DIAG == pSvcCfg->uSubId)
            {
                uLoop = pDiagMgr->sDevId;
                for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, (uDevIdx*2u));

                    if (0u == (pData[0]  & BQ7971X_DEV_STAT1_CUST_CRC_DONE_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_CUSTCRC_STAT = DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_CUSTCRC_STAT = DIAGNOERROR;
                    }
                    if (0u == (pData[0] & BQ7971X_DEV_STAT1_FACT_CRC_DONE_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_FACTCRC_STAT = DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_FACTCRC_STAT = DIAGNOERROR;
                    }
                }
                if (BMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
                    
                }
            }
            break;
        }
     case DIAG_STEP_READY:
        {
            uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_OTP_OFFSET, COMIF_LOCK) ,
                                                         &pDiagMgr->pDiagCfg[BMI_SM_CRC_STATE_DIAG], READ_1BYTE);
            if(BMI_OK == uRet)
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
       return Bmi_SetErrorDetails(BMI_SVCID_BMC_CRC_STATE_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);

}

/*********************************************************************************************************************
 * Function name:  Std_ReturnType FaultSysDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief This function is used to check about FAULT_SYS diagnosis.
 *
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7971x_diag_CODE) FaultSysDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx;
    uint8 uCmd;
    uint8 uRet = BMI_OK;
    uint8 uLoop;
    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
            if(BMI_FDTI_FAULT_SYS_DIAG == pSvcCfg->uSubId)
            {
                uLoop = pDiagMgr->sDevId;
                for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (0u == (pData[0] & BQ7971X_FAULT_SYS_TWARN_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_TWARN_DIE = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_TWARN_DIE = DIAGERROR;
                    }
                    if (0u == (pData[0] & BQ7971X_FAULT_SYS_TSHUT_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_TSHUT_STAT = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_TSHUT_STAT = DIAGERROR;
                    }
                    if (0u == (pData[0] & BQ7971X_FAULT_SYS_DRST_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_AVDD_UV = DIAGNOERROR;
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_DVDD_UV = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_AVDD_UV = DIAGERROR;
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_DVDD_UV = DIAGERROR;
                    }
                    if (0u == (pData[0] & BQ7971X_FAULT_SYS_LFO_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_LFOSC_FREQ = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_LFOSC_FREQ = DIAGERROR;
                    }
                }

                uCmd = BQ7971X_FAULT_RST1_RST_SYS_MSK;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);

                if (BMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
                    uRet = BMI_OK;
                }
            }
            break;
        }
     case DIAG_STEP_READY:
        {

            uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_SYS_OFFSET, COMIF_LOCK) ,
                                               &pDiagMgr->pDiagCfg[BMI_FDTI_FAULT_SYS_DIAG], READ_1BYTE);
            if(BMI_OK == uRet)
            {
                pDiagReq->uDiagStep = DIAG_STEP_FINISHED;
                pDiagReq->uDiagStat = BMI_DIAG_RUN;
            }
            break;
        }
     default :
      {
         break;
      }
    }
       return Bmi_SetErrorDetails(BMI_SVCID_BMC_FAULT_SYS_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);

}

/*************************************************************************************************************************************************
 * Function name:  Std_ReturnType VIFCrcDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 ********************************************************************************************************************* **************************/
/*! \brief This function is used to daisy chain CRC diagnostic.
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

STATIC FUNC(uint8, bq7971x_diag_CODE) VIFCrcDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq)
{
    uint8 uDevIdx;
    uint8 uCmd;
    uint8 uRet = BMI_OK;
    uint8 *pData;
    uint8 uLoop;

    pDiagReq->uDiagStat = BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {

        case DIAG_STEP_FINISHED:
        {
            if(BMI_SM_VLF_CRC_DIAG == pSvcCfg->uSubId)
            { 
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, (uDevIdx));
                    if(0u == (pData[0u]& BQ7971X_FAULT_SUMMARY_FAULT_COMM_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_VIF_CRCDIAG = DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_VIF_CRCDIAG = DIAGNOERROR;
                    }
                }

                /* Step5: Send a stack write command to register FAULT_RST2 to reset the faults. */
                uCmd = BQ7971X_FAULT_RST2_COMM;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST2_OFFSET, COMIF_UNLOCK) , &uCmd, WRITE_1BYTE);
                if (uRet == BMI_OK)
                {
                     pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
                }
            }

            break;
        }
        case DIAG_STEP_READY:
        {
            uCmd = BQ7971X_FAULT_RST2_COMM;
            uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST2_OFFSET, COMIF_LOCK) , &uCmd, WRITE_1BYTE);

             /* Step1: Sends a purposely incorrect communication CRC. */
            uCmd = (pDiagMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uDiag_Misc_Ctrl[0]  | BQ7971X_DIAG_MISC_CTRL1_FLIP_TR_CRC_MSK);
            uRet |=  Comif_SingleWrite(pDiagMgr->pComifCtx,  pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_MISC_CTRL1_OFFSET, COMIF_LOCK) , &uCmd, WRITE_1BYTE);

            /* Step2: Send a single device read command to */
            uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs ,COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_MISC_CTRL1_OFFSET, COMIF_LOCK) , &pDiagMgr->pDiagCfg[BMI_SM_COMM_FLT_MSK_DIAG], READ_3BYTE);

            /*   Step3: Clear the [FLIP_TR_CRC] bit.*/
             uCmd = (pDiagMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uDiag_Misc_Ctrl[0] & (BQ7971X_DIAG_NUM_FF ^ BQ7971X_DIAG_MISC_CTRL1_FLIP_TR_CRC_MSK));
             uRet |=  Comif_SingleWrite(pDiagMgr->pComifCtx,  pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_MISC_CTRL1_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
             /* Step4: Read the FAULT_SUMMARY register and ensure the [FAULT_COMM] bit is set for the stack devices. */
             uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_SUMMARY_OFFSET, COMIF_LOCK),
                                                            &pDiagMgr->pDiagCfg[BMI_SM_VLF_CRC_DIAG], READ_1BYTE);
             if(BMI_OK == uRet)
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

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_VLF_CRC_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/*************************************************************************************************************************************************
 * Function name:  Std_ReturnType NfaultDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 ************************************************************************************************************************************************/
/*! \brief          This function is used to NFAULT pin diagnostic.
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

STATIC FUNC(uint8, bq7971x_diag_CODE) NfaultDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq)
{

    uint8 uCmd;
    uint8 uRet = BMI_OK;

    pDiagReq->uDiagStat = BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
       {
           if(BMI_SM_FTDI_NFAULT == pSvcCfg->uSubId)
           {
               if ((uint8)STD_HIGH == Dio_ReadChannel(BQ7971X_DIO_NFAULT_PIN))
               {
                   pDiagMgr->pDiagResult[0].zFdtiResult.zSM_NFAULT &= 1u;
               }
               else
               {
                   pDiagMgr->pDiagResult[0].zFdtiResult.zSM_NFAULT |= 2u;
               }

                pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;

           }
           break;
       }
        case DIAG_STEP_STATE1:
        {
            if(BMI_SM_FTDI_NFAULT == pSvcCfg->uSubId)
            {
                 if ((uint8)STD_LOW == Dio_ReadChannel(BQ7971X_DIO_NFAULT_PIN))
                {
                   pDiagMgr->pDiagResult[0u].zFdtiResult.Comb.SM_NFAULT_DIAG = DIAGNOERROR;
                }
                else
                {
                    pDiagMgr->pDiagResult[0u].zFdtiResult.Comb.SM_NFAULT_DIAG = DIAGERROR;
                }
                /* Step3: Send comm clear. */
                //uRet = Comif_ComClearRequest(pDiagMgr->pComifCtx);  // Check with Channa
                /* Step4: Clear the fault. */
                uCmd = BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_MSK | BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_GO_MSK;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_ADC_CTRL3_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                uCmd = BQ7971X_NFAULT_DIAG_RST1;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                uCmd = BQ7971X_NFAULT_DIAG_RST2;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST2_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                uRet |=  Comif_SingleWrite(pDiagMgr->pComifCtx,  0u,COMIF_PRIO_EXCL_CELL(BQ79600_FAULT_RST_OFFSET, COMIF_LOCK) , &uCmd, WRITE_1BYTE);

                /* Step5: Verify NFAULT gets de-asserted. */
                uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_SUMMARY_OFFSET, COMIF_LOCK),
                                                            &pDiagMgr->pDiagCfg[BMI_SM_FTDI_NFAULT], READ_1BYTE);
                if(BMI_OK == uRet)
               {
                   pDiagReq->uDiagStep = DIAG_STEP_FINISHED;
                   pDiagReq->uDiagStat = BMI_DIAG_RUN;
               }
            }
            break;
        }
        case DIAG_STEP_READY:
        {
            /* [MSK_OTP_DATA] */
            uCmd = BQ7971X_FAULT_MSK2_MSK_OTP_DATA_MSK;
            uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_MSK2_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);


            /* Step1: Send a command with an incorrect initial byte. */
            uCmd = BQ7971X_DIAG_ADC_CTRL3_ACOMP_MPFLT_INJ_MSK| BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_MSK | BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_MODE_BOTH |BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_GO_MSK;
            uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_ADC_CTRL3_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);

            /* Step2: Verify NFAULT gets asserted. */
             uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_SUMMARY_OFFSET, COMIF_LOCK),
                                                            &pDiagMgr->pDiagCfg[BMI_SM_FTDI_NFAULT], READ_1BYTE);
             if(BMI_OK == uRet)
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



    return Bmi_SetErrorDetails(BMI_SVCID_BMC_NFAULT, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}



/*************************************************************************************************************************************************
 * Function name:  Std_ReturnType CommDebugDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 ************************************************************************************************************************************************/
/*! \brief         This function is used to check about DEBUG_UART and DEBUG_COM* diagnosis.
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

STATIC FUNC(uint8, bq7971x_diag_CODE) CommDebugDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq)
{
    uint8 uCmd;
    uint8 uRet = BMI_OK;
    uint8 uLoop;
    uint8   uDevIdx;
    uint8   uFaultComm;
    uint8   uDebugUart;
    uint8   uDebugComh[3u];
    uint8   uDebugComl[3u];
    uint8   uPerr, uBit, uCrc, uWait;
    uint8   uUnexp, uTxdis, uSync1, uSync2;
    uint8   uSof, uIerr, uBerr;



    pDiagReq->uDiagStat = BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
       {
           if(BMI_SM_FTDI_COMM_DEBUG_DIAG == pSvcCfg->uSubId)
           {

               // Traverse all device
               uLoop = pDiagMgr->sDevId;
               for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
               {
                   uFaultComm = BQ7971X_uFault_Summary[uDevIdx] & BQ7971X_FAULT_SUMMARY_FAULT_COMM_MSK;
                   if(0u == uFaultComm)
                   {
                       pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_VIF_BYTE = DIAGNOERROR;
                       pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_VIF_BIT = DIAGNOERROR;
                       pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_VIF_CRC = DIAGNOERROR;
                       pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_WAIT = DIAGNOERROR;
                       pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_UNEXP = DIAGNOERROR;
                       pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_TXDIS = DIAGNOERROR;
                       pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_SYNC1 = DIAGNOERROR;
                       pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_SYNC2 = DIAGNOERROR;
                       pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_SOF = DIAGNOERROR;
                       pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_IERR = DIAGNOERROR;
                       pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_BERR = DIAGNOERROR;
                   }
                   else
                   {
                       uDebugUart = BQ7971X_uDebug_Uart[uDevIdx ];
                       uDebugComh[0u] = BQ7971X_uDebug_Comh[uDevIdx][0u];
                       uDebugComh[1u] = BQ7971X_uDebug_Comh[uDevIdx][1u];
                       uDebugComh[2u] = BQ7971X_uDebug_Comh[uDevIdx][2u];
                       uDebugComl[0u] = BQ7971X_uDebug_Coml[uDevIdx][0u];
                       uDebugComl[1u] = BQ7971X_uDebug_Coml[uDevIdx][1u];
                       uDebugComl[2u] = BQ7971X_uDebug_Coml[uDevIdx][2u];
                       uPerr = ((uDebugComh[0u] | uDebugComl[0u]) & BQ7971X_DEBUG_COMH_BIT_PERR_MSK);
                       if(0u == uPerr)
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_VIF_BYTE = DIAGNOERROR;
                       }
                       else
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_VIF_BYTE = DIAGERROR;
                       }
                       uBit = ((uDebugComh[0u] | uDebugComl[0u]) & BQ7971X_DEBUG_COMH_BIT_BIT_MSK);
                       if(0u == uBit)
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_VIF_BIT = DIAGNOERROR;
                       }
                       else
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_VIF_BIT = DIAGERROR;
                       }
                       uCrc = ((uDebugComh[1u] | uDebugComh[2u] | uDebugComl[1u] | uDebugComl[2u]) & BQ7971X_DEBUG_COMH_RC_TR_RC_CRC_MSK);
                       if(0u == uCrc)
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_VIF_CRC = DIAGNOERROR;
                       }
                       else
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_VIF_CRC = DIAGERROR;
                       }
                       uWait = ((uDebugUart | uDebugComh[1u] | uDebugComl[1u]) & BQ7971X_DEBUG_UART_RC_TR_TR_WAIT_MSK);
                       if(0u == uWait)
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_WAIT = DIAGNOERROR;
                       }
                       else
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_WAIT = DIAGERROR;
                       }
                       uUnexp = ((uDebugUart | uDebugComh[1u] | uDebugComh[2u] | uDebugComl[1u] | uDebugComl[2u]) & BQ7971X_DEBUG_UART_RC_TR_RC_UNEXP_MSK);
                       if(0u == uUnexp)
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_UNEXP = DIAGNOERROR;
                       }
                       else
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_UNEXP = DIAGERROR;
                       }
                       uTxdis = ((uDebugUart | uDebugComh[1u] | uDebugComh[2u] | uDebugComl[1u] | uDebugComl[2u]) & BQ7971X_DEBUG_UART_RC_TR_RC_TXDIS_MSK);
                       if(0u == uTxdis)
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_TXDIS = DIAGNOERROR;
                       }
                       else
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_TXDIS = DIAGERROR;
                       }
                       uSync1 = ((uDebugComh[0u] | uDebugComl[0u]) & BQ7971X_DEBUG_COMH_BIT_SYNC1_MSK);
                       if(0u == uSync1)
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_SYNC1 = DIAGNOERROR;
                       }
                       else
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_SYNC1 = DIAGERROR;
                       }
                       uSync2 = ((uDebugComh[0u] | uDebugComl[0u]) & BQ7971X_DEBUG_COMH_BIT_SYNC2_MSK);
                       if(0u == uSync2)
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_SYNC2 = DIAGNOERROR;
                       }
                       else
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_SYNC2 = DIAGERROR;
                       }
                       uSof = ((uDebugUart | uDebugComh[1u] | uDebugComh[2u] | uDebugComl[1u] | uDebugComl[2u])
                               & BQ7971X_DEBUG_UART_RC_TR_RC_SOF_MSK) | (uDebugUart & BQ7971X_DEBUG_UART_RC_TR_TR_SOF_MSK);
                       if(0u == uSof)
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_SOF = DIAGNOERROR;
                       }
                       else
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_SOF = DIAGERROR;
                       }
                       uIerr = ((uDebugUart | uDebugComh[1u] | uDebugComl[1u]) & BQ7971X_DEBUG_UART_RC_TR_RC_IERR_MSK);
                       if(0u == uIerr)
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_IERR = DIAGNOERROR;
                       }
                       else
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_IERR = DIAGERROR;
                       }
                       uBerr = ((uDebugUart | uDebugComh[1u] | uDebugComh[2u] | uDebugComl[1u] | uDebugComl[2u]) & BQ7971X_DEBUG_UART_RC_TR_RC_BYTE_ERR_MSK);
                       if(0u == uBerr)
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_BERR = DIAGNOERROR;
                       }
                       else
                       {
                           pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_COMM_BERR = DIAGERROR;
                       }
                   }
               }



                uCmd = BQ7971X_FAULT_RST2_RST_COMM_UART_MSK | BQ7971X_FAULT_RST2_RST_COMM_DSY_MSK;
                uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST2_OFFSET, COMIF_LOCK) , &uCmd, WRITE_1BYTE);
                if (uRet == E_OK)
                {
                    pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
                }
           }
           break;
       }
        case DIAG_STEP_STATE3:
        {
            if(BMI_SM_FTDI_COMM_DEBUG_DIAG == pSvcCfg->uSubId)
            {
                uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL(BQ7971X_DEBUG_COML_BIT_OFFSET, COMIF_LOCK),
                                                            &pDiagMgr->pDiagCfg[BMI_SM_FTDI_COMM_DEBUG_DIAG], READ_1BYTE);
                if(BMI_OK == uRet)
               {
                   pDiagReq->uDiagStep = DIAG_STEP_FINISHED;
                   pDiagReq->uDiagStat = BMI_DIAG_RUN;
               }
            }
            break;
        }
        case DIAG_STEP_STATE2:
        {
            if(BMI_SM_FTDI_COMM_DEBUG_DIAG == pSvcCfg->uSubId)
            {
                uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL(BQ7971X_DEBUG_COMH_BIT_OFFSET, COMIF_LOCK),
                                                            &pDiagMgr->pDiagCfg[BMI_SM_FTDI_COMM_DEBUG_DIAG], READ_1BYTE);
                if(BMI_OK == uRet)
               {
                   pDiagReq->uDiagStep = DIAG_STEP_STATE3;
                   pDiagReq->uDiagStat = BMI_DIAG_RUN;
               }
            }
            break;
        }
        case DIAG_STEP_STATE1:
        {
            if(BMI_SM_FTDI_COMM_DEBUG_DIAG == pSvcCfg->uSubId)
            {
                uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL(BQ7971X_DEBUG_UART_RC_TR_OFFSET, COMIF_LOCK),
                                                            &pDiagMgr->pDiagCfg[BMI_SM_FTDI_COMM_DEBUG_DIAG], READ_1BYTE);
                if(BMI_OK == uRet)
               {
                   pDiagReq->uDiagStep = DIAG_STEP_STATE2;
                   pDiagReq->uDiagStat = BMI_DIAG_RUN;
               }
            }
            break;
        }
        case DIAG_STEP_READY:
        {
             uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_SUMMARY_OFFSET, COMIF_LOCK),
                                                            &pDiagMgr->pDiagCfg[BMI_SM_FTDI_COMM_DEBUG_DIAG], READ_1BYTE);
             if(BMI_OK == uRet)
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
    return Bmi_SetErrorDetails(BMI_SVCID_BMC_COMM_DEBUG_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*************************************************************************************************************************************************
 * Function name:  Std_ReturnType BBPlauDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 ********************************************************************************************************************* **************************/
/*! \brief API to detect open wire faults using the VCELL Plausibility check.
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

STATIC FUNC(uint8, bq7971x_diag_CODE) BBPlauDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7971x_DiagReqType *pDiagReq)
{
    uint8 uDevIdx;
    uint8 uCmd;
    uint8 uRet = BMI_OK;
    uint8 *pData;
    uint8 uLoop;
    uint8 uCheckTimesNum;
    static  uint8  BQ7971X_uBBPlauDebounce[BQ7971X_IFACES];
    static  uint32  BQ7971X_PreBBVoltage[BQ7971X_IFACES];
    uint32  xMainVoltage[BQ7971X_IFACES] = {0u};
    uint32  xRdntVoltage[BQ7971X_IFACES] = {0u};
    uint32  xBBVoltage[BQ7971X_IFACES] = {0u};

    pDiagReq->uDiagStat = BMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case DIAG_STEP_FINISHED:
        {
            if(BMI_SM_BB_PLAU_DIAG == pSvcCfg->uSubId)
            {
                for(uCheckTimesNum = 0u; uCheckTimesNum < BQ7971X_BBPLAU_CHECKTIMES;uCheckTimesNum++)
                {
                    BQ7971X_uPlauCheckTimesNum++;
                    //Initialize the diagnostic results, debounce value.
                    if(1u == BQ7971X_uPlauCheckTimesNum)
                    {   
                        uLoop = pDiagMgr->sDevId;
                        for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                        {
                            pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_BB_PLAU = DIAGNOERROR;
                            BQ7971X_uBBPlauDebounce[uDevIdx] = DIAGNOERROR;
                        }
                    }
                    else if(2u == BQ7971X_uPlauCheckTimesNum)
                    {
                        uLoop = pDiagMgr->sDevId;
                        for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                        {
                            BQ7971X_uBBPlauDebounce[uDevIdx] = DIAGERROR;
                        }
                    }
                    else
                    {
                        //Nothing
                    }

                    // Traverse all device
                    uLoop = pDiagMgr->sDevId;
                    for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                    {
                        // The BB voltage should be within the expected normal voltage range for the Bus Bar voltage.
                        xBBVoltage[uDevIdx] = xRdntVoltage[uDevIdx] - xMainVoltage[uDevIdx];
                        if((xBBVoltage[uDevIdx] < BQ7971X_BBMINPLAUVALUE) || (xBBVoltage[uDevIdx] > BQ7971X_BBMAXPLAUVALUE))
                        {
                            pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_BB_PLAU |= DIAGERROR;
                        }

                        // The calculated result should not repeatedly output the exact same unchanging value.
                        if((BQ7971X_uPlauCheckTimesNum > 1u) && (BQ7971X_PreBBVoltage[uDevIdx] != xBBVoltage[uDevIdx]))
                        {
                            BQ7971X_uBBPlauDebounce[uDevIdx] &= DIAGNOERROR;
                        }
                        BQ7971X_PreBBVoltage[uDevIdx] = xBBVoltage[uDevIdx];
                    }
                    // Check the debounce results
                       if(BQ7971X_BBPLAU_CHECKTIMES == BQ7971X_uPlauCheckTimesNum)
                       {
                           uLoop = pDiagMgr->sDevId;
                           for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                           {
                               pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_BB_PLAU |= BQ7971X_uBBPlauDebounce[uDevIdx];
                               pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                               if(pData[0] > 0u)
                               {
                                   pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_BB_PLAU = DIAGERROR;
                               }
                           }
                           BQ7971X_uPlauCheckTimesNum = 0u;
                       }
               }

                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, (uDevIdx));
                    if(0u == (pData[0u]& BQ7971X_FAULT_SUMMARY_FAULT_COMM_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_VIF_CRCDIAG = DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_VIF_CRCDIAG = DIAGNOERROR;
                    }
                }
                // Step5: Send a stack write command to register FAULT_RST2 to reset the faults.
                uCmd = BQ7971X_FAULT_RST2_COMM;
                uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST2_OFFSET, COMIF_UNLOCK) , &uCmd, WRITE_1BYTE);
                if (uRet == BMI_OK)
                {
                     pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
                }
            }
            break;
        }

        case DIAG_STEP_STATE1:
        {
            if(BMI_SM_BB_PLAU_DIAG == pSvcCfg->uSubId)
            {
                
                uLoop = pDiagMgr->sDevId;
                for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == (pData[0] & BQ7971X_ADC_DATA_RDY_DRDY_DIAG_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_DRDY_DIAG = DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_DRDY_DIAG = DIAGNOERROR;
                    }
                }
                 // Step4: ADC output results are frozen.This allows DIAG_MAIN, DIAG_RDNT register report the same moment
                   //                    result while reading sequentially
                 uLoop = pDiagMgr->sDevId;
                 for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                 {
                     uCmd = ((pDiagMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uAdc_Ctrl[1u] | BQ7971X_ADC_CTRL2_FREEZE_EN_MSK) & (BQ7971X_ADC_FREEZE_MSK));
                     uRet |=  Comif_SingleWrite(pDiagMgr->pComifCtx, (pDiagMgr->uNAFEs+1u),COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_CTRL2_OFFSET,COMIF_LOCK),&uCmd, WRITE_1BYTE);
                 }

                  // Step5: read out DIAG_RDNT_HI/MI/LO and DIAG_MAIN_HI/MI/LO, subtract
                  //  DIAG_MAIN_HI/MI/LO from DIAG_RDNT_HI/MI/LO yields busbar IR drop.
                 uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL(BQ7971X_DIAG_MAIN_HI_OFFSET, COMIF_LOCK),
                                        pSvcCfg, READ_3BYTE);

                 if(BMI_OK == uRet)
                 {
                     pDiagReq->uDiagStep = DIAG_STEP_FINISHED;
                     pDiagReq->uDiagStat = BMI_DIAG_RUN;
                 }
            }
            break;
        }
        case DIAG_STEP_READY:
        {
            // Step1: Disable the VC/CB comparison, [BB_MEAS_MODE] takes effect.
            uCmd = BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_MSK | BQ7971X_DIAG_ADC_CTRL3_BB_MEAS_MODE_MSK | BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_GO_MSK;
            uRet = Bq7971x_SetDiagAdcCtrl3(pDiagMgr->pBmcMgr, uCmd);
            // Step2: [DIAG_ANA_SEL] = CHANNEL 7; [DIAG_MEAS_GO]=0b1.
            uLoop = pDiagMgr->sDevId;
            for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {
                uCmd = ((pDiagMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uDev_Conf[1u] & BQ7971X_DEV_CONF2_BB_PIN_LOC_MSK) >> BQ7971X_DEV_CONF2_BB_PIN_LOC_POS) + BQ7971X_BB_PIN_LOC_RES;
                uRet |=  Comif_SingleWrite(pDiagMgr->pComifCtx, (pDiagMgr->uNAFEs+1u),COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_CTRL3_OFFSET,COMIF_LOCK),&uCmd, WRITE_1BYTE);
            }

            uCmd = BQ7971X_ADC_CTRL4_DIAG_MEAS_GO_MSK;
            uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_CTRL4_OFFSET, COMIF_LOCK) , &uCmd, WRITE_1BYTE);
            // Step3: Wait till [DRDY_DIAG] =1.
            uRet = Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_VCELLADC_DONE_DELAY);
            uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_DATA_RDY_OFFSET, COMIF_LOCK),
                                                            &pDiagMgr->pDiagCfg[BMI_SM_BB_PLAU_DIAG], READ_1BYTE);
             if(BMI_OK == uRet)
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

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_BB_PLAU_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*************************************************************************************************************************************************
 * Function name:  Std_ReturnType CBPlauDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
*********************************************************************************************************************/
/*! \brief Diagnostic cell balance voltage measurements are within a plausible range when cell balance is enabled
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

STATIC FUNC(uint8, bq7971x_diag_CODE) CBPlauDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7971x_DiagReqType *pDiagReq ,uint8 uChannel )
{
   uint8 uDevIdx;
   uint8 uCmd;
   uint8 *pData;
   uint8 uLoop;
   uint8 uCheckTimesNum ;
   uint8 uRet = BMI_OK;
   pDiagReq->uDiagStat = BMI_DIAG_ERROR;
   static  uint32 BQ7971X_qCBPlauDebounce[BQ7971X_IFACES];
   static  uint32  BQ7971X_PreCBVoltage[BQ7971X_IFACES];
   uint32  xCBVoltage[BQ7971X_IFACES] = {0u};


   switch(pDiagReq->uDiagStep)
   {
      case DIAG_STEP_FINISHED:
       {
           if(BMI_SM_CB_PLAU_DIAG == pSvcCfg->uSubId)
           {
               BQ7971X_uPlauCheckTimesNum++;
                //  BQ7971X_GetRsqDiagVolt(xCBVoltage, &BQ7971X_uDiag_RdntCheck);
                  // Initialize the diagnostic results, debounce value.
                  if(1u == BQ7971X_uPlauCheckTimesNum)
                  {   
                    uLoop = pDiagMgr->sDevId;
                      for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                      {
                          pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_CB_PLAU &= (~((uint32)DIAGERROR << BQ7971X_uCBPlauChannel));
                          BQ7971X_qCBPlauDebounce[uDevIdx] = DIAGNOERROR;
                      }
                  }
                  else if (2u == BQ7971X_uPlauCheckTimesNum)
                  {   
                      uLoop = pDiagMgr->sDevId;
                      for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                      {
                          BQ7971X_qCBPlauDebounce[uDevIdx] = (uint32)DIAGERROR << BQ7971X_uCBPlauChannel;
                      }
                  }
                  else
                  {
                      // Nothing
                  }

                  // Traverse all device
                  uLoop = pDiagMgr->sDevId;
                  for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                  {
                      if(BQ7971X_INVALDVOLVALUE == xCBVoltage[uDevIdx])
                      {
                          pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_CB_PLAU |= ((uint32)DIAGERROR << BQ7971X_uCBPlauChannel);
                      }
                      else if((xCBVoltage[uDevIdx] > BQ7971X_MAXPLAUVALUE) || (xCBVoltage[uDevIdx] < BQ7971X_MINPLAUVALUE))
                      {
                          pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_CB_PLAU |= ((uint32)DIAGERROR << BQ7971X_uCBPlauChannel);
                      }
                      else
                      {
                          // Nothing
                      }

                      // The calculated result should not repeatedly output the exact same unchanging value.
                      if((BQ7971X_uPlauCheckTimesNum > 1u) \
                         && (BQ7971X_PreCBVoltage[uDevIdx] != xCBVoltage[uDevIdx]))
                      {
                          BQ7971X_qCBPlauDebounce[uDevIdx] &= (~((uint32)DIAGERROR << BQ7971X_uCBPlauChannel));
                      }
                      BQ7971X_PreCBVoltage[uDevIdx] = xCBVoltage[uDevIdx];
                  }

                  // Check the debounce results
                  if(BQ7971X_CBPLAU_CHECKTIMES == BQ7971X_uPlauCheckTimesNum)
                  {
                      uLoop = pDiagMgr->sDevId;
                      for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                      {
                          pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_CB_PLAU |= BQ7971X_qCBPlauDebounce[uDevIdx];
                    //      if(s_BQ7971X_DrdyErr.BQ7971X_uDrdyMainRdntErr > 0u)
                          {
                              pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_CB_PLAU |= ((uint32)DIAGERROR << BQ7971X_uCBPlauChannel);
                          }
                      }
                      BQ7971X_uPlauCheckTimesNum = 0u;
                  }
               pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
           }
           break;
       }
       case DIAG_STEP_STATE1:
       {
           if(BMI_SM_CB_PLAU_DIAG == pSvcCfg->uSubId)
           {
               uLoop = pDiagMgr->sDevId;
               for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (((pData[0]  & BQ7971X_ADC_DATA_RDY_DRDY_DIAG_MSK)) == 0u)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_DRDY_DIAG = DIAGERROR;
                    }
                    else
                    {
                         pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_DRDY_DIAG = DIAGNOERROR;
                    }
               }
               for(uCheckTimesNum = 0u; uCheckTimesNum < BQ7971X_CBPLAU_CHECKTIMES;uCheckTimesNum++)
                {
                    uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_GPIO(BQ7971X_DIAG_RDNT_HI_OFFSET),
                                                  pSvcCfg, (READ_3BYTE));
                }
               if(BMI_OK == uRet)
              {
                   pDiagReq->uDiagStep = DIAG_STEP_FINISHED;
                   pDiagReq->uDiagStat = BMI_DIAG_RUN;
              }
           }
           break;
       }
       case DIAG_STEP_READY:
       {
            // Step1: Select the desired channel.
            uCmd = uChannel & BQ7971X_ADC_CTRL3_DIAG_ANA_SEL_MSK;
            uRet = Comif_StackWrite(pDiagMgr->pComifCtx,  (uint8) TRUE, BQ7971X_ADC_CTRL3_OFFSET, &uCmd, WRITE_1BYTE);
            uCmd = BQ7971X_ADC_CTRL4_DIAG_MEAS_GO_MSK;
            uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_ADC_CTRL4_OFFSET, &uCmd, WRITE_1BYTE);
            uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_VCELLADC_DONE_DELAY);
            // Step2: Configured channel has at least completed one measurement.
            uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_DATA_RDY_OFFSET, COMIF_LOCK),
                                  &pDiagMgr->pDiagCfg[BMI_SM_CB_PLAU_DIAG], READ_1BYTE);
            if(BMI_OK == uRet)
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
   return Bmi_SetErrorDetails(BMI_SVCID_BMC_CB_PLAU_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*************************************************************************************************************************************************
 * Function name:  Std_ReturnType GetDiagMainRdnt(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
*********************************************************************************************************************/
/*! \brief Diagnostic cell balance voltage measurements are within a plausible range when cell balance is enabled
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
STATIC FUNC(uint8, bq7971x_diag_CODE) GetDiagMainRdnt(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7971x_DiagReqType *pDiagReq,uint8 uChannel )
{
   uint8 uDevIdx;
   uint8 uCmd;
   uint8 *pData;
   uint8 uLoop;
   uint8 uRet = BMI_OK;
   switch(pDiagReq->uDiagStep)
   {
      case DIAG_STEP_FINISHED:
       {
           if(BMI_SM_GET_DIAG_MAIN_RDNT == pSvcCfg->uSubId)
           {
               uLoop = pDiagMgr->sDevId;
               for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    uCmd = (pDiagMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uAdc_Ctrl[1u] & BQ7971X_ADC_UNFREEZE_MSK);
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx+1u), COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_CTRL2_OFFSET, COMIF_LOCK),
                                            &uCmd, WRITE_1BYTE);
                }
                if (uRet == BMI_OK)
                {
                    pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
                }
           }
           break;
       }
       case DIAG_STEP_STATE2:
       {
           if(BMI_SM_GET_DIAG_MAIN_RDNT == pSvcCfg->uSubId)
           {
                uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_GPIO(BQ7971X_DIAG_RDNT_HI_OFFSET),
                                                  pSvcCfg, (READ_3BYTE));
               if(BMI_OK == uRet)
              {
                   pDiagReq->uDiagStep = DIAG_STEP_FINISHED;
                   pDiagReq->uDiagStat = BMI_DIAG_RUN;
              }
           }
           break;
       }
       case DIAG_STEP_STATE1:
       {
           if(BMI_SM_GET_DIAG_MAIN_RDNT == pSvcCfg->uSubId)
           {   
               uLoop = pDiagMgr->sDevId;
               for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (((pData[0]  & BQ7971X_ADC_DATA_RDY_DRDY_DIAG_MSK)) == 0u)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_DRDY_DIAG = DIAGERROR;
                    }
                    else
                    {
                         pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_DRDY_DIAG = DIAGNOERROR;
                    }
               }
               for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                uCmd = (pDiagMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uAdc_Ctrl[1u] |(BQ7971X_ADC_CTRL2_FREEZE_EN_MSK& BQ7971X_ADC_FREEZE_MSK));
                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx+1u), COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_CTRL2_OFFSET, COMIF_LOCK),
                                            &uCmd, WRITE_1BYTE);
               }
                uRet = Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_GPIO(BQ7971X_DIAG_MAIN_HI_OFFSET),
                                                  pSvcCfg, (READ_3BYTE));
               if(BMI_OK == uRet)
              {
                   pDiagReq->uDiagStep = DIAG_STEP_STATE2;
                   pDiagReq->uDiagStat = BMI_DIAG_RUN;
              }
           }
           break;
       }
       case DIAG_STEP_READY:
       {
            uCmd = uChannel & BQ7971X_ADC_CTRL3_DIAG_ANA_SEL_MSK;
            uRet = Comif_StackWrite(pDiagMgr->pComifCtx,  (uint8) TRUE, BQ7971X_ADC_CTRL3_OFFSET, &uCmd, WRITE_1BYTE);
            uCmd = BQ7971X_ADC_CTRL4_DIAG_MEAS_GO_MSK;
            uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE, BQ7971X_ADC_CTRL4_OFFSET, &uCmd, WRITE_1BYTE);
            if(0u == (uChannel & BQ7971X_ADC_CTRL3_DIAG_VCELL_GPIO_MSK))
            {
                uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_VCELLADC_DONE_DELAY);
            }
             else
            {
                uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_GPIOADC_DONE_DELAY);
            }
            uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_DATA_RDY_OFFSET, COMIF_LOCK),
                                  &pDiagMgr->pDiagCfg[BMI_SM_GET_DIAG_MAIN_RDNT], READ_1BYTE);
            if(BMI_OK == uRet)
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
   return Bmi_SetErrorDetails(BMI_SVCID_BMC_GET_DIAG_MAIN_RDNT, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}



/*************************************************************************************************************************************************
 * Function name:  Std_ReturnType FaultPwrDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
*********************************************************************************************************************/
/*! \brief Diagnostic cell balance voltage measurements are within a plausible range when cell balance is enabled
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
STATIC FUNC(uint8, bq7971x_diag_CODE) FaultPwrDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7971x_DiagReqType *pDiagReq)
{
   uint8 uDevIdx;
   uint8 uCmd;
   uint8 *pData;
   uint8 uRet = BMI_OK;
   uint8 uLoop;


   switch(pDiagReq->uDiagStep)
   {
      case DIAG_STEP_FINISHED:
       {
           if(BMI_SM_FAULT_PWR_DIAG == pSvcCfg->uSubId)
           {
               uLoop = pDiagMgr->sDevId;
               for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, (uDevIdx*2u));
                    if (((pData[0]  & BQ7971X_FAULT_PWR1_VSS_OPEN_MSK)) == 0u)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_REFVSS_OPEN = DIAGNOERROR;
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_VSS_OPEN    = DIAGNOERROR;

                    }
                    else
                    {
                         pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_REFVSS_OPEN = DIAGERROR;
                         pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_VSS_OPEN    = DIAGERROR;

                    }
                    if (((pData[0]  & BQ7971X_FAULT_PWR1_TSREF_OSC_MSK)) == 0u)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_TSREF_OSC = DIAGNOERROR;

                    }
                    else
                    {
                         pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_TSREF_OSC = DIAGERROR;
                    }
                    if (((pData[0]  & BQ7971X_FAULT_PWR1_TSREF_UV_MSK)) == 0u)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_TSREF_UV = DIAGNOERROR;

                    }
                    else
                    {
                         pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_TSREF_UV = DIAGERROR;
                    }
                    if (((pData[0]  & BQ7971X_FAULT_PWR1_TSREF_OV_MSK)) == 0u)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_TSREF_OV = DIAGNOERROR;

                    }
                    else
                    {
                         pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_TSREF_OV = DIAGERROR;
                    }
                    if (((pData[0]  & BQ7971X_FAULT_PWR1_DVDD_OV_MSK)) == 0u)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_DVDD_OV = DIAGNOERROR;

                    }
                    else
                    {
                         pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_DVDD_OV = DIAGERROR;
                    }
                    if (((pData[0]  & BQ7971X_FAULT_PWR1_AVDD_OSC_MSK)) == 0u)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_AVDD_OSC = DIAGNOERROR;

                    }
                    else
                    {
                         pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_AVDD_OSC = DIAGERROR;
                    }
                    if (((pData[0]  & BQ7971X_FAULT_PWR1_AVDD_OV_MSK)) == 0u)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_AVDD_OV = DIAGNOERROR;

                    }
                    else
                    {
                         pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_AVDD_OV = DIAGERROR;
                    }

                    pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs,( (uDevIdx*2u) +1u));
                    if (((pData[0]  & BQ7971X_FAULT_PWR2_NEG_CPUMP_MSK)) == 0u)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_NCPUMP_UV = DIAGNOERROR;

                    }
                    else
                    {
                         pDiagMgr->pDiagResult[uDevIdx].zFdtiResult.Comb.SM_NCPUMP_UV = DIAGERROR;
                    }
               }


               uCmd = BQ7971X_FAULT_RST1_RST_PWR_MSK;
               uRet = Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST1_OFFSET, COMIF_UNLOCK) , &uCmd, WRITE_1BYTE);

                if (uRet == BMI_OK)
                {
                    pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
                }
           }
           break;
       }
       case DIAG_STEP_READY:
       {
            uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_PWR1_OFFSET, COMIF_LOCK),
                                  &pDiagMgr->pDiagCfg[BMI_SM_FAULT_PWR_DIAG], READ_2BYTE);
            if(BMI_OK == uRet)
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
   return Bmi_SetErrorDetails(BMI_SVCID_BMC_FAULT_PWR_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7971X_FlipResetDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief This function is used to detect stuck bit failure in the ADC output registers.
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
STATIC FUNC(uint8, bq7971x_diag_CODE) FlipResetDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7971x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8  uRet = BMI_OK;
    uint8  uCmd;
    uint8  uLoop;
    uint8  uDevIdx;
    uint8  uIndex;
    uint8  uCheckReset;
    uint8  uCellDataNum;
    uint16 uGpioDataNum;
    uint16 xCellVoltage[BQ7971X_AFE_NUM_MAX * BQ7971X_CELL_NUM_ACT] = {0u};
    uint16 xGpioVoltage[BQ7971X_AFE_NUM_MAX * BQ7971X_GPIO_NUM_ACT] = {0u};

    switch(pDiagReq->uDiagStep)
    {
       case DIAG_STEP_FINISHED:
       { 
         if(BMI_FDTI_FLIP_RESET_DIAG == pSvcCfg->uSubId)
         {   
             uLoop = pDiagMgr->sDevId;
             for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
             {
               uCmd = pDiagMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uAdc_Ctrl;
               uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx + 1u), COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_CTRL1_OFFSET, COMIF_LOCK), &uCmd, BQ7971X_ADC_CTRL_REG_NUM);
             }
              uCmd = BQ7971X_FAULT_RST2_RST_ADC_MSK | BQ7971X_FAULT_RST2_RST_COMM_FCOMM_MSK;
              uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,
                                          COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST2_OFFSET, COMIF_UNLOCK),
                                          &uCmd, WRITE_1BYTE);
             if (uRet == (uint8) BMI_OK)
             {
               pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
             }
         }
         break;
       }
       case DIAG_STEP_STATE3:
       { 
         if(BMI_FDTI_FLIP_RESET_DIAG == pSvcCfg->uSubId)
         {     /* 8.2 BQ7971X_GetVCell*/
                uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL( (BQ7971X_VCELL18_HI_OFFSET + ((BQ7971X_CELL_NUM_MAX - BQ7971X_CELL_NUM_ACT) * 2u)), COMIF_LOCK),
                                           &pDiagMgr->pDiagCfg[BMI_FDTI_FLIP_RESET_DIAG],(BQ7971X_CELL_NUM_ACT * READ_2BYTE));
                /*8.3 BQ7971X_CheckReset() in BQ7971X_ResetCallBack() */
               uLoop = pDiagMgr->sDevId;
               for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
               {   
                 pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs,uDevIdx);
                 for (uIndex = 0u; uIndex < BQ7971X_CELL_NUM_ACT; uIndex++)
                   {
                       uCellDataNum = ((((uDevIdx + 1u) * BQ7971X_CELL_NUM_ACT) - uIndex) - 1u) * 2u;
                       xCellVoltage[(uDevIdx * BQ7971X_CELL_NUM_ACT) + uIndex] = (((uint16) pData[uCellDataNum] << 8u) \
                                                                              |(uint16) pData[uCellDataNum + 1u]);
                   }
               }
                 /* 8.3.2 BQ7971X_GetRsqVcell */
               for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
               {
                   pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs,uDevIdx);
                   for (uIndex = 0u; uIndex < BQ7971X_GPIO_NUM_ACT; uIndex ++)
                   {
                       uGpioDataNum = ((uDevIdx * BQ7971X_GPIO_NUM_ACT) + uIndex) * 2u;
                       xGpioVoltage[(uDevIdx * BQ7971X_GPIO_NUM_ACT) + uIndex] = (((uint16) pData[uGpioDataNum] << 8u) \
                                                                              |(uint16) pData[uGpioDataNum + 1u]);
                   }
               } 
               for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
               {
                   uCheckReset = 0u;
           
                   for(uIndex = 0u; uIndex < BQ7971X_CELL_NUM_ACT; uIndex++)
                   {
                       if( 0xFFFFu != xCellVoltage[(uDevIdx * BQ7971X_CELL_NUM_ACT) + uIndex])
                       {
                           uCheckReset = 1u;
                       }
                   }
                   for(uIndex = 0u; uIndex < BQ7971X_GPIO_NUM_ACT; uIndex ++)
                   {
                       if(0xFFFFu != xGpioVoltage[(uDevIdx * BQ7971X_GPIO_NUM_ACT) + uIndex])
                       {
                           uCheckReset = 1u;
                       }
                   }
                   if (0u == uCheckReset)
                   {
                       pDiagMgr->pDiagResult->zFdtiResult.Comb.SM_FLIP_RESET = DIAGNOERROR;
                   }
                   else
                   {
                       pDiagMgr->pDiagResult->zFdtiResult.Comb.SM_FLIP_RESET = DIAGERROR;
                   }
               } 
               if (uRet == BMI_OK)
                {
                   pDiagReq->uDiagStep = DIAG_STEP_FINISHED;
                   pDiagReq->uDiagStat = BMI_DIAG_RUN;   
                }   
         }
            break;
       }
       case DIAG_STEP_STATE2:
       {
          if(BMI_FDTI_FLIP_RESET_DIAG == pSvcCfg->uSubId)
          {
             /* 4.3 BQ7971X_CheckFlipReset() in BQ7971X_FlipResetCallBack()*/
                /* 4.3.1 BQ7971X_GetRsqVcell */
             uLoop = pDiagMgr->sDevId;
             for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
             {   
               pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs,uDevIdx);
               for (uIndex = 0u; uIndex < BQ7971X_CELL_NUM_ACT; uIndex++)
                 {   
                     
                     uCellDataNum = ((((uDevIdx + 1u) * BQ7971X_CELL_NUM_ACT) - uIndex) - 1u) * 2u;
                     xCellVoltage[(uDevIdx * BQ7971X_CELL_NUM_ACT) + uIndex] = (((uint16) pData[uCellDataNum] << 8u) \
                                                                            |(uint16) pData[uCellDataNum + 1u]);
                 }
             }
               /* 4.3.2 BQ7971X_GetRsqVcell */
             for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
             {
                 pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs,uDevIdx);
                 for (uIndex = 0u; uIndex < BQ7971X_GPIO_NUM_ACT; uIndex ++)
                 {
                     uGpioDataNum = ((uDevIdx * BQ7971X_GPIO_NUM_ACT) + uIndex) * 2u;
                     xGpioVoltage[(uDevIdx * BQ7971X_GPIO_NUM_ACT) + uIndex] = (((uint16) pData[uGpioDataNum] << 8u) \
                                                                            |(uint16) pData[uGpioDataNum + 1u]);
                 }
             } 
             for (uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
             {
                 uCheckReset = 0u;
         
                 for(uIndex = 0u; uIndex < BQ7971X_CELL_NUM_ACT; uIndex++)
                 {
                     if( 0u != xCellVoltage[(uDevIdx * BQ7971X_CELL_NUM_ACT) + uIndex])
                     {
                         uCheckReset = 1u;
                     }
                 }
                 for(uIndex = 0u; uIndex < BQ7971X_GPIO_NUM_ACT; uIndex ++)
                 {
                     if(0u != xGpioVoltage[(uDevIdx * BQ7971X_GPIO_NUM_ACT) + uIndex])
                     {
                         uCheckReset = 1u;
                     }
                 }
                 if (0u == uCheckReset)
                 {
                     pDiagMgr->pDiagResult->zFdtiResult.Comb.SM_FLIP_RESET = DIAGNOERROR;
                 }
                 else
                 {
                     pDiagMgr->pDiagResult->zFdtiResult.Comb.SM_FLIP_RESET = DIAGERROR;
                 }
             } 
              /*CheckFlipResetCompleted*/
             for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
             {
                 /* Step5: Set ADC_CTRL3[FLIP_RESET] = 0b0. */
                 uCmd = pDiagMgr->pBmicCfg->pBqRegsCfg[uDevIdx].zRegCtrl.uAdc_Ctrl[2u] & (BQ7971X_DIAG_NUM_FF ^ BQ7971X_ADC_CTRL3_FLIP_RESET_MSK);
                 uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx + 1u), COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_CTRL3_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
                 /* Step6: Set the [ADC_MODE] = 00, and set ADC_CTRL2[ADC_GO] = 1. */
                 uCmd = (pDiagMgr->pBmicCfg->pBqRegsCfg[uDevIdx].zRegCtrl.uAdc_Ctrl[1u] & (BQ7971X_DIAG_NUM_FF ^ BQ7971X_ADC_CTRL2_ADC_MODE_MSK)) | (uint8)BQ7971X_ADC_CTRL2_ADC_GO_MSK;
                 uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx + 1u), COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_CTRL2_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
                  /* Step7: Set [DIAG_D1D2_SEL] =000 and set ADC_CTRL4[DIAG_MEAS_GO] = 1. */
                 uCmd = (pDiagMgr->pBmicCfg->pBqRegsCfg[uDevIdx].zRegCtrl.uAdc_Ctrl[3u] & (BQ7971X_DIAG_NUM_FF ^ BQ7971X_ADC_CTRL4_DIAG_D1D2_SEL_MSK)) | (uint8)BQ7971X_ADC_CTRL4_DIAG_MEAS_GO_MSK;
                 uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx + 1u), COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_CTRL4_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
             }
                uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_VCCBSHRT_DELAY);
                /* Step8: Read the ADC output register values and confirm the reset values */
                /* 8.1 BQ7971X_GetVGpio*/
                uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                             COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO1_HI_OFFSET, COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[BMI_FDTI_FLIP_RESET_DIAG],
                                             (BQ7971X_GPIO_NUM_MAX * READ_2BYTE));
             if (uRet == (uint8) BMI_OK)
                {
                   pDiagReq->uDiagStep = DIAG_STEP_STATE3;
                   pDiagReq->uDiagStat = BMI_DIAG_RUN;   
                }  
             break;
          }
       }
       case DIAG_STEP_STATE1:
       {
          if(BMI_FDTI_FLIP_RESET_DIAG == pSvcCfg->uSubId)
          {
             /* 4.2 BQ7971X_GetVCell*/
             uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL( (BQ7971X_VCELL18_HI_OFFSET + ((BQ7971X_CELL_NUM_MAX - BQ7971X_CELL_NUM_ACT) * 2u)), COMIF_LOCK),
                                        &pDiagMgr->pDiagCfg[BMI_FDTI_FLIP_RESET_DIAG],(BQ7971X_CELL_NUM_ACT * READ_2BYTE));
             if (uRet == BMI_OK)
                {
                   pDiagReq->uDiagStep = DIAG_STEP_STATE2;
                   pDiagReq->uDiagStat = BMI_DIAG_RUN;   
                }  
             break;
          }
       }
       case DIAG_STEP_READY:
       {   
            for(uDevIdx = pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {
                /* Step1: Set ADC_CTRL3[FLIP_RESET] = 0b1. */
                uCmd = pDiagMgr->pBmicCfg->pBqRegsCfg[uDevIdx].zRegCtrl.uAdc_Ctrl[2u] | BQ7971X_ADC_CTRL3_FLIP_RESET_MSK;
                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx + 1u), COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_CTRL3_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
                /* Step2: Set the [ADC_MODE] = 00, and set ADC_CTRL2[ADC_GO] = 1. */
                uCmd = (pDiagMgr->pBmicCfg->pBqRegsCfg[uDevIdx].zRegCtrl.uAdc_Ctrl[1u] & (BQ7971X_DIAG_NUM_FF ^ BQ7971X_ADC_CTRL2_ADC_MODE_MSK)) | (uint8)BQ7971X_ADC_CTRL2_ADC_GO_MSK;
                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx + 1u), COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_CTRL2_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
                /* Step3: Set [DIAG_D1D2_SEL] =000 and set ADC_CTRL4[DIAG_MEAS_GO] = 1. */
                uCmd = (pDiagMgr->pBmicCfg->pBqRegsCfg[uDevIdx].zRegCtrl.uAdc_Ctrl[3u] & (BQ7971X_DIAG_NUM_FF ^ BQ7971X_ADC_CTRL4_DIAG_D1D2_SEL_MSK)) | (uint8)BQ7971X_ADC_CTRL4_DIAG_MEAS_GO_MSK;
                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx + 1u), COMIF_PRIO_EXCL_CELL(BQ7971X_ADC_CTRL4_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
            }
  
            uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7971X_VCCBSHRT_DELAY);
            /* Step4: Read the ADC output register values and confirm the reset values */
             /* 4.1 BQ7971X_GetVGpio*/
            uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,
                                         COMIF_PRIO_EXCL_GPIO(BQ7971X_GPIO1_HI_OFFSET, COMIF_LOCK),
                                         &pDiagMgr->pDiagCfg[BMI_FDTI_FLIP_RESET_DIAG],
                                         (BQ7971X_GPIO_NUM_ACT * READ_2BYTE));
            if (uRet == BMI_OK)
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
   return Bmi_SetErrorDetails(BMI_FDTI_FLIP_RESET_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}                                                    

/*************************************************************************************************************************************************
 * Function name:  Std_ReturnType VIFFaultDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg, Bq7971x_DiagReqType *pDiagReq)
*********************************************************************************************************************/
/*! \brief This function is used to daisy chain fault signal diagnostic.
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

STATIC FUNC(uint8, bq7971x_diag_CODE) VIFFaultDiag(const Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7971x_DiagReqType *pDiagReq)
{
   uint8 *pData;
   uint8 uDevIdx = BQ7971X_IFACES;
   uint8 uCmd;
   uint8 uRet = BMI_OK;
   uint8 uLoop;
   pDiagReq->uDiagStat = BMI_DIAG_ERROR;



   switch(pDiagReq->uDiagStep)
   {
       case DIAG_STEP_FINISHED:
      {
          if(BMI_SM_VLF_FAULT_DIAG == pSvcCfg->uSubId)
          {
               /* Step4: Assert the base device nFAULT pin. */
               if ((uint8)STD_LOW == Dio_ReadChannel(BQ7971X_DIO_NFAULT_PIN))
               {
                   pDiagMgr->pDiagResult[0].zFdtiResult.Comb.SM_VIF_FAULT = DIAGNOERROR;
               }
               else
               {
                   pDiagMgr->pDiagResult[0].zFdtiResult.Comb.SM_VIF_FAULT = DIAGERROR;
               }
                /* Step5: Correct the CRC value in register CUST_CRC_LO. */
                   pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, (uDevIdx*2u) - 1u);
                   uCmd = pData[0];
                   uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs, COMIF_PRIO_EXCL_CELL(BQ7971X_CUST_CRC_LO_OFFSET, COMIF_LOCK),
                                            &uCmd, WRITE_1BYTE);
               /* Step6: Reset the faults in all the devices and reset the nFAULT signal. */
               uCmd = BQ7971X_FAULT_RST2_COMM | BQ7971X_FAULT_RST2_RST_OTP_CRC_MSK;
               uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,
                                       COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST2_OFFSET, COMIF_UNLOCK),
                                       &uCmd, WRITE_1BYTE);

               if(BMI_OK == uRet)
               {
                    pDiagReq->uDiagStat = BMI_DIAG_COMPLETE;
               }
          }
          break;
      }
      case DIAG_STEP_STATE1:
       {
           if(BMI_SM_VLF_FAULT_DIAG == pSvcCfg->uSubId)
           {
               uLoop = pDiagMgr->sDevId;
               for(uDevIdx = uLoop; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
               {
                   pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, (uDevIdx*2u));
                   uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, (uDevIdx+1u), COMIF_PRIO_EXCL_CELL(BQ7971X_CUST_CRC_HI_OFFSET, COMIF_LOCK),
                                        &pData[0], WRITE_2BYTE);
               }
               uCmd = 0xFFu;
               uRet |= Comif_StackWrite(pDiagMgr->pComifCtx, (uint8) TRUE,
                                           COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_RST2_OFFSET, COMIF_LOCK),
                                           &uCmd, WRITE_1BYTE);

               uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, 0u,COMIF_PRIO_EXCL_CELL(0X2030u, COMIF_LOCK) ,
                                                &uCmd, WRITE_1BYTE); //    pData[0] --> uCmd to be checked
               /* Step2: Force a customer OTP CRC fault by uploading an incorrect CRC value into register CUST_CRC_LO. */

                uCmd = ~(BQ7971X_uCust_Crc_Rslt[(BQ7971X_IFACES - 1u) ][1u]);
                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_CUST_CRC_LO_OFFSET, COMIF_LOCK) ,
                                                &(uCmd), WRITE_1BYTE);
               /* Step3: Send a single device read command to the top of stack device. */
               uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, 8u);
               uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs , COMIF_PRIO_EXCL_CELL(BQ7971X_FAULT_SUMMARY_OFFSET, COMIF_LOCK),
                                           &pDiagMgr->pDiagCfg[BMI_SM_VLF_FAULT_DIAG], READ_1BYTE);
                if(BMI_OK == uRet)
                  {
                        pDiagReq->uDiagStep = DIAG_STEP_FINISHED;
                        pDiagReq->uDiagStat = BMI_DIAG_RUN;
                  }
           }

           break;
       }
       case DIAG_STEP_READY:
       {
            /* Step1: Read the device calculated CRC for the customer OTP page. */
           uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, 8u);
           uRet |= Comif_StackRead(pDiagMgr->pComifCtx, pDiagMgr->uNAFEs,COMIF_PRIO_EXCL_CELL(BQ7971X_CUST_CRC_RSLT_HI_OFFSET, COMIF_LOCK) ,
                                         &pDiagMgr->pDiagCfg[BMI_SM_VLF_FAULT_DIAG], READ_2BYTE);
            if(BMI_OK == uRet)
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
   return Bmi_SetErrorDetails(BMI_SVCID_BMC_VLF_FAULT_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/**********************************************************************************************************************
 *  FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagFdti_2(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
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

FUNC(uint8, bq7971x_diag_CODE) Bq7971x_DiagFdti_2(Bq7971x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
                                               Bq7971x_DiagReqType *pDiagReq , uint8 uChannel)
{
    uint8 uRet;

    switch(pDiagReq->uDiagReqId)
    {
        case BMI_DIAG_ABORT_DIAG:
        {
            uRet = DiagAbortDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_FAULT_ADC_MISC:
        {
            uRet = FaultAdcMiscDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_SPI_FAULT_DIAG:
        {
            uRet = SPIFaultDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_FACR_TM:
        {
            uRet = FactTmDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_FAULT_OTP_ERR_DIAG:
        {
            uRet = OtpErrDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_OTP_STATE_DIAG:
        {
            uRet = OtpStatDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_FACTORY_CRC_DIAG:
        {
            uRet = FactCrcDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_CRC_STATE_DIAG:
        {
            uRet = CrcStatDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_FDTI_FAULT_SYS_DIAG:
        {
            uRet = FaultSysDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_VLF_CRC_DIAG:
        {
            uRet = VIFCrcDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_FTDI_NFAULT:
        {
            uRet = NfaultDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_FTDI_COMM_DEBUG_DIAG:
        {
            uRet = CommDebugDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_BB_PLAU_DIAG:
        {
            uRet = BBPlauDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_CB_PLAU_DIAG:
        {
            uRet = CBPlauDiag(pDiagMgr, pSvcReqCfg, pDiagReq , uChannel);
            break;
        }
        case BMI_SM_GET_DIAG_MAIN_RDNT:
        {
            uRet = GetDiagMainRdnt(pDiagMgr, pSvcReqCfg, pDiagReq, uChannel);
            break;
        }
        case BMI_SM_FAULT_PWR_DIAG:
        {
            uRet = FaultPwrDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_SM_VLF_FAULT_DIAG:
        {
            uRet = VIFFaultDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case BMI_FDTI_FLIP_RESET_DIAG:
        {
            uRet = FlipResetDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
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





#define BQ7971X_DIAGFUNCS_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * End of File: bq7971x_diagfdti.c
 *********************************************************************************************************************/
