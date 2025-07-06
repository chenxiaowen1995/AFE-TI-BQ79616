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
 *  File:       Bq7973x_DiagMpfdi.c
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
 * 01.00.00       9Oct2023     Ashraf & Yehia       0000000000000    Initial version
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
#include "tibms_pmi.h"
#include "bq7973x_diag.h"
#include "bq7973x.h"
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

/**********************************************************************************************************************
**  LOCAL CONSTANT MACROS
**********************************************************************************************************************/

/***********************************************************/

/* Marcos Already Defined in BQ79731x_diag.h
    #define BQ7973X_DIAG_D1D2_SEL_BG             (4u)

    #define BQ7973X_DIAGNOERROR                  (0u)
    #define BQ7973X_DIAGERROR                    (1u)

    #define BQ7973X_DIAG_OCD1_CTRL1              (0x6u)   Set OCD1 below the OC threshold(30mV) and The OC1 over
    #define BQ7973X_DIAG_OCD1_CTRL2              (0x33u)  current detection mode is enabled
    #define BQ7973X_DIAG_OCC2_CTRL2              (0xD7u)  current detection mode is enabled
    #define BQ7973X_DIAG_OCC2_CTRL1              (0x79u)  Set OCC2 below the OC threshold(-30mV) and The OC2 over
    #define BQ7973X_DIAG_OCD2_CTRL1              (0x38u)  Set OCD2 above the OC threshold(275mV) and The OC2 over
    #define BQ7973X_DIAG_OCD2_CTRL2              (0x57u)  current detection mode is enabled

    #define BQ7973X_VF_NUM_MAX                   (2u)
    #define BQ7973X_GP_ADC_NUM                   (2u)
    #define BQ7973X_FAULT_OC_MSK                 (0x6Cu)  all fault flag [OCC1], [OCD1], [OCC2] and [OCD2] be set
    #define BQ7973X_GPIO_NUM_ACT                 (15u)
    #define BQ7973X_VF_NUM_ACT                   (2u)


    #define BQ7973X_OC_THRH                      (0x3u)   Set the desired over current threshold 137.5mV in registers OC*_THR*
    #define BQ7973X_OC_THRL                      (0x85u)  and the deglitch filter time 32us is set by [OC*_DEG].


    #define BQ7973X_OC_UNLOCKCODE1               (0xCBu)
    #define BQ7973X_OC_UNLOCKCODE2               (0xF6u)


    #define BQ7973X_SW_DIAG_SW_CTRL2             (0x44u)  SW1,SW3 output is off, SW2,SW4 output is high
*/

#define BQ7973X_FACT_CRC_DONE_DELAY          (8u)
#define BQ7973X_DIAG_NUM_FF                  ((uint8)0xFFu)
#define BQ7973X_DRDY_BIST_PWR_DELAY          (4u)
#define BQ7973X_FLIP_RESET_UINT16            (0u)
#define BQ7973X_FLIP_RESET_SINT16            (0x7FFFu)
#define BQ7973X_FLIP_RESET_SINT24            (0x7FFFFFu)
#define COM_REQ_CMD_ERR_CRC                  (0x04u)
#define COM_AFE_CUSTOM_CMD_LEN               (1u)
#define COM_REQ_CMD_INVALID_INIT_BYTE        (0x05u)
#define COM_REQ_CMD_ERR_CRC                  (0x04u)
#define COM_REQ_CMD_COMM_CLEAR               (0x01u)
#define BQ7973X_DEBUG_CTRL_UNLOCKCODE        (0xA5u)
#define BQ7973X_DEBUG_TX_DIS                 (0x0Bu)

/**********************************************************************************************************************
**  LOCAL DATA PROTOTYPES
**********************************************************************************************************************/

/*********************************************************************************************************************
 *  Local Functions Definition
 *********************************************************************************************************************/

#define BQ7973X_DIAGSTARTUP_START_SEC_CODE
#include "Cdd_MemMap.h"


//static uint8 BQ7973x_uStartAfeId = 0u;           If the first stack device in the daisy chain is BQ79731, set the flag bit to 1


/*********************************************************************************************************************
 *Std_ReturnType BQ7973X_AcompFltinjDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief This function is used to analog comparison fault injection diagnostic.
 *
 * Extended function description
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pDiagCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval PMI_OK: Insert command successful
 *         PMI_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, diag_CODE) BQ7973X_AcompFltinjDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal = (uint8)PMI_OK;
    uint8   uCmd;
    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint8 qFault;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {
            /* Step1: Reset the FAULT_ADC_* registers. */
            uCmd = BQ7973X_FAULT_RST2_RST_ADC_MSK;
            //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            { 
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST2_OFFSET, COMIF_LOCK),
                                            &uCmd, BQ7973X_WRITE_1BYTE);

                /* Step2: Select Both cell voltage measurement and GPIO measurement analog path diagnostic
                  and Enable the device ADC digital path diagnostic. */
                uCmd = BQ7973X_DIAG_ADC_CTRL2_DIAG_DIG_EN_MSK | BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_MODE_MSK | BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_GO_MSK;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                            COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_ADC_CTRL2_OFFSET, COMIF_LOCK),&uCmd,BQ7973X_WRITE_1BYTE);
            }
                /* Step3: Make sure the diagnostic is running. */ 
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_DEV_STAT1_OFFSET, COMIF_LOCK),
                                        &pDiagMgr->pDiagCfg[PMI_MPFDI_ACOMP_FLTINJ_DIAG], BQ7973X_READ_3BYTE);                                                                  

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE1:
        {
            /*  BQ7973X_AcompRunCallBack: to make sure the ACOMP is running */
            if((uint8)PMI_MPFDI_ACOMP_FLTINJ_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_ACOMP_GPIO = (uint16)BQ7973X_DIAGNOERROR;
                //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == (pData[0u] & BQ7973X_DEV_STAT1_DIAG_ANA_RUN_MSK ) )
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_ACOMP_GPIO = (uint16)BQ7973X_DIAGERROR;
                         RetVal = (uint8)PMI_NOT_OK;
                    }
                    else
                    {
                         pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_ACOMP_GPIO = (uint16)BQ7973X_DIAGNOERROR;
                    }
                }
            }
            /* Step4: Set DIAG_ADC_CTRL2[ACOMP_MPFLT_INJ] = 0b1. */
            uCmd = BQ7973X_DIAG_ADC_CTRL2_ACOMP_MPFLT_INJ_MSK | BQ7973X_DIAG_ADC_CTRL2_DIAG_DIG_EN_MSK | BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_MODE_MSK;
            //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {                                                                                       
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                                                COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_ADC_CTRL2_OFFSET, COMIF_LOCK),&uCmd,BQ7973X_WRITE_1BYTE);
            }
            /* Step5: It takes 133* tslot to finish the diagnose of ACOMP. */
            RetVal |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_ACOMP_DONE_DELAY);

            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_STAT1_OFFSET, COMIF_LOCK),
                                        &pDiagMgr->pDiagCfg[PMI_MPFDI_ACOMP_FLTINJ_DIAG], BQ7973X_READ_2BYTE);                                                                  
    
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE2;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE2:
        {
            /* BQ7973X_AcompDoneCallBack :to ensure that the ACOMP is complete */
            if((uint8)PMI_MPFDI_ACOMP_FLTINJ_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_ANA_VF = (uint8)BQ7973X_DIAGNOERROR;
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_ANA_GP = (uint8)BQ7973X_DIAGNOERROR;
                //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == (pData[1u] & BQ7973X_DIAG_STAT2_DRDY_ANA_VF_MSK ))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_ANA_VF = (uint8)BQ7973X_DIAGERROR;
                        RetVal = (uint8)PMI_NOT_OK;
                    }
                    else
                    {
                        RetVal = (uint8)PMI_OK;
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_ANA_VF = (uint8)BQ7973X_DIAGNOERROR;
                    }


                    if(0u == (pData[1u] & BQ7973X_DIAG_STAT2_DRDY_ANA_GPIO_MSK ))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_ANA_GP = (uint8)BQ7973X_DIAGERROR;
                        RetVal = (uint8)PMI_NOT_OK;
                    }
                    else
                    {
                        RetVal = (uint8)PMI_OK;
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_ANA_GP = (uint8)BQ7973X_DIAGNOERROR;
                    }
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_STAT1_OFFSET, COMIF_LOCK),
                                                    &pDiagMgr->pDiagCfg[PMI_MPFDI_ACOMP_FLTINJ_DIAG] ,BQ7973X_READ_2BYTE );
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE3;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE3:
        {
            /* BQ7973X_GetFltAdcGpioCallBack : to read the FAULT_ADC_VCEL */
            if((uint8)PMI_MPFDI_ACOMP_FLTINJ_DIAG != pSvcCfg->uSubId)
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, uDevIdx , COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_ADC_VF_OFFSET, COMIF_LOCK),
                                                    &pDiagMgr->pDiagCfg[PMI_MPFDI_ACOMP_FLTINJ_DIAG] ,BQ7973X_READ_1BYTE );

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE4;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE4:
        {
            /* BQ7973X_CheckAcompFltinjCallBack : to check ACOMP fault injection diagnostic result */
            uint16  xGpioCheckResult = (((uint16)1u << BQ7973X_GPIO_NUM_ACT) - 1u);
            uint8   uVFCheckResult = (((uint8)1u << BQ7973X_VF_NUM_ACT) - 1u);
            if((uint8)PMI_MPFDI_ACOMP_FLTINJ_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_ACOMP_FLTINJ_VF = (uint8)BQ7973X_DIAGNOERROR;
                //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    qFault = pData[0u] ^ uVFCheckResult;
                    pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_ACOMP_FLTINJ_VF = (uint8)qFault;

                    qFault = (uint8) (((uint16)pData[0u] << 8u) | ( ((uint16)pData[1u]) ^ xGpioCheckResult));
                    pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_ACOMP_FLTINJ_GP = (uint16)qFault;
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
            /* Step6: Stop digital fault injection by DIAG_ADC_CTRL2[ACOMP_MPFLT_INJ] = 0b0. */
            uCmd = BQ7973X_DIAG_ADC_CTRL2_DIAG_DIG_EN_MSK | BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_GO_MSK;
            //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {                                                                 
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                                                COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_ADC_CTRL2_OFFSET, COMIF_LOCK),&uCmd,BQ7973X_WRITE_1BYTE);
            }

            /* Step7: Reset the FAULT_ADC_* registers. */
            uCmd = BQ7973X_FAULT_RST2_RST_ADC_MSK;
            //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                                                COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST2_OFFSET, COMIF_LOCK),&uCmd,BQ7973X_WRITE_1BYTE);
            }

            /* Step8: Check the faults are cleared. */
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  , COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_ADC_GPIO1_OFFSET, COMIF_LOCK),
                                                    &pDiagMgr->pDiagCfg[PMI_MPFDI_ACOMP_FLTINJ_DIAG] ,BQ7973X_READ_2BYTE );
                                          
                

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE5;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE5:
        {
            /*  BQ7973X_GetFltAdcGpioCallBack : to read the FAULT_ADC_VCELL */
            if((uint8)PMI_MPFDI_ACOMP_FLTINJ_DIAG != pSvcCfg->uSubId)
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
                                                                             
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  , COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_ADC_VF_OFFSET, COMIF_UNLOCK),
                                                    &pDiagMgr->pDiagCfg[PMI_MPFDI_ACOMP_FLTINJ_DIAG] ,BQ7973X_READ_1BYTE );
            
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            /* BQ7973X_ClearAcompFltinjCallBack : to check ACOMP fault clean result */
            if((uint8)PMI_MPFDI_ACOMP_FLTINJ_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_ACOMP_FLTINJ_VF = (uint8)BQ7973X_DIAGNOERROR;
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_ACOMP_FLTINJ_GP = (uint16)BQ7973X_DIAGNOERROR;
                //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == pData[0u])
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_ACOMP_FLTINJ_VF &= (~((uint8)BQ7973X_DIAGERROR << BQ7973X_VF_NUM_MAX));
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_ACOMP_FLTINJ_VF |= ((uint8)BQ7973X_DIAGERROR << BQ7973X_VF_NUM_MAX);
                    }
                    if(0u == (((uint16)pData[0u] << 8u) | ((uint16)pData[1u])))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_ACOMP_FLTINJ_GP &= (~((uint16)BQ7973X_DIAGERROR << BQ7973X_GPIO_NUM_ACT));
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_ACOMP_FLTINJ_GP |= ((uint16)BQ7973X_DIAGERROR << BQ7973X_GPIO_NUM_ACT);
                    }
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
            }
            break;
        }
        default:
        {
            break;
        }
    }

    return Pmi_SetErrorDetails(PMI_MPFDI_ACOMP_FLTINJ_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************
 *Std_ReturnType BQ7973X_DcompFltinjDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \This function is used to digital comparison fault injection diagnostic.
 *
 * Extended function description
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pDiagCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval PMI_OK: Insert command successful
 *         PMI_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/

STATIC FUNC(uint8, diag_CODE) BQ7973X_DcompFltinjDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                Bq7973x_DiagReqType *pDiagReq )
{
    Std_ReturnType RetVal = (uint8)PMI_OK;
    uint8   uCmd;
    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint8 qFault;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {
            /* Step1: Reset the FAULT_ADC_* registers. */
            uCmd = BQ7973X_FAULT_RST2_RST_ADC_MSK;
            //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {                                                                   
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST2_OFFSET, COMIF_LOCK),
                                            &uCmd, BQ7973X_WRITE_1BYTE);
            }
            /* Step2: Make sure the diagnostic is running. */
                                                                                  
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  , COMIF_PRIO_EXCL_CELL(BQ7973X_DEV_STAT1_OFFSET, COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_DCOMP_FLTINJ_DIAG], BQ7973X_READ_2BYTE);
            

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE1:
        {
            /* BQ7973X_DcompRunCallBack function : to make sure the DCOMP is running */
            if((uint8)PMI_MPFDI_DCOMP_FLTINJ_DIAG  == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DCOMP_FLTINJ_VF = (uint8)BQ7973X_DIAGNOERROR;
                //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {                    
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == (pData[1u] & BQ7973X_DEV_STAT2_DIAG_DIG_RUN_MSK))
                    {
                        RetVal = (uint8)PMI_NOT_OK;
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DCOMP_FLTINJ_VF = (uint8)DIAGERROR;
                    }
                }    
            }
            /* Step3: Set DIAG_ADC_CTRL2[DCOMP_MPFLT_INJ] = 0b1. */
            uCmd = BQ7973X_DIAG_ADC_CTRL2_DCOMP_MPFLT_INJ_MSK | BQ7973X_DIAG_ADC_CTRL2_DIAG_DIG_EN_MSK;
            //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {                    
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                            COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_ADC_CTRL2_OFFSET, COMIF_LOCK),&uCmd,BQ7973X_WRITE_1BYTE);
            }

            /* Step4: Wait for the comparison to be done. */
            RetVal |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_DCOMP_DONE_DELAY);
           
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  , COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_STAT1_OFFSET, COMIF_LOCK),
                                                    &pDiagMgr->pDiagCfg[PMI_MPFDI_DCOMP_FLTINJ_DIAG ] ,BQ7973X_READ_2BYTE );
            

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE2;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE2:
        {
            /* BQ7973X_DcompDoneCallBack : to ensure that the DCOMP is complete*/
            if((uint8)PMI_MPFDI_DCOMP_FLTINJ_DIAG  == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DCOMP_FLTINJ_VF = (uint8)BQ7973X_DIAGNOERROR;
                //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u ==(pData[1u] & BQ7973X_DIAG_STAT2_DRDY_DIG_MSK ) )
                    {
                        RetVal = (uint8)PMI_NOT_OK;
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DCOMP_FLTINJ_VF = (uint8)BQ7973X_DIAGERROR;
                    }
                    else
                    {
                        RetVal = (uint8)PMI_OK;
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DCOMP_FLTINJ_VF = (uint8)BQ7973X_DIAGNOERROR;
                    }
                }
            }
            
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_ADC_DIG1_OFFSET, COMIF_LOCK),
                                          &pDiagMgr->pDiagCfg[PMI_MPFDI_DCOMP_FLTINJ_DIAG ] ,BQ7973X_READ_3BYTE );
            

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE3;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE3:
        {
            /* BQ7973X_CheckDcompFltinjCallBack : to check DCOMP fault injection diagnostic result */
            uint8  uGpioCheckResult = (((uint8)1u << BQ7973X_GP_ADC_NUM) - 1u);
            uint8  uVFCheckResult = (((uint8)1u << BQ7973X_VF_NUM_ACT) - 1u);

            if((uint8)PMI_MPFDI_DCOMP_FLTINJ_DIAG  == pSvcCfg->uSubId)
            {
                //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    /* Reg : 0x554 , bit: 0,1 */
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    qFault = pData[2u] ^ uVFCheckResult; 
                    pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DCOMP_FLTINJ_VF = (uint8)qFault;
                     /* Reg : 0x552 , bit: 2 */
                    qFault = (pData[0u] >> BQ7973X_FAULT_ADC_DIG1_GP1_DFAIL_POS) ^ uGpioCheckResult;
                    pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DCOMP_FLTINJ_GP = (uint8)qFault;
                }
            }

            /* Step5: Stop digital fault injection by Set DIAG_ADC_CTRL2[DCOMP_MPFLT_INJ] = 0b0. */
            uCmd = BQ7973X_DIAG_ADC_CTRL2_DIAG_DIG_EN_MSK;

            //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {                                                                   
                RetVal |= Comif_SingleWrite( pDiagMgr->pComifCtx ,uDevIdx
                                ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST2_OFFSET, COMIF_LOCK),&uCmd,BQ7973X_WRITE_1BYTE);
                /* Step6: Reset the FAULT_ADC_* registers. */
                uCmd = BQ7973X_FAULT_RST2_RST_ADC_MSK;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx
                                ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST2_OFFSET, COMIF_LOCK),&uCmd,BQ7973X_WRITE_1BYTE);
                /* Step7: Check the faults are cleared. */
            }
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_ADC_DIG1_OFFSET, COMIF_UNLOCK),
                                            &pDiagMgr->pDiagCfg[PMI_MPFDI_DCOMP_FLTINJ_DIAG ] ,BQ7973X_READ_3BYTE );
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            /* BQ7973X_ClearDcompFltinjCallBack : to check DCOMP fault clean result*/
            if((uint8)PMI_MPFDI_DCOMP_FLTINJ_DIAG  == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DCOMP_FLTINJ_VF = (uint8)BQ7973X_DIAGNOERROR;
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DCOMP_FLTINJ_GP = (uint8)BQ7973X_DIAGNOERROR;
                //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == pData[2u])
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DCOMP_FLTINJ_VF &= (~((uint8)BQ7973X_DIAGERROR << BQ7973X_VF_NUM_MAX));
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DCOMP_FLTINJ_VF |= ((uint8)BQ7973X_DIAGERROR << BQ7973X_VF_NUM_MAX);
                    }
                    if(0u == pData[0u])
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DCOMP_FLTINJ_GP &= (~((uint8)BQ7973X_DIAGERROR << BQ7973X_GP_ADC_NUM));
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DCOMP_FLTINJ_GP |= ((uint8)BQ7973X_DIAGERROR << BQ7973X_GP_ADC_NUM);
                    }
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
            }
            break;
        }
        default:
        {
            break;
        }

    }
    return Pmi_SetErrorDetails(PMI_MPFDI_DCOMP_FLTINJ_DIAG , RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************
 *Std_ReturnType BQ7973X_Bg1Diag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief Read BG1 voltage and ensure the value is within specification
 *
 * Extended function description
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pDiagCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval PMI_OK: Insert command successful
 *         PMI_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, diag_CODE) BQ7973X_Bg1Diag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal = (uint8)PMI_OK;
    uint8   uCmd;
    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint16  xVoltage;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {   /* request ADC conversions for BG1, BG2*/
            uCmd = BQ7973X_DIAG_D1D2_SEL_BG | BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_MSK;
            //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_CTRL4_OFFSET,COMIF_LOCK),
                                            &uCmd, BQ7973X_WRITE_1BYTE);
            }
            //BQ7973X_uAdcCtrlRegData[3u] = uCmd & (~BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_MSK);
            RetVal |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_DIAG_D1D2_DELAY);
               /* read ADC_DATA_RDY if the conversion is done and the Device has completed at least one measurement*/
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  , COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_DATA_RDY_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_BG1_DIAG], BQ7973X_READ_1BYTE);
            
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE1:
        {
            /* BQ7973X_DrdyDiagD1D2 : */
            if((uint8)PMI_MPFDI_BG1_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_DIAGD1D2 = (uint8)BQ7973X_DIAGNOERROR;
                //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {   /* check if measurement of DiagD1D2 conversion is done */
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == (pData[0u] & BQ7973X_ADC_DATA_RDY_DRDY_DIAG_D1D2_MSK ))
                    {
                        RetVal = (uint8)PMI_NOT_OK;
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_DIAGD1D2 = (uint8)BQ7973X_DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_DIAGD1D2 = (uint8)BQ7973X_DIAGNOERROR;
                    }
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }

            
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  , COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_D1_HI_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_BG1_DIAG], BQ7973X_READ_2BYTE);
            
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            /* BQ7973X_CheckDiagBg1 : to check BG1 reference diagnostic result.*/
            if((uint8)PMI_MPFDI_BG1_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_BG1_DIAG = (uint8)BQ7973X_DIAGNOERROR;
                //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    xVoltage = (((uint16)pData[0u] << 8u) | (uint16)pData[1u]);
                    if((xVoltage < BQ7973X_MAXBGPLAUVALUE) && (xVoltage > BQ7973X_MINBGPLAUVALUE))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_BG1_DIAG =(uint8) BQ7973X_DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_BG1_DIAG = (uint8)BQ7973X_DIAGERROR;
                    }
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }

            uCmd = BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_MSK;
            //BQ7973X_uAdcCtrlRegData[3u] = uCmd & (~BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_MSK);
            //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_CTRL4_OFFSET,COMIF_UNLOCK),
                                                &uCmd, BQ7973X_WRITE_1BYTE);
            }
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
            }
            break;
        }
        default:
        {
            break;
        }
    }
    return Pmi_SetErrorDetails(PMI_MPFDI_BG1_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/*********************************************************************************************************************
 *Std_ReturnType BQ7973X_Bg2Diag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief Read BG2 voltage and ensure the value is within specification
 *
 * Extended function description
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pDiagCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval PMI_OK: Insert command successful
 *         PMI_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, diag_CODE) BQ7973X_Bg2Diag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal = (uint8)PMI_OK;
    uint8   uCmd;
    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint16  xVoltage;
    
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {
            uCmd = BQ7973X_DIAG_D1D2_SEL_BG | BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_MSK;
            //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_CTRL4_OFFSET,COMIF_LOCK),
                                            &uCmd, BQ7973X_WRITE_1BYTE);
            }
            //BQ7973X_uAdcCtrlRegData[3u] = uCmd & (~BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_MSK);
            RetVal |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_DIAG_D1D2_DELAY);
            
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  , COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_DATA_RDY_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_BG2_DIAG], BQ7973X_READ_1BYTE);
            
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE1:
        {
            /* BQ7973X_DrdyDiagD1D2 : */
            if((uint8)PMI_MPFDI_BG2_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_DIAGD1D2 = (uint8)BQ7973X_DIAGNOERROR;
                //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == (pData[0u] & BQ7973X_ADC_DATA_RDY_DRDY_DIAG_D1D2_MSK ))
                    {
                        RetVal = (uint8)PMI_NOT_OK;
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_DIAGD1D2 = (uint8)BQ7973X_DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_DIAGD1D2 = (uint8)BQ7973X_DIAGNOERROR;
                    }
                }
            }
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  , COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_D2_HI_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_BG2_DIAG], BQ7973X_READ_2BYTE);
            
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat =(uint8) PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            /* BQ7973X_CheckDiagBg2 : to check BG2 reference diagnostic result.*/
            if((uint8)PMI_MPFDI_BG2_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_BG2_DIAG = (uint8)BQ7973X_DIAGNOERROR;
                //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    xVoltage = (((uint16)pData[0u] << 8u) | (uint16)pData[1u]);
                    if((xVoltage < BQ7973X_MAXBGPLAUVALUE) && (xVoltage > BQ7973X_MINBGPLAUVALUE))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_BG2_DIAG =(uint8) BQ7973X_DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_BG2_DIAG = (uint8)BQ7973X_DIAGERROR;
                    }
                }
            }

            uCmd = BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_MSK;
            //BQ7973X_uAdcCtrlRegData[3u] = uCmd & (~BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_MSK);
            //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_CTRL4_OFFSET,COMIF_UNLOCK),
                                                &uCmd, BQ7973X_WRITE_1BYTE);
                }
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
            }
            break;
        }
        default:
        {
            break;
        }
    }
    return Pmi_SetErrorDetails(PMI_MPFDI_BG2_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************
 *Std_ReturnType BQ7973X_RefCapDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief Read REF_CAP value and ensure the value is within specification
 *
 * Extended function description
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pDiagCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval PMI_OK: Insert command successful
 *         PMI_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, diag_CODE) BQ7973X_RefCapDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal = (uint8)PMI_OK;
    uint8 *pData;
    uint16  xVoltage;
    uint8 uDevIdx = 0u;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  , COMIF_PRIO_EXCL_CELL(BQ7973X_REF_CAP_HI_OFFSET, COMIF_UNLOCK),
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_REFCAP_DIAG], BQ7973X_READ_2BYTE);
            
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            /* BQ7973X_CheckRefCapCallBack : to check REF_CAP voltage diagnostic result */
            if((uint8)PMI_MPFDI_REFCAP_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_REFCAP_DIAG = (uint8)BQ7973X_DIAGNOERROR;
                //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    xVoltage = (((uint16)pData[0u] << 8u) | (uint16)pData[1u]);
                    if((xVoltage < BQ7973X_MAXREFCAPPLAUVALUE) && (xVoltage > BQ7973X_MINREFCAPPLAUVALUE))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_REFCAP_DIAG = (uint8)BQ7973X_DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_REFCAP_DIAG = (uint8)BQ7973X_DIAGERROR;
                    }
                }
            }
            else
            {
                RetVal =(uint8) PMI_NOT_OK;
            }

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
            }
            break;
        }
        default:
        {
            break;
        }
    }
    return Pmi_SetErrorDetails(PMI_MPFDI_REFCAP_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/*********************************************************************************************************************
 * BQ7973X_OCUnlockDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief This function is used to check diagnosis for unlock over current fault enable access.
 *
 * Extended function description
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pDiagCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval PMI_OK: Insert command successful
 *         PMI_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, diag_CODE) BQ7973X_OCUnlockDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal = (uint8)PMI_OK;
    uint8   uCmd;
    uint8   uDevIdx =0;
    uint8 *pData;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {
            //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {
                /* Step1: Clear the FAULT_OC .*/
                uCmd = BQ7973X_FAULT_RST3_RST_OC1_MSK | BQ7973X_FAULT_RST3_RST_OC2_MSK;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST3_OFFSET,COMIF_LOCK),
                                            &uCmd, BQ7973X_WRITE_1BYTE);
                /* Step2: Unlock by writing the unlock pattern to OC_UNLOCK1 and OC_UNLOCK2 registers.*/
                uCmd = BQ7973X_OC_UNLOCKCODE1;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_OC_UNLOCK1_OFFSET,COMIF_LOCK),
                                            &uCmd, BQ7973X_WRITE_1BYTE);
                uCmd = BQ7973X_OC_UNLOCKCODE2;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_OC_UNLOCK2_OFFSET,COMIF_LOCK),
                                            &uCmd, BQ7973X_WRITE_1BYTE);
                /* Step3: Writing [OC1_FLT_GO] = 1 will trigger OC1 fault and writing [OC2_FLT_GO] = 1 will trigger OC2 fault. .*/
                uCmd = BQ7973X_OC_CTRL2_OC1_FLT_GO_MSK | BQ7973X_OC_CTRL2_OC2_FLT_GO_MSK;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_OC_CTRL2_OFFSET,COMIF_LOCK),
                                            &uCmd, BQ7973X_WRITE_1BYTE);
            }
            /* Step4: The host MCU verifies FAULT_OC register bits indicate over current fault has been set.*/
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  , COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_OC_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_OC_UNLOCK_DIAG], BQ7973X_READ_1BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            /* BQ7973X_OCUnlockCallBack, TD_ConfigPtr); to check result of direct control of the OC1 and OC2 pin output. */
            if((uint8)PMI_MPFDI_OC_UNLOCK_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OC_UNLOCK = (uint8)BQ7973X_DIAGNOERROR;
                //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (BQ7973X_FAULT_OC_MSK == (pData[0u] & BQ7973X_FAULT_OC_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OC_UNLOCK = (uint8)BQ7973X_DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OC_UNLOCK =(uint8) BQ7973X_DIAGERROR;
                    }
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
            /* Reset OC_CTRL2 Register and clear the FAULT_OC */
            //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {
                uCmd = BQ7973X_OC_CTRL2_POR_VAL;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_OC_CTRL2_OFFSET, COMIF_LOCK),
                                            &uCmd,BQ7973X_WRITE_1BYTE );
                uCmd = BQ7973X_FAULT_RST3_RST_OC1_MSK | BQ7973X_FAULT_RST3_RST_OC2_MSK;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST3_OFFSET, COMIF_UNLOCK),
                                            &uCmd, BQ7973X_WRITE_1BYTE);
            }
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
            }
            break;
        }
        default:
        {
            break;
        }
    }
    return Pmi_SetErrorDetails(PMI_MPFDI_OC_UNLOCK_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}
/*********************************************************************************************************************
 *Std_ReturnType BQ7973X_OCDetDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief This function is used to perform over current fault detection diagnostic.
 *
 * Extended function description
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pDiagCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval PMI_OK: Insert command successful
 *         PMI_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, diag_CODE) BQ7973X_OCDetDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal = (uint8)PMI_OK;
    uint8   uCmd;
    uint8   uDevIdx =0u;
    uint8   *pData;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {
                /* Set OCC1 above the OC threshold to force an over current fault condition */
                /* Step1: Set the desired over current threshold in registers OC*_THR* and the deglitch filter time is set by [OC*_DEG].*/
                uCmd = BQ7973X_OC_THRH;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                                            BQ7973X_OC_CONF1_OFFSET,&uCmd, BQ7973X_WRITE_1BYTE);
                uCmd = BQ7973X_OC_THRL;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                                            BQ7973X_OC_CONF2_OFFSET ,&uCmd, BQ7973X_WRITE_1BYTE);
            
                //RetVal |= BQ7973X_OCCtrl1Init(pDiagMgr, pSvcCfg);
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx,BQ7973X_OC_CTRL1_OFFSET,
                                            pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uOC_Ctrl, WRITE_1BYTE);

                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , BQ7973X_OC_CTRL1_OFFSET,
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_OC_DET_DIAG], READ_1BYTE);
                
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }

        case (uint8)DIAG_xSTEP_STATE1:
        {
            if((uint8)PMI_MPFDI_OC_DET_DIAG == pSvcCfg->uSubId)
            {
                RetVal = (uint8)PMI_OK;
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
                /* Step2: Set the OCC1 diagnostic threshold input to be greater than the OC*_THR* setting and begin OC1 the diagnostic.*/
                uCmd = BQ7973X_DIAG_OCC1_CTRL1;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                                            BQ7973X_DIAG_OC_CTRL1_OFFSET ,&uCmd, BQ7973X_WRITE_1BYTE);
                uCmd = BQ7973X_DIAG_OCC1_CTRL2;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                                            BQ7973X_DIAG_OC_CTRL2_OFFSET ,&uCmd, BQ7973X_WRITE_1BYTE);
                /* Step3: Clear the FAULT_OC .*/
                uCmd = BQ7973X_FAULT_RST3_RST_OC1_MSK;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                                            BQ7973X_FAULT_RST3_OFFSET ,&uCmd, BQ7973X_WRITE_1BYTE);

                /* Step4: The host MCU verifies FAULT_OC register bits indicate an over current fault has been set.*/
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  , COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_OC_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_OC_DET_DIAG],BQ7973X_READ_1BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE2;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE2:
        {
            /* BQ7973X_OCC1Det : to check result of OCC1 fault detection diagnostic. */

            if((uint8)PMI_MPFDI_OC_DET_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OC_DET_DIAG = (uint8)BQ7973X_DIAGNOERROR;
                
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (0u == (pData[0u] & BQ7973X_FAULT_OC_OCC1_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OC_DET_DIAG |= (uint8)BQ7973X_DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OC_DET_DIAG &= (uint8)(~((uint8)BQ7973X_DIAGERROR));
                    }
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
            /* Set OCD1 below the over current detection threshold to check lower current do not trip the comparator */
            /* Step1: Set the desired over current threshold in registers OC*_THR* and the deglitch filter time is set by [OC*_DEG].*/
            
                uCmd = BQ7973X_OC_THRH;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                        COMIF_PRIO_EXCL_CELL(BQ7973X_OC_CONF5_OFFSET,COMIF_LOCK) ,&uCmd, BQ7973X_WRITE_1BYTE);
                uCmd = BQ7973X_OC_THRL;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                        COMIF_PRIO_EXCL_CELL(BQ7973X_OC_CONF6_OFFSET,COMIF_LOCK) ,&uCmd, BQ7973X_WRITE_1BYTE);
                //RetVal |= BQ7973X_OCCtrl1Init(pDiagMgr, pSvcCfg);
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx,BQ7973X_OC_CTRL1_OFFSET,
                                            pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uOC_Ctrl, WRITE_1BYTE);

                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , BQ7973X_OC_CTRL1_OFFSET,
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_OC_DET_DIAG], READ_1BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE3;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE3:
        {
            if((uint8)PMI_MPFDI_OC_DET_DIAG == pSvcCfg->uSubId)
            {
                RetVal = (uint8)PMI_OK;
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
                /* Step2: Set the OCD1 diagnostic threshold input to be less than the OC*_THR* setting and begin OC1 the diagnostic.*/
                uCmd = BQ7973X_DIAG_OCD1_CTRL1;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                        COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_OC_CTRL1_OFFSET,COMIF_LOCK) ,&uCmd, BQ7973X_WRITE_1BYTE);

                uCmd = BQ7973X_DIAG_OCD1_CTRL2;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                        COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_OC_CTRL2_OFFSET,COMIF_LOCK) ,&uCmd, BQ7973X_WRITE_1BYTE);

                /* Step3: Clear the FAULT_OC .*/
                uCmd = BQ7973X_FAULT_RST3_RST_OC1_MSK;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                        COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST3_OFFSET,COMIF_LOCK) ,&uCmd, BQ7973X_WRITE_1BYTE);

                /* Step4: The host MCU verifies FAULT_OC register bits indicate over current fault has not been set.*/
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId,
                COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_OC_OFFSET,COMIF_LOCK),&pDiagMgr->pDiagCfg[PMI_MPFDI_OC_DET_DIAG],BQ7973X_READ_1BYTE);
            
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE4;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE4:
        {
            /* BQ7973X_OCD1Det :  */
            if((uint8)PMI_MPFDI_OC_DET_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OC_DET_DIAG = (uint8)BQ7973X_DIAGNOERROR;
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (0u == (pData[0u] & BQ7973X_FAULT_OC_OCD1_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OC_DET_DIAG &= (uint8)(~((uint8)BQ7973X_DIAGERROR << 1u));
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OC_DET_DIAG |= (uint8)(BQ7973X_DIAGERROR << 1u);
                    }
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
                /* Set OCC2 below the over current detection threshold to check lower current do not trip the comparator */
                /* Step1: Set the desired over current threshold in registers OC*_THR* and the deglitch filter time is set by [OC*_DEG].*/
                uCmd = BQ7973X_OC_THRH;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                                            BQ7973X_OC_CONF5_OFFSET,&uCmd, BQ7973X_WRITE_1BYTE);
                uCmd = BQ7973X_OC_THRL;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                                            BQ7973X_OC_CONF6_OFFSET ,&uCmd, BQ7973X_WRITE_1BYTE);

                //RetVal |= BQ7973X_OCCtrl1Init(pDiagMgr, pSvcCfg);
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx,BQ7973X_OC_CTRL1_OFFSET,
                                            pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uOC_Ctrl, WRITE_1BYTE);

                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , BQ7973X_OC_CTRL1_OFFSET,
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_OC_DET_DIAG], READ_1BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE5;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE5:
        {
            if((uint8)PMI_MPFDI_OC_DET_DIAG == pSvcCfg->uSubId)
            {
                RetVal = (uint8)PMI_OK;
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
                /* Step2: Set the OCC2 diagnostic threshold input to be less than the OC*_THR* setting and begin OC2 the diagnostic.*/
                uCmd = BQ7973X_DIAG_OCC2_CTRL1;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                        COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_OC_CTRL1_OFFSET,COMIF_LOCK) ,&uCmd, BQ7973X_WRITE_1BYTE);

                uCmd = BQ7973X_DIAG_OCC2_CTRL2;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                        COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_OC_CTRL2_OFFSET,COMIF_LOCK) ,&uCmd, BQ7973X_WRITE_1BYTE);

                /* Step3: Clear the FAULT_OC .*/
                uCmd = BQ7973X_FAULT_RST3_RST_OC2_MSK;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                        COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST3_OFFSET,COMIF_LOCK) ,&uCmd, BQ7973X_WRITE_1BYTE);

                /* Step4: The host MCU verifies FAULT_OC register bits indicate over current fault has not been set.*/
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId,
                COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_OC_OFFSET,COMIF_LOCK),&pDiagMgr->pDiagCfg[PMI_MPFDI_OC_DET_DIAG],BQ7973X_READ_1BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE6;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE6:
        {
            /*BQ7973X_OCC2DetCallBack*/
            if((uint8)PMI_MPFDI_OC_DET_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OC_DET_DIAG = (uint8)BQ7973X_DIAGNOERROR;
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (0u == (pData[0u] & BQ7973X_FAULT_OC_OCC2_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OC_DET_DIAG &= (uint8)(~((uint8)BQ7973X_DIAGERROR << 2u));
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OC_DET_DIAG |= (uint8)(uint8)(BQ7973X_DIAGERROR << 2u);
                    }
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
                /* Set OCD2 above the OC threshold to force an over current fault condition */
                /* Step1: Set the desired over current threshold in registers OC*_THR* and the deglitch filter time is set by [OC*_DEG].*/
                uCmd = BQ7973X_OC_THRH;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                                            BQ7973X_OC_CONF7_OFFSET,&uCmd, BQ7973X_WRITE_1BYTE);
                uCmd = BQ7973X_OC_THRL;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                                            BQ7973X_OC_CONF8_OFFSET ,&uCmd, BQ7973X_WRITE_1BYTE);

                //RetVal |= BQ7973X_OCCtrl1Init(pDiagMgr, pSvcCfg);
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx,BQ7973X_OC_CTRL1_OFFSET,
                                            pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uOC_Ctrl, WRITE_1BYTE);

                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , BQ7973X_OC_CTRL1_OFFSET,
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_OC_DET_DIAG], READ_1BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE7;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE7:
        {
            if((uint8)PMI_MPFDI_OC_DET_DIAG == pSvcCfg->uSubId)
            {
                RetVal = (uint8)PMI_OK;
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
            /* Step2: Set the OCC2 diagnostic threshold input to be less than the OC*_THR* setting and begin OC2 the diagnostic.*/
                uCmd = BQ7973X_DIAG_OCC2_CTRL1;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                        COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_OC_CTRL1_OFFSET,COMIF_LOCK) ,&uCmd, BQ7973X_WRITE_1BYTE);

                uCmd = BQ7973X_DIAG_OCC2_CTRL2;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                        COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_OC_CTRL2_OFFSET,COMIF_LOCK) ,&uCmd, BQ7973X_WRITE_1BYTE);

                /* Step3: Clear the FAULT_OC .*/
                uCmd = BQ7973X_FAULT_RST3_RST_OC2_MSK;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                        COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST3_OFFSET,COMIF_LOCK) ,&uCmd, BQ7973X_WRITE_1BYTE);

                /* Step4: The host MCU verifies FAULT_OC register bits indicate over current fault has not been set.*/
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId,
                COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_OC_OFFSET,COMIF_LOCK),&pDiagMgr->pDiagCfg[PMI_MPFDI_OC_DET_DIAG],BQ7973X_READ_1BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE8;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE8:
        {
            /*BQ7973X_OCD2DetCallBack*/
            if((uint8)PMI_MPFDI_OC_DET_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OC_DET_DIAG = (uint8)BQ7973X_DIAGNOERROR;
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (0u == (pData[0u] & BQ7973X_FAULT_OC_OCC2_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OC_DET_DIAG &= (uint8)(~((uint8)BQ7973X_DIAGERROR << 3u));
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OC_DET_DIAG |= (uint8)(BQ7973X_DIAGERROR << 3u);
                    }
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }

            /* Reset OC_CONF* Register */
            uCmd = BQ7973X_DIAG_OC_CTRL1_POR_VAL;
            //BQ7973X_uDiagOcCtrlRegData[0u] = BQ7973X_DIAG_OC_CTRL1_POR_VAL;
            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                        COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_OC_CTRL1_OFFSET,COMIF_LOCK) ,&uCmd, BQ7973X_WRITE_1BYTE);

            //BQ7973X_uDiagOcCtrlRegData[1u] = BQ7973X_DIAG_OC_CTRL2_POR_VAL;
            uCmd = BQ7973X_DIAG_OC_CTRL2_DIAG_OC_GO_MSK;
            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                        COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_OC_CTRL2_OFFSET,COMIF_LOCK) ,&uCmd, BQ7973X_WRITE_1BYTE);
            //BQ7973X_SetOCConf(uActDevID, TD_ConfigPtr);
            /*initialize GPIO_CONFx*/
            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,COMIF_PRIO_EXCL_CELL(BQ7973X_OC_CONF1_OFFSET,COMIF_LOCK),
                                     pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uOC_Conf,
                                     BQ7973X_GPIO_CONF1_6_REG_NUM);

            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_OC_CONF7_OFFSET,COMIF_LOCK),
                                     &pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uOC_Conf[6u],
                                         BQ7973X_GPIO_CONF7_8_REG_NUM);
            //BQ7973X_OCCtrl1Init(uActDevID, TD_ConfigPtr);
            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx,BQ7973X_OC_CTRL1_OFFSET,
                                            pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uOC_Ctrl, WRITE_1BYTE);

            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , BQ7973X_OC_CTRL1_OFFSET,
                                            &pDiagMgr->pDiagCfg[PMI_MPFDI_OC_DET_DIAG], READ_1BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            if((uint8)PMI_MPFDI_OC_DET_DIAG == pSvcCfg->uSubId)
            {
                uCmd = BQ7973X_FAULT_RST3_RST_OC1_MSK | BQ7973X_FAULT_RST3_RST_OC2_MSK;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx,
                        COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST3_OFFSET,COMIF_UNLOCK) ,&uCmd, BQ7973X_WRITE_1BYTE);
                if((uint8)PMI_OK == RetVal)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    RetVal = (uint8)PMI_OK;
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
            break;
        }
        default:
        {
            break;
        }
    }
    return Pmi_SetErrorDetails(PMI_MPFDI_OC_DET_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************
 *Std_ReturnType BQ7973X_SWAdjShortDiag(const Bq7973x_DiagType *pDiagMgr, uint8 uSwConf, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief API to diagnose SW adjacent short.
 *
 * Extended function description
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pDiagCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 * \param[in]       uSwConf  - SW configuration
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval PMI_OK: Insert command successful
 *         PMI_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, diag_CODE) BQ7973X_SWAdjShortDiag(const Bq7973x_DiagType *pDiagMgr, uint8 uSwConf, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal = (uint8)PMI_OK;
    uint8   uCmd;
    uint8   uDevIdx =0;
    uint8 *pData;
    uint16 xSWAdjVoltage1;
    uint16 xSWAdjVoltage3;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {
            //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {
                /* Step1: SW1,SW3 output is off, SW2,SW4 output is high. */
                uCmd = BQ7973X_SW_DIAG_SW_CTRL2;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_SW_CTRL_OFFSET,COMIF_LOCK),
                                            &uCmd, (BQ7973X_SW_REG_NUM));
            }

            RetVal |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_SW_DIAG_DELAY);

            /* Step2: The host MCU monitors the divided voltage input to verify the SW pin remain HiZ. */
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  , COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO1_HI_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_SW_ADJ_DIAG], (BQ7973X_READ_2BYTE * BQ7973X_GPIO_REG_NUM) );
            
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            /* BQ7973X_CheckSWAdj : to check SW Adjacent Short result */
            if((uint8)PMI_MPFDI_SW_ADJ_DIAG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_SW_ADJSHRT = (uint8)BQ7973X_DIAGNOERROR;
                //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    xSWAdjVoltage1 = (((uint16)pData[0u] << 8u) | (uint16)pData[1u]);
                    xSWAdjVoltage3 = (((uint16)pData[14u] << 8u) | (uint16)pData[15u]);
                    if(xSWAdjVoltage1 > BQ7973X_MINSWADJSHRTVALUE)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_SW_ADJSHRT |= (uint8)BQ7973X_DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_SW_ADJSHRT &= (uint8)(~((uint8)BQ7973X_DIAGERROR));
                    }
                    if(xSWAdjVoltage3 > BQ7973X_MINSWADJSHRTVALUE)
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_SW_ADJSHRT |= (uint8)(BQ7973X_DIAGERROR << 2u);
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_SW_ADJSHRT &= (uint8)(~((uint8)BQ7973X_DIAGERROR << 2u));
                    }
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
            /* Step3: Reset SW_CTRL Register. */
            uCmd = uSwConf;
            //for(uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            {
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_SW_CTRL_OFFSET,COMIF_UNLOCK),
                                            &uCmd, (BQ7973X_SW_REG_NUM));
            }
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
            }
            break;
        }
        default:
        {
            break;
        }
    }
    return Pmi_SetErrorDetails(PMI_MPFDI_SW_ADJ_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_OTPProgram(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief API to perform OTP programming
 *
 * Extended function description
 *
 * Extended function description
 *  \param[in]      pDiagMgr - Diagnostic manager context
 *  \param[in]      pDiagCfg - Diagnostic context
 *  \param[in]      pDiagReq - Diag Request
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval PMI_OK: Insert command successful
 *         PMI_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, diag_CODE)  BQ7973X_OTPProgram(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq, uint8 uComDir)
{
    Std_ReturnType RetVal = (uint8)PMI_OK;
    uint8   uCmd;
    uint8   uDevIdx =0u;
    uint8 *pData;

    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {

            {
                /* Step1: Unlock the OTP programming */
                uCmd = BQ7973X_FIR_OTP_UNLOCKCODE;

                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx , COMIF_PRIO_EXCL_CELL(BQ7973X_OTP_PROG_UNLOCK1_OFFSET,COMIF_LOCK),
                                            &uCmd, BQ7973X_WRITE_1BYTE);
                uCmd = BQ7973X_SEC_OTP_UNLOCKCODE;

                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx , COMIF_PRIO_EXCL_CELL(BQ7973X_OTP_PROG_UNLOCK2_OFFSET,COMIF_LOCK),
                                            &uCmd, BQ7973X_WRITE_1BYTE);


            }
            /* Step2: Read to confirm OTP_STAT[UNLOCK] = 1 */
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_OTP_STAT_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_SW_OTP_PROG], BQ7973X_READ_1BYTE );
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE1:
        {
            /* Step3: Check and ensure that [LOADED], [UV_OVOK], [TRY] bits are all clear of error all are 0's */
            /* OtpDrdy : check about OTP_STAT* after unlock the OTP programming */

            if((uint8)PMI_MPFDI_SW_OTP_PROG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OTP_UNLCK = (uint8)BQ7973X_DIAGNOERROR;

                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (0u == (pData[0u] & BQ7973X_OTP_STAT_UNLOCK_MSK) )
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OTP_UNLCK |= (uint8)BQ7973X_DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OTP_UNLCK &= (uint8)(~((uint8)BQ7973X_DIAGERROR));
                    }
                    if ( pData[0u] != BQ7973X_OTP_UNLOCK_STATCHECK)
                    {
                        RetVal = (uint8) PMI_NOT_OK;
                    }
                }


                {
                    /* Step4: Start the OTP programming */
                    //TBC the reg. ->Ctrl_Regs.uControl2
                    uCmd = pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uControl2 | BQ7973X_CONTROL2_PROG_GO_MSK;
                    RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx , COMIF_PRIO_EXCL_CELL(BQ7973X_CONTROL2_OFFSET,COMIF_LOCK),
                                                &uCmd, BQ7973X_WRITE_1BYTE);
                }

                /* Step5: Start the OTP programming */
                RetVal |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_OTP_DONE_DELAY);

                /* Step6: Check to ensure there is no error during OTP programming */
                    RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_OTP_STAT_OFFSET,COMIF_LOCK),
                                                 &pDiagMgr->pDiagCfg[PMI_MPFDI_SW_OTP_PROG], BQ7973X_READ_1BYTE );

            }
            else
            {
                RetVal =(uint8) PMI_NOT_OK;
            }
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep =(uint8) DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            /* BQ7973X_OtpDone : to check about OTP_STAT* after the OTP programming */
            if((uint8)PMI_MPFDI_SW_OTP_PROG == pSvcCfg->uSubId)
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OTP_ECC = (uint8)BQ7973X_DIAGNOERROR;
                pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OTP_PERR = (uint8)BQ7973X_DIAGNOERROR;
                //for(uDevIdx = 0; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (0u == (pData[0u] & BQ7973X_OTP_STAT_DONE_MSK) )
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OTP_ECC |= (uint8)BQ7973X_DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OTP_ECC &= (uint8)(~((uint8)BQ7973X_DIAGERROR));
                    }
                    if (0u != (pData[0u] >> BQ7973X_OTP_STAT_PROGERR_POS) )
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OTP_PERR |=(uint8) BQ7973X_DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OTP_PERR &=(uint8) (~((uint8)BQ7973X_DIAGERROR));
                    }
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
            /* Step7: Issue a digital reset to reload the registers with the updated OTP values */
            uCmd = (uComDir | BQ7973X_CONTROL1_SOFT_RESET_MSK);
            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx,COMIF_PRIO_EXCL_CELL(BQ7973X_CONTROL1_OFFSET, COMIF_UNLOCK),&uCmd, BQ7973X_WRITE_1BYTE);

            RetVal |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_SOFT_RESET_DELAY);

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
            }
            break;
        }
        default:
        {
            break;
        }
    }

    return Pmi_SetErrorDetails(PMI_MPFDI_SW_OTP_PROG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}



/*********************************************************************************************************************
 * Function name:  uint8 BQ7973X_VIFCrcDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief         This function is used to daisy chain CRC diagnostic.
*
*  \param[in]      pDiagMgr - Diagnostic manager context
*  \param[in]      pDiagCfg - Diagnostic context
*  \param[in]      pDiagReq - Diag Request
*
*  \reentrant      TRUE
*  \synchronous    FALSE
*  \pre
*  \post
*  \return         uint8
*  \retval         PMI_OK
*                  PMI_NOT_OK
*  \trace
*********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_VIFCrcDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal = (uint8)PMI_OK;
    uint8   uCmd;
    uint8 *pData;
    uint8   uDevIdx =0u;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {
                uCmd = BQ7973X_FAULT_RST2_COMM | BQ7973X_FAULT_RST2_RST_OTP_CRC_MSK;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST2_OFFSET, COMIF_LOCK) ,
                                     &uCmd, WRITE_1BYTE);

                /* Step1: Sends a purposely incorrect communication CRC. */
                uCmd = BQ7973X_DIAG_MISC_CTRL1_FLIP_TR_CRC_MSK;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_MISC_CTRL1_OFFSET,COMIF_LOCK),
                                            &uCmd, WRITE_1BYTE);
                /* Step2: Send a single device read command to the top of stack device. */
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx,pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_MISC_CTRL1_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_VIF_CRC_DIAG], READ_2BYTE);
                /* Step3: Clear the [FLIP_TR_CRC] bit. */
                uCmd = BQ7973X_DIAG_MISC_CTRL1_POR_VAL;
               // BQ7973X_uDiagMiscCtrlRegData[0u] = BQ7973X_DIAG_MISC_CTRL1_POR_VAL;
                {
                    RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_MISC_CTRL1_OFFSET,COMIF_LOCK),
                                                &uCmd, WRITE_1BYTE);
                }

                /* Step4: Read the FAULT_SUMMARY register and ensure the [FAULT_COMM] bit is set for the stack devices. */
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx,pDiagMgr->sDevId, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_SUMMARY_OFFSET, COMIF_LOCK),
                                                                &pDiagMgr->pDiagCfg[PMI_FDTI_COMM_FLT_MSK_DIAG], READ_3BYTE);
                /* Step3: Clear the [FLIP_TR_CRC] bit. */
                uCmd = BQ7973X_DIAG_MISC_CTRL1_POR_VAL;
               // BQ7973X_uDiagMiscCtrlRegData[0u] = BQ7973X_DIAG_MISC_CTRL1_POR_VAL;
                {
                    RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_MISC_CTRL1_OFFSET,COMIF_LOCK),
                                                &uCmd, WRITE_1BYTE);
                }

                /* Step4: Read the FAULT_SUMMARY register and ensure the [FAULT_COMM] bit is set for the stack devices. */
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx,pDiagMgr->sDevId, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_SUMMARY_OFFSET, COMIF_LOCK),
                                                                &pDiagMgr->pDiagCfg[PMI_MPFDI_VIF_CRC_DIAG], READ_1BYTE);


            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            /* BQ7973X_VIFCrc : to check the daisy chain CRC diagnostic result. */
            if((uint8)PMI_MPFDI_VIF_CRC_DIAG == pSvcCfg->uSubId)
            {
                RetVal=(uint8)PMI_OK;
                pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                if (0u == (pData[0] & BQ7973X_FAULT_SUMMARY_FAULT_COMM_MSK))
                {
                    pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_VIF_CRCDIAG = (uint8)BQ7973X_DIAGERROR;
                }
                else
                {
                    pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_VIF_CRCDIAG = (uint8)BQ7973X_DIAGNOERROR;
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
            /* Step5: Send a stack write command to register FAULT_RST2 to reset the faults. */
            uCmd = BQ7973X_FAULT_RST2_COMM | BQ7973X_FAULT_RST2_RST_OTP_CRC_MSK;
            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST2_OFFSET, COMIF_UNLOCK) , &uCmd, WRITE_1BYTE);

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
            }
            break;
        }
        default:
        {
            break;
        }
    }
    return Pmi_SetErrorDetails(PMI_MPFDI_VIF_CRC_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/*********************************************************************************************************************
 * Function name:  uint8 BQ7973X_SleepFaultDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief         This function is used to check Diagnostics of OV UV Diagnostic.
*
*  \param[in]      pDiagMgr - Diagnostic manager context
*  \param[in]      pDiagCfg - Diagnostic context
*  \param[in]      pDiagReq - Diag Request
*
*  \reentrant      TRUE
*  \synchronous    FALSE
*  \pre
*  \post
*  \return         uint8
*  \retval         PMI_OK
*                  PMI_NOT_OK
*  \trace
*********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_SleepFaultDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal = (uint8)PMI_OK;
    uint8 *pData;
    uint8   uDevIdx =0u;
    pDiagReq->uDiagStat = (uint8)PMI_DIAG_ERROR;

    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {
             if ((uint8)STD_HIGH == Dio_ReadChannel(BQ7973X_DIO_NFAULT_PIN))
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uSM_SLEEP_FAULT = (uint8)DIAGNOERROR;
            }
            else
            {
                pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uSM_SLEEP_FAULT = (uint8)DIAGERROR;
            }
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx,pDiagMgr->sDevId, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_COMM1_OFFSET, COMIF_LOCK),
                                                            &pDiagMgr->pDiagCfg[PMI_MPFDI_SLEEP_FAULT_DIAG], READ_2BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }

        case (uint8)DIAG_xSTEP_FINISHED:
        {
            if(PMI_MPFDI_SLEEP_FAULT_DIAG== pSvcCfg->uSubId)
            {
                RetVal = (uint8)PMI_OK;
                pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                if (0u == (pData[0u] & BQ7973X_FAULT_COMM1_HB_FAIL_MSK))
                {
                    pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uSM_SLEEP_HB =    (uint8)BQ7973X_DIAGNOERROR;
                    pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uSM_SLEEP_FSTHB = (uint8)BQ7973X_DIAGNOERROR;
                }
                else
                {
                    pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uSM_SLEEP_HB =    (uint8)BQ7973X_DIAGNOERROR;
                    pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uSM_SLEEP_FSTHB = (uint8)BQ7973X_DIAGERROR;

                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
            }
            break;
        }
        default:
        {
            break;
        }
    }

   return Pmi_SetErrorDetails(PMI_MPFDI_SLEEP_FAULT_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************
 * Function name:  uint8 BQ7973X_FactCrcDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief         This function is used to factory CRC diagnostic.
*
*  \param[in]      pDiagMgr - Diagnostic manager context
*  \param[in]      pDiagCfg - Diagnostic context
*  \param[in]      pDiagReq - Diag Request
*
*  \reentrant      TRUE
*  \synchronous    FALSE
*  \pre
*  \post
*  \return         uint8
*  \retval         PMI_OK
*                  PMI_NOT_OK
*  \trace
*********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_FactCrcDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal = (uint8)PMI_OK;
    uint8   uCmd;
    uint8   uDevIdx =0u;
    uint8 *pData;
    switch(pDiagReq->uDiagStep)
    {
        case  (uint8)DIAG_xSTEP_READY:
        {
            /* Step1: Flip the expected CRC value. */
            uCmd = BQ7973X_DIAG_MISC_CTRL1_FLIP_FACT_CRC_MSK;
            {
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_MISC_CTRL1_OFFSET,COMIF_LOCK),
                                            &uCmd, WRITE_1BYTE);
            }
                RetVal |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_FACT_CRC_DONE_DELAY);
            /* Step2: Check factory CRC diagnostic result. */
            {
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx,pDiagMgr->sDevId, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_OTP_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_FACT_CRC_DIAG], READ_1BYTE);
            }
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            /* BQ7973X_FactCrc : to check factory CRC diagnostic result. */
            if(PMI_MPFDI_FACT_CRC_DIAG == pSvcCfg->uSubId) /*TBC update the flag here*/
            {
                {
                    RetVal = (uint8)PMI_OK;
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (0u == (pData[0u] & BQ7973X_FAULT_OTP_FACT_CRC_MSK))
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_FACT_CRCDIAG = (uint8)BQ7973X_DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_FACT_CRCDIAG = (uint8)BQ7973X_DIAGNOERROR;
                    }
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
            {
                /* Step3: Remove Flipping the expected CRC value. */
                uCmd = BQ7973X_DIAG_MISC_CTRL1_POR_VAL;
               // BQ7973X_uDiagMiscCtrlRegData[0u] = BQ7973X_DIAG_MISC_CTRL1_POR_VAL;

                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_MISC_CTRL1_OFFSET,COMIF_LOCK),
                                            &uCmd,WRITE_1BYTE );
                /* Step4: Reset the [FACT_CRC] registers. */
                uCmd = BQ7973X_FAULT_RST2_RST_OTP_CRC_MSK;
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST2_OFFSET,COMIF_UNLOCK),
                                            &uCmd, WRITE_1BYTE);
            }
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
            }
            break;

        }
        default:
        {
            break;
        }
    }
    return Pmi_SetErrorDetails(PMI_MPFDI_FACT_CRC_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************
 *Function name:  uint8 BQ7973X_PwrBistDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
*********************************************************************************************************************/
/*********************************************************************************************************************/
/*! \brief          This function is used to check the power supply diagnosis.
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
 *  \retval         PMI_OK
 *                  PMI_NOT_OK
 *  \trace
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_diag_CODE)BQ7973X_PwrBistDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal =(uint8) PMI_OK;
    uint8   uCmd;
    uint8  *pData;
    uint8   uDevIdx = 0u;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {
            /* Step1: Disable NFAULT Pin. */
            uCmd = (uint8)(pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uDev_Conf[0] & (BQ7973X_DIAG_NUM_FF ^ BQ7973X_DEV_CONF1_NFAULT_EN_MSK));
            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_DEV_CONF1_OFFSET,COMIF_LOCK),
                                            &uCmd, WRITE_1BYTE);

            /* Step2: Reset the FAULT_PWR* registers. */
            uCmd = BQ7973X_FAULT_RST1_RST_PWR_MSK;
            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST1_OFFSET,COMIF_LOCK),
                                            &uCmd, WRITE_1BYTE);
            /* Step3: Start power supply BIST diagnostic. */
            uCmd = BQ7973X_DIAG_MISC_CTRL2_PWRBIST_GO_MSK;
            //BQ7973X_uDiagMiscCtrlRegData[1u] = BQ7973X_DIAG_MISC_CTRL2_POR_VAL;
            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_MISC_CTRL2_OFFSET,COMIF_LOCK),
                                            &uCmd, WRITE_1BYTE);
            /* Step3.1: Wait power supply BIST diagnostic. */
            RetVal |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_DRDY_BIST_PWR_DELAY);
            /* Step4: Check whether the pwoer supply BIST is complete. */

            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId, COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_STAT1_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_PWR_BIST_DIAG],READ_2BYTE);

            if(PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE1:
        {
            if((uint8)PMI_MPFDI_PWR_BIST_DIAG == pSvcCfg->uSubId)
            {
                //(void)ComIface_AddCallbackFunc(BQ7973X_DrdyBistPwrCallBack, TD_ConfigPtr);
                RetVal = (uint8)PMI_OK;
                pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uDrdyPwrBist = BQ7973X_DIAGNOERROR;
                if(1u == (pData[0u] & BQ7973X_DIAG_STAT1_DRDY_PWRBIST_MSK ))
                {
                    RetVal = (uint8)PMI_OK;
                    pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uDrdyPwrBist        = (uint8)BQ7973X_DIAGNOERROR;
                    pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uDrdyPwrBistErr     = (uint8)BQ7973X_DIAGNOERROR;
                }
                else
                {
                    RetVal = (uint8)PMI_NOT_OK;
                    pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uDrdyPwrBist    = (uint8)BQ7973X_DIAGERROR;
                }
                // if(1u == (BQ7973X_uDiag_Stat[0u] & BQ7973X_DIAG_STAT1_DRDY_PWRBIST_MSK ))
                // {
                //     RetVal = E_OK;
                //     s_BQ7973X_DrdyCnt.BQ7973X_uDrdyPwrBist = 0u;
                //     s_BQ7973X_DrdyErr.BQ7973X_uDrdyPwrBistErr = 0u;
                // }
                // else
                // {
                //     RetVal = E_NOT_OK;
                //     s_BQ7973X_DrdyCnt.BQ7973X_uDrdyPwrBist++;
                // }
                // if(s_BQ7973X_DrdyCnt.BQ7973X_uDrdyPwrBist > BQ7973X_CALLBACK_TIMEOUT)
                // {
                //     s_BQ7973X_DrdyCnt.BQ7973X_uDrdyPwrBist = 0u;
                //     s_BQ7973X_DrdyErr.BQ7973X_uDrdyPwrBistErr = 1u;
                //     RetVal = E_OK;
                // }

            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
            /* Step5: Check the power supply diagnosis result. */
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_PWR1_OFFSET,COMIF_UNLOCK),
                                             &pDiagMgr->pDiagCfg[PMI_MPFDI_PWR_BIST_DIAG],READ_2BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            if((uint8)PMI_MPFDI_PWR_BIST_DIAG == pSvcCfg->uSubId)
            {
                //(void)ComIface_AddCallbackFunc(BQ7973X_PwrBistCallBack, TD_ConfigPtr);
                pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                //(BQ7973X_uFault_Pwr[0u] replaced with pData TBC
                if (0u == (pData[0u] & BQ7973X_FAULT_PWR1_PWRBIST_FAIL_MSK))
                {
                    pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uSM_PWR_BIST = (uint8)BQ7973X_DIAGNOERROR;
                }
                else
                {
                    pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uSM_PWR_BIST = (uint8)BQ7973X_DIAGERROR;
                }
                if(pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uDrdyPwrBistErr > 0u)
                {
                    pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uSM_PWR_BIST = (uint8)BQ7973X_DIAGERROR;
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
             /* Step6: Enable NFAULT Pin. */
            uCmd = pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uDev_Conf[0];
            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_DEV_CONF1_OFFSET,COMIF_UNLOCK),
                                            &uCmd, WRITE_1BYTE);
           if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
            }
            break;
        }
        default:
        {
            break;
        }
    }
   return Pmi_SetErrorDetails(PMI_MPFDI_PWR_BIST_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************
 *Function name:  uint8 BQ7973X_FlipResetDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
*********************************************************************************************************************/
/*********************************************************************************************************************/
/*! \brief          This function is used to detect stuck bit failure in the ADC output registers.
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
 *  \retval         PMI_OK
 *                  PMI_NOT_OK
 *  \trace
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_FlipResetDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                      Bq7973x_DiagReqType *pDiagReq)
{
    uint8   RetVal = (uint8)PMI_OK;
    uint8   uCmd;
    uint8  *pData;
    uint8   uDevIdx = 0u;
    static  uint8   uCheckReset = 0u;
    uint8   uIndex =0u;
    uint32  qCurrent[BQ7973X_CURRENT_REG_NUM] = {0u};

    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {
            /* Step1: Set ADC_CTRL3[FLIP_RESET] = 0b1. */
            uCmd =(uint8) pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uAdc_Ctrl[2] | BQ7973X_ADC_CTRL3_FLIP_RESET_MSK;

            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_CTRL3_OFFSET, COMIF_LOCK),
                                                &uCmd, WRITE_1BYTE);

            /* Step2: Set the [ADC_MODE] = 00, and set ADC_CTRL2[ADC_GO] = 1. */
            uCmd = (pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uAdc_Ctrl[1u] & (BQ7973X_DIAG_NUM_FF ^ BQ7973X_ADC_CTRL2_ADC_MODE_MSK))  | BQ7973X_ADC_CTRL2_ADC_GO_MSK;

            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_CTRL2_OFFSET, COMIF_LOCK),
                                                &uCmd, WRITE_1BYTE);
            /* Step3: Set [DIAG_D1D2_SEL] =000 and set ADC_CTRL4[DIAG_MEAS_GO] = 1. */
            uCmd = (pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uAdc_Ctrl[3u] & (BQ7973X_DIAG_NUM_FF ^ BQ7973X_ADC_CTRL4_DIAG_D1D2_SEL_MSK)) | BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_MSK;

            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_CTRL4_OFFSET, COMIF_LOCK),
                                                &uCmd, WRITE_1BYTE);

            RetVal |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_GPADC_DELAY);

            /* Step1: Set ADC_CTRL3[FLIP_RESET] = 0b1. */

            /* Step4: Read the ADC output register values and confirm the reset values */
            // RetVal |= BQ7973X_GetVGpio(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg);

            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL( BQ7973X_GPIO1_HI_OFFSET, COMIF_LOCK),
                                       &pDiagMgr->pDiagCfg[PMI_MFDI_FLIP_RESET_DIAG],(READ_2BYTE * BQ7973X_GPIO_REG_NUM));

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE1:
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
            for(uIndex = 0u; uIndex < BQ7973X_GPIO_NUM_ACT; uIndex ++)
            {
                if(BQ7973X_FLIP_RESET_UINT16 != pData[uIndex])
                {
                    uCheckReset = 1u;
                }
            }
            // Bq7973x_GetVf(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg);
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL( BQ7973X_VF2_HI_OFFSET, COMIF_LOCK),
                                       &pDiagMgr->pDiagCfg[PMI_MFDI_FLIP_RESET_DIAG],(BQ7973X_VF_REG_NUM * READ_2BYTE));

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE2;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE2:
        {
           pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
           for(uIndex = 0u; uIndex < BQ7973X_VF_NUM_MAX; uIndex ++)
            {
                if(BQ7973X_FLIP_RESET_UINT16 != pData[uIndex])
                {
                    uCheckReset = 1u;
                }
            }
            //RetVal |= Bq7973x_GetCurrent(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL( BQ7973X_CURRENT1_HI_OFFSET, COMIF_LOCK),
                                       &pDiagMgr->pDiagCfg[PMI_MFDI_FLIP_RESET_DIAG],(BQ7973X_CURRENT_REG_NUM * READ_3BYTE));

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE3;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE3:
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
            qCurrent[0u] = (((uint32)pData[0u] << 16u) | \
                                  ((uint32)pData[1u] << 8u) | (uint32)pData[2u]);
            qCurrent[1u] = (((uint32)pData[3u] << 16u) | \
                              ((uint32)pData[4u] << 8u) | (uint32)pData[5u]);
            if((BQ7973X_FLIP_RESET_SINT24 != qCurrent[0u]) || (BQ7973X_FLIP_RESET_SINT24 != qCurrent[1u]))
            {
                uCheckReset = 1u;
            }
            //RetVal |= BQ7973X_GetVRrfCap(Bq7973x_ManagerType *pPmicMgr,const ServiceCfgType *pSvcCfg )
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL( BQ7973X_REF_CAP_HI_OFFSET, COMIF_LOCK),
                                       &pDiagMgr->pDiagCfg[PMI_MFDI_FLIP_RESET_DIAG],READ_2BYTE);

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE4;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE4:
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
            if(BQ7973X_FLIP_RESET_UINT16 != pData)
            {
                uCheckReset = 1u;
            }
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_D1_HI_OFFSET, COMIF_LOCK),
                                            &pDiagMgr->pDiagCfg[PMI_MFDI_FLIP_RESET_DIAG], READ_2BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE5;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE5:
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
            if(BQ7973X_FLIP_RESET_UINT16 != pData)
            {
                uCheckReset = 1u;
            }
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_D2_HI_OFFSET, COMIF_LOCK),
                                            &pDiagMgr->pDiagCfg[PMI_MFDI_FLIP_RESET_DIAG], READ_2BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE6;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE6:
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
            if(BQ7973X_FLIP_RESET_UINT16 != pData)
            {
                uCheckReset = 1u;
            }
           //RetVal |= Bq7973x_GetDieTemp(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL( BQ7973X_DIETEMP1_HI_OFFSET, COMIF_LOCK),
                                       &pDiagMgr->pDiagCfg[PMI_MFDI_FLIP_RESET_DIAG],READ_4BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE7;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE7:
        {
            if((uint8)PMI_MFDI_FLIP_RESET_DIAG == pSvcCfg->uSubId)
            {
                //(void)ComIface_AddCallbackFunc(BQ7973X_FlipResetCallBack, TD_ConfigPtr);
                pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                if((BQ7973X_FLIP_RESET_SINT16 != pData[0u]) || (BQ7973X_FLIP_RESET_SINT16 != pData[1u]))
                {
                    uCheckReset = 1u;
                }
                if (0u == uCheckReset)
                {
                    pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uSM_FLIP_RESET = (uint8)BQ7973X_DIAGNOERROR;
                }
                else
                {
                    pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uSM_FLIP_RESET = (uint8)BQ7973X_DIAGERROR;
                }

            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }

            /* Step5: Set ADC_CTRL3[FLIP_RESET] = 0b0. */
            uCmd = pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uAdc_Ctrl[2u] & (~BQ7973X_ADC_CTRL3_FLIP_RESET_MSK);

            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_CTRL3_OFFSET, COMIF_LOCK),
                                                &uCmd, WRITE_1BYTE);
            /* Step6: Set the [ADC_MODE] = 00, and set ADC_CTRL2[ADC_GO] = 1. */
            uCmd = (pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uAdc_Ctrl[1u] & (~BQ7973X_ADC_CTRL2_ADC_MODE_MSK)) | BQ7973X_ADC_CTRL2_ADC_GO_MSK;

            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_CTRL2_OFFSET, COMIF_LOCK),
                                                &uCmd, WRITE_1BYTE);

            /* Step7: Set [DIAG_D1D2_SEL] =000 and set ADC_CTRL4[DIAG_MEAS_GO] = 1. */
            uCmd = (pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uAdc_Ctrl[3u] & (~BQ7973X_ADC_CTRL4_DIAG_D1D2_SEL_MSK)) | BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_MSK;

            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_CTRL4_OFFSET, COMIF_LOCK),
                                                &uCmd, WRITE_1BYTE);

            RetVal |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_GPADC_DELAY);

            /* Step8: Read the ADC output register values and confirm the reset values */
            //RetVal |= Bq7973x_GetVGpio(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg) Ask Yehia
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL( BQ7973X_GPIO1_HI_OFFSET, COMIF_LOCK),
                                                   &pDiagMgr->pDiagCfg[PMI_MFDI_FLIP_RESET_DIAG],(READ_2BYTE * BQ7973X_GPIO_REG_NUM)); // TBC

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE8;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;

        }
        case (uint8)DIAG_xSTEP_STATE8:
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
            for(uIndex = 0u; uIndex < BQ7973X_GPIO_NUM_ACT; uIndex ++)
            {
                if(BQ7973X_FLIP_RESET_UINT16 != pData[uIndex])
                {
                    uCheckReset = 1u;
                }
            }
            //RetVal |= BQ7973X_GetVF(uActDevID, &BQ7973X_xVFCheck, TD_ConfigPtr);
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL( BQ7973X_VF2_HI_OFFSET, COMIF_LOCK),
                                       &pDiagMgr->pDiagCfg[PMI_MFDI_FLIP_RESET_DIAG],(BQ7973X_VF_REG_NUM * READ_2BYTE));

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE9;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE9:
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
            for(uIndex = 0u; uIndex < BQ7973X_VF_NUM_MAX; uIndex ++)
            {
                if(BQ7973X_FLIP_RESET_UINT16 != pData[uIndex])
                {
                    uCheckReset = 1u;
                }
            }
            //RetVal |= BQ7973X_GetCurrent(uActDevID, &BQ7973X_uCurrentCheck, TD_ConfigPtr);
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL( BQ7973X_CURRENT1_HI_OFFSET, COMIF_LOCK),
                                       &pDiagMgr->pDiagCfg[PMI_MFDI_FLIP_RESET_DIAG],(BQ7973X_CURRENT_REG_NUM * READ_3BYTE));

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE10;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }

        case (uint8)DIAG_xSTEP_STATE10:
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
            qCurrent[0u] = (((uint32)pData[0u] << 16u) | \
                                  ((uint32)pData[1u] << 8u) | (uint32)pData[2u]);
            qCurrent[1u] = (((uint32)pData[3u] << 16u) | \
                              ((uint32)pData[4u] << 8u) | (uint32)pData[5u]);
            if((BQ7973X_FLIP_RESET_SINT24 != qCurrent[0u]) || (BQ7973X_FLIP_RESET_SINT24 != qCurrent[1u]))
            {
                uCheckReset = 1u;
            }
            //RetVal |= BQ7973X_GetVRrfCap(uActDevID, &BQ7973X_xRef_CapCheck, TD_ConfigPtr);
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL( BQ7973X_REF_CAP_HI_OFFSET, COMIF_LOCK),
                                       &pDiagMgr->pDiagCfg[PMI_MFDI_FLIP_RESET_DIAG],READ_2BYTE);

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE11;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }

        case (uint8)DIAG_xSTEP_STATE11:
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
            if(BQ7973X_FLIP_RESET_UINT16 != pData)
            {
                uCheckReset = 1u;
            }
            //RetVal |= ComIface_SingleRd(uActDevID, BQ7973X_DIAG_D1_HI_OFFSET, BQ7973X_READ_2BYTE, &BQ7973X_xDiag_D1Check, TD_ConfigPtr);
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_D1_HI_OFFSET, COMIF_LOCK),
                                            &pDiagMgr->pDiagCfg[PMI_MFDI_FLIP_RESET_DIAG], READ_2BYTE);

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE12;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE12:
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
            if(BQ7973X_FLIP_RESET_UINT16 != pData)
            {
                uCheckReset = 1u;
            }
            //RetVal |= ComIface_SingleRd(uActDevID, BQ7973X_DIAG_D2_HI_OFFSET, BQ7973X_READ_2BYTE, &BQ7973X_xDiag_D2Check, TD_ConfigPtr);
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_D2_HI_OFFSET, COMIF_LOCK),
                                            &pDiagMgr->pDiagCfg[PMI_MFDI_FLIP_RESET_DIAG], READ_2BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE13;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE13:
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
            if(BQ7973X_FLIP_RESET_UINT16 != pData)
            {
                uCheckReset = 1u;
            }
            //RetVal |= BQ7973X_GetDieTemp(uActDevID, &BQ7973X_xDieTempCheck, TD_ConfigPtr);
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL( BQ7973X_DIETEMP1_HI_OFFSET, COMIF_LOCK),
                                       &pDiagMgr->pDiagCfg[PMI_MFDI_FLIP_RESET_DIAG],READ_4BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            if((uint8)PMI_MFDI_FLIP_RESET_DIAG == pSvcCfg->uSubId)
            {
                //(void)ComIface_AddCallbackFunc(BQ7973X_ResetCallBack, TD_ConfigPtr);
                pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                if((BQ7973X_FLIP_RESET_SINT16 != pData[0u]) || (BQ7973X_FLIP_RESET_SINT16 != pData[1u]))
                {
                    uCheckReset = 1u;
                }
                if (0u == uCheckReset)
                {
                    pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uSM_FLIP_RESET = (uint8)BQ7973X_DIAGNOERROR;
                }
                else
                {
                    pDiagMgr->pDiagResult[uDevIdx].zPMIMpfdtiResult.uSM_FLIP_RESET = (uint8)BQ7973X_DIAGERROR;
                }
            }
            else
            {
                RetVal = (uint8)PMI_NOT_OK;
            }
            /*RetVal |= BQ7973X_AdcCtrlInit(uActDevID, TD_ConfigPtr);*/
            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, BQ7973X_ADC_CTRL1_OFFSET,
                                       pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uAdc_Ctrl,
                                 BQ7973X_ADC_CTRL_REG_NUM);

            RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx,BQ7973X_OC_CTRL1_OFFSET,
                                            pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uOC_Ctrl, WRITE_1BYTE);

            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , BQ7973X_OC_CTRL1_OFFSET,
                                            &pDiagMgr->pDiagCfg[PMI_MFDI_FLIP_RESET_DIAG], READ_1BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
            }
            break;
        }
    }
        return Pmi_SetErrorDetails(PMI_MFDI_FLIP_RESET_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}
/*********************************************************************************************************************
 *  External Functions Definition
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_diag_CODE) Bq7973x_DiagMpfdi(Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
 *                                                   Bq7973x_DiagReqDataType *pDiagReq)
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

FUNC(uint8, bq7973x_diag_CODE) Bq7973x_DiagMpfdi(Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
                                                 Bq7973x_DiagReqType *pDiagReq)
{
    uint8 uRet;

    switch(pDiagReq->uDiagReqId)
    {
        case (uint8)PMI_MPFDI_ACOMP_FLTINJ_DIAG:
        {
            uRet = BQ7973X_AcompFltinjDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_MPFDI_DCOMP_FLTINJ_DIAG:
        {
            uRet = BQ7973X_DcompFltinjDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_MPFDI_BG1_DIAG:
        {
            uRet = BQ7973X_Bg1Diag( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_MPFDI_BG2_DIAG:
        {
            uRet = BQ7973X_Bg2Diag( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_MPFDI_REFCAP_DIAG:
        {
            uRet = BQ7973X_RefCapDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_MPFDI_OC_DET_DIAG:
        {
            uRet = BQ7973X_OCDetDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_MPFDI_OC_UNLOCK_DIAG:
        {
            uRet = BQ7973X_OCUnlockDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_MPFDI_SW_ADJ_DIAG:
        {  
            uRet = BQ7973X_SWAdjShortDiag( pDiagMgr, 1,pSvcReqCfg, pDiagReq); // TBD the Config param
            break;
        }
        case (uint8)PMI_MPFDI_SW_OTP_PROG:
        {
            uRet = BQ7973X_OTPProgram( pDiagMgr, pSvcReqCfg, pDiagReq,0);
            break;
        }
        case (uint8)PMI_MPFDI_VIF_CRC_DIAG:
        {
            uRet = BQ7973X_VIFCrcDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_MPFDI_SLEEP_FAULT_DIAG:
        {
            uRet = BQ7973X_SleepFaultDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_MPFDI_FACT_CRC_DIAG:
        {
           uRet = BQ7973X_FactCrcDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
           break;
        }
        case (uint8)PMI_MPFDI_PWR_BIST_DIAG:
        {
           uRet = BQ7973X_PwrBistDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
           break;
        }
        case (uint8)PMI_MFDI_FLIP_RESET_DIAG:
        {
           uRet = BQ7973X_FlipResetDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
           break;
        }
        default:
        {
            uRet = (uint8)PMI_PMC_DIAG_INVALID_CMD;
            break;
        }
    }

    return uRet;
}

#define BQ7973X_DIAGSTARTUP_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * End of File: Bq7973x_DiagMpfdi.c
 *********************************************************************************************************************/
