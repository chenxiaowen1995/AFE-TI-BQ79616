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
 *  File:       Bq7973x_DiagFdti.c
 *  Project:    TIBMS
 *  Module:     DIAG
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for PMI DIAG interface
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
#include "bq7973x_regs.h"
#include "Dio.h"
#include "Icu.h"

/**********************************************************************************************************************
**  LOCAL CONSTANT MACROS
**********************************************************************************************************************/
#define BQ7973X_WRITE_1BYTE              (1u)
#define BQ7973X_WRITE_2BYTE              (2u)
#define BQ7973X_WRITE_3BYTE              (3u)
#define BQ7973X_WRITE_4BYTE              (4u)



uint8 BQ7973X_uGpioConfRegData[BQ7973X_GPIO_CONF1_6_REG_NUM + BQ7973X_GPIO_CONF7_8_REG_NUM] = {0u};
uint8 BQ7973X_uVFOpenDiagEn = 0u;

/**********************************************************************************************************************
**  LOCAL DATA PROTOTYPES
**********************************************************************************************************************/
static uint8  BQ7973X_uDiagAdcCtrlRegData[2u] = {0u};                   /* 0x315 - 0x316 */ 

static uint16  BQ7973X_GPVoltage[BQ7973X_GPIO_REG_NUM];
static uint16  BQ7973X_GPVoltage_B[BQ7973X_GPIO_REG_NUM];
static uint16  BQ7973X_GPVoltage_C[BQ7973X_GPIO_REG_NUM];
static uint16  BQ7973X_VFVoltage[BQ7973X_VF_REG_NUM];
static uint16  BQ7973X_VFVoltage_B[BQ7973X_VF_REG_NUM];


static uint16  BQ7973X_InsulVch1 [16u];
static uint16  BQ7973X_InsulVhv1 [16u];
static float32 BQ7973X_InsulRisop[16u];
static float32 BQ7973X_InsulRison[16u];



/*********************************************************************************************************************
 *  Local Inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/
/*********************************************************************************************************************
 * Function name:  uint16 BQ7973X_CalAbsDifferOfVoltage(uint16 voltage1, uint16 voltage2)
 *********************************************************************************************************************/
/** \brief calculate the ads diff value of two voltage
 *
 * Extended function description
 *
 * \param[in] uint16 voltage1: first voltage input parameter
 * \param[in] uint16 voltage2: second voltage input parameter
 * \pre Preconditions
 * \post Postconditions
 * \return uint16 diffVol
 * \retval the diff voltage of input voltage1 and 2
 *
 *********************************************************************************************************************/
LOCAL_INLINE uint16 BQ7973X_CalAbsDifferOfVoltage(uint16 voltage1, uint16 voltage2)
{
    uint16  xDiffVol;

    /* VOL1 < 65534, The vol1 is valid */
    if (voltage1 < BQ7973X_VAL_INVALDVOLVALUE)
    {
        /* VOL2 < 32768, The vol2 is valid */
        if (voltage2 < BQ7973X_VAL_INVALDVOLVALUE)
        {
            /* vol1 less than vol2 */
            if (voltage1 <= voltage2)
            {
                xDiffVol = voltage2 - voltage1;
            }
            else
            {
                xDiffVol = voltage1 - voltage2;
            }
        }
        else
        {
            /* the vol2 is 0xFFFF, invalid value */
            xDiffVol = BQ7973X_VAL_INVALDVOLVALUE;
        }
    }
    else
    {
        /* the vol1 is 0xFFFF, invalid value */
        xDiffVol = BQ7973X_VAL_INVALDVOLVALUE;
    }

    return xDiffVol;
}
/*********************************************************************************************************************
 * Function name:  void BQ7973X_CheckInsulDet(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/** \brief This function is used to measure the insulation resistances Riso_P and Riso N*.
 *
 * Extended function description
 *
 * \param[in] 
 * \pre Preconditions
 * \post Postconditions
 * \return None
 * \retval None
 *
 *********************************************************************************************************************/
LOCAL_INLINE void BQ7973X_CheckInsulDet(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg , uint8 uIndex )
{
    uint16  Vch1, Vhv1, Vch2, Vhv2;
    float32 Var, Res1, Res2, Res3;
    float32 MulVarVch1Vhv2, MulVarVch2Vhv1, MulVarVch1Vch2;
    float32 PosNum1,PosNum2,negNum1,negNum2;
    uint8 *pData;
    //uint8 uDevIdx = pPmicMgr->uPmicDevId;
    Vch1 = BQ7973X_InsulVch1[uIndex];
    Vhv1 = BQ7973X_InsulVhv1[uIndex];
    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uIndex);
    Vch2 = (((uint16)pData[2u] << 8u) |(uint16)pData[3u]);
    Vhv2 = (((uint16)pData[2u] << 8u) |(uint16)pData[3u]);

    Res1 = (float32)BQ7973X_INSUL_RES1;
    Res2 = (float32)BQ7973X_INSUL_RES2;
    Res3 = (float32)BQ7973X_INSUL_RES3;
    Var = (BQ7973X_INSUL_RES2 + BQ7973X_INSUL_RES3) / BQ7973X_INSUL_RES3;
    MulVarVch1Vhv2 = Var * (float32)Vch1 * (float32)Vhv2;
    MulVarVch2Vhv1 = Var * (float32)Vch2 * (float32)Vhv1;
    MulVarVch1Vch2 = Var * (float32)Vch1 * (float32)Vch2;

    /*New Implementation like the GRC and For the Static Analysis Fix*/
    /* the following var shall be read in the API "BQ7973X_GetInsulDetResult" */
        PosNum1 = Var * MulVarVch1Vch2;
        negNum1 = MulVarVch2Vhv1;
        PosNum2 = (Res1 * MulVarVch2Vhv1) + ((Res2 + Res3) * ((Var * MulVarVch1Vch2) + ((float32)Vhv1 * (float32)Vhv2)));
        negNum2 = (Res1 * MulVarVch1Vhv2) + ((Res2 + Res3) * (MulVarVch1Vhv2 + MulVarVch2Vhv1));

        if((PosNum1 != negNum1) && (PosNum2 != negNum2))
        {
            BQ7973X_InsulRisop[uIndex] = -((Res1 * (MulVarVch1Vhv2 - MulVarVch2Vhv1)) / (PosNum1 - negNum1));
            BQ7973X_InsulRison[uIndex] = ((Res2 + Res3) * (Res1 * (MulVarVch1Vhv2 - MulVarVch2Vhv1))) / (PosNum2 - negNum2);
        }
        else
        {
            BQ7973X_InsulRisop[uIndex] = 0.0f;
            BQ7973X_InsulRison[uIndex] = 0.0f;
        }
//Old Implementation
//        BQ7973X_InsulRisop[uIndex] = -(((Res1 * MulVarVch1Vhv2) - (Res1 * MulVarVch2Vhv1)) /(PosNum1 - negNum1));
//           BQ7973X_InsulRison[uIndex] = ((Res2 + Res3) * ((Res1 * MulVarVch1Vhv2) - (Res1 * MulVarVch2Vhv1))) \
//                               / ((Res2 * Var * MulVarVch1Vch2) + (Res3 * Var * MulVarVch1Vch2) - (Res1 * MulVarVch1Vhv2) \
//                               + (Res1 * MulVarVch2Vhv1) - (Res2 * MulVarVch1Vhv2) -(Res2 * MulVarVch2Vhv1) \
//                               - (Res3 * MulVarVch1Vhv2) - (Res3 * MulVarVch2Vhv1) + (Res2 * (float32)Vhv1 \
//                               * (float32)Vhv2) + (Res3 * (float32)Vhv1 * (float32)Vhv2));
    // not used
//    if (BQ7973X_InsulRisop[0] || BQ7973X_InsulRison[0])
//    {
//
//    }
}
/*********************************************************************************************************************
 * Function name:  uint16 BQ7973X_CalAbsDifferOfTemp(uint16 xTemp1, uint16 xTemp2)
 *********************************************************************************************************************/
/** \brief calculate the ads diff value of two temperature register value
 *
 * Extended function description
 *
 * \param[in] uint16 xTemp1: first temperature register value
 * \param[in] uint16 xTemp2: second temperature register value
 * \pre Preconditions
 * \post Postconditions
 * \return uint16 xDiffTemp
 * \retval the diff temperature of input temperature1 and 2
 *
 *********************************************************************************************************************/
LOCAL_INLINE uint16 BQ7973X_CalAbsDifferOfTemp(uint16 xTemp1, uint16 xTemp2)
{
    uint16  xDiffTemp;

    /* xTemp1 < 32768, The xTemp1 is positive */
    if (xTemp1 < BQ7973X_INVALDDIETEMPVALUE)
    {
        /* xTemp2 < 32768, The xTemp2 is positive */
        if (xTemp2 < BQ7973X_INVALDDIETEMPVALUE)
        {
            /* xTemp1 less than xTemp2 */
            if (xTemp1 <= xTemp2)
            {
                xDiffTemp = xTemp2 - xTemp1;
            }
            else
            {
                xDiffTemp = xTemp1 - xTemp2;
            }
        }
        /* xTemp2 > 32768, The xTemp2 is negative */
        else if (xTemp2 > BQ7973X_INVALDDIETEMPVALUE)
        {
            xDiffTemp = ((BQ7973X_INVALDVOLVALUE - xTemp2) + 1u) + xTemp1;
        }
        else
        {
            /* the xTemp2 is 0x8000, invalid value */
            xDiffTemp = BQ7973X_INVALDVOLVALUE;
        }
    }
    /* xTemp1 > 32768, The xTemp1 is negative */
    else if (xTemp1 > BQ7973X_INVALDDIETEMPVALUE)
    {
        /* xTemp2 < 32768, The xTemp2 is positive */
        if (xTemp2 < BQ7973X_INVALDDIETEMPVALUE)
        {
            xDiffTemp = ((BQ7973X_INVALDVOLVALUE - xTemp1) + 1u) + xTemp2;
        }
        /* xTemp2 > 32768, The xTemp2 is negative */
        else if (xTemp2 > BQ7973X_INVALDDIETEMPVALUE)
        {
            /* xTemp1 less than xTemp2 */
            if (xTemp1 <= xTemp2)
            {
                xDiffTemp = xTemp2 - xTemp1;
            }
            else
            {
                xDiffTemp = xTemp1 - xTemp2;
            }
        }
        else
        {
            /* the xTemp2 is 0x8000, invalid value */
            xDiffTemp = BQ7973X_INVALDVOLVALUE;
        }
    }
    else
    {
        /* the xTemp1 is 0x8000, invalid value */
        xDiffTemp = BQ7973X_INVALDVOLVALUE;
    }
    return xDiffTemp;
}
/*********************************************************************************************************************
 * Function name:  uint8 BQ7973X_CheckDieTemp(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/** \brief Check the Die Temp1/Temp2 plausibility diagnosis result
 * 
 * Extended function description
 *
 * \param[in] 
 * \pre Preconditions
 * \post Postconditions
 * \return None
 * \retval None
 *
 *********************************************************************************************************************/
LOCAL_INLINE uint8 BQ7973X_CheckDieTemp( Bq7973x_DiagType  * const pDiagMgr, const ServiceCfgType *pSvcCfg , uint8 uIndex )
{
    static uint8 BQ7973X_uDieTempPlauDebounce;
    static uint16  BQ7973X_PreDieTemp[BQ7973X_DIE_TEMP_NUM];
    uint16  xDieTemp[2u] = {0u};
    uint8 *pData;
    pDiagMgr->uCheckTimesNum++;

    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uIndex);
    xDieTemp[0u] = (((uint16)pData[0u] << 8u) |(uint16)pData[1u]);
    xDieTemp[1u] = (((uint16)pData[2u] << 8u) |(uint16)pData[3u]);
    if(1u == pDiagMgr->uCheckTimesNum)
    {
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DIE_TEMP = BQ7973X_DIAGNOERROR;
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DIE_TEMP_DIFF = BQ7973X_DIAGNOERROR;
        BQ7973X_uDieTempPlauDebounce = BQ7973X_DIAGNOERROR;
    }
    else if(2u == pDiagMgr->uCheckTimesNum)
    {
        BQ7973X_uDieTempPlauDebounce = (1u << BQ7973X_DIE_TEMP_NUM) - 1u;
    }
    else
    {
        /* Nothing */
    }

    for (uIndex = 0u; uIndex < BQ7973X_DIE_TEMP_NUM; uIndex ++)
    {
        if(BQ7973X_INVALDDIETEMPVALUE == xDieTemp[uIndex])
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DIE_TEMP |= (uint8) (BQ7973X_DIAGERROR << uIndex);
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DIE_TEMP_DIFF = BQ7973X_DIAGERROR;
        }
        else if((xDieTemp[uIndex] > BQ7973X_MAXDIETEMPPLAUVALUE) && (xDieTemp[uIndex] < BQ7973X_MINDIETEMPPLAUVALUE))
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DIE_TEMP |= (uint8) (BQ7973X_DIAGERROR << uIndex);
        }
        else
        {
            /* Nothing */
        }

        if((pDiagMgr->uCheckTimesNum > 1u) && (BQ7973X_PreDieTemp[uIndex] != xDieTemp[uIndex]))
        {
            BQ7973X_uDieTempPlauDebounce &= (uint8) (~(BQ7973X_DIAGERROR << uIndex));
        }
        BQ7973X_PreDieTemp[uIndex] = xDieTemp[uIndex];
    }

    if(BQ7973X_CalAbsDifferOfTemp(xDieTemp[0u], xDieTemp[1u]) > BQ7973X_TEMPDIFFPLAUVALUE)
    {
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DIE_TEMP_DIFF = BQ7973X_DIAGERROR;
    }

    if(BQ7973X_DIETEMP_CHECKTIMES == pDiagMgr->uCheckTimesNum)
    {
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DIE_TEMP |= BQ7973X_uDieTempPlauDebounce;
        pDiagMgr->uCheckTimesNum = 0u;
    }

    return 0;

}

/*********************************************************************************************************************
 * Function name:  void BQ7973X_CheckOC_OS(Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/** \brief Check about OC short/open diagnosis results.
 *
 * Extended function description
 *
 * \param[in] 
 * \pre Preconditions
 * \post Postconditions
 * \return None
 * \retval None
 *
 *********************************************************************************************************************/
LOCAL_INLINE void BQ7973X_CheckOC_OS(Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg)
{
    static Icu_DutyCycleType   BQ7973X_OC_PWM_Duty[2u];
    uint32  qOCPWMDuty[BQ7973X_OC_NUM_ACT];
    uint8   uFaultOC[BQ7973X_OC_NUM_ACT];
    uint8   uIndex;

    Icu_GetDutyCycleValues(BQ7973X_ICU_OC1CHANNEL, &BQ7973X_OC_PWM_Duty[0u]);
    Icu_GetDutyCycleValues(BQ7973X_ICU_OC2CHANNEL, &BQ7973X_OC_PWM_Duty[1u]);
    uFaultOC[0u] = (pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CS_OCC1 | pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CS_OCD1);
    uFaultOC[1u] = (pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CS_OCC2 | pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CS_OCD2);

    for (uIndex = 0u; uIndex < BQ7973X_OC_NUM_ACT; uIndex ++)
    {
        if(0u == BQ7973X_OC_PWM_Duty[uIndex].PeriodTime)
        {
            qOCPWMDuty[uIndex] = 0u;
        }
        else
        {
            qOCPWMDuty[uIndex] = (BQ7973X_OC_PWM_Duty[uIndex].ActiveTime * BQ7973X_OCPWMPERCENT) / BQ7973X_OC_PWM_Duty[uIndex].PeriodTime;
        }
        
        if(0u == uFaultOC[uIndex])
        {
            if((qOCPWMDuty[uIndex] < (BQ7973X_NOOCFAULTPWM - 1u)) || (qOCPWMDuty[uIndex] > BQ7973X_NOOCFAULTPWM))
            {
                pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OC_OS |= (uint8) (BQ7973X_DIAGERROR << uIndex);
                
            }
            else
            {
                pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OC_OS &= (uint8) (~(BQ7973X_DIAGERROR << uIndex));
            }
        }
        else
        {
            if((qOCPWMDuty[uIndex] < (BQ7973X_OCFAULTPWM - 1u)) || (qOCPWMDuty[uIndex] > BQ7973X_OCFAULTPWM))
            {
                pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OC_OS |= (uint8) (BQ7973X_DIAGERROR << uIndex);
            }
            else
            {
                pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OC_OS &= (uint8) (~(BQ7973X_DIAGERROR << uIndex));
            }
        }
    }
}
/*********************************************************************************************************************
 * Function name:  void BQ7973X_CheckGpioAdjShort(void)
 *********************************************************************************************************************/
/** \brief Check the GPIO pin to pin short diagnosis result
 *
 * Extended function description
 *
 * \param[in] 
 * \pre Preconditions
 * \post Postconditions
 * \return None
 * \retval None
 *
 *********************************************************************************************************************/
LOCAL_INLINE void BQ7973X_CheckGpioAdjShort(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,uint8  uDevIdx)
{
    uint8  uGpioIndex =1u;
    /* Traverse all gpio */
    //for (uGpioIndex = 1u; uGpioIndex < BQ7973X_GPIO_NUM_ACT; uGpioIndex += 2u)
    while (uGpioIndex < BQ7973X_GPIO_NUM_ACT)
    {  
        if (BQ7973X_CalAbsDifferOfVoltage(BQ7973X_GPVoltage[uGpioIndex], BQ7973X_GPVoltage_B[uGpioIndex]) >= BQ7973X_GPADJSHORT_THRESHOLD)
        {
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.xSM_GP_ADJSHRT |= ((uint16)BQ7973X_DIAGERROR << uGpioIndex);
        }
        else
        {
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.xSM_GP_ADJSHRT &= (~((uint16)BQ7973X_DIAGERROR << uGpioIndex));
        }
        uGpioIndex += 2u;
    }
    if (BQ7973X_CalAbsDifferOfVoltage(BQ7973X_VFVoltage[1u], BQ7973X_VFVoltage_B[1u]) >= BQ7973X_GPADJSHORT_THRESHOLD)
    {
        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_GPVF_ADJSHRT = BQ7973X_DIAGERROR;
    }
    else
    {
        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_GPVF_ADJSHRT = BQ7973X_DIAGNOERROR;
    }
    if(pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DRDY_GP > 0u)
    {
        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.xSM_GP_ADJSHRT = ((uint16)1u << BQ7973X_GPIO_NUM_ACT) - 1u;
        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_GPVF_ADJSHRT = BQ7973X_DIAGERROR;
    }
}

/*********************************************************************************************************************
 * Function name:  void BQ7973X_CheckGPPlau(Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/** \brief Check the Vn_GPIOn Input plausibility diagnosis result
 * 
 * Extended function description
 *
 * \param[in] 
 * \pre Preconditions
 * \post Postconditions
 * \return None
 * \retval None
 *
 *********************************************************************************************************************/
LOCAL_INLINE void BQ7973X_CheckGPPlau( Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg)
{
    static uint16 BQ7973X_xGPPlauDebounce;
    static uint16  BQ7973X_PreGPVoltage[BQ7973X_GPIO_NUM_ACT];
    uint16  xGpioVoltage[BQ7973X_GPIO_NUM_ACT];
    uint8   uIndex;
    uint8 *pData;

    pDiagMgr->uCheckTimesNum++;
    if(1u == pDiagMgr->uCheckTimesNum)
    {
        pDiagMgr->pDiagResult->zPMIFdtiResult.xSM_GP_PLAU = BQ7973X_DIAGNOERROR;
        BQ7973X_xGPPlauDebounce = BQ7973X_DIAGNOERROR;
    }
    else if(2u == pDiagMgr->uCheckTimesNum)
    {
        BQ7973X_xGPPlauDebounce = ((uint16)1u << BQ7973X_GPIO_NUM_ACT) - 1u;
    }
    else
    {
        /* Nothing */
    }

    for (uIndex = 0u; uIndex < BQ7973X_GPIO_NUM_ACT; uIndex ++)
    {
        pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uIndex);
        xGpioVoltage[uIndex] = (((uint16)pData[uIndex * 2u] << 8u) |(uint16)pData[(uIndex * 2u) + 1u]);

        if(BQ7973X_INVALDVOLVALUE == xGpioVoltage[uIndex])
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.xSM_GP_PLAU |= ((uint16)BQ7973X_DIAGERROR << uIndex);
        }
        else if((xGpioVoltage[uIndex] > BQ7973X_MAXGPPLAUVALUE) || (xGpioVoltage[uIndex] < BQ7973X_MINGPPLAUVALUE))
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.xSM_GP_PLAU |= ((uint16)BQ7973X_DIAGERROR << uIndex);
        }
        else
        {
            /* Nothing */
        }

        if((pDiagMgr->uCheckTimesNum > 1u) && (BQ7973X_PreGPVoltage[uIndex] != xGpioVoltage[uIndex]))
        {
            BQ7973X_xGPPlauDebounce &= (~((uint16)BQ7973X_DIAGERROR << uIndex));
        }
        BQ7973X_PreGPVoltage[uIndex] = xGpioVoltage[uIndex];
    }

    if(BQ7973X_GPPLAU_CHECKTIMES == pDiagMgr->uCheckTimesNum)
    {
        pDiagMgr->pDiagResult->zPMIFdtiResult.xSM_GP_PLAU |= BQ7973X_xGPPlauDebounce;
        pDiagMgr->uCheckTimesNum = 0u;
    }
}

/*********************************************************************************************************************
 * Function name:  void BQ7973X_CheckGpioOpenWire(Bq7973x_DiagType *pDiagMgr)
 *********************************************************************************************************************/
/** \brief Check the Gpio Open Wire diagnosis result
 *
 * Extended function description
 *
 * \param[in] 
 * \pre Preconditions
 * \post Postconditions
 * \return None
 * \retval None
 *
 *********************************************************************************************************************/
LOCAL_INLINE void BQ7973X_CheckGpioOpenWire(const Bq7973x_DiagType *pDiagMgr)
{
    uint8   uGpioIndex;
    uint16  xAbsVoltAB;
    uint16  xAbsVoltAC;

    /* Traverse all gpio */
    for (uGpioIndex = 0u; uGpioIndex < BQ7973X_GPIO_NUM_ACT; uGpioIndex++)
    {
        xAbsVoltAB = BQ7973X_CalAbsDifferOfVoltage(BQ7973X_GPVoltage[uGpioIndex], BQ7973X_GPVoltage_B[uGpioIndex]);
        xAbsVoltAC = BQ7973X_CalAbsDifferOfVoltage(BQ7973X_GPVoltage[uGpioIndex], BQ7973X_GPVoltage_C[uGpioIndex]);

        if ((xAbsVoltAB >= BQ7973X_GPOPENWIRE_THRESHOLD) || (xAbsVoltAC >= BQ7973X_GPOPENWIRE_THRESHOLD))
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.xSM_GP_OPEN |= ((uint16)BQ7973X_DIAGERROR << uGpioIndex);
        }
        else
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.xSM_GP_OPEN &= (~((uint16)BQ7973X_DIAGERROR << uGpioIndex));
        }
    }
    if(pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DRDY_GP > 0u)
    {
        pDiagMgr->pDiagResult->zPMIFdtiResult.xSM_GP_OPEN = ((uint16)1u << BQ7973X_GPIO_NUM_ACT) - 1u;
    }
}

/*********************************************************************************************************************
 * Function name:  void BQ7973X_CheckVFPlau(Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/** \brief Check the VF Input plausibility diagnosis result
 * 
 * Extended function description
 *
 * \param[in] 
 * \pre Preconditions
 * \post Postconditions
 * \return None
 * \retval None
 *
 *********************************************************************************************************************/
LOCAL_INLINE void BQ7973X_CheckVFPlau( Bq7973x_DiagType * const pDiagMgr, const ServiceCfgType *pSvcCfg)
{
    uint8 *pData;
    uint8 uIndex ;
    static uint8 BQ7973X_uVFPlauDebounce;
    static uint16  BQ7973X_PreVFVoltage[BQ7973X_VF_NUM_ACT];
    uint16  xVFVoltage[BQ7973X_VF_NUM_ACT];

    pDiagMgr->uCheckTimesNum++;
    if(1u == pDiagMgr->uCheckTimesNum)
    {
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VF_PLAU = BQ7973X_DIAGNOERROR;
        BQ7973X_uVFPlauDebounce = BQ7973X_DIAGNOERROR;
    }
    else if(2u == pDiagMgr->uCheckTimesNum)
    {
        BQ7973X_uVFPlauDebounce = (1u << BQ7973X_VF_NUM_ACT) - 1u;
    }
    else
    {
        /* Nothing */
    }

    for (uIndex = 0u; uIndex < BQ7973X_VF_NUM_ACT; uIndex ++)
    {
        // TBC shall we add another for loop for all the device ? what is the index ID ?
        pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uIndex); 
        xVFVoltage[uIndex] = (((uint16)pData[uIndex * 2u] << 8u) |(uint16)pData[(uIndex * 2u) + 1u]);

        if(BQ7973X_INVALDVOLVALUE == xVFVoltage[uIndex])
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VF_PLAU |= ((uint8)BQ7973X_DIAGERROR << uIndex);
        }
        else if((xVFVoltage[uIndex] > BQ7973X_MAXVFPLAUVALUE) || (xVFVoltage[uIndex] < BQ7973X_MINVFPLAUVALUE))
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VF_PLAU |= ((uint8)BQ7973X_DIAGERROR << uIndex);
        }
        else
        {
            /* Nothing */
        }
        if((pDiagMgr->uCheckTimesNum > 1u) && (BQ7973X_PreVFVoltage[uIndex] != xVFVoltage[uIndex]))
        {
            BQ7973X_uVFPlauDebounce &= (~((uint8)BQ7973X_DIAGERROR << uIndex));
        }
        BQ7973X_PreVFVoltage[uIndex] = xVFVoltage[uIndex];
    }

    if(BQ7973X_VFPLAU_CHECKTIMES == pDiagMgr->uCheckTimesNum)
    {
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VF_PLAU |= BQ7973X_uVFPlauDebounce;
        if(BQ7973X_uVFOpenDiagEn > 0u)
        {
            BQ7973X_uVFOpenDiagEn = 0u;
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VF_OPEN = pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VF_PLAU;
        }
        pDiagMgr->uCheckTimesNum = 0u;
    }
}
/*********************************************************************************************************************
 * Function name:  void BQ7973X_CheckCommDebug(Bq7973x_DiagType *pDiagMgr,const ServiceCfgType *pSvcCfg, uint8 uDevIdx)
 *********************************************************************************************************************/
/** \brief Check about DEBUG_UART and DEBUG_COM* diagnosis results.
 *
 * Extended function description
 *
 * \param[in] 
 * \pre Preconditions
 * \post Postconditions
 * \return None
 * \retval None
 *
 *********************************************************************************************************************/

LOCAL_INLINE void BQ7973X_CheckCommDebug(const Bq7973x_DiagType *pDiagMgr,const ServiceCfgType *pSvcCfg, uint8 uDevIdx)
{
    uint8   uFaultSummary;
    uint8   uDebugUart;
    uint8   *pData;
    uint8   uDebugComh[3u];
    uint8   uDebugComl[3u];
    uint8   uPerr, uBit, uCrc, uWait;
    uint8   uUnexp, uTxdis, uSync1, uSync2;
    uint8   uSof, uIerr, uBerr;
    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
    uFaultSummary =  pData[0] & BQ7973X_FAULT_SUMMARY_FAULT_COMM_MSK;
    if(0u == uFaultSummary)
    {
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_TXDIS    = (uint8)  PMI_OK;
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_WAIT     = (uint8)  PMI_OK;
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_UNEXP    = (uint8)  PMI_OK;
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VIF_BYTE      = (uint8)  PMI_OK;
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_SYNC2    = (uint8)  PMI_OK;
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_SYNC1    = (uint8)  PMI_OK;
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VIF_CRC       = (uint8)  PMI_OK;
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_IERR     = (uint8)  PMI_OK;
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VIF_BIT       = (uint8)  PMI_OK;
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_BERR     = (uint8)  PMI_OK;
        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_SOF      = (uint8)  PMI_OK;
    }
    else
    {
        uDebugUart = pData[0u];
        uDebugComh[0u] = pData[0u];
        uDebugComh[1u] = pData[1u];
        uDebugComh[2u] = pData[2u];
        uDebugComl[0u] = pData[0u];
        uDebugComl[1u] = pData[1u];
        uDebugComl[2u] = pData[2u];
        uTxdis = ((uDebugUart | uDebugComh[1u] | uDebugComh[2u] | uDebugComl[1u] | uDebugComl[2u]) & BQ7973X_DEBUG_UART_RC_TR_RC_TXDIS_MSK);
        if(0u == uTxdis)
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_TXDIS = (uint8)  PMI_OK;
        }
        else
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_TXDIS = (uint8)  PMI_NOT_OK;
        }
        uWait = ((uDebugUart | uDebugComh[1u] | uDebugComl[1u]) & BQ7973X_DEBUG_UART_RC_TR_TR_WAIT_MSK);
        if(0u == uWait)
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_WAIT = (uint8)  PMI_OK;
        }
        else
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_WAIT = (uint8)  PMI_NOT_OK;
        }
        uUnexp = ((uDebugUart | uDebugComh[1u] | uDebugComh[2u] | uDebugComl[1u] | uDebugComl[2u]) & BQ7973X_DEBUG_UART_RC_TR_RC_UNEXP_MSK);
        if(0u == uUnexp)
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_UNEXP = (uint8)  PMI_OK;
        }
        else
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_UNEXP = (uint8)  PMI_NOT_OK;
        }
        uPerr = ((uDebugComh[0u] | uDebugComl[0u]) & BQ7973X_DEBUG_COMH_BIT_PERR_MSK);
        if(0u == uPerr)
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VIF_BYTE = (uint8)  PMI_OK;
        }
        else
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VIF_BYTE = (uint8)  PMI_NOT_OK;
        }
        uSync2 = ((uDebugComh[0u] | uDebugComl[0u]) & BQ7973X_DEBUG_COMH_BIT_SYNC2_MSK);
        if(0u == uSync2)
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_SYNC2 = (uint8)  PMI_OK;
        }
        else
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_SYNC2 = (uint8)  PMI_NOT_OK;
        }
        uSync1 = ((uDebugComh[0u] | uDebugComl[0u]) & BQ7973X_DEBUG_COMH_BIT_SYNC1_MSK);
        if(0u == uSync1)
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_SYNC1 = (uint8)  PMI_OK;
        }
        else
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_SYNC1 = (uint8) PMI_NOT_OK;
        }
        uCrc = ((uDebugComh[1u] | uDebugComh[2u] | uDebugComl[1u] | uDebugComl[2u]) & BQ7973X_DEBUG_COMH_RC_TR_RC_CRC_MSK);
        if(0u == uCrc)
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VIF_CRC = (uint8)  PMI_OK;
        }
        else
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VIF_CRC = (uint8) PMI_NOT_OK;
        }
        uIerr = ((uDebugUart | uDebugComh[1u] | uDebugComl[1u]) & BQ7973X_DEBUG_UART_RC_TR_RC_IERR_MSK);
        if(0u == uIerr)
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_IERR = (uint8)  PMI_OK;
        }
        else
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_IERR =  (uint8) PMI_NOT_OK;
        }
        uBit = ((uDebugComh[0u] | uDebugComl[0u]) & BQ7973X_DEBUG_COMH_BIT_BIT_MSK);
        if(0u == uBit)
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VIF_BIT = (uint8) PMI_OK;
        }
        else
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VIF_BIT = (uint8) PMI_NOT_OK;
        }
        uBerr = ((uDebugUart | uDebugComh[1u] | uDebugComh[2u] | uDebugComl[1u] | uDebugComl[2u]) & BQ7973X_DEBUG_UART_RC_TR_RC_BYTE_ERR_MSK);
        if(0u == uBerr)
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_BERR = (uint8) PMI_OK;
        }
        else
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_BERR = (uint8) PMI_NOT_OK;
        }
        uSof = ((uDebugUart | uDebugComh[1u] | uDebugComh[2u] | uDebugComl[1u] | uDebugComl[2u])
                & BQ7973X_DEBUG_UART_RC_TR_RC_SOF_MSK) | (uDebugUart & BQ7973X_DEBUG_UART_RC_TR_TR_SOF_MSK);
        if(0u == uSof)
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_SOF = (uint8) PMI_OK;
        }
        else
        {
            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_COMM_SOF = (uint8) PMI_NOT_OK;
        }
    }
}


/*********************************************************************************************************************
 * Functions Definition
 *********************************************************************************************************************/


/*********************************************************************************************************************
 * Function name:  void AcompDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief This function is used to detect faults in VF and GPIO measurement analog diagnostic.
 *
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \uRet         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/
STATIC FUNC(uint8, diag_CODE) AcompDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                  Bq7973x_DiagReqType *pDiagReq)
{

    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint8 uCmd;
    uint8 uRet = (uint8)PMI_OK;
    static uint8 uDiagAnaRunErr = 0u;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            /* BQ7973X_CheckGpioAcompCallBack : to Check GPIO ACOMP diagnostic result */
            if((uint8)PMI_FDTI_ACOMP_DIAG == pSvcCfg->uSubId)
            {
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_ACOMP_GPIO = ((uint16)pData[0u] << 8u) | ((uint16)pData[1u]);
                }
                if((uDiagAnaRunErr | pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DRDY_ANA_VF) > 0u)
                {
                    pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_ACOMP_GPIO = ((uint16)1u << BQ7973X_GPIO_NUM_ACT) - 1u;
                } 

                /* Step6: Reset the FAULT_ADC_* registers. */
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                uCmd = BQ7973X_DIAG_ADC_CTRL2_DIAG_DIG_EN_MSK | BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_GO_MSK;
                BQ7973X_uDiagAdcCtrlRegData[1u] = (uint8) ( uCmd & (~BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_GO_MSK));
                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, 
                                COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_ADC_CTRL2_OFFSET, COMIF_LOCK), BQ7973X_uDiagAdcCtrlRegData, WRITE_1BYTE);

                uCmd = BQ7973X_FAULT_RST2_RST_ADC_MSK;
                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, 
                                COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST2_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);

                }
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    uRet = (uint8) PMI_OK;
                }
            }
            break;
        }

        case (uint8)DIAG_xSTEP_STATE3:
        {
            if((uint8)PMI_FDTI_ACOMP_DIAG == pSvcCfg->uSubId)
            {
                //for  (uDevIdx = 0u; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);

                    //TBC check if this element uSM_ACOMP_VCELL shall be replaced with uSM_VF_ACOMP    
                    pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_ACOMP_VCELL = pData[0u]; 
                
                    if((uDiagAnaRunErr | pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DRDY_ANA_VF) > 0u) 
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VF_ACOMP = (1u << BQ7973X_VF_NUM_ACT) - 1u;
                    }
                }
                  /* Step5: Check FAULT_ADC_GPIO* registers for the diagnostic result. */
                
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_ADC_GPIO1_OFFSET, COMIF_LOCK) ,
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_ACOMP_DIAG], READ_2BYTE);
                
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE2:
        {
            if((uint8)PMI_FDTI_ACOMP_DIAG == pSvcCfg->uSubId)
            {
                //for  (uDevIdx = 0u; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                     if(0u == (pData[1u] & BQ7973X_DIAG_STAT2_DRDY_ANA_VF_MSK ))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DRDY_ANA_VF = DIAGERROR;
                        uRet = E_NOT_OK;
                    }
                    else
                    {
                        uRet = E_OK;
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DRDY_ANA_VF = DIAGNOERROR;
                    }
                        if(0u == (pData[1u] & BQ7973X_DIAG_STAT2_DRDY_ANA_GPIO_MSK ))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DRDY_ANA_GP = DIAGERROR;
                        uRet = E_NOT_OK;
                    }
                    else
                    {
                        uRet = E_OK;
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DRDY_ANA_GP = DIAGNOERROR;
                    }
                }
                 /* Step4: Check FAULT_ADC_VF registers for the diagnostic result.*/
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_ADC_VF_OFFSET, COMIF_LOCK) ,
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_ACOMP_DIAG], READ_1BYTE);
                
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE3;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE1:
        {
            if((uint8)PMI_FDTI_ACOMP_DIAG == pSvcCfg->uSubId)
            {
                 uDiagAnaRunErr = 0u ;
                //for  (uDevIdx = 0u; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if(0u == (pData[1u] & BQ7973X_DEV_STAT1_DIAG_ANA_RUN_MSK))
                    {
                        uRet = (uint8)PMI_NOT_OK;
                        uDiagAnaRunErr = 1u ; //TBC this flag usage
                    }
                    else
                    {
                        uRet = (uint8) PMI_OK;
                    }

                }
                 /* Step3: Wait for the comparison to be done. */
                 (void)Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_ACOMP_DONE_DELAY);

                    uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  ,COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_STAT1_OFFSET, COMIF_LOCK) ,
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_ACOMP_DIAG], READ_2BYTE);

                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE2;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
     case (uint8)DIAG_xSTEP_READY:
        {
             /* Step1: Select Both VF voltage measurement and GPIO measurement analog path diagnostic
              and Enable the device ADC digital path diagnostic. */
            //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)


            {
                uCmd = BQ7973X_DIAG_ADC_CTRL2_DIAG_DIG_EN_MSK | BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_MODE_MSK | BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_GO_MSK;
                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, 
                                COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_ADC_CTRL2_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
            }
                /* Step2: Make sure the diagnostic is running. */
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  ,COMIF_PRIO_EXCL_CELL(BQ7973X_DEV_STAT1_OFFSET, COMIF_LOCK) ,
                                         &pDiagMgr->pDiagCfg[PMI_FDTI_ACOMP_DIAG], READ_2BYTE);
            
            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
     default:
        {
            break;
        }

    }
    return Pmi_SetErrorDetails(PMI_FDTI_ACOMP_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);

}

/*********************************************************************************************************************
 * Function name:  void DcompDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief API to perform VF voltage digital comparison and GPIO voltage digital comparison.
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
STATIC FUNC(uint8, diag_CODE) DcompDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                        Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 testData[2];
    uint8 uDevIdx = 0u;
    uint8 uCmd;
    uint8 uDiagDigRunErr = 0u ;
    uint8 uRet = (uint8) PMI_OK;

    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_FINISHED:
        {

            if((uint8)PMI_FDTI_DCOMP_DIAG == pSvcCfg->uSubId)
            {
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VF_DCOMP = pData[2u];
                    pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_GP_DCOMP = (pData[0u] >> BQ7973X_FAULT_ADC_DIG1_GP1_DFAIL_POS);

                }
                   //TBC this flag usage 
                if((uDiagDigRunErr | pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DRDY_DIG) > 0u)
                {
                    pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VF_DCOMP = (1u << BQ7973X_VF_NUM_ACT) - 1u;
                    pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_GP_DCOMP = (1u << BQ7973X_GP_ADC_NUM) - 1u;
                }

                /* Step8: Reset the FAULT_ADC_* registers. */
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    uCmd = BQ7973X_ADC_CTRL2_ADC_GO_MSK;
                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_CTRL2_OFFSET ,
                COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                }
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8) PMI_DIAG_COMPLETE;
                    uRet = (uint8) PMI_OK;
                }
            }
            break;
        }

   case (uint8)DIAG_xSTEP_STATE2:
        {
            if((uint8)PMI_FDTI_DCOMP_DIAG == pSvcCfg->uSubId)
            {

                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);

                    if(0u == (pData[1u] & BQ7973X_DIAG_STAT2_DRDY_DIG_MSK ))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DRDY_DIG |= DIAGERROR;
                        uRet = (uint8)PMI_NOT_OK;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DRDY_DIG |= DIAGNOERROR;
                        uRet = (uint8)PMI_OK;
                    }
                }
                /* Step7: Check FAULT_ADC_DIG for the diagnostic result [ADC*_DFAIL]. */
                uRet |=Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_ADC_DIG1_OFFSET, COMIF_LOCK)
                                     , &pDiagMgr->pDiagCfg[PMI_FDTI_DCOMP_DIAG], READ_3BYTE);
                
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE1:
        {
            uDiagDigRunErr = 0u ;
            if((uint8)PMI_FDTI_DCOMP_DIAG == pSvcCfg->uSubId)
            {
                //for  (uDevIdx = 0u; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    testData[0]=pData[0];
                    testData[1]=pData[1];
                    if(0u == (testData[1u] & BQ7973X_DEV_STAT2_DIAG_DIG_RUN_MSK))
                    {
                        uRet = (uint8)PMI_NOT_OK;
                        uDiagDigRunErr = 1u ;
                    }
                    else
                    {
                        uRet = (uint8)PMI_OK;
                    }

                }
                /* Step6: Wait for the comparison to be done. */

                uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_VF_OPEN_DELAY);

                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx,  pDiagMgr->sDevId  ,COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_STAT1_OFFSET, COMIF_LOCK) ,
                                     &pDiagMgr->pDiagCfg[PMI_FDTI_DCOMP_DIAG], READ_2BYTE);
                
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE2;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
     case (uint8)DIAG_xSTEP_READY:

        {
            uCmd = BQ7973X_DIAG_ADC_CTRL2_DIAG_DIG_EN_MSK ; //BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_GO_MSK ;
            uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_ADC_CTRL2_OFFSET,
                                COMIF_UNLOCK), &uCmd, WRITE_1BYTE);


             /* Step5: Make sure the diagnostic is running. */
            uRet |= Comif_SingleRead(pDiagMgr->pComifCtx,  pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_DEV_STAT1_OFFSET, COMIF_LOCK) ,
                                     &pDiagMgr->pDiagCfg[PMI_FDTI_DCOMP_DIAG], READ_2BYTE);
            

            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
     default:
            {
                break;
            }
    }
       return Pmi_SetErrorDetails(PMI_FDTI_DCOMP_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************
 * Function name:  uint8 CommDebugDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief This function is used to check about DEBUG_UART and DEBUG_COM* diagnosis.
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
STATIC FUNC(uint8, bq7973x_diag_CODE) CommDebugDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7973x_DiagReqType *pDiagReq)
{
    uint8 uCmd;
    uint8 uDevIdx = 0u;
    uint8 uRet = (uint8)PMI_OK;

    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            if((uint8)PMI_FDTI_COMM_DEBUG_DIAG == pSvcCfg->uSubId)
            {
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    BQ7973X_CheckCommDebug (pDiagMgr, pSvcCfg,uDevIdx); //TBC if Inlin func is used correctly
                    uCmd = BQ7973X_FAULT_RST2_RST_COMM_UART_MSK | BQ7973X_FAULT_RST2_RST_COMM_DSY_MSK;
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST2_OFFSET,
                     COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                }
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    uRet = (uint8)PMI_OK;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE3:
        {
            if((uint8)PMI_FDTI_COMM_DEBUG_DIAG == pSvcCfg->uSubId)
            {
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  ,COMIF_PRIO_EXCL_CELL(BQ7973X_DEBUG_COML_BIT_OFFSET, COMIF_LOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_COMM_DEBUG_DIAG], READ_3BYTE);
                
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE2:
        {
            if((uint8)PMI_FDTI_COMM_DEBUG_DIAG == pSvcCfg->uSubId)
            {
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  ,COMIF_PRIO_EXCL_CELL(BQ7973X_DEBUG_COMH_BIT_OFFSET, COMIF_LOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_COMM_DEBUG_DIAG], READ_3BYTE);
                
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE3;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE1:
        {
            if((uint8)PMI_FDTI_COMM_DEBUG_DIAG == pSvcCfg->uSubId)
            {
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId  ,COMIF_PRIO_EXCL_CELL(BQ7973X_DEBUG_UART_RC_TR_OFFSET, COMIF_LOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_COMM_DEBUG_DIAG], READ_1BYTE);

                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE2;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
     case (uint8)DIAG_xSTEP_READY:
        {
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId   ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_SUMMARY_OFFSET, COMIF_LOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_COMM_DEBUG_DIAG], READ_1BYTE);
            
            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
     default:
            {
                break;
            }
    }
    return Pmi_SetErrorDetails(PMI_FDTI_COMM_DEBUG_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/*********************************************************************************************************************
 *Function name:  uint8 NfaultDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
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
*  \retval         PMI_OK
*                  PMI_NOT_OK
*  \trace
*********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_diag_CODE) NfaultDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7973x_DiagReqType *pDiagReq)
{
    uint8 uDevIdx = 0u;
    uint8  uCmd;
    uint8 uRet = (uint8)PMI_OK;
    pDiagReq->uDiagStat = (uint8)PMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {

        case (uint8)DIAG_xSTEP_FINISHED:
       {
           if((uint8)PMI_FDTI_NFAULT == pSvcCfg->uSubId)
           {
               if ((uint8)STD_HIGH == Dio_ReadChannel(BQ7973X_DIO_NFAULT_PIN))
               { //TBC check if this element uSM_NFAULT shall be replaced with  uSM_NFAULT_DIAG
                   pDiagMgr->pDiagResult[0].zPMIFdtiResult.uSM_NFAULT &= 1u;  
               }
               else
               {
                   pDiagMgr->pDiagResult[0].zPMIFdtiResult.uSM_NFAULT |= 2u;
               }
               if (uRet == (uint8)PMI_OK)
               {
                   pDiagReq->uDiagStat = PMI_DIAG_COMPLETE;
               }
           }
           break;
       }
        case (uint8)DIAG_xSTEP_STATE1:
        {
            if((uint8)PMI_FDTI_NFAULT == pSvcCfg->uSubId)
            {
                if ((uint8)STD_LOW == Dio_ReadChannel(BQ7973X_DIO_NFAULT_PIN))
                {
                    uRet = (uint8)PMI_OK;
                }
                else
                {
                    uRet = (uint8)PMI_NOT_OK;
                }


                if((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
                /* Step3: Send comm clear. */
                uRet = Comif_ComClearRequest(pDiagMgr->pComifCtx);
                /* Step4: Clear the fault. */

                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    uCmd = BQ7973X_DIAG_ADC_CTRL2_DIAG_DIG_EN_MSK | BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_GO_MSK;
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_ADC_CTRL2_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
                    uCmd = BQ7973X_NFAULT_DIAG_RST1;
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST1_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
                    uCmd = BQ7973X_NFAULT_DIAG_RST2;
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST2_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
                }
                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, 0u, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST2_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
                /* Step5: Verify NFAULT gets de-asserted. */
                    uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_SUMMARY_OFFSET,
                                        COMIF_UNLOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_NFAULT], READ_1BYTE);
                
                if((uint8)PMI_OK == uRet)
                {
                   pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                   pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_READY:
        {
            /* [MSK_OTP_DATA] */
            uCmd = BQ7973X_FAULT_MSK2_MSK_OTP_DATA_MSK;
            //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
            {
                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_MSK2_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
            }
            /* Step1: Send a command with an incorrect initial byte. */
            uCmd = BQ7973X_DIAG_ADC_CTRL2_DIAG_DIG_EN_MSK | BQ7973X_DIAG_ADC_CTRL2_ACOMP_MPFLT_INJ_MSK | BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_MODE_MSK | BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_GO_MSK;
            //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
            {
                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_ADC_CTRL2_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
            }
            /* Step2: Verify NFAULT gets asserted. */
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_SUMMARY_OFFSET, COMIF_LOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_NFAULT], READ_1BYTE);
            

            if((uint8)PMI_OK == uRet)
             {
                 pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                 pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
             }

            break;
        }
        default:
               {
                   break;
               }
    }
    return Pmi_SetErrorDetails(PMI_FDTI_NFAULT, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************
 *Function name:  uint8 DieTempDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 ********************************************************************************************************************/
/*! \brief Diagnostic Die TEMP1 and TEMP2 voltage measurements are plausible and
 *  the difference in measure value between Die temp 1 and die temp 2 sensor is within specified limits.
 *
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of receive data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_diag_CODE) DieTempDiag( Bq7973x_DiagType * const pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7973x_DiagReqType *pDiagReq)
                             // uint8 BQ7973X_CheckDieTemp( Bq7973x_DiagType  * const pDiagMgr, const ServiceCfgType *pSvcCfg , uint8 uIndex )
{

    uint8 uDevIdx = 0u;

    uint8 uRet = (uint8)PMI_OK;

    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            /*BQ7973X_CheckDieTemp : to check Die Temp1/Temp2 are plausible*/
            //TBC the DevID is passed as argu ??

            {           
                (void) BQ7973X_CheckDieTemp(pDiagMgr,pSvcCfg,uDevIdx);
            }
            if (uRet == (uint8)PMI_OK)
          {
              pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
          }
            break;
        }
     case (uint8)DIAG_xSTEP_READY:
        {
            uRet |= Comif_SingleRead(pDiagMgr->pComifCtx,pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_DIETEMP1_HI_OFFSET,
                COMIF_UNLOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_DIETMEP_DIAG], READ_4BYTE);
            
            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
     default:
            {
                break;
            }
    }
    return Pmi_SetErrorDetails(PMI_FDTI_DIETMEP_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************
 * Function name:  uint8 FactCrcStatDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
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
STATIC FUNC(uint8, bq7973x_diag_CODE) FactCrcStatDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint8 uRet = (uint8)PMI_OK;

    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            if((uint8)PMI_FDTI_FACTORY_CRC_DIAG == pSvcCfg->uSubId)
            {
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    pData[0] &=  BQ7973X_DEV_STAT1_FACT_CRC_DONE_MSK;
                    if(BQ7973X_DEV_STAT1_FACT_CRC_DONE_MSK == pData[0])
                    {
                        uRet = (uint8)PMI_OK;
                    }
                    else
                    {
                        uRet = (uint8)PMI_NOT_OK;
                    }
                }
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat =(uint8) PMI_DIAG_COMPLETE;
                    uRet = (uint8)PMI_OK;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_READY:
        {
            
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_DEV_STAT1_OFFSET, COMIF_UNLOCK) ,
                                     &pDiagMgr->pDiagCfg[PMI_FDTI_FACTORY_CRC_DIAG], READ_2BYTE);
            
            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        default:
               {
                   break;
               }
    }
    return Pmi_SetErrorDetails(PMI_FDTI_FACTORY_CRC_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************
 * Function name:  uint8 FaultSysDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
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
STATIC FUNC(uint8, bq7973x_diag_CODE) FaultSysDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint8 uCmd;
    uint8 uRet = (uint8)PMI_OK;

    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            if((uint8)PMI_FDTI_FAULT_SYS_DIAG == pSvcCfg->uSubId)
            {
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (0u == (pData[0u] & BQ7973X_FAULT_SYS1_TSHUT_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_TSHUT_STAT= DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_TSHUT_STAT= DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_SYS1_DRST_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DVDD_UV= DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DVDD_UV= DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_SYS1_LFO_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_LFOSC_FREQ = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_LFOSC_FREQ = DIAGERROR;
                    }
                }

                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {

                    uCmd = BQ7973X_FAULT_RST1_RST_SYS_MSK;
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST1_OFFSET,
                                     COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                }
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    uRet = (uint8)PMI_OK;
                }
            }
            break;
        }
     case (uint8)DIAG_xSTEP_READY:
        {
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_SYS1_OFFSET, COMIF_LOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_FAULT_SYS_DIAG], READ_2BYTE);
            
            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
     default:
            {
                break;
            }
    }
       return Pmi_SetErrorDetails(PMI_FDTI_FAULT_SYS_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);

}


/*********************************************************************************************************************
 * Function name:  uint8 FaultOtpDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief This function is used to check about FAULT_OTP diagnosis.
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
STATIC FUNC(uint8, bq7973x_diag_CODE) FaultOtpDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint8 uCmd;
    uint8 uRet = (uint8)PMI_OK;

    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            if((uint8)PMI_FDTI_FAULT_OTP_DIAG == pSvcCfg->uSubId)
            {
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if((0u == (pData[0u] & BQ7973X_FAULT_OTP_SEC_DET_MSK)) && \
                        (0u == (pData[0u] & BQ7973X_FAULT_OTP_DED_DET_MSK)))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OTP_ECC = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OTP_ECC = DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_OTP_CUST_CRC_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CUST_CRC = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CUST_CRC = DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_OTP_FACT_CRC_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_FACT_CRC = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_FACT_CRC = DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_OTP_LOADERR_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OTP_LOAD_ERR = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OTP_LOAD_ERR = DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_OTP_GBLOVERR_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OTP_OVERR = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OTP_OVERR = DIAGERROR;
                    }
                }

                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {

                    uCmd = (BQ7973X_FAULT_RST2_RST_OTP_CRC_MSK | BQ7973X_FAULT_RST2_RST_OTP_DATA_MSK);
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST2_OFFSET,
                                     COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                }
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    uRet = (uint8)PMI_OK;
                }
            }
            break;
        }
     case (uint8)DIAG_xSTEP_READY:
        {
            
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_OTP_OFFSET, COMIF_LOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_FAULT_OTP_DIAG], READ_1BYTE);
            
            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
     default:
            {
                break;
            }
    }
       return Pmi_SetErrorDetails((uint8)PMI_FDTI_FAULT_OTP_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);

}


/*********************************************************************************************************************
 * Function name:  uint8 OtpStatDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief This function is used to OTP error detection.
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
STATIC FUNC(uint8, bq7973x_diag_CODE) OtpStatDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                  Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint8 uRet = (uint8)PMI_OK;

    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            if((uint8)PMI_FDTI_OTP_STATE_DIAG == pSvcCfg->uSubId)
            {
               
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                        if (0u == (pData[0u] & BQ7973X_OTP_STAT_PROGERR_MSK))
                        {
                            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OTP_PERR = DIAGNOERROR;
                        }
                        else
                        {
                            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OTP_PERR = DIAGERROR;
                        }
                        if (0u == (pData[0u] & BQ7973X_OTP_STAT_UV_OVERR_MSK))
                        {
                            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OTP_UVOVERR = DIAGNOERROR;
                        }
                        else
                        {
                            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OTP_UVOVERR = DIAGERROR;
                        }
                        if (0u == (pData[0u] & BQ7973X_OTP_STAT_SUV_SOVERR_MSK))
                        {
                            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OTP_SUV_SOVERR = DIAGNOERROR;
                        }
                        else
                        {
                            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OTP_SUV_SOVERR = DIAGERROR;
                        }
                        if (0u == (pData[0u] & BQ7973X_OTP_STAT_UNLOCK_MSK))
                        {
                            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OTP_UNLCKSTAT = DIAGNOERROR;
                        }
                        else
                        {
                            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OTP_UNLCKSTAT = DIAGERROR;
                        }
                }


                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    uRet = (uint8)PMI_OK;
                }
            }
            break;
        }
     case (uint8)DIAG_xSTEP_READY:
        {
            
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_OTP_OFFSET, 
                                COMIF_UNLOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_OTP_STATE_DIAG], READ_1BYTE);
            
            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
     default:
            {
                break;
            }
    }
       return Pmi_SetErrorDetails(PMI_FDTI_OTP_STATE_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);

}


/*********************************************************************************************************************
 * Function name:  uint8 VFOpenDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief This function is used to detect VF open connection faults.
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
STATIC FUNC(uint8, bq7973x_diag_CODE) VFOpenDiag( Bq7973x_DiagType * const pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7973x_DiagReqType *pDiagReq)
{
    uint8 uDevIdx = 0u;
    uint8 uCmd;
    uint8 uRet = (uint8)PMI_OK;

    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            if((uint8)PMI_FDTI_VF_OPEN_DIAG == pSvcCfg->uSubId)
            {

                //TBC should the uDevIdx passed as argu ?  for loop ??
                BQ7973X_CheckVFPlau(pDiagMgr,pSvcCfg); 
    
                uCmd = pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uDev_Conf[1u];
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_DEV_CONF2_OFFSET, COMIF_UNLOCK),
                                                 &uCmd, WRITE_1BYTE);
                }
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    uRet = (uint8)PMI_OK;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_READY:
        {
            BQ7973X_uVFOpenDiagEn = 1u;
            uCmd = pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uDev_Conf[1u] | (BQ7973X_DEV_CONF2_VF2_MSK | BQ7973X_DEV_CONF2_VF1_MSK);
            //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
            {
                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_DEV_CONF2_OFFSET, COMIF_LOCK), &uCmd, WRITE_1BYTE);
            }
            uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_VF_OPEN_DELAY);

            
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_VF2_HI_OFFSET, COMIF_LOCK) ,
                                         &pDiagMgr->pDiagCfg[PMI_FDTI_VF_OPEN_DIAG], BQ7973X_VF_REG_NUM *READ_2BYTE);
            
            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        default:
               {
                   break;
               }
    }
       return Pmi_SetErrorDetails(PMI_FDTI_VF_OPEN_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);

}

/*********************************************************************************************************************
 * Function name:  uint8 CSPlauDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief API to diagnostic current sense input plausibility and difference.

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
STATIC FUNC(uint8, bq7973x_diag_CODE) CSPlauDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint8 uRet = (uint8)PMI_OK;
    uint8 uCheckTimesNum;
    uint32  qCSVoltage[BQ7973X_CS_NUM_ACT] = {0u};
    uint8   uIndex;

    switch(pDiagReq->uDiagStep)
    {

        case (uint8)DIAG_xSTEP_FINISHED:
       {
           if((uint8)PMI_FDTI_CS_PLAU_DIAG == pSvcCfg->uSubId)
           {
               //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);

                    qCSVoltage[0u] = (((uint32)pData[0u] << 16u) | \
                                    ((uint32)pData[1u] << 8u) | (uint32)pData[2u]);
                    qCSVoltage[1u] = (((uint32)pData[3u] << 16u) | \
                                    ((uint32)pData[4u] << 8u) | (uint32)pData[5u]);
                    for (uIndex = 0u; uIndex < BQ7973X_CS_NUM_ACT; uIndex ++)
                    {
                        if(BQ7973X_INVALDCSVALUE == qCSVoltage[uIndex])
                        {
                           pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CS_PLAU |= ((uint8)DIAGERROR << uIndex);
                           pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CS_COMP = DIAGERROR;
                        }
                        else if((qCSVoltage[uIndex] > BQ7973X_MAXCSPLAUVALUE) && (qCSVoltage[uIndex] < BQ7973X_MINCSPLAUVALUE))
                        {
                            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CS_PLAU |= ((uint8)DIAGERROR << uIndex);
                        }
                        else
                        {
                            /* Nothing */
                        }
                    }
               }
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    uRet = (uint8)PMI_OK;
                }
           }
           break;
       }

        case (uint8)DIAG_xSTEP_READY:
        {
            for(uCheckTimesNum = 0u; uCheckTimesNum < BQ7973X_CSPLAU_CHECKTIMES; uCheckTimesNum++)
            {
                
                    uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_CURRENT1_HI_OFFSET, 
                                    COMIF_UNLOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_CS_PLAU_DIAG], (READ_3BYTE*BQ7973X_CURRENT_REG_NUM));
                

            }


            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                 pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        default:
               {
                   break;
               }
    }
       return Pmi_SetErrorDetails(PMI_FDTI_CS_PLAU_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/*********************************************************************************************************************
 * Function name:  uint8 FaultPwrDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/**\brief This function is used to check about FAULT_PWR* diagnosis.

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
STATIC FUNC(uint8, bq7973x_diag_CODE) FaultPwrDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint8 uCmd;
    uint8 uRet = (uint8)PMI_OK;

    switch(pDiagReq->uDiagStep)
    {

        case (uint8)DIAG_xSTEP_FINISHED:
       {
           if((uint8)PMI_FDTI_FAULT_PWR_DIAG == pSvcCfg->uSubId)
           {
               //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);

                     if (0u == (pData[0u] & BQ7973X_FAULT_PWR1_VSS_OPEN_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_REFVSS_OPEN = DIAGNOERROR;
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VSS_OPEN = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_REFVSS_OPEN = DIAGERROR;
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_VSS_OPEN = DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_PWR1_TSREF_OSC_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_TSREF_OSC = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_TSREF_OSC = DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_PWR1_TSREF_UV_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_TSREF_UV = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_TSREF_UV = DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_PWR1_TSREF_OV_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_TSREF_OV = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_TSREF_OV = DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_PWR1_DVDD_OV_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DVDD_OV = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DVDD_OV = DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_PWR1_AVDD_OSC_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_AVDD_OSC = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_AVDD_OSC = DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_PWR1_AVDD_OV_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_AVDD_OV = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_AVDD_OV = DIAGERROR;
                    }
                    if (0u == (pData[1u] & BQ7973X_FAULT_PWR2_CP_UV_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CP_UV = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CP_UV = DIAGERROR;
                    }
                    if (0u == (pData[1u] & BQ7973X_FAULT_PWR2_CP_OV_MSK))
                    {
                       pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CP_OV = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CP_OV = DIAGERROR;
                    }
               }
               uCmd = BQ7973X_FAULT_RST1_RST_PWR_MSK;
               //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                   uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST1_OFFSET, 
                                COMIF_UNLOCK ), &uCmd, WRITE_1BYTE);
               }
               if ((uint8)PMI_OK == uRet)
               {
                   pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                   uRet = (uint8)PMI_OK;
               }
           }
           break;
       }

        case (uint8)DIAG_xSTEP_READY:
        {
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_PWR1_OFFSET, COMIF_LOCK) ,
                                 &pDiagMgr->pDiagCfg[PMI_FDTI_FAULT_PWR_DIAG], READ_2BYTE);
            


            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                 pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        default:
               {
                   break;
               }
    }
       return Pmi_SetErrorDetails(PMI_FDTI_FAULT_PWR_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/*********************************************************************************************************************
 * Function name:  uint8 GPPlauDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief Diagnostic GP voltage measurements are plausible.
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
STATIC FUNC(uint8, bq7973x_diag_CODE) GPPlauDiag( Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7973x_DiagReqType *pDiagReq)
{

    uint8 uRet = (uint8)PMI_OK;
    uint8 uCheckTimesNum;


    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            if((uint8)PMI_FDTI_GP_PLAU_DIAG == pSvcCfg->uSubId)
            {
                 BQ7973X_CheckGPPlau(pDiagMgr,pSvcCfg);

                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    uRet = (uint8)PMI_OK;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_READY:
        {
            for(uCheckTimesNum = 0u; uCheckTimesNum < BQ7973X_GPPLAU_CHECKTIMES;uCheckTimesNum++)
            {

                
                    uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,BQ7973X_GPIO1_HI_OFFSET
                                 , &pDiagMgr->pDiagCfg[PMI_FDTI_GP_PLAU_DIAG], READ_2BYTE*BQ7973X_GPIO_REG_NUM);
                
            }
            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        default:
               {
                   break;
               }
    }
       return Pmi_SetErrorDetails(PMI_FDTI_GP_PLAU_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);

}


/*********************************************************************************************************************
 * Function name:  uint8 FaultAdcMiscDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 * *******************************************************************************************************************/
/** \brief This function is used to check about FAULT_ADC_MISC diagnosis.
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
STATIC FUNC(uint8, bq7973x_diag_CODE) FaultAdcMiscDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint8 uRet = (uint8)PMI_OK;
    uint8 uCmd;


    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {
            //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
            {
                if(pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_MAIN_PFAIL > 0u)
                {
                    uCmd = pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uAdc_Ctrl[1u] | BQ7973X_ADC_CTRL2_ADC_GO_MSK;
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_CTRL2_OFFSET, COMIF_LOCK),
                                     &uCmd, WRITE_1BYTE);
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_OC_CTRL1_OFFSET, COMIF_LOCK),
                                     &uCmd, WRITE_1BYTE);

                }
                if(pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DIAG_PFAIL > 0u)
                {
                    uCmd = pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uAdc_Ctrl[3u] | BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_MSK;
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_CTRL4_OFFSET, COMIF_LOCK),
                                                     &uCmd, WRITE_1BYTE);

                }
                if(pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_ANA_PFAIL > 0u)
                {
                    uCmd = BQ7973X_DIAG_ADC_CTRL2_DIAG_DIG_EN_MSK | BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_GO_MSK;
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_ADC_CTRL2_OFFSET, COMIF_LOCK),
                                                     &uCmd, WRITE_1BYTE);

                }
            }
                                                 
            uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_OC_CTRL1_OFFSET, COMIF_LOCK)
                                     , &pDiagMgr->pDiagCfg[PMI_FDTI_FAULT_ADC_MISC], READ_1BYTE);

            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE1:
        {                                     
            uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_ADC_MISC_OFFSET, COMIF_LOCK) ,
                                         &pDiagMgr->pDiagCfg[PMI_FDTI_FAULT_ADC_MISC], READ_1BYTE);  
            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            if((uint8)PMI_FDTI_FAULT_ADC_MISC == pSvcCfg->uSubId)
            {
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (0u == (pData[0u] & BQ7973X_FAULT_ADC_MISC_ADC_PFAIL_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_MAIN_PFAIL = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_MAIN_PFAIL = DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_ADC_MISC_DIAG_MEAS_PFAIL_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DIAG_PFAIL = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DIAG_PFAIL = DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_ADC_MISC_DIAG_ANA_PFAIL_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_ANA_PFAIL = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_ANA_PFAIL = DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_ADC_MISC_DIAG_ANA_ABORT_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DIAG_ERR = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DIAG_ERR = DIAGERROR;
                    }
                }

                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    uCmd = BQ7973X_FAULT_RST2_RST_ADC_MSK;
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST2_OFFSET, 
                                COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                }
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    uRet = (uint8)PMI_OK;
                }
            }
            break;
        }
        default:
               {
                   break;
               }
    }
    return Pmi_SetErrorDetails(PMI_FDTI_FAULT_ADC_MISC, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}



/*********************************************************************************************************************
 * Function name:  uint8 GpioOpenDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 * *******************************************************************************************************************/
/** \brief API to perform GPIO open wire detection.
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
STATIC FUNC(uint8, diag_CODE) GpioOpenDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                        Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx = 0;
    uint8 uRet = (uint8)PMI_OK;
    uint8   Cmd[BQ7973X_GPIO_CONF1_6_REG_NUM + BQ7973X_GPIO_CONF7_8_REG_NUM] = {0};
    uint8   uGpioIndex;

    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            if((uint8)PMI_FDTI_GPIO_OPNWR_DIAG == pSvcCfg->uSubId)
            {
                /* BQ7973X_CheckGpioOpen : GPIO open wire detection*/
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < BQ7973X_GPIO_NUM_ACT; uDevIdx ++)
                { 
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    BQ7973X_GPVoltage_C[uDevIdx] = (((uint16)pData[uDevIdx * 2u] << 8u) |(uint16)pData[(uDevIdx * 2u) + 1u]);
                }
                BQ7973X_CheckGpioOpenWire(pDiagMgr);
                /* Step6: Return to original function . */
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO_CONF1_OFFSET, COMIF_LOCK),
                                             &pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uGpio_Conf[0], BQ7973X_GPIO_CONF1_6_REG_NUM);
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO_CONF7_OFFSET, COMIF_LOCK),
                                             &pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uGpio_Conf[BQ7973X_GPIO_CONF1_6_REG_NUM], BQ7973X_GPIO_CONF7_8_REG_NUM);

                }
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < (BQ7973X_GPIO_CONF1_6_REG_NUM + BQ7973X_GPIO_CONF7_8_REG_NUM); uDevIdx ++)
                {
                    BQ7973X_uGpioConfRegData[uDevIdx] = pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uGpio_Conf[uDevIdx];
                }
                /* Step8: Reset the FAULT_ADC_* registers. */

                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO_CONF1_OFFSET, COMIF_LOCK) ,
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_GPIO_OPNWR_DIAG], BQ7973X_GPIO_CONF1_6_REG_NUM);
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO_CONF7_OFFSET, COMIF_UNLOCK),
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_GPIO_OPNWR_DIAG], BQ7973X_GPIO_CONF7_8_REG_NUM);

                uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_GPADC_DELAY);
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    uRet = (uint8)PMI_OK;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE3:
        {
            if((uint8)PMI_FDTI_GPIO_OPNWR_DIAG == pSvcCfg->uSubId)
            {
                /* BQ7973X_PreVGpio : */
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < BQ7973X_GPIO_NUM_ACT; uDevIdx ++)
                { 
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    BQ7973X_GPVoltage_B[uDevIdx] = (((uint16)pData[uDevIdx * 2u] << 8u) |(uint16)pData[(uDevIdx * 2u) + 1u]);
                }
                 /* Step4: Write GPIO configure ADC & Weak Pull-Down. */

                 /* Traverse all gpio */
                for (uGpioIndex = 0u; uGpioIndex < BQ7973X_GPIO_NUM_ACT; uGpioIndex++)
                {
                    if(uGpioIndex < ((2u * BQ7973X_GPIO_CONF1_6_REG_NUM)  - 1u))
                    {
                        Cmd[(uint8)(uGpioIndex / 2u)] |= (BQ7973X_GPIOSETPULLDOWN << ((uGpioIndex % 2u) * BQ7973X_GPIOSETNUM));
                    }
                    else
                    {
                        Cmd[(uint8) ((uGpioIndex + 1u) / 2u)] |=(BQ7973X_GPIOSETPULLDOWN << (((uGpioIndex + 1u) % 2u) * BQ7973X_GPIOSETNUM));
                    }
                }
                //for  (uDevIdx = 0u; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    Cmd[0u] |= (pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uGpio_Conf[0u] & BQ7973X_GPIO_CONF1_GPIO1_FLT_EN_MSK);
                    Cmd[1u] |= (pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uGpio_Conf[1u] & BQ7973X_GPIO_CONF2_CS_RDY_EN_MSK);
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO_CONF1_OFFSET, COMIF_LOCK),
                                                             Cmd, BQ7973X_GPIO_CONF1_6_REG_NUM);
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO_CONF7_OFFSET, COMIF_LOCK),
                             &Cmd[BQ7973X_GPIO_CONF1_6_REG_NUM], BQ7973X_GPIO_CONF7_8_REG_NUM);
                }
                 uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_GPADC_DELAY);
                 /* Step5: Save GPIO Value C. */
                 /* Check ABS Value A - Value B, ABS Value A - Value C */
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO1_HI_OFFSET, COMIF_LOCK) ,
                         &pDiagMgr->pDiagCfg[PMI_FDTI_GPIO_OPNWR_DIAG], (READ_2BYTE * BQ7973X_GPIO_REG_NUM));
                }
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE2:
        {
            if((uint8)PMI_FDTI_GPIO_OPNWR_DIAG == pSvcCfg->uSubId)
            {
                /*BQ7973X_VGpio : to read the GPIO voltage*/
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < BQ7973X_GPIO_NUM_ACT; uDevIdx ++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    BQ7973X_GPVoltage[uDevIdx] = (((uint16)pData[uDevIdx * 2u] << 8u) |(uint16)pData[(uDevIdx * 2u) + 1u]);
                }
                /* Step2: Write GPIO configure ADC & Weak Pull-Up. */
                /* Traverse all gpio */
                for (uGpioIndex = 0u; uGpioIndex < BQ7973X_GPIO_NUM_ACT; uGpioIndex++)
                {
                    if(uGpioIndex < ((2u * BQ7973X_GPIO_CONF1_6_REG_NUM)  - 1u))
                    {
                        Cmd[(uint8)(uGpioIndex / 2u)] |=(BQ7973X_GPIOSETPULLUP << ((uGpioIndex % 2u) * BQ7973X_GPIOSETNUM));
                    }
                    else
                    {
                        Cmd[(uint8) ((uGpioIndex + 1u) / 2u)] |=(BQ7973X_GPIOSETPULLUP << (((uGpioIndex + 1u) % 2u) * BQ7973X_GPIOSETNUM));
                    }
                }
                //for  (uDevIdx = 0u; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    Cmd[0u] |= (pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uGpio_Conf[0u] & BQ7973X_GPIO_CONF1_GPIO1_FLT_EN_MSK);
                    Cmd[1u] |= (pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uGpio_Conf[1u] & BQ7973X_GPIO_CONF2_CS_RDY_EN_MSK);
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO_CONF1_OFFSET, COMIF_LOCK),
                                             Cmd, BQ7973X_GPIO_CONF1_6_REG_NUM);
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO_CONF7_OFFSET, COMIF_LOCK),
                                     &Cmd[BQ7973X_GPIO_CONF1_6_REG_NUM], BQ7973X_GPIO_CONF7_8_REG_NUM);
                }
                uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_GPADC_DELAY);

                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO1_HI_OFFSET, COMIF_LOCK) ,
                                 &pDiagMgr->pDiagCfg[PMI_FDTI_GPIO_OPNWR_DIAG], (READ_2BYTE* BQ7973X_GPIO_REG_NUM));
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE3;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE1:
        {
            if((uint8)PMI_FDTI_GPIO_OPNWR_DIAG == pSvcCfg->uSubId)
            {
                //for  (uDevIdx = 0u; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                     if(0u == (pData[0u] & BQ7973X_ADC_DATA_RDY_DRDY_GPADC_MSK ))
                    {
                        uRet = (uint8)PMI_NOT_OK;
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DRDY_GP = DIAGERROR;
                    }
                    else
                    {
                        uRet = (uint8)PMI_OK;
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DRDY_GP |= DIAGNOERROR;
                    }
                }
                /* Step1: Save GPIO Value A. */

                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO1_HI_OFFSET, COMIF_LOCK)
                             , &pDiagMgr->pDiagCfg[PMI_FDTI_GPIO_OPNWR_DIAG], (READ_2BYTE* BQ7973X_GPIO_REG_NUM));

                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE2;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_READY:
        {
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_DATA_RDY_OFFSET, COMIF_LOCK)
                                     , &pDiagMgr->pDiagCfg[PMI_FDTI_GPIO_OPNWR_DIAG], READ_1BYTE);
            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        default:
               {
                   break;
               }
    }
    return Pmi_SetErrorDetails(PMI_FDTI_GPIO_OPNWR_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/*********************************************************************************************************************
 * Function name:  uint8 GpioAdjShortDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 * *******************************************************************************************************************/
/** \brief API to diagnose the adjacent GPIO pin short.
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
STATIC FUNC(uint8, diag_CODE) GpioAdjShortDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                        Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint8 uCmd;
    uint8 uRet = (uint8)PMI_OK;
    uint8   Cmd[BQ7973X_GPIO_CONF1_6_REG_NUM + BQ7973X_GPIO_CONF7_8_REG_NUM] = {0};
    uint8   uGpioIndex;
    uint8   uGpioConf;

    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            if((uint8)PMI_FDTI_GPIO_ADJSHRT == pSvcCfg->uSubId)
            {
                /* BQ7973X_CheckGpioAdj : to run GPIO adjacent short diagnostic*/

                { 
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    BQ7973X_GPVoltage_B[uDevIdx] = (((uint16)pData[uDevIdx * 2u] << 8u) |(uint16)pData[(uDevIdx * 2u) + 1u]);
                    BQ7973X_CheckGpioAdjShort(pDiagMgr,pSvcCfg,uDevIdx);
                }

                /* Step4: Write ODD GPIO configure ADC measure. */

                {
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO_CONF1_OFFSET, COMIF_LOCK),
                             &pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uGpio_Conf[0], BQ7973X_GPIO_CONF1_6_REG_NUM);
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO_CONF7_OFFSET, COMIF_LOCK),
                            &pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uGpio_Conf[BQ7973X_GPIO_CONF1_6_REG_NUM], BQ7973X_GPIO_CONF7_8_REG_NUM);

                }

                {
                    BQ7973X_uGpioConfRegData[uDevIdx] = pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegNVM.uGpio_Conf[uDevIdx];
                }
                /* Step8: Reset the FAULT_ADC_* registers. */

                    uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO_CONF1_OFFSET, COMIF_LOCK)
                                 , &pDiagMgr->pDiagCfg[PMI_FDTI_GPIO_ADJSHRT], READ_6BYTE );//TBC cmd parameter
                   uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO_CONF7_OFFSET, COMIF_UNLOCK)
                                 ,&pDiagMgr->pDiagCfg[PMI_FDTI_GPIO_ADJSHRT], READ_2BYTE); //TBC cmd parameter

                uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_GPADC_DELAY);
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    uRet = (uint8)PMI_OK;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE4:
        {
            if((uint8)PMI_FDTI_GPIO_ADJSHRT == pSvcCfg->uSubId)
            {
                /* BQ7973X_PreVF : read the VF voltage */

                { 
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    BQ7973X_VFVoltage_B[uDevIdx] = (((uint16)pData[uDevIdx * 2u] << 8u) |(uint16)pData[(uDevIdx * 2u) + 1u]);
                }
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO1_HI_OFFSET, COMIF_LOCK)
                                 , &pDiagMgr->pDiagCfg[PMI_FDTI_GPIO_ADJSHRT], (READ_2BYTE* BQ7973X_GPIO_REG_NUM));
                
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE3:
        {
            if((uint8)PMI_FDTI_GPIO_ADJSHRT == pSvcCfg->uSubId)
            {
                /* BQ7973X_VGpio : read the GPIO voltage */

                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    BQ7973X_GPVoltage[uDevIdx] = (((uint16)pData[uDevIdx * 2u] << 8u) |(uint16)pData[(uDevIdx * 2u) + 1u]);
                }
                /* Step2: Write ODD GPIO configure drive output logic low. */
                 /* Traverse all gpio */
                for (uGpioIndex = 0u; uGpioIndex < BQ7973X_GPIO_NUM_ACT; uGpioIndex++)
                {
                    if((uGpioIndex % 2u) == 1u)
                    {
                        uGpioConf = BQ7973X_GPIOSETADCONLY;
                    }
                    else
                    {
                        uGpioConf = BQ7973X_GPIOSETOUTLOW;
                    }
                    if(uGpioIndex < ((2u * BQ7973X_GPIO_CONF1_6_REG_NUM)  - 1u))
                    {
                        Cmd[uGpioIndex / 2u] |= (uGpioConf << ((uGpioIndex % 2u) * BQ7973X_GPIOSETNUM));
                    }
                    else
                    {
                        Cmd[(uGpioIndex + 1u) / 2u] |= (uGpioConf << (((uGpioIndex + 1u) % 2u) * BQ7973X_GPIOSETNUM));
                    }
                }
                uCmd= BQ7973X_GPIO_CTRL1_POR_VAL;

                {
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO_CONF1_OFFSET, 
                    COMIF_LOCK), &uCmd, READ_2BYTE);
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO_CONF1_OFFSET,
                     COMIF_LOCK), Cmd, BQ7973X_GPIO_CONF1_6_REG_NUM);
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO_CONF7_OFFSET, 
                    COMIF_LOCK), &Cmd[BQ7973X_GPIO_CONF1_6_REG_NUM], BQ7973X_GPIO_CONF7_8_REG_NUM);
                }
                 uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_GPADC_DELAY);

               /* Step3: Save EVEN GPIO Value B. */
               /* Check ABS Value A - Value B */

                    uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_VF2_HI_OFFSET,
                     COMIF_LOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_GPIO_ADJSHRT], (READ_2BYTE * BQ7973X_VF_REG_NUM));
                
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE4;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
   case (uint8)DIAG_xSTEP_STATE2:
        {
            if((uint8)PMI_FDTI_GPIO_ADJSHRT == pSvcCfg->uSubId)
            {
                /* BQ7973X_VF: read the VF voltage */
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    BQ7973X_VFVoltage[uDevIdx] = (((uint16)pData[uDevIdx * 2u] << 8u) |(uint16)pData[(uDevIdx * 2u) + 1u]);
                }

                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO1_HI_OFFSET, COMIF_LOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_GPIO_ADJSHRT], (READ_2BYTE* BQ7973X_GPIO_REG_NUM));
                
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE3;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE1:
        {
            if((uint8)PMI_FDTI_GPIO_ADJSHRT == pSvcCfg->uSubId)
            {

                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                     if(0u == (pData[0u] & BQ7973X_ADC_DATA_RDY_DRDY_GPADC_MSK ))
                    {
                         uRet = (uint8)PMI_NOT_OK;
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DRDY_GP = DIAGERROR;
                    }
                    else
                    {
                        uRet = (uint8)PMI_OK;
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_DRDY_GP = DIAGNOERROR;
                    }
                }
               /* Step1: Save EVEN GPIO Value A. */

                    uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_VF2_HI_OFFSET, COMIF_LOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_GPIO_ADJSHRT], (READ_2BYTE* BQ7973X_VF_REG_NUM));
                
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE2;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
     case (uint8)DIAG_xSTEP_READY:
        {
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_DATA_RDY_OFFSET, COMIF_LOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_GPIO_ADJSHRT], READ_1BYTE);
            
            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
     default:
            {
                break;
            }
    }
       return Pmi_SetErrorDetails(PMI_FDTI_GPIO_ADJSHRT, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}




/*********************************************************************************************************************
 * Function name:  uint8 UpdateCustCrc(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 * *******************************************************************************************************************/
/*! \brief This function is used to update CUST_CRC register
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
STATIC FUNC(uint8, bq7973x_diag_CODE) UpdateCustCrc(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint8 uCmd;
    uint8 uRet = (uint8)PMI_OK;
    pDiagReq->uDiagStat = PMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {

        case (uint8)DIAG_xSTEP_FINISHED:
        {
           if((uint8)PMI_FDTI_UPDATE_CUST_CRC == pSvcCfg->uSubId)
           {
                //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    uCmd = pData[0];
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_CUST_CRC_HI_OFFSET, COMIF_LOCK),
                                                         &uCmd, WRITE_2BYTE);
                    uCmd = BQ7973X_FAULT_RST2_RST_OTP_CRC_MSK | BQ7973X_FAULT_RST2_RST_OTP_DATA_MSK;
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST2_OFFSET, 
                                                        COMIF_UNLOCK), &uCmd, WRITE_1BYTE);

                }
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    uRet = (uint8)PMI_OK;
                }
           }
           break;
       }
        case (uint8)DIAG_xSTEP_READY:
        {
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_CUST_CRC_RSLT_HI_OFFSET, 
                                COMIF_LOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_UPDATE_CUST_CRC], READ_2BYTE);
            
            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                 pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        default:
               {
                   break;
               }
    }
       return Pmi_SetErrorDetails(PMI_FDTI_UPDATE_CUST_CRC, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);

}



/*********************************************************************************************************************
 * Function name:  uint8 FaultOCDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 * *******************************************************************************************************************/
/** \brief This function is used to check about FAULT_OC diagnosis.
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
STATIC FUNC(uint8, bq7973x_diag_CODE) FaultOCDiag( Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint8 uCmd;
    uint8 uRet = (uint8)PMI_OK;

     pDiagReq->uDiagStat = (uint8)PMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            if((uint8)PMI_FDTI_FLT_OTC_DIAG == pSvcCfg->uSubId)
            {

                {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (0u == (pData[0u] & BQ7973X_FAULT_OC_OCC1_MSK))
                    {
                         pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CS_OCC1 |= DIAGNOERROR;
                    }
                    else
                    {
                         pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CS_OCC1 |= DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_OC_OCC2_MSK))
                    {
                         pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CS_OCC2 |= DIAGNOERROR;
                    }
                    else
                    {
                         pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CS_OCC2 |= DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_OC_OCD1_MSK))
                    {
                         pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CS_OCD1 |= DIAGNOERROR;
                    }
                    else
                    {
                         pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CS_OCD1 |= DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_OC_OCD2_MSK))
                    {
                         pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CS_OCD2 |= DIAGNOERROR;
                    }
                    else
                    {
                         pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_CS_OCD2 |= DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_OC_OC_PFAIL_MSK))
                    {
                         pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OC_PFAIL |= DIAGNOERROR;
                    }
                    else
                    {
                         pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OC_PFAIL |= DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_OC_DIAG_OC_ABORT_MSK))
                    {
                         pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OC_ERR |= DIAGNOERROR;
                    }
                    else
                    {
                         pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OC_ERR |= DIAGERROR;
                    }
                }

                BQ7973X_CheckOC_OS(pDiagMgr, pSvcCfg); 

                uCmd = BQ7973X_FAULT_RST1_RST_DIAG_OC_MSK;

                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST1_OFFSET,
                                        COMIF_LOCK), &uCmd, WRITE_1BYTE);

                uCmd = BQ7973X_FAULT_RST3_RST_OC1_MSK | BQ7973X_FAULT_RST3_RST_OC2_MSK;

                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST3_OFFSET,
                                        COMIF_UNLOCK), &uCmd, WRITE_1BYTE);

                //if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    uRet = (uint8)PMI_OK;
                }
            }
            break;
        }
       case (uint8)DIAG_xSTEP_STATE1:
        {
            if((uint8)PMI_FDTI_FLT_OTC_DIAG == pSvcCfg->uSubId)
            {
                /* Read FAULT_OC register.*/
                    uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_OC_OFFSET, COMIF_LOCK)
                                         , &pDiagMgr->pDiagCfg[PMI_FDTI_FLT_OTC_DIAG], READ_1BYTE);
                
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
                }
            }
            break;
        }
        case (uint8)DIAG_xSTEP_READY:
        {
            if(pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OC_PFAIL > 0u)
            {


                {
                    uCmd = pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uAdc_Ctrl[1u] | BQ7973X_ADC_CTRL2_ADC_GO_MSK;
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_CTRL2_OFFSET,
                                                 COMIF_LOCK), &uCmd, WRITE_1BYTE);

                    uCmd = pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uOC_Ctrl[0] | BQ7973X_ADC_CTRL2_ADC_GO_MSK;
                    uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_OC_CTRL1_OFFSET, 
                                                COMIF_LOCK), &uCmd, WRITE_1BYTE);
                }
            }
             uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_OC_CTRL1_OFFSET,
                                COMIF_LOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_FLT_OTC_DIAG], READ_1BYTE);
            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                 pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        default:
               {
                   break;
               }
    }
       return Pmi_SetErrorDetails(PMI_FDTI_FLT_OTC_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);

}


/*********************************************************************************************************************
 * Function name:  uint8 SWMonDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 * *******************************************************************************************************************/
/**\brief API to diagnose SW output monitoring.

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
STATIC FUNC(uint8, bq7973x_diag_CODE) SWMonDiag(uint8 uSwConf , const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData = NULL;
    uint8 uDevIdx = 0u;
    uint8 uCmd;
    uint8 uRet = (uint8)PMI_OK;
    uint16 xSWMonVoltage1;
    uint16 xSWMonVoltage3;

    switch(pDiagReq->uDiagStep)
    {

        case (uint8)DIAG_xSTEP_FINISHED:
       {
           if((uint8)PMI_FDTI_SW_MON_DIAG == pSvcCfg->uSubId)
           {
               //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    xSWMonVoltage1 = (((uint16)pData[0u] << 8u) | (uint16)pData[1u]);
                    xSWMonVoltage3 = (((uint16)pData[14u] << 8u) | (uint16)pData[15u]);
                    if((xSWMonVoltage1 < BQ7973X_MINSWMONVALUE) || (xSWMonVoltage1 > BQ7973X_MAXSWMONVALUE))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_SW_MON |= DIAGERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_SW_MON &= (~DIAGERROR);
                    }
                    if((xSWMonVoltage3 < BQ7973X_MINSWMONVALUE) || (xSWMonVoltage3 > BQ7973X_MAXSWMONVALUE))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_SW_MON |= ((uint8)DIAGERROR << 2u);
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_SW_MON &= (~((uint8)DIAGERROR << 2u));
                    }
               }
               uCmd = uSwConf;
            //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
            {
                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_SW_CTRL_OFFSET, 
                                        COMIF_UNLOCK), &uCmd, BQ7973X_SW_REG_NUM);
            }
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    uRet = (uint8)PMI_OK;
                }
           }
           break;
       }

        case (uint8)DIAG_xSTEP_READY:
        {
            /* Step1: SW1,SW3 output is high. */
            uCmd = (uSwConf & (BQ7973X_SW_CTRL_SW4_MSK | BQ7973X_SW_CTRL_SW2_MSK)) | BQ7973X_SW_DIAG_SW_CTRL1;
            //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
            {
                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_SW_CTRL_OFFSET, COMIF_LOCK), &uCmd, BQ7973X_SW_REG_NUM);
            }
            uRet |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_SW_DIAG_DELAY);
             /* Step2: The host MCU monitors the divided voltage input to verify the SW pin output is in the correct state. */
                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO1_HI_OFFSET, COMIF_LOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_SW_MON_DIAG], (READ_2BYTE * BQ7973X_GPIO_REG_NUM));
            


            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                 pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        default:
               {
                   break;
               }
    }
       return Pmi_SetErrorDetails(PMI_FDTI_SW_MON_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);

}
/*********************************************************************************************************************
 *Std_ReturnType OCPlauDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*\brief API to diagnostic over current sense input plausibility and difference.

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
STATIC FUNC(uint8, bq7973x_diag_CODE) OCPlauDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData = NULL;
    uint8 uDevIdx = 0u;
    uint8 uRet = (uint8)PMI_OK;
    uint16  xOCVoltage[BQ7973X_OC_NUM_ACT] = {0};
    uint8   uIndex;

    switch(pDiagReq->uDiagStep)
    {

        case (uint8)DIAG_xSTEP_FINISHED:
       {
           if((uint8)PMI_FDTI_OC_PLAU_DIAG == pSvcCfg->uSubId)
           {
               //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx); 
                    xOCVoltage[0u] = (((uint16)pData[0u] << 8u) |(uint16)pData[1u]);
                    xOCVoltage[1u] = (((uint16)pData[2u] << 8u) |(uint16)pData[3u]);
                    for (uIndex = 0u; uIndex < BQ7973X_OC_NUM_ACT; uIndex ++)
                    {
                        if(BQ7973X_INVALDOCVALUE == xOCVoltage[uIndex])
                        {
                            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OC_PLAU |= ((uint8)DIAGERROR << uIndex);
                            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OC_COMP = (uint8)DIAGERROR;
                        }
                        else if((xOCVoltage[uIndex] > BQ7973X_MAXOCPLAUVALUE) && (xOCVoltage[uIndex] < BQ7973X_MINOCPLAUVALUE))
                        {
                            pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_OC_PLAU |= ((uint8)DIAGERROR << uIndex);
                        }
                        else
                        {
                            /*Do Nothing*/ 
                        }
                    }

               }
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    uRet = (uint8)PMI_OK;
                }
           }
           break;
       }

        case (uint8)DIAG_xSTEP_READY:
        {
           //for(uCheckTimesNum = 0u; uCheckTimesNum < BQ7973X_OCPLAU_CHECKTIMES; uCheckTimesNum++) ask CHANNA
            {
                    uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_OC1_HI_OFFSET, 
                                                COMIF_UNLOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_OC_PLAU_DIAG], READ_4BYTE);
                

            }


            if((uint8)PMI_OK == uRet)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                 pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        default:
               {
                   break;
               }
    }
       return Pmi_SetErrorDetails(PMI_FDTI_OC_PLAU_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);

}


/*********************************************************************************************************************
 *Std_ReturnType BQ7973X_VFPlauDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief Diagnostic VF voltage measurements are plausible.
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
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_VFPlauDiag( Bq7973x_DiagType *const pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal = (uint8)PMI_OK;

    pDiagReq->uDiagStat = (uint8)PMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {
            /*read VF voltage*/     
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_VF2_HI_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_VF_PLAU_DIAG], (BQ7973X_VF_REG_NUM * BQ7973X_READ_2BYTE));
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }   
        case (uint8)DIAG_xSTEP_FINISHED:
        {

        /*CheckVFPlau : Check the VF Input plausibility diagnosis result */

            BQ7973X_CheckVFPlau(pDiagMgr,pSvcCfg);

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
    return Pmi_SetErrorDetails(PMI_FDTI_VF_PLAU_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/*************************************************************************************************************************
 * Function name:  uint8 FaultCommDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *************************************************************************************************************************/
/**\brief This is a function to check about FAULT_COMM diagnosis.

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
STATIC FUNC(uint8, bq7973x_diag_CODE) FaultCommDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,
                                                     Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint8 uCmd;
    uint8 uRet = (uint8)PMI_OK;

    switch(pDiagReq->uDiagStep)
    {

        case (uint8)DIAG_xSTEP_FINISHED:
       {
           if((uint8)PMI_FDTI_FAULT_COMM_DIAG == pSvcCfg->uSubId)
           {
               //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
               {
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);

                    if (0u == (pData[0u] & BQ7973X_FAULT_COMM1_STOP_DET_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_UART_STOPBIT = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_UART_STOPBIT = DIAGERROR;
                    }
                    if (0u == (pData[0u] & BQ7973X_FAULT_COMM1_UART_FRAME_MSK))
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_UART_CRC = DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_UART_CRC = DIAGERROR;
                    }
               }
               uCmd = BQ7973X_FAULT_RST2_RST_COMM_UART_MSK;
            //for (uDevIdx =pDiagMgr->sDevId; uDevIdx < pDiagMgr->uNAFEs; uDevIdx ++)
            {
                uRet |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_RST1_OFFSET, 
                                                COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
            }
                if ((uint8)PMI_OK == uRet)
                {
                    pDiagReq->uDiagStat = (uint8)PMI_DIAG_COMPLETE;
                    uRet = (uint8)PMI_OK;
                }
           }
           break;
       }

        case (uint8)DIAG_xSTEP_READY:
        {

                uRet |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_COMM1_OFFSET, 
                                COMIF_LOCK) , &pDiagMgr->pDiagCfg[PMI_FDTI_FAULT_COMM_DIAG], READ_2BYTE);
            


            if((uint8)PMI_OK == uRet)
            {
                 pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                 pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        default:
               {
                   break;
               }
    }
       return Pmi_SetErrorDetails(PMI_FDTI_FAULT_COMM_DIAG, uRet, pDiagReq->uDiagStep, pDiagReq->uDiagStat);

}




// TBC which categ. of the following APIs


/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_DrdyDiagD1D2Diag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief API to diagnose D1D2 ADC DIAG data ready status bit.
 *
 * Extended function description
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval PMI_OK: Ready status bit check successful
 *         PMI_NOT_OK: Ready status bit check failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_DrdyDiagD1D2Diag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8   uDevIdx =0u;
    Std_ReturnType RetVal = (uint8)PMI_OK;
    pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_DIAGD1D2 = BQ7973X_DIAGNOERROR;

    {
        pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
        if(0u == (pData[0u] & BQ7973X_ADC_DATA_RDY_DRDY_DIAG_D1D2_MSK ))
        {
            RetVal = (uint8)PMI_NOT_OK;
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_DIAGD1D2 = BQ7973X_DIAGERROR;
        }
        else
        {
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_DIAGD1D2 |= BQ7973X_DIAGNOERROR;
        }
    }
    return Pmi_SetErrorDetails(PMI_FDTI_DRY_DIG_D1_D2_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}

/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_GetCs1Vref(uint8 uActDevID, Com_RspDataCheck_Struct_Type * RspDataCheck, \
 *                                                  Com_TD_Config_Struct_Type *TD_ConfigPtr)
 *********************************************************************************************************************/
/** \brief This function is used to read the CS1 VREF voltage.
 *
 * Extended function description
 *
 * \param[in] uActDevID: device ID active in the current communication direction
 * \param[out] RspDataCheck: Pointer to a variable to store CS1 VREF voltage
 * \param[out] TD_ConfigPtr: Pointer to a transaction descriptor instance
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_GetCs1Vref(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal = (uint8)PMI_OK;
    uint8 uCmd;
    uint8  uDevIdx = 0u;

    pDiagReq->uDiagStat = (uint8)PMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {
            uCmd = BQ7973X_DIAG_D1D2_SEL_CS1VREF | BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_MSK;

            {     
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_CTRL4_OFFSET,
                             COMIF_LOCK), &uCmd, BQ7973X_WRITE_1BYTE);  
            }        
            //BQ7973X_uAdcCtrlRegData[3u] = uCmd & (~BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_MSK);  TBC

            RetVal |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_DIAG_D1D2_DELAY);  // BQ7973X_VF_OPEN_DELAY with BQ7973X_DIAG_D1D2_DELAY from the GRC
             //&BQ7973X_uAdc_Data_RdyCheck
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_ADC_DATA_RDY_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_GET_CS1_VREF], BQ7973X_READ_1BYTE);
                    
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }  
        case (uint8)DIAG_xSTEP_FINISHED:
        {
 
            /* BQ7973X_DrdyDiagD1D2 : check DIAG_D1/D2 conversion result available*/
            RetVal |= BQ7973X_DrdyDiagD1D2Diag(pDiagMgr,pSvcCfg,pDiagReq);
            //    if(E_OK == RetVal)
            //    {
            //        s_BQ7973X_DrdyCnt.BQ7973X_uDrdyDiagD1D2 = 0u;
            //        s_BQ7973X_DrdyErr.BQ7973X_uDrdyDiagD1D2Err = 0u;
            //    }
            //    else
            //    {
            //        s_BQ7973X_DrdyCnt.BQ7973X_uDrdyDiagD1D2++;
            //    }
            //    if(s_BQ7973X_DrdyCnt.BQ7973X_uDrdyDiagD1D2 > BQ7973X_CALLBACK_TIMEOUT)
            //    {
            //        s_BQ7973X_DrdyCnt.BQ7973X_uDrdyDiagD1D2 = 0u;
            //        s_BQ7973X_DrdyErr.BQ7973X_uDrdyDiagD1D2Err = 1u;
            //        RetVal = E_OK;
            //    }

            //  RetVal |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_DCOMP_DONE_DELAY); TBD
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_D2_HI_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_GET_CS1_VREF], BQ7973X_READ_2BYTE);





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
    return Pmi_SetErrorDetails(PMI_FDTI_GET_CS1_VREF, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);

}


/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_InsulationDet(Bq7973x_ManagerType *pPmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/** \brief This function is used to measure the insulation resistances Riso_P and Riso N*.
 *
 * Extended function description
 *
 *  \param[in]      pPmicMgr: Manager
 *  \param[in]      pSvcCfg - service config

 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_InsulationDet(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal = (uint8)PMI_OK;
    uint8   uCmd;
    uint8 *pData;
    uint8  uDevIdx =0u;
    switch(pDiagReq->uDiagStep) 
    {
        case (uint8)DIAG_xSTEP_READY:
        {
            uCmd = (pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uSW_Ctrl & (BQ7973X_SW_CTRL_SW4_MSK | BQ7973X_SW_CTRL_SW3_MSK)) | BQ7973X_INSUL_DET_SW_CTRL1;
            /*Bq7973x_SetSWInit : initialize CC Control*/
           
            {     
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_SW_CTRL_OFFSET,
                             COMIF_LOCK), &uCmd, BQ7973X_SW_REG_NUM);  
            } 

            RetVal |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_INSUL_DET_DELAY);
            
            /*Bq7973x_GetVf*/
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_VF2_HI_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_INSULAT_DET], BQ7973X_VF_REG_NUM * READ_2BYTE);
            
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE1;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }


        case (uint8)DIAG_xSTEP_STATE1:
        {
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO1_HI_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_INSULAT_DET], (READ_2BYTE * BQ7973X_GPIO_REG_NUM) );
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE2;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_STATE2:
        {
            /*BQ7973X_InsulDetOn : to measure the insulation resistances */
            // TBC for loop here is needed ? if yes the var BQ7973X_InsulVch1 shall be an array
           
            {
                pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                BQ7973X_InsulVch1[uDevIdx] = (((uint16)pData[2u] << 8u) |(uint16)pData[3u]);
                BQ7973X_InsulVhv1[uDevIdx] = (((uint16)pData[2u] << 8u) |(uint16)pData[3u]);
            }
            uCmd = (pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uSW_Ctrl & (BQ7973X_SW_CTRL_SW4_MSK | BQ7973X_SW_CTRL_SW3_MSK)) | BQ7973X_INSUL_DET_SW_CTRL2;
    
            /*Bq7973x_SetSWInit : initialize CC Control*/
           
            {     
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_SW_CTRL_OFFSET,
                             COMIF_LOCK), &uCmd, BQ7973X_SW_REG_NUM);  
            } 
            RetVal |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_INSUL_DET_DELAY);
            /*Bq7973x_GetVf*/ 
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_VF2_HI_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_INSULAT_DET], BQ7973X_VF_REG_NUM * READ_2BYTE);
            
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_STATE3;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }

        case (uint8)DIAG_xSTEP_STATE3:
        {
            /*Bq7973x_GetVGpio*/
             RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , COMIF_PRIO_EXCL_CELL(BQ7973X_GPIO1_HI_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_INSULAT_DET], (READ_2BYTE * BQ7973X_GPIO_REG_NUM));
            

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {

            
            {     
                BQ7973X_CheckInsulDet(pDiagMgr,pSvcCfg,uDevIdx);

                uCmd =pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uSW_Ctrl;
            /*Bq7973x_SetSWInit */
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_SW_CTRL_OFFSET,
                             COMIF_LOCK), &uCmd, BQ7973X_SW_REG_NUM);  
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
    return Pmi_SetErrorDetails(PMI_SVCID_INSULATION_DET, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}
/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_GetVAvdd(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief This function is used to read the AVDD voltage.
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
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_GetVAvdd(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal = E_OK;
    uint8 ucmd;
    uint8 uDevIdx =0 ;
    uint8 *pData;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {

            ucmd = BQ7973X_DIAG_D1D2_SEL_AVDD | BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_MSK;

            {
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx,BQ7973X_ADC_CTRL4_OFFSET,&ucmd, WRITE_1BYTE);
            }

            RetVal |= Comif_SetDelay(pDiagMgr->pComifCtx, BQ7973X_DIAG_D1D2_DELAY);
                    //BQ7973X_uAdcCtrlRegData[3u] = ucmd & (~BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_MSK);

            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , BQ7973X_ADC_DATA_RDY_OFFSET,
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_GET_VAVDD], BQ7973X_READ_1BYTE);
            
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            /*BQ7973X_DrdyDiagD1D2 : */ 
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_DIAGD1D2 = BQ7973X_DIAGNOERROR;

            {   /* check if measurement of DiagD1D2 conversion is done */
                pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                if(0u == (pData[0u] & BQ7973X_ADC_DATA_RDY_DRDY_DIAG_D1D2_MSK ))
                {
                    RetVal = (uint8)PMI_NOT_OK;
                    pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_DIAGD1D2 = BQ7973X_DIAGERROR;
                }
                else
                {
                    pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_DIAGD1D2 |= BQ7973X_DIAGNOERROR;
                }
            }
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , BQ7973X_DIAG_D1_HI_OFFSET,
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_GET_VAVDD], BQ7973X_READ_2BYTE);
            

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
    return RetVal;
}

/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_ReadSPIData(uint8 uDevId, Com_RspDataCheck_Struct_Type * RspDataCheck, Com_TD_Config_Struct_Type *TD_ConfigPtr)
 *********************************************************************************************************************/
/** \brief This function is used to read the data from the SPI_TX1 to SPI_TX3 registers.
 *
 * Extended function description
 *
 * \param[in] uDevId: Device Id.
 * \param[out] RspDataCheck: Pointer to response SPI data.
 * \param[out] TD_ConfigPtr: Pointer to a transaction descriptor instance
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Insert command successful
 *         E_NOT_OK: Insert command failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_ReadSPIData(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal = E_OK;
    uint8 ucmd;
    uint8 uDevIdx = 0u ;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {
            ucmd = BQ7973X_MSPI_EXE_MSPI_GO_MSK;           
            //for(uDevIdx = 0; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            { 
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx,BQ7973X_MSPI_EXE_OFFSET,&ucmd, WRITE_1BYTE);

            }
            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , BQ7973X_MSPI_RX4_OFFSET,
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_READ_SPI_DATA], BQ7973X_READ_4BYTE);
            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            ucmd = BQ7973X_MSPI_EXE_POR_VAL;
            //for(uDevIdx = 0; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
            { 
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx,BQ7973X_MSPI_EXE_OFFSET,&ucmd, WRITE_1BYTE);

            }
            if((uint8)PMI_OK == RetVal)
            {
                 pDiagReq->uDiagStat = PMI_DIAG_COMPLETE;
            }
            break;
        }
        default:
        {
            break;
        }
    }


    return RetVal;
}


/*********************************************************************************************************************
 * Function name:  STATIC FUNC(uint8, bq7973x_CODE) BQ7973X_SetVIsync(Bq7973x_ManagerType *pPmicMgr ,const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/** \brief API to perform voltage and current (V/I sync) synchronization.
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
STATIC FUNC(uint8, bq7973x_CODE) BQ7973X_SetVIsync(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    uint8 RetVal = (uint8)PMI_OK;
    uint8 uCmd1 = pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uAdc_Ctrl[0u] | BQ7973X_ADC_CTRL1_V_I_SYNC_MSK;
    uint8 uCmd2 = pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uAdc_Ctrl[1u];
    uint8 uDevIdx = 0u;
    switch(pDiagReq->uDiagStep) 
    {
        case (uint8)DIAG_xSTEP_READY:
        {

                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx,BQ7973X_ADC_CTRL1_OFFSET,&uCmd1, WRITE_1BYTE);
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx,BQ7973X_ADC_CTRL2_OFFSET,&uCmd2, WRITE_1BYTE);

            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , BQ7973X_ADC_CTRL1_OFFSET,
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_SET_VISYNC], READ_2BYTE);
    //TBC this reg ?
    //BQ7973X_uAdcCtrlRegData[0u] = uCmd1;
    //BQ7973X_uAdcCtrlRegData[1u] = uCmd2 & (~BQ7973X_ADC_CTRL2_ADC_GO_MSK);

            if((uint8)PMI_OK == RetVal)
            {
                pDiagReq->uDiagStep = (uint8)DIAG_xSTEP_FINISHED;
                pDiagReq->uDiagStat = (uint8)PMI_DIAG_RUN;
            }
            break;
        }
        case (uint8)DIAG_xSTEP_FINISHED:
        {
            /*BQ7973X_OCCtrl1Init*/


                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx, uDevIdx,BQ7973X_OC_CTRL1_OFFSET,
                                            pDiagMgr->pPmcCfg->p_rcBq7973xRegsCfg->zRegCtrl.uOC_Ctrl, WRITE_1BYTE);

            RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId , BQ7973X_OC_CTRL1_OFFSET,
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_SET_VISYNC], READ_1BYTE);

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

    return RetVal;
}   

/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_DrdyVFDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief API to diagnose main ADC data ready status bit.
 *
 * Extended function description
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval PMI_OK: Ready status bit check successful
 *         PMI_NOT_OK: Ready status bit check failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_DrdyVFDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8   uDevIdx =0u;
    Std_ReturnType RetVal = (uint8)PMI_OK;
    pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_VF = BQ7973X_DIAGNOERROR;
    //for(uDevIdx = 0; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
    {
        pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
        if(0u == (pData[0u] & BQ7973X_ADC_DATA_RDY_DRDY_VFADC_MSK ))
        {
            RetVal = (uint8)PMI_NOT_OK;
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_VF = BQ7973X_DIAGERROR;
        }
        else
        {
            RetVal = (uint8)PMI_OK;
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_VF |= BQ7973X_DIAGNOERROR;
        }
    }
    return Pmi_SetErrorDetails(PMI_FDTI_DRY_VF_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}
 
/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_DrdyGpioDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief API to diagnose GPIO ADC data ready status bit.
 *
 * Extended function description
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval PMI_OK: Ready status bit check successful
 *         PMI_NOT_OK: Ready status bit check failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_DrdyGpioDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8  uDevIdx =0u;
    Std_ReturnType RetVal = (uint8)PMI_OK;
    pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_GP = BQ7973X_DIAGNOERROR; 
    //for(uDevIdx = 0; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
    {
        pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
        if(0u == (pData[0u] & BQ7973X_ADC_DATA_RDY_DRDY_GPADC_MSK ))
        {
            RetVal = (uint8)PMI_NOT_OK;
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_GP = BQ7973X_DIAGERROR;
        }
        else
        {
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_GP |= BQ7973X_DIAGNOERROR;
        }
    }
    return Pmi_SetErrorDetails(PMI_FDTI_DRY_GPIO_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}
 
/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_DrdyDigDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief API to diagnose digital comp data ready status bit.
 *
 * Extended function description
 *
 *  \param[in]      pDiagMgr - Diagnostic manager context
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval PMI_OK: Ready status bit check successful
 *         PMI_NOT_OK: Ready status bit check failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_DrdyDigDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8   uDevIdx =0u;
    Std_ReturnType RetVal = (uint8)PMI_OK;
    pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_DIG = BQ7973X_DIAGNOERROR; 
    //for(uDevIdx = 0; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
    {
        pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
        if(0u == (pData[1u] & BQ7973X_DIAG_STAT2_DRDY_DIG_MSK ))
        {
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_DIG = BQ7973X_DIAGERROR;
            RetVal = (uint8)PMI_NOT_OK;
        }
        else
        {
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_DIG |= BQ7973X_DIAGNOERROR;
        }
    }
    return Pmi_SetErrorDetails(PMI_FDTI_DRY_DIG_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}
 
 
/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_DrdyCSDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief API to diagnose CS ADC data ready status bit.
 *
 * Extended function description
 *
*  \param[in]      pDiagMgr - Diagnostic manager context
*  \param[in]      pDiagCfg - Diagnostic context
*  \param[in]      pDiagReq - Diag Request
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval PMI_OK: Ready status bit check successful
 *         PMI_NOT_OK: Ready status bit check failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_DrdyCSDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8   uDevIdx =0u;
    Std_ReturnType RetVal = (uint8)PMI_OK;
    pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_CS = BQ7973X_DIAGNOERROR;  
    //for(uDevIdx = 0; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
    {
        pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
        if(0u == (pData[0u] & BQ7973X_ADC_DATA_RDY_DRDY_CSADC_MSK ))
        {
            RetVal = (uint8)PMI_NOT_OK;
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_CS = BQ7973X_DIAGERROR;
        }
        else
        {
            RetVal = (uint8)PMI_OK;
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_CS |= BQ7973X_DIAGNOERROR;
        }
    }
        return Pmi_SetErrorDetails(PMI_FDTI_DRY_CS_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}



/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_DrdyAnaVFDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief API to diagnose VF comp data ready status bit.
 *
 * Extended function description
 *  \param[in]      pDiagMgr - Diagnostic manager context
*  \param[in]      pDiagCfg - Diagnostic context
*  \param[in]      pDiagReq - Diag Request
 * \param[in] DiagStatRspDataCheck: Pointer to the result of the DRDY_ANA_VF return.
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Ready status bit check successful
 *         E_NOT_OK: Ready status bit check failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_DrdyAnaVFDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8   uDevIdx =0u;
    Std_ReturnType RetVal =(uint8) PMI_OK;
    pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_ANA_VF = BQ7973X_DIAGNOERROR;
    //for(uDevIdx = 0; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
    {
        pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
        if(0u == (pData[1u] & BQ7973X_DIAG_STAT2_DRDY_ANA_VF_MSK ))
        {
            RetVal = (uint8)PMI_NOT_OK;
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_ANA_VF = BQ7973X_DIAGERROR;
        }
        else
        {
            RetVal = (uint8)PMI_OK;
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_ANA_VF |= BQ7973X_DIAGNOERROR;
        }
    }
        return Pmi_SetErrorDetails(PMI_FDTI_DRDY_ANA_VF_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************
 * Function name:  Std_ReturnType BQ7973X_DrdyAnaGpioDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/** \brief API to diagnose GPIO comp data ready status bit.
 *
 * Extended function description
*  \param[in]      pDiagMgr - Diagnostic manager context
*  \param[in]      pDiagCfg - Diagnostic context
*  \param[in]      pDiagReq - Diag Request
 * \param[in] DiagStatRspDataCheck: Pointer to the result of the DRDY_ANA_GPIO return.
 * \pre Preconditions
 * \post Postconditions
 * \return Std_ReturnType
 * \retval E_OK: Ready status bit check successful
 *         E_NOT_OK: Ready status bit check failed.
 *
 *********************************************************************************************************************/
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_DrdyAnaGpioDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8   uDevIdx =0u;
    Std_ReturnType RetVal =(uint8) PMI_OK;
    pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_ANA_GP = BQ7973X_DIAGNOERROR;
    //for(uDevIdx = 0; uDevIdx < pDiagMgr->uNAFEs; uDevIdx++)
    {
        pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
        if(0u == (pData[1u] & BQ7973X_DIAG_STAT2_DRDY_ANA_GPIO_MSK ))
        {
            RetVal = (uint8)PMI_NOT_OK;
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_ANA_GP = BQ7973X_DIAGERROR;
        }
        else
        {
            RetVal = (uint8)PMI_OK;
            pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_DRDY_ANA_GP |= BQ7973X_DIAGNOERROR;
        }
    }
    return Pmi_SetErrorDetails(PMI_FDTI_DRDY_ANA_GPIO_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************
 * Function name:  uint8 BQ7973X_FactTmDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief         This function is used to fact testmode detection.
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
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_FactTmDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    uint8 *pData;
    uint8 uDevIdx = 0u;
    uint8 RetVal =         (uint8)  PMI_OK;
     pDiagReq->uDiagStat = (uint8)  PMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep){

        case (uint8) DIAG_xSTEP_READY:

        {
                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx, pDiagMgr->sDevId ,COMIF_PRIO_EXCL_CELL(BQ7973X_FACT_TM_REG, COMIF_UNLOCK) ,
                                     &pDiagMgr->pDiagCfg[PMI_FDTI_FACT_TM_DIAG], READ_1BYTE);

                if((uint8) PMI_OK == RetVal)
                {
                    pDiagReq->uDiagStep = (uint8) DIAG_xSTEP_FINISHED;;
                    pDiagReq->uDiagStat = (uint8) PMI_DIAG_RUN;
                }
            break;
        }
        case (uint8) DIAG_xSTEP_FINISHED:
        {
            /* BQ7973X_FactTmDiagCallBack */
            if((uint8) PMI_FDTI_FACT_TM_DIAG == pSvcCfg->uSubId )

            {
                RetVal = (uint8) PMI_OK;
                pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                if (0u != pData[0u])
                {
                    pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_FACT_TM = (uint8)BQ7973X_DIAGERROR;
                    RetVal = (uint8) PMI_NOT_OK;
                }
                else
                {
                    pDiagMgr->pDiagResult->zPMIFdtiResult.uSM_FACT_TM = (uint8)BQ7973X_DIAGNOERROR;
                }

                if((uint8) PMI_OK == RetVal)
                {
                    pDiagReq->uDiagStat = (uint8) PMI_DIAG_COMPLETE;
                }
            }
            break;
        }
          default:
        {
            break;
        }
    }
   return Pmi_SetErrorDetails(PMI_FDTI_FACT_TM_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************
 * Function name:  uint8 BQ7973X_OtpMarginDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
 *********************************************************************************************************************/
/*! \brief         This function is used to check OTP read margin test.
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
STATIC FUNC(uint8, bq7973x_diag_CODE) BQ7973X_OtpMarginDiag(const Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcCfg,Bq7973x_DiagReqType *pDiagReq)
{
    Std_ReturnType RetVal = (uint8)PMI_OK;
    uint8  uCmd;
    uint8  uDevIdx =0u;
    uint8 *pData;
    pDiagReq->uDiagStat = (uint8)PMI_DIAG_ERROR;
    switch(pDiagReq->uDiagStep)
    {
        case (uint8)DIAG_xSTEP_READY:
        {
            uCmd = BQ7973X_MARGIN_MODE_M1RD | BQ7973X_DIAG_MISC_CTRL1_MARGIN_GO_MSK;

            {

                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_MISC_CTRL1_OFFSET,COMIF_LOCK),
                                            &uCmd, WRITE_1BYTE);

                RetVal |= Comif_SingleRead(pDiagMgr->pComifCtx,pDiagMgr->sDevId, COMIF_PRIO_EXCL_CELL(BQ7973X_FAULT_OTP_OFFSET,COMIF_LOCK),
                                             &pDiagMgr->pDiagCfg[PMI_FDTI_OTP_MARGIN_DIAG],READ_1BYTE);
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
            /*BQ7973X_OtpMargin : to check OTP read margin test results */
            if((uint8)PMI_FDTI_OTP_MARGIN_DIAG == pSvcCfg->uSubId)
            {

                {
                    RetVal = (uint8) PMI_OK;
                    pData = BQ7973x_GET_DATA_PTR(pSvcCfg, pDiagMgr->uNAFEs, uDevIdx);
                    if (0u == pData[0u])
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OTP_MARGIN = BQ7973X_DIAGNOERROR;
                    }
                    else
                    {
                        pDiagMgr->pDiagResult[uDevIdx].zPMIFdtiResult.uSM_OTP_MARGIN = BQ7973X_DIAGERROR;
                    }
                }
            }
            else
            {
               RetVal = (uint8)PMI_NOT_OK;
            }

            uCmd = BQ7973X_DIAG_MISC_CTRL1_MARGIN_GO_MSK;
            //BQ7973X_uDiagMiscCtrlRegData[0u] = BQ7973X_DIAG_MISC_CTRL1_POR_VAL;
            {
                RetVal |= Comif_SingleWrite(pDiagMgr->pComifCtx ,uDevIdx, COMIF_PRIO_EXCL_CELL(BQ7973X_DIAG_MISC_CTRL1_OFFSET,COMIF_UNLOCK),
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
    return Pmi_SetErrorDetails(PMI_FDTI_OTP_MARGIN_DIAG, RetVal, pDiagReq->uDiagStep, pDiagReq->uDiagStat);
}


/*********************************************************************************************************************
 *  External Functions Definition
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_diag_CODE) Bq7973x_DiagFdti(Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
 *                                                   Bq7973x_DiagReqDataType *pDiagReq)
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

FUNC(uint8, bq7973x_diag_CODE) Bq7973x_DiagFdti(Bq7973x_DiagType *pDiagMgr, const ServiceCfgType *pSvcReqCfg,
                                               Bq7973x_DiagReqType *pDiagReq)
{
    uint8 uRet;

    switch(pDiagReq->uDiagReqId)
    {
        case (uint8)PMI_FDTI_ACOMP_DIAG:
        {
             uRet = AcompDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
               break;
        }
        case (uint8)PMI_FDTI_DCOMP_DIAG:
        {
             uRet = DcompDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
               break;
        }
        case (uint8)PMI_FDTI_COMM_DEBUG_DIAG:
        {
           uRet = CommDebugDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
             break;
        }
        case (uint8)PMI_FDTI_NFAULT:
        {
           uRet = NfaultDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
             break;
        }
        case (uint8)PMI_FDTI_DIETMEP_DIAG:
        {
           uRet = DieTempDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
             break;
        }
        case (uint8)PMI_FDTI_FACTORY_CRC_DIAG:
        {
           uRet = FactCrcStatDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
             break;
        }

        case (uint8)PMI_FDTI_FAULT_SYS_DIAG:
        {
            uRet = FaultSysDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
              break;
        }
        case (uint8)PMI_FDTI_FAULT_OTP_DIAG:
        {
            uRet = FaultOtpDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
              break;
        }
        case (uint8)PMI_FDTI_OTP_STATE_DIAG:
        {
            uRet = OtpStatDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
              break;
        }

        case (uint8)PMI_FDTI_VF_OPEN_DIAG:
        {
             uRet = VFOpenDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
               break;
        }
        case (uint8)PMI_FDTI_GP_PLAU_DIAG:
        {
             uRet = GPPlauDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
               break;
        }
        case (uint8)PMI_FDTI_FAULT_ADC_MISC:
        {
             uRet = FaultAdcMiscDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
               break;
        }
        case (uint8)PMI_FDTI_GPIO_OPNWR_DIAG:
        {
            uRet = GpioOpenDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_FDTI_GPIO_ADJSHRT:
        {
            uRet = GpioAdjShortDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_FDTI_UPDATE_CUST_CRC:
        {
            uRet = UpdateCustCrc(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_FDTI_FLT_OTC_DIAG:
        {
            uRet = FaultOCDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_FDTI_CS_PLAU_DIAG:
        {
            uRet = CSPlauDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_FDTI_SW_MON_DIAG:
        {
            uRet = SWMonDiag(1 , pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_FDTI_FAULT_PWR_DIAG:
        {
            uRet = FaultPwrDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_FDTI_FAULT_COMM_DIAG:
        {
            uRet = FaultCommDiag(pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_FDTI_OC_PLAU_DIAG:
        {
            uRet = OCPlauDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_FDTI_VF_PLAU_DIAG:
        {  
            uRet = BQ7973X_VFPlauDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
//        case (uint8)PMI_FDTI_DRY_VF_DIAG:
//        {  
//            uRet = BQ7973X_DrdyVFDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
//            break;
//        }
//        case (uint8)PMI_FDTI_DRY_GPIO_DIAG:
//        {  
//            uRet = BQ7973X_DrdyGpioDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
//            break;
//        }
//        case (uint8)PMI_FDTI_DRY_DIG_DIAG:
//        {  
//            uRet = BQ7973X_DrdyDigDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
//            break;
//        }
//        case (uint8)PMI_FDTI_DRY_DIG_D1_D2_DIAG:
//        {  
//            uRet = BQ7973X_DrdyDiagD1D2Diag( pDiagMgr, pSvcReqCfg, pDiagReq);
//            break;
//        }
//        case (uint8)PMI_FDTI_DRY_CS_DIAG:
//        {  
//            uRet = BQ7973X_DrdyCSDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
//            break;
//        }
//        case (uint8)PMI_FDTI_DRDY_ANA_VF_DIAG:
//        {  
//            uRet = BQ7973X_DrdyAnaVFDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
//            break;
//        }
//        case (uint8)PMI_FDTI_DRDY_ANA_GPIO_DIAG:
//        {  
//            uRet = BQ7973X_DrdyAnaGpioDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
//            break;
//        }
        case (uint8)PMI_FDTI_GET_CS1_VREF:
        {
            uRet = BQ7973X_GetCs1Vref( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_FDTI_INSULAT_DET:
        {
            uRet = BQ7973X_InsulationDet( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_FDTI_READ_SPI_DATA:
        {
            uRet = BQ7973X_ReadSPIData( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_FDTI_SET_VISYNC:
        {
            uRet = BQ7973X_SetVIsync( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_FDTI_GET_VAVDD:
        {
            uRet =  BQ7973X_GetVAvdd( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_FDTI_FACT_TM_DIAG:
        {
            uRet =  BQ7973X_FactTmDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
            break;
        }
        case (uint8)PMI_FDTI_OTP_MARGIN_DIAG:
        {
            uRet =  BQ7973X_OtpMarginDiag( pDiagMgr, pSvcReqCfg, pDiagReq);
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
