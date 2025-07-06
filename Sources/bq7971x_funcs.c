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
 *  File:       bq7971x_funcs.c
 *  Project:    TIBMS
 *  Module:     BMI
 *  Generator:  Code generation tool (ifany)
 *
 *  Description:  Exposed functionalities for Bq7971x chipsets
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05May2022    SEM                0000000000000    Initial version
 * 01.00.01       05April2023  SEM                0000000000000    Re-writing Error reporting
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
#include "tibms_bmi.h"
#include "tibms_utils.h"
#include "bq7971x_diag.h"

/**********************************************************************************************************************
 * Version Check (if required)
 *********************************************************************************************************************/

/**	Major Software Config C Version number */
#define BQ7971X_FUNC_C_MAJOR_VERSION             (0x01u)

/**	Minor Software Config C Version number */
#define BQ7971X_FUNC_C_MINOR_VERSION             (0x00u)

/** Software Patch Config C Version number */
#define BQ7971X_FUNC_C_PATCH_VERSION             (0x00u)

#if ((BQ7971X_FUNC_C_MAJOR_VERSION != BQ7971X_CFG_MAJOR_VERSION) || \
     (BQ7971X_FUNC_C_MINOR_VERSION != BQ7971X_CFG_MINOR_VERSION) || \
	 (BQ7971X_FUNC_C_PATCH_VERSION != BQ7971X_CFG_PATCH_VERSION))
#error "tibms: Config version numbers of bq7971x_funcs.c and bq7971x_cfg.h are inconsistent!"
#endif

#if ((BQ7971X_SW_MAJOR_VERSION != BQ7971X_FUNC_C_MAJOR_VERSION) || \
     (BQ7971X_SW_MINOR_VERSION != BQ7971X_FUNC_C_MINOR_VERSION) || \
	 (BQ7971X_SW_PATCH_VERSION != BQ7971X_FUNC_C_PATCH_VERSION))
#error "tibms: Config version numbers of bq7971x_funcs.c and bq7971x.h are inconsistent!"
#endif

#if ((BQ7971X_REGS_MAJOR_VERSION != BQ7971X_FUNC_C_MAJOR_VERSION) || \
     (BQ7971X_REGS_MINOR_VERSION != BQ7971X_FUNC_C_MINOR_VERSION) || \
	 (BQ7971X_REGS_PATCH_VERSION != BQ7971X_FUNC_C_PATCH_VERSION))
#error "tibms: Config version numbers of bq7971x_funcs.c and bq7971x_regs.h are inconsistent!"
#endif

/**********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

#define BQ7971X_GUARD_START                                     (0xCCA1AB1FU)
#define BQ7971X_GUARD_END                                       (0xCAFECEEEU)
#define BQ7971X_INIT                                            (0xCAU)

#define BQ7971X_INTEGRITY_CHECK(pBmicMgr)                       ((NULL != (pBmicMgr)) && \
                                                                (BQ7971X_GUARD_START == pBmicMgr->qGuardStart) && \
                                                                (BQ7971X_GUARD_END == pBmicMgr->qGuardEnd) && \
                                                                (BQ7971X_INIT == pBmicMgr->uInit))

#define CB_STATUS_PAUSE_NORMAL                                  (0x00U)
#define CB_STATUS_PAUSE_ENABLED                                 (0x01U)

/**********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

typedef enum DriverStateType_Tag
{
    BQ7971X_STATE_UINIT,
    BQ7971X_STATE_WAKEUP,
    BQ7971X_STATE_INIT,
    BQ7971X_STATE_NORMAL,
    BQ7971X_STATE_DIAG,
    BQ7971X_STATE_ERROR,
}
DriverStateType;


/**********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

#define BQ7971X_FUNCS_START_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

static Bq7971x_ManagerType zBq7971xMgr[BQ7971X_IFACES];
static Bq7971x_CellBalType zBq7971xCBCtrl[BQ7971X_IFACES];

#define BQ7971X_FUNCS_STOP_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

/***********************************************************************************************************************
 *  Local CONST Object Definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  Local Function Prototypes
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  Local Inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  Local Functions Definition
*********************************************************************************************************************/

#define BQ7971X_FUNCS_START_SEC_CODE
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 * Function name: Bq7971x_ConfigInit
 *********************************************************************************************************************/
/*! \brief          Internal Function for Configurations Init
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

STATIC FUNC(uint8, bq7971x_CODE) Bq7971x_ConfigInit(Bq7971x_ManagerType *pBmicMgr)
{
    uint8 uRet;

    do
    {
        //initialize DEV_CONF
        uRet = Bq7971x_DevConfInit(pBmicMgr);
        if((uint8)BMI_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_DEVCFG_FAILED;
            break;
        }

        //initialize CONTROL2 Register
        uRet =Bq7971x_Control2Init(pBmicMgr);
        if((uint8)BMI_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_CTRL2_FAILED;
            break;
        }

        //initialize GPIO CONF Register
        uRet = Bq7971x_GpioConfInit(pBmicMgr);
        if((uint8)BMI_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_GPIOCFG_FAILED;
        }

        //initialize ADC_CTRLx
        uRet = Bq7971x_AdcCtrlInit(pBmicMgr);
        if((uint8)BMI_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_ADCCTRL_FAILED;
            break;
        }

        uRet = Bq7971x_OvUvThreshInit(pBmicMgr);
        if((uint8)BMI_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_OVUVCFG_FAILED;
            break;
        }

        uRet = Bq7971x_OtUtThreshInit(pBmicMgr);
        if((uint8)BMI_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_OTUTCFG_FAILED;
            break;
        }

        uRet = Bq7971x_DiagServiceReq(pBmicMgr->pBmicDiag, BMI_SM_MPFDI_STARTUPINIT, NULL, 0U);
        if((uint8)BMI_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_DIAG_STARTUP_ERROR;
            break;
        }

        pBmicMgr->eDrvState = (uint8)BQ7971X_STATE_NORMAL;
        uRet = (uint8)BMI_BMC_CFGINIT_SUCCESS;
    }
    while(FALSE);

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_CFGDATA_INIT, uRet, pBmicMgr->uBmicIface, pBmicMgr->eDrvState);
}

/**********************************************************************************************************************
 * Function name: Bq7971x_AutoAddress
 *********************************************************************************************************************/
/*! \brief          Function for Auto addressing
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Config
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

STATIC FUNC(uint8, bq7971x_CODE) Bq7971x_AutoAddress(Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg)
{
    uint16 wReg;
    uint8 uCmd;
    uint8 uRet;
    uint8 uDevId;

    do
    {
        /* TODO: Sync up internal Dll(write) */
        uRet = Comif_BroadcastWrite(pBmicMgr->pComifCtx, TRUE, BQ7971X_CONTROL1_OFFSET,&pBmicMgr->eCommDir, WRITE_1BYTE);
        if((uint8)COMIF_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_DIR_SET_FAILED;
            break;
        }

        /* clear [TOP_STACK] information after communication direction is changed. */
        uCmd = 0x00u;
        uRet = Comif_BroadcastWrite(pBmicMgr->pComifCtx, FALSE, BQ7971X_COMM_CTRL_OFFSET, &uCmd, WRITE_1BYTE);
        if((uint8)COMIF_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_TOP_STACK_FAILED;
            break;
        }

// Note: This is work around for the silicon bug upto B1 Sample
#if (BQ7971X_SILICON_REV_A0 >= BQ7971X_SILICON_REV)
        uCmd = 0xA5u;

        uRet = Comif_BroadcastWrite(pBmicMgr->pComifCtx, FALSE, BQ7971X_DEBUG_CTRL_UNLOCK_OFFSET, &uCmd, WRITE_1BYTE);
        if((uint8)COMIF_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_BPATCH_FAILED;
            break;
        }

        uCmd = 0x23u;
        uRet = Comif_BroadcastWrite(pBmicMgr->pComifCtx, FALSE, BQ7971X_DEBUG_COMM_CTRL1_OFFSET, &uCmd, WRITE_1BYTE);
        if((uint8)COMIF_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_BPATCH_FAILED;
            break;
        }

        uCmd = 0x94u;
        uRet = Comif_BroadcastWrite(pBmicMgr->pComifCtx, FALSE, 0xC00, &uCmd, WRITE_1BYTE);
        if((uint8)COMIF_OK != uRet)
        {
            uRet =(uint8)BMI_BMC_BPATCH_FAILED;
            break;
        }

        uCmd = 0xA3u;
        uRet = Comif_BroadcastWrite(pBmicMgr->pComifCtx, FALSE, 0xDFF, &uCmd, WRITE_1BYTE);
        if((uint8)COMIF_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_BPATCH_FAILED;
            break;
        }

        uCmd = 0x14u;
        uRet = Comif_BroadcastWrite(pBmicMgr->pComifCtx, FALSE, 0x83E, &uCmd, WRITE_1BYTE);
        if((uint8)COMIF_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_BPATCH_FAILED;
            break;
        }

        uCmd = 0x00u;
        uRet = Comif_BroadcastWrite(pBmicMgr->pComifCtx, FALSE, 0xDFF, &uCmd, WRITE_1BYTE);
        if((uint8)COMIF_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_BPATCH_FAILED;
            break;
        }
#endif

        /* Enable stack device auto addressing */
        uCmd = pBmicMgr->eCommDir | BQ7971X_CONTROL1_ADDR_WR_MSK;
        uRet = Comif_BroadcastWrite(pBmicMgr->pComifCtx, FALSE, BQ7971X_CONTROL1_OFFSET, &uCmd, WRITE_1BYTE);
        if((uint8)COMIF_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_AUTOADDREN_FAILED;
            break;
        }

        /* address 1-n assigned to stack devices, 0 assigned to base device */
        wReg = BQ7971X_DIR0_ADDR_OFFSET;
        if((uint8)DIR_COML2H == pBmicMgr->eCommDir)
        {
            wReg = BQ7971X_DIR1_ADDR_OFFSET;
        }

        for(uDevId = 0u; uDevId <= pBmicMgr->uNAFEs; uDevId++)
        {
            uRet |= Comif_BroadcastWrite(pBmicMgr->pComifCtx, FALSE, wReg, &uDevId, WRITE_1BYTE);
        }

        if((uint8)COMIF_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_ADDRASSIGN_FAILED;
            break;
        }

        /* set last as top of stack */
        uCmd = BQ7971X_COMM_CTRL_TOP_STACK_MSK;
        uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs, BQ7971X_COMM_CTRL_OFFSET, &uCmd, WRITE_1BYTE);
        if((uint8)COMIF_OK != uRet)
        {
            uRet = (uint8)BMI_BMC_TOP_STACK_FAILED;
            break;
        }

        /* TODO: Sync up internal Dll(read) */
        /* read back to verify address are correct forstack devices */
        uRet = Comif_BroadcastRead(pBmicMgr->pComifCtx, pBmicMgr->uNDevices, wReg, pSvcCfg, READ_1BYTE);
        if((uint8) COMIF_OK != uRet)
        {
            break;
        }

        uRet = BMI_AUTOADDR_SUCCESS;
    }
    while(FALSE);

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_AUTOADDRESS, uRet, pBmicMgr->uBmicIface, pBmicMgr->eDrvState);
}

/**********************************************************************************************************************
 * Function name: Bq7971x_StartupInit
 *********************************************************************************************************************/
/*! \brief          Startup Init
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      eNotifyType: Notify Type
 *  \param[in]      pSvcCfg: Service Config
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

STATIC FUNC(uint8, bq7971x_CODE) Bq7971x_StartupInit(Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = (uint8)BMI_INIT_PROGRESS;
    uint8 uCmd;

    do
    {
        if((uint8)COMIF_WIRED == pBmicMgr->eComType)
        {
            if((uint8)BQ7971X_STATE_WAKEUP == pBmicMgr->eDrvState)
            {
                pBmicMgr->nCurrNoOfDevs = 0u;
                //if((0u == pBmicMgr->uIsPmicInIf) || ((uint8)PM_DEVID_BEGIN != pBmicMgr->uSDevId))
                {
                    uRet = Comiface_WakePing(pBmicMgr->pComifCtx, BQ7971X_WAKEUP_PING_DELAY);
                    if((uint8)COMIF_OK != uRet)
                    {
                        uRet =(uint8) BMI_BMC_WAKEUP_CMD_FAILED;
                        break;
                    }
                }

                // TODO: Need to implement power state manager according to the wake up
                uCmd = (pBmicMgr->eCommDir | BQ7971X_CONTROL1_SEND_WAKE_MSK);
                uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, 0u, BQ7971X_CONTROL1_OFFSET, &uCmd, WRITE_1BYTE);
                if((uint8)COMIF_OK != uRet)
                {
                    uRet = (uint8)BMI_BMC_WAKEUP_STACK_FAILED;
                    break;
                }

                uRet = Comif_SetDelay(pBmicMgr->pComifCtx, (BQ7971X_WAKEUP_TONE_DELAY * pBmicMgr->uNAFEs));
                if((uint8)COMIF_OK != uRet)
                {
                    uRet = (uint8)BMI_BMC_STACK_DELAY_FAILED;
                    break;
                }
               
		if((1u == pBmicMgr->uIsPmicInIf) || ((uint8)PM_DEVID_BEGIN == pBmicMgr->uSDevId))
		{

                   uCmd = (pBmicMgr->eCommDir | BQ7971X_CONTROL1_SEND_WAKE_MSK);
                   uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, 0u, BQ7971X_CONTROL1_OFFSET, &uCmd, WRITE_1BYTE);
                   if((uint8)COMIF_OK != uRet)
                   {
                      uRet = (uint8)BMI_BMC_WAKEUP_STACK_FAILED;
                      break;
                   }

                   uRet = Comif_SetDelay(pBmicMgr->pComifCtx, (BQ7971X_WAKEUP_TONE_DELAY * pBmicMgr->uNAFEs));
                   if((uint8)COMIF_OK != uRet)
                   {
                       uRet = (uint8)BMI_BMC_STACK_DELAY_FAILED;
                       break;
                   }

		}

                uRet = Bq7971x_AutoAddress(pBmicMgr, pSvcCfg);
                if((uint8)COMIF_OK != uRet)
                {
                    break;
                }

                pBmicMgr->eDrvState = (uint8)BQ7971X_STATE_INIT;
                uRet = (uint8)BMI_BMC_STARTUP_SUCCESS;
            }
        }
        else
        {
            if((uint8)COMIF_WIRELESS == pBmicMgr->eComType)
            {
                pBmicMgr->nCurrNoOfDevs = pBmicMgr->uNAFEs;
				uRet = Comiface_WakePing(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs);
                if((uint8)COMIF_OK == uRet)
                {
                    pBmicMgr->eDrvState = (uint8)BQ7971X_STATE_INIT;
                    uRet = (uint8)BMI_BMC_STARTUP_SUCCESS;
                }
            }
            else
            {
                uRet = (uint8)BMI_BMC_INVALID_COMTYPE;
            }
        }

        if((uint8)BQ7971X_STATE_NORMAL == pBmicMgr->eDrvState)
        {
            uRet = (uint8)BMI_BMC_NORMAL_STATE;
        }
    }
    while(FALSE);

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_STARTUP_INIT, uRet, pBmicMgr->uBmicIface, pBmicMgr->eComType);
}


/**********************************************************************************************************************
 *  FUNC(uint8, bq7971x_CODE) Bq7971x_GetGpio(Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function Initiates the read for temperature values
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Data
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_CODE) Bq7971x_GetGpio(Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        uRet = Comif_StackRead(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs, COMIF_PRIO_GPIO(BQ7971X_GPIO1_HI_OFFSET),
                               pSvcCfg, (BQ7971X_GPIO_NUM_MAX * READ_2BYTE));
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7971x_CODE) Bq7971x_GetVCell(Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function Initiates the read for Voltage values
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Data
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           The request will queued in the bridge layer
 *
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_CODE) Bq7971x_GetVCell(Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        uRet = Comif_StackRead(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs, COMIF_PRIO_CELL((BQ7971X_VCELL18_HI_OFFSET +
                               ((BQ7971X_CELL_NUM_MAX - BQ7971X_CELL_NUM_ACT) * READ_2BYTE))),
                               pSvcCfg, (BQ7971X_CELL_NUM_ACT * READ_2BYTE));
    }

    return uRet;
}

/**********************************************************************************************************************
 * Function name: Bq7971x_CellBalProc
 *********************************************************************************************************************/
/** \brief This function is used to execute the Service ID functionalities
 *
 *  \param[in]      pBmcCtx: Manager
 *  \param[in]      uServiceTyp: Service request for process
 *  \param[in]      pData: pointer to generic data
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
uint32 CB_BalTimeChannelNum;

#define SetIndexForCBTimeRead(x, index)  ( (x) |= (0x1U << ((index) - 1U)))
#define CheckIndexForCBTimeRead(x, index) ( (x) & (0x1U << ((index) - 1U)))
#define ClearIndexForCBTimeRead(x, index) ( (x) &= (~(0x1U << ((index) - 1U))))

STATIC FUNC(uint8, bq7971x_CODE) Bq7971x_CellBalProc(Bq7971x_ManagerType *pBmicMgr, uint8 uCbReq, void *pData)
{
    const Bmi_uCbTime *pCBCfg;
    const ServiceCfgType *pSvcCfg;
    uint8 eData = 0;
    uint8 uRet = BMI_NOT_OK;
    uint8 uCmd;
    uint8 isAutoBal_u8;

    switch(uCbReq)
    {
        case (uint8)BMI_CB_GET_BALSTAT:
        {
            pSvcCfg = &pBmicMgr->pCellBalCfg[BMI_CB_GET_BALSTAT];
            if((uint8)BMI_CB_GET_BALSTAT == pSvcCfg->uSubId)
            {
                uRet = Comif_StackRead(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs, BQ7971X_BAL_STAT_OFFSET,
                                       pSvcCfg, READ_1BYTE);
            }
            break;
        }
        case (uint8)BMI_CB_GET_BALSWSTAT:
        {
            pSvcCfg = &pBmicMgr->pCellBalCfg[BMI_CB_GET_BALSWSTAT];
            if((uint8)BMI_CB_GET_BALSWSTAT == pSvcCfg->uSubId)
            {
                uRet = Comif_StackRead(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs, BQ7971X_BAL_SW_STAT1_OFFSET,
                                       pSvcCfg, READ_3BYTE);
            }
            break;
        }
        case (uint8)BMI_CB_GET_BALDONESTAT:
        {
            pSvcCfg = &pBmicMgr->pCellBalCfg[BMI_CB_GET_BALDONESTAT];
            if((uint8)BMI_CB_GET_BALDONESTAT == pSvcCfg->uSubId)
            {
                uRet = Comif_StackRead(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs, BQ7971X_BAL_DONE1_OFFSET,
                                       pSvcCfg, READ_3BYTE);
            }
            break;
        }
        case (uint8)BMI_CB_GET_BALTIME:
        {
            if(NULL != pData)
            {
                eData = (uint8) (*((uint8 *) pData)); // Channel from Application
                uCmd = (((eData & (BQ7971X_BAL_CTRL3_BAL_TIME_SEL_MSK >> BQ7971X_BAL_CTRL3_BAL_TIME_SEL_POS)) - 1u) \
                                   << BQ7971X_BAL_CTRL3_BAL_TIME_SEL_POS) | BQ7971X_BAL_CTRL3_BAL_TIME_GO_MSK;
		        SetIndexForCBTimeRead(CB_BalTimeChannelNum, eData);
                uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_BAL_CTRL3_OFFSET, &uCmd, WRITE_1BYTE);
                if((uint8)BMI_OK == uRet)
                {
                    pSvcCfg = &pBmicMgr->pCellBalCfg[BMI_CB_GET_BALTIME];
                    if((uint8)BMI_CB_GET_BALTIME == pSvcCfg->uSubId)
                    {
                        uRet = Comif_StackRead(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs, BQ7971X_BAL_TIME_OFFSET,
                                               pSvcCfg, READ_1BYTE);
                    }
                }
            }
            break;
        }
        case (uint8)BMI_CB_SET_VCBDONETH:
        {
            if(NULL != pData)
            {
                eData = (uint8) (*((uint8 *) pData));
                uCmd = (eData & BQ7971X_VCBDONE_THRESH_CB_THR_MSK);
                uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_VCBDONE_THRESH_OFFSET, &uCmd, WRITE_1BYTE);
                if((uint8)BMI_OK == uRet)
                {
                    uCmd = (pBmicMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uOVUV_Ctrl[1u] | BQ7971X_OVUV_CTRL2_OVUV_GO_MSK);
                    uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_OVUV_CTRL2_OFFSET, &uCmd, WRITE_1BYTE);
                }
            }
            break;
        }
        case (uint8)BMI_CB_SET_OTCBTH:
        {
            if(NULL != pData)
            {
                eData = (uint8) (*((uint8 *) pData));
                uCmd = eData & BQ7971X_OTCB_THRESH_OTCB_THR_MSK;
                uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_OTCB_THRESH_OFFSET, &uCmd, WRITE_1BYTE);
                if((uint8)BMI_OK == uRet)
                {
                    uCmd = (pBmicMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uOTUT_Ctrl[1u] | BQ7971X_OTUT_CTRL2_OTUT_GO_MSK);
                    uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_OTUT_CTRL2_OFFSET, &uCmd, WRITE_1BYTE);
                }
            }
            break;
        }
        case (uint8)BMI_CB_SET_BALDUTY:
        {
            if(NULL != pData)
            {
                eData = (uint8) (*((uint8 *) pData));
                uCmd = eData & BQ7971X_BAL_CTRL1_PWM_MSK;
                uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_BAL_CTRL1_OFFSET, &uCmd, WRITE_1BYTE);
            }
            break;
        }
        case (uint8)BMI_CB_SET_FLTSTOP_EN:
        {
            uCmd = BQ7971X_BAL_CTRL2_FLTSTOP_EN_MSK;
            uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_BAL_CTRL2_OFFSET, &uCmd, WRITE_1BYTE);
            break;
        }
        case (uint8)BMI_CB_SET_BAL_ACT:
        {
            uCmd = BQ7971X_BAL_CTRL2_BAL_ACT_MSK;
            uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_BAL_CTRL2_OFFSET, &uCmd, WRITE_1BYTE);
            break;
        }
        case (uint8)BMI_CB_SET_AUTO_BAL:
        {
            uCmd = BQ7971X_BAL_CTRL2_AUTO_BAL_MSK;
            uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_BAL_CTRL2_OFFSET, &uCmd, WRITE_1BYTE);
            break;
        }
        case (uint8)BMI_CB_SET_CBTIME:
        {
            pCBCfg = (const Bmi_uCbTime *) pData;
            if(pCBCfg != NULL)
            {
                uRet = Bq7971x_SetCbTime(pBmicMgr, pCBCfg);
            }
            break;
        }
        case (uint8)BMI_CB_START_BALCTRL:
        {
            isAutoBal_u8 = (uint8) (*((uint8 *) pData));

            uRet = Bq7971x_StartBalCtrl(pBmicMgr, BQ7971X_BAL_CTRL2_OFFSET, BQ7971X_CB_APP_RUN, BQ7971X_CB_APP_CTRL, isAutoBal_u8);
            break;
        }
        case (uint8)BMI_CB_PAUSE_BALCTRL:
        {
            uRet = Bq7971x_PauseBalCtrl(pBmicMgr, BQ7971X_BAL_CTRL2_OFFSET, BQ7971X_CB_APP_PAUSE, BQ7971X_CB_APP_CTRL);
            break;
        }
        case (uint8)BMI_CB_UNPAUSE_BALCTRL:
        {
            uRet = Bq7971x_UnpauseBalCtrl(pBmicMgr, BQ7971X_BAL_CTRL2_OFFSET, BQ7971X_CB_APP_UNPAUSE, BQ7971X_CB_APP_CTRL);
            break;
        }
        default:
        {
            break;
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_CB_PROC, uRet, uCbReq, eData);
}

/**********************************************************************************************************************
 *  External Functions Definition
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * uint8 Bq7971x_StartBalCtrl(const Bq7971x_ManagerType *pBmicMgr, uint32 qReg, uint8 uRequest,uint8 uCtrlSrc)
 *********************************************************************************************************************/
/*! \brief          Function sets the cell balancing time
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      qReg: Register to Write including Exclusive settings
 *  \param[in]      uRequest: Request for the Balance Control
 *  \param[in]      uCtrlSrc: Source Requesting for the Balance Control or switching to new Source
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7971x_CODE) Bq7971x_StartBalCtrl(const Bq7971x_ManagerType *pBmicMgr, uint32 qReg, uint8 uRequest,
                                               uint8 uCtrlSrc, uint8 isAutoBal_u8)
{
    uint8 ucmd = BQ7971X_BAL_CTRL2_BAL_GO_MSK;
    uint8 uRet = (uint8)BMI_BMC_INVALID_CBCTRL_CTX;

    if(isAutoBal_u8 != 0u)
    {
        ucmd |= BQ7971X_BAL_CTRL2_AUTO_BAL_MSK;
    }
    else
    {
        /* Do nothing */
    }

    if(((uint8)BQ7971X_CB_APP_CTRL == pBmicMgr->pCbCtrl->uCbSrcCtrl) || (uRequest == (uint8) BQ7971X_CB_DIAG_RUN))
    {
        uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, qReg, &ucmd, WRITE_1BYTE);
        if((uint8)BMI_OK == uRet)
        {
            pBmicMgr->pCbCtrl->uCbStatus = uRequest;
            pBmicMgr->pCbCtrl->uCbSrcCtrl = uCtrlSrc;
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_SETCB_CTRL, uRet, uRequest, pBmicMgr->pCbCtrl->uCbSrcCtrl);
}

/**********************************************************************************************************************
 * uint8 Bq7971x_PauseBalCtrl(const Bq7971x_ManagerType *pBmicMgr, uint32 qReg, uint8 uRequest, uint8 uCtrlSrc)
 *********************************************************************************************************************/
/*! \brief          Function sets the cell balancing time
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      qReg: Register to Write including Exclusive settings
 *  \param[in]      uRequest: Request for the Balance Control
 *  \param[in]      uCtrlSrc: Source Requesting for the Balance Control
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7971x_CODE) Bq7971x_PauseBalCtrl(const Bq7971x_ManagerType *pBmicMgr, uint32 qReg, uint8 uRequest,
                                               uint8 uCtrlSrc)
{
    uint8 ucmd = BQ7971X_BAL_CTRL2_CB_PAUSE_MSK;
    uint8 uRet = (uint8)BMI_BMC_INVALID_CBCTRL_CTX;

    if(((uint8)BQ7971X_CB_APP_CTRL == pBmicMgr->pCbCtrl->uCbSrcCtrl) || (uRequest == (uint8) BQ7971X_CB_DIAG_PAUSE))
    {
        uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, qReg, &ucmd, WRITE_1BYTE);
        if((uint8)BMI_OK == uRet)
        {
            pBmicMgr->pCbCtrl->uCbStatus = uRequest;
            pBmicMgr->pCbCtrl->uCbSrcCtrl = uCtrlSrc;
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_SETCB_CTRL, uRet, uRequest, pBmicMgr->pCbCtrl->uCbSrcCtrl);
}

/**********************************************************************************************************************
 * uint8 Bq7971x_UnpauseBalCtrl(Bq7971x_ManagerType *pBmicMgr, uint32 qReg, uint8 uRequest, uint8 uCtrlSrc)
 *********************************************************************************************************************/
/*! \brief          Function sets the cell balancing time
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      qReg: Register to Write including Exclusive settings
 *  \param[in]      uRequest: Request for the Balance Control
 *  \param[in]      uCtrlSrc: Source Requesting for the Balance Control
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7971x_CODE) Bq7971x_UnpauseBalCtrl(const Bq7971x_ManagerType *pBmicMgr, uint32 qReg, uint8 uRequest,
                                                 uint8 uCtrlSrc)
{
    uint8 ucmd = 0x00;
    uint8 uRet = (uint8)BMI_BMC_INVALID_CBCTRL_CTX;

    if(((uint8)BQ7971X_CB_APP_CTRL == pBmicMgr->pCbCtrl->uCbSrcCtrl) || (uRequest == (uint8) BQ7971X_CB_DIAG_UNPAUSE))
    {
        uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, qReg, &ucmd, WRITE_1BYTE);
        if((uint8)BMI_OK == uRet)
        {
            pBmicMgr->pCbCtrl->uCbStatus = uRequest;
            pBmicMgr->pCbCtrl->uCbSrcCtrl = uCtrlSrc;
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_SETCB_CTRL, uRet, uRequest, pBmicMgr->pCbCtrl->uCbSrcCtrl);
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) Bq7971x_GetBalCtrlStatus(Bq7971x_ManagerType *pBmicMgr)
 *********************************************************************************************************************/
/*! \brief          Function Getting the cell balancing status
 *
 *  \param[in]      pBmicMgr: Manager
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7971x_CODE) Bq7971x_GetBalCtrlStatus(const Bq7971x_ManagerType *pBmicMgr)
{
    return (uint8)pBmicMgr->pCbCtrl->uCbStatus;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7971x_CODE) Bq7971x_SetCbTime(const Bq7971x_ManagerType *pBmicMgr, const Bmi_CBCfgType *pCBCfg)
 *********************************************************************************************************************/
/*! \brief          Function Starts the Balance Control
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pCBCfg: CellBalancing Configuration
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

FUNC(uint8, bq7971x_CODE) Bq7971x_SetCbTime(const Bq7971x_ManagerType *pBmicMgr, const Bmi_uCbTime * uCbTime )
{
    uint8 uCbCtrl[BQ7971X_CELL_NUM_ACT];
    uint8 uRet = (uint8)BMI_BMC_INVALID_CBCTRL;
    uint8 uDevId;
    uint8 uLoop = pBmicMgr->uSDevId;
    uint8 uCellIdx;
    uint8 uVal;
    uint8 numOfCellsPending;
    uint16 startCbTimeReg_u16;

    for (uDevId = uLoop; uDevId < pBmicMgr->uNAFEs; uDevId++)
    {
        if(uCbTime != NULL)
        {
            for (uCellIdx = 0u; uCellIdx < BQ7971X_CELL_NUM_ACT; uCellIdx++)
            {
                uVal = 0u;
                uVal = (*uCbTime)[uDevId][uCellIdx];
                uCbCtrl[BQ7971X_CELL_NUM_ACT - 1u - uCellIdx] = uVal;
            }

            numOfCellsPending = BQ7971X_CELL_NUM_ACT;
            startCbTimeReg_u16 = BQ7971X_CB_CELL18_CTRL_OFFSET + ( BQ7971X_CELL_NUM_MAX - BQ7971X_CELL_NUM_ACT);

            uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, (uDevId+1u), startCbTimeReg_u16, &uCbCtrl[0], 8u);
            if((uint8) BMI_OK == uRet)
            {
                numOfCellsPending -= 8U;
                startCbTimeReg_u16 += 8U;

		if (numOfCellsPending >= 8U)
		{
                   uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, (uDevId+1u), startCbTimeReg_u16,
                                         &uCbCtrl[8], 8u);
                   numOfCellsPending -= 8U;
                   startCbTimeReg_u16 += 8U;
                }
		else
		{
                   uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, (uDevId+1u), startCbTimeReg_u16,
                                         &uCbCtrl[8], numOfCellsPending);
                   numOfCellsPending = 0U;
		}


                if((uint8)BMI_OK == uRet)
                {
                    if (numOfCellsPending > 0U)
		    {
                       uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, (uDevId+1u), startCbTimeReg_u16,
                                         &uCbCtrl[8], numOfCellsPending);
                    }
		    else
                    {
                       /* Do nothing */
		    }
                }
            }
        }

        if((uint8)BMI_OK != uRet)
        {
            break;
        }
    }

    return Bmi_SetErrorDetails(BMI_SVCID_BMC_SETCBTIME, uRet, pBmicMgr->uBmicIface, pBmicMgr->eDrvState);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7971x_CODE) Bq7971x_SetDiagAdcCtrl1(Bq7971x_ManagerType *pBmicMgr, uint8 uAdcCtrl)
 *********************************************************************************************************************/
/*! \brief          Function Sets the Diag ADC Control 1 register
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      uAdcCtrl: Control Value
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

FUNC(uint8, bq7971x_CODE) Bq7971x_SetDiagAdcCtrl1(const Bq7971x_ManagerType *pBmicMgr, uint8 uAdcCtrl1)
{
    uint8 uRet;

    uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_DIAG_ADC_CTRL1_OFFSET, &uAdcCtrl1, WRITE_1BYTE);

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7971x_CODE) Bq7971x_SetDiagAdcCtrl2(Bq7971x_ManagerType *pBmicMgr, uint8 uAdcCtrl)
 *********************************************************************************************************************/
/*! \brief          Function Sets the Diag ADC Control 3 register
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      uAdcCtrl: Control Value
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

FUNC(uint8, bq7971x_CODE) Bq7971x_SetDiagAdcCtrl2(const Bq7971x_ManagerType *pBmicMgr, uint8 uAdcCtrl2)
{
    uint8 uRet;

    uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_DIAG_ADC_CTRL2_OFFSET, &uAdcCtrl2, WRITE_1BYTE);

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7971x_CODE) Bq7971x_SetDiagAdcCtrl3(Bq7971x_ManagerType *pBmicMgr, uint8 uAdcCtrl)
 *********************************************************************************************************************/
/*! \brief          Function Sets the Diag ADC Control 3 register
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      uAdcCtrl: Control Value
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

FUNC(uint8, bq7971x_CODE) Bq7971x_SetDiagAdcCtrl3(const Bq7971x_ManagerType *pBmicMgr, uint8 uAdcCtrl3)
{
    uint8 uRet;

    uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_DIAG_ADC_CTRL3_OFFSET, &uAdcCtrl3, WRITE_1BYTE);

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) Bq7971x_DevConfInit(Bq7971x_ManagerType *pBmicMgr)
 *********************************************************************************************************************/
/*! \brief          Function initializes the Device Configuration
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

FUNC(uint8, bq7971x_CODE) Bq7971x_DevConfInit(Bq7971x_ManagerType *pBmicMgr)
{
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;
    uint8 uDevId;
    uint8 uLoop = pBmicMgr->uSDevId;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        if(pBmicMgr->pBmicCfg->pBqRegsCfg->uCellCfgPerDevInit != 0u)
        {
            for(uDevId = uLoop; uDevId < pBmicMgr->uNAFEs; uDevId++)
            {
                uRet |= Comif_SingleWrite(pBmicMgr->pComifCtx, (uDevId+1u), BQ7971X_DEV_CONF1_OFFSET,
                                   &pBmicMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevId-pBmicMgr->uSDevId].uDev_Conf[0],
                                   WRITE_2BYTE);
            }
        }
        else
        {
            //initialize DEV_CONF
            uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_DEV_CONF1_OFFSET,
                               &pBmicMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uDev_Conf[0], WRITE_2BYTE);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) Bq7971x_Control2Init(Bq7971x_ManagerType *pBmicMgr)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Control 2
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

FUNC(uint8, bq7971x_CODE) Bq7971x_Control2Init(Bq7971x_ManagerType *pBmicMgr)
{
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        //initialize CONTROL2 Register
        uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_CONTROL2_OFFSET,
                               &pBmicMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uControl[1u], WRITE_1BYTE);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) Bq7971x_GpioConfInit(const Bq7971x_ManagerType *pBmicMgr)
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

FUNC(uint8, bq7971x_CODE) Bq7971x_GpioConfInit(const Bq7971x_ManagerType *pBmicMgr)
{
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;
    uint8 uDevId;
    uint8 uLoop = pBmicMgr->uSDevId;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        uRet = (uint8)BMI_BMC_GPIOCFG_FAILED;

        //initialize GPIO CONF Register
        if(pBmicMgr->pBmicCfg->pBqRegsCfg->uGpioCfgPerDevInit != 0u)
        {
            for(uDevId = uLoop; uDevId < pBmicMgr->uNAFEs; uDevId++)
            {
                uRet |= Comif_SingleWrite(pBmicMgr->pComifCtx, (uDevId+1u),
                                          BQ7971X_GPIO_CONF1_OFFSET,
                                &pBmicMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevId-pBmicMgr->uSDevId].uGpio_Conf[0],
                                          BQ7971X_GPIO_CONF_REG_NUM);
            }
        }
        else
        {
            uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE,
                                    BQ7971X_GPIO_CONF1_OFFSET,
                                    &pBmicMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uGpio_Conf[0],
                                    BQ7971X_GPIO_CONF_REG_NUM);
        }

    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) Bq7971x_AdcCtrlInit(Bq7971x_ManagerType *pBmicMgr)
 *********************************************************************************************************************/
/*! \brief          Function Starts the Balance Control
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

FUNC(uint8, bq7971x_CODE) Bq7971x_AdcCtrlInit(Bq7971x_ManagerType *pBmicMgr)
{
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        //initialize ADC_CTRLx
        uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_ADC_CTRL1_OFFSET,
                               &pBmicMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uAdc_Ctrl[0],
                               BQ7971X_ADC_CTRL_REG_NUM);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) Bq7971x_OvUvThreshInit(Bq7971x_ManagerType *pBmicMgr)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the OvUv Threshhold
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

FUNC(uint8, bq7971x_CODE) Bq7971x_OvUvThreshInit(Bq7971x_ManagerType *pBmicMgr)
{
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        //initialize OvUvThresh
        uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_OV_THRESH_OFFSET,
                               &pBmicMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uOVUV_Thresh[0],
                               WRITE_2BYTE);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) Bq7971x_OtUtThreshInit(Bq7971x_ManagerType *pBmicMgr)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the OvUv Threshhold
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

FUNC(uint8, bq7971x_CODE) Bq7971x_OtUtThreshInit(Bq7971x_ManagerType *pBmicMgr)
{
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        //initialize OvUvThresh
        uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_OTUT_THRESH_OFFSET,
                               &pBmicMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uOTUT_Thresh,
                               WRITE_1BYTE);
    }

    return uRet;
}

/**********************************************************************************************************************
 * Function name: Bq7971x_ControlProc
 *********************************************************************************************************************/
/** \brief This function is used to execute the control functionalities and generic commands
 *
 *  \param[in]      pBmcCtx: Manager
 *  \param[in]      uCmd: command request for process
 *  \param[in]      pData: pointer to generic data
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

FUNC(uint8, bq7971x_CODE) Bq7971x_ControlProc(void *pBmcCtx, uint8 uCmd, void *pData, uint16 wLength)
{
    Bq7971x_ManagerType *pBmicMgr = (Bq7971x_ManagerType *) pBmcCtx;
    const ServiceCfgType *pFuncSvcCfg;
    VersionInfoType *pBmcVer;
    uint8 uRet = (uint8)BMI_NOT_OK;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        switch(uCmd)
        {
            case (uint8)BMI_GET_DRIVER_STATE:
            {
                uRet = (uint8)BMI_INIT_PROGRESS;
                if((uint8)BQ7971X_STATE_NORMAL == pBmicMgr->eDrvState)
                {
                    uRet = (uint8)BMI_BMC_NORMAL_STATE;
                }
                break;
            }
            case (uint8)BMI_GET_DIAG_STATE:
            {
                uRet = Bq7971x_GetDiagInitState(pBmicMgr->pBmicDiag);
                break;
            }
            case (uint8)BMI_SET_ADC_CTRL1:
            {
                uRet = Bq7971x_SetDiagAdcCtrl1(pBmicMgr,wLength);
                break;
            }
            case (uint8)BMI_SET_ADC_CTRL2:
            {
                uRet = Bq7971x_SetDiagAdcCtrl2(pBmicMgr,wLength);
                break;
            }
            case (uint8)BMI_SET_ADC_CTRL3:
            {
                uRet = Bq7971x_SetDiagAdcCtrl3(pBmicMgr,wLength);
                break;
            }
            case (uint8)BMI_GET_DIAG_STATUS:
            {
                if((NULL != pData) && (wLength > 0u))
                {
                    pFuncSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_DIAGNOSTICS];
                    if((NULL != pFuncSvcCfg) && (NULL != pFuncSvcCfg->pDecodeBuf2))
                    {
                        pFuncSvcCfg->pSvcStat->uInfo = (uint8)RESP_USED;

                        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                        (void)memcpy(pData, pFuncSvcCfg->pDecodeBuf2, sizeof(DiagStatusType));
                        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

                        uRet = (uint8)BMI_OK;
                    }
                }
                break;
            }
            case (uint8)BMI_GET_DIAG_RESULT:
            {
                if((NULL != pData) && (wLength > 0u))
                {
                    pFuncSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_DIAGNOSTICS];
                    if((NULL != pFuncSvcCfg) && (NULL != pFuncSvcCfg->pDecodeBuf1))
                    {
                        pFuncSvcCfg->pSvcStat->uInfo = (uint8)RESP_USED;
                        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                        (void)memcpy(pData, pFuncSvcCfg->pDecodeBuf1, (sizeof(DiagResultType) * pBmicMgr->uNBmics));           // TBD: Need to check this how Diagnostics is copied
                        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

                        uRet = (uint8)BMI_OK;
                    }
                }
                break;
            }
            case (uint8)BMI_GET_VERSION:
            {
                pBmcVer = (VersionInfoType *) pData;
                if(pBmcVer != NULL)
                {
                    pBmcVer->zBmcVersion.vendorID = BQ7971X_VENDOR_ID;
                    pBmcVer->zBmcVersion.moduleID = BQ7971X_MODULE_ID;
                    pBmcVer->zBmcVersion.sw_major_version = BQ7971X_SW_MAJOR_VERSION;
                    pBmcVer->zBmcVersion.sw_minor_version = BQ7971X_SW_MINOR_VERSION;
                    pBmcVer->zBmcVersion.sw_patch_version = BQ7971X_SW_PATCH_VERSION;

                    uRet = Comif_GetVersion(pBmicMgr->pComifCtx, pBmcVer);
                }
                break;
            }
            default:
            {
                uRet = (uint8)BMI_BMC_INVALID_CTRL_CMD;
                break;
            }
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * Function name: Bq7971x_ServiceProc
 *********************************************************************************************************************/
/** \brief This function is used to execute the Service ID functionalities
 *
 *  \param[in]      pBmcCtx: Manager
 *  \param[in]      uServiceTyp: Service request for process
 *  \param[in]      pData: pointer to generic data
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

FUNC(uint8, bq7971x_CODE) Bq7971x_ServiceProc(void *pBmcCtx, uint8 uServiceTyp, uint8 uCmd, uint8 uNodeId, void *pData)
{
    Bq7971x_ManagerType *pBmicMgr = (Bq7971x_ManagerType *)  pBmcCtx;
    const ServiceCfgType *pSvcCfg = NULL;
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        switch(uServiceTyp)
        {
           case (uint8)BMI_SERVICE_VBAT:
           {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_VBAT];
                if((pSvcCfg != NULL) && (pSvcCfg->pSvcStat != NULL))
                {
                    pSvcCfg->pSvcStat->uRespStatus = (uint8) RESP_BUSY;
                    uRet = BQ7971X_GetVBat(pBmicMgr, pSvcCfg);
                }
                break;
            }
            case (uint8)BMI_SERVICE_VCAP_REF:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_VCAP_REF];
                if((pSvcCfg != NULL) && (pSvcCfg->pSvcStat != NULL))
                {
                    pSvcCfg->pSvcStat->uRespStatus = (uint8) RESP_BUSY;
                    uRet = BQ7971X_GetVRefCap(pBmicMgr, pSvcCfg);
                }
                break;
            }
            case (uint8)BMI_SERVICE_CELL_VOLT:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_CELL_VOLT];
                if((NULL != pSvcCfg) && (NULL != pSvcCfg->pSvcStat))
                {
                    pSvcCfg->pSvcStat->uRespStatus = (uint8)RESP_BUSY;
                    uRet = Bq7971x_GetVCell(pBmicMgr, pSvcCfg);
                }
                break;
            }
            case (uint8)BMI_SERVICE_BALANCING:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_BALANCING];
                if((NULL != pSvcCfg) && (NULL != pSvcCfg->pSvcStat))
                {
                    uRet = Bq7971x_CellBalProc(pBmicMgr, uCmd, pData);
                }
                break;
            }
            case (uint8)BMI_SERVICE_GPIO_READ:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_GPIO_READ];
                if((NULL != pSvcCfg) && (NULL != pSvcCfg->pSvcStat))
                {
                    pSvcCfg->pSvcStat->uRespStatus = (uint8)RESP_BUSY;
                    uRet = Bq7971x_GetGpio(pBmicMgr, pSvcCfg);
                }
                break;
            }
            case (uint8)BMI_SERVICE_RESET:
            case (uint8)BMI_SERVICE_POWER_CONTROL:
            {
                (void)BQ7971X_DevPowerManager(pBmicMgr, uCmd);
                break;
            }

            case (uint8)BMI_SERVICE_DIAGNOSTICS:
            {
					pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_DIAGNOSTICS];
					if(pSvcCfg != NULL)
					{
						uRet = Bq7971x_DiagServiceReq(pBmicMgr->pBmicDiag, uCmd, pData , uNodeId);
					}
                break;
            }
            case (uint8)BMI_SERVICE_COMMUNICATION:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_COMMUNICATION];
                if(pSvcCfg != NULL)
                {
					uRet = Comiface_HandleRequest(pBmicMgr->pComifCtx, uCmd, uNodeId, pData);
                }
                break;
            }
            case (uint8)BMI_SERVICE_CONFIG:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_CONFIG];
                if((NULL != pSvcCfg) && (NULL != pSvcCfg->pSvcStat))
                {
                    pSvcCfg->pSvcStat->uRespStatus = (uint8)RESP_BUSY;
                    if(pData != NULL)
                    {
                        uRet = Bq7971x_StartupInit(pBmicMgr, pSvcCfg);
                    }
                }
                break;
            }
            case (uint8)BMI_SERVICE_SET_COMM_CONF:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_SET_COMM_CONF];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_SetCommConf(pBmicMgr, uNodeId);
                }
                break;
            }
            case (uint8)BMI_SERVICE_SET_BB_CONF:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_SET_BB_CONF];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_SetBBConf(pBmicMgr, uNodeId);
                }
                break;
            }
            case (uint8)BMI_SERVICE_SET_ENB_SPI:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_SET_ENB_SPI];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_SetEnableSPI(pBmicMgr, uNodeId);
                }
                break;
            }
            case (uint8)BMI_SERVICE_SET_SPI_CONF:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_SET_SPI_CONF];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_SetSPIConf(pBmicMgr, uNodeId);
                }
                break;
            }
            case (uint8)BMI_SERVICE_SET_SPI_EXE:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_SET_SPI_EXE];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_SetSPIExe(pBmicMgr, uNodeId,uCmd);
                }
                break;
            }
            case (uint8)BMI_SERVICE_WRITE_SPI_DATA:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_WRITE_SPI_DATA];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_WriteSPIData(pBmicMgr, uNodeId ,pData);
                }
                break;
            }
            case (uint8)BMI_SERVICE_READ_SPI_DATA:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_READ_SPI_DATA];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_ReadSPIData(pBmicMgr, pSvcCfg);
                }
                break;
            }
            case (uint8)BMI_SERVICE_SET_ENB_I2C:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_SET_ENB_I2C];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_SetEnableI2C(pBmicMgr, pSvcCfg, uNodeId);
                }
                break;
            }
            case (uint8)BMI_SERVICE_SET_I2C_CTRL:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_SET_I2C_CTRL];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_SetI2CCtrl(pBmicMgr, pSvcCfg, uNodeId,uCmd);
                }
                break;
            }
            case (uint8)BMI_SERVICE_SET_I2C_DATA:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_SET_I2C_DATA];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_SetI2CData(pBmicMgr, pSvcCfg, uNodeId,uCmd);
                }
                break;
            }
            case (uint8)BMI_SERVICE_GET_I2C_DATA:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_GET_I2C_DATA];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_GetI2CData(pBmicMgr, pSvcCfg);
                }
                break;
            }
            case (uint8)BMI_SERVICE_GET_I2C_FAULT:
            {

                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_GET_I2C_FAULT];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_GetI2CFault(pBmicMgr, pSvcCfg);
                }
                break;
            }
            case (uint8)BMI_SERVICE_GET_DIAG_MAIN:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_GET_DIAG_MAIN];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_GetDiagMain(pBmicMgr,pSvcCfg);
                }
                break;
            }
            case (uint8)BMI_SERVICE_GET_DIAG_RDNT:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_GET_DIAG_RDNT];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_GetDiagRdnt(pBmicMgr,pSvcCfg);
                }
                break;
            }
            case (uint8)BMI_SERVICE_SET_GPIO_CONF:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_SET_GPIO_CONF];
                if((pSvcCfg != NULL) && (pData != NULL))
                {
                    uRet = BQ7971X_GpioAdjSetGpioConf(pBmicMgr, ( Gpio_conf *)pData, pSvcCfg);
                }
                break;
            }
            case (uint8)BMI_SERVICE_SET_GPIO_DOWN:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_SET_GPIO_DOWN];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_GpioOpenSetGpioDown(pBmicMgr);
                }
                break;
            }
            case (uint8)BMI_SERVICE_GET_VCELL_ACTSUM:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_GET_VCELL_ACTSUM];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_GetVcellActSum(pBmicMgr,pSvcCfg);
                }
                break;
            }
            case (uint8)BMI_SERVICE_GET_DIE_TEMP:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_GET_DIE_TEMP];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_GetDieTemp(pBmicMgr,pSvcCfg);
                }
                break;
            }
            case (uint8)BMI_SERVICE_EN_ADC_FREEZE:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_EN_ADC_FREEZE];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_EnADCFreeze(pBmicMgr);
                }
                break;
            }
            case (uint8)BMI_SERVICE_DIS_ADC_FREEZE:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_DIS_ADC_FREEZE];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_DisADCFreeze(pBmicMgr);
                }
                break;
            }
            case (uint8)BMI_SERVICE_SET_ADC_DELAY:
            {
                pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_SET_ADC_DELAY];
                if(pSvcCfg != NULL)
                {
                    uRet = BQ7971X_SetADCDelay(pBmicMgr, uCmd,uNodeId);
                }
                break;
            }
            default:
            {
                uRet = (uint8)BMI_BMC_INVALID_SVC_CMD;
                break;
            }
        }

        if((uint8)BMI_OK == uRet)
        {
            pBmicMgr->pBswCfg->timestamp_req(&pSvcCfg->pSvcStat->cReqTimeMs, NULL);
            pSvcCfg->pSvcStat->nReqMsgs++;
        }

        uRet = Bmi_SetErrorDetails(BMI_SVCID_BMC_SERVICE_PROC, uRet, pBmicMgr->uBmicIface, uServiceTyp);
    }

    return uRet;
}

/**********************************************************************************************************************
 * Function name: Bq7971x_TxHandlerProc
 *********************************************************************************************************************/
/** \brief This function is used to execute the Service ID functionalities
 *
 *  \param[in]      pBmcCtx: Manager
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

FUNC(uint8, bq7971x_CODE) Bq7971x_TxHandlerProc(void *pBmcCtx)
{
    Bq7971x_ManagerType *pBmicMgr = (Bq7971x_ManagerType *)  pBmcCtx;
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        uRet = Comif_HandleTransfer(pBmicMgr->pComifCtx);
    }

    return uRet;
}

/**********************************************************************************************************************
 * Function name: Bq7971x_RxHandlerProc
 *********************************************************************************************************************/
/** \brief This function is used to execute the Rx Handler
 *
 *  \param[in]      pBmcCtx: Manager
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

FUNC(uint8, bq7971x_CODE) Bq7971x_RxHandlerProc(void *pBmcCtx)
{
    Bq7971x_ManagerType *pBmicMgr = (Bq7971x_ManagerType *)  pBmcCtx;
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        uRet = Comif_HandleRecieve(pBmicMgr->pComifCtx);
    }

    return uRet;
}

/**********************************************************************************************************************
 * Function name: Bq7971x_NotifyProc
 *********************************************************************************************************************/
/** \brief This function is used to Notify processing
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

FUNC(uint8, bq7971x_CODE) Bq7971x_NotifyProc(void *pBmcCtx, uint8 uType)
{
    Bq7971x_ManagerType *pBmicMgr = (Bq7971x_ManagerType *)  pBmcCtx;
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        uRet = Comif_HandleNotify(pBmicMgr->pComifCtx, uType);
    }

    return uRet;
}

/**********************************************************************************************************************
 * Function name: Bq7971x_DecodeGpio
 *********************************************************************************************************************/
/** \brief This function is used to Decode the GPIO Buffer
 *
 *  \param[in]      pBmcCtx: Manager
 *  \param[in]      uType: Type of the Notify
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *
 *  \return         uint8
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeGpio(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg,
                                             uint16 *pGpioDecVolt)
{
    uint8 *pData;
    uint8 uRet = (uint8)BMI_NOT_OK;
    uint8 uDevIdx;
    uint8 uGpioIdx;
    uint8 uSrcIdx = 0;
    uint8 uLoop = pBmicMgr->uSDevId;

    if((NULL != pGpioDecVolt) && ((uint8)RESP_VALID == pSvcCfg->pSvcStat->uRespStatus))
    {
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
        for(uDevIdx = uLoop; uDevIdx < pBmicMgr->uNAFEs; uDevIdx++)
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pBmicMgr->uNAFEs, uDevIdx);
            for(uGpioIdx = 0 ; uGpioIdx < BQ7971X_GPIO_NUM_MAX ; uGpioIdx++)
            {
                if(TIBMS_BitTest(pBmicMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNGpioCfg, uGpioIdx) != 0u)
                {
                    pGpioDecVolt[((uDevIdx-pBmicMgr->uSDevId)*BQ7971X_GPIO_NUM_MAX) + uGpioIdx] = \
                                  (((uint16) pData[uSrcIdx] << 8u) | pData[uSrcIdx+1u]);
		            uSrcIdx += 2U;
                }
            }
        }
        pSvcCfg->pSvcStat->uInfo = (uint8)RESP_DECODED;
        uRet = (uint8)BMI_OK;
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);
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

FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeVcell(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg,
                                              uint16 *pOutVcellVolt)
{
    uint8 *pData;
    uint8 uRet = (uint8)BMI_NOT_OK;
    uint8 uDevIdx;
    uint8 uCellIdx;
    uint8 uSrcIdx;
    uint8 uLoop = pBmicMgr->uSDevId;

    if((NULL != pOutVcellVolt) && ((uint8)RESP_VALID == pSvcCfg->pSvcStat->uRespStatus))
    {
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
        for(uDevIdx = uLoop; uDevIdx < pBmicMgr->uNAFEs; uDevIdx++)
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pBmicMgr->uNAFEs, uDevIdx);
            for(uCellIdx = 0; uCellIdx < BQ7971X_CELL_NUM_ACT; uCellIdx++)
            {
                if(TIBMS_BitTest(pBmicMgr->pBmicCfg->pBqRegsCfg->pDevRegCfg[uDevIdx].uNCellsCfg, uCellIdx) != 0u)
                {
                    uSrcIdx = BQ7971X_CELL_NUM_ACT - uCellIdx - 1u;
                    pOutVcellVolt[((uDevIdx-pBmicMgr->uSDevId)*BQ7971X_CELL_NUM_ACT) + uCellIdx] = \
                                   (((uint16) pData[uSrcIdx+(1u*uSrcIdx)] << 8) | pData[uSrcIdx+(1u*uSrcIdx)+1u]);
                }
            }
        }
        pSvcCfg->pSvcStat->uInfo = (uint8)RESP_DECODED;
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

        uRet = (uint8)BMI_OK;
    }

    return uRet;
}

/**********************************************************************************************************************
 * uint8 Bq7971x_DecodeVBat(Bq7971x_ManagerType *pBmcCtx, const ServiceCfgType *pSvcCfg, uint16 *pOutVcellVolt)
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

FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeVBat(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg,
                                              uint16 *pOutVcellVolt)
{
    uint8 *pData;
    uint8 uRet = (uint8) BMI_NOT_OK;
    uint8 uDevIdx;
    uint8 uLoop = pBmicMgr->uSDevId;


    if((pOutVcellVolt != NULL) && ((uint8) RESP_VALID == pSvcCfg->pSvcStat->uRespStatus))
    {
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
        for(uDevIdx = uLoop; uDevIdx < pBmicMgr->uNAFEs; uDevIdx++)
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pBmicMgr->uNAFEs, uDevIdx);

            pOutVcellVolt[((uDevIdx-pBmicMgr->uSDevId))] = \
                                   (((uint16) pData[0] << 8) | pData[1u]);

        }
        pSvcCfg->pSvcStat->uInfo = (uint8) RESP_DECODED;
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

        uRet = (uint8) BMI_OK;
    }

    return uRet;
}

/**********************************************************************************************************************
 * uint8 Bq7971x_DecodeVRefCap(Bq7971x_ManagerType *pBmcCtx, const ServiceCfgType *pSvcCfg, uint16 *pOutVcellVolt)
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

FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeVRefCap(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg,
                                              uint16 *pOutVcellVolt)
{
    uint8 *pData;
    uint8 uRet = BMI_NOT_OK;
    uint8 uDevIdx;
    uint8 uLoop = pBmicMgr->uSDevId;


    if((pOutVcellVolt != NULL) && ((uint8) RESP_VALID == pSvcCfg->pSvcStat->uRespStatus))
    {
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
        for(uDevIdx = uLoop; uDevIdx < pBmicMgr->uNAFEs; uDevIdx++)
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pBmicMgr->uNAFEs, uDevIdx);

            pOutVcellVolt[((uDevIdx-pBmicMgr->uSDevId))] = \
                                   ((uint16) pData[0] << 8 | pData[1u]);

        }
        pSvcCfg->pSvcStat->uInfo = (uint8) RESP_DECODED;
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

        uRet =(uint8) BMI_OK;
    }

    return uRet;
}
/**********************************************************************************************************************
 * uint8 Bq7971x_DecodeCellBal(Bq7971x_ManagerType *pBmcCtx, const ServiceCfgType *pSvcCfg, uint16 *pOutVcellVolt)
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

FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeCellBal(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg)
{
    uint8 *pData;
    uint8 *pOutData;
    uint8 uRet = (uint8)BMI_NOT_OK;
    uint8 uDevIdx;
    uint8 uDataIdx;
    uint8 uNDataSiz = 0u;
    uint8 readIdx = 0u, i;
    uint8 uLoop = pBmicMgr->uSDevId;

    if((uint8)RESP_VALID == pSvcCfg->pSvcStat->uRespStatus)
    {
        switch(pSvcCfg->uSubId)
        {
            case (uint8)BMI_CB_GET_BALTIME:
            {
                 if ( CB_BalTimeChannelNum > 0u )
		         {
                    for ( i = 1U; i <= BQ7971X_CELL_NUM_ACT; i++ )
                    {
                        if( CheckIndexForCBTimeRead(CB_BalTimeChannelNum, i) != 0u )
                        {
                            readIdx = i;
			                ClearIndexForCBTimeRead(CB_BalTimeChannelNum, i);
			                break;
                        }
                     }
		          }
                  break;
            }
            case (uint8)BMI_CB_GET_BALSTAT:
            {
                uNDataSiz = 1u;
                break;
            }
            case (uint8)BMI_CB_GET_BALDONESTAT:
            case (uint8)BMI_CB_GET_BALSWSTAT:
            {
                uNDataSiz = 3u;
                break;
            }
            default:
            {
                break;
            }
        }

        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
        pOutData = (uint8 *) pSvcCfg->pDecodeBuf0;
        if((NULL != pOutData) && (uNDataSiz > 0u))
        {
            for(uDevIdx = uLoop; uDevIdx < pBmicMgr->uNAFEs; uDevIdx++)
            {
                pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pBmicMgr->uNAFEs, uDevIdx);
                if(pData != NULL)
                {
                    for(uDataIdx = 0; uDataIdx < uNDataSiz; uDataIdx++)
                    {
                       if ( pSvcCfg->uSubId != (uint8) BMI_CB_GET_BALTIME )
		               {
                           pOutData[ (pSvcCfg->uDecodeLen * uDevIdx) + uDataIdx] = pData[uDataIdx];
		               }
		               else
                       {
			               pOutData[(pSvcCfg->uDecodeLen * uDevIdx) + readIdx - 1U] = pData[uDataIdx];
			           }
			   
                    }
                }
            }
        }

            uRet = (uint8) BMI_OK;
            pSvcCfg->pSvcStat->uInfo = (uint8) RESP_DECODED;
    }
    pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

    return uRet;
}

/**********************************************************************************************************************
 * Function name: Bq7971x_ReceiveProc
 *********************************************************************************************************************/
/** \brief This function is used to Recieve processing
 *
 *  \param[in]      pBmcCtx: Manager
 *  \param[in]      pSvcCfg: Type of the Notify
 *  \param[in]      pRxData: Recieved data
 *  \param[in]      uResp: Response status
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

FUNC(uint8, bq7971x_CODE) Bq7971x_ReceiveProc(void *pBmcCtx, const ServiceCfgType *pSvcCfg, const uint8 *pRxData,
                                              uint8 uRespStatus)
{
    Bq7971x_ManagerType *pBmicMgr = (Bq7971x_ManagerType *)  pBmcCtx;
    const uint8 *pData;
    uint32 qRespCurTimeMs = 0;
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;
    uint8 uDevIdx = 0;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        uRet = (uint8)BMI_BMC_INVALID_RX_PROC;
        if(pSvcCfg != NULL)
        {
            pSvcCfg->pSvcStat->uRespStatus = uRespStatus;
            switch(pSvcCfg->uServiceId)
            {
                case (uint8)BMI_SERVICE_CELL_VOLT:
                {
                    uRet = Bq7971x_DecodeVcell(pBmicMgr, pSvcCfg, (uint16 *) pSvcCfg->pDecodeBuf0);
                    break;
                }
                case (uint8)BMI_SERVICE_VBAT:
                {
                    uRet = Bq7971x_DecodeVBat(pBmicMgr, pSvcCfg, (uint16 *) pSvcCfg->pDecodeBuf0);
                    break;
                }
                case (uint8)BMI_SERVICE_GET_VCELL_ACTSUM:
                {
                    uRet = Bq7971x_DecodeVcellActSum(pBmicMgr, pSvcCfg, (uint16 *) pSvcCfg->pDecodeBuf0);
                    break;
                }
                case (uint8)BMI_SERVICE_GET_DIE_TEMP:
                {
                    uRet = Bq7971x_DecodeDieTemp(pBmicMgr, pSvcCfg, (uint32 *) pSvcCfg->pDecodeBuf0);
                    break;
                }
                case (uint8)BMI_SERVICE_GET_DIAG_MAIN:
                {
                    uRet = Bq7971x_DecodeDiagMain(pBmicMgr, pSvcCfg, (uint32 *) pSvcCfg->pDecodeBuf0);
                    break;
                }
                case (uint8)BMI_SERVICE_GET_DIAG_RDNT:
                {
                    uRet = Bq7971x_DecodeDiagRdnt(pBmicMgr, pSvcCfg, (uint32 *) pSvcCfg->pDecodeBuf0);
                    break;
                }
                case (uint8)BMI_SERVICE_GET_I2C_DATA:
                {
                    uRet = Bq7971x_DecodeI2CData(pBmicMgr, pSvcCfg, (uint16 *) pSvcCfg->pDecodeBuf0);
                    break;
                }
                case (uint8)BMI_SERVICE_GET_I2C_FAULT:
                {
                    uRet = Bq7971x_DecodeI2CFault(pBmicMgr, pSvcCfg, (uint16 *) pSvcCfg->pDecodeBuf0);
                    break;
                }
                case (uint8)BMI_SERVICE_VCAP_REF:
                {
                    uRet = Bq7971x_DecodeVRefCap(pBmicMgr, pSvcCfg, (uint16 *) pSvcCfg->pDecodeBuf0);
                    break;
                }
                case (uint8)BMI_SERVICE_BALANCING:
                {
                    uRet = Bq7971x_DecodeCellBal(pBmcCtx, pSvcCfg);
                    break;
                }
                case (uint8)BMI_SERVICE_GPIO_READ:
                {
                    uRet = Bq7971x_DecodeGpio(pBmcCtx, pSvcCfg, (uint16 *) pSvcCfg->pDecodeBuf0);
                    break;
                }
                case (uint8)BMI_SERVICE_DIAGNOSTICS:
                {
                    uRet = Bq7971x_DiagDataProc(pBmicMgr->pBmicDiag, pSvcCfg);
                    break;
                }
				case (uint8)BMI_SERVICE_COMMUNICATION:
                {
                    uRet = Comiface_DataProc(pBmicMgr->pComifCtx, pSvcCfg, pRxData);
                    if((uint8)BQ7971X_STATE_INIT == pBmicMgr->eDrvState)
                    {
                        if(((uint8)COMIF_WIRELESS == pBmicMgr->eComType) && ((uint8)COMM_GETNUMOFCONN == pSvcCfg->uSubId))
                        {
                            if(((uint8)RESP_VALID == pSvcCfg->pSvcStat->uRespStatus) && (NULL != pRxData))
                            {
                                if(pRxData[pSvcCfg->uDataOffset] == pBmicMgr->nCurrNoOfDevs)
                                {
                                    uRet = (uint8)BMI_INIT_PROGRESS;
                                    if(pBmicMgr->nCurrNoOfDevs == pBmicMgr->uNDevices)
                                    {
                                        uRet = Bq7971x_ConfigInit(pBmicMgr);
                                    }
                                }
                            }
                        }
                    }
                    break;
                }
                case (uint8)BMI_SERVICE_CONFIG:
                {
                    uRet = (uint8)BMI_BMC_INVALID_STATE;
					if((uint8)BQ7971X_STATE_INIT == pBmicMgr->eDrvState)
					{
                        uRet = (uint8)BMI_BMC_INVALID_CFG_DATA;
                        if((NULL != pRxData) && ((uint8)RESP_VALID == pSvcCfg->pSvcStat->uRespStatus))
                        {
                            for(uDevIdx = 0; uDevIdx < pBmicMgr->uNDevices; uDevIdx++)
                            {
                                pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pBmicMgr->uNDevices, uDevIdx);
                                if(pData[0u] == pBmicMgr->nCurrNoOfDevs)
                                {
                                    pBmicMgr->nCurrNoOfDevs++;
                                    uRet = (uint8)BMI_INIT_PROGRESS;
                                }
                            }

                            if(pBmicMgr->nCurrNoOfDevs == pBmicMgr->uNDevices)
                            {
                                uRet = Bq7971x_ConfigInit(pBmicMgr);
#if (PMIF_BQ7973X_ENABLE == STD_ON)
                                if(pBmicMgr->uIsPmicInIf != 0u)
                                {
                                    uRet = Pmi_ConfigInit(pBmicMgr->uBmicIface, 0);
                                    if((uint8)BMI_OK != uRet)
                                    {
                                        uRet = (uint8)BMI_PMC_CFGINIT_FAILED;
                                        break;
                                    }
                                }
#endif
                            }
                        }
                    }
                    break;
                }
		case (uint8)BMI_SERVICE_SET_GPIO_CONF:
                {
                    uRet = (uint8)BMI_BMC_INVALID_SVC_CMD;
                    break;
		}
                default:
                {
                    uRet = (uint8)BMI_BMC_INVALID_SVC_CMD;
                    break;
                }
            }

            if(pSvcCfg->uServiceId < (uint8)BMI_SERVICE_DIAGNOSTICS)
            {
                pSvcCfg->pSvcStat->nRespMsgs++;
                pBmicMgr->pBswCfg->timestamp_req(&pSvcCfg->pSvcStat->cReqTimeMs, (uint32 *)&qRespCurTimeMs);
                if(pSvcCfg->pSvcStat->cRespMaxTimeMs < qRespCurTimeMs)
                {
                    pSvcCfg->pSvcStat->cRespMaxTimeMs = (uint8)(qRespCurTimeMs & 0xFFu);
                }
            }
        }

        uRet = Bmi_SetErrorDetails(BMI_SVCID_BMC_RX_DATA_PROC, uRet, pBmicMgr->uBmicIface, pSvcCfg->uServiceId);
    }

    return uRet;
}

/**********************************************************************************************************************
 * Function name: Bq7971x_DataDecode
 *********************************************************************************************************************/
/** \brief This function is used to Decode the recieved data
 *
 *  \param[in]      pBmcCtx: Manager
 *  \param[in]      uServiceId: Service ID
 *  \param[out]     pBuffer: Output Buffer
 *  \param[in]      wLength: Length of the buffer
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

FUNC(uint8, bq7971x_CODE) Bq7971x_DataDecode(void *pBmcCtx, uint8 uServiceId, uint8 uSubId, void *pBuffer, uint16 wLength)
{
    Bq7971x_ManagerType *pBmicMgr = (Bq7971x_ManagerType *)  pBmcCtx;
    const ServiceCfgType *pFuncSvcCfg;
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        uRet = (uint8)BMI_BMC_INVALID_RX_BUFFER;
        switch(uServiceId)
        {
            case (uint8)BMI_SERVICE_CELL_VOLT:
            {
                if(pBuffer != NULL)
                {
                    uRet = (uint8)BMI_BMC_DECODE_VCELL_PREV_DATA;
                    pFuncSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_CELL_VOLT];
                    if((NULL != pFuncSvcCfg) && ((uint8)RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        if((wLength >= (uint16)(pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics)) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            pFuncSvcCfg->pSvcStat->uInfo = (uint8)RESP_USED;
                            (void)memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, (pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics));
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

                            uRet = (uint8)BMI_OK;
                        }
                    }
                }
                break;
            }
            case (uint8)BMI_SERVICE_VBAT:
            {
                if(pBuffer != NULL)
                {
                    uRet = (uint8) BMI_BMC_DECODE_VBAT_PREV_DATA;
                    pFuncSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_VBAT];
                    if((pFuncSvcCfg != NULL) && ((uint8) RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        if((wLength >= (uint16)(pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics)) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            pFuncSvcCfg->pSvcStat->uInfo = (uint8) RESP_USED;
                            (void) memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, (pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics));
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

                            uRet = (uint8) BMI_OK;
                        }
                    }
                }
                break;
            }
            case (uint8)BMI_SERVICE_GET_VCELL_ACTSUM:
            {
                if(pBuffer != NULL)
                {
                    uRet = BMI_BMC_DECODE_VCELLACTSUM_PREV_DATA;
                    pFuncSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_GET_VCELL_ACTSUM];
                    if((pFuncSvcCfg != NULL) && ((uint8) RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        if((wLength >= (uint16)(pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics)) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            pFuncSvcCfg->pSvcStat->uInfo = (uint8) RESP_USED;
                            (void) memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, (pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics));
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

                            uRet = (uint8) BMI_OK;
                        }
                    }
                }
                break;
            }
            case (uint8)BMI_SERVICE_GET_DIE_TEMP:
            {
                if(pBuffer != NULL)
                {
                    uRet = (uint8) BMI_BMC_DECODE_DIETEMP_PREV_DATA;
                    pFuncSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_GET_DIE_TEMP];
                    if((pFuncSvcCfg != NULL) && ((uint8) RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        if((wLength >= (uint16)(pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics)) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            pFuncSvcCfg->pSvcStat->uInfo = (uint8) RESP_USED;
                            (void) memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, (pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics));
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

                            uRet = (uint8) BMI_OK;
                        }
                    }
                }
                break;
            }
            case (uint8)BMI_SERVICE_GET_DIAG_MAIN:
            {
                if(pBuffer != NULL)
                {
                    uRet = BMI_BMC_DECODE_DIAGMAIN_PREV_DATA;
                    pFuncSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_GET_DIAG_MAIN];
                    if((pFuncSvcCfg != NULL) && ((uint8) RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        if((wLength >= (uint16)(pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics)) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            pFuncSvcCfg->pSvcStat->uInfo = (uint8) RESP_USED;
                            (void) memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, (pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics));
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

                            uRet = (uint8) BMI_OK;
                        }
                    }
                }
                break;
            }
            case (uint8)BMI_SERVICE_GET_DIAG_RDNT:
            {
                if(pBuffer != NULL)
                {
                    uRet = BMI_BMC_DECODE_DIAGRDNT_PREV_DATA;
                    pFuncSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_GET_DIAG_RDNT];
                    if((pFuncSvcCfg != NULL) && ((uint8) RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        if((wLength >= (uint16)(pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics)) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            pFuncSvcCfg->pSvcStat->uInfo = (uint8) RESP_USED;
                            (void) memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, (pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics));
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

                            uRet = (uint8) BMI_OK;
                        }
                    }
                }
                break;
            }
            case (uint8)BMI_SERVICE_GET_I2C_DATA:
            {
                if(pBuffer != NULL)
                {
                    uRet = BMI_BMC_DECODE_I2CDATA_PREV_DATA;
                    pFuncSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_GET_I2C_DATA];
                    if((pFuncSvcCfg != NULL) && ((uint8) RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        if((wLength >= (uint16)(pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics)) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            pFuncSvcCfg->pSvcStat->uInfo = (uint8) RESP_USED;
                            (void) memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, (pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics));
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

                            uRet = (uint8) BMI_OK;
                        }
                    }
                }
                break;
            }
            case (uint8)BMI_SERVICE_GET_I2C_FAULT:
            {
                if(pBuffer != NULL)
                {
                    uRet = (uint8) BMI_BMC_DECODE_I2CFAULT_PREV_DATA;
                    pFuncSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_GET_I2C_FAULT];
                    if((pFuncSvcCfg != NULL) && ((uint8) RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        if((wLength >= (uint16)(pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics)) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            pFuncSvcCfg->pSvcStat->uInfo = (uint8) RESP_USED;
                            (void) memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, (pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics));
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

                            uRet = (uint8) BMI_OK;
                        }
                    }
                }
                break;
            }
            case (uint8)BMI_SERVICE_VCAP_REF:
            {
                if(pBuffer != NULL)
                {
                    uRet = BMI_BMC_DECODE_VREFCAP_PREV_DATA;
                    pFuncSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_VCAP_REF];
                    if((pFuncSvcCfg != NULL) && ((uint8) RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        if((wLength >= (uint16)(pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics)) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            pFuncSvcCfg->pSvcStat->uInfo = (uint8) RESP_USED;
                            (void) memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, (pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics));
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

                            uRet = (uint8) BMI_OK;
                        }
                    }
                }
                break;
            }
            case (uint8)BMI_SERVICE_GPIO_READ:
            {
                if(pBuffer != NULL)
                {
                    uRet = (uint8) BMI_BMC_DECODE_GPIO_PREV_DATA;
                    pFuncSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_GPIO_READ];
                    if((pFuncSvcCfg != NULL) && ((uint8)RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        if((wLength >= (uint16)(pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics)) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            pFuncSvcCfg->pSvcStat->uInfo = (uint8)RESP_USED;
                            (void)memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, (pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics));
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

                            uRet = (uint8)BMI_OK;
                        }
                    }
                }
                break;
            }
            case (uint8)BMI_SERVICE_BALANCING:
            {
                if(pBuffer != NULL)
                {
                    uRet = (uint8) BMI_BMC_DECODE_CELL_BALLANCING_PREV_DATA;
                    pFuncSvcCfg = &pBmicMgr->pCellBalCfg[uSubId];
                    if((pFuncSvcCfg != NULL) && ((uint8) RESP_DECODED == pFuncSvcCfg->pSvcStat->uInfo))
                    {
                        if((wLength >= (uint16)(pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics)) && (NULL != pFuncSvcCfg->pDecodeBuf0))
                        {
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
                            pFuncSvcCfg->pSvcStat->uInfo = (uint8) RESP_USED;
                            (void) memcpy(pBuffer, pFuncSvcCfg->pDecodeBuf0, (pFuncSvcCfg->uDecodeLen *pBmicMgr->uNBmics));
                            pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

                            uRet = (uint8) BMI_OK;
                        }
                    }
                }
                break;
            }
            default:
            {
                uRet = (uint8)BMI_BMC_INVALID_SVC_CMD;
                break;
            }
        }
        uRet = Bmi_SetErrorDetails(BMI_SVCID_BMC_DATA_DECODE, uRet, pBmicMgr->uBmicIface, uServiceId);
    }

    return uRet;
}

/**********************************************************************************************************************
 *  void* Bq7971x_Init(const InterfaceCfgType *pIfaceCfg, Comif_ManagerType *pComifCtx, uint8 uWupReq)
 *********************************************************************************************************************/
/*! \brief          This function is used to init the Bq7971x core for corresponding chains in the config set
 *
 *  \param[in]      pIfaceCfg - Configurations for the Interface
 *  \param[in]      pComifCtx - Communication Interface context
 *  \param[in]      uNfaultCfg - Fault pin config
 *  \param[in]      uStartupCmdCfg -Startup Command config
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

FUNC(void*, bq7971x_CODE) Bq7971x_Init(const InterfaceCfgType *pIfaceCfg, Comif_ManagerType *pComifCtx,uint8 uNfaultCfg, uint8 uStartUp,uint8 uWupReq)
{
    Bq7971x_ManagerType *pBmicMgr;
    Bq7971x_ManagerType *pBq7971xCtxRet = NULL;
    const Bmi_ConfigType *pBmiCfg;
    uint8 uRet;

    do
    {
        if(NULL == pIfaceCfg)
        {
            uRet =(uint8)BMI_BMC_ILLEGALCTX;
            break;
        }

        pBmiCfg = pIfaceCfg->pBmiCfg;
        if((NULL == pBmiCfg) ||
           (NULL == pBmiCfg->pBmicCfg) ||
           (NULL == pBmiCfg->pFuncCfg))
        {
            uRet = (uint8)BMI_BMC_INVALID_CFG;
            break;
        }

        if(NULL == pComifCtx)
        {
            uRet = (uint8)BMI_BMC_INVALID_COMIFMGR;
            break;
        }

        if(pIfaceCfg->uIfaceCfg >= BQ7971X_IFACES)
        {
            uRet = (uint8)BMI_BMC_INVALID_IFACECFG;
            break;
        }

        pBmicMgr = &zBq7971xMgr[pIfaceCfg->uIfaceCfg];
        if(NULL == pBmicMgr)
        {
            uRet = (uint8)BMI_BMC_INVALID_MGR;
            break;
        }

        if(BQ7971X_INIT == pBmicMgr->uInit)
        {
            uRet = (uint8)BMI_BMC_ALREADY_INUSE;
            break;
        }

        (void)memset(pBmicMgr, 0, sizeof(Bq7971x_ManagerType));

        pBmicMgr->eDrvState = (uint8)BQ7971X_STATE_UINIT;
        pBmicMgr->uBmicIface = pIfaceCfg->uIfaceCfg;
        pBmicMgr->eComType = pIfaceCfg->eComTypeCfg;
        pBmicMgr->uNfaultPin = uNfaultCfg;

        pBmicMgr->pBmicCfg = (Bq7971x_ConfigType *) pBmiCfg->pBmicCfg;
        pBmicMgr->pFuncCfg = pBmiCfg->pFuncCfg;
        pBmicMgr->pCellBalCfg = pBmiCfg->pCellBalCfg;

        pBmicMgr->eCommDir = pIfaceCfg->eReqComDirCfg;
        pBmicMgr->nCurrNoOfDevs = 0u;
        pBmicMgr->uNAFEs = (pIfaceCfg->uNBmicsCfg + pIfaceCfg->uNPmicsCfg);                 /** Total Number of AFE's in the system configuration **/
        pBmicMgr->uNBmics = pIfaceCfg->uNBmicsCfg;                                          /** Total Number of BMIC AFE's in the system configuration **/
        pBmicMgr->uNDevices = pIfaceCfg->uNDevsCfg;                                         /** Total Number of AFE's + Communication Device **/

        pBmicMgr->pComifCtx = pComifCtx;
        pBmicMgr->uIsPmicInIf = pIfaceCfg->uIsPmicInIfCfg;

        pBmicMgr->uSDevId = 0;
        if((0U != pBmicMgr->uIsPmicInIf) && ((uint8)PM_DEVID_BEGIN == pIfaceCfg->uPmicDevIdCfg))         /** Pack monitor device id with 1 or last of the chain, If it end of the chain, Start ID for BMIC is zero **/
        {
            pBmicMgr->uSDevId = 1u;                                                         /** DevID zero is for the  PMIC and 1 for BMIC **/
        }

        pBmicMgr->pCbCtrl = &zBq7971xCBCtrl[pBmicMgr->uBmicIface];
        if(NULL == pBmicMgr->pCbCtrl)
        {
            uRet = (uint8)BMI_BMC_INVALID_MGR;
            break;
        }
        (void)memset(pBmicMgr->pCbCtrl, 0, sizeof(Bq7971x_CellBalType));

        if(NULL == pBmiCfg->pBswCfg)
        {
            uRet = (uint8)BMI_BMC_INVALID_BSWCFG;
            break;
        }

        pBmicMgr->pBswCfg = pBmiCfg->pBswCfg;
        if(NULL == pBmicMgr->pBswCfg)
        {
            uRet = (uint8)BMI_BMC_INVALID_IFACECFG;
            break;
        }
        pBmicMgr->qApplRes = pBmicMgr->pBswCfg->qApplRes;
        pBmicMgr->pBmicDiag = Bq7971x_DiagInit(pBmicMgr, pBmiCfg, pComifCtx, pBmicMgr->uBmicIface,
                                             pBmicMgr->uNAFEs, uNfaultCfg, pBmicMgr->eComType,
                                             pBmicMgr->uNBmics, pBmicMgr->uSDevId);
        if(NULL == pBmicMgr->pBmicDiag)
        {
            uRet = (uint8)BMI_BMC_DIAG_INIT_FAILED;
            break;
        }

        pBmicMgr->eDrvState = (uint8)BQ7971X_STATE_WAKEUP;
        pBmicMgr->qGuardStart = BQ7971X_GUARD_START;
        pBmicMgr->qGuardEnd = BQ7971X_GUARD_END;

        pBmicMgr->uInit = BQ7971X_INIT;
        pBq7971xCtxRet = pBmicMgr;
        uRet = (uint8)BMI_OK;
    }
    while(FALSE);

    (void)Bmi_SetErrorDetails(BMI_SVCID_BMC_INIT, uRet, 0u, 0u);

    return pBq7971xCtxRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) Bq7971x_DeInit(void *pBmcCtx)
 *********************************************************************************************************************/
/*! \brief          This function is used to De-init the Bq7971x core for corresponding iface in the config set
 *
 *  \param[in]      pComifCtx - Communication Interface context
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            Already Initialized Core
 *  \post
 *  \return         uint8
 *  \retval         BMI_OK if success
 *                  BMI_NOT_OK if failed
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq7971x_CODE) Bq7971x_DeInit(void *pBmcCtx)
{
    Bq7971x_ManagerType *pBmicMgr = (Bq7971x_ManagerType *)  pBmcCtx;
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        (void)Bq7971x_DiagDeInit(pBmicMgr->uBmicIface);
        (void)memset(pBmicMgr, 0, sizeof(Bq7971x_ManagerType));
	}
        uRet = (uint8)BMI_OK;
		return uRet;
}
/**********************************************************************************************************************
 *  STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_GetVRefCap(Bq7971x_ManagerType *pBmicMgr,  ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function Initiates the read for temperature values
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Data
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_GetVRefCap(Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        uRet = Comif_StackRead(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs, COMIF_PRIO_GPIO(BQ7971X_REF_CAP_HI_OFFSET),
                               pSvcCfg, (READ_2BYTE));
    }

    return uRet;
}


/**********************************************************************************************************************
 * STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_GetVBat(Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          This function is used to read the BAT voltage.
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Data
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

 STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_GetVBat(Bq7971x_ManagerType *pBmicMgr,const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        uRet = Comif_StackRead(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs, COMIF_PRIO_GPIO(BQ7971X_BAT_HI_OFFSET),
                               pSvcCfg, (READ_2BYTE));
    }
    return uRet;
}


/**********************************************************************************************************************
 *  STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_GetVcellActSum(Bq7971x_ManagerType *pBmicMgr,  ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function Initiates the read for temperature values
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Data
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_GetVcellActSum(Bq7971x_ManagerType *pBmicMgr,const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = (uint8) BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        uRet = Comif_StackRead(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs, COMIF_PRIO_GPIO(BQ7971X_VCELL_ACT_SUM_HI_OFFSET),
                               pSvcCfg, (READ_2BYTE));
    }

    return uRet;
}


/**********************************************************************************************************************
 *  STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_GetDieTemp(Bq7971x_ManagerType *pBmicMgr,  ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function Initiates the read for temperature values
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Data
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_GetDieTemp(Bq7971x_ManagerType *pBmicMgr,const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        uRet = Comif_StackRead(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs, COMIF_PRIO_GPIO(BQ7971X_DIETEMP1_HI_OFFSET),
                               pSvcCfg, (READ_4BYTE));
    }

    return uRet;
}




/**********************************************************************************************************************
 *  FUNC(uint8, bq7971x_CODE) BQ7971X_GetDiagRdnt(Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          This function is used to read the DIAG_RDNT.
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Data
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_GetDiagRdnt(Bq7971x_ManagerType *pBmicMgr,const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        uRet = Comif_StackRead(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs, COMIF_PRIO_GPIO(BQ7971X_DIAG_RDNT_HI_OFFSET),
                               pSvcCfg, (READ_3BYTE));
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7971x_CODE) BQ7971X_GetDiagMain(Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          This function is used to read the DIAG_MAIN.
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Data
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_GetDiagMain(Bq7971x_ManagerType *pBmicMgr,const ServiceCfgType *pSvcCfg)
{
    uint8 uRet = (uint8)BMI_BMC_INTEGRITY_FAILED;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        uRet = Comif_StackRead(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs, COMIF_PRIO_GPIO(BQ7971X_DIAG_MAIN_HI_OFFSET),
                               pSvcCfg, (READ_3BYTE));
    }

    return uRet;
}

/*********************************************************************************************************************
 * Function name: STATIC FUNC(uint8, bq7971x_CODE)BQ7971X_DevPowerManager(Bq7971x_ManagerType *pBmicMgr, uint8 uState)
*********************************************************************************************************************
*!\brief This function is used to perform 'shutdown' of bridge and stack devices.
*
*  \param[in]      pBmicMgr - Diagnostic context
*  \param[in]      uState: target state.
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

STATIC FUNC(uint8, bq7971x_CODE) BQ7971X_DevPowerManager(Bq7971x_ManagerType *pBmicMgr, uint8 uState)
{
    uint8 uRet = (uint8)BMI_OK;
    uint8 uCmd;
    uint8 uDevIdx;
    const ServiceCfgType *pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_CONFIG];
    switch (uState)
    {
        case (uint8)BMI_SET_DEV_SLEEP:
        {
            uCmd = (pBmicMgr->eCommDir | BQ7971X_CONTROL1_GOTO_SLEEP_MSK);
            uRet |= Comif_SingleWrite(pBmicMgr->pComifCtx, 0, COMIF_PRIO_EXCL_CELL(BQ7971X_CONTROL1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
            uRet |= Comif_SetDelay(pBmicMgr->pComifCtx, BQ7971X_GOTO_SLEEP_DELAY);
            break;
        }
        case (uint8)BMI_SET_SOFT_RESET:
        {
            uCmd = (pBmicMgr->eCommDir | BQ7971X_CONTROL1_SOFT_RESET_MSK);
            uRet |= Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) FALSE, COMIF_PRIO_EXCL_CELL(BQ7971X_CONTROL1_OFFSET, COMIF_LOCK),
                                        &uCmd, WRITE_1BYTE);
            uRet |= Comif_SetDelay(pBmicMgr->pComifCtx, BQ7971X_SOFT_RESET_DELAY);
            uRet |= Comif_SingleWrite(pBmicMgr->pComifCtx, 0, COMIF_PRIO_EXCL_CELL(BQ7971X_CONTROL1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
            uRet |= Comif_SetDelay(pBmicMgr->pComifCtx, BQ7971X_SOFT_RESET_DELAY);
            break;
        }
        case (uint8)BMI_SET_HW_RESET:
        {
            /* perform hard reset of the stack device. */
            uCmd = (pBmicMgr->eCommDir  | BQ7971X_CONTROL1_SEND_SD_HW_RST_MSK);
            for (uDevIdx = 0u; uDevIdx < pBmicMgr->uNAFEs; uDevIdx ++)
            {
                uRet |= Comif_SingleWrite(pBmicMgr->pComifCtx, (uDevIdx+1u), COMIF_PRIO_EXCL_CELL(BQ7971X_CONTROL1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
                uRet |= Comif_SetDelay(pBmicMgr->pComifCtx, BQ7971X_SOFT_RESET_DELAY);
            }
            uRet |= Comif_SingleWrite(pBmicMgr->pComifCtx, 0u, COMIF_PRIO_EXCL_CELL(BQ7971X_CONTROL1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
            uRet |= Comif_SetDelay(pBmicMgr->pComifCtx, BQ7971X_SOFT_RESET_DELAY);
            /* perform hard reset of the bridge device. */
            uCmd = (pBmicMgr->eCommDir  | BQ7971X_CONTROL1_GOTO_SHUTDOWN_MSK);
            uRet |= Comif_SingleWrite(pBmicMgr->pComifCtx, 0u, COMIF_PRIO_EXCL_CELL(BQ7971X_CONTROL1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
            uRet |= Comif_SetDelay(pBmicMgr->pComifCtx, BQ7971X_HW_RESET_DELAY);
            break;
        }
        case (uint8)BMI_GOTO_SHUTDOWN:
        {
            uCmd = (pBmicMgr->eCommDir | BQ7971X_CONTROL1_GOTO_SHUTDOWN_MSK);
            uRet |= Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) FALSE, COMIF_PRIO_EXCL_CELL(BQ7971X_CONTROL1_OFFSET, COMIF_LOCK),
                                        &uCmd, WRITE_1BYTE);
            uRet |= Comif_SingleWrite(pBmicMgr->pComifCtx, 0u, COMIF_PRIO_EXCL_CELL(BQ7971X_CONTROL1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
            uRet |= Comif_SetDelay(pBmicMgr->pComifCtx, BQ7971X_GOTO_SHUTDOWN_DELAY);
            break;
        }
        case (uint8)BMI_SET_DEV_SLP2WAKEUP:
        {

            uCmd = (pBmicMgr->eCommDir | BQ7971X_CONTROL1_SEND_SLPTOACT_MSK);
            uRet |= Comif_SingleWrite(pBmicMgr->pComifCtx, 0, COMIF_PRIO_EXCL_CELL(BQ7971X_CONTROL1_OFFSET, COMIF_UNLOCK), &uCmd, WRITE_1BYTE);
            uRet |= Comif_SetDelay(pBmicMgr->pComifCtx, (BQ7971X_SLP2ACT_TONE_DELAY* (uint32)pBmicMgr->uNAFEs));
            pBmicMgr->eDrvState = (uint8) BQ7971X_STATE_WAKEUP;

            pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_CONFIG];
            if((pSvcCfg != NULL) && (pSvcCfg->pSvcStat != NULL))
            {
                pSvcCfg->pSvcStat->uRespStatus = (uint8) RESP_BUSY;

                uRet = Bq7971x_StartupInit(pBmicMgr, pSvcCfg);

            }
            break;
        }
        case (uint8)BMI_SET_DEV_SHUT2WAKEUP:
        {
            pBmicMgr->eDrvState = (uint8) BQ7971X_STATE_WAKEUP;

            pSvcCfg = &pBmicMgr->pFuncCfg[BMI_SERVICE_CONFIG];
            if((pSvcCfg != NULL) && (pSvcCfg->pSvcStat != NULL))
            {
                pSvcCfg->pSvcStat->uRespStatus = (uint8) RESP_BUSY;

                    uRet = Bq7971x_StartupInit(pBmicMgr, pSvcCfg);

            }
            break;
        }
        default:
        {
            uRet = (uint8)BMI_NOT_OK;
            break;
        }
    }
    return uRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) BQ7971X_SetCommConf( Bq7971x_ManagerType *pBmicMgr, uint8 uDevId)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      uDevId: Device Id.
 *  \param[in]
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

FUNC(uint8, bq7971x_CODE) BQ7971X_SetCommConf( Bq7971x_ManagerType *pBmicMgr , uint8 uDevId)
{
    uint8 uRet = (uint8) BMI_OK;
    if (uDevId > 0u)
    {
        uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, uDevId,BQ7971X_COMM_CONF_OFFSET,
                                 &pBmicMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uComm_Conf,WRITE_1BYTE);
    }
    else
    {
        uRet = E_NOT_OK;
    }
    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) BQ7971X_SetBBConf( Bq7971x_ManagerType *pBmicMgr, uint8 uDevId)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in] uDevId: Device Id.
 *  \param[in] uConfIndex: Configuration Index.
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

FUNC(uint8, bq7971x_CODE) BQ7971X_SetBBConf(Bq7971x_ManagerType *pBmicMgr , uint8 uDevId)
{
    uint8 uRet = (uint8) BMI_OK;
    if (uDevId > 0u)
    {
        uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, uDevId,BQ7971X_BBVC_POSN1_OFFSET,
                                 &pBmicMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uBBVC_Posn[0],WRITE_3BYTE);

         uRet |= Comif_SingleWrite(pBmicMgr->pComifCtx, uDevId,BQ7971X_UV_DISABLE1_OFFSET,
                                 &pBmicMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uBBVC_Posn[0],WRITE_3BYTE);
    }
    else
    {
        uRet = E_NOT_OK;
    }
    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) BQ7971X_SetEnableSPI(const Bq7971x_ManagerType *pBmicMgr , uint8 uDevId)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      uDevId: Device Id.
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

FUNC(uint8, bq7971x_CODE) BQ7971X_SetEnableSPI(const Bq7971x_ManagerType *pBmicMgr , uint8 uDevId)
{
    uint8 uRet = (uint8) BMI_OK;
    uint8 uCmd ;
    if (uDevId > 0u)
    {
        uCmd = pBmicMgr->pBmicCfg->pBqRegsCfg->zRegNVM.uGpio_Conf[0] | BQ7971X_GPIO_CONF1_SPI_EN_MSK;
        uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, uDevId,BQ7971X_GPIO_CONF1_OFFSET,
                                 &uCmd,WRITE_1BYTE);
    }
    else
    {
        uRet = E_NOT_OK;
    }
    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) BQ7971X_SetSPIConf(const Bq7971x_ManagerType *pBmicMgr , uint8 uDevId)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      uDevId: Device Id.
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

FUNC(uint8, bq7971x_CODE) BQ7971X_SetSPIConf(const Bq7971x_ManagerType *pBmicMgr , uint8 uDevId)
{

    uint8 uRet = (uint8) BMI_OK;
    uint8 uCmd ;
    if (uDevId > 0u)
    {
        uCmd = pBmicMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uSpi_Conf ;
        uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, uDevId,BQ7971X_SPI_CONF_OFFSET,
                                 &uCmd,WRITE_1BYTE);
    }
    else
    {
        uRet = E_NOT_OK;
    }
    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) BQ7971X_SetSPIExe(const Bq7971x_ManagerType *pBmicMgr , uint8 uDevId)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      uDevId: Device Id.
 *  \param[in] uSPIExe: SPI_EXE register value.
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

FUNC(uint8, bq7971x_CODE) BQ7971X_SetSPIExe(const Bq7971x_ManagerType *pBmicMgr , uint8 uDevId, uint8 uSPIExe)
{

    uint8 uRet = (uint8) BMI_OK;
    uint8 uCmd ;
    if (uDevId > 0u)
    {
        uCmd = uSPIExe & (BQ7971X_SPI_EXE_SS_CTRL_MSK | BQ7971X_SPI_EXE_SPI_GO_MSK);
        uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, uDevId,BQ7971X_SPI_EXE_OFFSET,
                                 &uCmd,WRITE_1BYTE);
    }
    else
    {
        uRet = E_NOT_OK;
    }
    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) BQ7971X_WriteSPIData(const Bq7971x_ManagerType *pBmicMgr , uint8 uDevId)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      uDevId: Device Id.
 *  \param[in]      uSPIData: Pointer to data to be used to write to SPI slave device.
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

FUNC(uint8, bq7971x_CODE) BQ7971X_WriteSPIData(const Bq7971x_ManagerType *pBmicMgr , uint8 uDevId, const uint8 *uSPIData)
{

    uint8 uRet = (uint8) BMI_OK;
    uint8 uCmd ;
    if (uDevId > 0u)
    {
        uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, uDevId,BQ7971X_SPI_TX3_OFFSET,
                                 uSPIData,WRITE_3BYTE);

        uCmd = BQ7971X_SPI_EXE_SPI_GO_MSK;
        uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, uDevId,BQ7971X_SPI_EXE_OFFSET,
                                 &uCmd,WRITE_1BYTE);
        uCmd = BQ7971X_SPI_EXE_SS_CTRL_MSK;
        uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, uDevId,BQ7971X_SPI_EXE_OFFSET,
                                 &uCmd,WRITE_1BYTE);
    }
    else
    {
        uRet = E_NOT_OK;
    }
    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) BQ7971X_ReadSPIData(const Bq7971x_ManagerType *pBmicMgr , uint8 uDevId)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Config
 *  \param[in]

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

FUNC(uint8, bq7971x_CODE) BQ7971X_ReadSPIData( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg)
{

    uint8 uRet = (uint8) BMI_OK;
    uint8 uDevId;

    for(uDevId = 1u; uDevId <= pBmicMgr->uNAFEs; uDevId ++)
    {
        uRet |= BQ7971X_SetSPIExe(pBmicMgr,uDevId , BQ7971X_SPI_EXE_SPI_GO_MSK);
    }
        uRet = Comif_StackRead(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs, COMIF_PRIO_GPIO(BQ7971X_SPI_RX3_OFFSET),
                               pSvcCfg, (READ_3BYTE));

    for(uDevId = 1u; uDevId <= pBmicMgr->uNAFEs; uDevId ++)
    {
        uRet |= BQ7971X_SetSPIExe(pBmicMgr,uDevId , BQ7971X_SPI_EXE_SS_CTRL_MSK);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, bq7971x_CODE) BQ7971X_SetEnableI2C(const Bq7971x_ManagerType *pBmicMgr , uint8 uDevId)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Config
 *  \param[in]

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

FUNC(uint8, bq7971x_CODE) BQ7971X_SetEnableI2C( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg , uint8 uDevId)
{

    uint8 uRet = (uint8) BMI_OK;
    uint8 uCmd ;
    if (uDevId > 0u)
    {
        uCmd = pBmicMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uControl[0] | BQ7971X_CONTROL2_I2C_EN_MSK;;
        uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, uDevId,BQ7971X_CONTROL2_OFFSET,
                                 &uCmd,WRITE_1BYTE);
        uRet |= Comif_SingleRead(pBmicMgr->pComifCtx, uDevId, COMIF_PRIO_GPIO(BQ7971X_CONTROL2_OFFSET),
                                        pSvcCfg, READ_1BYTE);
    }
     else
    {
        uRet = E_NOT_OK;
    }

    return uRet;
}

/**********************************************************************************************************************
FUNC(uint8, bq7971x_CODE) BQ7971X_SetI2CCtrl( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg , uint8 uDevId, uint8 uI2CCtrl)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Config
 *  \param[in]      uI2CCtrl: I2C_CTRL register value.
 *  \param[in]      uDevId: Device Id.
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

FUNC(uint8, bq7971x_CODE) BQ7971X_SetI2CCtrl( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg , uint8 uDevId, uint8 uI2CCtrl)
{

    uint8 uRet = (uint8) BMI_OK;
    uint8 uCmd ;

    uCmd = uI2CCtrl;
    uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, uDevId,BQ7971X_I2C_CTRL_OFFSET,
                                 &uCmd,WRITE_1BYTE);

    return uRet;
}

/**********************************************************************************************************************
FUNC(uint8, bq7971x_CODE) BQ7971X_SetI2CData( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg , uint8 uDevId, uint8 uI2CData)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Config
 *  \param[in]      uI2CCtrl: I2C_CTRL register value.
 *  \param[in]      uDevId: Device Id.
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

FUNC(uint8, bq7971x_CODE) BQ7971X_SetI2CData( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg , uint8 uDevId, uint8 uI2CData)
{

    uint8 uRet = (uint8) BMI_OK;
    uint8 uCmd ;

    uCmd = uI2CData & BQ7971X_I2C_WR_DATA_DATA_MSK;
    uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, uDevId,BQ7971X_I2C_WR_DATA_OFFSET,
                                 &uCmd,WRITE_1BYTE);

    return uRet;
}

/**********************************************************************************************************************
FUNC(uint8, bq7971x_CODE) BQ7971X_GetI2CData( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Config
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

FUNC(uint8, bq7971x_CODE) BQ7971X_GetI2CData( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg)
{

    uint8 uRet = (uint8) BMI_OK;

    uRet = Comif_StackRead(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs, COMIF_PRIO_GPIO(BQ7971X_I2C_RD_DATA_OFFSET),
                               pSvcCfg, (READ_1BYTE));

    return uRet;
}

/**********************************************************************************************************************
FUNC(uint8, bq7971x_CODE) BQ7971X_GetI2CFault( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Config
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

FUNC(uint8, bq7971x_CODE) BQ7971X_GetI2CFault( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg)
{

    uint8 uRet = (uint8) BMI_OK;
    
    uRet = Comif_StackRead(pBmicMgr->pComifCtx, pBmicMgr->uNAFEs, COMIF_PRIO_GPIO(BQ7971X_FAULT_SYS_OFFSET),
                               pSvcCfg, (READ_1BYTE));

    return uRet;
}

/**********************************************************************************************************************
FUNC(uint8, bq7971x_CODE) BQ7971X_GpioAdjSetGpioConf( Bq7971x_ManagerType *pBmicMgr)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Config
 *  \param[in]      uI2CCtrl: I2C_CTRL register value.
 *  \param[in]      uDevId: Device Id.
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

FUNC(uint8, bq7971x_CODE) BQ7971X_GpioAdjSetGpioConf( Bq7971x_ManagerType *pBmicMgr, Gpio_conf *g_conf,const ServiceCfgType *pSvcCfg)
{

    uint8 uRet = (uint8) BMI_OK;
    uint8 uDevId;
    uint8 gpio_conf_addr;

    /* Traverse all gpio */
    {
       if ( g_conf != NULL )
        {
           gpio_conf_addr = BQ7971X_GPIO_CONF1_OFFSET + g_conf->gpio_conf_addr;
            /* Write ODD GPIO configure drive output logic low. */
          if ((gpio_conf_addr >= BQ7971X_GPIO_CONF1_OFFSET) && (gpio_conf_addr <= BQ7971X_GPIO_CONF6_OFFSET))
            {
             uDevId = g_conf->devId;
             uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, uDevId, gpio_conf_addr, &g_conf->gpio_conf_val, WRITE_1BYTE);
            }
          else
               {
                  uRet = (uint8) BMI_NOT_OK;
               }
        }
       /*else
       {
        uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, g_conf->devId, (BQ7971X_GPIO_CONF1_OFFSET + uGpioIndex), &uCmd, WRITE_1BYTE);
        state = 0U;
       }*/
     }

    return uRet;
}


/**********************************************************************************************************************
FUNC(uint8, bq7971x_CODE) BQ7971X_GpioOpenSetGpioDown( Bq7971x_ManagerType *pBmicMgr)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      pSvcCfg: Service Config
 *  \param[in]      uI2CCtrl: I2C_CTRL register value.
 *  \param[in]      uDevId: Device Id.
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

FUNC(uint8, bq7971x_CODE) BQ7971X_GpioOpenSetGpioDown( Bq7971x_ManagerType *pBmicMgr)
{

    uint8 uRet = (uint8) BMI_OK;
    uint8 uCmd[BQ7971X_GPIO_CONF_REG_NUM] = {0};
    uint8   uGpioIndex;

    /* Traverse all gpio */
        for (uGpioIndex = 0u; uGpioIndex < BQ7971X_GPIO_NUM_ACT; uGpioIndex++)
        {
            uCmd[uGpioIndex / 2u] |= (BQ7971X_GPIOSETPULLDOWN << ((uGpioIndex % 2u) * BQ7971X_GPIOSETNUM));
        }

    uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8)TRUE,BQ7971X_GPIO_CONF1_OFFSET,
                                 uCmd,(uint8)BQ7971X_GPIO_CONF_REG_NUM);

    return uRet;
}


/**********************************************************************************************************************
FUNC(uint8, bq7971x_CODE) BQ7971X_EnADCFreeze( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg , uint8 uDevId, uint8 uI2CData)
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

FUNC(uint8, bq7971x_CODE) BQ7971X_EnADCFreeze( Bq7971x_ManagerType *pBmicMgr)
{

    uint8 uRet = (uint8) BMI_OK;
    uint8 uCmd ;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
	uCmd = ((pBmicMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uAdc_Ctrl[1u] | BQ7971X_ADC_CTRL2_FREEZE_EN_MSK) & 0xFEu);
        uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_ADC_CTRL2_OFFSET,
                               &uCmd, WRITE_1BYTE);
    }


    return uRet;
}


/**********************************************************************************************************************
FUNC(uint8, bq7971x_CODE) BQ7971X_DisADCFreeze( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg , uint8 uDevId, uint8 uI2CData)
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

FUNC(uint8, bq7971x_CODE) BQ7971X_DisADCFreeze( Bq7971x_ManagerType *pBmicMgr)
{

    uint8 uRet = (uint8) BMI_OK;
    uint8 uCmd ;

    if(BQ7971X_INTEGRITY_CHECK(pBmicMgr))
    {
        //initialize ADC_CTRLx
	uCmd = ((pBmicMgr->pBmicCfg->pBqRegsCfg->zRegCtrl.uAdc_Ctrl[1]  & (~BQ7971X_ADC_CTRL2_FREEZE_EN_MSK)) & 0xFE);
        uRet = Comif_StackWrite(pBmicMgr->pComifCtx, (uint8) TRUE, BQ7971X_ADC_CTRL2_OFFSET,
                               &uCmd, WRITE_1BYTE);
    }


    return uRet;
}

/**********************************************************************************************************************
FUNC(uint8, bq7971x_CODE) BQ7971X_SetADCDelay( Bq7971x_ManagerType *pBmicMgr  , const ServiceCfgType *pSvcCfg , uint8 uDevId, uint8 uI2CData)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      xTimeUs: Physical time(unit / us)
 *  \param[in]      uDevId: Device Id.
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

FUNC(uint8, bq7971x_CODE) BQ7971X_SetADCDelay( Bq7971x_ManagerType *pBmicMgr  ,  uint16 xTimeUs , uint8 uDevId)
{

    uint8 uRet = (uint8) BMI_OK;
    uint8 uCmd ;

        if(xTimeUs <= BQ7971X_MAX_ADC_DLY)
        {
            uCmd = (xTimeUs / BQ7971X_UNIT_ADC_DLY);
        }
        else
        {
            uCmd = BQ7971X_MAX_ADC_DLY / BQ7971X_UNIT_ADC_DLY;
        }

        uRet = Comif_SingleWrite(pBmicMgr->pComifCtx, uDevId ,BQ7971X_ADC_CONF_OFFSET, &uCmd,WRITE_1BYTE);


    return uRet;
}


/**********************************************************************************************************************
 * uint8 Bq7971x_DecodeVcellActSum(Bq7971x_ManagerType *pBmcCtx, const ServiceCfgType *pSvcCfg, uint16 *pOutVcellVolt)
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

FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeVcellActSum(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg,
                                              uint16 *pOutVcellActSum)
{
    uint8 *pData;
    uint8 uRet = (uint8) BMI_NOT_OK;
    uint8 uDevIdx;
    uint8 uLoop = pBmicMgr->uSDevId;


    if((pOutVcellActSum != NULL) && ((uint8) RESP_VALID == pSvcCfg->pSvcStat->uRespStatus))
    {
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
        for(uDevIdx = uLoop; uDevIdx < pBmicMgr->uNAFEs; uDevIdx++)
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pBmicMgr->uNAFEs, uDevIdx);

            pOutVcellActSum[((uDevIdx-pBmicMgr->uSDevId))] = \
                                   ((uint16) pData[0] << 8 | pData[1u]);

        }
        pSvcCfg->pSvcStat->uInfo = (uint8) RESP_DECODED;
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

        uRet = (uint8) BMI_OK;
    }

    return uRet;
}


/**********************************************************************************************************************
 * uint8 Bq7971x_DecodeDieTemp(Bq7971x_ManagerType *pBmcCtx, const ServiceCfgType *pSvcCfg, uint16 *pOutVcellVolt)
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

FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeDieTemp(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg,
                                              uint32 *pOutDieTemp)
{
    uint8 *pData;
    uint8 uRet = (uint8) BMI_NOT_OK;
    uint8 uDevIdx;
    uint8 uLoop = pBmicMgr->uSDevId;


    if((pOutDieTemp != NULL) && ((uint8) RESP_VALID == pSvcCfg->pSvcStat->uRespStatus))
    {
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
        for(uDevIdx = uLoop; uDevIdx < pBmicMgr->uNAFEs; uDevIdx++)
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pBmicMgr->uNAFEs, uDevIdx);

            pOutDieTemp[((uDevIdx-pBmicMgr->uSDevId))] = \
                             (((uint32) pData[0] << 24) | ((uint32) pData[1u]<< 16) | ((uint16) pData[2u] << 8) | (pData[3u]));


        }
        pSvcCfg->pSvcStat->uInfo = (uint8) RESP_DECODED;
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

        uRet = (uint8) BMI_OK;
    }

    return uRet;
}

/**********************************************************************************************************************
 * uint8 Bq7971x_DecodeDiagMain(Bq7971x_ManagerType *pBmcCtx, const ServiceCfgType *pSvcCfg, uint16 *pOutVcellVolt)
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

FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeDiagMain(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg,
                                              uint32 *pOutDiagMain)
{
    uint8 *pData;
    uint8 uRet = (uint8) BMI_NOT_OK;
    uint8 uDevIdx;
    uint8 uLoop = pBmicMgr->uSDevId;


    if((pOutDiagMain != NULL) && ((uint8) RESP_VALID == pSvcCfg->pSvcStat->uRespStatus))
    {
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
        for(uDevIdx = uLoop; uDevIdx < pBmicMgr->uNAFEs; uDevIdx++)
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pBmicMgr->uNAFEs, uDevIdx);

            pOutDiagMain[((uDevIdx-pBmicMgr->uSDevId))] = \
                     (((uint32) pData[0] << 16) | ((uint16) pData[1u] << 8) | (pData[2u]));


        }
        pSvcCfg->pSvcStat->uInfo = (uint8) RESP_DECODED;
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

        uRet = (uint8) BMI_OK;
    }

    return uRet;
}

/**********************************************************************************************************************
 * uint8 Bq7971x_DecodeDiagRdnt(Bq7971x_ManagerType *pBmcCtx, const ServiceCfgType *pSvcCfg, uint16 *pOutVcellVolt)
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

FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeDiagRdnt(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg,
                                              uint32 *pOutDiagRdnt)
{
    uint8 *pData;
    uint8 uRet = (uint8) BMI_NOT_OK;
    uint8 uDevIdx;
    uint8 uLoop = pBmicMgr->uSDevId;


    if((pOutDiagRdnt != NULL) && ((uint8) RESP_VALID == pSvcCfg->pSvcStat->uRespStatus))
    {
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
        for(uDevIdx = uLoop; uDevIdx < pBmicMgr->uNAFEs; uDevIdx++)
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pBmicMgr->uNAFEs, uDevIdx);

            pOutDiagRdnt[((uDevIdx-pBmicMgr->uSDevId))] = \
                                   (((uint32) pData[0] << 16) | ((uint16) pData[1u]<< 8) | (pData[2u]));

        }
        pSvcCfg->pSvcStat->uInfo = (uint8) RESP_DECODED;
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

        uRet = (uint8) BMI_OK;
    }

    return uRet;
}

/**********************************************************************************************************************
 * uint8 Bq7971x_DecodeI2CData(Bq7971x_ManagerType *pBmcCtx, const ServiceCfgType *pSvcCfg, uint16 *pOutVcellVolt)
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

FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeI2CData(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg,
                                              uint16 *pOutI2CData)
{
    uint8 *pData;
    uint8 uRet = (uint8) BMI_NOT_OK;
    uint8 uDevIdx;
    uint8 uLoop = pBmicMgr->uSDevId;


    if((pOutI2CData != NULL) && ((uint8) RESP_VALID == pSvcCfg->pSvcStat->uRespStatus))
    {
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
        for(uDevIdx = uLoop; uDevIdx < pBmicMgr->uNAFEs; uDevIdx++)
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pBmicMgr->uNAFEs, uDevIdx);

            pOutI2CData[((uDevIdx-pBmicMgr->uSDevId))] = \
                                   (uint16) ((pData[0] << 8) | pData[1u]);

        }
        pSvcCfg->pSvcStat->uInfo = (uint8) RESP_DECODED;
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

        uRet = (uint8) BMI_OK;
    }

    return uRet;
}

/**********************************************************************************************************************
 * uint8 Bq7971x_DecodeI2CFault(Bq7971x_ManagerType *pBmcCtx, const ServiceCfgType *pSvcCfg, uint16 *pOutVcellVolt)
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

FUNC(uint8, bq7971x_CODE) Bq7971x_DecodeI2CFault(const Bq7971x_ManagerType *pBmicMgr, const ServiceCfgType *pSvcCfg,
                                              uint16 *pOutI2CFault)
{
    uint8 *pData;
    uint8 uRet = (uint8) BMI_NOT_OK;
    uint8 uDevIdx;
    uint8 uLoop = pBmicMgr->uSDevId;


    if((pOutI2CFault != NULL) && ((uint8) RESP_VALID == pSvcCfg->pSvcStat->uRespStatus))
    {
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, GET_RESOURCE);
        for(uDevIdx = uLoop; uDevIdx < pBmicMgr->uNAFEs; uDevIdx++)
        {
            pData = BQ7971X_GET_DATA_PTR(pSvcCfg, pBmicMgr->uNAFEs, uDevIdx);

            pOutI2CFault[((uDevIdx-pBmicMgr->uSDevId))] = \
                                   (((uint16) pData[0] << 8) | pData[1u]);

        }
        pSvcCfg->pSvcStat->uInfo = (uint8) RESP_DECODED;
        pBmicMgr->pBswCfg->resource_req(pBmicMgr->pBswCfg->qApplRes, RELEASE_RESOURCE);

        uRet = (uint8) BMI_OK;
    }

    return uRet;
}


#define BQ7971X_FUNCS_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 * End of File: bq7971x_funcs.c
 *********************************************************************************************************************/
