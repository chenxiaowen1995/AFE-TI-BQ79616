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
 *  File:       tibms_Lcfg_2.c
 *  Project:    TIBMS
 *  Module:     CFG
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Linktime configuration for TI WBMS
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05May2022    SEM                0000000000000    Initial version
 * 01.01.00       04Aug2023    SEM                0000000000000    Configuration Updates
 *
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Standard Header Files
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Other Header Files
 *********************************************************************************************************************/

#include "tibms_api.h"
#include "tibms_bmi.h"
#include "tibms_comif.h"
#include "Spi.h"
#include "Os.h"
#include "Icu.h"
#include "Dio.h"


/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

/**	Major Software Config C Version number */
#define TIBMS_LCFG_C_MAJOR_VERSION             (0x01u)

/**	Minor Software Config C Version number */
#define TIBMS_LCFG_C_MINOR_VERSION             (0x01u)

/** Software Patch Config C Version number */
#define TIBMS_LCFG_C_PATCH_VERSION             (0x00u)


#if (  (TIBMS_CFG_MAJOR_VERSION != TIBMS_LCFG_C_MAJOR_VERSION) \
    || (TIBMS_CFG_MINOR_VERSION != TIBMS_LCFG_C_MINOR_VERSION) \
	|| (TIBMS_CFG_PATCH_VERSION != TIBMS_LCFG_C_PATCH_VERSION))
#error "tibms: Config version numbers of tibms_Lcfg.c and tibms_cfg.h are inconsistent!"
#endif

/*********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

#define BMI_SGL_RSP_PREFIX_LEN                  (CC2662_RSP_FRAME_PREFIX_LEN)       //5u
#define BMI_SGL_RSP_FIXED_LEN                   (CC2662_REQ_FRAME_FIXED_LEN)        //6u

#define BMI_DEVADDR_READ_BLOCK_LEN_2            (BMI_SGL_RSP_FIXED_LEN + 1u)
#define BMI_VCELL_READ_BLOCK_LEN_2              (BMI_SGL_RSP_FIXED_LEN + (BQ7971X_CELL_NUM_ACT * 2u))
#define BMI_GPIO_READ_BLOCK_LEN_2               (BMI_SGL_RSP_FIXED_LEN + (BQ7971X_GPIO_NUM_MAX * 2u))

#define BMI_CELLBAL_GETTIME_BLOCK_LEN_2         (BMI_SGL_RSP_FIXED_LEN + 1u)
#define BMI_CELLBAL_GETDONESTAT_BLOCK_LEN_2     (BMI_SGL_RSP_FIXED_LEN + 3u)
#define BMI_CELLBAL_GETSWSTAT_BLOCK_LEN_2       (BMI_SGL_RSP_FIXED_LEN + 3u)
#define BMI_CELLBAL_GETSTAT_BLOCK_LEN_2         (BMI_SGL_RSP_FIXED_LEN + 1u)

#define BMI_DIAG_GENERIC_DATA_BUF_LEN_2         (BMI_SGL_RSP_FIXED_LEN + 10u)
#define BMI_DIAG_DIETEMP_BUF_LEN_2              (BMI_SGL_RSP_FIXED_LEN + 2u)

#define BMI_GENERIC_DATA_BUF_LEN_2              (BMI_SGL_RSP_FIXED_LEN + 10u)
#define COMM_GENERIC_DATA_BUF_LEN_2             (BMI_SGL_RSP_FIXED_LEN + 6u)

#define WBMS_RAWDATA_LEN                        (128)

#define WBMS_CTRL_WD_RXPKT_LEN                  (5)
#define WBMS_CTRL_WM_RXPKT_LEN                  (5)
#define WBMS_CTRL_WM_NOC_LEN                    (2)
#define WBMS_DIAG_NETTIME_LEN                   (4)
#define WBMS_DIAG_KEEPAL_LEN                    (4)

#define WBMS_CTRL_WM_IDEN_LEN 					(33)
#define WBMS_CTRL_WM_NUMOFCONN_LEN 				(1)
#define WBMS_CTRL_WM_NUMOFTXPACKETS_LEN 		(4)
#define WBMS_CTRL_WM_NUMOFTXFAILEDPACKETS_LEN 	(4)
#define WBMS_CTRL_WM_TXTHROUGHPUT_LEN 			(4)
#define WBMS_CTRL_WM_RXTHROUGHPUT_LEN 			(4)

#define WBMS_CTRL_WD_RSSIOFNODE_LEN 			(4U)
#define WBMS_CTRL_WD_NUMOFRXPACKETS_LEN 		(5U)
#define WBMS_CTRL_WD_NUMOFMISSEDRXPACKETS_LEN 	(5U)
#define WBMS_CTRL_WD_GETPEROFNODE_LEN 			(5U)

#define WBMS_CTRL_APP_DIAG_SETMAINCONFIG_LEN 	(1)
#define WBMS_CTRL_APP_DIAG_STARTNETWORK_LEN 	(1)
#define WBMS_CTRL_APP_DIAG_SETPOWERMODE_LEN 	(1)
#define WBMS_CTRL_APP_DIAG_SETPARAMS_LEN 	    (2)
#define WBMS_CTRL_APP_DIAG_STARTNETWORK_LEN 	(1)
#define WBMS_CTRL_APP_DIAG_SETJOINMODE_LEN 		(1)
#define WBMS_CTRL_APP_DIAG_SETDEVTBLCFG_LEN 	(1)
#define WBMS_CTRL_APP_DIAG_UNPAIRREQ_LEN 		(1)
#define WBMS_CTRL_APP_DIAG_REPAIRREQ_LEN        (1)
#define WBMS_CTRL_APP_DIAG_DISCOVER_LEN 		(1)
#define WBMS_CTRL_APP_DIAG_RESYNC_LEN 			(1)
#define WBMS_CTRL_APP_DIAG_SETNWTIME_LEN 		(4)
#define WBMS_CTRL_APP_DIAG_GETNETWORKSTATS_LEN 	(9)
#define WBMS_CTRL_APP_DIAG_GETTXPACKETS_LEN 	(8)
#define WBMS_CTRL_APP_DIAG_GETHROUGHTPUT_LEN 	(8)
#define WBMS_CTRL_APP_DIAG_GETRXPACKETS_LEN 	(9)
#define WBMS_CTRL_APP_DIAG_GETNWTIME_LEN 		(4)
#define WBMS_CTRL_APP_DIAG_GETSTATS_LEN 		(37)
#define WBMS_CTRL_APP_DIAG_GETPARAMS_LEN 		(36)
#define WBMS_CTRL_APP_DIAG_SETWKUPPRD_LEN 		(4)
#define WBMS_CTRL_APP_DIAG_GETKLVDURATION_LEN 	(2)
#define WBMS_CTRL_APP_DIAG_FWVERSION_LEN	 	(4)
#define WBMS_CTRL_APP_DIAG_OADRESP_LEN		 	(35)
#define WBMS_CTRL_APP_DIAG_DFURESP_LEN          (35)

#define WBMS_CALLBACK_STATECHANGE_LEN           (10)
#define WBMS_CALLBACK_PAIRIND_LEN               (11)
#define WBMS_CALLBACK_RESNCCNF_LEN              (1)
#define WBMS_CALLBACK_DIAGERROR_LEN             (1)
#define WBMS_CALLBACK_RESETIND_LEN              (12)
#define WBMS_CALLBACK_WKUPIND_LEN               (0)
#define WBMS_CALLBACK_BQFAULT_LEN               (64U)
#define WBMS_CALLBACK_TXCNF_LEN	                (1)
#define WBMS_CALLBACK_FORMATIONTIME_LEN         (4)
#define WBMS_CALLBACK_DISCOVERIND_LEN           (19)
#define WBMS_CALLBACK_DISCOVERCNF_LEN           (1)
#define WBMS_CALLBACK_APP_USER_LEN              (20)

/*********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

#define TIBMS_LCFG_2_START_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

/** Functional  Services **/

ServiceStatusType zWbmsServiceStatus_2[BMI_SERVICE_MAX];
ServiceStatusType zWbmsCellBalStatus_2[BMI_CB_RESP_END];
uint8 zWbmsVCellDataBuf_2[TIBMS_MAX_AFES_IN_IFACE_2][BMI_VCELL_READ_BLOCK_LEN_2];
uint8 zWbmsGpioDataBuf_2[TIBMS_MAX_AFES_IN_IFACE_2][BMI_GPIO_READ_BLOCK_LEN_2];
uint8 zWbmsCellBalGetTimeData_2[TIBMS_MAX_AFES_IN_IFACE_2][BMI_CELLBAL_GETTIME_BLOCK_LEN_2];
uint8 zWbmsCellBalDoneStatData_2[TIBMS_MAX_AFES_IN_IFACE_2][BMI_CELLBAL_GETDONESTAT_BLOCK_LEN_2];
uint8 zWbmsCellBalSWStatData_2[TIBMS_MAX_AFES_IN_IFACE_2][BMI_CELLBAL_GETSWSTAT_BLOCK_LEN_2];
uint8 zWbmsCellBalStatData_2[TIBMS_MAX_AFES_IN_IFACE_2][BMI_CELLBAL_GETSTAT_BLOCK_LEN_2];

Bmi_uCbTime zWbmsCBDataBuf_2;


/** Diagnostics **/
ServiceStatusType zWbmsDiagStatus_2[BMI_DIAGSERVICE_END];
static uint16 zWbmsDiagVCellRead1_2[TIBMS_MAX_AFES_IN_IFACE_2][BQ7971X_CELL_NUM_ACT];
static uint16 zWbmsDiagVCellRead2_2[TIBMS_MAX_AFES_IN_IFACE_2][BQ7971X_CELL_NUM_ACT];
static uint16 zWbmsDiagVCellOpenPlau_2[TIBMS_MAX_AFES_IN_IFACE_2][BQ7971X_CELL_NUM_ACT];
static uint16 zWbmsDiagGpioRead1_2[TIBMS_MAX_AFES_IN_IFACE_2][BQ7971X_GPIO_NUM_MAX];
static uint16 zWbmsDiagGpioRead2_2[TIBMS_MAX_AFES_IN_IFACE_2][BQ7971X_GPIO_NUM_MAX];
static uint8 zWbmsDiagRespData_2[TIBMS_MAX_AFES_IN_IFACE_2][BMI_GENERIC_DATA_BUF_LEN_2];

DiagResultType zWbmsDiagResult_2[TIBMS_MAX_AFES_IN_IFACE_2];

/** Communication **/
ServiceStatusType zBqComStatus_2;
uint8 zWbmsCmdBuf[WBMS_RAWDATA_LEN];

/** Customer Data Buffer */
uint16 zWbmsVCellResultBuf_2[TIBMS_MAX_AFES_IN_IFACE_2][BQ7971X_CELL_NUM_ACT];
uint16 zWbmsGpioResultBuf_2[TIBMS_MAX_AFES_IN_IFACE_2][BQ7971X_GPIO_NUM_MAX];

uint8 zWbmsCellBalGetTimeResult_2[TIBMS_MAX_AFES_IN_IFACE_2][BMI_CELLBAL_GETTIME_BLOCK_LEN_2-BMI_SGL_RSP_FIXED_LEN];
uint8 zWbmsCellBalDoneStatResult_2[TIBMS_MAX_AFES_IN_IFACE_2][BMI_CELLBAL_GETDONESTAT_BLOCK_LEN_2-BMI_SGL_RSP_FIXED_LEN];
uint8 zWbmsCellBalSWStatResult_2[TIBMS_MAX_AFES_IN_IFACE_2][BMI_CELLBAL_GETSWSTAT_BLOCK_LEN_2-BMI_SGL_RSP_FIXED_LEN];
uint8 zWbmsCellBalStatResult_2[TIBMS_MAX_AFES_IN_IFACE_2][BMI_CELLBAL_GETSTAT_BLOCK_LEN_2-BMI_SGL_RSP_FIXED_LEN];

DiagResultType zWbmsDiagCustResult_2[TIBMS_MAX_AFES_IN_IFACE_2];
DiagStatusType zWbmsDiagCustStat_2;

#define TIBMS_LCFG_2_STOP_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 * Local CONST Object Definitions
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Function Prototypes
 *********************************************************************************************************************/

FUNC(void, WbmsApp_CODE) CDD_WbmsWmCmdRsp(uint8 uIface, uint8 uServiceId, uint8 uSubId, uint8 uStatus, void *pRxBuf);
FUNC(void, WbmsApp_CODE) CDD_WbmsFwDldRsp_OadHandler(uint8 uIface, uint8 uServiceId, uint8 uSubId, uint8 uStatus, void *pRxBuf);
FUNC(void, WbmsApp_CODE) CDD_WbmsCbk(uint8 uIface, uint8 uServiceId, uint8 uSubId, uint8 uStatus, void *pRxBuf);
FUNC(void, WbmsApp_CODE) CDD_WbmsUserCbk(uint8 uIface, uint8 uServiceId, uint8 uSubId, uint8 uStatus, void *pRxBuf);
uint8 UartTransfer(void *pTxdata, uint16 reqLenUart, void *pRxData, uint16 respLenUart);
uint8 UartRecieveData(uint8 *pRxBuf, uint16 *pOutLength, uint8 *pPendRx);
void mibspiSlaveTransferInit(void);
uint8 mibSpiSlaveGetRxData(uint8 *pRxBuf, uint16 *pOutLength, uint8 *uPendingRx);

/*********************************************************************************************************************
 * Local Inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Functions Definition
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * FUNC(uint32, tibms_Lcfg_CODE) TransferRequest_2(void *pTxBuf, uint16 wTxLength, void *pRxBuf, uint16 wRxLength)
 *********************************************************************************************************************/
/*! \brief          This function is Transfer the Data
 *
 *  \param[in]       pTxBuf: Pointer to Txdata
 *  \param[in]       wTxLength: Length of the TxBuffer
 *  \param[in]       pRxBuf: Pointer to Rxdata
 *  \param[in]       wRxLength: Length of the TxBuffer
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            None
 *  \post           None
 *  \return         uint32
 *  \retval         E_OK: Successfull return if the transfer request is accepeted
 *                  E_NOT_OK: Failed for inserting transfer request
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint32, tibms_Lcfg_CODE) TransferRequest_2(void *pTxBuf, uint16 wTxLength, void *pRxBuf, uint16 wRxLength)
{
    uint32 uRet = E_OK;

#if (WBMS_ENABLE == 1)
#if(WBMS_UART_ENABLE == 1)
#if (CONFIG_CPU_FAMILY == CPU_FAMILY_TI_TMS570)
    if((pRxBuf) && (wRxLength > 0))
    {
        sciReceive(sciREG, wRxLength, pRxBuf);
    }

    if((pTxBuf) && (wTxLength > 0))
    {
        sciSend(sciREG, wTxLength, pTxBuf);

    }
#elif (CONFIG_CPU_FAMILY == CPU_FAMILY_TI_SITARA)
    uRet = UartTransfer(pTxBuf, wTxLength, pRxBuf, wRxLength);
#endif

#else
    if((pTxBuf) && (wTxLength > 0))
    {
        if(E_OK == Spi_SetupEB(Spi_SpiChannel_0, pTxBuf, NULL, wTxLength))
        {
            if(SPI_SEQ_PENDING == Spi_GetSequenceResult(Spi_SpiSequence_0))
            {
                Spi_Cancel(Spi_SpiSequence_0);
            }
            uRet = Spi_AsyncTransmit(Spi_SpiSequence_0);
        }
    }
    else if(wRxLength > 0)
    {
        /*if(E_OK == Spi_SetupEB(Spi_SpiChannel_1, NULL, pRxBuf, wRxLength))
        {
            if(SPI_SEQ_PENDING == Spi_GetSequenceResult(Spi_SpiSequence_1))
            {
                Spi_Cancel(Spi_SpiSequence_1);
            }
            uRet = Spi_AsyncTransmit(Spi_SpiSequence_1);
        }*/
    }
#endif
#endif

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint32, tibms_Lcfg_CODE) TransferCtrl_2(uint32 qCtrlReq)
 *********************************************************************************************************************/
/*! \brief          This function is Transfer the Tx Data
 *
 *  \param[in]       pTxData: Pointer to Txdata
 *  \param[in]       qLength: Length of the TxBuffer
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            None
 *  \post           None
 *  \return         uint32
 *  \retval         E_OK: Successfull return if the transfer request is accepeted
 *                  E_NOT_OK: Failed for inserting transfer request
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint32, tibms_Lcfg_CODE) TransferCtrl_2(uint32 qReq, void *pData, uint16 *pOutData, uint8 *uPendingRx)
{
    uint32 qRet = E_NOT_OK;

    switch(qReq)
    {
        case CANCEL_TX_REQ:
        {
#if(WBMS_SPI_ENABLE == 1)
            Spi_Cancel(Spi_SpiSequence_0);
#endif
            qRet = E_OK;
            break;
        }
        case CANCEL_RX_REQ:
        {
           // Spi_Cancel(Spi_SpiSequence_1);
            qRet = E_OK;
            break;
        }
        case NOTIFY_TX_REQ:
        {
            SetEvent(TaskId_BmiIface_1, EVENT_BMI_DRIVER_TRANSFER_REQ_1);
            qRet = E_OK;
            break;
        }
        case NOTIFY_RX_REQ:
        {
            SetEvent(TaskId_BmiIface_1, EVENT_BMI_DRIVER_RECV_COMPLETE_1);
            qRet = E_OK;
            break;
        }
        case NOTIFY_RX_END_REQ:
        {
            SetEvent(TaskId_BmiIface_1, EVENT_BMI_DRIVER_TRANSFER_END_1);
            qRet = E_OK;
            break;
        }
        case GET_RX_INFO:
        {
#if (WBMS_ENABLE == 1)
#if(WBMS_UART_ENABLE == 1)
#if (CONFIG_CPU_FAMILY == CPU_FAMILY_TI_TMS570)
            qRet = sciReceiveData(sciREG, pOutData, (uint8 *) pData);
#else
            qRet = UartRecieveData(pData, pOutData, uPendingRx);
#endif
#endif
#if(WBMS_SPI_ENABLE == 1)
            if((NULL != pData) && (NULL != pOutData))
            {
                qRet = mibSpiSlaveGetRxData(pData, pOutData, uPendingRx);
            }
#endif
#endif
        }
        default:
        {
            break;
        }
    }

    return qRet;
}

/**********************************************************************************************************************
 * FUNC(uint32, tibms_Lcfg_CODE) TimerRequest_2(uint32 qTimeMs)
 *********************************************************************************************************************/
/*! \brief          This function is Transfer the Tx Data
 *
 *  \param[in]       qTimeMs: Time in Millisecond
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            None
 *  \post           None
 *  \return         uint32
 *  \retval         E_OK: Successfull return if the timer request is accepeted or cancelled
 *                  E_NOT_OK: Failed for inserting timer request
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint32, tibms_Lcfg_CODE) TimerRequest_2(uint32 qTimerReq, uint32 qTimeMs)
{
    uint32 qRet = E_OK;
    uint32 qTimerId;

    qTimerId = AlarmID_Delay_BmiIface_1;
    if(TIMER_TRANSFER == qTimerReq)
    {
        qTimerId = AlarmID_Timeout_BmiIface_1;
    }

    if(qTimeMs > 0U)
    {
        SetRelAlarm(qTimerId, qTimeMs, 0);
    }
    else
    {
        CancelAlarm(qTimerId);
    }

    return qRet;
}

/**********************************************************************************************************************
 * FUNC(uint32, tibms_Lcfg_CODE) DioRequest_2(uint32 qCtrlReq, uint32 qValue)
 *********************************************************************************************************************/
/*! \brief          This function is Transfer the Tx Data
 *
 *  \param[in]       qReq: Request type
 *  \param[in]       qValue: Value to Set
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            None
 *  \post           None
 *  \return         uint32
 *  \retval         E_OK: Successfull return if the transfer request is accepeted
 *                  E_NOT_OK: Failed for inserting transfer request
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint32, tibms_Lcfg_CODE) DioRequest_2(uint32 qCtrlReq)
{
    uint32 qRet = E_NOT_OK;

    switch(qCtrlReq)
    {
        case DIO_READ_CHANNEL:
        {
            break;
        }
        case DIO_WRITE_CHANNEL:
        {
            break;
        }
        case DIO_EDGE_ENABLE:
        {
            Icu_EnableEdgeDetection(IcuChannel_WM2H_RTC);
            Icu_EnableNotification(IcuChannel_WM2H_RTC);
            break;
        }
        case DIO_EDGE_DISABLE:
        {
            break;
        }
        default:
        {
            break;
        }
    }

    return qRet;
}

/**********************************************************************************************************************
 * FUNC(uint32, tibms_Lcfg_CODE) ResourceRequest_2(uint32 qResId, uint32 qCtrlReq)
 *********************************************************************************************************************/
/*! \brief          This function is Transfer the Tx Data
 *
 *  \param[in]       qReq: Request type
 *  \param[in]       qResId: Resource to set
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            None
 *  \post           None
 *  \return         uint32
 *  \retval         E_OK: Successfull return if the transfer request is accepeted
 *                  E_NOT_OK: Failed for inserting transfer request
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint32, tibms_Lcfg_CODE) ResourceRequest_2(uint32 qResId, uint32 qCtrlReq)
{
    uint32 qRet = E_NOT_OK;

    switch(qCtrlReq)
    {
        case GET_RESOURCE:
        {
            GetResource(qResId);
            qRet = E_OK;
            break;
        }
        case RELEASE_RESOURCE:
        {
            ReleaseResource(qResId);
            qRet = E_OK;
            break;
        }
        default:
        {
            break;
        }
    }

    return qRet;
}

/**********************************************************************************************************************
 * FUNC(uint32, tibms_Lcfg_CODE) Bsw_TimestampRequest_2(uint32 *Value, uint32 *ElapsedValue)
 *********************************************************************************************************************/
/*! \brief          This function is Transfer the Tx Data
 *
 *  \param[in]       qReq: Request type
 *  \param[in]       qResId: Resource to set
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            None
 *  \post           None
 *  \return         uint32
 *  \retval         E_OK: Successfull return if the transfer request is accepeted
 *                  E_NOT_OK: Failed for inserting transfer request
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint32, tibms_Lcfg_CODE) TimestampRequest_2(uint32 *Value, uint32 *ElapsedValue)
{
    uint32 qRet = E_NOT_OK;

    if(ElapsedValue != NULL)
    {
        qRet = GetElapsedValue(OsCounter, (TickRefType ) Value, (TickRefType) ElapsedValue);
    }
    else
    {
        qRet = GetCounterValue(OsCounter, Value);
    }

    return qRet;
}

/**********************************************************************************************************************
 * uint32 Bsw_CfgInit_2(void)
 *********************************************************************************************************************/
/*! \brief          This function is for peripheral init.
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            None
 *  \post           None
 *  \return         uint32
 *  \retval         E_OK: CRC
 *                  E_NOT_OK:
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint32, tibms_Lcfg_CODE) BasicCfgInit_2(void)
{
    uint32 qRet = E_OK;

#if (CONFIG_CPU_FAMILY == CPU_FAMILY_TI_TMS570)

#if(WBMS_SPI_ENABLE == 1)
    mibspiInit();
    mibspiSlaveTransferInit();
#endif

#if(WBMS_UART_ENABLE == 1)
    Icu_EnableEdgeDetection(UartIcuChannel_0);
    Icu_EnableNotification(UartIcuChannel_0);
#endif
#endif

    return qRet;
}

/**********************************************************************************************************************
 * uint32 Bsw_CfgDeInit_2(void)
 *********************************************************************************************************************/
/*! \brief          This function is for peripheral Deinit.
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            None
 *  \post           None
 *  \return         uint32
 *  \retval         E_OK:
 *                  E_NOT_OK:
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint32, tibms_Lcfg_CODE) BasicCfgDeInit_2(void)
{
    uint32 qRet = E_OK;

    return qRet;
}

/*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

#define TIBMS_LCFG_2_START_SEC_CONST
#include "Cdd_MemMap.h"

CONST(ServiceCfgType, TIBMSCFG_CONST) zBmiFuncCfg_2[BMI_SERVICE_MAX] =
{
	{
		BMI_SERVICE_CONFIG,
        COMM_GETNUMOFCONN,
		BMI_IFACE_1,
        0u,                                         // Offset zero for Number of connections
		(void *) zWbmsDiagRespData_2,
        NULL,
        NULL,
        NULL,
		(TIBMS_MAX_DEVS_IN_IFACE_2 * BMI_DEVADDR_READ_BLOCK_LEN_2),
		BMI_DEVADDR_READ_BLOCK_LEN_2,
        (BMI_DEVADDR_READ_BLOCK_LEN_2 - BMI_SGL_RSP_FIXED_LEN),
        &zWbmsServiceStatus_2[BMI_SERVICE_CONFIG],
        Bmi_ProcessRxData,
        NULL
	},
	{
	    BMI_SERVICE_BALANCING,
        0u,
		BMI_IFACE_1,
        0u,
		(void *) zWbmsCBDataBuf_2,
        NULL,
        NULL,
        NULL,
		(TIBMS_MAX_AFES_IN_IFACE_2 * BQ7971X_CELL_NUM_ACT),
		BQ7971X_CELL_NUM_ACT,
        BQ7971X_CELL_NUM_ACT,
        &zWbmsServiceStatus_2[BMI_SERVICE_BALANCING],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_CELL_VOLT,
        0u,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsVCellDataBuf_2,
		(void *) zWbmsVCellResultBuf_2,
        NULL,
        NULL,
		(TIBMS_MAX_AFES_IN_IFACE_2 * BMI_VCELL_READ_BLOCK_LEN_2),
		BMI_VCELL_READ_BLOCK_LEN_2,
        (BMI_VCELL_READ_BLOCK_LEN_2 - BMI_SGL_RSP_FIXED_LEN),
        &zWbmsServiceStatus_2[BMI_SERVICE_CELL_VOLT],
        Bmi_ProcessRxData,
        NULL
	},
    {
        BMI_SERVICE_GPIO_READ,
        0u,
        BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zWbmsGpioDataBuf_2,
        (void *) zWbmsGpioResultBuf_2,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * BMI_GPIO_READ_BLOCK_LEN_2),
        BMI_GPIO_READ_BLOCK_LEN_2,
        (BMI_GPIO_READ_BLOCK_LEN_2 - BMI_SGL_RSP_FIXED_LEN),
        &zWbmsServiceStatus_2[BMI_SERVICE_GPIO_READ],
        Bmi_ProcessRxData,
        NULL
    },
    {
        BMI_SERVICE_DIAGNOSTICS,
        0u,
        BMI_IFACE_1,
        0u,
        NULL,
        (void *) &zWbmsDiagResult_2[0],
        (void *) &zWbmsDiagCustResult_2[0],
        (void *) &zWbmsDiagCustStat_2,
        0u,
        0u,
        0u,
        &zWbmsServiceStatus_2[BMI_SERVICE_DIAGNOSTICS],
        Bmi_ProcessRxData,
        NULL
    },
    {
        BMI_SERVICE_COMMUNICATION,
        0u,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        32,
        0u,
        0u,
        &zWbmsServiceStatus_2[BMI_SERVICE_COMMUNICATION],
        Bmi_ProcessRxData,
        NULL
    }
};

CONST(ServiceCfgType, TIBMSCFG_CONST) zBmiComCfg_2[] =
{
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_GETIDENTITY,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_WM_IDEN_LEN, //response length
        WBMS_CTRL_WM_IDEN_LEN, //response length
        WBMS_CTRL_WM_IDEN_LEN, //response length
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_GETNUMOFCONN,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_WM_NUMOFCONN_LEN,
		WBMS_CTRL_WM_NUMOFCONN_LEN,
		WBMS_CTRL_WM_NUMOFCONN_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
	{
        BMI_SERVICE_COMMUNICATION,
        COMM_GETNUMOFTXPACKETS,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_WM_NUMOFTXPACKETS_LEN,
		WBMS_CTRL_WM_NUMOFTXPACKETS_LEN,
		WBMS_CTRL_WM_NUMOFTXPACKETS_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
	{
        BMI_SERVICE_COMMUNICATION,
        COMM_GETNUMOFTXFAILEDPACKETS,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_WM_NUMOFTXFAILEDPACKETS_LEN,
		WBMS_CTRL_WM_NUMOFTXFAILEDPACKETS_LEN,
		WBMS_CTRL_WM_NUMOFTXFAILEDPACKETS_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_GETTXTHROUGHPUT,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_WM_TXTHROUGHPUT_LEN,
		WBMS_CTRL_WM_TXTHROUGHPUT_LEN,
		WBMS_CTRL_WM_TXTHROUGHPUT_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
	{
        BMI_SERVICE_COMMUNICATION,
        COMM_GETRXTHROUGHPUT,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_WM_RXTHROUGHPUT_LEN ,
		WBMS_CTRL_WM_RXTHROUGHPUT_LEN ,
		WBMS_CTRL_WM_RXTHROUGHPUT_LEN ,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_GETNUMOFRXPACKETS,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (WBMS_CTRL_WD_NUMOFRXPACKETS_LEN)),
		WBMS_CTRL_WD_NUMOFRXPACKETS_LEN,
		WBMS_CTRL_WD_NUMOFRXPACKETS_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_GETNUMOFMISSEDRXPACKETS,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (WBMS_CTRL_WD_NUMOFMISSEDRXPACKETS_LEN)),
		WBMS_CTRL_WD_NUMOFMISSEDRXPACKETS_LEN,
		WBMS_CTRL_WD_NUMOFMISSEDRXPACKETS_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_GETLATENCYOFNODE,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (WBMS_CTRL_WD_GETPEROFNODE_LEN)),
        5,
        5,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_GETPEROFNODE,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (WBMS_CTRL_WD_GETPEROFNODE_LEN )),
		WBMS_CTRL_WD_GETPEROFNODE_LEN ,
		WBMS_CTRL_WD_GETPEROFNODE_LEN ,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_SENDBQRAWFRAME,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (BMI_SGL_RSP_FIXED_LEN+1u)),
        (BMI_SGL_RSP_FIXED_LEN+2u),
        2u,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_SENDWDSTORAGE,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (BMI_SGL_RSP_FIXED_LEN+1u)),
        (BMI_SGL_RSP_FIXED_LEN+2u),
        2u,
        &zBqComStatus_2,
		Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_RESETDEVICE,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (BMI_SGL_RSP_FIXED_LEN+2u)),
        (BMI_SGL_RSP_FIXED_LEN+2u),
        2u,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
	{
        BMI_SERVICE_COMMUNICATION,
        COMM_GETRSSIOFNODE,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (WBMS_CTRL_WD_RSSIOFNODE_LEN )),
		WBMS_CTRL_WD_RSSIOFNODE_LEN ,
		WBMS_CTRL_WD_RSSIOFNODE_LEN ,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_SETMAINCONFIG,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_SETMAINCONFIG_LEN ,
		WBMS_CTRL_APP_DIAG_SETMAINCONFIG_LEN ,
		WBMS_CTRL_APP_DIAG_SETMAINCONFIG_LEN ,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_OADREQ,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_OADRESP_LEN,
        WBMS_CTRL_APP_DIAG_OADRESP_LEN,
        WBMS_CTRL_APP_DIAG_OADRESP_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsFwDldRsp_OadHandler
    },
	{
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_STARTNETWORK,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_STARTNETWORK_LEN ,
		WBMS_CTRL_APP_DIAG_STARTNETWORK_LEN ,
		WBMS_CTRL_APP_DIAG_STARTNETWORK_LEN ,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_SETPOWERMODE,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_SETPOWERMODE_LEN ,
		WBMS_CTRL_APP_DIAG_SETPOWERMODE_LEN ,
		WBMS_CTRL_APP_DIAG_SETPOWERMODE_LEN ,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
		CDD_WbmsWmCmdRsp
    },
	{
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_RUNDWM,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_SETPOWERMODE_LEN ,
		WBMS_CTRL_APP_DIAG_SETPOWERMODE_LEN ,
		WBMS_CTRL_APP_DIAG_SETPOWERMODE_LEN ,
        &zBqComStatus_2,
		Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_DISCOVERREQ,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_DISCOVER_LEN ,
		WBMS_CTRL_APP_DIAG_DISCOVER_LEN ,
		WBMS_CTRL_APP_DIAG_DISCOVER_LEN ,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_SETNWKOPMODE,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_STARTNETWORK_LEN,
		WBMS_CTRL_APP_DIAG_STARTNETWORK_LEN,
		WBMS_CTRL_APP_DIAG_STARTNETWORK_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_SETJOINMODE,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_SETJOINMODE_LEN,
		WBMS_CTRL_APP_DIAG_SETJOINMODE_LEN,
		WBMS_CTRL_APP_DIAG_SETJOINMODE_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_SETDEVTBLCFG,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_SETDEVTBLCFG_LEN ,
		WBMS_CTRL_APP_DIAG_SETDEVTBLCFG_LEN ,
		WBMS_CTRL_APP_DIAG_SETDEVTBLCFG_LEN ,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_UNPAIRREQ,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_UNPAIRREQ_LEN,
		WBMS_CTRL_APP_DIAG_UNPAIRREQ_LEN,
		WBMS_CTRL_APP_DIAG_UNPAIRREQ_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_RESYNC,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_RESYNC_LEN ,
		WBMS_CTRL_APP_DIAG_RESYNC_LEN ,
		WBMS_CTRL_APP_DIAG_RESYNC_LEN ,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_WM_REPAIRREQ,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_REPAIRREQ_LEN,
		WBMS_CTRL_APP_DIAG_REPAIRREQ_LEN,
		WBMS_CTRL_APP_DIAG_REPAIRREQ_LEN,
        &zBqComStatus_2,
		Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
	{
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_SETWKUPPRD,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_SETWKUPPRD_LEN ,
		WBMS_CTRL_APP_DIAG_SETWKUPPRD_LEN ,
		WBMS_CTRL_APP_DIAG_SETWKUPPRD_LEN ,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
	{
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_SETPARAMS,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_SETPARAMS_LEN ,
		WBMS_CTRL_APP_DIAG_SETPARAMS_LEN ,
		WBMS_CTRL_APP_DIAG_SETPARAMS_LEN ,
        &zBqComStatus_2,
		Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
	{
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_DFUREQ,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_DFURESP_LEN,
        WBMS_CTRL_APP_DIAG_DFURESP_LEN,
        WBMS_CTRL_APP_DIAG_DFURESP_LEN,
        &zBqComStatus_2,
		Bmi_ProcessRxData,
        CDD_WbmsFwDldRsp_OadHandler
    },
	{
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_GETFWVERSION,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_FWVERSION_LEN,
		WBMS_CTRL_APP_DIAG_FWVERSION_LEN,
		WBMS_CTRL_APP_DIAG_FWVERSION_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_GETNETWORKSTATS,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_GETNETWORKSTATS_LEN ,
		WBMS_CTRL_APP_DIAG_GETNETWORKSTATS_LEN ,
		WBMS_CTRL_APP_DIAG_GETNETWORKSTATS_LEN ,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
	{
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_GETTXPACKETS,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_GETTXPACKETS_LEN ,
		WBMS_CTRL_APP_DIAG_GETTXPACKETS_LEN ,
		WBMS_CTRL_APP_DIAG_GETTXPACKETS_LEN ,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
	{
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_GETHROUGHTPUT,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_GETHROUGHTPUT_LEN ,
		WBMS_CTRL_APP_DIAG_GETHROUGHTPUT_LEN ,
		WBMS_CTRL_APP_DIAG_GETHROUGHTPUT_LEN ,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
	{
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_GETRXPACKETS,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_GETRXPACKETS_LEN ,
		WBMS_CTRL_APP_DIAG_GETRXPACKETS_LEN ,
		WBMS_CTRL_APP_DIAG_GETRXPACKETS_LEN ,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_GETSTATS,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_GETSTATS_LEN,
        WBMS_CTRL_APP_DIAG_GETSTATS_LEN,
        WBMS_CTRL_APP_DIAG_GETSTATS_LEN,
        &zBqComStatus_2,
		Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_GETPARAMS,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_GETPARAMS_LEN,
        WBMS_CTRL_APP_DIAG_GETPARAMS_LEN,
        WBMS_CTRL_APP_DIAG_GETPARAMS_LEN,
        &zBqComStatus_2,
		Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
	{
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_GETKLVDURATION,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_GETKLVDURATION_LEN,
		WBMS_CTRL_APP_DIAG_GETKLVDURATION_LEN,
		WBMS_CTRL_APP_DIAG_GETKLVDURATION_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_DEVICEPAIRIND_CB,
        2u,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CALLBACK_PAIRIND_LEN,
		WBMS_CALLBACK_PAIRIND_LEN,
		WBMS_CALLBACK_PAIRIND_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsCbk
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_OAD_CB,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_OADRESP_LEN,
        WBMS_CTRL_APP_DIAG_OADRESP_LEN,
        WBMS_CTRL_APP_DIAG_OADRESP_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsFwDldRsp_OadHandler
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_STATECHANGE_CB,
        2u,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CALLBACK_STATECHANGE_LEN,
		WBMS_CALLBACK_STATECHANGE_LEN,
		WBMS_CALLBACK_STATECHANGE_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsCbk
    },
	{
        BMI_SERVICE_COMMUNICATION,
		COMM_APP_DIAG_TXCNF_CB,
        2u,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
		WBMS_CALLBACK_TXCNF_LEN,
		WBMS_CALLBACK_TXCNF_LEN,
		WBMS_CALLBACK_TXCNF_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsCbk
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_APP_DIAG_ERROR_CB,
        2u,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CALLBACK_DIAGERROR_LEN,
		WBMS_CALLBACK_DIAGERROR_LEN,
		WBMS_CALLBACK_DIAGERROR_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsCbk
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_FORMATIONTIME_CB,
        2u,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CALLBACK_FORMATIONTIME_LEN,
		WBMS_CALLBACK_FORMATIONTIME_LEN,
		WBMS_CALLBACK_FORMATIONTIME_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsCbk
    },
	{
        BMI_SERVICE_COMMUNICATION,
		COMM_APP_DIAG_BQFAULT_CB,
        2u,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
		(TIBMS_MAX_AFES_IN_IFACE_2 * WBMS_CALLBACK_BQFAULT_LEN),
		WBMS_CALLBACK_BQFAULT_LEN,
		WBMS_CALLBACK_BQFAULT_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsCbk
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_RESYNCCNF_CB,
        2u,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CALLBACK_RESNCCNF_LEN,
		WBMS_CALLBACK_RESNCCNF_LEN,
		WBMS_CALLBACK_RESNCCNF_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsCbk
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_RESET_CB,
        2u,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CALLBACK_RESETIND_LEN,
		WBMS_CALLBACK_RESETIND_LEN,
		WBMS_CALLBACK_RESETIND_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsCbk
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_EXITPS_CB,
        2u,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CALLBACK_WKUPIND_LEN,
		WBMS_CALLBACK_WKUPIND_LEN,
		WBMS_CALLBACK_WKUPIND_LEN,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsCbk
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_DFU_CB,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CTRL_APP_DIAG_DFURESP_LEN,
        WBMS_CTRL_APP_DIAG_DFURESP_LEN,
        WBMS_CTRL_APP_DIAG_DFURESP_LEN,
        &zBqComStatus_2,
		Bmi_ProcessRxData,
        CDD_WbmsFwDldRsp_OadHandler
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_DISCOVERIND_CB,                                          /* 0 */
        2u,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CALLBACK_DISCOVERIND_LEN,
        WBMS_CALLBACK_DISCOVERIND_LEN,
        WBMS_CALLBACK_DISCOVERIND_LEN,
        &zBqComStatus_2,
		Bmi_ProcessRxData,
		CDD_WbmsCbk
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_DIAG_DISCOVERCNF_CB,                                          /* 0 */
        2u,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CALLBACK_DISCOVERCNF_LEN,
        WBMS_CALLBACK_DISCOVERCNF_LEN,
        WBMS_CALLBACK_DISCOVERCNF_LEN,
        &zBqComStatus_2,
		Bmi_ProcessRxData,
        CDD_WbmsCbk
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_APP_USER_CB,                                          
        2u,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        WBMS_CALLBACK_APP_USER_LEN,
        WBMS_CALLBACK_APP_USER_LEN,
        WBMS_CALLBACK_APP_USER_LEN,
        &zBqComStatus_2,
		Bmi_ProcessRxData,
        CDD_WbmsUserCbk
    },
        {
        BMI_SERVICE_COMMUNICATION,
        COMM_RESET,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (BMI_SGL_RSP_FIXED_LEN+2u)),
        (BMI_SGL_RSP_FIXED_LEN+2u),
        2u,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    },
    {
        BMI_SERVICE_COMMUNICATION,
        COMM_SLEEP,
        BMI_IFACE_1,
        0u,
        (void *) zWbmsCmdBuf,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (BMI_SGL_RSP_FIXED_LEN+2u)),
        (BMI_SGL_RSP_FIXED_LEN+2u),
        2u,
        &zBqComStatus_2,
        Bmi_ProcessRxData,
        CDD_WbmsWmCmdRsp
    }
};

CONST(ServiceCfgType, TIBMSCFG_CONST) zBmiCellBalCfg_2[] =
{
    {
		BMI_SERVICE_BALANCING,
        BMI_CB_GET_BALSTAT,
		BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsCellBalStatData_2,
		(void *) zWbmsCellBalStatResult_2,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+1u)),
        (6u+1u),
        1u,
        &zWbmsCellBalStatus_2[BMI_CB_GET_BALSTAT],
        Bmi_ProcessRxData,
        NULL
	},
    {
		BMI_SERVICE_BALANCING,
        BMI_CB_GET_BALSWSTAT,
		BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsCellBalSWStatData_2,
		(void *) zWbmsCellBalSWStatResult_2,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+3u)),
        (6u+3u),
        3u,
        &zWbmsCellBalStatus_2[BMI_CB_GET_BALSWSTAT],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_BALANCING,
        BMI_CB_GET_BALDONESTAT,
		BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsCellBalDoneStatData_2,
		(void *) zWbmsCellBalDoneStatResult_2,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+3u)),
        (6u+3u),
        3u,
        &zWbmsCellBalStatus_2[BMI_CB_GET_BALDONESTAT],
        Bmi_ProcessRxData,
        NULL
	},
    {
		BMI_SERVICE_BALANCING,
        BMI_CB_GET_BALTIME,
		BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsCellBalGetTimeData_2,
		(void *) zWbmsCellBalGetTimeResult_2,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+1u)),
        (6u+1u),
        1u,
        &zWbmsCellBalStatus_2[BMI_CB_GET_BALTIME],
        Bmi_ProcessRxData,
        NULL
	}
};

CONST(Basic_ConfigType, TIBMSCFG_CONST) zBmiBswCfg_2 =
{
    BasicCfgInit_2,
    TransferRequest_2,
    TransferCtrl_2,
    TimerRequest_2,
    DioRequest_2,
    ResourceRequest_2,
    TimestampRequest_2,
    BasicCfgDeInit_2,

    ResID_BmiIface_1,
    ResID_App_2,
    ResID_DiagApp_2,
};

CONST(ServiceCfgType, TIBMSCFG_CONST) zBmiDiagCfg_2[] =
{
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_REG_READ_1BYTE,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+1u)),
        (6u+1u),
        1u,
        &zWbmsDiagStatus_2[BMI_SM_REG_READ_1BYTE],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_REG_READ_2BYTE,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+2u)),
        (6u+2u),
        2u,
        &zWbmsDiagStatus_2[BMI_SM_REG_READ_2BYTE],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_REG_READ_3BYTE,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+3u)),
        (6u+3u),
        3u,
        &zWbmsDiagStatus_2[BMI_SM_REG_READ_3BYTE],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_REG_READ_4BYTE,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+4u)),
        (6u+4u),
        4u,
        &zWbmsDiagStatus_2[BMI_SM_REG_READ_4BYTE],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_REG_READ_5BYTE,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+5u)),
        (6u+5u),
        5u,
        &zWbmsDiagStatus_2[BMI_SM_REG_READ_5BYTE],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_STARTUPINIT,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+2u)),
        (6u+2u),
        2u,
        &zWbmsDiagStatus_2[BMI_SM_MPFDI_STARTUPINIT],
        Bmi_ProcessRxData,
        Bq7971x_DiagInitComplete
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_BGX_DIAG,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+4u)),
        (6u+4u),
        4u,
        &zWbmsDiagStatus_2[BMI_SM_MPFDI_BGX_DIAG],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_REFCAP_DIAG,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+2u)),
        (6u+2u),
        2u,
        &zWbmsDiagStatus_2[BMI_SM_MPFDI_REFCAP_DIAG],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_PWR_BIST,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+2u)),
        (6u+2u),
        2u,
        &zWbmsDiagStatus_2[BMI_SM_MPFDI_PWR_BIST],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_ACOMP_FLTINJ,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+5u)),
        (6u+5u),
        5u,
        &zWbmsDiagStatus_2[BMI_SM_MPFDI_ACOMP_FLTINJ],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_DCOMP_FLTINJ,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+3u)),
        (6u+3u),
        3u,
        &zWbmsDiagStatus_2[BMI_SM_MPFDI_DCOMP_FLTINJ],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_CBFET_DIAG,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+3u)),
        (6u+3u),
        3u,
        &zWbmsDiagStatus_2[BMI_SM_MPFDI_CBFET_DIAG],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_OV_DAC,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+3u)),
        (6u+3u),
        3u,
        &zWbmsDiagStatus_2[BMI_SM_MPFDI_OV_DAC],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_UV_DAC,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+3u)),
        (6u+3u),
        3u,
        &zWbmsDiagStatus_2[BMI_SM_MPFDI_UV_DAC],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_OT_DAC,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+3u)),
        (6u+3u),
        3u,
        &zWbmsDiagStatus_2[BMI_SM_MPFDI_OT_DAC],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_UT_DAC,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+3u)),
        (6u+3u),
        3u,
        &zWbmsDiagStatus_2[BMI_SM_MPFDI_UT_DAC],
        Bmi_ProcessRxData,
        NULL
	},
    {
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_CBOW_FLTINJ,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+3u)),
        (6u+3u),
        3u,
        &zWbmsDiagStatus_2[BMI_SM_CBOW_FLTINJ],
        Bmi_ProcessRxData,
        NULL
	},

    // FDTI Routines
    {
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_CYCLE_INIT,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+5u)),
        (6u+5u),
        5u,
        &zWbmsDiagStatus_2[BMI_SM_FDTI_CYCLE_INIT],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_GPIO_DIAG,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zWbmsGpioDataBuf_2,
        (void *) zWbmsDiagGpioRead1_2,
        (void *) zWbmsDiagGpioRead2_2,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * BMI_GPIO_READ_BLOCK_LEN_2),
        BMI_GPIO_READ_BLOCK_LEN_2,
        (BMI_GPIO_READ_BLOCK_LEN_2 - BMI_SGL_RSP_FIXED_LEN),
        &zWbmsDiagStatus_2[BMI_SM_FDTI_GPIO_DIAG],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_GPIO_OPNWR,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zWbmsGpioDataBuf_2,
        (void *) zWbmsDiagGpioRead1_2,
        (void *) zWbmsDiagGpioRead2_2,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * BMI_GPIO_READ_BLOCK_LEN_2),
        BMI_GPIO_READ_BLOCK_LEN_2,
        (BMI_GPIO_READ_BLOCK_LEN_2 - BMI_SGL_RSP_FIXED_LEN),
        &zWbmsDiagStatus_2[BMI_SM_FDTI_GPIO_OPNWR],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_GPIO_ADJSHRT,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zWbmsGpioDataBuf_2,
        (void *) zWbmsDiagGpioRead1_2,
        (void *) zWbmsDiagGpioRead2_2,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * BMI_GPIO_READ_BLOCK_LEN_2),
        BMI_GPIO_READ_BLOCK_LEN_2,
        (BMI_GPIO_READ_BLOCK_LEN_2 - BMI_SGL_RSP_FIXED_LEN),
        &zWbmsDiagStatus_2[BMI_SM_FDTI_GPIO_ADJSHRT],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_VC_OPN_DET,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsVCellDataBuf_2,
		(void *) zWbmsDiagVCellRead1_2,
		(void *) zWbmsDiagVCellOpenPlau_2,
		NULL,
		(TIBMS_MAX_AFES_IN_IFACE_2 * BMI_VCELL_READ_BLOCK_LEN_2),
		BMI_VCELL_READ_BLOCK_LEN_2,
        (BMI_VCELL_READ_BLOCK_LEN_2 - BMI_SGL_RSP_FIXED_LEN),
        &zWbmsDiagStatus_2[BMI_SM_FDTI_VC_OPN_DET],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_VCCB_SHRT,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsVCellDataBuf_2,
		(void *) zWbmsDiagVCellRead1_2,
		(void *) zWbmsDiagVCellRead2_2,
		NULL,
		(TIBMS_MAX_AFES_IN_IFACE_2 * BMI_VCELL_READ_BLOCK_LEN_2),
		BMI_VCELL_READ_BLOCK_LEN_2,
        (BMI_VCELL_READ_BLOCK_LEN_2 - BMI_SGL_RSP_FIXED_LEN),
        &zWbmsDiagStatus_2[BMI_SM_FDTI_VCCB_SHRT],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_VCELLGPIO_ACOMP,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+5u)),
        (6u+5u),
        5u,
        &zWbmsDiagStatus_2[BMI_SM_FDTI_VCELLGPIO_ACOMP],
        Bmi_ProcessRxData,
        NULL
	},
    {
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_DCOMP_DIAG,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+3u)),
        (6u+3u),
        3u,
        &zWbmsDiagStatus_2[BMI_SM_FDTI_DCOMP_DIAG],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_ADC_COMP_DIAG,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+3u)),
        (6u+3u),
        3u,
        &zWbmsDiagStatus_2[BMI_SM_FDTI_ADC_COMP_DIAG],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_FAULT_SUMMARY,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+1u)),
        (6u+1u),
        1u,
        &zWbmsDiagStatus_2[BMI_SM_FDTI_FAULT_SUMMARY],
        Bmi_ProcessRxData,
        NULL
	},
	{
		BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_DIETMEP_DIAG,
		BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
		(void *) zWbmsDiagRespData_2,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+2u)),
        (6u+2u),
        2u,
        &zWbmsDiagStatus_2[BMI_SM_FDTI_DIETMEP_DIAG],
        Bmi_ProcessRxData,
        NULL
    },
	{
	    BMI_SERVICE_DIAGNOSTICS,
	    BMI_SM_FTDI_NFAULT,
	    BMI_IFACE_1,
	    BMI_SGL_RSP_PREFIX_LEN,
	    (void *) zWbmsDiagRespData_2,
	    NULL,
	    NULL,
	    NULL,
	    (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+2u)),
	    (6u+2u),
	    2u,
	    &zWbmsDiagStatus_2[BMI_SM_FTDI_NFAULT],
	    Bmi_ProcessRxData,
	    NULL
	},
	{
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SM_COMM_FAULT,
         BMI_IFACE_1,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zWbmsDiagRespData_2,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+1u)),
         (6u+1u),
         1u,
         &zWbmsDiagStatus_2[BMI_SM_COMM_FAULT],
         Bmi_ProcessRxData,
         NULL
	 },
	 {
	    BMI_SERVICE_DIAGNOSTICS,
	    BMI_SM_ECC_DIAG,
	    BMI_IFACE_1,
	    BMI_SGL_RSP_PREFIX_LEN,
	    (void *) zWbmsDiagRespData_2,
	    NULL,
	    NULL,
	    NULL,
	    (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+1u)),
	    (6u+1u),
	    1u,
	    &zWbmsDiagStatus_2[BMI_SM_ECC_DIAG],
	    Bmi_ProcessRxData,
	    NULL
     },
     {
	    BMI_SERVICE_DIAGNOSTICS,
	    BMI_SM_COMM_FLT_MSK_DIAG,
	    BMI_IFACE_1,
	    BMI_SGL_RSP_PREFIX_LEN,
	    (void *) zWbmsDiagRespData_2,
	    NULL,
	    NULL,
	    NULL,
	    (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+2u)),
	    (6u+2u),
	    2u,
	    &zWbmsDiagStatus_2[BMI_SM_COMM_FLT_MSK_DIAG],
	    Bmi_ProcessRxData,
	    NULL
	 },
	 {
	    BMI_SERVICE_DIAGNOSTICS,
	    BMI_SM_FACTORY_CRC_DIAG,
	    BMI_IFACE_1,
	    BMI_SGL_RSP_PREFIX_LEN,
	    (void *) zWbmsDiagRespData_2,
	    NULL,
	    NULL,
	    NULL,
	    (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+1u)),
	    (6u+1u),
	    1u,
	    &zWbmsDiagStatus_2[BMI_SM_FACTORY_CRC_DIAG],
	    Bmi_ProcessRxData,
	    NULL
	 },
	 {
	    BMI_SERVICE_DIAGNOSTICS,
	    BMI_SM_VLF_CRC_DIAG,
	    BMI_IFACE_1,
	    BMI_SGL_RSP_PREFIX_LEN,
	    (void *) zWbmsDiagRespData_2,
	    NULL,
	    NULL,
	    NULL,
	    (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+1u)),
	    (6u+1u),
	    1u,
	    &zWbmsDiagStatus_2[BMI_SM_VLF_CRC_DIAG],
	    Bmi_ProcessRxData,
	    NULL
     },
     {
	    BMI_SERVICE_DIAGNOSTICS,
	    BMI_SM_SPI_FAULT_DIAG,
	    BMI_IFACE_1,
	    BMI_SGL_RSP_PREFIX_LEN,
	    (void *) zWbmsDiagRespData_2,
	    NULL,
	    NULL,
	    NULL,
	    (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+3u)),
	    (6u+3u),
	    3u,
	    &zWbmsDiagStatus_2[BMI_SM_SPI_FAULT_DIAG],
	    Bmi_ProcessRxData,
	    NULL
    },
    {
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FAULT_ADC_MISC,
        BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zWbmsDiagRespData_2,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+1u)),
        (6u+1u),
        1u,
        &zWbmsDiagStatus_2[BMI_SM_FAULT_ADC_MISC],
        Bmi_ProcessRxData,
        NULL
	},
	{
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_VLF_FAULT_DIAG,
        BMI_IFACE_1,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zWbmsDiagRespData_2,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+1u)),
        (6u+1u),
        1u,
        &zWbmsDiagStatus_2[BMI_SM_VLF_FAULT_DIAG],
        Bmi_ProcessRxData,
        NULL
  },
  {
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SM_GET_DIAG_D1,
         BMI_IFACE_1,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zWbmsDiagRespData_2,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+2u)),
         (6u+2u),
         2u,
         &zWbmsDiagStatus_2[BMI_SM_GET_DIAG_D1],
         Bmi_ProcessRxData,
         NULL
    },
    {
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SM_GET_DIAG_D2,
         BMI_IFACE_1,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zWbmsDiagRespData_2,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+2u)),
         (6u+2u),
         2u,
         &zWbmsDiagStatus_2[BMI_SM_GET_DIAG_D2],
         Bmi_ProcessRxData,
         NULL
      },
      {
         BMI_SERVICE_DIAGNOSTICS,
         PMI_FDTI_SW_MON_DIAG,
         BMI_IFACE_1,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zWbmsDiagRespData_2,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+2u)),
         (6u+2u),
         2u,
         &zWbmsDiagStatus_2[PMI_FDTI_SW_MON_DIAG],
         Bmi_ProcessRxData,
         NULL
      },



};

#define TIBMS_LCFG_2_STOP_SEC_CONST
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * External Functions Definition
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * End of File: tibms_Lcfg_2.c
 *********************************************************************************************************************/
