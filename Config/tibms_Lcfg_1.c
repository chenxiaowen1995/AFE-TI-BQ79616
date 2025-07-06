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
 *  File:       tibms_Lcfg_0.c
 *  Project:    TIBMS
 *  Module:     CFG
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Linktime configuration for TI BMS
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
#include "Os.h"
#include "tibms_api.h"
#include "tibms_pmi.h"
#include "tibms_comif.h"
#include "bq7973x_diag.h"
/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

/**	Major Software Config C Version number */
#define TIBMS_LCFG_C_MAJOR_VERSION             (0x01u)

/**	Minor Software Config C Version number */
#define TIBMS_LCFG_C_MINOR_VERSION             (0x01u)

/** Software Patch Config C Version number */
#define TIBMS_LCFG_C_PATCH_VERSION             (0x00u)

#if (  (TIBMS_LCFG_MAJOR_VERSION != TIBMS_CFG_C_MAJOR_VERSION) \
    || (TIBMS_LCFG_MINOR_VERSION != TIBMS_CFG_C_MINOR_VERSION) \
	|| (TIBMS_LCFG_PATCH_VERSION != TIBMS_CFG_C_PATCH_VERSION))
#error "tibms: Config version numbers of tibms_Lcfg.c and tibms_cfg.h are inconsistent!"
#endif

/*********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

#define PMI_SGL_RSP_PREFIX_LEN                  (BQ7973X_RSP_FRAME_PREFIX_LEN)
#define PMI_SGL_RSP_FIXED_LEN                   (BQ7973X_REQ_FRAME_FIXED_LEN)

#define PMI_PCURRENT_READ_BLOCK_LEN             (PMI_SGL_RSP_FIXED_LEN + (BQ7973X_CURRENT_REG_NUM * 3u))
#define PMI_VF_READ_BLOCK_LEN                   (PMI_SGL_RSP_FIXED_LEN + (BQ7973X_VF_REG_NUM * 2u))
#define PMI_CP_VOLT_READ_BLOCK_LEN              (PMI_SGL_RSP_FIXED_LEN + 2u)
#define PMI_CC_ACC_CNT_READ_BLOCK_LEN           (PMI_SGL_RSP_FIXED_LEN + (BQ7973X_CC_REG_NUM * 1u))
#define PMI_VGPIO_READ_BLOCK_LEN                (PMI_SGL_RSP_FIXED_LEN + (BQ7973X_GPIO_REG_NUM * 2u))
#define PMI_DIETEMP_READ_BLOCK_LEN              (PMI_SGL_RSP_FIXED_LEN + 4u)
#define PMI_GENERIC_DATA_BUF_LEN_0              (PMI_SGL_RSP_FIXED_LEN + 10u)
#define PMI_1BYTE_READ_BLOCK_LEN              (PMI_SGL_RSP_FIXED_LEN + 1u)
/*********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

uint8 zPmiPackCurrentRead[TIBMS_MAX_AFES_IN_IFACE_1][PMI_PCURRENT_READ_BLOCK_LEN];
uint8 zPmiVfVoltRead[TIBMS_MAX_AFES_IN_IFACE_1][PMI_VF_READ_BLOCK_LEN];
uint8 zPmiCpVoltRead[TIBMS_MAX_AFES_IN_IFACE_1][PMI_CP_VOLT_READ_BLOCK_LEN];
uint8 zPmiCcAccCntRead[TIBMS_MAX_AFES_IN_IFACE_1][PMI_CC_ACC_CNT_READ_BLOCK_LEN];
uint8 zPmiVGpioRead[TIBMS_MAX_AFES_IN_IFACE_1][PMI_VGPIO_READ_BLOCK_LEN];
uint8 zPmiDieTempRead[TIBMS_MAX_AFES_IN_IFACE_1][PMI_VGPIO_READ_BLOCK_LEN];
ServiceStatusType zPmiDiagStatus_1[PMI_DIAGSERVICE_END];
static ServiceStatusType zPmiServiceStats[PMI_SERVICE_MAX];
static ServiceStatusType zPmiCommStats;

uint8 zPmiGenericRespData_1[TIBMS_MAX_AFES_IN_IFACE_0][PMI_GENERIC_DATA_BUF_LEN_0];
/** Customer Data Buffer */

uint32 zPmiPackCurrentResult[TIBMS_MAX_AFES_IN_IFACE_1][BQ7973X_CURRENT_REG_NUM];
uint16 zPmiVfVoltResult[TIBMS_MAX_AFES_IN_IFACE_1][BQ7973X_VF_REG_NUM];
uint16 zPmiCpVoltResult[TIBMS_MAX_AFES_IN_IFACE_1];
uint8 zPmiCcAccCntResult[BQ7973X_CC_REG_NUM];
uint16 zPmiVGpioRsult[TIBMS_MAX_AFES_IN_IFACE_1][BQ7973X_GPIO_REG_NUM];
uint16 zPmiDieTempResult[TIBMS_MAX_AFES_IN_IFACE_1][2];
uint8  zPmiCcClearResult         =0u;
uint8  zPmiSetI2CCtrlResult      =0u;
uint8  zPmiSetI2CDataResult      =0u;
uint8  zPmiEnableI2CResult       =0u;
uint8  zPmiEnableI2CRead         =0u;
uint8  zPmiWriteSPIDataResult    =0u;
uint8  zPmiSetSPIExeResult       =0u;
uint8  zPmiSetSPICtrlResult      =0u;
uint8  zPmiMSPIConfResult        =0u;
uint8  zPmiEnADCFreezeResult     =0u;
uint8  zPmiDisADCFreezeResult    =0u;
uint8  zPmiSetADCDelayResult     =0u;
uint8  zPmiEnterCommDebugResult  =0u;
uint8  zPmiSetOCOutputResult     =0u;



DiagResultType zPmiDiagResult_0[TIBMS_MAX_AFES_IN_IFACE_0];
DiagResultType zPmiDiagCustResult_0[TIBMS_MAX_AFES_IN_IFACE_0];
DiagStatusType zPmiDiagCustStat_0;

/**********************************************************************************************************************
 * Local CONST Object Definitions
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Function Prototypes
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Functions Definition
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * FUNC(uint32, tibms_Lcfg_CODE) TransferRequest_0(void *pTxBuf, uint16 wTxLength, void *pRxBuf, uint16 wRxLength)
 *********************************************************************************************************************/
/*! \brief          This function is Transfer the Data
 *
 *  \param[in]      pTxBuf: Pointer to Txdata
 *  \param[in]      wTxLength: Length of the TxBuffer
 *  \param[in]      pRxBuf: Pointer to Rxdata
 *  \param[in]      wRxLength: Length of the TxBuffer
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

STATIC FUNC(uint32, tibms_Lcfg_CODE) TransferRequest_0(void *pTxBuf, uint16 wTxLength, void *pRxBuf, uint16 wRxLength)
{
    uint32 uRet = E_NOT_OK;
#if (PMI_ENABLE == TRUE)
#if (PMI_BRIDGE_SPI_ENABLE  == TRUE)
    if((wTxLength > 0) && (wRxLength > 0))
    {
        if(E_OK == Spi_SetupEB(Spi_SpiChannel_0, pTxBuf, NULL, wTxLength))
        {
            if(E_OK == Spi_SetupEB(Spi_SpiChannel_1, NULL, pRxBuf, wRxLength))
            {
                if(SPI_SEQ_PENDING == Spi_GetSequenceResult(Spi_SpiSequence_1))
                {
                    Spi_Cancel(Spi_SpiSequence_1);
                }
                uRet = Spi_AsyncTransmit(Spi_SpiSequence_1);
            }
        }
    }
    else
    {
        if(wRxLength > 0)
        {
            if(E_OK == Spi_SetupEB(Spi_SpiChannel_1, NULL, pRxBuf, wRxLength))
            {
                if(SPI_SEQ_PENDING == Spi_GetSequenceResult(Spi_SpiSequence_1))
                {
                    Spi_Cancel(Spi_SpiSequence_1);
                }
                uRet = Spi_AsyncTransmit(Spi_SpiSequence_1);
            }
        }
        else if(wTxLength > 0)
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
    }
#else
    if((wTxLength > 0) && (wRxLength > 0))
    {
        if(E_OK == Spi_SetupEB(Spi_SpiChannel_2, pTxBuf, NULL, wTxLength))
        {
            if(E_OK == Spi_SetupEB(Spi_SpiChannel_3, NULL, pRxBuf, wRxLength))
            {
                if(SPI_SEQ_PENDING == Spi_GetSequenceResult(Spi_SpiSequence_3))
                {
                    Spi_Cancel(Spi_SpiSequence_3);
                }
                uRet = Spi_AsyncTransmit(Spi_SpiSequence_3);
            }
        }
    }
    else
    {
        if(wRxLength > 0)
        {
            if(E_OK == Spi_SetupEB(Spi_SpiChannel_3, NULL, pRxBuf, wRxLength))
            {
                if(SPI_SEQ_PENDING == Spi_GetSequenceResult(Spi_SpiSequence_3))
                {
                    Spi_Cancel(Spi_SpiSequence_3);
                }
                uRet = Spi_AsyncTransmit(Spi_SpiSequence_3);
            }
        }
        else if(wTxLength > 0)
        {
            if(E_OK == Spi_SetupEB(Spi_SpiChannel_2, pTxBuf, NULL, wTxLength))
            {
                if(SPI_SEQ_PENDING == Spi_GetSequenceResult(Spi_SpiSequence_2))
                {
                    Spi_Cancel(Spi_SpiSequence_2);
                }
                uRet = Spi_AsyncTransmit(Spi_SpiSequence_2);
            }
        }
    }
#endif
#endif
    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint32, tibms_Lcfg_CODE) TransferCtrl_0(uint32 qCtrlReq)
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

STATIC FUNC(uint32, tibms_Lcfg_CODE) TransferCtrl_0(uint32 qCtrlReq, void *pData, uint16 *pOutData, uint8 *uPendingRx)
{
    uint32 qRet = E_NOT_OK;

    switch(qCtrlReq)
    {
#if (PMI_ENABLE == TRUE)
        case CANCEL_TX_REQ:
        {
#if (PMI_BRIDGE_SPI_ENABLE  == TRUE)
            Spi_Cancel(Spi_SpiSequence_0);
#else
            Spi_Cancel(Spi_SpiSequence_2);
#endif
            qRet = E_OK;
            break;
        }
        case CANCEL_RX_REQ:
        {
#if (PMI_BRIDGE_SPI_ENABLE  == TRUE)
            Spi_Cancel(Spi_SpiSequence_1);
#else
            Spi_Cancel(Spi_SpiSequence_3);
#endif
            qRet = E_OK;
            break;
        }
#endif
        case NOTIFY_TX_REQ:
        {
            SetEvent(TaskId_PmiIface_0, EVENT_PMI_DRIVER_TRANSFER_REQ_0);
            qRet = E_OK;
            break;
        }
        case NOTIFY_RX_REQ:
        {
            SetEvent(TaskId_PmiIface_0, EVENT_PMI_DRIVER_RECV_COMPLETE_0);
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
 * FUNC(uint32, tibms_Lcfg_CODE) TimerRequest_0(uint32 qTimeMs)
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
 *  \retval         E_OK: Successfull return if the transfer request is accepeted
 *                  E_NOT_OK: Failed for inserting transfer request
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint32, tibms_Lcfg_CODE) TimerRequest_0(uint32 qTimerReq, uint32 qTimeMs)
{
    uint32 qRet = E_NOT_OK;

    if(qTimeMs > 0U)
    {
        SetRelAlarm(AlarmID_PmiIface_0, qTimeMs, 0);
        qRet = E_OK;
    }
    else
    {
        CancelAlarm(AlarmID_PmiIface_0);
        qRet = E_OK;
    }

    return qRet;
}

/**********************************************************************************************************************
 * FUNC(uint32, tibms_Lcfg_CODE) DioRequest_0(uint32 qCtrlReq, uint32 qValue)
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

STATIC FUNC(uint32, tibms_Lcfg_CODE) DioRequest_0(uint32 qCtrlReq)
{
    uint32 qRet = E_NOT_OK;

    switch(qCtrlReq)
    {
        case DIO_READ_CHANNEL:
        {
            qRet = Dio_ReadChannel(DioChannel_0);
            break;
        }
        case DIO_WRITE_CHANNEL:
        {
            qRet = E_OK;
            break;
        }
        case DIO_EDGE_ENABLE:
        {
            Icu_SetActivationCondition(IcuChannel_0, ICU_RISING_EDGE);
            Icu_EnableEdgeDetection(IcuChannel_0);
            Icu_EnableNotification(IcuChannel_0);
            qRet = E_OK;
            break;
        }
        case DIO_EDGE_DISABLE:
        {
            Icu_DisableNotification(IcuChannel_0);
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
 * FUNC(uint32, tibms_Lcfg_CODE) ResourceRequest_0(uint32 qCtrlReq, uint32 qResId)
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

STATIC FUNC(uint32, tibms_Lcfg_CODE) ResourceRequest_0(uint32 qResId, uint32 qCtrlReq)
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
 * FUNC(uint32, tibms_Lcfg_CODE) TimestampRequest_0(uint32 *Value, uint32 *ElapsedValue)
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

STATIC FUNC(uint32, tibms_Lcfg_CODE) TimestampRequest_0(uint32 *Value, uint32 *ElapsedValue)
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
 * FUNC(uint32, tibms_Lcfg_CODE) BasicCfgInit_0(void)
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

STATIC FUNC(uint32, tibms_Lcfg_CODE) BasicCfgInit_0(void)
{
    uint32 qRet = E_OK;

    return qRet;
}

/**********************************************************************************************************************
 * FUNC(uint32, tibms_Lcfg_CODE) BasicCfgDeInit_0(void)
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

STATIC FUNC(uint32, tibms_Lcfg_CODE) BasicCfgDeInit_0(void)
{
    uint32 qRet = E_NOT_OK;

    // TBD:
    return qRet;
}

/*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

CONST(ServiceCfgType, TIBMSCFG_CONST) zPmiFuncCfg_1[PMI_SERVICE_MAX] =
{
    {
        PMI_SERVICE_STARTUP,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        NULL,
        NULL,
        NULL,
        NULL,
        0u,
        0u,
        0u,
        &zPmiServiceStats[PMI_SERVICE_STARTUP],
        Pmi_ProcessRxData,
        NULL
    },
	{
		PMI_SERVICE_CONFIG,
        0u,
		PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
		NULL,
        NULL,
        NULL,
        NULL,
		0u,
		0u,
        0u,
        &zPmiServiceStats[PMI_SERVICE_CONFIG],
        Pmi_ProcessRxData,
        NULL
	},
	{
		PMI_SERVICE_PACK_CURRENT,
        0u,
		PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zPmiPackCurrentRead,
		(void*) zPmiPackCurrentResult,
        NULL,
        NULL,
		(TIBMS_MAX_AFES_IN_IFACE_1 * PMI_PCURRENT_READ_BLOCK_LEN),
		PMI_PCURRENT_READ_BLOCK_LEN,
        (PMI_PCURRENT_READ_BLOCK_LEN - PMI_SGL_RSP_FIXED_LEN),
        &zPmiServiceStats[PMI_SERVICE_PACK_CURRENT],
        Pmi_ProcessRxData,
        NULL
	},
	{
		PMI_SERVICE_VF_VOLT,
        0u,
		PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zPmiVfVoltRead,
        (void *) zPmiVfVoltResult,
        NULL,
        NULL,
		(TIBMS_MAX_AFES_IN_IFACE_1 * PMI_VF_READ_BLOCK_LEN),
		PMI_VF_READ_BLOCK_LEN,
        (PMI_VF_READ_BLOCK_LEN - PMI_SGL_RSP_FIXED_LEN),
        &zPmiServiceStats[PMI_SERVICE_VF_VOLT],
        Pmi_ProcessRxData,
        NULL
	},
	{
		PMI_SERVICE_CP_VOLT,
        0u,
		PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zPmiCpVoltRead,
        (void *) zPmiCpVoltResult,
        NULL,
        NULL,
		(TIBMS_MAX_AFES_IN_IFACE_1 * PMI_CP_VOLT_READ_BLOCK_LEN),
		PMI_CP_VOLT_READ_BLOCK_LEN,
        (PMI_CP_VOLT_READ_BLOCK_LEN - PMI_SGL_RSP_FIXED_LEN),
        &zPmiServiceStats[PMI_SERVICE_CP_VOLT],
        Pmi_ProcessRxData,
        NULL
	},
	{
		PMI_SERVICE_CC_ACC_CNT,
        0u,
		PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zPmiCcAccCntRead,
        (void *) zPmiCcAccCntResult,
        NULL,
        NULL,
		(TIBMS_MAX_AFES_IN_IFACE_1 * PMI_CC_ACC_CNT_READ_BLOCK_LEN),
		PMI_CC_ACC_CNT_READ_BLOCK_LEN,
        (PMI_CC_ACC_CNT_READ_BLOCK_LEN - PMI_SGL_RSP_FIXED_LEN),
        &zPmiServiceStats[PMI_SERVICE_CC_ACC_CNT],
        Pmi_ProcessRxData,
        NULL
	},
	{
		PMI_SERVICE_VGPIO,
        0u,
		PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zPmiVGpioRead,
        (void *) zPmiVGpioRsult,
        NULL,
        NULL,
		(TIBMS_MAX_AFES_IN_IFACE_1 * PMI_VGPIO_READ_BLOCK_LEN),
		PMI_VGPIO_READ_BLOCK_LEN,
        (PMI_VGPIO_READ_BLOCK_LEN - PMI_SGL_RSP_FIXED_LEN),
        &zPmiServiceStats[PMI_SERVICE_VGPIO],
        Pmi_ProcessRxData,
        NULL
	},
    {
        PMI_SERVICE_DIETEMP,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zPmiDieTempRead,
        (void *) zPmiDieTempResult,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * PMI_DIETEMP_READ_BLOCK_LEN),
        PMI_DIETEMP_READ_BLOCK_LEN,
        (PMI_DIETEMP_READ_BLOCK_LEN - PMI_SGL_RSP_FIXED_LEN),
        &zPmiServiceStats[PMI_SERVICE_DIETEMP],
        Pmi_ProcessRxData,
        NULL
    },
    {
        PMI_SERVICE_GET_CCOVF,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zPmiDieTempRead,
        (void *) zPmiDieTempResult,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * PMI_1BYTE_READ_BLOCK_LEN),
        PMI_1BYTE_READ_BLOCK_LEN,
        (PMI_1BYTE_READ_BLOCK_LEN - PMI_SGL_RSP_FIXED_LEN),
        &zPmiServiceStats[PMI_SERVICE_GET_CCOVF],
        Pmi_ProcessRxData,
        NULL
    },
    {
        PMI_SERVICE_GET_I2C_DATA,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zPmiDieTempRead,
        (void *) zPmiDieTempResult,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * PMI_1BYTE_READ_BLOCK_LEN),
        PMI_1BYTE_READ_BLOCK_LEN,
        (PMI_1BYTE_READ_BLOCK_LEN - PMI_SGL_RSP_FIXED_LEN),
        &zPmiServiceStats[PMI_SERVICE_GET_I2C_DATA],
        Pmi_ProcessRxData,
        NULL
    },
		{
		PMI_SERVICE_RESET,
        0u,
		PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zPmiPackCurrentRead,
		(void*) zPmiPackCurrentResult,
        NULL,
        NULL,
		8,
		8,
        6,
        &zPmiServiceStats[PMI_SERVICE_RESET],
        Pmi_ProcessRxData,
        NULL
	},
	{
		PMI_SERVICE_DIAGNOSTICS,
        0u,
		PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        NULL,
        (void *) &zPmiDiagResult_0[0], 
        (void *) &zPmiDiagCustResult_0[0], 
        (void *) &zPmiDiagCustStat_0, 
		8,
		8,
        6,
        &zPmiServiceStats[PMI_SERVICE_DIAGNOSTICS],
        Pmi_ProcessRxData,
        NULL
	},
	{
	    PMI_SERVICE_CC_CLEAR,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        &zPmiCcClearResult,
        NULL,
        NULL,
        NULL,
        8,
        8,
        6,
        &zPmiServiceStats[PMI_SERVICE_CC_CLEAR],
        Pmi_ProcessRxData,
        NULL
	},
	{
	    PMI_SERVICE_SET_I2C_DATA,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        &zPmiSetI2CDataResult,
        NULL,
        NULL,
        NULL,
        8,
        8,
        6,
        &zPmiServiceStats[PMI_SERVICE_SET_I2C_DATA],
        Pmi_ProcessRxData,
        NULL
    },
    {
        PMI_SERVICE_SET_I2C_CTRL,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        &zPmiSetI2CCtrlResult,
        NULL,
        NULL,
        NULL,
        8,
        8,
        6,
        &zPmiServiceStats[PMI_SERVICE_SET_I2C_CTRL],
        Pmi_ProcessRxData,
        NULL
    },
    {
        PMI_SERVICE_ENABLE_I2C,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        &zPmiSetI2CCtrlResult,
        (void *)&zPmiEnableI2CRead,
        NULL,
        NULL,
        8,
        8,
        6,
        &zPmiServiceStats[PMI_SERVICE_ENABLE_I2C],
        Pmi_ProcessRxData,
        NULL
    },
    {
        PMI_SERVICE_WRITE_SPI_DATA,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        &zPmiWriteSPIDataResult,
        NULL,
        NULL,
        NULL,
        10,
        10,
        8,
        &zPmiServiceStats[PMI_SERVICE_WRITE_SPI_DATA],
        Pmi_ProcessRxData,
        NULL
    },
    {
        PMI_SERVICE_SET_SPI_EXE,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        &zPmiSetSPIExeResult,
        NULL,
        NULL,
        NULL,
        10,
        10,
        8,
        &zPmiServiceStats[PMI_SERVICE_SET_SPI_EXE],
        Pmi_ProcessRxData,
        NULL
    },
    {
        PMI_SERVICE_SET_SPI_CTRL,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        &zPmiSetSPICtrlResult,
        NULL,
        NULL,
        NULL,
        8,
        8,
        6,
        &zPmiServiceStats[PMI_SERVICE_SET_SPI_CTRL],
        Pmi_ProcessRxData,
        NULL
    },
    {
        PMI_SERVICE_MSPI_CONF,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        &zPmiMSPIConfResult,
        NULL,
        NULL,
        NULL,
        8,
        8,
        6,
        &zPmiServiceStats[PMI_SERVICE_MSPI_CONF],
        Pmi_ProcessRxData,
        NULL
    },
    {
        PMI_SERVICE_EN_ADC_FREEZE,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        &zPmiEnADCFreezeResult,
        NULL,
        NULL,
        NULL,
        8,
        8,
        6,
        &zPmiServiceStats[PMI_SERVICE_EN_ADC_FREEZE],
        Pmi_ProcessRxData,
        NULL
     },
     {
        PMI_SERVICE_DIS_ADC_FREEZE,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        &zPmiEnADCFreezeResult,
        NULL,
        NULL,
        NULL,
        8,
        8,
        6,
        &zPmiServiceStats[PMI_SERVICE_DIS_ADC_FREEZE],
        Pmi_ProcessRxData,
        NULL
     },
     {
        PMI_SERVICE_DIS_ADC_FREEZE,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        &zPmiDisADCFreezeResult,
        NULL,
        NULL,
        NULL,
        8,
        8,
        6,
        &zPmiServiceStats[PMI_SERVICE_DIS_ADC_FREEZE],
        Pmi_ProcessRxData,
        NULL
     },
     {
        PMI_SERVICE_SET_ADC_DELAY,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        &zPmiSetADCDelayResult,
        NULL,
        NULL,
        NULL,
        8,
        8,
        6,
        &zPmiServiceStats[PMI_SERVICE_SET_ADC_DELAY],
        Pmi_ProcessRxData,
        NULL
     },
     {
        PMI_SERVICE_ENTER_COMM_DEBUG,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        &zPmiEnterCommDebugResult,
        NULL,
        NULL,
        NULL,
        8,
        8,
        6,
        &zPmiServiceStats[PMI_SERVICE_ENTER_COMM_DEBUG],
        Pmi_ProcessRxData,
        NULL
     },
     {
        PMI_SERVICE_SET_OC_OUTPUT,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        &zPmiSetOCOutputResult,
        NULL,
        NULL,
        NULL,
        8,
        8,
        6,
        &zPmiServiceStats[PMI_SERVICE_SET_OC_OUTPUT],
        Pmi_ProcessRxData,
        NULL
     },
     {
        PMI_SERVICE_COMMUNICATION,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        &zPmiSetADCDelayResult,
        NULL,
        NULL,
        NULL,
        8,
        8,
        6,
        &zPmiServiceStats[PMI_SERVICE_COMMUNICATION],
        Pmi_ProcessRxData,
        NULL
     },
     {
        PMI_SERVICE_GET_VRF_CAP,
        0u,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        &zPmiSetADCDelayResult,
        NULL,
        NULL,
        NULL,
        8,
        8,
        6,
        &zPmiServiceStats[PMI_SERVICE_GET_VRF_CAP],
        Pmi_ProcessRxData,
        NULL
     }
};

CONST(ServiceCfgType, TIBMSCFG_CONST) zPmiComCfg_0[] =
{
    {
        PMI_SERVICE_COMMUNICATION,
        0u,
        PMI_IFACE_0,
        0u,
        NULL,
        NULL,
        NULL,
        NULL,
        0u,
        0u,
        0u,
        &zPmiCommStats,
        Pmi_ProcessRxData,
        NULL
    },
};

CONST(ServiceCfgType, TIBMSCFG_CONST) zPmiDiagCfg_1[] =
{
	{/* Element '0'  in PMI_DiagServiceType */
		PMI_SERVICE_DIAGNOSTICS,
        PMI_MPFDI_STARTUPINIT,
		PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
		(void *) zPmiGenericRespData_1,
		NULL,
		NULL,
		NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_MPFDI_STARTUPINIT],
        Pmi_ProcessRxData,
        Bq7973x_DiagInitComplete
	},
    { /* Element '1'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_MPFDI_BG1_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_MPFDI_BG1_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '2'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_MPFDI_BG2_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_MPFDI_BG2_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '3'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_MPFDI_ACOMP_FLTINJ_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+3u)),
        (6u+3u),
        3u,
        &zPmiDiagStatus_1[PMI_MPFDI_ACOMP_FLTINJ_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '4'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_MPFDI_DCOMP_FLTINJ_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+3u)),
        (6u+3u),
        3u,
        &zPmiDiagStatus_1[PMI_MPFDI_DCOMP_FLTINJ_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '5'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_MPFDI_OC_UNLOCK_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+1u)),
        (6u+1u),
        1u,
        &zPmiDiagStatus_1[PMI_MPFDI_OC_UNLOCK_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '6'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_MPFDI_OC_DET_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+1u)),
        (6u+1u),
        1u,
        &zPmiDiagStatus_1[PMI_MPFDI_OC_DET_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '7'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_MPFDI_SW_ADJ_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+30u)),
        (6u+30u),
        30u,
        &zPmiDiagStatus_1[PMI_MPFDI_SW_ADJ_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '8'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_MPFDI_REFCAP_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_MPFDI_REFCAP_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '9'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_MPFDI_SW_OTP_PROG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+1u)),
        (6u+1u),
        1u,
        &zPmiDiagStatus_1[PMI_MPFDI_SW_OTP_PROG],
        NULL
    },
    { /* Element '10'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_MPFDI_VIF_CRC_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_MPFDI_VIF_CRC_DIAG],
        NULL
    },
    { /* Element '11'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_MPFDI_SLEEP_FAULT_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_MPFDI_SLEEP_FAULT_DIAG],
        NULL
    },
    { /* Element '12'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_MPFDI_FACT_CRC_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+1u)),
        (6u+1u),
        1u,
        &zPmiDiagStatus_1[PMI_MPFDI_FACT_CRC_DIAG],
        NULL
        },
    { /* Element '13'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_MPFDI_PWR_BIST_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_MPFDI_PWR_BIST_DIAG],
        NULL
    },
    { /* Element '14'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_MFDI_FLIP_RESET_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+30u)),
        (6u+30u),
        30u,
        &zPmiDiagStatus_1[PMI_MFDI_FLIP_RESET_DIAG],
        NULL
    },
    { /* Element '15'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_CYCLE_INIT,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+1u)),
        (6u+1u),
        1u,
        &zPmiDiagStatus_1[PMI_FDTI_CYCLE_INIT],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '16'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_ACOMP_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_FDTI_ACOMP_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '17'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_DCOMP_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+3u)),
        (6u+3u),
        3u,
        &zPmiDiagStatus_1[PMI_FDTI_DCOMP_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '18'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_GP_PLAU_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+30u)),
        (6u+30u),
        30u,
        &zPmiDiagStatus_1[PMI_FDTI_GP_PLAU_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '19'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_FAULT_ADC_MISC,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+1u)),
        (6u+1u),
        1u,
        &zPmiDiagStatus_1[PMI_FDTI_FAULT_ADC_MISC],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '20'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_GPIO_OPNWR_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+30u)),
        (6u+30u),
        30u,
        &zPmiDiagStatus_1[PMI_FDTI_GPIO_OPNWR_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '21'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_GPIO_ADJSHRT,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+30u)),
        (6u+30u),
        30u,
        &zPmiDiagStatus_1[PMI_FDTI_GPIO_ADJSHRT],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '22'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_UPDATE_CUST_CRC,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_FDTI_UPDATE_CUST_CRC],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '19'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_FLT_OTC_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_FDTI_FLT_OTC_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '20'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_CS_PLAU_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+6u)),
        (6u+6u),
        6u,
        &zPmiDiagStatus_1[PMI_FDTI_CS_PLAU_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '21'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_FAULT_PWR_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_FDTI_FAULT_PWR_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '22'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_FAULT_COMM_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_FDTI_FAULT_COMM_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '23'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_SW_MON_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+30u)),
        (6u+30u),
        30u,
        &zPmiDiagStatus_1[PMI_FDTI_SW_MON_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '24'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_OC_PLAU_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+4u)),
        (6u+4u),
        4u,
        &zPmiDiagStatus_1[PMI_FDTI_OC_PLAU_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '25'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_COMM_FLT_MSK_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+3u)),
        (6u+3u),
        3u,
        &zPmiDiagStatus_1[PMI_FDTI_COMM_FLT_MSK_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '26'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_NFAULT,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+1u)),
        (6u+1u),
        1u,
        &zPmiDiagStatus_1[PMI_FDTI_NFAULT],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '27'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_DIETMEP_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+4u)),
        (6u+4u),
        4u,
        &zPmiDiagStatus_1[PMI_FDTI_DIETMEP_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '28'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_FACTORY_CRC_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_FDTI_FACTORY_CRC_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '29'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_COMM_DEBUG_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+3u)),
        (6u+3u),
        3u,
        &zPmiDiagStatus_1[PMI_FDTI_COMM_DEBUG_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '30'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_FAULT_SYS_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_FDTI_FAULT_SYS_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '31'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_FAULT_OTP_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+1u)),
        (6u+1u),
        1u,
        &zPmiDiagStatus_1[PMI_FDTI_FAULT_OTP_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '32'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_OTP_STATE_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+1u)),
        (6u+1u),
        1u,
        &zPmiDiagStatus_1[PMI_FDTI_OTP_STATE_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '33'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_VF_OPEN_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+30u)),
        (6u+30u),
        30u,
        &zPmiDiagStatus_1[PMI_FDTI_VF_OPEN_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '34'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_VF_PLAU_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+4u)),
        (6u+4u),
        4u,
        &zPmiDiagStatus_1[PMI_FDTI_VF_PLAU_DIAG],
        Pmi_ProcessRxData,
        NULL
    },

    { /* Element '35'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_GET_CS1_VREF,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_FDTI_GET_CS1_VREF],
        NULL
    },    
    { /* Element '36'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_INSULAT_DET,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+30u)),
        (6u+30u),
        30u,
        &zPmiDiagStatus_1[PMI_FDTI_INSULAT_DET],
        NULL
    },
    { /* Element '37'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_READ_SPI_DATA,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+4u)),
        (6u+4u),
        4u,
        &zPmiDiagStatus_1[PMI_FDTI_READ_SPI_DATA],
        NULL
    },
    { /* Element '38'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_SET_VISYNC,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_FDTI_SET_VISYNC],
        NULL
    },
    { /* Element '39'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_GET_VAVDD,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_FDTI_GET_VAVDD],
        NULL
    },

    { /* Element '40'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_DRY_VF_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+1u)),
        (6u+1u),
        1u,
        &zPmiDiagStatus_1[PMI_FDTI_DRY_VF_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '41'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_DRY_GPIO_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+1u)),
        (6u+1u),
        1u,
        &zPmiDiagStatus_1[PMI_FDTI_DRY_GPIO_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '42'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_FACT_TM_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+1u)),
        (6u+1u),
        1u,
        &zPmiDiagStatus_1[PMI_FDTI_FACT_TM_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '43'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_OTP_MARGIN_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+1u)),
        (6u+1u),
        1u,
        &zPmiDiagStatus_1[PMI_FDTI_OTP_MARGIN_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '44'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_DRY_DIG_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+1u)),
        (6u+1u),
        1u,
        &zPmiDiagStatus_1[PMI_FDTI_DRY_DIG_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '45'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_DRY_DIG_D1_D2_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+1u)),
        (6u+1u),
        1u,
        &zPmiDiagStatus_1[PMI_FDTI_DRY_DIG_D1_D2_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '46'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_DRY_CS_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+1u)),
        (6u+1u),
        1u,
        &zPmiDiagStatus_1[PMI_FDTI_DRY_CS_DIAG],
        Pmi_ProcessRxData,
        NULL
    },
    { /* Element '47'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_DRDY_ANA_VF_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_FDTI_DRDY_ANA_VF_DIAG],
        NULL
    }, 
    { /* Element '48'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_DRDY_ANA_GPIO_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_FDTI_DRDY_ANA_GPIO_DIAG],
        NULL
    }, 
    { /* Element '49'  in PMI_DiagServiceType */
        PMI_SERVICE_DIAGNOSTICS,
        PMI_FDTI_DRDY_VF_DIAG,
        PMI_IFACE_0,
        PMI_SGL_RSP_PREFIX_LEN,
        (void *) zPmiGenericRespData_1,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_1 * (6u+2u)),
        (6u+2u),
        2u,
        &zPmiDiagStatus_1[PMI_FDTI_DRDY_VF_DIAG],
        NULL
    }
};


CONST(Basic_ConfigType, TIBMSCFG_CONST) zPmiBswCfg_0 =
{
    BasicCfgInit_0,
    TransferRequest_0,
    TransferCtrl_0,
    TimerRequest_0,
    DioRequest_0,
    ResourceRequest_0,
    TimestampRequest_0,
    BasicCfgDeInit_0,

    ResID_PmiIface_0,
    ResID_App_1,
    ResID_DiagApp_1,
};

/*********************************************************************************************************************
 * External Functions Definition
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * End of File: tibms_Lcfg_0.c
 *********************************************************************************************************************/
