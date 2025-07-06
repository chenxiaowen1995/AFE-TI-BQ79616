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
 *  Description:  Linktime configuration for TI BMS interface 0
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

/** Major Software Config C Version number */
#define TIBMS_LCFG_C_MAJOR_VERSION             (0x01u)

/** Minor Software Config C Version number */
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

#define BMI_SGL_RSP_PREFIX_LEN                  (BQ79600_RSP_FRAME_PREFIX_LEN) //4
#define BMI_SGL_RSP_FIXED_LEN                   (BQ79600_REQ_FRAME_FIXED_LEN) //6
#define BMI_DEVADDR_READ_BLOCK_LEN_0            (BMI_SGL_RSP_FIXED_LEN + 1u)
#define BMI_VCELL_READ_BLOCK_LEN_0              (BMI_SGL_RSP_FIXED_LEN + (BQ7971X_CELL_NUM_ACT * 2u))
#define BMI_GPIO_READ_BLOCK_LEN_0               (BMI_SGL_RSP_FIXED_LEN + (BQ7971X_GPIO_NUM_MAX * 2u))
#define BMI_VBAT_READ_BLOCK_LEN_0               (BMI_SGL_RSP_FIXED_LEN +  2u)
#define BMI_VCAPREF_READ_BLOCK_LEN_0            (BMI_SGL_RSP_FIXED_LEN +  2u)
/**************** 8.2 **************************/
#define BMI_SPIData_READ_BLOCK_LEN_0            (BMI_SGL_RSP_FIXED_LEN +  3u)
#define BMI_SPIEnb_READ_BLOCK_LEN_0             (BMI_SGL_RSP_FIXED_LEN + 1u)
#define BMI_I2CData_READ_BLOCK_LEN_0            (BMI_SGL_RSP_FIXED_LEN + 1u)
#define BMI_I2CFault_READ_BLOCK_LEN_0           (BMI_SGL_RSP_FIXED_LEN + 1u)
#define BMI_DIAG_READ_BLOCK_LEN_0               (BMI_SGL_RSP_FIXED_LEN + 3u)
#define BMI_VCELL_ACTSUM_BLOCK_LEN_0            (BMI_SGL_RSP_FIXED_LEN + 2u)
#define BMI_GET_DIE_TEMP_BLOCK_LEN_0            (BMI_SGL_RSP_FIXED_LEN + 4u)


#define BMI_CELLBAL_GETTIME_BLOCK_LEN_0         (BMI_SGL_RSP_FIXED_LEN + BQ7971X_CELL_NUM_ACT)
#define BMI_CELLBAL_GETDONESTAT_BLOCK_LEN_0     (BMI_SGL_RSP_FIXED_LEN + 3u)
#define BMI_CELLBAL_GETSWSTAT_BLOCK_LEN_0       (BMI_SGL_RSP_FIXED_LEN + 3u)
#define BMI_CELLBAL_GETSTAT_BLOCK_LEN_0         (BMI_SGL_RSP_FIXED_LEN + 1u)

#define BMI_GENERIC_DATA_BUF_LEN_0              (BMI_SGL_RSP_FIXED_LEN + 10u)
#define PMI_GENERIC_DATA_BUF_LEN_0              (BMI_SGL_RSP_FIXED_LEN + 10u)
/*********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

#define TIBMS_LCFG_0_START_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"



static ServiceStatusType zPmiServiceStats[PMI_SERVICE_MAX];
/** Functional  Services **/
ServiceStatusType zBmiServiceStatus_0[BMI_SERVICE_MAX];

uint8 zBmiVBatBuf_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_VBAT_READ_BLOCK_LEN_0];
uint8 zBmiVRefCapBuf_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_VCAPREF_READ_BLOCK_LEN_0];
uint8 zBmiVCellDataBuf_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_VCELL_READ_BLOCK_LEN_0];

/********************************* 8.2 ***********************/
uint8 zBmiSPIDataBuf_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_SPIData_READ_BLOCK_LEN_0];
uint8 zBmiI2CEnbBuf_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_SPIEnb_READ_BLOCK_LEN_0];
uint8 zBmiI2CDataBuf_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_I2CData_READ_BLOCK_LEN_0];
uint8 zBmiI2CFaultBuf_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_I2CFault_READ_BLOCK_LEN_0];
uint8 zBmiDiagMainBuf_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_DIAG_READ_BLOCK_LEN_0];
uint8 zBmiDiagRdntBuf_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_DIAG_READ_BLOCK_LEN_0];
uint8 zBmiVCellActBuf_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_VCELL_ACTSUM_BLOCK_LEN_0];
uint8 zBmiDieTempBuf_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_GET_DIE_TEMP_BLOCK_LEN_0];

uint8 zBmiGpioDataBuf_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_GPIO_READ_BLOCK_LEN_0];
uint8 zBmiGpioDataBuf_1[TIBMS_MAX_AFES_IN_IFACE_0][BMI_GPIO_READ_BLOCK_LEN_0];
uint8 zBmiGenericRespData_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_GENERIC_DATA_BUF_LEN_0];
uint8 zPmiGenericRespData_0[TIBMS_MAX_AFES_IN_IFACE_0][PMI_GENERIC_DATA_BUF_LEN_0];


ServiceStatusType zBmiCellBalStatus_0[BMI_CB_RESP_END];
uint8 zBmiCellBalGetTimeData_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_CELLBAL_GETTIME_BLOCK_LEN_0];
uint8 zBmiCellBalDoneStatData_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_CELLBAL_GETDONESTAT_BLOCK_LEN_0];
uint8 zBmiCellBalSWStatData_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_CELLBAL_GETSWSTAT_BLOCK_LEN_0];
uint8 zBmiCellBalStatData_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_CELLBAL_GETSTAT_BLOCK_LEN_0];
Bmi_uCbTime zBmiCBDataBuf_0;

/** Diagnostics **/
ServiceStatusType zBmiDiagStatus_0[BMI_DIAGSERVICE_END];
uint16 zBmiDiagVCellRead1_0[TIBMS_MAX_AFES_IN_IFACE_0][BQ7971X_CELL_NUM_ACT];
uint16 zBmiDiagVCellRead2_0[TIBMS_MAX_AFES_IN_IFACE_0][BQ7971X_CELL_NUM_ACT];
uint16 zBmiDiagVCellOpenPlau_0[TIBMS_MAX_AFES_IN_IFACE_0][BQ7971X_CELL_NUM_ACT];
uint16 zBmiDiagGpioRead1_0[TIBMS_MAX_AFES_IN_IFACE_0][BQ7971X_GPIO_NUM_MAX];
uint16 zBmiDiagGpioRead2_0[TIBMS_MAX_AFES_IN_IFACE_0][BQ7971X_GPIO_NUM_MAX];

DiagResultType zBmiDiagResult_0[TIBMS_MAX_AFES_IN_IFACE_0];

/** Customer Data Buffer */
uint16 zBmiVBatResultBuf_0[TIBMS_MAX_AFES_IN_IFACE_0];
uint16 zBmiVRefCapResultBuf_0 [TIBMS_MAX_AFES_IN_IFACE_0];
uint16 zBmiVCellDataResultBuf_0[TIBMS_MAX_AFES_IN_IFACE_0][BQ7971X_CELL_NUM_ACT];
/********************************* 8.2 ***********************/
uint16 zBmiSPIDataResultBuf_0[TIBMS_MAX_AFES_IN_IFACE_0];
uint16 zBmiI2CEnbResultBuf_0[TIBMS_MAX_AFES_IN_IFACE_0];
uint16 zBmiI2CDataResultBuf_0[TIBMS_MAX_AFES_IN_IFACE_0];
uint16 zBmiI2CFaultResultBuf_0[TIBMS_MAX_AFES_IN_IFACE_0];
uint16 zBmiDiagMainResultBuf_0[TIBMS_MAX_AFES_IN_IFACE_0];
uint16 zBmiDiagRdntResultBuf_0[TIBMS_MAX_AFES_IN_IFACE_0];
uint16 zBmiVcellActResultBuf_0[TIBMS_MAX_AFES_IN_IFACE_0];
uint16 zBmiDieTempResultBuf_0[TIBMS_MAX_AFES_IN_IFACE_0];

uint16 zBmiGpioResultBuf_0[TIBMS_MAX_AFES_IN_IFACE_0][BQ7971X_GPIO_NUM_MAX];

uint8 zBmiCellBalGetTimeResult_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_CELLBAL_GETTIME_BLOCK_LEN_0-BMI_SGL_RSP_FIXED_LEN];
uint8 zBmiCellBalDoneStatResult_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_CELLBAL_GETDONESTAT_BLOCK_LEN_0-BMI_SGL_RSP_FIXED_LEN];
uint8 zBmiCellBalSWStatResult_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_CELLBAL_GETSWSTAT_BLOCK_LEN_0-BMI_SGL_RSP_FIXED_LEN];
uint8 zBmiCellBalStatResult_0[TIBMS_MAX_AFES_IN_IFACE_0][BMI_CELLBAL_GETSTAT_BLOCK_LEN_0-BMI_SGL_RSP_FIXED_LEN];

DiagResultType zBmiDiagCustResult_0[TIBMS_MAX_AFES_IN_IFACE_0];
DiagStatusType zBmiDiagCustStat_0;


#define PMI_SGL_RSP_PREFIX_LEN                  (BQ7973X_RSP_FRAME_PREFIX_LEN)
#define PMI_SGL_RSP_FIXED_LEN                   (BQ7973X_REQ_FRAME_FIXED_LEN)
#define TIBMS_LCFG_0_STOP_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

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

STATIC FUNC(uint32, tibms_Lcfg_CODE) TransferRequest_0(void *pTxBuf, uint16 wTxLength, void *pRxBuf, uint16 wRxLength)
{
    uint32 uRet = E_NOT_OK;

#if(BMI_UART_ENABLE == 1)
#if (CONFIG_CPU_FAMILY == CPU_FAMILY_TI_TMS570)
    if((pRxBuf) && (wRxLength > 0))
    {
        sciReceive(sciREG, wRxLength, pRxBuf);
    }

    if( (wTxLength > 0))
    {
        sciSend(sciREG, wTxLength, pTxBuf);

    }
    uRet = E_OK;
#elif (CONFIG_CPU_FAMILY == CPU_FAMILY_TI_SITARA)
    uRet = UartTransfer(pTxBuf, wTxLength, pRxBuf, wRxLength);
#endif

#else
   if(wTxLength > 0)
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
        if(E_OK == Spi_SetupEB(Spi_SpiChannel_1, NULL, pRxBuf, wRxLength))
        {
            if(SPI_SEQ_PENDING == Spi_GetSequenceResult(Spi_SpiSequence_1))
            {
                Spi_Cancel(Spi_SpiSequence_1);
            }
            uRet = Spi_AsyncTransmit(Spi_SpiSequence_1);
        }
    }
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
        case CANCEL_TX_REQ:
        {
            Spi_Cancel(Spi_SpiSequence_0);
            qRet = E_OK;
            break;
        }
        case CANCEL_RX_REQ:
        {
            Spi_Cancel(Spi_SpiSequence_1);
            qRet = E_OK;
            break;
        }
        case NOTIFY_TX_REQ:
        {
            SetEvent(TaskId_BmiIface_0, EVENT_BMI_DRIVER_TRANSFER_REQ_0);
            qRet = E_OK;
            break;
        }
        case NOTIFY_RX_REQ:
        {
            SetEvent(TaskId_BmiIface_0, EVENT_BMI_DRIVER_RECV_COMPLETE_0);
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

    if(qTimeMs > 0)
    {
        SetRelAlarm(AlarmID_BmiIface_0, qTimeMs, 0);
        qRet = E_OK;
    }
    else
    {
        CancelAlarm(AlarmID_BmiIface_0);
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
 * uint32 Bsw_ResourceRequest_0(uint32 qResId, uint32 qCtrlReq)
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
 * FUNC(uint32, tibms_Lcfg_CODE) Bsw_TimestampRequest_0(uint32 *Value, uint32 *ElapsedValue)
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
 * uint32 Bsw_CfgInit_0(void)
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
#if (BMI_UART_ENABLE == 1)
    sciInit();
    qRet = E_OK;
#endif

    return qRet;
}

/**********************************************************************************************************************
 * uint32 Bsw_CfgDeInit_0(void)
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
#if (BMI_UART_ENABLE == 1)
    scideinit(sciREG);
    qRet = E_OK;
#endif
    // TBD:
    return qRet;
}

/*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

#define TIBMS_LCFG_0_START_SEC_CONST
#include "Cdd_MemMap.h"

CONST(Basic_ConfigType, TIBMSCFG_CONST) zBmiBswCfg_0 =
{
    BasicCfgInit_0,
    TransferRequest_0,
    TransferCtrl_0,
    TimerRequest_0,
    DioRequest_0,
    ResourceRequest_0,
    TimestampRequest_0,
    BasicCfgDeInit_0,

    ResID_BmiIface_0,
    ResID_App_0,
    ResID_DiagApp_0,
};

CONST(ServiceCfgType, TIBMSCFG_CONST) zBmiFuncCfg_0[BMI_SERVICE_MAX] =
{
    {
        BMI_SERVICE_CONFIG,
        0u,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_DEVS_IN_IFACE_0 * BMI_DEVADDR_READ_BLOCK_LEN_0),
        BMI_DEVADDR_READ_BLOCK_LEN_0,
        (BMI_DEVADDR_READ_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
        &zBmiServiceStatus_0[BMI_SERVICE_CONFIG],
        Bmi_ProcessRxData,
        NULL
    },
    {
        BMI_SERVICE_BALANCING,
        0u,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        NULL,
        (uint8 *) zBmiCBDataBuf_0,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * BQ7971X_CELL_NUM_ACT),
        BQ7971X_CELL_NUM_ACT,
        BQ7971X_CELL_NUM_ACT,
        &zBmiServiceStatus_0[BMI_SERVICE_BALANCING],
        Bmi_ProcessRxData,
        NULL
    },
    {
        BMI_SERVICE_CELL_VOLT,
        0u,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zBmiVCellDataBuf_0,
        (void *) zBmiVCellDataResultBuf_0,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_VCELL_READ_BLOCK_LEN_0),
        BMI_VCELL_READ_BLOCK_LEN_0,
        (BMI_VCELL_READ_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
        &zBmiServiceStatus_0[BMI_SERVICE_CELL_VOLT],
        Bmi_ProcessRxData,
        NULL
    },
    {
        BMI_SERVICE_GPIO_READ,
        0u,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGpioDataBuf_0,
        (void *) zBmiGpioResultBuf_0,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_GPIO_READ_BLOCK_LEN_0),
        BMI_GPIO_READ_BLOCK_LEN_0,
        (BMI_GPIO_READ_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
        &zBmiServiceStatus_0[BMI_SERVICE_GPIO_READ],
        Bmi_ProcessRxData,
        NULL
    },
    {
        BMI_SERVICE_DIAGNOSTICS,
        0u,
        BMI_IFACE_0,
        0u,
        NULL,
        (void *) &zBmiDiagResult_0[0],
        (void *) &zBmiDiagCustResult_0[0],
        (void *) &zBmiDiagCustStat_0,
        0u,
        0u,
        0u,
        &zBmiServiceStatus_0[BMI_SERVICE_DIAGNOSTICS],
        Bmi_ProcessRxData,
        NULL
    },
    {
        BMI_SERVICE_COMMUNICATION,
        0u,
        BMI_IFACE_0,
        0u,
        NULL,
        NULL,
        NULL,
        NULL,
        0u,
        0u,
        0u,
        &zBmiServiceStatus_0[BMI_SERVICE_COMMUNICATION],
        Bmi_ProcessRxData,
        NULL
    },
    {
        BMI_SERVICE_USER,
        0u,
        BMI_IFACE_0,
        0u,
        NULL,
        NULL,
        NULL,
        NULL,
        0u,
        0u,
        0u,
        &zBmiServiceStatus_0[BMI_SERVICE_USER],
        Bmi_ProcessRxData,
        NULL
    },
    {
        BMI_SERVICE_VBAT,
        0u,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zBmiVBatBuf_0,
        (void *) zBmiVBatResultBuf_0,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_VBAT_READ_BLOCK_LEN_0),
        BMI_VBAT_READ_BLOCK_LEN_0,
        (BMI_VBAT_READ_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
        &zBmiServiceStatus_0[BMI_SERVICE_VBAT],
        Bmi_ProcessRxData,
        NULL
     },
     {
        BMI_SERVICE_VCAP_REF,
        0u,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zBmiVRefCapBuf_0,
        (void *) zBmiVRefCapResultBuf_0,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_VCAPREF_READ_BLOCK_LEN_0),
        BMI_VCAPREF_READ_BLOCK_LEN_0,
        (BMI_VCAPREF_READ_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
        &zBmiServiceStatus_0[BMI_SERVICE_VCAP_REF],
        Bmi_ProcessRxData,
        NULL
     },
    {
        BMI_SERVICE_POWER_CONTROL,
        0u,
        BMI_IFACE_0,
        0u,
        NULL,
        NULL,
        NULL,
        NULL,
        0u,
        0u,
        0u,
        &zBmiServiceStatus_0[BMI_SERVICE_POWER_CONTROL],
        Bmi_ProcessRxData,
        NULL
    },
    {
       BMI_SERVICE_RESET,
       0u,
       BMI_IFACE_0,
       0u,
       NULL,
       NULL,
       NULL,
       NULL,
       0u,
       0u,
       0u,
       &zBmiServiceStatus_0[BMI_SERVICE_RESET],
       Bmi_ProcessRxData,
       NULL
   },
   {
    BMI_SERVICE_SET_COMM_CONF,
      0u,
      BMI_IFACE_0,
      0u,
      NULL,
      NULL,
      NULL,
      NULL,
      0u,
      0u,
      0u,
      &zBmiServiceStatus_0[BMI_SERVICE_SET_COMM_CONF],
      Bmi_ProcessRxData,
      NULL
  },
  {
   BMI_SERVICE_SET_BB_CONF,
     0u,
     BMI_IFACE_0,
     0u,
     NULL,
     NULL,
     NULL,
     NULL,
     0u,
     0u,
     0u,
     &zBmiServiceStatus_0[BMI_SERVICE_SET_BB_CONF],
     Bmi_ProcessRxData,
     NULL
     },
     {
      BMI_SERVICE_SET_ENB_SPI,
        0u,
        BMI_IFACE_0,
        0u,
        NULL,
        NULL,
        NULL,
        NULL,
        0u,
        0u,
        0u,
        &zBmiServiceStatus_0[BMI_SERVICE_SET_ENB_SPI],
        Bmi_ProcessRxData,
        NULL
    },
    {
     BMI_SERVICE_SET_SPI_CONF,
       0u,
       BMI_IFACE_0,
       0u,
       NULL,
       NULL,
       NULL,
       NULL,
       0u,
       0u,
       0u,
       &zBmiServiceStatus_0[BMI_SERVICE_SET_SPI_CONF],
       Bmi_ProcessRxData,
       NULL
    },
    {
     BMI_SERVICE_SET_SPI_EXE,
       0u,
       BMI_IFACE_0,
       0u,
       NULL,
       NULL,
       NULL,
       NULL,
       0u,
       0u,
       0u,
       &zBmiServiceStatus_0[BMI_SERVICE_SET_SPI_EXE],
       Bmi_ProcessRxData,
       NULL
    },
    {
     BMI_SERVICE_WRITE_SPI_DATA,
       0u,
       BMI_IFACE_0,
       0u,
       NULL,
       NULL,
       NULL,
       NULL,
       0u,
       0u,
       0u,
       &zBmiServiceStatus_0[BMI_SERVICE_WRITE_SPI_DATA],
       Bmi_ProcessRxData,
       NULL
    },
    {
       BMI_SERVICE_READ_SPI_DATA,
       0u,
       BMI_IFACE_0,
       BMI_SGL_RSP_PREFIX_LEN,
       (uint8 *) zBmiSPIDataBuf_0,
       (void *) zBmiSPIDataResultBuf_0,
       NULL,
       NULL,
       (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_SPIData_READ_BLOCK_LEN_0),
       BMI_SPIData_READ_BLOCK_LEN_0,
       (BMI_SPIData_READ_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
       &zBmiServiceStatus_0[BMI_SERVICE_READ_SPI_DATA],
       Bmi_ProcessRxData,
       NULL
    },
    {
       BMI_SERVICE_SET_ENB_I2C,
       0u,
       BMI_IFACE_0,
       BMI_SGL_RSP_PREFIX_LEN,
       (uint8 *) zBmiI2CEnbBuf_0,
       (void *) zBmiI2CEnbResultBuf_0,
       NULL,
       NULL,
       (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_SPIEnb_READ_BLOCK_LEN_0),
       BMI_SPIEnb_READ_BLOCK_LEN_0,
       (BMI_SPIEnb_READ_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
       &zBmiServiceStatus_0[BMI_SERVICE_SET_ENB_I2C],
       Bmi_ProcessRxData,
       NULL
    },
    {
     BMI_SERVICE_SET_I2C_CTRL,
       0u,
       BMI_IFACE_0,
       0u,
       NULL,
       NULL,
       NULL,
       NULL,
       0u,
       0u,
       0u,
       &zBmiServiceStatus_0[BMI_SERVICE_SET_I2C_CTRL],
       Bmi_ProcessRxData,
       NULL
    },
    {
       BMI_SERVICE_SET_I2C_DATA,
       0u,
       BMI_IFACE_0,
       0u,
       NULL,
       NULL,
       NULL,
       NULL,
       0u,
       0u,
       0u,
       &zBmiServiceStatus_0[BMI_SERVICE_SET_I2C_DATA],
       Bmi_ProcessRxData,
       NULL
    },
    {
       BMI_SERVICE_GET_I2C_DATA,
       0u,
       BMI_IFACE_0,
       BMI_SGL_RSP_PREFIX_LEN,
       (uint8 *) zBmiI2CDataBuf_0,
       (void *) zBmiI2CDataResultBuf_0,
       NULL,
       NULL,
       (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_SPIData_READ_BLOCK_LEN_0),
       BMI_SPIData_READ_BLOCK_LEN_0,
       (BMI_SPIData_READ_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
       &zBmiServiceStatus_0[BMI_SERVICE_GET_I2C_DATA],
       Bmi_ProcessRxData,
       NULL
    },
    {
       BMI_SERVICE_GET_I2C_FAULT,
       0u,
       BMI_IFACE_0,
       BMI_SGL_RSP_PREFIX_LEN,
       (uint8 *) zBmiI2CFaultBuf_0,
       (void *) zBmiI2CFaultResultBuf_0,
       NULL,
       NULL,
       (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_I2CFault_READ_BLOCK_LEN_0),
       BMI_I2CFault_READ_BLOCK_LEN_0,
       (BMI_I2CFault_READ_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
       &zBmiServiceStatus_0[BMI_SERVICE_GET_I2C_FAULT],
       Bmi_ProcessRxData,
       NULL
    },
    {
       BMI_SERVICE_GET_DIAG_RDNT,
       0u,
       BMI_IFACE_0,
       BMI_SGL_RSP_PREFIX_LEN,
       (uint8 *) zBmiDiagRdntBuf_0,
       (void *) zBmiDiagRdntResultBuf_0,
       NULL,
       NULL,
       (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_DIAG_READ_BLOCK_LEN_0),
       BMI_DIAG_READ_BLOCK_LEN_0,
       (BMI_DIAG_READ_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
       &zBmiServiceStatus_0[BMI_SERVICE_GET_DIAG_RDNT],
       Bmi_ProcessRxData,
       NULL
    },
    {
       BMI_SERVICE_GET_DIAG_MAIN,
       0u,
       BMI_IFACE_0,
       BMI_SGL_RSP_PREFIX_LEN,
       (uint8 *) zBmiDiagMainBuf_0,
       (void *) zBmiDiagMainResultBuf_0,
       NULL,
       NULL,
       (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_DIAG_READ_BLOCK_LEN_0),
       BMI_DIAG_READ_BLOCK_LEN_0,
       (BMI_DIAG_READ_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
       &zBmiServiceStatus_0[BMI_SERVICE_GET_DIAG_MAIN],
       Bmi_ProcessRxData,
       NULL
    },
    {
       BMI_SERVICE_SET_GPIO_CONF,
       0u,
         BMI_IFACE_0,
         0u,
         NULL,
         NULL,
         NULL,
         NULL,
         0u,
         0u,
         0u,
       &zBmiServiceStatus_0[BMI_SERVICE_SET_GPIO_CONF],
       Bmi_ProcessRxData,
       NULL
    },
    {
       BMI_SERVICE_SET_GPIO_DOWN,
        0u,
         BMI_IFACE_0,
         0u,
         NULL,
         NULL,
         NULL,
         NULL,
         0u,
         0u,
         0u,
       &zBmiServiceStatus_0[BMI_SERVICE_SET_GPIO_DOWN],
       Bmi_ProcessRxData,
       NULL
    },
    {
      BMI_SERVICE_GET_VCELL_ACTSUM,
      0u,
      BMI_IFACE_0,
      BMI_SGL_RSP_PREFIX_LEN,
      (uint8 *) zBmiVCellActBuf_0,
      (void *) zBmiVcellActResultBuf_0,
      NULL,
      NULL,
      (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_VCELL_ACTSUM_BLOCK_LEN_0),
      BMI_VCELL_ACTSUM_BLOCK_LEN_0,
      (BMI_VCELL_ACTSUM_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
      &zBmiServiceStatus_0[BMI_SERVICE_GET_VCELL_ACTSUM],
      Bmi_ProcessRxData,
      NULL
     },
     {
       BMI_SERVICE_GET_DIE_TEMP,
       0u,
       BMI_IFACE_0,
       BMI_SGL_RSP_PREFIX_LEN,
       (uint8 *) zBmiDieTempBuf_0,
       (void *) zBmiDieTempResultBuf_0,
       NULL,
       NULL,
       (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_GET_DIE_TEMP_BLOCK_LEN_0),
       BMI_GET_DIE_TEMP_BLOCK_LEN_0,
       (BMI_GET_DIE_TEMP_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
       &zBmiServiceStatus_0[BMI_SERVICE_GET_DIE_TEMP],
       Bmi_ProcessRxData,
       NULL
      },
      {
         BMI_SERVICE_EN_ADC_FREEZE,
         0u,
         BMI_IFACE_0,
         0u,
         NULL,
         NULL,
         NULL,
         NULL,
         0u,
         0u,
         0u,
         &zBmiServiceStatus_0[BMI_SERVICE_EN_ADC_FREEZE],
         Bmi_ProcessRxData,
         NULL
        },
        {
         BMI_SERVICE_DIS_ADC_FREEZE,
         0u,
         BMI_IFACE_0,
         0u,
         NULL,
         NULL,
         NULL,
         NULL,
         0u,
         0u,
         0u,
         &zBmiServiceStatus_0[BMI_SERVICE_DIS_ADC_FREEZE],
         Bmi_ProcessRxData,
         NULL
        },
        {
         BMI_SERVICE_SET_ADC_DELAY,
         0u,
         BMI_IFACE_0,
         0u,
         NULL,
         NULL,
         NULL,
         NULL,
         0u,
         0u,
         0u,
         &zBmiServiceStatus_0[BMI_SERVICE_SET_ADC_DELAY],
         Bmi_ProcessRxData,
         NULL
        },
};

CONST(ServiceCfgType, TIBMSCFG_CONST) zBmiCellBalCfg_0[] =
{
    {
        BMI_SERVICE_BALANCING,
        BMI_CB_GET_BALSTAT,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiCellBalStatData_0,
        (void *) zBmiCellBalStatResult_0,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
        (6u+1u),
        1u,
        &zBmiCellBalStatus_0[BMI_CB_GET_BALSTAT],
        Bmi_ProcessRxData,
        NULL
    },
    {
        BMI_SERVICE_BALANCING,
        BMI_CB_GET_BALSWSTAT,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiCellBalSWStatData_0,
        (void *) zBmiCellBalSWStatResult_0,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+3u)),
        (6u+3u),
        3u,
        &zBmiCellBalStatus_0[BMI_CB_GET_BALSWSTAT],
        Bmi_ProcessRxData,
        NULL
    },  {
        BMI_SERVICE_BALANCING,
        BMI_CB_GET_BALDONESTAT,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiCellBalDoneStatData_0,
        (void *) zBmiCellBalDoneStatResult_0,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+3u)),
        (6u+3u),
        3u,
        &zBmiCellBalStatus_0[BMI_CB_GET_BALDONESTAT],
        Bmi_ProcessRxData,
        NULL
    },
    {
        BMI_SERVICE_BALANCING,
        BMI_CB_GET_BALTIME,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiCellBalGetTimeData_0,
        (void *) zBmiCellBalGetTimeResult_0,
        NULL,
        NULL,
        (TIBMS_MAX_BMICS_IN_IFACE_0 * (6u+1)),
        (6u+1u),
        TIBMS_BMIC_CELLS_IFACE_0,
        &zBmiCellBalStatus_0[BMI_CB_GET_BALTIME],
        Bmi_ProcessRxData,
        NULL
    }
};

CONST(ServiceCfgType, TIBMSCFG_CONST) zBmiComCfg_0[] =
{
    {
        BMI_SERVICE_COMMUNICATION,
        0u,
        BMI_IFACE_0,
        0u,
        NULL,
        NULL,
        NULL,
        NULL,
        0u,
        0u,
        0u,
        &zBmiServiceStatus_0[BMI_SERVICE_COMMUNICATION],
        Bmi_ProcessRxData,
        NULL
    },
};
CONST(ServiceCfgType, TIBMSCFG_CONST) zPmiDiagCfg_0[] =
{
 {
          PMI_SERVICE_DIAGNOSTICS,
          PMI_FDTI_SW_MON_DIAG,
          BMI_IFACE_0,
          PMI_SGL_RSP_PREFIX_LEN,
          (void *) zPmiGenericRespData_0,
          NULL,
          NULL,
          NULL,
          (TIBMS_MAX_AFES_IN_IFACE_2 * (6u+2u)),
          (6u+2u),
          2u,
          &zPmiServiceStats[PMI_FDTI_SW_MON_DIAG],
          Pmi_ProcessRxData,
          NULL
        },
        {
          PMI_SERVICE_DIAGNOSTICS,
          PMI_FDTI_OC_PLAU_DIAG,
          PMI_IFACE_0,
          PMI_SGL_RSP_PREFIX_LEN,
          (void *) zPmiGenericRespData_0,
          NULL,
          NULL,
          NULL,
          (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
          (6u+2u),
          2u,
          &zPmiServiceStats[PMI_FDTI_OC_PLAU_DIAG],
          Pmi_ProcessRxData,
          NULL
        },


};
CONST(ServiceCfgType, TIBMSCFG_CONST) zBmiDiagCfg_0[] =
{
    {//0
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_REG_READ_1BYTE,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
        (6u+1u),
        1u,
        &zBmiDiagStatus_0[BMI_SM_REG_READ_1BYTE],
        Bmi_ProcessRxData,
        NULL
    },
    {//1
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_REG_READ_2BYTE,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
        (6u+2u),
        2u,
        &zBmiDiagStatus_0[BMI_SM_REG_READ_2BYTE],
        Bmi_ProcessRxData,
        NULL
    },
    {//2
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_REG_READ_3BYTE,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+3u)),
        (6u+3u),
        3u,
        &zBmiDiagStatus_0[BMI_SM_REG_READ_3BYTE],
        Bmi_ProcessRxData,
        NULL
    },
    {//3
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_REG_READ_4BYTE,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+4u)),
        (6u+4u),
        4u,
        &zBmiDiagStatus_0[BMI_SM_REG_READ_4BYTE],
        Bmi_ProcessRxData,
        NULL
    },
    {//4
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_REG_READ_5BYTE,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+5u)),
        (6u+5u),
        5u,
        &zBmiDiagStatus_0[BMI_SM_REG_READ_5BYTE],
        Bmi_ProcessRxData,
        NULL
    },
    {//5
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_STARTUPINIT,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
        (6u+2u),
        2u,
        &zBmiDiagStatus_0[BMI_SM_MPFDI_STARTUPINIT],
        Bmi_ProcessRxData,
        Bq7971x_DiagInitComplete
    },
    {//6
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_BGX_DIAG,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+4u)),
        (6u+4u),
        4u,
        &zBmiDiagStatus_0[BMI_SM_MPFDI_BGX_DIAG],
        Bmi_ProcessRxData,
        NULL
    },
    {//7
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_REFCAP_DIAG,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
        (6u+2u),
        2u,
        &zBmiDiagStatus_0[BMI_SM_MPFDI_REFCAP_DIAG],
        Bmi_ProcessRxData,
        NULL
    },
    {//8
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_PWR_BIST,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
        (6u+2u),
        2u,
        &zBmiDiagStatus_0[BMI_SM_MPFDI_PWR_BIST],
        Bmi_ProcessRxData,
        NULL
    },
    {//9
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_ACOMP_FLTINJ,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+5u)),
        (6u+5u),
        5u,
        &zBmiDiagStatus_0[BMI_SM_MPFDI_ACOMP_FLTINJ],
        Bmi_ProcessRxData,
        NULL
    },
    {//10
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_DCOMP_FLTINJ,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+3u)),
        (6u+3u),
        3u,
        &zBmiDiagStatus_0[BMI_SM_MPFDI_DCOMP_FLTINJ],
        Bmi_ProcessRxData,
        NULL
    },
    {//11
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_CBFET_DIAG,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+3u)),
        (6u+3u),
        3u,
        &zBmiDiagStatus_0[BMI_SM_MPFDI_CBFET_DIAG],
        Bmi_ProcessRxData,
        NULL
    },
    {//12
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_OV_DAC,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+3u)),
        (6u+3u),
        3u,
        &zBmiDiagStatus_0[BMI_SM_MPFDI_OV_DAC],
        Bmi_ProcessRxData,
        NULL
    },
    {//13
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_UV_DAC,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+3u)),
        (6u+3u),
        3u,
        &zBmiDiagStatus_0[BMI_SM_MPFDI_UV_DAC],
        Bmi_ProcessRxData,
        NULL
    },
    {//14
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_OT_DAC,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+3u)),
        (6u+3u),
        3u,
        &zBmiDiagStatus_0[BMI_SM_MPFDI_OT_DAC],
        Bmi_ProcessRxData,
        NULL
    },
    {//15
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_MPFDI_UT_DAC,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+3u)),
        (6u+3u),
        3u,
        &zBmiDiagStatus_0[BMI_SM_MPFDI_UT_DAC],
        Bmi_ProcessRxData,
        NULL
    },
    {//15
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_CBOW_FLTINJ,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+3u)),
        (6u+3u),
        3u,
        &zBmiDiagStatus_0[BMI_SM_CBOW_FLTINJ],
        Bmi_ProcessRxData,
        NULL
    },
    {//17
       BMI_SERVICE_DIAGNOSTICS,
       BMI_SM_MPFDI_OTP_PROGRAM,
       BMI_IFACE_0,
       BMI_SGL_RSP_PREFIX_LEN,
       (void *) zBmiGenericRespData_0,
       NULL,
       NULL,
       NULL,
       (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
       (6u+1u),
       1u,
       &zBmiDiagStatus_0[BMI_SM_MPFDI_OTP_PROGRAM],
       Bmi_ProcessRxData,
       NULL
      },
      {//18
       BMI_SERVICE_DIAGNOSTICS,
       BMI_SM_MPFDI_OTUT_SC_DIAG,
       BMI_IFACE_0,
       BMI_SGL_RSP_PREFIX_LEN,
       (void *) zBmiGenericRespData_0,
       NULL,
       NULL,
       NULL,
       (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
       (6u+2u),
       2u,
       &zBmiDiagStatus_0[BMI_SM_MPFDI_OTUT_SC_DIAG],
       Bmi_ProcessRxData,
       NULL
      },
      {//19
       BMI_SERVICE_DIAGNOSTICS,
       BMI_SM_MPFDI_OVUV_SC_DIAG,
       BMI_IFACE_0,
       BMI_SGL_RSP_PREFIX_LEN,
       (void *) zBmiGenericRespData_0,
       NULL,
       NULL,
       NULL,
       (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+3u)),
       (6u+3u),
       3u,
       &zBmiDiagStatus_0[BMI_SM_MPFDI_OVUV_SC_DIAG],
       Bmi_ProcessRxData,
       NULL
      },
      {//20
       BMI_SERVICE_DIAGNOSTICS,
       BMI_SM_MPFDI_OTUT_RR_DIAG,
       BMI_IFACE_0,
       BMI_SGL_RSP_PREFIX_LEN,
       (void *) zBmiGenericRespData_0,
       NULL,
       NULL,
       NULL,
       (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
       (6u+2u),
       2u,
       &zBmiDiagStatus_0[BMI_SM_MPFDI_OTUT_RR_DIAG],
       Bmi_ProcessRxData,
       NULL
      },
      {//21
       BMI_SERVICE_DIAGNOSTICS,
       BMI_SM_MPFDI_SLEEP_FAULT_DIAG,
       BMI_IFACE_0,
       BMI_SGL_RSP_PREFIX_LEN,
       (void *) zBmiGenericRespData_0,
       NULL,
       NULL,
       NULL,
       (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
       (6u+1u),
       1u,
       &zBmiDiagStatus_0[BMI_SM_MPFDI_SLEEP_FAULT_DIAG],
       Bmi_ProcessRxData,
       NULL
      },
      {//22
       BMI_SERVICE_DIAGNOSTICS,
       BMI_SM_MPFDI_HW_RESET_DIAG,
       BMI_IFACE_0,
       BMI_SGL_RSP_PREFIX_LEN,
       (void *) zBmiGenericRespData_0,
       NULL,
       NULL,
       NULL,
       (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
       (6u+1u),
       1u,
       &zBmiDiagStatus_0[BMI_SM_MPFDI_HW_RESET_DIAG],
       Bmi_ProcessRxData,
       NULL
      },
      {//23
       BMI_SERVICE_DIAGNOSTICS,
       BMI_SM_MPFDI_DEV_ADDR_DIAG,
       BMI_IFACE_0,
       BMI_SGL_RSP_PREFIX_LEN,
       (void *) zBmiGenericRespData_0,
       NULL,
       NULL,
       NULL,
       (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
       (6u+1u),
       1u,
       &zBmiDiagStatus_0[BMI_SM_MPFDI_DEV_ADDR_DIAG],
       Bmi_ProcessRxData,
       NULL
      },
    // FDTI Routines
    {//18
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_CYCLE_INIT,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+5u)),
        (6u+5u),
        5u,
        &zBmiDiagStatus_0[BMI_SM_FDTI_CYCLE_INIT],
        Bmi_ProcessRxData,
        NULL
    },
    {//19
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_GPIO_DIAG,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zBmiGpioDataBuf_1,
        (void *) zBmiDiagGpioRead1_0,
        (void *) zBmiDiagGpioRead2_0,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_GPIO_READ_BLOCK_LEN_0),
        BMI_GPIO_READ_BLOCK_LEN_0,
        (BMI_GPIO_READ_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
        &zBmiDiagStatus_0[BMI_SM_FDTI_GPIO_DIAG],
        Bmi_ProcessRxData,
        NULL
    },
    {//20
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_GPIO_OPNWR,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zBmiGpioDataBuf_1,
        (void *) zBmiDiagGpioRead1_0,
        (void *) zBmiDiagGpioRead2_0,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_GPIO_READ_BLOCK_LEN_0),
        BMI_GPIO_READ_BLOCK_LEN_0,
        (BMI_GPIO_READ_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
        &zBmiDiagStatus_0[BMI_SM_FDTI_GPIO_OPNWR],
        Bmi_ProcessRxData,
        NULL
    },
    {//21
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_GPIO_ADJSHRT,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (uint8 *) zBmiGpioDataBuf_1,
        (void *) zBmiDiagGpioRead1_0,
        (void *) zBmiDiagGpioRead2_0,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_GPIO_READ_BLOCK_LEN_0),
        BMI_GPIO_READ_BLOCK_LEN_0,
        (BMI_GPIO_READ_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
        &zBmiDiagStatus_0[BMI_SM_FDTI_GPIO_ADJSHRT],
        Bmi_ProcessRxData,
        NULL
    },
    {//22
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_VC_OPN_DET,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiVCellDataBuf_0,
        (void *) zBmiDiagVCellRead1_0,
        (void *) zBmiDiagVCellOpenPlau_0,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_VCELL_READ_BLOCK_LEN_0),
        BMI_VCELL_READ_BLOCK_LEN_0,
        (BMI_VCELL_READ_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
        &zBmiDiagStatus_0[BMI_SM_FDTI_VC_OPN_DET],
        Bmi_ProcessRxData,
        NULL
    },
    {//23
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_VCCB_SHRT,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiVCellDataBuf_0,
        (void *) zBmiDiagVCellRead1_0,
        (void *) zBmiDiagVCellRead2_0,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * BMI_VCELL_READ_BLOCK_LEN_0),
        BMI_VCELL_READ_BLOCK_LEN_0,
        (BMI_VCELL_READ_BLOCK_LEN_0 - BMI_SGL_RSP_FIXED_LEN),
        &zBmiDiagStatus_0[BMI_SM_FDTI_VCCB_SHRT],
        Bmi_ProcessRxData,
        NULL
    },
    {//24
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_VCELLGPIO_ACOMP,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+5u)),
        (6u+5u),
        5u,
        &zBmiDiagStatus_0[BMI_SM_FDTI_VCELLGPIO_ACOMP],
        Bmi_ProcessRxData,
        NULL
    },
    {//25
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_DCOMP_DIAG,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+3u)),
        (6u+3u),
        3u,
        &zBmiDiagStatus_0[BMI_SM_FDTI_DCOMP_DIAG],
        Bmi_ProcessRxData,
        NULL
    },
    {//26
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_ADC_COMP_DIAG,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+3u)),
        (6u+3u),
        3u,
        &zBmiDiagStatus_0[BMI_SM_FDTI_ADC_COMP_DIAG],
        Bmi_ProcessRxData,
        NULL
    },
    {//27
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_FAULT_SUMMARY,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
        (6u+1u),
        1u,
        &zBmiDiagStatus_0[BMI_SM_FDTI_FAULT_SUMMARY],
        Bmi_ProcessRxData,
        NULL
    },
    {//28
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FDTI_DIETMEP_DIAG,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+4u)),
        (6u+4u),
        4u,
        &zBmiDiagStatus_0[BMI_SM_FDTI_DIETMEP_DIAG],
        Bmi_ProcessRxData,
        NULL
    },
    {//29
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FTDI_NFAULT,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
        (6u+2u),
        2u,
        &zBmiDiagStatus_0[BMI_SM_FTDI_NFAULT],
        Bmi_ProcessRxData,
        NULL
    },
    {//30
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SM_COMM_FAULT,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
         (6u+1u),
         1u,
         &zBmiDiagStatus_0[BMI_SM_COMM_FAULT],
         Bmi_ProcessRxData,
         NULL
     },
     {//31
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_ECC_DIAG,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
        (6u+1u),
        1u,
        &zBmiDiagStatus_0[BMI_SM_ECC_DIAG],
        Bmi_ProcessRxData,
        NULL
     },
     {//32
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_COMM_FLT_MSK_DIAG,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
        (6u+2u),
        2u,
        &zBmiDiagStatus_0[BMI_SM_COMM_FLT_MSK_DIAG],
        Bmi_ProcessRxData,
        NULL
     },
     {//33
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FACTORY_CRC_DIAG,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
        (6u+1u),
        1u,
        &zBmiDiagStatus_0[BMI_SM_FACTORY_CRC_DIAG],
        Bmi_ProcessRxData,
        NULL
      },
      {//34
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_VLF_CRC_DIAG,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
        (6u+1u),
        1u,
        &zBmiDiagStatus_0[BMI_SM_VLF_CRC_DIAG],
        Bmi_ProcessRxData,
        NULL
     },
     {//35
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_SPI_FAULT_DIAG,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+3u)),
        (6u+3u),
        3u,
        &zBmiDiagStatus_0[BMI_SM_SPI_FAULT_DIAG],
        Bmi_ProcessRxData,
        NULL
     },
     {//36
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_FAULT_ADC_MISC,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
        (6u+1u),
        1u,
        &zBmiDiagStatus_0[BMI_SM_FAULT_ADC_MISC],
        Bmi_ProcessRxData,
        NULL
     },
     {//37
        BMI_SERVICE_DIAGNOSTICS,
        BMI_SM_VLF_FAULT_DIAG,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
        (6u+2u),
        2u,
        &zBmiDiagStatus_0[BMI_SM_VLF_FAULT_DIAG],
        Bmi_ProcessRxData,
        NULL
     },
     {//38
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SM_GET_DIAG_D1,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
         (6u+2u),
         2u,
         &zBmiDiagStatus_0[BMI_SM_GET_DIAG_D1],
         Bmi_ProcessRxData,
         NULL
     },
     {//39
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SM_GET_DIAG_D2,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
         (6u+2u),
         2u,
         &zBmiDiagStatus_0[BMI_SM_GET_DIAG_D2],
         Bmi_ProcessRxData,
         NULL
     },
     {//40
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SET_SOFT_RESET,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
         (6u+2u),
         2u,
         &zBmiDiagStatus_0[BMI_SET_SOFT_RESET],
         Bmi_ProcessRxData,
         NULL
      },
      {//41
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SET_HW_RESET,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
         (6u+2u),
         2u,
         &zBmiDiagStatus_0[BMI_SET_HW_RESET],
         Bmi_ProcessRxData,
         NULL
      },
      {//42
         BMI_SERVICE_DIAGNOSTICS,
         BMI_GOTO_SHUTDOWN,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
         (6u+2u),
         2u,
         &zBmiDiagStatus_0[BMI_GOTO_SHUTDOWN],
         Bmi_ProcessRxData,
         NULL
      },
      {//43
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SET_DEV_SLEEP,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
         (6u+2u),
         2u,
         &zBmiDiagStatus_0[BMI_SET_DEV_SLEEP],
         Bmi_ProcessRxData,
         NULL
       },
       {//44
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SET_DEV_SHUT2WAKEUP,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
         (6u+2u),
         2u,
         &zBmiDiagStatus_0[BMI_SET_DEV_SHUT2WAKEUP],
         Bmi_ProcessRxData,
         NULL
       },
       {//45
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SET_DEV_SLP2WAKEUP,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
         (6u+2u),
         2u,
         &zBmiDiagStatus_0[BMI_SET_DEV_SLP2WAKEUP],
         Bmi_ProcessRxData,
         NULL
       },
       {//46
         BMI_SERVICE_DIAGNOSTICS,
         BMI_DIAG_ABORT_DIAG,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
         (6u+1u),
         1u,
         &zBmiDiagStatus_0[BMI_DIAG_ABORT_DIAG],
         Bmi_ProcessRxData,
         NULL
       },
       {//47
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SM_BB_PLAU_DIAG,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
         (6u+1u),
         1u,
         &zBmiDiagStatus_0[BMI_SM_BB_PLAU_DIAG],
         Bmi_ProcessRxData,
         NULL
       },
       {//48
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SM_CB_PLAU_DIAG,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
         (6u+1u),
         1u,
         &zBmiDiagStatus_0[BMI_SM_CB_PLAU_DIAG],
         Bmi_ProcessRxData,
         NULL
       },
       {//49
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SM_FACR_TM,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
         (6u+1u),
         1u,
         &zBmiDiagStatus_0[BMI_SM_FACR_TM],
         Bmi_ProcessRxData,
         NULL
       },
       {//50
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SM_FAULT_OTP_ERR_DIAG,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
         (6u+1u),
         1u,
         &zBmiDiagStatus_0[BMI_SM_FAULT_OTP_ERR_DIAG],
         Bmi_ProcessRxData,
         NULL
       },
       {//51
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SM_OTP_STATE_DIAG,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
         (6u+1u),
         1u,
         &zBmiDiagStatus_0[BMI_SM_OTP_STATE_DIAG],
         Bmi_ProcessRxData,
         NULL
       },
       {//52
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SM_CRC_STATE_DIAG,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
         (6u+1u),
         1u,
         &zBmiDiagStatus_0[BMI_SM_CRC_STATE_DIAG],
         Bmi_ProcessRxData,
         NULL
       },
       {//53
         BMI_SERVICE_DIAGNOSTICS,
         BMI_FDTI_FAULT_SYS_DIAG,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
         (6u+1u),
         1u,
         &zBmiDiagStatus_0[BMI_FDTI_FAULT_SYS_DIAG],
         Bmi_ProcessRxData,
         NULL
       },
       {//54
        BMI_SERVICE_DIAGNOSTICS,
        BMI_FDTI_FLIP_RESET_DIAG,
        BMI_IFACE_0,
        BMI_SGL_RSP_PREFIX_LEN,
        (void *) zBmiGenericRespData_0,
        NULL,
        NULL,
        NULL,
        (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+32u)),
        (6u+32u),
        32u,
        &zBmiDiagStatus_0[BMI_FDTI_FLIP_RESET_DIAG],
        Bmi_ProcessRxData,
        NULL
       },
       {//55
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SM_FTDI_COMM_DEBUG_DIAG,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
         (6u+1u),
         1u,
         &zBmiDiagStatus_0[BMI_SM_FTDI_COMM_DEBUG_DIAG],
         Bmi_ProcessRxData,
         NULL
       },
       {//55
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SM_GET_DIAG_MAIN_RDNT,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+1u)),
         (6u+1u),
         1u,
         &zBmiDiagStatus_0[BMI_SM_GET_DIAG_MAIN_RDNT],
         Bmi_ProcessRxData,
         NULL
       },
       {//55
         BMI_SERVICE_DIAGNOSTICS,
         BMI_SM_FAULT_PWR_DIAG,
         BMI_IFACE_0,
         BMI_SGL_RSP_PREFIX_LEN,
         (void *) zBmiGenericRespData_0,
         NULL,
         NULL,
         NULL,
         (TIBMS_MAX_AFES_IN_IFACE_0 * (6u+2u)),
         (6u+2u),
         2u,
         &zBmiDiagStatus_0[BMI_SM_FAULT_PWR_DIAG],
         Bmi_ProcessRxData,
         NULL
       },
};

#define TIBMS_LCFG_0_STOP_SEC_CONST
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * External Functions Definition
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * End of File: tibms_Lcfg_0.c
 *********************************************************************************************************************/

