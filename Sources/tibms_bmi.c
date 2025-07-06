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
 *  File:       tibms_bmi.c
 *  Project:    TIBMS
 *  Module:     BMI
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Functionalities for Battery Monitor Interface
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       13July2022   SEM                  0000000000000    Initial version
 * 01.00.01       18April2023  SEM                  0000000000000    Version and BmiSetErrorDetails
 * 01.01.00       04Aug2023    SEM                  0000000000000    Integrated Pack monitor in the same interface
 *                                                                   Corrected version report with Autosar Std version
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

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

/**	Major Software Config C Version number */
#define BMI_SW_C_MAJOR_VERSION                  (0x01u)

/**	Minor Software Config C Version number */
#define BMI_SW_C_MINOR_VERSION                  (0x01u)

/** Software Patch Config C Version number */
#define BMI_SW_C_PATCH_VERSION                  (0x00u)

#if ((BMI_SW_C_MAJOR_VERSION != TIBMS_CFG_MAJOR_VERSION) || \
     (BMI_SW_C_MINOR_VERSION != TIBMS_CFG_MINOR_VERSION) || \
	 (BMI_SW_C_PATCH_VERSION != TIBMS_CFG_PATCH_VERSION))
#error "tibms: Config version numbers of tibms_bmi.c and tibms_cfg.h are inconsistent!"
#endif

#if ((BMI_SW_MAJOR_VERSION != BMI_SW_C_MAJOR_VERSION) || \
     (BMI_SW_MINOR_VERSION != BMI_SW_C_MINOR_VERSION) || \
	 (BMI_SW_PATCH_VERSION != BMI_SW_C_PATCH_VERSION))
#error "tibms: Config version numbers of tibms_bmi.c and tibms_bmi.h are inconsistent!"
#endif

#if ((TIBMS_API_MAJOR_VERSION != BMI_SW_C_MAJOR_VERSION) || \
     (TIBMS_API_MINOR_VERSION != BMI_SW_C_MINOR_VERSION) || \
	 (TIBMS_API_PATCH_VERSION != BMI_SW_C_PATCH_VERSION))
#error "tibms: Config version numbers of tibms_bmi.c and tibms_api.h are inconsistent!"
#endif

/*********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

#define BMI_GUARD_START                         (0xCCA1AB1BU)
#define BMI_GUARD_END                           (0xCAFECEEBU)
#define BMI_INIT                                (0xBBU)

#define BMI_INTEGRITY_CHECK(pBmiMgr)            ((NULL != pBmiMgr) && \
                                                 (BMI_GUARD_START == pBmiMgr->qGuardStart) && \
                                                 (BMI_GUARD_END == pBmiMgr->qGuardEnd) && \
                                                 (BMI_INIT == pBmiMgr->uInit))

/*********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

typedef struct Bmi_ManagerType_Tag
{
    uint32 qGuardStart;

    const InterfaceCfgType *pIfaceCfg;
    const Bmi_OperationType *pBmcOps;
    void *pBmicMgr;
    Comif_ManagerType *pComifMgr;

    uint8 uInit;
    uint8 uLastWupReq;
    uint8 uResetReq;
    uint8 uBmiIface;

    uint8 uIsPmicInIface;
    uint8 uPmicDevId;
    uint8 uRsvd[2];

    uint32 qGuardEnd;
}
Bmi_ManagerType;

typedef struct Pmi_ManagerType_Tag
{
    uint32 qGuardStart;

    const InterfaceCfgType *pIfaceCfg;
    const Pmi_OperationType *pPmcOps;
    void *pPmicMgr;
    Comif_ManagerType *pComifMgrCtx;

    uint8 uInit;
    uint8 uWupReq;
    uint8 uResetReq;
    uint8 uPmiIface;

    uint32 qGuardEnd;
}
Pmi_ManagerType;


/*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

#define TIBMS_BMI_START_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

static Bmi_ManagerType zBmiManager[TIBMS_NO_OF_BMI_IFACES];

#if(TIMBS_BMI_EMEM_ENABLE == STD_ON)
static Emem_ManagerType zBmiEmemMgr;
#endif

#define TIBMS_BMI_STOP_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

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

/*********************************************************************************************************************
 *  External Functions Definition
 *********************************************************************************************************************/

#define TIBMS_BMI_START_SEC_CODE
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_ConfigureOtp(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief         Configure OTP
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_ConfigureOtp(uint8 uBmiIfaceIn)
{
    uint8 uRet = (uint8)BMI_NOT_OK;
	uRet = uBmiIfaceIn; // Functionality not implemented so return Iface value to avaoid compilation warnings

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_SpiMasterTransfer(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          SPI Master Transfer
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_SpiMasterTransfer(uint8 uBmiIfaceIn)
{
    uint8 uRet = (uint8)BMI_NOT_OK;
	uRet = uBmiIfaceIn; // Functionality not implemented so return Iface value to avaoid compilation warnings

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_I2cTransfer(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          I2C Transfer
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_I2cTransfer(uint8 uBmiIfaceIn)
{
    uint8 uRet = (uint8)BMI_NOT_OK;
	uRet = uBmiIfaceIn; // Functionality not implemented so return Iface value to avaoid compilation warnings

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetFaultStatus(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Fault Status
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetFaultStatus(uint8 uBmiIfaceIn)
{
    uint8 uRet = (uint8)BMI_NOT_OK;
	uRet = uBmiIfaceIn; // Functionality not implemented so return Iface value to avaoid compilation warnings

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, tibms_bmi_CODE) Bmi_CellBalCtrl(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Start the Cell balancing control
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      Bmi_CbCtrlType: Cell Balancing Control Request
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_CellBalCtrl(uint8 uBmiIfaceIn, uint8 uCmd, Bmi_uCbTime *pCBCtrl)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
    }
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_BALANCING, uCmd,0u, (void *) pCBCtrl);

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetVGpio(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Get Temperatures
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetVGpio(uint8 uBmiIfaceIn)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_GPIO_READ, 0u,0u, (void *) NULL);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetGpioData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the Gpio Measurement Data recieved from the devices
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetGpioData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = (uint8)BMI_INVALID_GET_BUF;
        if((pBuffer!=NULL) && (wLength > 0U))
        {
            uRet = pBmiMgr->pBmcOps->decode_rx(pBmiMgr->pBmicMgr, BMI_SERVICE_GPIO_READ,0U, pBuffer, wLength);
        }
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetVRefCap(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Request for Cell voltages
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetVRefCap(uint8 uBmiIfaceIn)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_VCAP_REF, 0u,0u, (void *) NULL);
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetVCapRefData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the Cell Voltage Data recieved from the devices
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetVCapRefData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = (uint8)BMI_INVALID_GET_BUF;
        if((pBuffer!=NULL) && (wLength > 0U))
        {
            uRet = pBmiMgr->pBmcOps->decode_rx(pBmiMgr->pBmicMgr, BMI_SERVICE_VCAP_REF,0U, pBuffer, wLength);
        }
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetVBat(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Request for Cell voltages
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetVBat(uint8 uBmiIfaceIn)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_VBAT, 0u,0u, (void *) NULL);
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetVBatData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the Cell Voltage Data recieved from the devices
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetVBatData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = (uint8)BMI_INVALID_GET_BUF;
        if((pBuffer!=NULL) && (wLength > 0U))
        {
            uRet = pBmiMgr->pBmcOps->decode_rx(pBmiMgr->pBmicMgr, BMI_SERVICE_VBAT,0U, pBuffer, wLength);
        }
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetVCell(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Request for Cell voltages
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetVCell(uint8 uBmiIfaceIn)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_CELL_VOLT, 0u,0u, (void *) NULL);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetVcellData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the Cell Voltage Data recieved from the devices
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetVcellData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = (uint8)BMI_INVALID_GET_BUF;
        if((pBuffer!=NULL) && (wLength > 0U))
        {
            uRet = pBmiMgr->pBmcOps->decode_rx(pBmiMgr->pBmicMgr, BMI_SERVICE_CELL_VOLT,0U, pBuffer, wLength);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_DiagServiceRequest(uint8 uBmiIfaceIn, uint8 uCmd, void *pData)
 *********************************************************************************************************************/
/*! \brief          Diagnostic Service Request
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uCmd: Request Command
 *  \param[inout]   pData: Data Request
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_DiagServiceRequest(uint8 uBmiIfaceIn, uint8 uCmd, void *pData , uint8 uChannel)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;


    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_DIAGNOSTICS, uCmd, uChannel, pData);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagStatus(uint8 uBmiIfaceIn, void *pDiagStat, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the Gpio Measurement Data recieved from the devices
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagStatus(uint8 uBmiIfaceIn, void *pDiagStat, uint16 wLength)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = (uint8)BMI_INVALID_GET_BUF;
        if((pDiagStat!=NULL) && (wLength > 0U))
        {
            uRet = pBmiMgr->pBmcOps->ioctl(pBmiMgr->pBmicMgr, BMI_GET_DIAG_STATUS, pDiagStat, wLength);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagResult(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the Gpio Measurement Data recieved from the devices
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagResult(uint8 uBmiIfaceIn, void *pDiagResult, uint16 wLength)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet =(uint8) BMI_INVALID_GET_BUF;
        if((pDiagResult!=NULL) && (wLength > 0U))
        {
            uRet = pBmiMgr->pBmcOps->ioctl(pBmiMgr->pBmicMgr, BMI_GET_DIAG_RESULT, pDiagResult, wLength);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_WmServiceRequest(uint8 uBmiIfaceIn, uint8 uCmd, uint8 uNodeId, void *pData)
 *********************************************************************************************************************/
/*! \brief          Wireless Manager Service Request
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uCmd: Request Command
 *  \param[in]      uNodeId: Node Id request
 *  \param[inout]   pData: Data Request
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_WmServiceRequest(uint8 uBmiIfaceIn, uint8 uCmd, uint8 uNodeId, void *pData)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_COMMUNICATION, uCmd, uNodeId, pData);
    }

    return uRet;
}



/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_ShutdownRequest(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing the shutdown request
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_ShutdownRequest(uint8 uBmiIface)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_POWER_CONTROL, BMI_GOTO_SHUTDOWN,0u, (void *) NULL);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_SoftReset(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing the soft reset
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_SoftReset(uint8 uBmiIface)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_RESET, BMI_SET_SOFT_RESET,0u, (void *) NULL);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_HWReset(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing hardware reset
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_HWReset(uint8 uBmiIface)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_RESET, BMI_SET_HW_RESET,0u, (void *) NULL);
    }

    return uRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_Sleep(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing setting the dev to sleep
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_DevSleep(uint8 uBmiIface)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_POWER_CONTROL, BMI_SET_DEV_SLEEP,0u, (void *) NULL);
    }

    return uRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_ShutTowake(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing shutdown to wakeup
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_ShutTowake(uint8 uBmiIface)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_POWER_CONTROL, BMI_SET_DEV_SHUT2WAKEUP,0u, (void *) NULL);
    }

    return uRet;
}



/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_SleepToWake(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing the sleep to wakeup 
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_SleepToWake(uint8 uBmiIface)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_POWER_CONTROL, BMI_SET_DEV_SLP2WAKEUP,0u, (void *) NULL);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_HandleReqTransfer(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Handle will be called once driver has data to send
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_HandleReqTransfer(uint8 uBmiIfaceIn)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet =(uint8) BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->handle_tx(pBmiMgr->pBmicMgr);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_HandleRecvComplete(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Handle will be called once driver successfully recieve the complete packet
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_HandleRecvComplete(uint8 uBmiIfaceIn)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->handle_rx(pBmiMgr->pBmicMgr);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_ProcessRxData(const ServiceDataType *, uint8 *, uint8 , uint16 )
 *********************************************************************************************************************/
/*! \brief          process the Rx Data recieved from the devices
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      pSvcCtx: Pointer to Service Data
 *  \param[in]      pRxData: Pointer to Response Data
 *  \param[in]      uRespStatus: status of the transfer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           This calls the applicatin layer callback function to process the recieved data
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_ProcessRxData(const ServiceCfgType *pSvcCfg, const uint8 *pRxData, uint8 uRespStatus)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[pSvcCfg->uIface];
    if (BMI_INTEGRITY_CHECK(pBmiMgr) && (pSvcCfg!=NULL))
    {
        uRet = pBmiMgr->pBmcOps->process_rx(pBmiMgr->pBmicMgr, pSvcCfg, pRxData, uRespStatus);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_ProcessNotify(uint8 uBmiIfaceIn, uint8 uType)
 *********************************************************************************************************************/
/*! \brief          Process the notification recieved from the external interface
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uType: Notification information
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_ProcessNotify(uint8 uBmiIfaceIn, uint8 uType)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->notify(pBmiMgr->pBmicMgr, uType);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetDriverState(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Initialization during startup
 *
 *  \param[in]      uBmiIfaceIn
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           Bmi_StartupInit will be completed after BMIC returns successfull and transfer to into Normal mode
 *  \return         returns the eDriver State
 *  \retval
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetDriverState(uint8 uBmiIfaceIn)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->ioctl(pBmiMgr->pBmicMgr, BMI_GET_DRIVER_STATE, NULL, 0u);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagState(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Initialization during startup
 *
 *  \param[in]      uBmiIfaceIn
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           Bmi_StartupInit will be completed after BMIC returns successfull and transfer to into Normal mode
 *  \return         returns the eDriver State
 *  \retval
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagState(uint8 uBmiIfaceIn)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->ioctl(pBmiMgr->pBmicMgr, BMI_GET_DIAG_STATE, NULL, 0u);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagMain(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Initialization during startup
 *
 *  \param[in]      uBmiIfaceIn
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           Bmi_StartupInit will be completed after BMIC returns successfull and transfer to into Normal mode
 *  \return         returns the eDriver State
 *  \retval
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagMain(uint8 uBmiIfaceIn)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_GET_DIAG_MAIN,  0U,0u, (void *) NULL);
    }

    return uRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetVcellActSum(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Initialization during startup
 *
 *  \param[in]      uBmiIfaceIn
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           Bmi_StartupInit will be completed after BMIC returns successfull and transfer to into Normal mode
 *  \return         returns the eDriver State
 *  \retval
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetVcellActSum(uint8 uBmiIfaceIn)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_GET_VCELL_ACTSUM,  0U,0u, (void *) NULL);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetDieTemp(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Initialization during startup
 *
 *  \param[in]      uBmiIfaceIn
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           Bmi_StartupInit will be completed after BMIC returns successfull and transfer to into Normal mode
 *  \return         returns the eDriver State
 *  \retval
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetDieTemp(uint8 uBmiIfaceIn)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_GET_DIE_TEMP,  0U,0u, (void *) NULL);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagRdnt(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Initialization during startup
 *
 *  \param[in]      uBmiIfaceIn
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           Bmi_StartupInit will be completed after BMIC returns successfull and transfer to into Normal mode
 *  \return         returns the eDriver State
 *  \retval
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagRdnt(uint8 uBmiIfaceIn)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_GET_DIAG_RDNT,  0U,0u, (void *) NULL);
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_StartupInit(uint8 uBmiIfaceIn, uint8 eNotify)
 *********************************************************************************************************************/
/*! \brief          Initialization during startup
 *
 *  \param[in]      uBmiIfaceIn
 *  \param[in]      eNotify - Notification Method after completing the Init
 *                  INIT_NOTIFY_BLOCKING
 *                  INIT_NOTIFY_CALLBACK
 *                  INIT_NOTIFY_POLLING
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           Bmi_StartupInit will be completed after BMIC returns successfull and transfer to into Normal mode
 *  \return         status of the initialization
 *  \retval         BMI_OK: Successful Reset Request Init
 *                  BMI_NOT_OK: Failed to initiate startup
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_StartupInit(uint8 uBmiIfaceIn, uint8 eNotify)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
#if (PMIF_BQ7973X_ENABLE == STD_ON)                                                 /** PMI will be woken-up first since at BEGIN **/
        if((pBmiMgr->uIsPmicInIface) && (PM_DEVID_BEGIN == pBmiMgr->uPmicDevId))
        {
            uRet = Pmi_StartupInit(uBmiIfaceIn, eNotify);
            if(BMI_OK != uRet)
            {
                uRet = (uint8)BMI_PMC_INIT_FAILED;
            }
        }
#endif
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_CONFIG, 0u, 0u, (void *) &eNotify);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_SetErrorDetails(uint8 uSvcId, uint8 uErrorId, uint32 qParam1, uint32 qParam2)
 *********************************************************************************************************************/
/*! \brief          This function is Set the Error Memory Details
 *
 *
 *  \param[in]       uSvcId
 *  \param[in]       uErrorId
 *  \param[in]       param_1
 *  \param[in]       param_2
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *
 *  \return         Set the Error Memory Information
 *  \retval         BMI_OK: Successful
 *                  BMI_NOT_OK: Failed
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_SetErrorDetails(uint8 uSvcId, uint8 uErrorId, uint32 qParam1, uint32 qParam2)
{
#if(TIMBS_BMI_EMEM_ENABLE == STD_ON)
    Emem_EntryType *pEmemEntry;
    uint8 uErrIdx;
    uint8 uEMIdOcpd;

    if((uErrorId > BMI_OK) && (uErrorId < BMI_INIT_PROGRESS))
    {
        if(ERRMEM_STATE_INIT == zBmiEmemMgr.uEmemState)
        {
            uEMIdOcpd = zBmiEmemMgr.uEmemSiz;

            zBmiEmemMgr.resource_req(zBmiEmemMgr.qEmemRes, GET_RESOURCE);
            for(uErrIdx = 0; uErrIdx < zBmiEmemMgr.uEmemSiz; uErrIdx++)
            {
                if((uErrorId == zBmiEmemMgr.pEmemEntry[uErrIdx].uErrorId) &&
                   (uSvcId == zBmiEmemMgr.pEmemEntry[uErrIdx].eServiceId))
                {
                    uEMIdOcpd = uErrIdx;
                }
            }

            if(zBmiEmemMgr.uEmemSiz == uEMIdOcpd)
            {
                pEmemEntry = &zBmiEmemMgr.pEmemEntry[zBmiEmemMgr.uEmemIdx++];
                if(pEmemEntry!=NULL)
                {
                    pEmemEntry->eServiceId = uSvcId;
                    pEmemEntry->uErrorId = uErrorId;
                    pEmemEntry->qParam1 = qParam1;
                    pEmemEntry->qParam2 = qParam2;
                }
            }
            else
            {
                pEmemEntry = &zBmiEmemMgr.pEmemEntry[uEMIdOcpd];
                if(pEmemEntry!=NULL)
                {
                    pEmemEntry->nError++;
                }
            }

            if(zBmiEmemMgr.uEmemIdx == zBmiEmemMgr.uEmemSiz)
            {
                zBmiEmemMgr.uEmemIdx = 0;
            }
            zBmiEmemMgr.resource_req(zBmiEmemMgr.qEmemRes, RELEASE_RESOURCE);
        }
    }
#endif

     return EMEM_ERRORSTATUS2USERSTATUS(uErrorId, ((uErrorId > BMI_OK) && (uErrorId < BMI_INIT_PROGRESS)));
}

/**********************************************************************************************************************
 * FUNC(EmemInfoType *, tibms_bmi_CODE) Bmi_ErrMemoryInit(void)
 *********************************************************************************************************************/
/*! \brief          Error Memory Init
 *
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *
 *  \return         status of version information
 *  \retval         BMI_OK: Successful
 *                  BMI_NOT_OK: Failed
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_ErrMemoryInit(void)
{
    uint8 uRet = (uint8)BMI_NOT_OK;

#if(TIMBS_BMI_EMEM_ENABLE == STD_ON)
    if(zBmiEmemConfig.pEmemEntry)
    {
        zBmiEmemMgr.resource_req = zBmiEmemConfig.resource_req;
        zBmiEmemMgr.qEmemRes = zBmiEmemConfig.qEmemRes;

        zBmiEmemMgr.pEmemEntry = zBmiEmemConfig.pEmemEntry;
        zBmiEmemMgr.uEmemIdx = 0u;
        zBmiEmemMgr.uEmemSiz = (uint8) zBmiEmemConfig.wEmemEntrySiz;

        zBmiEmemMgr.uEmemState = ERRMEM_STATE_INIT;

        uRet = (uint8)BMI_OK;
    }
#else
    uRet = BMI_OK;
#endif

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_Init(uint8 uCfgSet, uint8 uWupReq)
 *********************************************************************************************************************/
/*! \brief          This function is used to init the Bmi core for all chains in the config set
 *
 *  \param[in]      uCfgSet
 *  \param[in]      uWupReq
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         status of the initialization
 *  \retval         BMI_OK: Successful init
 *                  BMI_NOT_OK: Failed to init
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_Init(const Bms_ConfigType *pConfigSet_PB, uint8 uWupReq)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uNfaultPinCfg = 0;
    uint8 uStartupModeCfg = 0;
    uint8 uRet = (uint8)BMI_INIT_FAILED;


    do
    {
        #if(TIMBS_BMI_EMEM_ENABLE == STD_ON)
        uRet = Bmi_ErrMemoryInit();
        if(BMI_NOT_OK == uRet)
        {
            uRet = (uint8)BMI_INVALID_EMEM_CFG;
            break;
        }
        #endif

        if((NULL == pConfigSet_PB) ||
           (TIBMS_CFG_START_SIG != pConfigSet_PB->qStartSig) ||
           (TIBMS_CFG_STOP_SIG != pConfigSet_PB->qEndSig))
        {
            uRet = (uint8)BMI_INVALID_CONFIG;
            break;
        }

        if(NULL == pConfigSet_PB->pIfaceCfg)
        {
            uRet = (uint8)BMI_INVALID_IFACE_CFG;
            break;
        }

        if(NULL == pConfigSet_PB->pIfaceCfg->pBmiCfg)
        {
            uRet = (uint8)BMI_INVALID_BMI_CFG;
            break;
        }

        if(pConfigSet_PB->pIfaceCfg->uIfaceCfg >= TIBMS_NO_OF_BMI_IFACES)
        {
            uRet = (uint8)BMI_INVALID_CONFIG;
            break;
        }

        pBmiMgr = &zBmiManager[pConfigSet_PB->pIfaceCfg->uIfaceCfg];
        if((pBmiMgr!=NULL) && (BMI_INIT != pBmiMgr->uInit))
        {
            memset(pBmiMgr, 0, sizeof(Bmi_ManagerType));

            pBmiMgr->uLastWupReq = uWupReq;
            pBmiMgr->uBmiIface = pConfigSet_PB->pIfaceCfg->uIfaceCfg;
            pBmiMgr->pIfaceCfg = pConfigSet_PB->pIfaceCfg;

            pBmiMgr->pBmcOps = pBmiMgr->pIfaceCfg->pBmiCfg->pBmcOpsCfg;
            if(NULL == pBmiMgr->pBmcOps)
            {
                uRet = (uint8)BMI_INVALID_OPS_CFG;
                break;
            }

            pBmiMgr->pComifMgr = Comif_Init(pBmiMgr->pIfaceCfg->pComIfCfg, &uNfaultPinCfg);
            if(NULL == pBmiMgr->pComifMgr)
            {
                uRet = (uint8)BMI_COMIF_INIT_FAILED;
                break;
            }

            pBmiMgr->pBmicMgr = pBmiMgr->pBmcOps->init(pBmiMgr->pIfaceCfg, pBmiMgr->pComifMgr, uNfaultPinCfg,
                                                       uStartupModeCfg, uWupReq);
            if(NULL == pBmiMgr->pBmicMgr)
            {
                uRet = (uint8)BMI_BMC_INIT_FAILED;
                break;
            }

            pBmiMgr->uIsPmicInIface = pConfigSet_PB->pIfaceCfg->uIsPmicInIfCfg;
            pBmiMgr->uPmicDevId = pConfigSet_PB->pIfaceCfg->uPmicDevIdCfg;
#if (PMIF_BQ7973X_ENABLE == STD_ON)

            if(pBmiMgr->uIsPmicInIface)
            {
                uRet = Pmi_Init(pConfigSet_PB, pBmiMgr->pComifMgr, uWupReq);
                if(BMI_OK != uRet)
                {
                    uRet = (uint8)BMI_PMC_INIT_FAILED;
                    break;
                }
            }
#endif
            pBmiMgr->qGuardStart = BMI_GUARD_START;
            pBmiMgr->qGuardEnd = BMI_GUARD_END;
            pBmiMgr->uInit = BMI_INIT;
        }
        else
        {
            uRet = (uint8)BMI_ALREADY_INIT;
            break;
        }

        uRet = (uint8)BMI_INIT_SUCCESS;
	}
    while(0);

    return Bmi_SetErrorDetails(BMI_SVCID_INIT, uRet, uWupReq, uStartupModeCfg);
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_Deinit(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          This function is used to De-init the Bmi core which is already initilized
 *
 *
 *  \param[in]      uBmiIfaceIn
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            Already Initialized by Bmi_Init
 *  \post
 *
 *  \return         status of the de-initialization
 *  \retval         BMI_OK: Successful De-init
 *                  BMI_NOT_OK: Failed to De-init
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_Deinit(uint8 uBmiIfaceIn)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = Comif_DeInit(pBmiMgr->pComifMgr);
        pBmiMgr->pBmcOps->deinit(pBmiMgr->pBmicMgr);
        memset(pBmiMgr, 0u, sizeof(Bmi_ManagerType));
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_SetDiagAdcCtrl1(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Initialization during startup
 *
 *  \param[in]      uBmiIfaceIn
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           Bmi_StartupInit will be completed after BMIC returns successfull and transfer to into Normal mode
 *  \return         returns the eDriver State
 *  \retval
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_SetDiagAdcCtrl1(uint8 uBmiIfaceIn, uint8 uAdcCtr1)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->ioctl(pBmiMgr->pBmicMgr, BMI_SET_ADC_CTRL1, NULL, uAdcCtr1);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_SetDiagAdcCtrl2(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Initialization during startup
 *
 *  \param[in]      uBmiIfaceIn
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           Bmi_StartupInit will be completed after BMIC returns successfull and transfer to into Normal mode
 *  \return         returns the eDriver State
 *  \retval
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_SetDiagAdcCtrl2(uint8 uBmiIfaceIn, uint8 uAdcCtr2)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->ioctl(pBmiMgr->pBmicMgr, BMI_SET_ADC_CTRL2, NULL, uAdcCtr2);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_SetDiagAdcCtrl3(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          Initialization during startup
 *
 *  \param[in]      uBmiIfaceIn
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           Bmi_StartupInit will be completed after BMIC returns successfull and transfer to into Normal mode
 *  \return         returns the eDriver State
 *  \retval
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_SetDiagAdcCtrl3(uint8 uBmiIfaceIn, uint8 uAdcCtr3)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->ioctl(pBmiMgr->pBmicMgr, BMI_SET_ADC_CTRL3, NULL, uAdcCtr3);
    }

    return uRet;
}



/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetVersionInfo(uint8 uBmiIfaceIn, Bmi_VersionInfoType *pBmifVersion)
 *********************************************************************************************************************/
/*! \brief          This function is used to De-init the Bmi core which is already initilized
 *
 *
 *  \param[in]       uBmiIfaceIn
 *  \param[out]      pBmifVersion
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *
 *  \return         status of version information
 *  \retval         BMI_OK: Successful
 *                  BMI_NOT_OK: Failed
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetVersionInfo(uint8 uBmiIfaceIn, VersionInfoType *pBmifVersion)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        if(NULL != pBmifVersion)
        {
            uRet = pBmiMgr->pBmcOps->ioctl(pBmiMgr->pBmicMgr, BMI_GET_VERSION, pBmifVersion, 0u);
            if(BMI_OK == uRet)
            {
                pBmifVersion->zFwVersion.vendorID = TIBMS_VENDOR_ID;
                pBmifVersion->zFwVersion.moduleID = TIBMS_MODULE_ID;
                pBmifVersion->zFwVersion.sw_major_version = TIBMS_VERSION_MAJOR;
                pBmifVersion->zFwVersion.sw_minor_version = TIBMS_VERSION_MINOR;
                pBmifVersion->zFwVersion.sw_patch_version = TIBMS_VERSION_PATCH;

                pBmifVersion->zBmiVersion.vendorID = BMI_VENDOR_ID;
                pBmifVersion->zBmiVersion.moduleID = BMI_MODULE_ID;
                pBmifVersion->zBmiVersion.sw_major_version = BMI_SW_MAJOR_VERSION;
                pBmifVersion->zBmiVersion.sw_minor_version = BMI_SW_MINOR_VERSION;
                pBmifVersion->zBmiVersion.sw_patch_version = BMI_SW_PATCH_VERSION;
            }
        }
    }

    return uRet;
}

                         /****************** 8.2 *****************/
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_SetCommConf(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing the soft reset
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_SetCommConf(uint8 uBmiIface,uint8 uNodeId )
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_SET_COMM_CONF, 0U,uNodeId, (void *) NULL);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) BMI_SetBBConf(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing the soft reset
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_SetBBConf(uint8 uBmiIface , uint8 uDevId)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_SET_BB_CONF, 0U,uDevId, (void *) NULL);
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) BMI_SetEnableSPI(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing the soft reset
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_SetEnableSPI(uint8 uBmiIface, uint8 uDevId)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_SET_ENB_SPI, 0U,uDevId, (void *) NULL);
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) BMI_SetSPIConf(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing the soft reset
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_SetSPIConf(uint8 uBmiIface, uint8 uDevId)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_SET_SPI_CONF, 0U,uDevId, (void *) NULL);
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) BMI_SetSPIExe(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing the soft reset
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_SetSPIExe(uint8 uBmiIface, uint8 uDevId, uint8 uSPIExe)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_SET_SPI_EXE, uSPIExe,uDevId, (void *) NULL);
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) BMI_WriteSPIData(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing the soft reset
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_WriteSPIData(uint8 uBmiIface, uint8 uDevId, const uint8 *uSPIData)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr,BMI_SERVICE_WRITE_SPI_DATA , 0U,uDevId,(void *)   uSPIData);
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) BMI_ReadSPIData(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing the soft reset
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_ReadSPIData(uint8 uBmiIface)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_READ_SPI_DATA, 0U,0u, (void *) NULL);
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) BMI_SetEnableI2C(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing the soft reset
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_SetEnableI2C(uint8 uBmiIface , uint8 uDevId)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_SET_ENB_I2C, 0U,uDevId, (void *) NULL);
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) BMI_SetI2CCtrl(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing the soft reset
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_SetI2CCtrl(uint8 uBmiIface, uint8 uDevId, uint8 uI2CCtrl)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
       uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_SET_I2C_CTRL, uI2CCtrl,uDevId, (void *) NULL);
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) BMI_SetI2CData(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing the soft reset
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_SetI2CData(uint8 uBmiIface, uint8 uDevId, uint8 uI2CData)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_SET_I2C_DATA, uI2CData,uDevId, (void *) NULL);
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) BMI_GetI2CData(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing the soft reset
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetI2CData(uint8 uBmiIface)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_GET_I2C_DATA, 0U,0u, (void *) NULL);
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) BMI_GetI2CFault(uint8 uBmiIfaceIn, uint8 uResetReq)
 *********************************************************************************************************************/
/*! \brief          for the servicing the soft reset
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]      uResetReq: Request Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetI2CFault(uint8 uBmiIface)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_GET_I2C_FAULT,  0U,0u, (void *) NULL);
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) BMI_GpioOpenSetGpioDown(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \\brief Write ODD GPIO configure drive output logic low.
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/


FUNC(uint8, tibms_bmi_CODE) BMI_GpioOpenSetGpioConf(uint8 uBmiIface, uint8 devId, uint8 gpio_conf_index, Gpio_val lower_gpio_val, Gpio_val higher_gpio_val )
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;
    Gpio_conf g_conf;

    {
       g_conf.gpio_conf_addr = gpio_conf_index;
       g_conf.gpio_conf_val = (higher_gpio_val << 3U) | lower_gpio_val;
       g_conf.devId = devId;
    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
           uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_SET_GPIO_CONF,  0U,0u, (void *)&g_conf);
       }
    }

    return uRet;
}
/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) BMI_GpioOpenSetGpioDown(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          brief Write GPIO configure ADC & Weak Pull-Down.
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GpioOpenSetGpioDown(uint8 uBmiIface)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_SET_GPIO_DOWN,  0U,0u, (void *) NULL);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) BMI_EnADCFreeze(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          brief Write GPIO configure ADC & Weak Pull-Down.
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_EnADCFreeze(uint8 uBmiIface)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_EN_ADC_FREEZE,  0U,0u, (void *) NULL);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_DisADCFreeze(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          brief Write GPIO configure ADC & Weak Pull-Down.
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_DisADCFreeze(uint8 uBmiIface)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_DIS_ADC_FREEZE,  0U,0u, (void *) NULL);
    }

    return uRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_SetADCDelay(uint8 uBmiIfaceIn)
 *********************************************************************************************************************/
/*! \brief          brief Write GPIO configure ADC & Weak Pull-Down.
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[in]
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_SetADCDelay(uint8 uBmiIface,  uint16 xTimeUs , uint8 uDevId)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIface];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = pBmiMgr->pBmcOps->service(pBmiMgr->pBmicMgr, BMI_SERVICE_SET_ADC_DELAY,  xTimeUs ,uDevId, (void *) NULL);
    }

    return uRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetBalTimeData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the get ballance time
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetBalTimeData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = (uint8)BMI_INVALID_GET_BUF;
        if((pBuffer!=NULL) && (wLength > 0U))
        {
            uRet = pBmiMgr->pBmcOps->decode_rx(pBmiMgr->pBmicMgr, BMI_SERVICE_BALANCING,BMI_CB_GET_BALTIME, pBuffer, wLength);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetBalStatData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the get ballance time
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetBalStatData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = (uint8)BMI_INVALID_GET_BUF;
        if((pBuffer!=NULL) && (wLength > 0U))
        {
            uRet = pBmiMgr->pBmcOps->decode_rx(pBmiMgr->pBmicMgr, BMI_SERVICE_BALANCING,BMI_CB_GET_BALSTAT, pBuffer, wLength);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetBalSWStatData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the get ballance time
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetBalSWStatData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = (uint8)BMI_INVALID_GET_BUF;
        if((pBuffer!=NULL) && (wLength > 0U))
        {
            uRet = pBmiMgr->pBmcOps->decode_rx(pBmiMgr->pBmicMgr, BMI_SERVICE_BALANCING,BMI_CB_GET_BALSWSTAT, pBuffer, wLength);
        }
    }

    return uRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetBalDoneStatData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the get ballance time
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetBalDoneStatData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = (uint8)BMI_INVALID_GET_BUF;
        if((pBuffer!=NULL) && (wLength > 0U))
        {
            uRet = pBmiMgr->pBmcOps->decode_rx(pBmiMgr->pBmicMgr, BMI_SERVICE_BALANCING,BMI_CB_GET_BALDONESTAT, pBuffer, wLength);
        }
    }

    return uRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetVcellActSumData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the Cell Voltage Data recieved from the devices
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetVcellActSumData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = (uint8)BMI_INVALID_GET_BUF;
        if((pBuffer!=NULL) && (wLength > 0U))
        {
            uRet = pBmiMgr->pBmcOps->decode_rx(pBmiMgr->pBmicMgr, BMI_SERVICE_GET_VCELL_ACTSUM,0U, pBuffer, wLength);
        }
    }

    return uRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetDieTempData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the Cell Voltage Data recieved from the devices
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetDieTempData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = (uint8)BMI_INVALID_GET_BUF;

        if((pBuffer!=NULL) && (wLength > 0U))
        {
            uRet = pBmiMgr->pBmcOps->decode_rx(pBmiMgr->pBmicMgr, BMI_SERVICE_GET_DIE_TEMP,0U, pBuffer, wLength);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagMainData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the Cell Voltage Data recieved from the devices
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagMainData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = (uint8)BMI_INVALID_GET_BUF;

        if((pBuffer!=NULL) && (wLength > 0U))
        {
            uRet = pBmiMgr->pBmcOps->decode_rx(pBmiMgr->pBmicMgr, BMI_SERVICE_GET_DIAG_MAIN,0U, pBuffer, wLength);
        }
    }

    return uRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagRdntData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the Cell Voltage Data recieved from the devices
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetDiagRdntData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = (uint8)BMI_INVALID_GET_BUF;

        if((pBuffer!=NULL) && (wLength > 0U))
        {
            uRet = pBmiMgr->pBmcOps->decode_rx(pBmiMgr->pBmicMgr, BMI_SERVICE_GET_DIAG_RDNT,0U, pBuffer, wLength);
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetI2CDataReadings(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the Cell Voltage Data recieved from the devices
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_Get_I2CData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = (uint8)BMI_INVALID_GET_BUF;

        if((pBuffer!=NULL) && (wLength > 0U))
        {
            uRet = pBmiMgr->pBmcOps->decode_rx(pBmiMgr->pBmicMgr, BMI_SERVICE_GET_I2C_DATA,0U, pBuffer, wLength);
        }
    }

    return uRet;
}



/**********************************************************************************************************************
 * FUNC(uint8, tibms_bmi_CODE) Bmi_GetI2CFaultData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          Decode the Cell Voltage Data recieved from the devices
 *                  This will be called once the data successfully recieved at the driver layer
 *
 *  \param[in]      uBmiIfaceIn: corresponding bmi interface
 *  \param[inOut]   pBuffer: buffer to fill in
 *  \param[in]      wLength: Length of the buffer
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post
 *  \return         returns the process state
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_GetI2CFaultData(uint8 uBmiIfaceIn, void *pBuffer, uint16 wLength)
{
    Bmi_ManagerType *pBmiMgr;
    uint8 uRet = (uint8)BMI_NOT_OK;

    pBmiMgr = &zBmiManager[uBmiIfaceIn];
    if(BMI_INTEGRITY_CHECK(pBmiMgr))
    {
        uRet = (uint8)BMI_INVALID_GET_BUF;

        if((pBuffer!=NULL) && (wLength > 0U))
        {
            uRet = pBmiMgr->pBmcOps->decode_rx(pBmiMgr->pBmicMgr, BMI_SERVICE_GET_I2C_FAULT,0U, pBuffer, wLength);
        }
    }

    return uRet;
}

#define TIBMS_BMI_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * End of File: tibms_bmi.c
 *********************************************************************************************************************/
