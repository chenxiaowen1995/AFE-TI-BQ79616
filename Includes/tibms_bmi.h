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
 *  File:       tibms_bmi.h
 *  Project:    TIBMS
 *  Module:     BMI
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for BMI interface
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05May2022    SEM                0000000000000    Initial version
 * 01.01.00       05May2022    SEM                0000000000000    Packmonitor Update in the same network
 *
 *********************************************************************************************************************/

#ifndef TIBMS_BMI_H
#define TIBMS_BMI_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************************************************************
 * Standard Header Files
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Other Header Files
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

#define BMI_SW_MAJOR_VERSION                (0x01u)
#define BMI_SW_MINOR_VERSION                (0x01u)
#define BMI_SW_PATCH_VERSION                (0x00u)

/** Texas Instruments Vendor ID*/
#define BMI_VENDOR_ID                       (44u)

/** BMI Driver Module ID       */
#define BMI_MODULE_ID                       (255u)

/** BMI Driver Instance ID */
#define BMI_INSTANCE_ID                     (1u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef enum Bmi_ServiceIdType_Tag
{
    BMI_SVCID_GETVERSIONINFO,
    BMI_SVCID_INIT,
    BMI_SVCID_CONFIG,
    BMI_SVCID_CHECKINIT,
    BMI_SVCID_VOLTAGE_READ,
    BMI_SVCID_GPIO_READ,
    BMI_SVCID_CELLBALANCE,
    BMI_SVCID_DEINIT,
    BMI_SVCID_STARTUPINIT,
    BMI_SVCID_NOTIFICATION,
    BMI_SVCID_BMI_RX_PROC,
    BMI_SVCID_BMI_RX_DECODE,
    BMI_SVCID_HANDLE_RX,
    BMI_SVCID_HANDLE_TX,
    BMI_SVCID_BMC_INIT,
    BMI_SVCID_BMC_DIAG_INIT,
    BMI_SVCID_BMC_DIAGFUNC_INIT,
    BMI_SVCID_BMC_DIAGSTARTUP_INIT,
    BMI_SVCID_BMC_DEINIT,
    BMI_SVCID_BMC_STARTUP_INIT,
    BMI_SVCID_BMC_AUTOADDRESS,
    BMI_SVCID_BMC_CFGDATA_INIT,

    BMI_SVCID_BMC_SERVICE_PROC,
    BMI_SVCID_BMC_RX_DATA_PROC,
    BMI_SVCID_BMC_DATA_DECODE,
    BMI_SVCID_BMC_CTRL_PROC,

    BMI_SVCID_BMC_GETVCELL,
    BMI_SVCID_BMC_GETGPIO,
    BMI_SVCID_BMC_SETCBTIME,
    BMI_SVCID_BMC_SETCB_CTRL,
    BMI_SVCID_BMC_CB_PROC,

    BMI_SVCID_BMC_COMM_PROC,
    BMI_SVCID_BMC_DIAG_PROC,
    BMI_SVCID_BMC_DIAG_SERVICE,

    BMI_SVCID_BMC_DIAG_STARTUP,
    BMI_SVCID_BMC_DIAG_BGX,
    BMI_SVCID_BMC_REFCAP_DIAG,
    BMI_SVCID_BMC_PWRBIST_DIAG,
    BMI_SVCID_BMC_ACOMP_FLTINJ,
    BMI_SVCID_BMC_DCOMP_FLTINJ,
    BMI_SVCID_BMC_DIAG_CBFET,
    BMI_SVCID_BMC_DIAG_OVUV_DAC,
    BMI_SVCID_BMC_OTUT_DAC_DIAG,
    BMI_SVCID_BMC_CBOW_FLTINJ,

    BMI_SVCID_BMC_DIAG_STATUS,
    BMI_SVCID_BMC_FAULT_SUMMARY,
    BMI_SVCID_BMC_GPIO_DIAG,
    BMI_SVCID_BMC_GPIO_OPNWR,
    BMI_SVCID_BMC_GPIO_ADJSHRT,
    BMI_SVCID_BMC_VC_OPN_DET,
    BMI_SVCID_BMC_VCCB_SHRT_DET,
    BMI_SVCID_BMC_VCELLGPIO_ACOMP,
    BMI_SVCID_BMC_DCOMP_DIAG,
    BMI_SVCID_BMC_DIE_TEMP,
    BMI_SVCID_BMC_GET_DIAG_D1,
    BMI_SVCID_BMC_GET_DIAG_D2,
    BMI_SVCID_BMC_DIAG_ABORT,
    BMI_SVCID_BMC_FAULT_ADC_MISC,
    BMI_SVCID_BMC_SPI_FAULT_DIAG,
    BMI_SVCID_BMC_OTP_PROGRAM,
    BMI_SVCID_FACR_TM,
    BMI_SVCID_BMC_OTP_ERR_DIAG,
    BMI_SVCID_BMC_OTP_STATE_DIAG,
    BMI_SVCID_BMC_CRC_STATE_DIAG,
    BMI_SVCID_BMC_FACTORY_CRC_DIAG,
    BMI_SVCID_BMC_FAULT_SYS_DIAG,
    BMI_SVCID_BMC_VLF_CRC_DIAG,
    BMI_SVCID_BMC_NFAULT,
    BMI_SVCID_BMC_COMM_DEBUG_DIAG,
    BMI_SVCID_BMC_BB_PLAU_DIAG,
    BMI_SVCID_BMC_CB_PLAU_DIAG,
    BMI_SVCID_BMC_GET_DIAG_MAIN_RDNT,
    BMI_SVCID_BMC_FAULT_PWR_DIAG,
    BMI_SVCID_BMC_VLF_FAULT_DIAG,
    BMI_SVCID_BMC_DIAG_OTUT_SC,
    BMI_SVCID_BMC_DIAG_OVUV_SC,
    BMI_SVCID_BMC_DIAG_OTUT_RR,
    BMI_SVCID_BMC_DIAG_SLEEP_FAULT,
    BMI_SVCID_BMC_DIAG_HW_RESET,
    BMI_SVCID_BMC_DIAG_DEV_ADDR,
    BMI_SVCID_MAX_COUNT
}
Bmi_ServiceIdType;

typedef enum Bmi_StatusType_Tag
{
    BMI_OK,
    BMI_NOT_OK,
    BMI_PENDING,

    BMI_INIT_FAILED,
    BMI_INVALID_CONFIG,
    BMI_ALREADY_INIT,
    BMI_ERROR_MEM_FAILED,
    BMI_INVALID_OPS_CFG,
    BMI_INVALID_EMEM_CFG,
    BMI_INVALID_IFACE_CFG,
    BMI_INVALID_BMI_CFG,
    BMI_INTEGRITY_FAILED,
    BMI_INVALID_GET_BUF,
    BMI_COMIF_INIT_FAILED,

    BMI_BMC_NOT_OK,
    BMI_BMC_INIT_FAILED,
    BMI_PMC_INIT_FAILED,
    BMI_PMC_CFGINIT_FAILED,
    BMI_BMC_INVALID_WKUP,
    BMI_BMC_INVALID_MGR,
    BMI_BMC_INVALID_DIAG_MGR,
    BMI_BMC_DEVCFG_FAILED,
    BMI_BMC_CTRL2_FAILED,
    BMI_BMC_GPIOCFG_FAILED,
    BMI_BMC_ADCCTRL_FAILED,
    BMI_BMC_OVUVCFG_FAILED,
    BMI_BMC_OTUTCFG_FAILED,
    BMI_BMC_INVALID_COMTYPE,
    BMI_BMC_INVALID_BSWCFG,
    BMI_BMC_INVALID_QUECFG,

    BMI_BMC_ILLEGALCTX,
    BMI_BMC_INVALID_CFG,
    BMI_BMC_INVALID_IFACECFG,
    BMI_BMC_INVALID_STATE,
    BMI_BMC_ALREADY_INUSE,
    BMI_BMC_INVALID_COMIFMGR,
    BMI_BMC_INTEGRITY_FAILED,
    BMI_BMC_DIR_SET_FAILED,
    BMI_BMC_TOP_STACK_FAILED,
    BMI_BMC_AUTOADDREN_FAILED,
    BMI_BMC_BPATCH_FAILED,
    BMI_BMC_ADDRASSIGN_FAILED,
    BMI_BMC_CHECKCFG_PENDING,
    BMI_BMC_INVALID_CTRL_CMD,
    BMI_BMC_INVALID_SVC_CMD,
    BMI_BMC_INVALID_RX_PROC,
    BMI_BMC_WAKEUP_CMD_FAILED,
    BMI_BMC_WAKEUP_STACK_FAILED,
    BMI_BMC_STACK_DELAY_FAILED,
    BMI_BMC_INVALID_CFG_DATA,
    BMI_BMC_INVALID_RX_DATA,
    BMI_BMC_DECODE_VCELL_PREV_DATA,
    BMI_BMC_DECODE_GPIO_PREV_DATA,
    BMI_BMC_DECODE_CELL_BALLANCING_PREV_DATA,
    BMI_BMC_INVALID_RX_BUFFER,
    BMI_BMC_INVALID_CBCTRL,
    BMI_BMC_INVALID_CBCTRL_CTX,

    BMI_BMC_COMM_INVALID_SVC_ID,

    BMI_BMC_DIAG_INIT_FAILED,
    BMI_BMC_DIAG_INVALID_BMCMGR,
    BMI_BMC_DIAG_INVALID_DIAGMGR,
    BMI_BMC_DIAG_INTEGRITY_FAILED,
    BMI_BMC_DIAG_INVALID_CMD,
    BMI_BMC_DIAG_SUBID_INVALID,
    BMI_BMC_DIAG_INVALID_DATA,
    BMI_BMC_DIAG_INVALID_EXEC_REQ,
    BMI_BMC_DIAG_QUE_FAILED,
    BMI_BMC_DIAG_QUE_INVALID_REQ,
    BMI_BMC_DIAG_STARTUP_ERROR,
    BMI_BMC_DIAG_EXEC_ERROR,
    BMI_BMC_DECODE_VBAT_PREV_DATA,
    BMI_BMC_DECODE_VREFCAP_PREV_DATA,
    BMI_BMC_DECODE_VCELLACTSUM_PREV_DATA,
    BMI_BMC_DECODE_DIETEMP_PREV_DATA,
    BMI_BMC_DECODE_DIAGMAIN_PREV_DATA,
    BMI_BMC_DECODE_DIAGRDNT_PREV_DATA,
    BMI_BMC_DECODE_I2CDATA_PREV_DATA,
    BMI_BMC_DECODE_I2CFAULT_PREV_DATA,
    BMI_BMC_ERROR_STATE             = BMI_STATUS_ERROR_STATE,
    BMI_INIT_PROGRESS               = BMI_STATUS_INIT_PROGRESS,
    BMI_AUTOADDR_SUCCESS            = BMI_STATUS_AUTOADDR_SUCCESS,
    BMI_BMC_CFGINIT_SUCCESS         = BMI_STATUS_CFGINIT_SUCCESS,
    BMI_BMC_STARTUP_SUCCESS         = BMI_STATUS_STARTUP_SUCCESS,
    BMI_DIAG_INIT_PROGRESS          = BMI_STATUS_DIAG_INIT_PROGRESS,
    BMI_DIAG_INIT_COMPLETED         = BMI_STATUS_DIAG_INIT_COMPLETED,
    BMI_INIT_SUCCESS                = BMI_STATUS_INIT_SUCCESS,
    BMI_BMC_NORMAL_STATE            = BMI_STATUS_NORMAL_STATE,

    BMI_STATUS_END
}
Bmi_StatusType;

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

#define BQ7971X_DEV_SLEEP                    (0u)
#define BQ7971X_DEV_SHUT2WAKEUP              (1u)
#define BQ7971X_DEV_SLP2WAKEUP               (2u)
#define BQ7971X_DEV_SOFT_RESET               (3u)
#define BQ7971X_DEV_HW_RESET                 (4u)
#define BQ7971X_DEV_SHUTDOWN                 (5u)
#define BQ7971X_DEV_IDLE                     (6u)
/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

#define TIBMS_BMI_START_SEC_CODE
#include "Cdd_MemMap.h"

FUNC(uint8, tibms_bmi_CODE) Bmi_ProcessRxData(const ServiceCfgType *pSvcCtx, const uint8 *pRxData, uint8 uResult);
FUNC(uint8, tibms_bmi_CODE) Bmi_SetErrorDetails(uint8 uSvcId, uint8 uErrorId, uint32 qParam1, uint32 qParam2);

#define TIBMS_BMI_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * Exported inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#if (BMIF_BQ7971X_ENABLE == STD_ON)
#include "bq7971x.h"
#endif

#endif /*TIBMS_BMI_H*/

/*********************************************************************************************************************
 * End of File: tibms_bmi.h
 *********************************************************************************************************************/
