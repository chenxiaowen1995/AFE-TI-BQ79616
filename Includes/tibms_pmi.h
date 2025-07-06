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
 *  File:       pmi.h
 *  Project:    TIBMS
 *  Module:     PMI
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for PMI interface
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05May2022    SEM                0000000000000    Initial version
 * 01.01.00       04Aug2023    SEM                0000000000000    Pack monitor updates for including daisychain
 *                                                                 Pack monitor updates for read data*
 *
 *********************************************************************************************************************/

#ifndef TIBMS_PMI_H
#define TIBMS_PMI_H

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

/**	Major Software Config C Version number */
#define PMI_SW_MAJOR_VERSION                (0x01u)

/**	Minor Software Config C Version number */
#define PMI_SW_MINOR_VERSION                (0x01u)

/** Software Patch Config C Version number */
#define PMI_SW_PATCH_VERSION                (0x00u)

/** Texas Instruments Vendor ID*/
#define PMI_VENDOR_ID                       (44u)

/** PMI Driver Module ID       */
#define PMI_MODULE_ID                       (255u)

/** PMI Driver Instance ID */
#define PMI_INSTANCE_ID                     (2u)


/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef enum Pmi_ServiceIdType_Tag
{
    PMI_SVCID_GETVERSIONINFO        = 1u,
    PMI_SVCID_INIT                  = 2u,
    PMI_SVCID_CONFIG                = 3u,
    PMI_SVCID_CHECKINIT             = 4u,
    PMI_SVCID_DEINIT                = 8u,
    PMI_SVCID_STARTUPINIT           = 9u,
    PMI_SVCID_NOTIFICATION          = 10u,
    PMI_SVCID_PMI_RX_PROC           = 11u,
    PMI_SVCID_PMI_RX_DECODE         = 12u,
    PMI_SVCID_HANDLE_RX             = 13u,
    PMI_SVCID_HANDLE_TX             = 14u,
    PMI_SVCID_PMC_INIT              = 15u,
    PMI_SVCID_PMC_DIAG_INIT         = 16u,
    PMI_SVCID_PMC_DIAGFUNC_INIT     = 17u,
    PMI_SVCID_PMC_DIAGSTARTUP_INIT  = 18u,
    PMI_SVCID_PMC_DEINIT            = 19u,
    PMI_SVCID_PMC_STARTUP_INIT      = 20u,
    PMI_SVCID_PMC_CFGDATA_INIT      = 21u,
    PMI_SVCID_PMC_GPIO_CFG_INIT     = 22u,
    PMI_SVCID_PMC_ADC_CFG_INIT      = 23u,
    PMI_SVCID_PMC_CC_CFG_INIT       = 24u,
    PMI_SVCID_PMC_SETSW_CFG_INIT    = 25u,
    PMI_SVCID_PMC_CTRL2_CFG_INIT    = 26u,
    PMI_SVCID_PMC_OC_CFG_INIT       = 27u,
    PMI_SVCID_PMC_OC_CTRL1_INIT     = 28u,

    PMI_SVCID_PMC_SERVICE_PROC      = 30u,
    PMI_SVCID_PMC_RX_DATA_PROC      = 31u,
    PMI_SVCID_PMC_RX_DECODE         = 32u,
    PMI_SVCID_PMC_CTRL_PROC         = 33u,
    PMI_SVCID_PMC_DIAG_PROC         = 42u,

    PMI_SVCID_PMC_GETPCURRENT       = 43u,
    PMI_SVCID_PMC_GETVF_VOLT        = 44u,
    PMI_SVCID_PMC_GETCP_VOLT        = 45u,
    PMI_SVCID_PMC_GETCC_ACC_CNT     = 46u,
    PMI_SVCID_PMC_CC_CLEAR          = 47u,
    PMI_SVCID_PMC_GET_VGPIO         = 48u,
    PMI_SVCID_PMC_GET_DIETEMP       = 49u,

    PMI_SVCID_INSULATION_DET        = 50u,
    PMI_SVCID_PMC_ENABLE_I2C        = 51u,
    PMI_SVCID_PMC_SET_I2C_CTRL      = 52u,
    PMI_SVCID_PMC_SET_I2C_DATA      = 53u,
    PMI_SVCID_PMC_GET_I2C_DATA      = 54u,
    PMI_SVCID_PMC_SET_OC_OUT        = 55u,
    PMI_SVCID_PMC_ENTR_COMM_DEBUG   = 56u,
    PMI_SVCID_PMC_SET_SPI_CTR       = 57u,
    PMI_SVCID_PMC_MSPI_CONF         = 58u,
    PMI_SVCID_PMC_DIAG_DEINIT       = 59u,
    PMI_SVCID_PMC_DIAG_SERVICE      = 60u,    
      
    PMI_SVCID_PMC_SET_SLP2WAKEUP ,
    PMI_SVCID_PMC_SET_SOFT_RESET ,
	PMI_SVCID_PMC_SET_HW_RESET ,

    PMI_SVCID_MAX_COUNT             = 127
}
Pmi_ServiceIdType;

typedef enum Pmi_StatusType_Tag
{
    PMI_OK,
    PMI_NOT_OK,
    PMI_PENDING,

    PMI_INIT_UNINIT,
    PMI_INIT_FAILED,
    PMI_INVALID_CONFIG,
    PMI_ILLEGAL_CONFIG,
    PMI_ALREADY_INIT,
    PMI_INVALID_OPS_CFG,
    PMI_INVALID_CHAIN_CFG,
    PMI_INVALID_EMEM_CFG,
    PMI_INTEGRITY_FAILED,
    PMI_ERROR_MEM_FAILED,
    PMI_INVALID_DECODE_BUF,

    PMI_COMIF_INIT_FAILED,

    PMI_PMC_NOT_OK,
    PMI_PMC_INIT_FAILED,
    PMI_PMC_INVALID_WKUP,
    PMI_PMC_INVALID_MGR,
    PMI_PMC_INVALID_DIAG_MGR,
    PMI_PMC_DEVCFG_FAILED,
    PMI_PMC_CTRL2_FAILED,
    PMI_PMC_GPIOCFG_FAILED,
    PMI_PMC_SETOC_FAILED,
    PMI_PMC_ADCCTRL_FAILED,
    PMI_PMC_CCCTRL_FAILED,
    PMI_PMC_SETSW_FAILED,
    PMI_PMC_OCCTRL1_FAILED,
    PMI_PMC_WAKEUP_STACK_FAILED,
    PMI_PMC_CONF_INIT_FAILED,

    PMI_PMC_ILLEGALCTX,
    PMI_PMC_INVALID_CFG,
    PMI_PMC_INVALID_BSWCFG,
    PMI_PMC_INVALID_IFACECFG,
    PMI_PMC_INVALID_QUECFG,
    PMI_PMC_INVALID_STATE,
    PMI_PMC_ALREADY_INUSE,
    PMI_PMC_INVALID_COMIFMGR,
    PMI_PMC_INTEGRITY_FAILED,
    PMI_PMC_DIR_SET_FAILED,
    PMI_PMC_TOP_STACK_FAILED,
    PMI_PMC_AUTOADDREN_FAILED,
    PMI_PMC_ADDRASSIGN_FAILED,
    PMI_PMC_CHECKCFG_PENDING,
    PMI_PMC_INVALID_CTRL_CMD,
    PMI_PMC_INVALID_SVC_CMD,
    PMI_PMC_INVALID_RX_PROC,
    PMI_PMC_WAKEUP_CMD_FAILED,
    PMI_PMC_STACK_DELAY_FAILED,
    PMI_PMC_INVALID_CFG_DATA,

    PMI_PMC_INVALID_RX_DATA,
    PMI_PMC_DECODE_RX_BUSY,
    PMI_PMC_DECODE_PC_PREV_DATA,
    PMI_PMC_DECODE_VF_PREV_DATA,
    PMI_PMC_DECODE_CP_PREV_DATA,
    PMI_PMC_DECODE_CCACCCNT_PREV_DATA,
    PMI_PMC_DECODE_GPIO_PREV_DATA,
    PMI_PMC_DECODE_DIETEMP_PREV_DATA,
    PMI_PMC_INVALID_CBCTRL,
    PMI_PMC_DIAG_INIT_FAILED,
    PMI_PMC_DIAG_INVALID_PMCMGR,
    PMI_PMC_DIAG_INVALID_DIAGMGR,
    PMI_PMC_DIAG_INTEGRITY_FAILED,
    PMI_PMC_DIAG_INVALID_CMD,
    PMI_PMC_DIAG_SUBID_INVALID,
    PMI_PMC_DIAG_INVALID_DATA,
    PMI_PMC_DIAG_INVALID_EXEC_REQ,
    PMI_PMC_DIAG_INVALID_BMCMGR,
    PMI_PMC_DIAG_QUE_FAILED,
    PMI_PMC_DIAG_QUE_INVALID_REQ,

    PMI_PMC_ERROR_STATE = PMI_STATUS_ERROR_STATE,
    PMI_INIT_PROGRESS = PMI_STATUS_INIT_PROGRESS,
    PMI_AUTOADDR_SUCCESS = PMI_STATUS_AUTOADDR_SUCCESS,
    PMI_PMC_CFGINIT_SUCCESS = PMI_STATUS_CFGINIT_SUCCESS,
    PMI_PMC_STARTUP_SUCCESS = PMI_STATUS_STARTUP_SUCCESS,
    PMI_DIAG_INIT_PROGRESS = PMI_STATUS_DIAG_INIT_PROGRESS,
    PMI_DIAG_INIT_COMPLETED = PMI_STATUS_DIAG_INIT_COMPLETED,
    PMI_INIT_SUCCESS = PMI_STATUS_INIT_SUCCESS,
    PMI_PMC_NORMAL_STATE = PMI_STATUS_NORMAL_STATE,

    PMI_STATUS_MAX
}
Pmi_StatusType;

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Pmi_SetErrorDetails(uint8 uSvcId, uint8 uErrorId, uint32 qParam1, uint32 qParam2);

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

#if (PMIF_BQ7973X_ENABLE == STD_ON)
#include "bq7973x.h"
#endif

#endif /*TIBMS_PMI_H*/

/*********************************************************************************************************************
 * End of File: pmi.h
 *********************************************************************************************************************/
