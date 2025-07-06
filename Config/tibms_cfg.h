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
 *  File:       tibms_cfg.h
 *  Project:    TIBMS
 *  Module:     CONFIG
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  TI BMS Pre-compile configurations
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05May2022    SEM                  0000000000000    Initial version
 * 01.00.01       03March2023  SEM                  0000000000000    Updated and removed all unncessary configs
 * 01.00.02       04Aug2023    SEM                  0000000000000    Updated the PMIC configuraiton and integration
 *
 *********************************************************************************************************************/

#ifndef TIBMS_CFG_H
#define TIBMS_CFG_H

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

#include "tibms_platform.h"
#include "tibms_types.h"
#include "emem_cfg.h"

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

/**	Major Software Config Version number */
#define TIBMS_CFG_MAJOR_VERSION             (0x01u)

/**	Minor Software Config Version number */
#define TIBMS_CFG_MINOR_VERSION             (0x01u)

/** Software Patch Config Version number */
#define TIBMS_CFG_PATCH_VERSION             (0x00u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

#define BMIF_BQ7971X_ENABLE                 (STD_ON)
#define PMIF_BQ7973X_ENABLE                 (STD_ON)
#define COMIF_BQ79600_ENABLE                (STD_ON)
#define COMIF_BQ7973X_ENABLE                (STD_ON)
#define COMIF_CC2662_ENABLE                 (STD_ON)

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

/** Configuration **/

#define TIBMS_NO_OF_BMI_IFACES              (7u)        /** Max Number of Battery Monitor Interfaces **/
#define TIBMS_NO_OF_PMI_IFACES              (1u)        /** Max Number of Pack Monitor Interfaces **/
#define TIBMS_NO_OF_COMIF_IFACES            (3u)        /** Max Number of Communicaiton interfaces including BMI and PMI **/

/** Interface 0, BMI APP Configuration **/

#define TIBMS_MAX_BMICS_IN_IFACE_0          (12u)       /** Max Number of Battery Monitor IC's in the Interface 0 **/
#define TIBMS_MAX_PMICS_IN_IFACE_0          (1u)        /** Max Number of Pack Monitor IC's in the Interface 0 **/                // 1 - for Testing the PMIC in the same chain
#define TIBMS_MAX_BRIDGE_IN_FACE_0          (1u)        /** Number of Communication IC's in the Interface 0 **/
#define TIBMS_MAX_AFES_IN_IFACE_0           (TIBMS_MAX_BMICS_IN_IFACE_0 + TIBMS_MAX_PMICS_IN_IFACE_0)
#define TIBMS_MAX_DEVS_IN_IFACE_0           (TIBMS_MAX_AFES_IN_IFACE_0 + TIBMS_MAX_BRIDGE_IN_FACE_0)

#define TIBMS_BMIC_CELLS_IFACE_0            (BMIAPP_NUM_OF_CELLS)       /** Cells Count in BMICs, and This is used at Application **/
#define TIBMS_BMIC_GPIOS_IFACE_0            (BMIAPP_NUM_OF_GPIOS)       /** GPIO Count in BMICs, and This is used at Application **/

/** Interface 1, PMI APP configuration **/

#define TIBMS_MAX_BMICS_IN_IFACE_1          (0u)        /** Max Number of Battery Monitor IC's in the Interface 0 **/
#define TIBMS_MAX_PMICS_IN_IFACE_1          (1u)        /** Max Number of Pack Monitor IC's in the Interface 0 **/
#define TIBMS_MAX_BRIDGE_IN_FACE_1          (0u)        /** Number of Communication IC's in the Interface 0 **/
#define TIBMS_MAX_AFES_IN_IFACE_1           (TIBMS_MAX_BMICS_IN_IFACE_1 + TIBMS_MAX_PMICS_IN_IFACE_1)
#define TIBMS_MAX_DEVS_IN_IFACE_1           (TIBMS_MAX_AFES_IN_IFACE_1 + TIBMS_MAX_BRIDGE_IN_FACE_1)

/** Interface 2, WBMS APP UART configuration **/

#define TIBMS_MAX_BMICS_IN_IFACE_2          (12u)       /** Max Number of Battery Monitor IC's in the Interface 0 **/
#define TIBMS_MAX_PMICS_IN_IFACE_2          (0u)        /** Max Number of Pack Monitor IC's in the Interface 0 **/
#define TIBMS_MAX_BRIDGE_IN_FACE_2          (0u)        /** Max Number of Communication IC's in the Interface 0 **/
#define TIBMS_MAX_AFES_IN_IFACE_2           (TIBMS_MAX_BMICS_IN_IFACE_2 + TIBMS_MAX_PMICS_IN_IFACE_2)
#define TIBMS_MAX_DEVS_IN_IFACE_2           (TIBMS_MAX_AFES_IN_IFACE_2 + TIBMS_MAX_BRIDGE_IN_FACE_2)

#define TIBMS_BMIC_CELLS_IFACE_2            (18u)       /** Cells Count in BMICs, and This is used at Application **/
#define TIBMS_BMIC_GPIOS_IFACE_2            (11u)       /** GPIO Count in BMICs, and This is used at Application **/

#define TIBMS_CFG_START_SIG                 (0x5CA1AB1EU)    /** Start Signature to validate the Configurations internally */
#define TIBMS_CFG_STOP_SIG                  (0x0D15EA5EU)    /** End Signature to validate the Configurations internally */

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

#define TIBMS_CFG_START_SEC_CONST
#include "Cdd_MemMap.h"

extern CONST(Bms_ConfigType, TIBMSCFG_CONST) zBmiConfigSet_PB_0;        /** Sample Configuration for Battery monitor usage **/
extern CONST(Bms_ConfigType, TIBMSCFG_CONST) zBmsConfigSet_PB_0;        /** Sample Configuration for Battery monitor usage and Pack monitor in one chain **/
extern CONST(Bms_ConfigType, TIBMSCFG_CONST) zBmiConfigSet_PB_1;        /** Sample Configuration for Battery monitor usage for Wireless BMS**/
extern CONST(Bms_ConfigType, TIBMSCFG_CONST) zPmiConfigSet_PB_0;        /** Sample Configuration for Pack monitor usage using Direct SPI communication **/

#define TIBMS_CFG_STOP_SEC_CONST
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /**TIBMS_CFG_H**/

/*********************************************************************************************************************
 * End of File: tibms_cfg.h
 *********************************************************************************************************************/
