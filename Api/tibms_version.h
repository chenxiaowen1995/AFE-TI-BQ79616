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
 *  File:       tibms_version.h
 *  Project:    TIBMS
 *  Module:     TIBMS
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  This file is to maintain overall version and updates
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05May2022    SEM                0000000000000    Initial version
 *********************************************************************************************************************/

#ifndef TIBMS_VERSION_H
#define TIBMS_VERSION_H

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

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

#define TIBMS_VERSION_MAJOR     (8)
#define TIBMS_VERSION_MINOR     (1)
#define TIBMS_VERSION_PATCH     (0)

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef struct VersionInfoType_Tag
{
    Std_VersionInfoType zFwVersion;
    Std_VersionInfoType zBmiVersion;
    Std_VersionInfoType zPmiVersion;
    Std_VersionInfoType zBmcVersion;
    Std_VersionInfoType zPmcVersion;
    Std_VersionInfoType zComifVersion;
    Std_VersionInfoType zCmdVersion;
}
VersionInfoType;

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

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

#endif /*TIBMS_VERSION_H*/

/*********************************************************************************************************************
 * Version  API      SDK Version            Description																	
 *********************************************************************************************************************
 * 1.0.0    0100                            Initial Version
 *
 * 2.0.0 						            Support UART
 *  							            Basic Voltage and temperature read has been added
 *
 * 3.0.0						            Support for DualSPI is added
 *
 * 4.0.0						            MemMap files has been added.
 *
 *								            Manual network start is default setting.
 *								            Bugs fixed which were observed during testing.
 *								            New Wired Software Integrated
 *
 * 5.0.0						            Support for SPI ASYNC Reception with 128 bytes fixed length has been added
 *								            WBMS Callback notification functions has been added
 *								            Macro has been added to disable / enable the manual network change
 *
 * 6.0.0						            Command aggregation support in CDD
 *								            New WBMS Config parameters added.
 *								            New Wired Software Integration
 *								            SPI Functionality are moved to tibms_Lcfg_x.c
 *								            Common API added for all WBMS Command and removed individual Bmi functions
 *								            Os Counters & Resources handling is moved tibms_Lcfg_x.c
 *								            B5 Bug fixes are included.
 *
 * 7.0.0						            SDK 2.0.3.9 support has been added
 *								            Get Firmware Version command added
 *								            Merged UART and SPI functionalities
 *								            Trace Functionality added
 *								            Selective joining bug fixed
 *								            UART GPIO Functionality Implemented
 *
 * 7.1.0						            Bq Diagnostics issue fixed (with or without switch case).
 *								            Selective joining now works with multiple of 1, 4, and all devices together.
 *								            New wired software integrated
 *								            Voltage & Temperature trace functionality added
 *
 * 7.1.1				 2.00.03.09         Command Lock issue fix in startup init function
 *								            Compilation error fix with Command aggregation STD_OFF
 *								            Improved que handling in delay notify function
 *
 * 8.0.0   				 2.00.04.07         Communication handling re-design
 *                                          Dual MAC QUE enabling, handling for SYNC and ASYN Messages
 * 
 * 8.1.0                 2.00.04.14         Optimized the error handling mechanism
 *                                          Resolved the Bug Fixes in 8.0.0
 *                                          Added the Enhanced OAD feature for Main and Device
 *                                          Added the New WD Command and Diag Commands 
 *********************************************************************************************************************/

/*********************************************************************************************************************
 *  End of File: tibms_version.h
 *********************************************************************************************************************/
