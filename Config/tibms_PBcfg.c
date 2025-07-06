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
 *  File:       tibms_PBcfg.c
 *  Project:    TIBMS
 *  Module:     CONFIG
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Post Build configuration for the TI BMS
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05052022      SEM                 0000000000000    Initial version
 * 01.00.01       03032023      SEM                 0000000000000    Updates for the version
 * 01.01.00       04Aug2023     SEM                 0000000000000    Configuration Updates
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
#define TIBMS_CFG_C_MAJOR_VERSION             (0x01u)

/**	Minor Software Config C Version number */
#define TIBMS_CFG_C_MINOR_VERSION             (0x01u)

/** Software Patch Config C Version number */
#define TIBMS_CFG_C_PATCH_VERSION             (0x00u)

#if ((TIBMS_CFG_MAJOR_VERSION != TIBMS_CFG_C_MAJOR_VERSION) || \
     (TIBMS_CFG_MINOR_VERSION != TIBMS_CFG_C_MINOR_VERSION) || \
     (TIBMS_CFG_PATCH_VERSION != TIBMS_CFG_C_PATCH_VERSION))
#error "tibms: Config version numbers of tibms_PBcfg.c and tibms_cfg.h are inconsistent!"
#endif

/*********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Local CONST Object Definitions
 *********************************************************************************************************************/

#define TIBMS_PBCFG_START_SEC_CONST
#include "Cdd_MemMap.h"

STATIC CONST(InterfaceCfgType, TIBMSCFG_CONST) zBmiConfigIface_0[] =
{
    {
        &zBmiComifCfgWiredCfg_0,         /* pComIfCfg       */
        &zBmiCfg_0,                         /* pBmiCfg         */
        NULL,                               /* pPmiCfg         */

        BMI_IFACE_0,                        /* uIfaceCfg       */
        COMIF_WIRED,                        /* eComType        */
        DIR_COML2H,                         /* eReqComDirCfg   */
        (TIBMS_NO_OF_BMI_IFACES +1u),                                 /* uNDevsCfg       */

        TIBMS_NO_OF_BMI_IFACES,                                 /* uNBmcDevsCfg    */
        0u,                                 /* uNPmcDevsCfg    */
        TIBMS_NO_OF_BMI_IFACES,                                 /* uTopOfDevCfg    */

        0u,                                 /* uIsPmcInIfCfg   */
        0u,                                 /* uPmcInIfPosCfg  */
    }
};

STATIC CONST(InterfaceCfgType, TIBMSCFG_CONST) zBmsConfigIface_0[] =
{
    {
        &zBmiComifCfgWiredCfg_0,         /* pComIfCfg       */
        &zBmiCfg_0,                         /* pBmiCfg         */
        &zPmiCfgDaisyChain,                 /* pPmiCfg         */

        BMI_IFACE_0,                        /* uIfaceCfg       */
        COMIF_WIRED,                        /* eComType        */
        DIR_COMH2L,                         /* eReqComDirCfg   */
        8u,                                 /* uNDevsCfg       */

        6u,                                 /* uNBmcDevsCfg    */
        1u,                                 /* uNPmcDevsCfg    */
        2u,                                 /* uTopOfDevCfg    */

        1u,                                 /* uIsPmcInIfCfg   */
        1u,                                 /* uPmcInIfPosCfg  */
    }
};

STATIC CONST(InterfaceCfgType, TIBMSCFG_CONST) zBmiConfigIface_1[] =
{
    {
        &zBmiComifCfgWbmsCfg_0,             /* pComIfCfg       */
        &zBmiCfg_1,                         /* pBmiCfg         */
        NULL,                               /* pPmiCfg         */
		BMI_IFACE_1,						/* uIfaceCfg       */
        COMIF_WIRELESS,                     /* eComType        */
        DIR_COMH2L,                         /* eReqComDir      */
        2u,                                 /* uNDevsCfg       */
        2u,                                 /* uNBmcDevsCfg    */
        0u,                                 /* uNPmcDevsCfg    */
        6u,                                 /* uTopOfDevCfg    */

        0u,                                 /* uIsPmcInIfCfg   */
        0u,                                 /* uPmcInIfPosCfg  */
    }
};

STATIC CONST(InterfaceCfgType, TIBMSCFG_CONST) zPmiConfigIface_0[] =
{
    {
        &zPmiComifCfgDirectSpiCfg,          /* pComIfCfg       */
        NULL,                               /* pBmiCfg         */
        &zPmiCfg_0,                         /* pPmiCfg         */

        PMI_IFACE_0,                        /* uIfaceCfg       */
        COMIF_WIRED,                        /* eComType        */
        DIR_DIRECT,                         /* eReqComDirCfg   */
        1u,                                 /* uNDevsCfg       */

        0u,                                 /* uNBmcDevsCfg    */
        1u,                                 /* uNPmcDevsCfg    */
        1u,                                 /* uTopOfDevCfg    */

        1u,                                 /* uIsPmcInIfCfg   */
        0u,                                 /* uPmicDevIdCfg  */
    }
};
//
///*PMI Via Bridge*/
//STATIC CONST(InterfaceCfgType, TIBMSCFG_CONST) zPmiConfigIface_1[] =
//{
//    {
//        &zPmiComifCfgWiredBridgeSpiCfg,   //zPmiComifCfgDirectSpiCfg       /* pComIfCfg       */
//        NULL,                               /* pBmiCfg         */
//        &zPmiCfg_0,                         /* pPmiCfg         */
//
//        PMI_IFACE_0,                        /* uIfaceCfg       */
//        DIR_COMH2L,                        /* eComType        */
//        DIR_DIRECT,                         /* eReqComDirCfg   */
//        2u,                                 /* uNDevsCfg       */
//
//        0u,                                 /* uNBmcDevsCfg    */
//        1u,                                 /* uNPmcDevsCfg    */
//        1u,                                 /* uTopOfDevCfg    */
//
//        1u,                                 /* uIsPmcInIfCfg   */
//        0u,                                 /* uPmicDevIdCfg  */
//    }
//};

#define TIBMS_PBCFG_STOP_SEC_CONST
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

#define TIBMS_PBCFG_START_SEC_CONST
#include "Cdd_MemMap.h"

/** Exposed configurations to applications **/

CONST(Bms_ConfigType, TIBMSCFG_CONST) zBmiConfigSet_PB_0 =
{
    TIBMS_CFG_START_SIG,
    &zBmiConfigIface_0[0],
    TIBMS_CFG_STOP_SIG
};

CONST(Bms_ConfigType, TIBMSCFG_CONST) zBmsConfigSet_PB_0 =
{
    TIBMS_CFG_START_SIG,
    &zBmsConfigIface_0[0],
    TIBMS_CFG_STOP_SIG
};

CONST(Bms_ConfigType, TIBMSCFG_CONST) zPmiConfigSet_PB_0 =
{
    TIBMS_CFG_START_SIG,
    &zPmiConfigIface_0[0],
//	&zPmiConfigIface_1[0], //with bridge
    TIBMS_CFG_STOP_SIG
};

CONST(Bms_ConfigType, TIBMSCFG_CONST) zBmiConfigSet_PB_1 =
{
    TIBMS_CFG_START_SIG,
    &zBmiConfigIface_1[0],
    TIBMS_CFG_STOP_SIG
};

#define TIBMS_PBCFG_STOP_SEC_CONST
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * Local Function Prototypes
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Functions Definition
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * External Functions Definition
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * End of File: tibms_PBcfg.c
 *********************************************************************************************************************/
