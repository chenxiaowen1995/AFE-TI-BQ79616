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
 *  File:       tibms_com.h
 *  Project:    TIBMS
 *  Module:     TIBMS
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  API's for the TI BMS Communication functionalities
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       08April2023   SEM                 0000000000000    Initial version
 * 01.01.00       16Nov2023     MF                  0000000000000    New WD Commands and OAD
 *********************************************************************************************************************/

#ifndef TIBMS_COMM_H
#define TIBMS_COMM_H

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

#include "tibms_cfg.h"

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

#define TIBMS_COM_API_MAJOR_VERSION             (0x01u)
#define TIBMS_COM_API_MINOR_VERSION             (0x01u
#define TIBMS_COM_API_PATCH_VERSION             (0x00u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef enum CommServiceType_Tag
{
    COMM_GETIDENTITY,
    COMM_GETNUMOFCONN,
    COMM_GETNUMOFTXPACKETS,
	COMM_GETNUMOFTXFAILEDPACKETS,
	COMM_GETTXTHROUGHPUT,
	COMM_GETRXTHROUGHPUT,
	COMM_GETNUMOFRXPACKETS,
	COMM_GETNUMOFMISSEDRXPACKETS,
	COMM_GETLATENCYOFNODE,
	COMM_GETPEROFNODE,
	COMM_SENDBQRAWFRAME,
	COMM_SENDWDSTORAGE,
	COMM_RESETDEVICE,
	COMM_GETRSSIOFNODE,
    
    COMM_APP_DIAG_SETMAINCONFIG,
	COMM_APP_DIAG_OADREQ,
	COMM_APP_DIAG_STARTNETWORK,
	COMM_APP_DIAG_SETPOWERMODE,
	COMM_APP_DIAG_RUNDWM,
	COMM_APP_DIAG_DISCOVERREQ,
	COMM_APP_DIAG_SETNWKOPMODE,
	COMM_APP_DIAG_SETJOINMODE,
	COMM_APP_DIAG_SETDEVTBLCFG,
	COMM_APP_DIAG_UNPAIRREQ,
	COMM_APP_DIAG_RESYNC,
	COMM_APP_DIAG_WM_REPAIRREQ,
	COMM_APP_DIAG_SETWKUPPRD,
	COMM_APP_DIAG_SETPARAMS,
	COMM_APP_DIAG_DFUREQ,
	COMM_APP_DIAG_GETFWVERSION,
	COMM_APP_DIAG_GETNETWORKSTATS,
	COMM_APP_DIAG_GETTXPACKETS,
	COMM_APP_DIAG_GETHROUGHTPUT,
	COMM_APP_DIAG_GETRXPACKETS,
	COMM_APP_DIAG_GETSTATS,
	COMM_APP_DIAG_GETPARAMS,
	COMM_APP_DIAG_GETKLVDURATION,
	COMM_APP_DIAG_DEVICEPAIRIND_CB,
    COMM_APP_DIAG_OAD_CB,
    COMM_APP_DIAG_STATECHANGE_CB,
    COMM_APP_DIAG_TXCNF_CB,
	COMM_APP_DIAG_APP_DIAG_ERROR_CB,
	COMM_APP_DIAG_FORMATIONTIME_CB,
	COMM_APP_DIAG_BQFAULT_CB,
	COMM_APP_DIAG_RESYNCCNF_CB,
	COMM_APP_DIAG_RESET_CB,
	COMM_APP_DIAG_EXITPS_CB,
	COMM_APP_DIAG_DFU_CB,
	COMM_APP_DIAG_DISCOVERIND_CB,
	COMM_APP_DIAG_DISCOVERCNF_CB,
	
    COMM_APP_USER_CB, 
    COMM_RESET,
    COMM_SLEEP,
	COMM_SERVICE_END,
}
CommServiceType;

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_WmServiceRequest(uint8 uBmiIfaceIn, uint8 uCmd, uint8 uNodeId, void *pData);

/*********************************************************************************************************************
 * Exported Inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/
// WBMS Commands
#define Bmi_WmGetIdentity(uIface)          			     Bmi_WmServiceRequest(uIface,COMM_GETIDENTITY,0,NULL)
#define Bmi_WmGetNumOfConn(uIface)          		     Bmi_WmServiceRequest(uIface,COMM_GETNUMOFCONN,0,NULL)
#define Bmi_WmGetRssiOfNode(uIface, uNodeId)             Bmi_WmServiceRequest(uIface,COMM_GETRSSIOFNODE,uNodeId,NULL)
#define Bmi_WmGetNumOfTxPackets(uIface)                  Bmi_WmServiceRequest(uIface,COMM_GETNUMOFTXPACKETS,0,NULL)
#define Bmi_WmGetNumOfTxFailedPackets(uIface)            Bmi_WmServiceRequest(uIface,COMM_GETNUMOFTXFAILEDPACKETS,0,NULL)
#define Bmi_WmGetTxThroughtput(uIface)                   Bmi_WmServiceRequest(uIface,COMM_GETTXTHROUGHPUT,0,NULL)
#define Bmi_WmGetNumOfRxPackets(uIface, uNodeId)         Bmi_WmServiceRequest(uIface,COMM_GETNUMOFRXPACKETS,uNodeId,NULL)
#define Bmi_WmGetRxThroughput(uIface)                    Bmi_WmServiceRequest(uIface,COMM_GETRXTHROUGHPUT,0,NULL)
#define Bmi_WmGetNumOfMissedRxPackets(uIface, uNodeId)   Bmi_WmServiceRequest(uIface,COMM_GETNUMOFMISSEDRXPACKETS,uNodeId,NULL)
#define Bmi_WmGetLatencyOfNode(uIface, uNodeId)          Bmi_WmServiceRequest(uIface,COMM_GETLATENCYOFNODE,uNodeId,NULL)
#define Bmi_WmGetPerOfNode(uIface, uNodeId)              Bmi_WmServiceRequest(uIface,COMM_GETPEROFNODE,uNodeId,NULL)
#define Bmi_WmResetDevice(uIface)                        Bmi_WmServiceRequest(uIface,COMM_RESETDEVICE,0,NULL)
#define Bmi_WmAppDiagGetTxPackets(uIface)       		 Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_GETTXPACKETS,0,NULL)
#define Bmi_WmAppDiagGetThroughPut(uIface)               Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_GETHROUGHTPUT,0,NULL)
#define Bmi_WmAppDiagGetRxPackets(uIface, uNodeId)       Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_GETRXPACKETS,uNodeId,NULL)
#define Bmi_WmAppDiagGetNetworkstats(uIface,uNodeId)     Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_GETNETWORKSTATS,uNodeId,NULL)
#define Bmi_WmAppGetklvduration(uIface)                  Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_GETKLVDURATION,0,NULL)
#define Bmi_WmAppDiagGetFwVersion(uIface)                Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_GETFWVERSION,0,NULL)
#define Bmi_WmAppDiagSetwkupprd(uIface, uWakeUpPrd)      Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_SETWKUPPRD,0,uWakeUpPrd)
#define Bmi_WmAppDiagUnpairReq(uIface)                   Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_UNPAIRREQ,0,NULL)
#define Bmi_WmSendBqRawFrame(uIface,uBqPayload)          Bmi_WmServiceRequest(uIface,COMM_SENDBQRAWFRAME,0,uBqPayload)
#define Bmi_WmWDStorageFrame(uIface)                     Bmi_WmServiceRequest(uIface,COMM_SENDWDSTORAGE,0,NULL)
#define Bmi_WmAppDiagSetPowerMode(uIface,uPowerMode)     Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_SETPOWERMODE,0,uPowerMode)
#define Bmi_WmAppDiagOADReq(uIface,zNewFwData)           Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_OADREQ,0,zNewFwData)
#define Bmi_WmAppDiagUnpairReq(uIface)                   Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_UNPAIRREQ,0,NULL)
#define Bmi_WmAppDiagReSync(uIface)                      Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_RESYNC,0,NULL)
#define Bmi_WmAppDiagSetMainConfig(uIface)               Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_SETMAINCONFIG,0,NULL)
#define Bmi_WmAppDiagStartNetwork(uIface)                Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_STARTNETWORK,0,NULL)
#define Bmi_WmAppDiagRunDwm(uIface)                      Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_RUNDWM,0,NULL)
#define Bmi_WmAppDiagDiscoverReq(uIface)                 Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_DISCOVERREQ,0,NULL)
#define Bmi_WmAppDiagSetJoinMode(uIface)                 Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_SETJOINMODE,0,NULL)
#define Bmi_WmAppDiagSetDevTblCfg(uIface)                Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_SETDEVTBLCFG,0,NULL)
#define Bmi_WmAppDiagWmRepairReq(uIface)                 Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_WM_REPAIRREQ,0,NULL)
#define Bmi_WmAppDiagSetParams(uIface)                   Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_SETPARAMS,0,NULL)
#define Bmi_WmAppDiagDfuReq(uIface)                      Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_DFUREQ,0,NULL)
#define Bmi_WmAppDiagGetStats(uIface,uNodeId, StatsType) Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_GETSTATS,uNodeId,StatsType)
#define Bmi_WmAppDiagGetParams(uIface, wParamType)       Bmi_WmServiceRequest(uIface,COMM_APP_DIAG_GETPARAMS,0,wParamType)




//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /*TIBMS_COMM_H*/

/*********************************************************************************************************************
 * End of File: tibms_diag.h
 *********************************************************************************************************************/
