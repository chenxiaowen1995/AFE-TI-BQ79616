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
 *  File:       cc2662.h
 *  Project:    TIBMS
 *  Module:     COMIF
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for COMIF CC2662 interface
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05May2022    SEM                0000000000000    Initial version
 * 01.01.00       22June2023   SEM                0000000000000    Command Aggregation Implementation
 *                                                                 Performance Improvements
 * 02.00.00       31Oct2023    SEM                0000000000000    Code Optimization for B8.0 Release
 * 02.01.00       09Nov2023    MF                 0000000000000    Added OAD and New WD Cmd for 8.1 Release
 *
 *********************************************************************************************************************/

#ifndef CC2662_H
#define CC2662_H

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

#define CC2662_SW_MAJOR_VERSION                                 (0x02u)
#define CC2662_SW_MINOR_VERSION                                 (0x02u)
#define CC2662_SW_PATCH_VERSION                                 (0x00u)

/** Texas Instruments Vendor ID*/
#define CC2662_VENDOR_ID                                        (44u)

/** CC2662 Driver Module ID       */
#define CC2662_MODULE_ID                                        (255u)

/** CC2662 Driver Instance ID */
#define CC2662_INSTANCE_ID                                      (41u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

/* INIT(1) + REG(2) + DATA(1) + CRC(2) */
#define CC2662_REQ_FRAME_FIXED_LEN                  (6u)

/* LEN(1) + ID(1) + REG(2) + DATA(1) + CRC(2) */
#define CC2662_RSP_FRAME_FIXED_LEN                  (7u)

/* Data_Len(1) + DEV_ID(1) + REG_ADR(2) */
#define CC2662_RSP_FRAME_PREFIX_LEN                 (4u)

#define CC2662_DEV_ADR_RANGE_MAX                    (0x3Fu)

#define CC2662_MAC_NwkOpMode_Volatile               (0x00)
#define CC2662_MAC_NwkOpMode_Persistent             (0x01)
#define CC2662_NON_SELECTIVE                        (0x00)
#define CC2662_SELECTIVE                            (0x01)

/** Host Communication Commands **/
#define CC2662_WM_IDENTIFY                          (0x00)      //!< wMain Address
#define CC2662_WM_NUM_WD_CONN                       (0x01)      //!< Number of wDevices connected
#define CC2662_WM_NUM_TX_PACKETS                    (0x02)      //!< Number of packets transmitted
#define CC2662_WM_NUM_TX_FAILED_PACKETS             (0x03)      //!< Number of failed transmissions
#define CC2662_WM_TX_THROUGHPUT                     (0x04)      //!< Tx throughput
#define CC2662_WM_RX_THROUGHPUT                     (0x05)      //!< Rx throughput
#define CC2662_WD_NUM_RX_PACKETS                    (0x06)      //!< Number of packets received x wDevice
#define CC2662_WD_NUM_RX_TIMEOUT_EVENTS             (0x07)      //!< Number of received timeout events x wDevice
#define CC2662_WD_LATENCY                           (0x08)      //!< Round trip latency x wDevice
#define CC2662_WD_PER                               (0x09)      //!< Packet Error Rate x wDevice
#define CC2662_WD_BQ_RAW_FRAME                      (0x0A)      //!< Send a RAW command to the BQ device
#define CC2662_WD_STORAGE                           (0x0D) 
#define CC2662_WM_RESET_DEVICE                      (0x12)      //!< wMain reset
#define CC2662_WD_RSSI                              (0x13)      //!< RSSI x wDevice

/** Host Diagnostic Commands Description(A/Synchronous) **/
#define CC2662_APP_DIAG_SETMAINCONFIG               (0x40)
#define CC2662_APP_DIAG_OADREQ		                (0x41)
#define CC2662_APP_DIAG_STARTNETWORK                (0x42)
#define CC2662_APP_DIAG_SETPOWERMODE                (0x44)
#define CC2662_APP_DIAG_RUNDWM                      (0x45)
#define CC2662_APP_DIAG_DISCOVERREQ                 (0x46)
#define CC2662_APP_DIAG_SETNWKOPMODE                (0x47)
#define CC2662_APP_DIAG_SETJOINMODE                 (0x48)
#define CC2662_APP_DIAG_SETDEVTBLCFG                (0x49)
#define CC2662_APP_DIAG_UNPAIRREQ                   (0x4A)
#define CC2662_APP_DIAG_RESYNC                      (0x4B)
#define CC2662_APP_DIAG_SETNWTIME                   (0x4C)
#define CC2662_APP_DIAG_SETWKUPPRD                  (0x4D)
#define CC2662_APP_DIAG_SETPARAMS                   (0x4E)
#define CC2662_APP_DIAG_DFUREQ		                (0x4F)
#define CC2662_APP_DIAG_WM_REPAIRREQ                (0x4C)
#define CC2662_APP_DIAG_SETWKUPPRD                  (0x4D)
#define CC2662_APP_DIAG_GETFWVERSION                (0x30)
#define CC2662_APP_DIAG_GETNETWORKSTATS             (0x32)
#define CC2662_APP_DIAG_GETTXPACKETS                (0x33)
#define CC2662_APP_DIAG_GETHROUGHTPUT               (0x34)
#define CC2662_APP_DIAG_GETRXPACKETS                (0x35)
#define CC2662_APP_DIAG_GETRSSI                     (0x36)       // TODO: this is not there
#define CC2662_APP_DIAG_GETSTATS             		(0x38)
#define CC2662_APP_DIAG_GETNWTIME                   (0x3C)
#define CC2662_APP_DIAG_GETKLVDURATION              (0x3D)
#define CC2662_APP_DIAG_GETPARAMS                   (0x3E)
#define CC2662_APP_DIAG_DEVICEPAIRIND_CB            (0x20)
#define CC2662_APP_DIAG_OAD_CB  		            (0x21)
#define CC2662_APP_DIAG_STATECHANGE_CB              (0x22)
#define CC2662_APP_DIAG_TXCNF_CB                     (0x23)
#define CC2662_APP_DIAG_RXIND_CB                     (0x24)
#define CC2662_APP_DIAG_ERROR_CB                     (0x25)
#define CC2662_APP_DIAG_FORMATIONTIME_CB             (0x26)
#define CC2662_APP_DIAG_BQFAULT_CB      	         (0x27)
#define CC2662_APP_DIAG_RESET_CB                     (0x29)
#define CC2662_APP_DIAG_EXITPS_CB                    (0x2A)
#define CC2662_APP_DIAG_DFU_CB                       (0x2B)
#define CC2662_APP_DIAG_DISCOVERIND_CB               (0x2C)
#define CC2662_APP_DIAG_DISCOVERCNF_CB               (0x2D)


/** Host Diagnostic Commands (Asyn callback ) **/
#define CC2662_APP_DIAG_DEVICEPAIRIND_CB            (0x20)
#define CC2662_APP_DIAG_OAD_CB  		            (0x21)
#define CC2662_APP_DIAG_STATECHANGE_CB              (0x22)
#define CC2662_APP_DIAG_TXCNF_CB                    (0x23)
#define CC2662_APP_DIAG_RXIND_CB                    (0x24)
#define CC2662_APP_DIAG_ERROR_CB                    (0x25)
#define CC2662_APP_DIAG_FORMATIONTIME_CB            (0x26)
#define CC2662_APP_DIAG_BQFAULT_CB      	        (0x27)
#define CC2662_APP_DIAG_RESYNCCNF_CB      	        (0x28)
#define CC2662_APP_DIAG_RESET_CB                    (0x29)
#define CC2662_APP_DIAG_EXITPS_CB                   (0x2A)

#define CC2662_WM_POWERMODE_ACTIVE                  (0x00)
#define CC2662_WM_POWERMODE_KEEPALIVE               (0x01)
#define CC2662_CHIP_ID_SIZE                         (0x06)

#define CC2662_IDENTITY_RSP_LEN                     (10)
#define CC2662_NUMOFCONN_RSP_LEN                    (1)
#define CC2662_NUMOFTXPACKETS_RSP_LEN               (4)
#define CC2662_RESET_RSP_LEN                        (10)
#define CC2662_NUMOFTXFAILEDPACKETS_RSP_LEN         (4)
#define CC2662_TXTHROUGHPUT_RSP_LEN                 (4)
#define CC2662_RXTHROUGHPUT_RSP_LEN                 (4)
#define CC2662_NUMOFRXPACKETS_RSP_LEN               (4)
#define CC2662_NUMOFMISSEDRXPACKETS_RSP_LEN         (4)
#define CC2662_LATENCYOFNODE_RSP_LEN                (4)
#define CC2662_PEROFNODE_RSP_LEN                    (4)
#define CC2662_FWVERSION_RSP_LEN                    (4)
#define CC2662_RSSIOFNODE_RSP_LEN                   (4)
#define CC2662_SETMAINCONFIG_RSP_LEN                (1)
#define CC2662_STARTNETWORK_RSP_LEN                 (1)
#define CC2662_SETPOWERMODE_RSP_LEN                 (1)
#define CC2662_SETPOWERMODE_RSP_LEN                 (1)
#define CC2662_RUNDWM_RSP_LEN                       (1)
#define CC2662_DISCOVER_RSP_LEN                     (1)  
#define CC2662_SETPARAMS_RSP_LEN                    (2)
#define CC2662_SETNWKOPMODE_RSP_LEN                 (1)
#define CC2662_SETJOINMODE_RSP_LEN                  (1)
#define CC2662_SETDEVTBLCFG_RSP_LEN                 (1)
#define CC2662_UNPAIRREQ_RSP_LEN                    (1)
#define CC2662_RESYNC_RSP_LEN                       (1)
#define CC2662_REPAIR_RSP_LEN                       (1)
#define CC2662_SETNWTIME_RSP_LEN                    (4)
#define CC2662_NETWORKSTATS_RSP_LEN                 (9)
#define CC2662_TXPACKETS_RSP_LEN                    (8)
#define CC2662_THROUGHTPUT_RSP_LEN                  (8)
#define CC2662_RXPACKETS_RSP_LEN                    (9)
#define CC2662_RSSI_RSP_LEN                         (4)
#define CC2662_NWTIME_RSP_LEN                       (4)
#define CC2662_KLVDURATION_RSP_LEN                  (2)
#define CC2662_SETWKUPPRD_RSP_LEN                   (2)
#define CC2662_GETSTATS_RSP_LEN                     (37) 
#define CC2662_GETPARAMS_RSP_LEN                    (36) 
#define CC2662_OAD_REQ_RSP_LEN                      (2)
#define CC2662_OAD_FWVER_RSP_LEN                    (35)
#define CC2662_BQFAULT_CB_RSP_LEN                   (18)
#define CC2662_SENDWDSTORAGE_RSP_LEN                (8)

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef enum Cc2662_MacStatusType_Tag
{
    MAC_STATUS_SUCCESS = 0x00,                                          /** Function returned successfully **/
    MAC_STATUS_FAIL = 0x01,                                             /** Error occurred in the function **/
    MAC_STATUS_NWID_MISMATCH = 0x02,                                    /** Master ID of the received packet does not match the Master ID for the network **/
    MAC_STATUS_BUFFER_ERROR = 0x03,                                     /** Buffer sizing error **/
    MAC_STATUS_FT_ERROR = 0x04,                                         /** The frame type is not supported **/
    MAC_STATUS_NO_RESOURCE = 0x05,                                      /** Out of Memory Error **/
    MAC_STATUS_INVALID_PARAM = 0x06,                                    /** Invalid input parameter **/
    MAC_STATUS_INVALID_STATE = 0x07,                                    /** Invalid state **/
    MAC_STATUS_INIT_NV_FAILURE = 0x08,                                  /** NV initialization failure **/
    MAC_STATUS_NWCFG_MISMATCH = 0x09,                                   /** Network configuration mismatch **/
    MAC_STATUS_INVALID_JOINMODE = 0x0A,                                 /** Selective Joining - invalid mode **/
    MAC_STATUS_DUPLICATE_DEVID = 0x0B,                                  /** Selective Joining - device ID already exists **/
    MAC_STATUS_DUPLICATE_SLOTID = 0x0C,                                 /** Selective Joining - slot ID prev. alloc. to another device **/
    MAC_STATUS_INVALID_SLOTCFG = 0x0D,                                  /** Selective Joining - slot ID incorrect **/
    MAC_STATUS_DEVLIMIT_REACHED = 0x0E,                                 /** Selective Joining - exceeds number of uplink slots **/
    MAC_STATUS_SCAN_TIMEOUT = 0x0F,                                     /** WM: All WD not found by WM **/
    MAC_STATUS_PAIR_TIMEOUT = 0x10,                                     /** (Not currently supported) Pairing Req sent, but no response from WD **/
    MAC_STATUS_SYNC_INPROG = 0x11,                                      /** Sync in progress for a sync lost device **/
    MAC_STATUS_FALLBACK_INPROG = 0x12,                                  /** Dual WM fallback in progress **/
}
Cc2662_MacStatusType;

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

#define CC2662_START_SEC_CODE
#include "Cdd_MemMap.h"

FUNC(uint8, cc2662_CODE) Cc2662_Control(void *pComifCtx, uint8 uCmd, void *pData);
FUNC(uint8, cc2662_CODE) Cc2662_HandleTransfer(void *pComifCtx);
FUNC(uint8, cc2662_CODE) Cc2662_HandleRecieve(void *pComifCtx);
FUNC(uint8, cc2662_CODE) Cc2662_Notify(void *pComifCtx, uint8 uNotifyType);
FUNC(void*, cc2662_CODE) Cc2662_Init(const void *pComCfgCtx, Emem_StasticsType *pEmemCfg, const ServiceCfgType *pComm);
FUNC(uint8, cc2662_CODE) Cc2662_Deinit(void *pComifCtx);

#define CC2662_STOP_SEC_CODE
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

#include "cc2662_cfg.h"

#endif /*CC2662_H*/

/*********************************************************************************************************************
 * End of File: cc2662.h
 *********************************************************************************************************************/
