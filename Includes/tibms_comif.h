/**********************************************************************************************************************
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
 *  File:       tibms_comif.h
 *  Project:    TIBMS
 *  Module:     COMIF
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for Communication interface
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05May2022    SEM                  0000000000000    Initial version
 * 01.01.00       04Aug2023    SEM                  0000000000000    Configuration updates
 *
 *********************************************************************************************************************/

#ifndef TIBMS_COMIFACE_H
#define TIBMS_COMIFACE_H

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

/**********************************************************************************************************************
 * Standard Header Files
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Other Header Files
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

#define COMIF_SW_MAJOR_VERSION                      (0x01u)
#define COMIF_SW_MINOR_VERSION                      (0x01u)
#define COMIF_SW_PATCH_VERSION                      (0x00u)

/** Texas Instruments Vendor ID*/
#define COMIF_VENDOR_ID                             (44u)

/** COMIF Driver Module ID       */
#define COMIF_MODULE_ID                             (255u)

/** COMIF Driver Instance ID */
#define COMIF_INSTANCE_ID                           (03u)

/**********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

#define WRITE_1BYTE                                 (1u)
#define WRITE_2BYTE                                 (2u)
#define WRITE_3BYTE                                 (3u)
#define WRITE_4BYTE                                 (4u)
#define READ_1BYTE                                  (1u)
#define READ_2BYTE                                  (2u)
#define READ_3BYTE                                  (3u)
#define READ_4BYTE                                  (4u)
#define READ_5BYTE                                  (5u)
#define READ_6BYTE                                  (6u)
#define READ_7BYTE                                  (7u)
#define READ_8BYTE                                  (8u)
#define READ_11BYTE                                 (11u)
#define READ_16BYTE                                 (16u)

/** The Register Command Lock is formed like below
 * < 8bit Reserved > <8 bits of CMD INFO > <16 bits of Reg Addr >
 * CMDINOF is <1 bit of Lock Info > < 4 bits of PRIO >
 * Register address is OR with prio and lock
 * If register address is used directly, this will go as normal priority
 **/

#define COMIF_DELAY_CMD_LEN                         (0x02u)

#define COMIF_DATA_MASK                             (0xFFFF)
#define COMIF_PRIO_EXT_MASK                         (0xFF)
#define COMIF_PRIO_MASK                             (0xF)
#define COMIF_PRIO_EXCL_MASK                        (0x7)
#define COMIF_PRIO_LOCK_MASK                        (0x8u)

#define COMIF_UNLOCK                                (0x00u << 3u)
#define COMIF_LOCK                                  (0x01u << 3u)

#define COMIF_GET_REG(qData)                        (uint16) (qData & COMIF_DATA_MASK)
#define COMIF_GET_INFO(qData)                       (uint8) ((qData >> 16u) & COMIF_PRIO_EXT_MASK)
#define COMIF_GET_PRIO(uData)                       (uint8) ((uData) & COMIF_PRIO_MASK)
#define COMIF_GET_PRIO_EXCL(uData)                  (uint8) ((uData >> 4u) & COMIF_PRIO_EXCL_MASK)
#define COMIF_GET_PRIO_LOCK(uData)                  (uint8) ((uData >> 4u) & COMIF_PRIO_LOCK_MASK)

#define COMIF_PRIO_NORMAL(qData)                    (qData | (COMIF_CMD_PRIO_NORMAL << 16u))
#define COMIF_PRIO_GPIO(qData)                      (qData | (COMIF_CMD_PRIO_GPIO << 16u))
#define COMIF_PRIO_EXCL_GPIO(qData, uExcl)          (qData | (COMIF_CMD_PRIO_GPIO_EXCL << 16u) |  \
                                                             ((uExcl|COMIF_CMD_PRIO_GPIO) << 20u))
#define COMIF_PRIO_CELL(qData)                      (qData | (COMIF_CMD_PRIO_CELL << 16u))
#define COMIF_PRIO_EXCL_CELL(qData, uExcl)          (qData | (COMIF_CMD_PRIO_CELL_EXCL << 16u) | \
                                                             ((uExcl|COMIF_CMD_PRIO_CELL) << 20u))

/**********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef enum Comif_ServiceIdType_Tag
{
    COMIF_SVCID_GETVERSIONINFO,
    COMIF_SVCID_INIT,
    COMIF_SVCID_CONFIG,
    COMIF_SVCID_CHECKINIT,
    COMIF_SVCID_DEINIT,
    COMIF_SVCID_NOTIFICATION,
    COMIF_SVCID_BMI_PROCESSRX,
    COMIF_SVCID_RECV_DATAPROC,
    COMIF_SVCID_HANDLE_TX,
    COMIF_SVCID_WAKEUP_PING,
    COMIF_SVCID_SET_DELAY,

    COMIF_SVCID_SGL_WRITE,
    COMIF_SVCID_SGL_READ,
    COMIF_SVCID_STK_WRITE,
    COMIF_SVCID_STK_READ,
    COMIF_SVCID_ALL_WRITE,
    COMIF_SVCID_ALL_READ,

    COMIF_SVCID_HANDLE,
    COMIF_SVCID_HANDLE_REQ,

    COMIF_SVCID_CMD_INIT,
    COMIF_SVCID_CMD_DEINIT,
    COMIF_SVCID_ASYNC_TX_PROC,
    COMIF_SVCID_SYNC_TX_PROC,
    COMIF_SVCID_CMD_RX_PROC,
    COMIF_SVCID_CMD_TX_PROC,
    COMIF_SVCID_CMD_CTRL_PROC,
    COMIF_SVCID_CMD_NOTIFY,
    COMIF_SVCID_CMD_CFGDATA,
    COMIF_SVCID_CMD_WAKEUP_PING,
    COMIF_SVCID_CMD_SET_DELAY,
    COMIF_SVCID_CMD_RDY_NOTIFY,
    COMIF_SVCID_CMD_SEQ_NOTIFY,
    COMIF_SVCID_CMD_RX_NOTIFY,
    COMIF_SVCID_CMD_DELAY_NOTIFY,
    COMIF_SVCID_CMD_TIMOEUT_NOTIFY,
    COMIF_SVCID_CMD_SCHEDULE,
    COMIF_SVCID_CMD_GETVCELL,
    COMIF_SVCID_CMD_COMM_CTRL,
    COMIF_SVCID_CMD_COMM_STARTUP,

    COMIF_SVCID_END
}
Comif_ServiceIdType;

typedef enum Comif_StatusType_Tag
{
    COMIF_OK,
    COMIF_NOT_OK,
    COMIF_PENDING,

    COMIF_INTEGRITY_FAILED,
    COMIF_FATAL_ERROR,
    COMIF_INVALID_CONFIG,
    COMIF_INVALID_IFACECFG,
    COMIF_ALREADY_INUSE,
    COMIF_INVALID_OPS_CFG,
    COMIF_INVALID_MGR,
    COMIF_DEVICE_INIT_FAILED,
    COMIF_ERROR_MEM_FAILED,

    COMIF_CMD_INTEGRITY_FAILED,
    COMIF_CMD_INVALID_WKUP,
    COMIF_CMD_INVALID_CFG,
    COMIF_CMD_INVALID_IFACE_CFG,
    COMIF_CMD_INVALID_MGR,
    COMIF_CMD_ALREADY_INUSE,
    COMIF_CMD_INVALID_COMMCFG,
    COMIF_CMD_INVALID_NWCFG,
    COMIF_CMD_INVALID_QUECFG,
    COMIF_CMD_INVALID_TPBUF,
    COMIF_CMD_INVALID_BSW,
    COMIF_CMD_INVALID_BSW_INIT,

    COMIF_CMD_NWOPMODE_FAILED,
    COMIF_CMD_SETMAINCFG_FAILED,
    COMIF_CMD_SETJOINMODE_FAILED,
    COMIF_CMD_SETDEVTBLCFG_FAILED,
    COMIF_CMD_STARTNETWORK_FAILED,

    COMIF_CMD_QUECFG_INVALID,
    COMIF_CMD_CFQUE_FAILED,
    COMIF_CMD_CFQGET_EMPTY,
    COMIF_CMD_CEQUE_FAILED,
    COMIF_CMD_REQUE_FAILED,
    COMIF_CMD_MTFQUE_FAILED,
    COMIF_CMD_MTPQUE_FAILED,
    COMIF_CMD_MTEQUE_FAILED,

    COMIF_CMD_MRFQUE_FAILED,
    COMIF_CMD_MREQUE_FAILED,
    COMIF_CMD_SPQUE_FAILED,

    COMIF_CMD_INVALID_READ,
    COMIF_CMD_INVALID_REQCMD,
    COMIF_CMD_INVALID_SVCCFG,
    COMIF_CMD_INVALID_SVCREQ,
    COMIF_CMD_INVALID_CMDDATA,
    COMIF_CMD_INVALID_REQDATA,
    COMIF_CMD_INVALID_NOTIFY,
    COMIF_CMD_CRC_FAILED,

    COMIF_CMD_UART_TXERROR,
    COMIF_CMD_SPI_TXERROR,
    COMIF_CMD_SPI_RXERROR,
    COMIF_CMD_INVALID_DRVSTATE,
    COMIF_CMD_INVALID_RXDATABUF,
    COMIF_CMD_CTRL_PROC_DATA,

    COMIF_CMD_CFGINIT_SUCCESS,
    COMIF_CMD_STARTUP_SUCCESS,

    COMIF_INIT_SUCCESS,
    COMIF_NORMAL_STATE,

    COMIF_STATUS_MAX
}
Comif_StatusType;

typedef enum Comif_CmdReqType_Tag
{
    REQTYPE_SINGLE_R = 0x00u,
    REQTYPE_SINGLE_W = 0x01u,
    REQTYPE_STACK_R = 0x02u,
    REQTYPE_STACK_W = 0x03u,
    REQTYPE_BROADCAST_R = 0x04u,
    REQTYPE_BROADCAST_W = 0x05u,
    REQTYPE_BROADCAST_W_REV = 0x06u,
    REQTYPE_GENERIC_W = 0x07u,
    REQTYPE_GENERIC_R = 0x08u,

    REQTYPE_CUSTOM_CMD_W = 0xF9u,
    REQTYPE_CUSTOM_ERROR = 0xFAu,
    REQTYPE_CUSTOM_COMCLEAR = 0xFBu,
    REQTYPE_CUSTOM_WAKEUP = 0xFCu,
    REQTYPE_CUSTOM_DELAY = 0xFDu,
    REQTYPE_CUSTOM_INTDELAY = 0xFEu
}
Comif_CmdReqType;

typedef enum Comif_CommadType_Tag
{
    COMIF_CMD_UNKNOWN,
    COMIF_CMD_WAKEUP,
    COMIF_CMD_DELAY,
    COMIF_CMD_SGLWRITE,
    COMIF_CMD_SGLREAD,
    COMIF_CMD_STKWRITE,
    COMIF_CMD_STKREAD,
    COMIF_CMD_ALLWRITE,
    COMIF_CMD_ALLREAD,
    COMIF_CMD_GNRCREAD,
    COMIF_CMD_GNRCWRITE,
    COMIF_CMD_ERROR,
    COMIF_CMD_COMCLEAR,
    COMIF_CMD_GETVERSION,

    COMIF_CMD_MAX
}
Comif_CommadType;

typedef enum Comif_CommadPrioType_Rag
{
    COMIF_CMD_PRIO_NORMAL = 0x0u,                    // This is the Low Priority
    COMIF_CMD_PRIO_GPIO = 0x1u,
    COMIF_CMD_PRIO_GPIO_EXCL = 0x2u,
    COMIF_CMD_PRIO_CELL = 0x3u,
    COMIF_CMD_PRIO_CELL_EXCL = 0x4u,
    COMIF_CMD_PRIO_PRIO = 0x5u,                      // Prio Or Retry Packets are kept here
    COMIF_CMD_PRIO_CTRL = 0x6u,                      // WM/WD Packets are kept here
    COMIF_CMD_PRIO_EXCL = 0x7u,                      // Critical Admit packets

    COMIF_CMD_PRIO_MAX = 0x8u                        // MAX of 7 (3bits) priority channel possible with this configuration
}
Comif_CommadPrioType;

typedef struct Comif_ReqCmdType_Tag
{
    const uint8 *pData;
    const ServiceCfgType *pServiceCfg;

    uint16 wRegAdr;
    uint16 wReqCmdSiz;

    uint8 uReqType;
    uint8 uGroupExclude;
    uint8 uDevId;
    uint8 uCmdInfo;
}
Comif_ReqCmdType;

/**********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

#define TIBMS_COMIF_START_SEC_CODE
#include "Cdd_MemMap.h"

FUNC(uint8, tibms_comif_CODE) Comiface_StartupInit(const Comif_ManagerType *pComifMgr, uint8 uNumofAfes);
FUNC(uint8, tibms_comif_CODE) Comiface_HandleRequest(const Comif_ManagerType *pComifMgr, uint8 uCmd, uint8 uNodeId,
                                                     void *pData);
FUNC(uint8, tibms_comif_CODE) Comiface_DataProc(const Comif_ManagerType *pComifMgr, const ServiceCfgType *pCommCfg,
                                                const uint8 *pRxData);

FUNC(uint8, tibms_comif_CODE) Comif_ComClearRequest(const Comif_ManagerType *pComifMgr);
FUNC(uint8, tibms_comif_CODE) Comif_BroadcastRead(const Comif_ManagerType *pComifMgr, uint8 uDevId, uint32 qRegAddr,
                                                   const ServiceCfgType *pRespData, uint8 uReadLen);
FUNC(uint8, tibms_comif_CODE) Comif_BroadcastWrite(const Comif_ManagerType *pComifMgr, uint8 uReverseEn, uint32 qRegAddr,
                                                   const uint8 *pData, uint8 uDataLen);
FUNC(uint8, tibms_comif_CODE) Comif_StackRead(const Comif_ManagerType *pComifMgr, uint8 uDevId, uint32 qRegAddr,
                                              const ServiceCfgType *pRespData, uint8 uReadLen);
FUNC(uint8, tibms_comif_CODE) Comif_StackWrite(const Comif_ManagerType *pComifMgr, uint8 uGroupEn, uint32 qRegAddr,
                                               const uint8 *pData, uint8 uDataLen);
FUNC(uint8, tibms_comif_CODE) Comif_SingleWriteError(const Comif_ManagerType *pComifMgr, uint8 uDevId, uint32 qRegAddr,
                                                     const uint8 *pData, uint8 uDataLen);
FUNC(uint8, tibms_comif_CODE) Comif_SingleRead(const Comif_ManagerType *pComifMgr, uint8 uDevId, uint32 qRegAddr,
											   const ServiceCfgType *pRespData, uint8 uReadLen);
FUNC(uint8, tibms_comif_CODE) Comif_SingleWrite(const Comif_ManagerType *pComifMgr, uint8 uDevId, uint32 qRegAddr,
                                                 const uint8 *pData, uint8 uDataLen);

FUNC(uint8, tibms_comif_CODE) Comif_GenericWrite(const Comif_ManagerType *pComifMgr, uint8 uDevId, uint32 qData,
                                                 const uint8 *pData, uint8 uDataLen);
FUNC(uint8, tibms_comif_CODE) Comif_GenericRead(const Comif_ManagerType *pComifMgr, uint8 uDataId, uint32 qData,
                                                const ServiceCfgType *pServiceCfg, const uint8 *pData, uint8 uReadLen);

FUNC(uint8, tibms_comif_CODE) Comif_SetDelay(const Comif_ManagerType *pComifMgr, uint32 qDelay);
FUNC(uint8, tibms_comif_CODE) Comiface_WakePing(const Comif_ManagerType *pComifMgr, uint32 qDelay);

FUNC(uint8, tibms_comif_CODE) Comif_HandleTransfer(const Comif_ManagerType *pComifMgr);
FUNC(uint8, tibms_comif_CODE) Comif_HandleRecieve(const Comif_ManagerType *pComifMgr);
FUNC(uint8, tibms_comif_CODE) Comif_HandleNotify(const Comif_ManagerType *pComifMgr, uint8 uType);
FUNC(Comif_ManagerType*, tibms_comif_CODE) Comif_Init(const Comif_ConfigType *pComifCfg, uint8 *pNfaultPin);
FUNC(uint8, tibms_comif_CODE) Comif_DeInit(Comif_ManagerType *pComifMgr);
FUNC(uint8, tibms_comif_CODE) Comif_GetVersion(const Comif_ManagerType *pComifMgr, VersionInfoType *pComifVer);
FUNC(uint8, tibms_comif_CODE) Comif_SetErrorDetails(uint8 uSvcId, uint8 uErrorId, uint32 qParam1, uint32 qParam2);

#define TIBMS_COMIF_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
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

#if (COMIF_BQ79600_ENABLE == STD_ON)
#include "bq79600.h"
#endif

#if (COMIF_BQ7973X_ENABLE == STD_ON)
#include "bq7973x.h"
#endif

#if (COMIF_CC2662_ENABLE == STD_ON)
#include "cc2662.h"
#endif

#endif /*TIBMS_COMIFACE_H*/

/**********************************************************************************************************************
 * End of File: tibms_comif.h
 *********************************************************************************************************************/
