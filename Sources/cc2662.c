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
 *  File:       cc2662.c
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
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Standard Header Files
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Other Header Files
 *********************************************************************************************************************/

#include "tibms_api.h"
#include "tibms_comif.h"
#include "tibms_bmi.h"
#include "tibms_utils.h"

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

/**	Major Software Config C Version number */
#define CC2662_C_MAJOR_VERSION                      (0x02u)

/**	Minor Software Config C Version number */
#define CC2662_C_MINOR_VERSION                      (0x02u)

/** Software Patch Config C Version number */
#define CC2662_C_PATCH_VERSION                      (0x00u)

#if((CC2662_C_MAJOR_VERSION != CC2662_CFG_MAJOR_VERSION) || \
     (CC2662_C_MINOR_VERSION != CC2662_CFG_MINOR_VERSION) || \
	 (CC2662_C_PATCH_VERSION != CC2662_CFG_PATCH_VERSION))
#error "tibms: Config version numbers of cc2662.c and cc2662_cfg.h are inconsistent!"
#endif

#if((CC2662_SW_MAJOR_VERSION != CC2662_C_MAJOR_VERSION) || \
     (CC2662_SW_MINOR_VERSION != CC2662_C_MINOR_VERSION) || \
	 (CC2662_SW_PATCH_VERSION != CC2662_C_PATCH_VERSION))
#error "tibms: Config version numbers of cc2662.c and cc2662.h are inconsistent!"
#endif

/*********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

#define CC2662_GUARD_START                          (0xCCA1AB1EU)
#define CC2662_GUARD_END                            (0xCAFECEEDU)
#define CC2662_INIT                                 (0xCAU)

#define CC2662_DELAY_CMD_LEN                        (0x02u)

#define CC2662_WRITE_DATA_LEN_MIN                   (0x01u)
#define CC2662_WRITE_DATA_LEN_MAX                   (0x08u)
#define CC2662_READ_DATA_LEN_MAX                    (0x200u)
#define CC2662_READ_DATA_LEN_MIN                    (0x01u)
#define CC2662_DEV_ADR_RANGE_MAX                    (0x3Fu)
#define CC2662_READ_LEN_OFFSET                      (0x01u)
#define CC2662_REQUEST_BYTE_NUM_MASK                (0x7Fu)

#define CC2662_TXN_TIMEOUT_FACTOR                   (CC2662_USER_CFG_MAX_PKT_TX_RETRIES*2)

#define NPI_STATUS_OK                               (0x0<<0)
#define NPI_STATUS_PENDING                          (0x1<<0)
#define NPI_STATUS_INCOMPLETE                       (0x1<<1)
#define NPI_STATUS_CRC_ERR                          (0x1<<2)
#define NPI_STATUS_FORMAT_ERR                       (0x1<<3)

#define NPI_MAX_RESPLEN                             (0x100U)
#define NPI_BUF_RESPLEN                             (0x10)
#define NPI_MAX_REQLEN                              (0x100)

#if(CC2662_CMD_AGGREGATION == STD_ON )
#define NPI_CMD_AGGR_MAX                            (0x08u)
#define NPI_CMD_AGGR_MAX_READCMD                    (0x02u)
#else
#define NPI_CMD_AGGR_MAX                            (0x01u)
#define NPI_CMD_AGGR_MAX_READCMD                    (0x01u)
#endif

#if(CC2662_MAC_QUE_ENABLED == STD_ON)
#define NPI_MAC_TXQUE_CFG                           (0x02u)
#else
#define NPI_MAC_TXQUE_CFG                           (0x01u)
#endif

#define NPI_MAC_RXQUE_CFG                           (0x02u)
#define NPI_MAC_DATA_MAX_LEN                        (NPI_MAX_RESPLEN)
#define NPI_MAC_TXQUE_RXPEND_MAX                    (NPI_CMD_AGGR_MAX_READCMD)

#define NPI_MSG_SOF                                 (0xFEU)
#define NPI_HDR_LEN                                 (0x04u)
#define NPI_MSG_START_LEN                           (0x01u)
#define NPI_FCS_LEN                                 (0x01u)
#define NPI_PROTO_LEN                               (0x06u)
#define NPI_MIN_MSG_LEN                             (0x07u)

#define NPI_FRAME_HEADER_LEN                        (0x06u)
#define NPI_FRAME_STATUS_LEN                        (0x07u)
#define NPI_FRAME_FIXED_LEN                         (NPI_HDR_LEN+NPI_FCS_LEN)
#define NPI_FRAME_TIMESTAMP_LEN                     (0x05u)

#define NPI_LENGTH_LSB_IDX                          (0x01u)
#define NPI_LENGTH_MSB_IDX                          (0x02u)
#define NPI_CMDTYPE_IDX                             (0x03u)
#define NPI_CMDID_IDX                               (0x04u)
#define NPI_NODEID_IDX                              (0x05u)
#define NPI_BQPAYLOAD_IDX                           (0x06u)
#define NPI_BQCMD_IDX                               (0x08u)
#define NPI_MAC_STATUS_IDX                          (0x05u)

#define NPI_BQ_HDR_LEN                              (0x07u)
#define NPI_BQ_CMD_IDX                              (0x01u)
#define NPI_BQ_RESP_CMD_IDX                         (0x02u)

#define NPI_ASYNC_REQ                               (0x5AU)
#define NPI_SYNC_REQ                                (0x3AU)
#define NPI_ASYNC_RESP                              (0x5AU)
#define NPI_SYNC_RESP                               (0x7AU)

#define NPI_TIMESTAMP_SIZE                          (0x0C)

#define NPI_MAC_TABLE_SIZE                          (128u)      /** MAC Table Size for 8 Devices **/
#define NPI_MACTABLE_SLOT_SIZE                      (9u)
#define NPI_MACTABLE_DEVS_INSLOT                    (15u)       /** 14x9 devices can set with one Slot, 15 devices for incrementing the slots **/
#define NPI_OAD_BUF_SIZE                            (0x100u)

/*********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

#define CC2662_INTEGRITY_CHECK(pCc2662Mgr)          ((NULL != pCc2662Mgr) && \
                                                    (CC2662_GUARD_START == pCc2662Mgr->qGuardStart) && \
                                                    (CC2662_GUARD_END == pCc2662Mgr->qGuardEnd) && \
                                                    (CC2662_INIT == pCc2662Mgr->uInit))

#define Cc2662_FrameInitpack(type, group, size)     \
                        ((uint8)0x80u | (((type) & 7u) << 4) | (((group) & 1u) << 3) | ((((uint8)size) - 1u) & 7u))

#define Cc2662_GetCmdType(pData)                    (((pData)[0u] >> 4u) & 0x07u)
#define Cc2662_GetCmdDevAdr(pData)                  ((pData)[1u] & 0x3Fu)
#define Cc2662_GetCmd16bit(pData, Pos)              (((uint16)((pData)[Pos]) << 8u) | ((uint16)((pData)[Pos+ 1u])))
#define Cc2662_GetReqCmdData(pData, pos)            (((pData)[pos] & 0x7Fu))
#define Cc2662_GetRespCmdData(pData)                (((pData)[0u] & 0x7Fu) + 1u)
#define Cc2662_GetRespCmdCrc(pData, Pos)            (((uint16)(pData)[(Pos) + 1u] << 8u) | ((uint16)(pData)[(Pos)]))
#define Cc2662_GetRespCmdDataByte(pData, offset)    ((pData)[4u + (offset)])

/*********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

typedef enum Cc2662_NpiRespType_Tag
{
    NPI_RESP_WAIT,
    NPI_RESP_RETRY,
    NPI_RESP_CNFWAIT,
    NPI_RESP_VALID,
    NPI_RESP_COMPLETE,
}
Cc2662_NpiRespType;

typedef struct Cc2662_TxDataType_Tag
{
    ctPeQueueType zTxPendRespQue;
    void *pTxQueRxPendMem[NPI_MAC_TXQUE_RXPEND_MAX+1u];

    uint8 npiTxBuf[NPI_MAC_DATA_MAX_LEN];
    uint32 wTxnTimeOut:16;
    uint32 uTxLength:8;
    uint32 uRespPend:6;
    uint32 uReqPend:1;
    uint32 uReqActive:1;
}
Cc2662_TxDataType;

typedef struct Cc2662_TxTransType_Tag
{
    void *pMacTxQueFreeMem[NPI_MAC_TXQUE_CFG+1u];
    void *pMacTxQueExecMem[NPI_MAC_TXQUE_CFG+1u];
    ctPeQueueType zMacTxFreeQue;
    ctPeQueueType zMacTxExecQue;

    Cc2662_TxDataType zMacTxData[NPI_MAC_TXQUE_CFG];
    Cc2662_TxDataType zSyncTxData;
}
Cc2662_TxTransType;

typedef struct Cc2662_ContextType_Tag
{
    uint32 qGuardStart;

    const Basic_ConfigType *pBswCfg;                                                /** BSW Configuration - wrapper for Platform dependancy **/
    const ServiceCfgType *pCommCfg;                                                 /** Communication Service Configuration - this is specific to CC2662 **/
    const Cc2662_NwCfgType *pNwCfg;                                                 /** Network configuraiton for WBMS **/
    Emem_StasticsType *pEmemStats;                                                  /** Stastics for seeing the functionality **/

    ctPeQueueType zCmdQueFree;                                                      /** QUEs for handling the communication **/
    ctPeQueueType zCmdQueExec[COMIF_CMD_PRIO_MAX];
    ctPeQueueType zCmdReadQueExec;

    Cc2662_TxTransType zTransTd;                                                    /** MAC RX and TX QUE **/

    uint8 *pNpiTpBuf;                                                               /** TP buffer allocation for the aggregating the responses **/
    uint8 npiRxBuf[NPI_MAX_RESPLEN];                                                /** Buffer allocation for the recieved responses **/
    uint8 npiGenBuf[NPI_MAX_MACTABLE_IDX][NPI_MAC_TABLE_SIZE];                   /** MAC Table Allocation **/
    uint8 npiWdBuf[NPI_OAD_BUF_SIZE];

    float32 fTimestampInMs[NPI_TIMESTAMP_SIZE];                                     /** Timestamp **/
    uint32 qNwFormationTime;                                                        /** Network Formation time **/
    uint32 qTimeInNanoS;

    uint16 wTimeOutCfg;
    uint16 wNpiRxBufLenCfg;
    uint16 wNpiRxBufLen;
    uint16 wNpiRxBufIdx;

    uint16 wBqRxBufLen;
    uint16 wBqNpiTpBufLen;

    uint8 uQueLockStat;
    uint8 uTxDrvState;
    uint8 uBQRespStat;
    uint8 uRetryStat;

    uint8 nDevMacCfg;
    uint8 uDevTblIdx;
    uint8 uDevTblSendIdx;
    uint8 uBufSIdx;

    uint8 uNAFEs;
    uint8 uSfTime;
    uint8 uIntDelay;
    uint8 uExtDelay;

    uint8 uCmdIface;
    uint8 uTimerProg;
    uint8 uInit;
    uint8 uWaitStartupInit;

    uint8 uRxDrvLenValid;

    uint32 qGuardEnd;
}
Cc2662_ManagerType;

typedef enum Cc2662_StatusType_Tag
{
    CC2662_STATE_IDLE = 0x20,
    CC2662_STATE_TRANSIENT = 0x21,
    CC2662_STATE_WAIT_TXN_COMPLETE = 0x22,
    CC2662_STATE_WAIT_CTRL_COMPLETE = 0x23,
    CC2662_STATE_WAIT_SYNC_COMPLETE = 0x24,
    CC2662_STATE_WAIT_DELAY = 0x24,

    CC2662_STATE_ERROR = 0x30
}
Cc2662_StatusType;

typedef enum Cc2662_NwStatusType_Tag
{
    CC2662_WBMS_SCANNING = 0x40,
    CC2662_WBMS_CONNECTED = 0x41,
    CC2662_WBMS_NW_FORMED = 0x42,

    CC2662_WBMS_ERROR = 0x50
}
Cc2662_NwStatusType;

typedef enum Cc2662_FrameCmdType_Tag
{
    CC2662_REQ_READ = 0x00,
    CC2662_REQ_WRITE = 0x01,
    CC2662_REQ_CHECK = 0x02,
    CC2662_REQ_INTDELAY = REQTYPE_CUSTOM_INTDELAY,
    CC2662_REQ_COMCLEAR = REQTYPE_CUSTOM_COMCLEAR,
    CC2662_REQ_WAKEUP = REQTYPE_CUSTOM_WAKEUP,
    CC2662_REQ_DELAY = REQTYPE_CUSTOM_DELAY,
    CC2662_REQ_CUSTOM = REQTYPE_CUSTOM_CMD_W
}
Cc2662_FrameCmdType;

/*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

#define CC2662_START_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

static Cc2662_ManagerType zCc2662Ctx[CC2662_IFACES];

#define CC2662_STOP_SEC_VAR_NOINIT_UNSPECIFIED
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

#define CC26662_START_SEC_CODE
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 * FUNC(uint16, cc2662_CODE) Cc2662_NpiCalcFcs(const uint8 buf[], uint16 wDataLen)
 *********************************************************************************************************************/
/*! \brief          This function is used to check the FCS of each frame response data.
 *
 *  This functions returns the last error stored incase of any issues. Memory dump of array can be take to analyze
 *  further issues
 *
 * \param[in]       pData: Pointer to data
 * \param[in]       uSFrameLen: single frame length
 * \param[in]       uFrameNum: Number of Frames
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            None
 *  \post           None
 *  \return         uint8
 *  \retval         FCS Value
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_NpiCalcFcs(const uint8 buf[], uint32 qLength)
{
    uint16 i;
    uint8 fcs = 0;

    for(i = 0; i < qLength; i++)
    {
        fcs ^= buf[i];
    }

    return fcs;
}

/**********************************************************************************************************************
 * FUNC(uint8, cc2662_CODE) Cc2662_FrameCheck(const uint8 pRxData[], uint16 wLength)
 *********************************************************************************************************************/
/*! \brief          This function is used to check the CRC of each frame response data.
 *
 *  This functions returns the last error stored incase of any issues. Memory dump of array can be take to analyze
 *  further issues
 *
 * \param[in]       pCc2662Mgr: Pointer to data
 * \param[in]       pRxData: single frame length
 * \param[in]       wLength: Number of Frames
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            None
 *  \post           None
 *  \return         uint8
 *  \retval         RESP_VALID: CRC check passed.
 *                  RESP_INVALID: CRC check failed.
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_FrameCheck(const uint8 pRxData[], uint32 qLength)
{
    uint8 rxFcs = 0;
    uint8 uRet = NPI_STATUS_FORMAT_ERR;

    if(pRxData[0] == NPI_MSG_SOF)
    {
		uRet = NPI_STATUS_CRC_ERR;
		rxFcs = Cc2662_NpiCalcFcs((uint8*) &pRxData[1], (qLength - 2U));
		if(rxFcs == pRxData[qLength - 1U])
		{
		    uRet = (uint8)COMIF_OK;
	    }
	}

	return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, cc2662_CODE) Cc2662_ResponseCrcCheck(const uint8 *pData, uint8 uSFrameLen, uint8 uFrameNum)
 *********************************************************************************************************************/
/*! \brief          This function is used to check the CRC of each frame response data.
 *
 *  This functions returns the last error stored incase of any issues. Memory dump of array can be take to analyze
 *  further issues
 *
 * \param[in]       pData: Pointer to data
 * \param[in]       uSFrameLen: single frame length
 * \param[in]       uFrameNum: Number of Frames
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            None
 *  \post           None
 *  \return         uint8
 *  \retval         RESP_VALID: CRC check passed.
 *                  RESP_INVALID: CRC check failed.
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_ResponseCrcCheck(const uint8 pData[], uint8 uSFrameLen, uint8 uFrameNum)
{
    uint8 uRet = (uint8)RESP_VALID;
    uint8 uIndex;
    uint16 wCrc;

    for(uIndex = 0u; uIndex < uFrameNum; uIndex++)
    {
        wCrc = CalculateCRC16(&pData[uSFrameLen * uIndex], uSFrameLen);
        if(0u != wCrc)
        {
            uRet = (uint8)RESP_INVALID;
            break;
        }
    }

    return uRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, cc2662_CODE) Cc2662_QueInit(Cc2662_ManagerType *pCc2662Mgr, Cc2662_ConfigType *pCc2662Cfg)
 *********************************************************************************************************************/
/*! \brief          Initialization of communication device
 *
 *  \param[in]      pCc2662Mgr: Communication device Manager
 *  \param[in]      pCc2662Cfg: Communication device configuration
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the communication device manager context
 *  \retval         COMIF_OK or COMIF_NOT_OK
 *
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_QueInit(Cc2662_ManagerType *pCc2662Mgr, Cc2662_ConfigType *pCc2662Cfg)
{
    uint16 wQueIdx;
    uint8 uRet = (uint8)COMIF_NOT_OK;

    do
    {
        // Command Data Check
        if((NULL == pCc2662Cfg->pCmdQueFreeMemCfg) || (NULL == pCc2662Cfg->pCmdDataMemCfg) ||
           (!pCc2662Cfg->wCmdQueCntCfg))
        {
            uRet = (uint8)COMIF_CMD_QUECFG_INVALID;
            break;
        }

        // Free CMD QUEUE
        uRet = ctPeQueueCreate(&pCc2662Mgr->zCmdQueFree, pCc2662Mgr->pBswCfg->qDriverRes, pCc2662Mgr->pBswCfg,
                               pCc2662Cfg->pCmdQueFreeMemCfg, pCc2662Cfg->wCmdQueSizeCfg, pCc2662Cfg->wCmdQueCntCfg);
        if((uint8)QUEUE_E_OK != uRet)
        {
            uRet = (uint8)COMIF_CMD_CFQUE_FAILED;
            break;
        }

        // Adding Data from CMD Free QUEUE
        for(wQueIdx = 0; wQueIdx < pCc2662Cfg->wCmdQueCntCfg; wQueIdx++)
        {
            uRet = ctPeQueueAddData(&pCc2662Mgr->zCmdQueFree, (void*) &pCc2662Cfg->pCmdDataMemCfg[wQueIdx]);
            if((uint8)QUEUE_E_OK != uRet)
            {
                uRet = (uint8)COMIF_CMD_CFQUE_FAILED;
                break;
            }
        }

        if((uint8)COMIF_CMD_CFQUE_FAILED == uRet)
        {
            break;
        }

        // Exec Queue for each priority
        if((NULL == pCc2662Cfg->pCmdQueExecMemCfg) || (!pCc2662Cfg->wExecQueCntCfg))
        {
            uRet = (uint8)COMIF_CMD_QUECFG_INVALID;
            break;
        }

        for(wQueIdx = 0; wQueIdx < (uint8)COMIF_CMD_PRIO_MAX; wQueIdx++)
        {
            uRet = ctPeQueueCreate(&pCc2662Mgr->zCmdQueExec[wQueIdx], pCc2662Mgr->pBswCfg->qDriverRes,
                                   pCc2662Mgr->pBswCfg, pCc2662Cfg->pCmdQueExecMemCfg[wQueIdx],
                                   pCc2662Cfg->wExecQueSizeCfg, pCc2662Cfg->wExecQueCntCfg);

            if((uint8)QUEUE_E_OK != uRet)
            {
                uRet = (uint8)COMIF_CMD_CEQUE_FAILED;
                break;
            }
        }

        if((uint8)COMIF_CMD_CEQUE_FAILED == uRet)
        {
            break;
        }

        // Read Exec Command Queue
        if((NULL == pCc2662Cfg->pReadQueExecMemCfg)|| (!pCc2662Cfg->wReadQueCntCfg))
        {
            uRet = (uint8)COMIF_CMD_QUECFG_INVALID;
            break;
        }

        uRet = ctPeQueueCreate(&pCc2662Mgr->zCmdReadQueExec, pCc2662Mgr->pBswCfg->qDriverRes,
                               pCc2662Mgr->pBswCfg, pCc2662Cfg->pReadQueExecMemCfg,
                               pCc2662Cfg->wReadQueSizeCfg, pCc2662Cfg->wReadQueCntCfg);
        if((uint8)QUEUE_E_OK != uRet)
        {
            uRet = (uint8)COMIF_CMD_REQUE_FAILED;
            break;
        }

        // MAC TX QUEUE
        // FREE MAC QUE
        uRet = (uint8)ctPeQueueCreate(&pCc2662Mgr->zTransTd.zMacTxFreeQue, pCc2662Mgr->pBswCfg->qDriverRes,
                               pCc2662Mgr->pBswCfg, pCc2662Mgr->zTransTd.pMacTxQueFreeMem, sizeof(void*),
                               NPI_MAC_TXQUE_CFG);
        if((uint8)QUEUE_E_OK != uRet)
        {
            uRet = (uint8)COMIF_CMD_MTFQUE_FAILED;
            break;
        }

        // Adding to Free MAC QUE
        for(wQueIdx = 0; wQueIdx < NPI_MAC_TXQUE_CFG; wQueIdx++)
        {
            uRet = ctPeQueueAddData(&pCc2662Mgr->zTransTd.zMacTxFreeQue,
                                    (void*) &pCc2662Mgr->zTransTd.zMacTxData[wQueIdx]);
            if((uint8)QUEUE_E_OK != uRet)
            {
                uRet = (uint8)COMIF_CMD_MTFQUE_FAILED;
                break;
            }
        }

        if((uint8)COMIF_CMD_MTFQUE_FAILED == uRet)
        {
            break;
        }

        // RX Pending MAC QUE for TX
        for(wQueIdx = 0; wQueIdx < NPI_MAC_TXQUE_CFG; wQueIdx++)
        {
            uRet = (uint8)ctPeQueueCreate(&pCc2662Mgr->zTransTd.zMacTxData[wQueIdx].zTxPendRespQue,
                                   pCc2662Mgr->pBswCfg->qDriverRes,
                                   pCc2662Mgr->pBswCfg, pCc2662Mgr->zTransTd.zMacTxData[wQueIdx].pTxQueRxPendMem,
                                   sizeof(void*), NPI_MAC_TXQUE_RXPEND_MAX);

            if((uint8)QUEUE_E_OK != uRet)
            {
                uRet = (uint8)COMIF_CMD_MTPQUE_FAILED;
                break;
            }
        }

        if((uint8)COMIF_CMD_MTPQUE_FAILED == uRet)
        {
            break;
        }

        // TX MAC EXEC QUEU
        uRet = (uint8)ctPeQueueCreate(&pCc2662Mgr->zTransTd.zMacTxExecQue, pCc2662Mgr->pBswCfg->qDriverRes,
                               pCc2662Mgr->pBswCfg, pCc2662Mgr->zTransTd.pMacTxQueExecMem, sizeof(void*),
                               NPI_MAC_TXQUE_CFG);
        if((uint8)QUEUE_E_OK != uRet)
        {
            uRet = (uint8)COMIF_CMD_MTEQUE_FAILED;
            break;
        }

        // TX Sync Rx Pending QUEUE
        uRet = (uint8)ctPeQueueCreate(&pCc2662Mgr->zTransTd.zSyncTxData.zTxPendRespQue,
                               pCc2662Mgr->pBswCfg->qDriverRes,
                               pCc2662Mgr->pBswCfg, pCc2662Mgr->zTransTd.zSyncTxData.pTxQueRxPendMem,
                               sizeof(void*), NPI_MAC_TXQUE_RXPEND_MAX);
        if((uint8)QUEUE_E_OK != uRet)
        {
            uRet = (uint8)COMIF_CMD_SPQUE_FAILED;
            break;
        }

        uRet = (uint8)COMIF_OK;
    }
    while(0);

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, cc2662_CODE) ScheduleInternalRequest(Cc2662_ManagerType *pCc2662Mgr, Comif_ReqCmdType *pReqCmd)
 *********************************************************************************************************************/
/*! \brief          Scheduling the request for transfer
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *  \param[in]      pReqCmd: Request Command
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) ScheduleInternalRequest(Cc2662_ManagerType *pCc2662Mgr, Comif_ReqCmdType *pReqCmd,
                                                        Cc2662_ReqDataType *pFrame)
{
    uint8 *pDevCfg = NULL;
    uint8 uScheduleFrame = TRUE;
    uint16 wData;
    uint8 uDevIdx;
    uint8 uDataIdx;
    uint8 nDevices;

    pFrame->wReqDataLen = 0u;
    pFrame->uCmdTyp = NPI_SYNC_REQ;
    pFrame->pServiceReq = pReqCmd->pServiceCfg;
    if(pReqCmd->wRegAdr < (uint8)COMM_RESET)
    {
        pFrame->uCmd = (uint8)CC2662_REQ_READ;
    }
    else
    {
        pFrame->uCcReqTyp = CC2662_WM_RESET_DEVICE;
        pFrame->uBlockSiz = CC2662_RESET_RSP_LEN;
        pFrame->uCmd = (uint8)CC2662_REQ_WRITE;
    }

    switch (pReqCmd->wRegAdr)
    {
        case (uint8)COMM_GETIDENTITY:
        {
            pFrame->uCcReqTyp = CC2662_WM_IDENTIFY;
            pFrame->uBlockSiz = CC2662_IDENTITY_RSP_LEN;
            break;
        }
        case (uint8)COMM_GETNUMOFCONN:
        {
            pFrame->uCcReqTyp = CC2662_WM_NUM_WD_CONN;
            pFrame->uBlockSiz = CC2662_NUMOFCONN_RSP_LEN;
            break;
        }
        case (uint8)COMM_GETNUMOFTXPACKETS:
        {
            pFrame->uCcReqTyp = CC2662_WM_NUM_TX_PACKETS;
            pFrame->uBlockSiz = CC2662_NUMOFTXPACKETS_RSP_LEN;
            break;
        }
        case (uint8)COMM_GETNUMOFTXFAILEDPACKETS:
        {
            pFrame->uCcReqTyp = CC2662_WM_NUM_TX_FAILED_PACKETS;
            pFrame->uBlockSiz = CC2662_NUMOFTXFAILEDPACKETS_RSP_LEN;
            break;
        }
        case (uint8)COMM_GETTXTHROUGHPUT:
        {
            pFrame->uCcReqTyp = CC2662_WM_TX_THROUGHPUT;
            pFrame->uBlockSiz = CC2662_TXTHROUGHPUT_RSP_LEN;
            break;
        }
        case (uint8)COMM_GETRXTHROUGHPUT:
        {
            pFrame->uCcReqTyp = CC2662_WM_RX_THROUGHPUT;
            pFrame->uBlockSiz = CC2662_RXTHROUGHPUT_RSP_LEN;
            break;
        }
        case (uint8)COMM_GETNUMOFRXPACKETS:
        {
            pFrame->uCcReqTyp = CC2662_WD_NUM_RX_PACKETS;
            pFrame->wReqDataLen = READ_1BYTE;
            pFrame->uBlockSiz = CC2662_NUMOFRXPACKETS_RSP_LEN;
            pFrame->zTxCmd[0] = pReqCmd->uDevId;
            break;
        }
        case (uint8)COMM_GETNUMOFMISSEDRXPACKETS:
        {
            pFrame->uCcReqTyp = CC2662_WD_NUM_RX_TIMEOUT_EVENTS;
            pFrame->wReqDataLen = READ_1BYTE;
            pFrame->uBlockSiz = CC2662_NUMOFMISSEDRXPACKETS_RSP_LEN;
            pFrame->zTxCmd[0] = pReqCmd->uDevId;
            break;
        }
        case (uint8)COMM_GETLATENCYOFNODE:
        {
            pFrame->uCcReqTyp = CC2662_WD_LATENCY;
            pFrame->wReqDataLen = READ_1BYTE;
            pFrame->uBlockSiz = CC2662_LATENCYOFNODE_RSP_LEN;
            pFrame->zTxCmd[0] = pReqCmd->uDevId;
            break;
        }
        case (uint8)COMM_GETPEROFNODE:
        {
            pFrame->uCcReqTyp = CC2662_WD_PER;
            pFrame->wReqDataLen = READ_1BYTE;
            pFrame->uBlockSiz = CC2662_PEROFNODE_RSP_LEN;
            pFrame->zTxCmd[0] = pReqCmd->uDevId;
            break;
        }
        case (uint8)COMM_SENDBQRAWFRAME:
        {
            pFrame->uCcReqTyp = CC2662_WD_BQ_RAW_FRAME;
            pFrame->uCmdTyp = NPI_ASYNC_REQ;
            pFrame->wReqDataLen = pReqCmd->pData[0] + 2U;                            // +2 bytes crc
            pFrame->uBlockSiz = CC2662_REQ_FRAME_FIXED_LEN + pReqCmd->pData[4] + 1U; //BQ Header + Bq rsp payload length
            for(uDataIdx = 0; uDataIdx < pReqCmd->pData[0]; uDataIdx++)
            {
                pFrame->zTxCmd[uDataIdx] = pReqCmd->pData[uDataIdx + 1U];
            }
            wData = CalculateCRC16(pFrame->zTxCmd, pReqCmd->pData[0]);
            pFrame->zTxCmd[uDataIdx] = (uint8)(wData & TIBMS_8BIT_MASK);
            pFrame->zTxCmd[uDataIdx + 1U] = (uint8)((wData >> TIBMS_8BIT_OFFSET) & TIBMS_8BIT_MASK);
            break;
        }
        case (uint8)COMM_SENDWDSTORAGE:
        {
            pFrame->wReqDataLen = READ_11BYTE;        
            pFrame->uBlockSiz = CC2662_SENDWDSTORAGE_RSP_LEN;
            pFrame->uCcReqTyp = CC2662_WD_STORAGE;
            pFrame->uCmdTyp = NPI_ASYNC_REQ;
            pCc2662Mgr->npiWdBuf[0] = 0x00; // Reserved
            pCc2662Mgr->npiWdBuf[1] = (uint8)(pCc2662Mgr->pNwCfg->wdStorageCmd);
            pCc2662Mgr->npiWdBuf[2] = (uint8)(pCc2662Mgr->pNwCfg->wdStorageMode); // Scan Mode NO or OFF
            
            pCc2662Mgr->npiWdBuf[3] = (uint8)(pCc2662Mgr->pNwCfg->wdBackOffTime);
            pCc2662Mgr->npiWdBuf[4] = (uint8)(pCc2662Mgr->pNwCfg->wdBackOffTime >> 8);
            pCc2662Mgr->npiWdBuf[5] = (uint8)(pCc2662Mgr->pNwCfg->wdBackOffTime >> 16);
            pCc2662Mgr->npiWdBuf[6] = (uint8)(pCc2662Mgr->pNwCfg->wdBackOffTime >> 24);
            
            pCc2662Mgr->npiWdBuf[7] = (uint8)(pCc2662Mgr->pNwCfg->wdScanTimeOut);
            pCc2662Mgr->npiWdBuf[8] = (uint8)(pCc2662Mgr->pNwCfg->wdScanTimeOut >> 8);
            pCc2662Mgr->npiWdBuf[9] = (uint8)(pCc2662Mgr->pNwCfg->wdScanTimeOut >> 16);
            pCc2662Mgr->npiWdBuf[10] = (uint8)(pCc2662Mgr->pNwCfg->wdScanTimeOut >> 24);
            break;
        }
        case (uint8)COMM_RESETDEVICE:
        {
            pFrame->uCcReqTyp = CC2662_WM_RESET_DEVICE;
            pFrame->uBlockSiz = READ_1BYTE;
            break;
        }
        case (uint8)COMM_GETRSSIOFNODE:
        {
            pFrame->uCcReqTyp = CC2662_WD_RSSI;
            pFrame->wReqDataLen = READ_1BYTE;
            pFrame->uBlockSiz = CC2662_RSSIOFNODE_RSP_LEN;
            pFrame->zTxCmd[0] = pReqCmd->uDevId;
            break;
        }
        case (uint8)COMM_APP_DIAG_SETMAINCONFIG:
        {
            pFrame->uCcReqTyp = CC2662_APP_DIAG_SETMAINCONFIG;
            pFrame->wReqDataLen = READ_16BYTE;
            pFrame->uBlockSiz = CC2662_SETMAINCONFIG_RSP_LEN;

            pFrame->zTxCmd[0] = (uint8) (pCc2662Mgr->pNwCfg->wWbmsNwId);            // as per WM user config file
            pFrame->zTxCmd[1] = (uint8) (pCc2662Mgr->pNwCfg->wWbmsNwId >> 8);       // as per WM user config file
            pFrame->zTxCmd[2] = pCc2662Mgr->uNAFEs;                                 // Number of devices in the network
            pFrame->zTxCmd[3] = pCc2662Mgr->pNwCfg->uUplinkTime;                    // Uplink time
            pFrame->zTxCmd[5] = (uint8) (pCc2662Mgr->pNwCfg->wDownlinkTime >> 8);   // Downlink
            pFrame->zTxCmd[4] = (uint8) (pCc2662Mgr->pNwCfg->wDownlinkTime);        // Downlink
            pFrame->zTxCmd[6] = pCc2662Mgr->uNAFEs;                                 // Partail Number of devices
            pFrame->zTxCmd[7] = pCc2662Mgr->pNwCfg->uMaxRetries;                    // Max retries
            pFrame->zTxCmd[8] = pCc2662Mgr->pNwCfg->uKeepAliveInt;                  // Keep alive interval
            pFrame->zTxCmd[9] = pCc2662Mgr->pNwCfg->uUltoDlTime;
            pFrame->zTxCmd[10] = pCc2662Mgr->pNwCfg->uNumofSkipAlivInt;             // Number of skipped aliv intervals
            pFrame->zTxCmd[11] = pCc2662Mgr->pNwCfg->uDenylist[0];                  // Denylist
            pFrame->zTxCmd[12] = pCc2662Mgr->pNwCfg->uDenylist[1];                  // Denylist
            pFrame->zTxCmd[13] = pCc2662Mgr->pNwCfg->uDenylist[2];                  // Denylist
            pFrame->zTxCmd[14] = pCc2662Mgr->pNwCfg->uDenylist[3];                  // Denylist
            pFrame->zTxCmd[15] = pCc2662Mgr->pNwCfg->uDenylist[4];                  // Denylist
            break;
        }
		case (uint8)COMM_APP_DIAG_STARTNETWORK:
        {
            pFrame->uCcReqTyp = CC2662_APP_DIAG_STARTNETWORK;
            pFrame->wReqDataLen = READ_4BYTE;
            pFrame->uBlockSiz = CC2662_STARTNETWORK_RSP_LEN;

            pFrame->zTxCmd[0] = (uint8) (CC2662_START_NW_TIMEOUT);
            pFrame->zTxCmd[1] = (uint8) (CC2662_START_NW_TIMEOUT >> 8);
            pFrame->zTxCmd[2] = (uint8) (CC2662_START_NW_TIMEOUT >> 16);
            pFrame->zTxCmd[3] = (uint8) (CC2662_START_NW_TIMEOUT >> 24);
            break;
        }
        case (uint8)COMM_APP_DIAG_SETPOWERMODE:
        {
            pFrame->uCcReqTyp = CC2662_APP_DIAG_SETPOWERMODE;
            pFrame->wReqDataLen = READ_1BYTE;
            pFrame->uBlockSiz = CC2662_SETPOWERMODE_RSP_LEN;
            pFrame->zTxCmd[0] = pReqCmd->pData[0];                                   //WM_POWERMODE_ACTIVE;//WM_POWERMODE_KEEPALIVE;
            break;
        }
        case (uint8)COMM_APP_DIAG_RUNDWM:
        {
            
            pFrame->uCcReqTyp = CC2662_APP_DIAG_RUNDWM;
            pFrame->wReqDataLen = READ_5BYTE;
            pFrame->uBlockSiz = CC2662_RUNDWM_RSP_LEN;
            pFrame->zTxCmd[0] =pCc2662Mgr->pNwCfg->uDualWmMode;                     // Dual Main mode
			pFrame->zTxCmd[1] =(uint8)(CC2662_DUALNW_TIMEOUT);
			pFrame->zTxCmd[2] =(uint8)(CC2662_DUALNW_TIMEOUT >> 8);
			pFrame->zTxCmd[3] =(uint8)(CC2662_DUALNW_TIMEOUT >> 16);
			pFrame->zTxCmd[4] =(uint8)(CC2662_DUALNW_TIMEOUT >> 24);
            break;
        }
        case (uint8)COMM_APP_DIAG_DISCOVERREQ:
        {
            pFrame->uCcReqTyp = CC2662_APP_DIAG_DISCOVERREQ;
            pFrame->wReqDataLen = READ_4BYTE;  
            pFrame->uBlockSiz = CC2662_DISCOVER_RSP_LEN;
			pFrame->zTxCmd[0] =(uint8)(pCc2662Mgr->pNwCfg->DevDisc_timeout);
			pFrame->zTxCmd[1] =(uint8)(pCc2662Mgr->pNwCfg->DevDisc_timeout >> 8);
			pFrame->zTxCmd[2] =(uint8)(pCc2662Mgr->pNwCfg->DevDisc_timeout >> 16);
			pFrame->zTxCmd[3] =(uint8)(pCc2662Mgr->pNwCfg->DevDisc_timeout >> 24);
            break;
        }
        case (uint8)COMM_APP_DIAG_SETNWKOPMODE:
        {
            pFrame->uCcReqTyp = CC2662_APP_DIAG_SETNWKOPMODE;
            pFrame->wReqDataLen = READ_1BYTE;
            pFrame->uBlockSiz = CC2662_SETNWKOPMODE_RSP_LEN;
            pFrame->zTxCmd[0] = pCc2662Mgr->pNwCfg->uNwOpMode;
            break;
        }
        case (uint8)COMM_APP_DIAG_SETJOINMODE:
        {
            pFrame->uCcReqTyp = CC2662_APP_DIAG_SETJOINMODE;
            pFrame->wReqDataLen = READ_1BYTE;
            pFrame->uBlockSiz = CC2662_SETJOINMODE_RSP_LEN;
            pFrame->zTxCmd[0] = pCc2662Mgr->pNwCfg->uNwJoinMode;                //Network joining mode (0- Non-Selective; 1- Selective)
            break;
        }
        case (uint8)COMM_APP_DIAG_SETDEVTBLCFG:
        {
            if((pCc2662Mgr->uDevTblIdx < NPI_MAX_MACTABLE_IDX) && (pCc2662Mgr->nDevMacCfg > 0u))
            {
                pFrame->uCcReqTyp = CC2662_APP_DIAG_SETDEVTBLCFG;
                nDevices = NPI_MACTABLE_DEVS_INSLOT-1u;
                if(pCc2662Mgr->nDevMacCfg < (NPI_MACTABLE_DEVS_INSLOT-1u))
                {
                     nDevices = pCc2662Mgr->nDevMacCfg;
                }
                pFrame->wReqDataLen = ((nDevices * NPI_MACTABLE_SLOT_SIZE) + 1U);
                pFrame->uBlockSiz = CC2662_SETDEVTBLCFG_RSP_LEN;

                uDevIdx = (pCc2662Mgr->uNAFEs - pCc2662Mgr->nDevMacCfg);
                pCc2662Mgr->npiGenBuf[pCc2662Mgr->uDevTblIdx][0u] = nDevices;
                for(; uDevIdx < nDevices; uDevIdx++)
                {
                    pDevCfg = (uint8 *) pCc2662Mgr->pNwCfg->pMacTbl[uDevIdx];
                    if(NULL != pDevCfg)
                    {
                        for(uDataIdx = 0u; uDataIdx < NPI_MACTABLE_SLOT_SIZE; uDataIdx++)
                        {
                            pCc2662Mgr->npiGenBuf[pCc2662Mgr->uDevTblIdx][uDataIdx+1U+(uDevIdx*NPI_MACTABLE_SLOT_SIZE)] = pDevCfg[uDataIdx];
                        }
                    }
                }
                pCc2662Mgr->nDevMacCfg -= nDevices;
                pCc2662Mgr->uDevTblIdx++;
            }
            break;
        }
        case (uint8)COMM_APP_DIAG_UNPAIRREQ:
        {
            pFrame->uCcReqTyp = CC2662_APP_DIAG_UNPAIRREQ;
            pFrame->wReqDataLen = READ_8BYTE;
            pFrame->uBlockSiz = CC2662_UNPAIRREQ_RSP_LEN;

            pFrame->zTxCmd[0] = 0xFF;
            pFrame->zTxCmd[1] = 0xFF;
            pFrame->zTxCmd[2] = 0xFF;
            pFrame->zTxCmd[3] = 0xFF;
            pFrame->zTxCmd[4] = 0xFF;
            pFrame->zTxCmd[5] = 0xFF;
            pFrame->zTxCmd[6] = 0xFF;
            pFrame->zTxCmd[7] = 0xFF;
            break;
        }
        case (uint8)COMM_APP_DIAG_RESYNC:
        {
            pFrame->uCcReqTyp = CC2662_APP_DIAG_RESYNC;
            pFrame->wReqDataLen = READ_4BYTE;
            pFrame->uBlockSiz = CC2662_RESYNC_RSP_LEN;
            pFrame->uCmdInfo = (uint8)COMIF_CMD_PRIO_EXCL;                      // This Packet should go high priority

            pFrame->zTxCmd[0] = pReqCmd->pData[3];
            pFrame->zTxCmd[1] = pReqCmd->pData[2];
            pFrame->zTxCmd[2] = pReqCmd->pData[1];
            pFrame->zTxCmd[3] = pReqCmd->pData[0];
            break;
        }
		 case (uint8)COMM_APP_DIAG_WM_REPAIRREQ:
        {   
            pFrame->uCcReqTyp = CC2662_APP_DIAG_WM_REPAIRREQ;
            pFrame->wReqDataLen = READ_4BYTE; 
            pFrame->uBlockSiz = CC2662_REPAIR_RSP_LEN;
			pFrame->zTxCmd[0] =(uint8)(pCc2662Mgr->pNwCfg->RePair_timeout);
			pFrame->zTxCmd[1] =(uint8)(pCc2662Mgr->pNwCfg->RePair_timeout >> 8);
			pFrame->zTxCmd[2] =(uint8)(pCc2662Mgr->pNwCfg->RePair_timeout >> 16);
			pFrame->zTxCmd[3] =(uint8)(pCc2662Mgr->pNwCfg->RePair_timeout >> 24);
            break;
        }
        case (uint8)COMM_APP_DIAG_SETWKUPPRD:
        {
            pFrame->uCcReqTyp = CC2662_APP_DIAG_SETWKUPPRD;
            pFrame->wReqDataLen = READ_2BYTE;
            pFrame->uBlockSiz = CC2662_SETWKUPPRD_RSP_LEN;
            pFrame->zTxCmd[0] = pReqCmd->pData[0];
            pFrame->zTxCmd[1] = pReqCmd->pData[1];
            pCc2662Mgr->pBswCfg->dio_req(DIO_EDGE_ENABLE);
            break;
        }
		case (uint8)COMM_APP_DIAG_SETPARAMS:
        {
            pFrame->uCcReqTyp = CC2662_APP_DIAG_SETPARAMS;
            pFrame->wReqDataLen = READ_16BYTE;
            pFrame->uBlockSiz = CC2662_SETPARAMS_RSP_LEN;
            pFrame->zTxCmd[0] = pCc2662Mgr->pNwCfg->uDualWmType;
            pFrame->zTxCmd[1] = 13; //length
            pFrame->zTxCmd[2] = 0;  //length
            pFrame->zTxCmd[3] = pReqCmd->pData[0];                                  // 0 - Primary Main, 1 - Secondary Main
            pDevCfg = (uint8 *) pCc2662Mgr->pNwCfg->pDfuMacTbl[pReqCmd->pData[0]];
            for(uDataIdx=0; uDataIdx < 8U; uDataIdx++)
            {
                pFrame->zTxCmd[uDataIdx+4U] = pDevCfg[uDataIdx];                     //main MacID's -make this Configuration
            }
			pFrame->zTxCmd[uDataIdx+4U] =(uint8)(pCc2662Mgr->pNwCfg->DualNwStart_timeout);
			uDataIdx++;
			pFrame->zTxCmd[uDataIdx+4U] =(uint8)(pCc2662Mgr->pNwCfg->DualNwStart_timeout >> 8);
			uDataIdx++;
			pFrame->zTxCmd[uDataIdx+4U] =(uint8)(pCc2662Mgr->pNwCfg->DualNwStart_timeout >> 16);
			uDataIdx++;
			pFrame->zTxCmd[uDataIdx+4U] =(uint8)(pCc2662Mgr->pNwCfg->DualNwStart_timeout >> 24);
			break;
        }
   		case (uint8)COMM_APP_DIAG_OADREQ:
		case (uint8)COMM_APP_DIAG_DFUREQ:
        {
            pFrame->wReqDataLen  = pReqCmd->pData[0];
            pFrame->uBlockSiz = pReqCmd->pData[0];
            pFrame->uCmd = (uint8)CC2662_REQ_READ;
            
            if((uint8)COMM_APP_DIAG_OADREQ == pReqCmd->wRegAdr)
            {
                pFrame->uCcReqTyp = CC2662_APP_DIAG_OADREQ;
            }
            else
            {
                pFrame->uCcReqTyp = CC2662_APP_DIAG_DFUREQ;
            }
            pFrame->uCmdTyp = NPI_ASYNC_REQ;

            for(uDataIdx = 0u; uDataIdx < (pFrame->wReqDataLen); uDataIdx++)
            {
                pCc2662Mgr->npiWdBuf[uDataIdx] = pReqCmd->pData[uDataIdx + 1U]; 
            }
            break;
        }
		case (uint8)COMM_APP_DIAG_GETFWVERSION:
        {
            pFrame->uCcReqTyp = CC2662_APP_DIAG_GETFWVERSION;
            pFrame->uBlockSiz = CC2662_FWVERSION_RSP_LEN;
            break;
        }
        case (uint8)COMM_APP_DIAG_GETNETWORKSTATS:
        {
            pFrame->uCcReqTyp = CC2662_APP_DIAG_GETNETWORKSTATS;
            pFrame->wReqDataLen = READ_1BYTE;
            pFrame->uBlockSiz = CC2662_NETWORKSTATS_RSP_LEN;
            pFrame->zTxCmd[0] = pReqCmd->uDevId;
            break;
        }
		case (uint8)COMM_APP_DIAG_GETTXPACKETS:
		{
            pFrame->uCcReqTyp = CC2662_APP_DIAG_GETTXPACKETS;
            pFrame->uBlockSiz = CC2662_TXPACKETS_RSP_LEN;
			break;
		}
        case (uint8)COMM_APP_DIAG_GETHROUGHTPUT:
        {
            pFrame->uCcReqTyp = CC2662_APP_DIAG_GETHROUGHTPUT;
            pFrame->uBlockSiz = CC2662_THROUGHTPUT_RSP_LEN;
            break;
        }
        case (uint8)COMM_APP_DIAG_GETRXPACKETS:
        {
            pFrame->uCcReqTyp = CC2662_APP_DIAG_GETRXPACKETS;
            pFrame->wReqDataLen = READ_1BYTE;
            pFrame->uBlockSiz = CC2662_RXPACKETS_RSP_LEN;
            pFrame->zTxCmd[0] = pReqCmd->uDevId;
            break;
        }
		case (uint8)COMM_APP_DIAG_GETSTATS:
        {
            pFrame->wReqDataLen = READ_3BYTE;
            pFrame->uBlockSiz = CC2662_GETSTATS_RSP_LEN;
            pFrame->uCmd = (uint8)CC2662_REQ_READ;
            pFrame->uCcReqTyp = CC2662_APP_DIAG_GETSTATS;
            pFrame->uCmdTyp = NPI_SYNC_REQ;
            pFrame->zTxCmd[0] = pReqCmd->pData[0];  
            pFrame->zTxCmd[1] = pReqCmd->uDevId;   
            pFrame->zTxCmd[2] = 0x04U;  
            break;
        }
		case (uint8)COMM_APP_DIAG_GETPARAMS:
        {
            pFrame->wReqDataLen = READ_1BYTE;
            pFrame->uBlockSiz = CC2662_GETPARAMS_RSP_LEN;
            pFrame->uCmd = (uint8)CC2662_REQ_READ;
            pFrame->uCcReqTyp = CC2662_APP_DIAG_GETPARAMS;
            pFrame->uCmdTyp = NPI_SYNC_REQ;
            pFrame->zTxCmd[0] = pReqCmd->pData[0];  
            break;
        }
        
        case (uint8)COMM_APP_DIAG_GETKLVDURATION:
        {
            pFrame->uCcReqTyp = CC2662_APP_DIAG_GETKLVDURATION;
            pFrame->uBlockSiz = CC2662_KLVDURATION_RSP_LEN;
            break;
        }
		
        default:
        {
            uScheduleFrame = FALSE;
            break;
        }
    }

    if(0U != uScheduleFrame)
    {
        pFrame->pRxData = (uint8*) pReqCmd->pServiceCfg->pRawData;
        pFrame->uStatus = (uint8)COMIF_PENDING;
    }

    return uScheduleFrame;
}

/**********************************************************************************************************************
 *  FUNC(uint8, cc2662_CODE) ScheduleFrameRequest(Cc2662_ManagerType *pCc2662Mgr, Comif_ReqCmdType *pReqCmd)
 *********************************************************************************************************************/
/*! \brief          Scheduling the request for transfer
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *  \param[in]      pReqCmd: Request Command
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) ScheduleFrameRequest(Cc2662_ManagerType *pCc2662Mgr, Comif_ReqCmdType *pReqCmd)
{
    Cc2662_ReqDataType *pFrame;
    uint8 uRet = (uint8)COMIF_CMD_CFQGET_EMPTY;
    uint16 wData;
    uint16 wIndex = 0u;
    uint8 uPrioReq = FALSE;
    uint8 uReqCmdInit = FALSE;
    uint8 uScheduleFrame = FALSE;
    uint8 uSingleReq = FALSE;

    pFrame = (Cc2662_ReqDataType*) ctPeQueueRemoveData(&pCc2662Mgr->zCmdQueFree);
    if(NULL != pFrame)
    {
        switch (pReqCmd->uReqType)
        {
            case (uint8)REQTYPE_STACK_W:
            case (uint8)REQTYPE_BROADCAST_W:
            case (uint8)REQTYPE_BROADCAST_W_REV:
            {
                pFrame->uCmd = (uint8)CC2662_REQ_WRITE;
                pFrame->uCmdTyp = NPI_ASYNC_REQ;
                pFrame->uCcReqTyp = CC2662_WD_BQ_RAW_FRAME;
                pFrame->pServiceReq = NULL;
                pFrame->wReqDataLen = (CC2662_REQ_FRAME_FIXED_LEN + (pReqCmd->wReqCmdSiz - 1u));
                uReqCmdInit = TRUE;
                break;
            }
            case (uint8)REQTYPE_STACK_R:
            case (uint8)REQTYPE_SINGLE_R:
            case (uint8)REQTYPE_BROADCAST_R:
            {
                pFrame->uCmd = (uint8)CC2662_REQ_READ;
                pFrame->uCmdTyp = NPI_ASYNC_REQ;
                pFrame->uCcReqTyp = CC2662_WD_BQ_RAW_FRAME;
                pFrame->wReqDataLen = CC2662_REQ_FRAME_FIXED_LEN;
                pFrame->uBlockSiz = ((pReqCmd->pData[0u] & CC2662_REQUEST_BYTE_NUM_MASK) + CC2662_RSP_FRAME_FIXED_LEN);

                pFrame->pServiceReq = pReqCmd->pServiceCfg;
                pFrame->pRxData = (uint8*) pReqCmd->pServiceCfg->pRawData;
                uReqCmdInit = TRUE;
                if((NULL == pFrame->pServiceReq) ||
                   (pFrame->pServiceReq->wDataLen < (uint16) ((pReqCmd->pData[0u] +
                                                               CC2662_RSP_FRAME_FIXED_LEN) * pReqCmd->uDevId)))
                {
                    (void)ctPeQueueAddData(&pCc2662Mgr->zCmdQueFree, pFrame);
                    uRet = (uint8)COMIF_CMD_INVALID_CMDDATA;
                    uReqCmdInit = FALSE;
                }
                break;
            }
            case (uint8)REQTYPE_SINGLE_W:
            {
                pFrame->uCmd = (uint8)CC2662_REQ_WRITE;
                pFrame->uCmdTyp = NPI_ASYNC_REQ;
                pFrame->uCcReqTyp = CC2662_WD_BQ_RAW_FRAME;
                pFrame->pServiceReq = NULL;
                pFrame->wReqDataLen = (CC2662_REQ_FRAME_FIXED_LEN + pReqCmd->wReqCmdSiz);
                uReqCmdInit = TRUE;
                uSingleReq = TRUE;
                break;
            }
            case (uint8)REQTYPE_CUSTOM_ERROR:
            {
                pFrame->uCmd = (uint8)CC2662_REQ_WRITE;
                pFrame->uCmdTyp = NPI_ASYNC_REQ;
                pFrame->uCcReqTyp = CC2662_WD_BQ_RAW_FRAME;
                pFrame->pServiceReq = NULL;
                pFrame->wReqDataLen = (CC2662_REQ_FRAME_FIXED_LEN + pReqCmd->wReqCmdSiz + 1U);
                pFrame->zTxCmd[wIndex++] = 0xFFu;
                uReqCmdInit = TRUE;
                uSingleReq = TRUE;
                break;
            }
            case (uint8)REQTYPE_CUSTOM_DELAY:
            case (uint8)REQTYPE_CUSTOM_INTDELAY:
            case (uint8)REQTYPE_CUSTOM_WAKEUP:
            {
                pFrame->uCcReqTyp = pReqCmd->uReqType;
                pFrame->uCmd = pReqCmd->uReqType;
                for(wData = 0; wData < pReqCmd->wReqCmdSiz; wData++)
                {
                    pFrame->zTxCmd[wIndex++] = pReqCmd->pData[wData];
                }
                uScheduleFrame = TRUE;
                break;
            }
            case (uint8)REQTYPE_GENERIC_W:
            case (uint8)REQTYPE_GENERIC_R:
            {
                uScheduleFrame = ScheduleInternalRequest(pCc2662Mgr, pReqCmd, pFrame);
                if(0U != uScheduleFrame)
                {
                    break;
                }
            }
            default:
            {
                (void)ctPeQueueAddData(&pCc2662Mgr->zCmdQueFree, pFrame);
                uRet = (uint8)COMIF_CMD_INVALID_REQCMD;
                break;
            }
        }
    }
    else
    {
        EMEM_UPDATE_STATICS(pCc2662Mgr->pEmemStats, qNReqQueFail);
        uRet = (uint8)COMIF_CMD_CFQGET_EMPTY;
    }

    if(0U != uReqCmdInit)
    {
        pFrame->zTxCmd[wIndex++] = Cc2662_FrameInitpack(pReqCmd->uReqType, pReqCmd->uGroupExclude, pReqCmd->wReqCmdSiz);
        if(TRUE == uSingleReq)
        {
            pFrame->zTxCmd[wIndex++] = (pReqCmd->uDevId - 1);
        }

        pFrame->zTxCmd[wIndex++] = (uint8)(pReqCmd->wRegAdr >> TIBMS_8BIT_OFFSET) & TIBMS_8BIT_MASK;
        pFrame->zTxCmd[wIndex++] = (uint8)(pReqCmd->wRegAdr) & TIBMS_8BIT_MASK;
        for(wData = 0; wData < pReqCmd->wReqCmdSiz; wData++)
        {
            pFrame->zTxCmd[wIndex++] = pReqCmd->pData[wData];
        }

        wData = CalculateCRC16(pFrame->zTxCmd, wIndex);
        pFrame->zTxCmd[wIndex++] = (uint8)(wData & TIBMS_8BIT_MASK);
        pFrame->zTxCmd[wIndex++] = (uint8)((wData >> TIBMS_8BIT_OFFSET) & TIBMS_8BIT_MASK);
        uScheduleFrame = TRUE;
    }

    if(0U != uScheduleFrame)
    {
        uRet = (uint8)COMIF_CMD_CFQUE_FAILED;
        pFrame->uCmdInfo = pReqCmd->uCmdInfo;
        uPrioReq = COMIF_GET_PRIO(pFrame->uCmdInfo);

        if((uPrioReq < (uint8)COMIF_CMD_PRIO_MAX) &&
           ((uint8)QUEUE_E_OK == ctPeQueueAddData(&pCc2662Mgr->zCmdQueExec[uPrioReq], pFrame)))
        {
            pCc2662Mgr->zCmdQueExec[uPrioReq].nMaxUsage = max(pCc2662Mgr->zCmdQueExec[uPrioReq].nMaxUsage,
                                                   (pCc2662Mgr->zCmdQueExec[uPrioReq].nElements));
            if((uint8)CC2662_STATE_IDLE == pCc2662Mgr->uTxDrvState)
            {
                pCc2662Mgr->uTxDrvState = (uint8)CC2662_STATE_TRANSIENT;
                pCc2662Mgr->pBswCfg->transfer_ctrl(NOTIFY_TX_REQ, NULL, NULL, NULL);
            }
            uRet = (uint8)COMIF_OK;
        }
    }

    pCc2662Mgr->zCmdQueFree.nMaxUsage = max(pCc2662Mgr->zCmdQueFree.nMaxUsage,
                                           (pCc2662Mgr->zCmdQueFree.wSizeQueue-pCc2662Mgr->zCmdQueFree.nElements));
    EMEM_UPDATE_STATICS_DATA(pCc2662Mgr->pEmemStats, qQueMaxUsage, pCc2662Mgr->zCmdQueFree.nMaxUsage);

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_SCHEDULE, uRet, pReqCmd->uReqType, uPrioReq);
}

/**********************************************************************************************************************
 *  FUNC(uint8, cc2662_CODE) Cc2662_SetIntDelay(Cc2662_ManagerType *pCc2662Mgr, uint32 qTimeMs)
 *********************************************************************************************************************/
/*! \brief          Adding Delay request between the commands
 *
 *  \param[in]      pCc2662Mgr: Communication device Manager
 *  \param[in]      qTimeMs: Delay time in Millisecond
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_SetIntDelay(Cc2662_ManagerType *pCc2662Mgr, uint32 qTimeMs)
{
    Comif_ReqCmdType zReqcmd;
    uint8 uRet;
    uint8 uData[CC2662_DELAY_CMD_LEN];

    uData[0u] = (uint8)((qTimeMs + pCc2662Mgr->uSfTime) >> 8u) & TIBMS_8BIT_MASK;
    uData[1u] = (uint8)((qTimeMs + pCc2662Mgr->uSfTime) >> 0u) & TIBMS_8BIT_MASK;

    zReqcmd.uReqType = (uint8)CC2662_REQ_INTDELAY;
    zReqcmd.uGroupExclude = 0u;
    zReqcmd.wReqCmdSiz = CC2662_DELAY_CMD_LEN;
    zReqcmd.uDevId = 0u;
    zReqcmd.wRegAdr = 0u;
    zReqcmd.pData = uData;
    zReqcmd.pServiceCfg = NULL;
    zReqcmd.uCmdInfo = (uint8)COMIF_CMD_PRIO_CTRL;

    uRet = ScheduleFrameRequest(pCc2662Mgr, &zReqcmd);

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, cc2662_CODE) Cc2662_TimeoutNotify(Cc2662_ManagerType *pCc2662Mgr)
 *********************************************************************************************************************/
/*! \brief          Timeout Expired Notification
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_TimeoutNotify(Cc2662_ManagerType *pCc2662Mgr)
{
    Cc2662_TxDataType *pTxData = NULL;
    Cc2662_TxDataType *pMacTxData = NULL;
    Cc2662_ReqDataType *pFrame;
    uint8 uRet = (uint8)COMIF_OK;
    uint8 uMacQueData = FALSE;
    uint16 wIdx = 0u;
    uint16 wNTxPend;

    if(0U != pCc2662Mgr->uTimerProg)
    {
        if(0U != pCc2662Mgr->zTransTd.zSyncTxData.uReqActive)
        {
            wNTxPend = ctPeQueueGetNOfElements(&pCc2662Mgr->zTransTd.zSyncTxData.zTxPendRespQue);
            if(0U != wNTxPend)
            {
                pTxData = &pCc2662Mgr->zTransTd.zSyncTxData;
            }
        }
        else
        {
            wNTxPend = ctPeQueueGetNOfElements(&pCc2662Mgr->zTransTd.zMacTxExecQue);
            while(wIdx < wNTxPend)
            {
                pMacTxData = ctPeQueueGetData(&pCc2662Mgr->zTransTd.zMacTxExecQue);
                if(0U != pMacTxData->uReqActive)
                {
                    uMacQueData = TRUE;
                    pTxData = pMacTxData;
                    break;
                }
                wIdx++;
            }
        }

        if(NULL != pTxData)
        {
#if (CC2662_MAX_RETRY_DISCARD == STD_ON)
            pCc2662Mgr->uRetryStat++;
#endif
            if(CC2662_MAX_RETRY > pCc2662Mgr->uRetryStat)
            {
                pCc2662Mgr->uBQRespStat = (uint8)NPI_RESP_WAIT;
                (void)memset(pCc2662Mgr->pNpiTpBuf, 0u, pCc2662Mgr->wBqNpiTpBufLen);       // Flushing the recieved bytes for the Tx packet
                uRet = (uint8)pCc2662Mgr->pBswCfg->transfer_req((void*) pTxData->npiTxBuf, pTxData->uTxLength, NULL, 0u);
                if(E_OK == uRet)
                {
                    pCc2662Mgr->pBswCfg->timer_req(TIMER_TRANSFER, pTxData->wTxnTimeOut);
                    EMEM_UPDATE_STATICS(pCc2662Mgr->pEmemStats, qNReTxs);
                }
            }
            else
            {
                pTxData->uReqActive = FALSE;
                pCc2662Mgr->uRetryStat = 0u;
                pCc2662Mgr->uTimerProg = 0u;
                wNTxPend = ctPeQueueGetNOfElements(&pTxData->zTxPendRespQue);
                for(wIdx = 0u; wIdx < wNTxPend; wIdx++)
                {
                    pFrame = ctPeQueueRemoveData(&pTxData->zTxPendRespQue);
                    if(NULL != pFrame)
                    {
                        pFrame->uStatus = (uint8)RESP_INVALID;
                        if((uint8)QUEUE_E_OK == ctPeQueueAddData(&pCc2662Mgr->zCmdQueExec[COMIF_CMD_PRIO_PRIO], pFrame))
                        {
                            pCc2662Mgr->pBswCfg->transfer_ctrl(NOTIFY_TX_REQ, NULL, NULL, NULL);
                        }
                    }
                }

                if(0U != uMacQueData)
                {
                    (void)ctPeQueueRemoveData(&pCc2662Mgr->zTransTd.zMacTxExecQue);
                    (void)ctPeQueueAddData(&pCc2662Mgr->zTransTd.zMacTxFreeQue, pTxData);
                }

                EMEM_UPDATE_STATICS(pCc2662Mgr->pEmemStats, qNReTxsRetry);
                pCc2662Mgr->pBswCfg->transfer_ctrl(NOTIFY_TX_REQ, NULL, NULL, NULL);
            }
        }
    }
    else
    {
        if(0U != pCc2662Mgr->uWaitStartupInit)
        {
            // Restart the Startup Init
        }
    }

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_TIMOEUT_NOTIFY, uRet, wNTxPend, pCc2662Mgr->uRetryStat);
}

/**********************************************************************************************************************
 *  FUNC(uint8, cc2662_CODE) Cc2662_DelayNotify(Cc2662_ManagerType *pCc2662Mgr)
 *********************************************************************************************************************/
/*! \brief          Delay Expired Notification
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_DelayNotify(Cc2662_ManagerType *pCc2662Mgr)
{
    uint8 uRet;

    uRet = (uint8)pCc2662Mgr->pBswCfg->transfer_ctrl(NOTIFY_TX_REQ, NULL, NULL, NULL);
    if(TRUE == pCc2662Mgr->uExtDelay)
    {
        pCc2662Mgr->uExtDelay = FALSE;
    }
    else
    {
        pCc2662Mgr->uIntDelay = FALSE;
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, cc2662_CODE) Cc2662_MACStatusVerify(Cc2662_ManagerType *pCc2662Mgr)
 *********************************************************************************************************************/
/*! \brief
 *
 *  \param[in]      pCc2662Mgr: Communication device Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         NULL if fails
 *                  current QUEUE pointer
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_MACStatusVerify(Cc2662_ManagerType *pCc2662Mgr, uint8 uMacStatus)
{
    uint8 uRet = (uint8)COMIF_NOT_OK;

    switch(uMacStatus)
    {
        case (uint8)MAC_STATUS_SUCCESS:
        {
            uRet = (uint8)COMIF_OK;
            break;
        }
        case (uint8)MAC_STATUS_FAIL:
        {
            break;
        }
        case (uint8)MAC_STATUS_NWID_MISMATCH:
        {
            break;
        }
        case (uint8)MAC_STATUS_BUFFER_ERROR:
        {
            uRet = (uint8)COMIF_FATAL_ERROR;
            break;
        }
        case (uint8)MAC_STATUS_FT_ERROR:
        {
            break;
        }
        case (uint8)MAC_STATUS_NO_RESOURCE:
        {
            uRet = (uint8)COMIF_FATAL_ERROR;
            break;
        }
        case (uint8)MAC_STATUS_INVALID_PARAM:
        {
            break;
        }
        case (uint8)MAC_STATUS_INVALID_STATE:
        {
            break;
        }
        case (uint8)MAC_STATUS_INIT_NV_FAILURE:
        {
            break;
        }
        case (uint8)MAC_STATUS_NWCFG_MISMATCH:
        {
            break;
        }
        case (uint8)MAC_STATUS_INVALID_JOINMODE:
        {
            break;
        }
        case (uint8)MAC_STATUS_DUPLICATE_DEVID:
        {
            break;
        }
        case (uint8)MAC_STATUS_DUPLICATE_SLOTID:
        {
            break;
        }
        case (uint8)MAC_STATUS_INVALID_SLOTCFG:
        {
            break;
        }
        case (uint8)MAC_STATUS_DEVLIMIT_REACHED:
        {
            break;
        }
        case (uint8)MAC_STATUS_SCAN_TIMEOUT:
        {
            break;
        }
        case (uint8)MAC_STATUS_PAIR_TIMEOUT:
        {
            break;
        }
        case (uint8)MAC_STATUS_SYNC_INPROG:
        {
            break;
        }
        case (uint8)MAC_STATUS_FALLBACK_INPROG:
        {
            break;
        }
        default:
        {
            break;
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, cc2662_CODE) Cc2662_TimerReTrigger(Cc2662_ManagerType *pCc2662Mgr, Cc2662_TxDataType *pTxCurrent)
 *********************************************************************************************************************/
/*! \brief          Function retriggers the timer for active transfer
 *
 *  \param[in]      pCc2662Mgr: Communication device Manager
 *  \param[in]      pTxCurrent: Current Transcation TD
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         NULL if fails
 *                  current QUEUE pointer
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_TimerReTrigger(Cc2662_ManagerType *pCc2662Mgr, Cc2662_TxDataType *pTxCurrent)
{
    Cc2662_TxDataType *pTxMacTx;
    uint8 uMacIdx;

    pCc2662Mgr->uRetryStat = 0u;
    pTxCurrent->uReqPend = 0;
    if(0U != pTxCurrent->uReqActive)
    {
        pCc2662Mgr->pBswCfg->timer_req(TIMER_TRANSFER, 0u);
        pTxCurrent->uReqActive = FALSE;
        pCc2662Mgr->uTimerProg = FALSE;
    }

    if(FALSE == pCc2662Mgr->uTimerProg)
    {
        for(uMacIdx = 0; uMacIdx < ctPeQueueGetNOfElements(&pCc2662Mgr->zTransTd.zMacTxExecQue); uMacIdx++)
        {
            pTxMacTx = (Cc2662_TxDataType *) ctPeQueueGetData(&pCc2662Mgr->zTransTd.zMacTxExecQue);
            if(0U != pTxMacTx->uReqPend)
            {
                pCc2662Mgr->pBswCfg->timer_req(TIMER_TRANSFER, pTxMacTx->wTxnTimeOut);
                pTxMacTx->uReqActive = TRUE;
                pTxMacTx->uReqPend = FALSE;
                pCc2662Mgr->uTimerProg = TRUE;
                break;
            }
        }
    }

    if(FALSE == pCc2662Mgr->uTimerProg)
    {
        if(0U != pCc2662Mgr->zTransTd.zSyncTxData.uReqPend)
        {
            pCc2662Mgr->pBswCfg->timer_req(TIMER_TRANSFER, pCc2662Mgr->zTransTd.zSyncTxData.wTxnTimeOut);
            pCc2662Mgr->zTransTd.zSyncTxData.uReqPend = FALSE;
            pCc2662Mgr->zTransTd.zSyncTxData.uReqActive = TRUE;
            pCc2662Mgr->uTimerProg = TRUE;
        }
    }

    return (uint8)NPI_RESP_COMPLETE;
}

/**********************************************************************************************************************
 * FUNC(uint8, cc2662_CODE) Cc2662_SyncRxHandle(Cc2662_ManagerType *pCc2662Mgr, uint8 *pNpiRxBuf, uint16 wNpiLen)
 *********************************************************************************************************************/
/*! \brief          Verify the Rx Data and take neccessary actions
 *
 *  \param[in]      pCc2662Mgr: Communication device Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of receive data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_SyncWmRespHandle(Cc2662_ManagerType *pCc2662Mgr, uint8 *pNpiRxBuf, uint32 qSIdx)
{
    Cc2662_ReqDataType *pFrame;
    Cc2662_TxDataType *pTxTransData = NULL;
    uint16 wIdx = 0u;
    uint8 uCmdId;
    uint8 uRet = (uint8)NPI_RESP_WAIT;
    uint8 uMacStatus;
    uint8 uRespExp = TRUE;

    if(0U != ctPeQueueGetNOfElements(&pCc2662Mgr->zTransTd.zSyncTxData.zTxPendRespQue))
    {
        pTxTransData = &pCc2662Mgr->zTransTd.zSyncTxData;
    }

    if(NULL != pTxTransData)
    {
        pFrame = (Cc2662_ReqDataType *)  ctPeQueueGetData(&pTxTransData->zTxPendRespQue);
        if(NULL != pFrame)
        {
            pFrame->uStatus = (uint8)RESP_BUSY;
            uCmdId = pNpiRxBuf[NPI_CMDID_IDX+qSIdx];
            if(uCmdId == pFrame->uCcReqTyp)
            {
                pFrame->uStatus = (uint8)RESP_VALID;
            }

            switch(uCmdId)
            {
                case CC2662_WM_IDENTIFY:
                case CC2662_WM_NUM_WD_CONN:
                case CC2662_WM_NUM_TX_PACKETS:
                case CC2662_WM_NUM_TX_FAILED_PACKETS:
                case CC2662_WM_TX_THROUGHPUT:
                case CC2662_WM_RX_THROUGHPUT:
                case CC2662_APP_DIAG_GETFWVERSION:
                case CC2662_APP_DIAG_GETNETWORKSTATS:
                case CC2662_APP_DIAG_GETTXPACKETS:
                case CC2662_APP_DIAG_GETHROUGHTPUT:
                case CC2662_APP_DIAG_GETRXPACKETS:
                case CC2662_APP_DIAG_GETNWTIME:
                case CC2662_APP_DIAG_GETKLVDURATION:
                case CC2662_APP_DIAG_GETPARAMS:
                case CC2662_APP_DIAG_GETSTATS:
                case CC2662_APP_DIAG_RUNDWM:
                case CC2662_APP_DIAG_DISCOVERREQ:
                {
                    if((uint8)RESP_VALID == pFrame->uStatus)
                    {
                        (void)memcpy((void*) pFrame->pRxData, &pNpiRxBuf[NPI_FRAME_FIXED_LEN+qSIdx],
                               pFrame->pServiceReq->uBlockLen);
                    }
                    break;
                }
                case CC2662_WD_NUM_RX_PACKETS:
                case CC2662_WD_NUM_RX_TIMEOUT_EVENTS:
                case CC2662_WD_LATENCY:
                case CC2662_WD_PER:
                case CC2662_WD_RSSI:
                {
                    if((uint8)RESP_VALID == pFrame->uStatus)
                    {
                        wIdx = (pNpiRxBuf[NPI_NODEID_IDX+pCc2662Mgr->uBufSIdx]*pFrame->pServiceReq->uBlockLen);
                        (void)memcpy((void *) &pFrame->pRxData[wIdx], &pNpiRxBuf[NPI_FRAME_FIXED_LEN+qSIdx],
                               pFrame->pServiceReq->uBlockLen);
                    }
                    break;
                }
                case CC2662_APP_DIAG_SETMAINCONFIG:
                case CC2662_APP_DIAG_STARTNETWORK:
                case CC2662_APP_DIAG_SETPOWERMODE:
                case CC2662_APP_DIAG_SETNWKOPMODE:
                case CC2662_APP_DIAG_SETJOINMODE:
                case CC2662_APP_DIAG_SETDEVTBLCFG:
                case CC2662_APP_DIAG_UNPAIRREQ:
                case CC2662_APP_DIAG_RESYNC:
                {
                    uMacStatus = pNpiRxBuf[NPI_MAC_STATUS_IDX+qSIdx];
                    uRet = Cc2662_MACStatusVerify(pCc2662Mgr, uMacStatus);                     // TODO: MAC Status to be checked and action
                    break;
                }
                case CC2662_APP_DIAG_SETNWTIME:
                case CC2662_APP_DIAG_SETWKUPPRD:
                case CC2662_APP_DIAG_SETPARAMS:
                default:
                {
                    uRespExp = FALSE;
                    break;
                }
            }

            if((uint8)RESP_VALID == pFrame->uStatus)
            {
                pTxTransData->uRespPend--;
                (void)ctPeQueueRemoveData(&pTxTransData->zTxPendRespQue);
                if(0U != uRespExp)
                {
                    if((uint8)QUEUE_E_OK == ctPeQueueAddData(&pCc2662Mgr->zCmdReadQueExec, pFrame))
                    {
                        pCc2662Mgr->pBswCfg->transfer_ctrl(NOTIFY_RX_REQ, NULL, NULL, NULL);
                    }
                }
            }
            else if((uint8)RESP_BUSY != pFrame->uStatus)
            {
                // This not supposed to be happening
                pTxTransData->uRespPend--;
                (void)ctPeQueueRemoveData(&pTxTransData->zTxPendRespQue);
                (void)ctPeQueueAddData(&pCc2662Mgr->zCmdQueExec[COMIF_CMD_PRIO_CTRL], pFrame);
            }

            if(0u == pTxTransData->uRespPend)
            {
                uRet = Cc2662_TimerReTrigger(pCc2662Mgr, pTxTransData);
            }
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, cc2662_CODE) Cc2662_AsyncWmRespHandle(Cc2662_ManagerType *pCc2662Mgr, uint8 uCmdId, uint8 *pNpiRxBuf)
 *********************************************************************************************************************/
/*! \brief          Verify the Rx Data and take neccessary actions
 *
 *  \param[in]      pCc2662Mgr: Communication device Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_AsyncWmRespHandle(Cc2662_ManagerType *pCc2662Mgr, uint8 uCmdId,
                                                         uint8 *pNpiRxBuf, uint32 qSIdx)
{
    const ServiceCfgType *pSvcCfg;
    Comif_ReqCmdType zReqcmd;
    sint8 xSvcId = -1;
    uint8 uValidBuf = FALSE;
    uint8 uRet = (uint8)NPI_RESP_WAIT;
    
    switch(uCmdId)
    {
        case CC2662_APP_DIAG_DEVICEPAIRIND_CB:
         {
             xSvcId = (uint8)COMM_APP_DIAG_DEVICEPAIRIND_CB;
             break;
         }
         case CC2662_APP_DIAG_OAD_CB:
         {
             xSvcId = (uint8)COMM_APP_DIAG_OAD_CB;
             break;
         }
         case CC2662_APP_DIAG_STATECHANGE_CB:
         {
             xSvcId = (uint8)COMM_APP_DIAG_STATECHANGE_CB;
             break;
         }
         case CC2662_APP_DIAG_BQFAULT_CB:
         {
             xSvcId = (uint8)COMM_APP_DIAG_BQFAULT_CB;
             break;
         }
         case CC2662_APP_DIAG_ERROR_CB:
         {
             xSvcId = (uint8)COMM_APP_DIAG_APP_DIAG_ERROR_CB;
             break;
         }
         case CC2662_APP_DIAG_RESET_CB:
         {
             xSvcId = (uint8)COMM_APP_DIAG_RESET_CB;
             break;
         }
         case CC2662_APP_DIAG_EXITPS_CB:
         {
             xSvcId = (uint8)COMM_APP_DIAG_EXITPS_CB;
             break;
         }
         case CC2662_APP_DIAG_FORMATIONTIME_CB:
         {
             pCc2662Mgr->qNwFormationTime |= (uint32)(pNpiRxBuf[NPI_HDR_LEN + qSIdx + 1U ] << 24);
             pCc2662Mgr->qNwFormationTime |= (uint32)(pNpiRxBuf[NPI_HDR_LEN + qSIdx + 2U ] << 16);
             pCc2662Mgr->qNwFormationTime |= (uint32)(pNpiRxBuf[NPI_HDR_LEN + qSIdx + 3U ] << 8);
             pCc2662Mgr->qNwFormationTime |= (uint32)(pNpiRxBuf[NPI_HDR_LEN + qSIdx + 4U ] );
             xSvcId = (uint8)COMM_APP_DIAG_FORMATIONTIME_CB;

             if(0U != pCc2662Mgr->uWaitStartupInit)
             {
                 pCc2662Mgr->uWaitStartupInit = FALSE;
                 pSvcCfg = &pCc2662Mgr->pCommCfg[COMM_GETNUMOFCONN];
                 if((NULL != pSvcCfg) && ((uint8)COMM_GETNUMOFCONN == pSvcCfg->uSubId))
                 {
                     (void)Cc2662_SetIntDelay(pCc2662Mgr, CC2662_BQDRIVER_INIT_DELAY);

                     zReqcmd.uReqType = (uint8)REQTYPE_GENERIC_R;
                     zReqcmd.uGroupExclude = 0u;
                     zReqcmd.wReqCmdSiz = READ_1BYTE;
                     zReqcmd.uDevId = 0u;
                     zReqcmd.pData = NULL;
                     zReqcmd.uCmdInfo =(uint8) COMIF_CMD_PRIO_CTRL;
                     zReqcmd.wRegAdr = COMIF_GET_REG((uint8)COMM_GETNUMOFCONN);
                     zReqcmd.pServiceCfg = pSvcCfg;

                     (void)ScheduleFrameRequest(pCc2662Mgr, &zReqcmd);
                 }
             }
             uValidBuf = TRUE;
             break;
         }
         case CC2662_APP_DIAG_RESYNCCNF_CB:
         {
             xSvcId = (uint8)COMM_APP_DIAG_RESYNCCNF_CB;
             break;
         }
         case CC2662_APP_DIAG_DFU_CB:
         {
             xSvcId = (uint8)COMM_APP_DIAG_DFU_CB;
             break;
         }
         case CC2662_APP_DIAG_DISCOVERIND_CB:
         {
             xSvcId = (uint8)COMM_APP_DIAG_DISCOVERIND_CB;
             break;
         }
         case CC2662_APP_DIAG_DISCOVERCNF_CB:
         {
             xSvcId = (uint8)COMM_APP_DIAG_DISCOVERCNF_CB;
             break;
         }
         case CC2662_WD_STORAGE: // Not Callback, Async Command with 5A
        {
            xSvcId = (uint8)COMM_SENDWDSTORAGE;
            break;
        }
        default:
        {
            break;
        }
     }

     if(xSvcId >= 0u)
     {
         pSvcCfg = &pCc2662Mgr->pCommCfg[xSvcId];
         if((NULL != pSvcCfg) && (xSvcId ==  pSvcCfg->uSubId))
         {
             if((0U != uValidBuf) && (0U != pNpiRxBuf))
             {
                 (void)memcpy(pSvcCfg->pRawData, &pNpiRxBuf[NPI_FRAME_FIXED_LEN + qSIdx], pSvcCfg->uBlockLen);
             }

             pSvcCfg->pSvcStat->nRespMsgs++;
             if(NULL != pSvcCfg->cbApplSvc)
             {
                 pSvcCfg->cbApplSvc(pSvcCfg->uIface, pSvcCfg->uServiceId,
                                    pSvcCfg->uSubId, RESP_VALID, &pNpiRxBuf[NPI_MAC_STATUS_IDX + qSIdx]);
             }

             uRet = (uint8)NPI_RESP_COMPLETE;
         }
     }


    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, cc2662_CODE) Cc2662_AsyncBqRespHandle(Cc2662_ManagerType *pCc2662Mgr)
 *********************************************************************************************************************/
/*! \brief          Verify the Rx Data and take neccessary actions
 *
 *  \param[in]      pCc2662Mgr: Communication device Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_AsyncBqRespHandle(Cc2662_ManagerType *pCc2662Mgr)
{
    Cc2662_ReqDataType *pFrame;
    Cc2662_TxDataType *pMacTxData;
    uint32 qSIdx = 0u;
    uint16 wPendResps = 0u;
    uint16 wNPendTx;
    uint16 wTxQueIdx = 0u;
    uint16 wPendRecheck;
    uint8 uNodeId;
    uint8 uRet = (uint8)NPI_RESP_WAIT;

    wNPendTx = ctPeQueueGetNOfElements(&pCc2662Mgr->zTransTd.zMacTxExecQue);
    while(wTxQueIdx < wNPendTx)
    {
        pMacTxData = (Cc2662_TxDataType *) ctPeQueueGetData(&pCc2662Mgr->zTransTd.zMacTxExecQue);
        if(NULL == pMacTxData)
        {
            break;
        }

        wPendRecheck = FALSE;
        wPendResps = ctPeQueueGetNOfElements(&pMacTxData->zTxPendRespQue);
        if((uint8)NPI_RESP_VALID == pCc2662Mgr->uBQRespStat)
        {
            while(wPendResps)
            {
                pFrame = (Cc2662_ReqDataType *)  ctPeQueueGetData(&pMacTxData->zTxPendRespQue);
                if((NULL != pFrame) && (NULL != pFrame->pRxData))
                {
                    pFrame->uStatus = (uint8)RESP_BUSY;
                    if((pFrame->zTxCmd[NPI_BQ_CMD_IDX] == pCc2662Mgr->pNpiTpBuf[qSIdx+NPI_BQ_RESP_CMD_IDX]) &&
                       (pFrame->zTxCmd[NPI_BQ_CMD_IDX+1u] == pCc2662Mgr->pNpiTpBuf[qSIdx+NPI_BQ_RESP_CMD_IDX+1u]))
                    {
                        for(uNodeId = 0u; uNodeId < pCc2662Mgr->uNAFEs; uNodeId++)
                        {
                            (void)memcpy((void*) &pFrame->pRxData[uNodeId * pFrame->uBlockSiz],
                                   (void*) &pCc2662Mgr->pNpiTpBuf[(uNodeId* pCc2662Mgr->wBqRxBufLen) + qSIdx],
                                   pFrame->uBlockSiz);
                        }
                        qSIdx += pFrame->uBlockSiz;
                        pFrame->uStatus = Cc2662_ResponseCrcCheck(pFrame->pRxData,pFrame->uBlockSiz, pCc2662Mgr->uNAFEs);
                    }
                    else
                    {
                        wPendRecheck = TRUE;
                        (void)ctPeQueueRemoveData(&pCc2662Mgr->zTransTd.zMacTxExecQue);
                        (void)ctPeQueueAddData(&pCc2662Mgr->zTransTd.zMacTxExecQue, pMacTxData);
                        if(++wTxQueIdx < wNPendTx)
                        {
                            break;
                        }
                    }

                    if((uint8)RESP_VALID == pFrame->uStatus)
                    {
                        pMacTxData->uRespPend--;
                        (void)ctPeQueueRemoveData(&pMacTxData->zTxPendRespQue);
                        if((uint8)QUEUE_E_OK == ctPeQueueAddData(&pCc2662Mgr->zCmdReadQueExec, pFrame))
                        {
                            pCc2662Mgr->pBswCfg->transfer_ctrl(NOTIFY_RX_REQ, NULL, NULL, NULL);
                        }
                    }
                    else if((uint8)RESP_BUSY != pFrame->uStatus)
                    {
                        // This not supposed to be happening
                        EMEM_UPDATE_STATICS(pCc2662Mgr->pEmemStats, qNCrcFails);
                        do
                        {
                            pFrame = ctPeQueueRemoveData(&pMacTxData->zTxPendRespQue);
                            if(NULL != pFrame)
                            {
                                (void)ctPeQueueAddData(&pCc2662Mgr->zCmdQueExec[COMIF_CMD_PRIO_PRIO], pFrame);
                            }
                            pMacTxData->uRespPend--;
                            wPendResps = ctPeQueueGetNOfElements(&pMacTxData->zTxPendRespQue);
                        }
                        while(wPendResps);
                        pMacTxData->uReqPend = 0U;
                    }
                    else
                    {
                        break;
                    }
                }
                wPendResps = ctPeQueueGetNOfElements(&pMacTxData->zTxPendRespQue);
            }
        }

        if(0u == wPendResps)
        {
            (void)memset(pCc2662Mgr->pNpiTpBuf, 0, pCc2662Mgr->wBqNpiTpBufLen);
            pCc2662Mgr->uBQRespStat = (uint8)NPI_RESP_WAIT;
            (void)ctPeQueueRemoveData(&pCc2662Mgr->zTransTd.zMacTxExecQue);
            (void)ctPeQueueAddData(&pCc2662Mgr->zTransTd.zMacTxFreeQue, pMacTxData);

            uRet = Cc2662_TimerReTrigger(pCc2662Mgr, pMacTxData);
            break;
        }
        else
        {
            if(FALSE == wPendRecheck)
            {
                (void)ctPeQueueRemoveData(&pCc2662Mgr->zTransTd.zMacTxExecQue);  // Check if Next packet needed to be checked
                (void)ctPeQueueAddData(&pCc2662Mgr->zTransTd.zMacTxExecQue, pMacTxData);
                if(++wTxQueIdx >= wNPendTx)
                {
                    break;
                }
            }
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, cc2662_CODE) Cc2662_RxDataVerify(Cc2662_ManagerType *pCc2662Mgr)
 *********************************************************************************************************************/
/*! \brief          Verify the Rx Data and take neccessary actions
 *
 *  \param[in]      pCc2662Mgr: Communication device Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_RxDataVerify(Cc2662_ManagerType *pCc2662Mgr)
{
    uint8 *pNpiRxBuf;
    uint32 qRxBufIdx;
    uint32 qSIdx;
    uint16 wNpiLength;
    uint8 uRet = (uint8)COMIF_NOT_OK;
    uint8 uPktProc = 0u;
    uint8 uNodeId;
    uint8 uCmdType;
    uint8 uCmdId;

    qRxBufIdx = 0u;
    pNpiRxBuf = (uint8*) &pCc2662Mgr->npiRxBuf[0];
    while(qRxBufIdx < pCc2662Mgr->wNpiRxBufLen)
    {
        qSIdx = qRxBufIdx + pCc2662Mgr->uBufSIdx;
        wNpiLength = (uint16) (((pNpiRxBuf[NPI_LENGTH_MSB_IDX + qSIdx] << 8u) |
                               (pNpiRxBuf[NPI_LENGTH_LSB_IDX + qSIdx])) + (NPI_PROTO_LEN + pCc2662Mgr->uBufSIdx));
        if((pCc2662Mgr->wNpiRxBufLen - qRxBufIdx) >= wNpiLength)
        {
            uRet = Cc2662_FrameCheck(&pNpiRxBuf[qSIdx], (wNpiLength-pCc2662Mgr->uBufSIdx));
            if((uint8)COMIF_OK == uRet)
            {
                uCmdType = (uint8) pNpiRxBuf[NPI_CMDTYPE_IDX + qSIdx];
                if(NPI_ASYNC_RESP == uCmdType)
                {
                    uCmdId = pNpiRxBuf[NPI_CMDID_IDX + qSIdx];
                    if(CC2662_WD_BQ_RAW_FRAME == uCmdId)
                    {
                        uNodeId = pNpiRxBuf[NPI_NODEID_IDX + qSIdx];
                        if(uNodeId <= pCc2662Mgr->uNAFEs)
                        {
                            pCc2662Mgr->uBQRespStat = (uint8)NPI_RESP_CNFWAIT;
                            pCc2662Mgr->wBqRxBufLen = (uint16)(wNpiLength-(NPI_PROTO_LEN + pCc2662Mgr->uBufSIdx));
                            (void)memcpy(&pCc2662Mgr->pNpiTpBuf[(pCc2662Mgr->wBqRxBufLen * uNodeId)],
                                   &pNpiRxBuf[NPI_BQPAYLOAD_IDX + qSIdx], pCc2662Mgr->wBqRxBufLen);
                        }
                    }
                    else
                    {
                        if(CC2662_APP_DIAG_TXCNF_CB == uCmdId)
                        {
                            if((uint8)MAC_STATUS_SUCCESS == pNpiRxBuf[NPI_MAC_STATUS_IDX + qSIdx])
                            {
                                 if((uint8)NPI_RESP_CNFWAIT == pCc2662Mgr->uBQRespStat)
                                 {
                                     pCc2662Mgr->uBQRespStat = (uint8)NPI_RESP_VALID;
                                 }
                                 uRet = Cc2662_AsyncBqRespHandle(pCc2662Mgr);
                             }
                             else
                             {
                                 EMEM_UPDATE_STATICS(pCc2662Mgr->pEmemStats, qNProtReTxs);
                                 pCc2662Mgr->uBQRespStat = (uint8)NPI_RESP_RETRY;
                                 pCc2662Mgr->wBqRxBufLen = 0u;
                             }
                        }
                        else
                        {
                            uRet = Cc2662_AsyncWmRespHandle(pCc2662Mgr, uCmdId, pNpiRxBuf, qSIdx);
                        }
                    }
                }
                else
                {
                    if(NPI_SYNC_RESP == uCmdType)
                    {
                        uRet = Cc2662_SyncWmRespHandle(pCc2662Mgr, pNpiRxBuf, qSIdx);
                    }
                    else
                    {
                        /** This should never Happen!!  **/
                        break;         /** Packet Format Error **/
                    }
                }

                qRxBufIdx += wNpiLength;
                if((uint8)NPI_RESP_COMPLETE == uRet)
                {
                    uPktProc++;
                }
            }
            else
            {
                EMEM_UPDATE_STATICS(pCc2662Mgr->pEmemStats, qNFCSFails);
                break;         /** FCS Error **/
            }
        }
        else
        {
            pCc2662Mgr->wNpiRxBufIdx = (uint8)(qSIdx-pCc2662Mgr->uBufSIdx);
            break;
        }
    }

    return uPktProc;
}

/**********************************************************************************************************************
 *  FUNC(uint8, cc2662_CODE) Cc2662_SeqEndNotify(Cc2662_ManagerType *pCc2662Mgr)
 *********************************************************************************************************************/
/*! \brief          Reception Notification
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_SeqEndNotify(Cc2662_ManagerType *pCc2662Mgr)
{
    uint8 uTrigTxn;
    uint8 uPendingTrig = 0u;
    uint16 wUnProc = 0u;

    if(pCc2662Mgr->wNpiRxBufIdx > 0U)
    {
        wUnProc = (pCc2662Mgr->wNpiRxBufLen - pCc2662Mgr->wNpiRxBufIdx);
        (void)memcpy(pCc2662Mgr->npiRxBuf, &pCc2662Mgr->npiRxBuf[pCc2662Mgr->wNpiRxBufIdx], wUnProc);
        pCc2662Mgr->wNpiRxBufIdx = 0u;
    }
    (void)memset(&pCc2662Mgr->npiRxBuf[wUnProc], 0u, (NPI_MAX_RESPLEN-wUnProc));

    pCc2662Mgr->wNpiRxBufLen = (pCc2662Mgr->wNpiRxBufLenCfg - wUnProc);
    pCc2662Mgr->pBswCfg->transfer_ctrl(GET_RX_INFO, (void*) &pCc2662Mgr->npiRxBuf[wUnProc],
                                       &pCc2662Mgr->wNpiRxBufLen, &uPendingTrig);

    pCc2662Mgr->wNpiRxBufLen += wUnProc;
    uTrigTxn = Cc2662_RxDataVerify(pCc2662Mgr);
    if(0U != uTrigTxn)
    {
        pCc2662Mgr->pBswCfg->transfer_ctrl(NOTIFY_TX_REQ, NULL, NULL, NULL);
    }

    if(0U != uPendingTrig)
    {
        pCc2662Mgr->pBswCfg->transfer_ctrl(NOTIFY_RX_END_REQ, NULL, NULL, NULL);
    }

    return (uint8)COMIF_OK;
}


/**********************************************************************************************************************
 * FUNC(uint8, cc2662_CODE) Cc2662_StartupInit(Cc2662_ManagerType *pCc2662Mgr, uint8 uNAFEs)
 *********************************************************************************************************************/
/*! \brief          Start Up Init Function
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of Init
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_StartupInit(Cc2662_ManagerType *pCc2662Mgr, uint32 uNAFEs)
{
    const ServiceCfgType *pSvcCfg;
    Comif_ReqCmdType zReqcmd;
    uint8 uRet = (uint8)COMIF_NOT_OK;

    pCc2662Mgr->uNAFEs = (uint8) uNAFEs;
    pCc2662Mgr->nDevMacCfg = (uint8)uNAFEs;
    pCc2662Mgr->uDevTblIdx = 0;
    pCc2662Mgr->uSfTime = (uint8)(((CC2662_USER_CFG_T_SLOT_DL_TIME*16U) + (CC2662_USER_CFG_T_SLOT_UL_TIME*16U* uNAFEs))/1000U);
    pCc2662Mgr->wTimeOutCfg = 200;//(pCc2662Mgr->uSfTime * CC2662_TXN_TIMEOUT_FACTOR);

    zReqcmd.uReqType = (uint8)REQTYPE_GENERIC_R;
    zReqcmd.uGroupExclude = 0u;
    zReqcmd.wReqCmdSiz = READ_1BYTE;
    zReqcmd.uDevId = 0u;
    zReqcmd.pData = NULL;
    zReqcmd.uCmdInfo = (uint8)COMIF_CMD_PRIO_CTRL;

    do
    {
#if(CC2662_MANUAL_NETWORK_START == STD_ON)
        uRet = (uint8)COMIF_CMD_NWOPMODE_FAILED;
        pSvcCfg = &pCc2662Mgr->pCommCfg[COMM_APP_DIAG_SETNWKOPMODE];
        if((NULL != pSvcCfg) && ((uint8)COMM_APP_DIAG_SETNWKOPMODE == pSvcCfg->uSubId))
        {
            zReqcmd.wRegAdr = COMIF_GET_REG((uint8)COMM_APP_DIAG_SETNWKOPMODE);
            zReqcmd.pServiceCfg = pSvcCfg;
            uRet = ScheduleFrameRequest(pCc2662Mgr, &zReqcmd);
            if((uint8)COMIF_OK != uRet)
            {
                break;
            }
            (void)Cc2662_SetIntDelay(pCc2662Mgr, 5);
        }

        uRet = (uint8)COMIF_CMD_SETMAINCFG_FAILED;
        pSvcCfg = &pCc2662Mgr->pCommCfg[COMM_APP_DIAG_SETMAINCONFIG];
        if((NULL != pSvcCfg) && ((uint8)COMM_APP_DIAG_SETMAINCONFIG == pSvcCfg->uSubId))
        {
            zReqcmd.wRegAdr = COMIF_GET_REG((uint8)COMM_APP_DIAG_SETMAINCONFIG);
            zReqcmd.pServiceCfg = pSvcCfg;
            uRet = ScheduleFrameRequest(pCc2662Mgr, &zReqcmd);
            if((uint8)COMIF_OK != uRet)
            {
                break;
            }
            (void)Cc2662_SetIntDelay(pCc2662Mgr, 10);
        }

        uRet = (uint8)COMIF_CMD_SETJOINMODE_FAILED;
        pSvcCfg = &pCc2662Mgr->pCommCfg[COMM_APP_DIAG_SETJOINMODE];
        if((NULL != pSvcCfg) && ((uint8)COMM_APP_DIAG_SETJOINMODE == pSvcCfg->uSubId))
        {
            zReqcmd.wRegAdr = COMIF_GET_REG((uint8)COMM_APP_DIAG_SETJOINMODE);
            zReqcmd.pServiceCfg = pSvcCfg;
            uRet = ScheduleFrameRequest(pCc2662Mgr, &zReqcmd);
            if((uint8)COMIF_OK != uRet)
            {
                break;
            }
        }

        if(CC2662_SELECTIVE == pCc2662Mgr->pNwCfg->uNwJoinMode)
        {
            uint8 uMacTableIdx;

            uRet = (uint8)COMIF_CMD_SETDEVTBLCFG_FAILED;
            if(pCc2662Mgr->uNAFEs > pCc2662Mgr->pNwCfg->wMacCfgSiz)
            {
                break;
            }

            (void)Cc2662_SetIntDelay(pCc2662Mgr, 10);

            pCc2662Mgr->uDevTblIdx = 0u;
            uMacTableIdx = (uint8)(pCc2662Mgr->pNwCfg->wMacCfgSiz/NPI_MACTABLE_DEVS_INSLOT) + 1U;
            pSvcCfg = &pCc2662Mgr->pCommCfg[COMM_APP_DIAG_SETDEVTBLCFG];
            if((NULL != pSvcCfg) && ((uint8)COMM_APP_DIAG_SETDEVTBLCFG == pSvcCfg->uSubId))
            {
                while(uMacTableIdx--)
                {
                    zReqcmd.wRegAdr = COMIF_GET_REG((uint8)COMM_APP_DIAG_SETDEVTBLCFG);
                    zReqcmd.pServiceCfg = pSvcCfg;
                    uRet = ScheduleFrameRequest(pCc2662Mgr, &zReqcmd);
                    if((uint8)COMIF_OK != uRet)
                    {
                        break;
                    }
                    (void)Cc2662_SetIntDelay(pCc2662Mgr, 10);
                }
            }
        }

        uRet = (uint8)COMIF_CMD_STARTNETWORK_FAILED;
        (void)Cc2662_SetIntDelay(pCc2662Mgr, 5);
        pSvcCfg = &pCc2662Mgr->pCommCfg[COMM_APP_DIAG_STARTNETWORK];
        if((NULL != pSvcCfg) && ((uint8)COMM_APP_DIAG_STARTNETWORK == pSvcCfg->uSubId))
        {
            zReqcmd.wRegAdr = COMIF_GET_REG((uint8)COMM_APP_DIAG_STARTNETWORK);
            zReqcmd.pServiceCfg = pSvcCfg;
            uRet = ScheduleFrameRequest(pCc2662Mgr, &zReqcmd);
            if((uint8)COMIF_OK != uRet)
            {
                break;
            }
        }
        pCc2662Mgr->uWaitStartupInit = TRUE;
#else

        if(COMIF_OK == uRet)
        {
            uRet = COMIF_NOT_OK;
            pSvcCfg = &pCc2662Mgr->pCommCfg[COMM_GETNUMOFCONN];
            if((pSvcCfg) && (COMM_GETNUMOFCONN == pSvcCfg->uSubId))
            {
                zReqcmd.wRegAdr = COMIF_GET_REG(COMM_GETNUMOFCONN);
                zReqcmd.pServiceCfg = pSvcCfg;
                uRet = ScheduleFrameRequest(pCc2662Mgr, &zReqcmd);
            }
        }
#endif
    }
    while(0);

    return uRet;
}

/**********************************************************************************************************************
 * STATIC FUNC(ctPeQueueType *, cc2662_CODE) GetNextFrameQueue(Cc2662_ManagerType *pCc2662Mgr)
 *********************************************************************************************************************/
/*! \brief          Get Next Frame from QUEUE
 *
 *  \param[in]      pCc2662Mgr: Communication device Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         NULL if fails
 *                  current QUEUE pointer
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(ctPeQueueType *, cc2662_CODE) GetNextFrameQueue(Cc2662_ManagerType *pCc2662Mgr, sint8 xPrioIdxCheck)
{
    ctPeQueueType *pNextQRet = NULL;
    Cc2662_ReqDataType *pCmdFrame;
    ctPeQueueType *pNextQ;
    sint8 xPrioIdx = xPrioIdxCheck;
    uint8 uExclQue;
    uint8 uExclLock;

    do
    {
        pNextQ = &pCc2662Mgr->zCmdQueExec[xPrioIdx];
        if(NULL != pNextQ)
        {
            pCmdFrame = ctPeQueueGetData(pNextQ);
            if(NULL != pCmdFrame)
            {
                uExclQue = (uint8)(0x1u << COMIF_GET_PRIO_EXCL(pCmdFrame->uCmdInfo));
                uExclLock = (!(!(COMIF_GET_PRIO_LOCK(pCmdFrame->uCmdInfo))));
                pCc2662Mgr->uQueLockStat ^= (-uExclLock ^ pCc2662Mgr->uQueLockStat) & uExclQue;
                if(0u == TIBMS_BitTest(pCc2662Mgr->uQueLockStat, xPrioIdx))
                {
                    pNextQRet = pNextQ;
                    break;
                }
            }
        }
        xPrioIdx--;
    }
    while (xPrioIdx >= 0);

    return pNextQRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, cc2662_CODE) Cc2662_HandleDelay(Cc2662_ManagerType *,  Cc2662_ReqDataType *, ctPeQueueType *)
 *********************************************************************************************************************/
/*! \brief          Handle Delay Request
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *  \param[in]      pFrame: Request Data
 *  \param[in]      pFrameQ : CMD QUE
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of Delay
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_HandleDelay(Cc2662_ManagerType *pCc2662Mgr,  Cc2662_ReqDataType *pFrame,
                                                   ctPeQueueType *pFrameQ)
{
    uint16 wDelay;

    wDelay = ((pFrame->zTxCmd[0u] << 8u) | (pFrame->zTxCmd[1u] << 0u));
    if(wDelay > pCc2662Mgr->uSfTime)
    {
        if((uint8)CC2662_REQ_DELAY == pFrame->uCmd)
        {
            pCc2662Mgr->uExtDelay = TRUE;
        }
        else
        {
            pCc2662Mgr->uIntDelay = TRUE;
        }
        pCc2662Mgr->pBswCfg->timer_req(TIMER_DELAY, (wDelay-pCc2662Mgr->uSfTime));
    }
    else
    {
        pCc2662Mgr->pBswCfg->transfer_ctrl(NOTIFY_TX_REQ, NULL, NULL, NULL);
    }

    if(NULL != ctPeQueueRemoveData(pFrameQ))
    {
        (void)ctPeQueueAddData(&pCc2662Mgr->zCmdQueFree, pFrame);
    }

    return (uint8)COMIF_OK;
}

/**********************************************************************************************************************
 * FUNC(uint8, cc2662_CODE) Cc2662_TransferRequest(Cc2662_ManagerType *pCc2662Mgr)
 *********************************************************************************************************************/
/*! \brief          Transfer Request data processing
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_AsyncTransfer(Cc2662_ManagerType *pCc2662Mgr)
{
    ctPeQueueType *pNextFrameQ;
    Cc2662_ReqDataType *pFrame;
    Cc2662_TxDataType *pMacTxData;
    uint8 *pNpiTxBuf;
    uint16 wNpiLength;
    uint8 uRet;
    uint8 uDefPktSiz;
    uint8 uDefIdx;
    uint8 uNCmdAggr;
    uint8 uGoodPkt;

    uRet = (uint8)COMIF_CMD_MTFQUE_FAILED;
    pMacTxData = (Cc2662_TxDataType *) ctPeQueueGetData(&pCc2662Mgr->zTransTd.zMacTxFreeQue);                                   /** Get the Data pointer from the MAC QUEUE */
    if(NULL != pMacTxData)
    {
        uDefIdx = (pCc2662Mgr->uBufSIdx + NPI_FRAME_FIXED_LEN);
        uGoodPkt = 0u;
        uNCmdAggr = 0u;
        wNpiLength = 0;
        uDefPktSiz = 1u;

        pMacTxData->uRespPend = 0u;                                                                                             /** Number of Read Packets **/
        pNpiTxBuf = pMacTxData->npiTxBuf;
        while((uNCmdAggr < NPI_CMD_AGGR_MAX) && (pMacTxData->uRespPend < NPI_CMD_AGGR_MAX_READCMD))
        {
            pNextFrameQ = GetNextFrameQueue(pCc2662Mgr, COMIF_CMD_PRIO_PRIO);                                                   /** Take the Priority Message Out of the QUEUE **/
            pFrame = (Cc2662_ReqDataType*) ctPeQueueGetData(pNextFrameQ);
            if(NULL != pFrame)
            {
                if((uint8)CC2662_REQ_CHECK > pFrame->uCmd)                                                                             /* BQ Command Request formation */
                {
                    if(CC2662_WD_BQ_RAW_FRAME == pFrame->uCcReqTyp)
                    {
                        (void)memcpy(&pNpiTxBuf[wNpiLength+uDefPktSiz+uDefIdx], &pFrame->zTxCmd[0], pFrame->wReqDataLen);             /** Copy BQ Request Command to the NPI Frame payload **/
                        if((uint8)CC2662_REQ_READ == pFrame->uCmd)
                        {
                            (void)ctPeQueueAddData(&pMacTxData->zTxPendRespQue, pFrame);
                            pMacTxData->uRespPend++;
                        }
                        else
                        {
                            (void)ctPeQueueAddData(&pCc2662Mgr->zCmdQueFree, pFrame);                                                 /** Response is received for TX Packet **/
                        }
                        (void)ctPeQueueRemoveData(pNextFrameQ);
                        wNpiLength += (pFrame->wReqDataLen + uDefPktSiz);
                        uDefPktSiz = 0u;
                        uGoodPkt++;
                        uNCmdAggr++;
                    }
                    else
                    {
                        break;      /** This should not come here , Incase of WM/WD commands handled seperately **/
                    }
                }
                else
                {
                    if((uint8)CC2662_REQ_DELAY == pFrame->uCmd)
                    {
                        uRet = Cc2662_HandleDelay(pCc2662Mgr, pFrame, pNextFrameQ);
                        break;                                                                                                  /** Delay will be the last packet with or without command Aggregation **/
                    }
                }
            }
            else
            {
                break;
            }
        }

        if(uGoodPkt > 0U)
        {
            pNpiTxBuf[0u] = 0u;                                                                                 /** Protocol Dummy byte - works in case of SPI or overwritten incase of UART **/
            pNpiTxBuf[pCc2662Mgr->uBufSIdx] = NPI_MSG_SOF;                                                      /** Start of NPI Message **/
            pNpiTxBuf[pCc2662Mgr->uBufSIdx + NPI_LENGTH_LSB_IDX] = (uint8)(wNpiLength & 0xFFU);                         /** Payload length higher byte **/
            pNpiTxBuf[pCc2662Mgr->uBufSIdx + NPI_LENGTH_MSB_IDX] = (uint8)((wNpiLength& 0xFF00U) >> 8u);                /** Payload length lower byte **/
            pNpiTxBuf[pCc2662Mgr->uBufSIdx + NPI_CMDTYPE_IDX] = NPI_ASYNC_REQ;                                  /** Command type asynchronous for BQ Pkt **/

            pNpiTxBuf[pCc2662Mgr->uBufSIdx + NPI_CMDID_IDX] = CC2662_WD_BQ_RAW_FRAME;                           /** Command ID **/
            pNpiTxBuf[pCc2662Mgr->uBufSIdx + NPI_NODEID_IDX] = 0u;                                              /** Device ID Handling **/
            wNpiLength = (wNpiLength + NPI_PROTO_LEN + pCc2662Mgr->uBufSIdx);

            /* Calculate the CRC for Request frame */
            pNpiTxBuf[wNpiLength - NPI_FCS_LEN] = Cc2662_NpiCalcFcs(&pNpiTxBuf[NPI_LENGTH_LSB_IDX + pCc2662Mgr->uBufSIdx],
                                             (wNpiLength - (NPI_MSG_START_LEN + NPI_FCS_LEN + pCc2662Mgr->uBufSIdx)));
            pMacTxData->uTxLength = wNpiLength;
            if((uint8)QUEUE_E_OK == ctPeQueueAddData(&pCc2662Mgr->zTransTd.zMacTxExecQue, pMacTxData))
            {
                (void)ctPeQueueRemoveData(&pCc2662Mgr->zTransTd.zMacTxFreeQue);
                pCc2662Mgr->pBswCfg->transfer_req((void*) pNpiTxBuf, wNpiLength, NULL, 0u);                     /** Failure will be handled in Re-Transmission **/
                EMEM_UPDATE_STATICS(pCc2662Mgr->pEmemStats, qNTxReqs);

                pMacTxData->uReqPend = TRUE;
                pMacTxData->wTxnTimeOut = pCc2662Mgr->wTimeOutCfg;
                if(FALSE == pCc2662Mgr->uTimerProg)
                {
                    pCc2662Mgr->uTimerProg = TRUE;
                    pMacTxData->uReqActive = TRUE;
                    pMacTxData->uReqPend = FALSE;
                    pCc2662Mgr->pBswCfg->timer_req(TIMER_TRANSFER, pMacTxData->wTxnTimeOut);
                }
                uRet = (uint8)COMIF_OK;
            }

            if(uGoodPkt > 1U)
            {
                uGoodPkt = 0u;
                EMEM_UPDATE_STATICS_DATAI(pCc2662Mgr->pEmemStats, qNCmdAggr, uGoodPkt);
            }
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, cc2662_CODE) Cc2662_WdAsyncTransfer(Cc2662_ManagerType *pCc2662Mgr, Cc2662_TxDataType *pTxTransData,
                                                    Cc2662_ReqDataType *pFrame, ctPeQueueType *pExecFrameQ)
 *********************************************************************************************************************/
/*! \brief          Transfer Request data processing
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, cc2662_CODE) Cc2662_WdAsyncTransfer(Cc2662_ManagerType *pCc2662Mgr, Cc2662_TxDataType *pTxTransData,
                                                    Cc2662_ReqDataType *pFrame, ctPeQueueType *pExecFrameQ)
{
    uint8 *pNpiTxBuf;
    uint16 wLength;
    uint8 uDefIdx;
    uint8 uRet = (uint8)COMIF_NOT_OK;

    pNpiTxBuf = pTxTransData->npiTxBuf;
    if((NULL != pNpiTxBuf) && (NULL != pFrame))
    {
        pNpiTxBuf[0u] = 0u;                                                                                             /** Protocol Dummy byte - works in case of SPI or overwritten incase of UART **/
        pNpiTxBuf[pCc2662Mgr->uBufSIdx] = NPI_MSG_SOF;                                                                  /** Start of NPI Message **/
        pNpiTxBuf[pCc2662Mgr->uBufSIdx + NPI_LENGTH_LSB_IDX] = (pFrame->wReqDataLen & 0xFF);                            /** Payload length higher byte **/
        pNpiTxBuf[pCc2662Mgr->uBufSIdx + NPI_LENGTH_MSB_IDX] = ((pFrame->wReqDataLen & 0xFF00) >> 8u);                  /** Payload length lower byte **/
        pNpiTxBuf[pCc2662Mgr->uBufSIdx + NPI_CMDTYPE_IDX] = pFrame->uCmdTyp;                                            /** Command type synchronous or asynchronous **/
        pNpiTxBuf[pCc2662Mgr->uBufSIdx + NPI_CMDID_IDX] = pFrame->uCcReqTyp;                                            /** Command ID **/
        uDefIdx = (pCc2662Mgr->uBufSIdx + NPI_FRAME_FIXED_LEN);
        
        if((pFrame->uCcReqTyp == CC2662_APP_DIAG_OADREQ) || (pFrame->uCcReqTyp == CC2662_APP_DIAG_DFUREQ) ||
                (pFrame->uCcReqTyp == CC2662_WD_STORAGE))
        {
			if((pFrame->wReqDataLen > 0U) )
			{
			    (void)memcpy(&pNpiTxBuf[uDefIdx], &pCc2662Mgr->npiWdBuf, pFrame->wReqDataLen);                                        /** Load the Data from OAD Buffer **/
			}
			wLength = (pFrame->wReqDataLen + NPI_PROTO_LEN + pCc2662Mgr->uBufSIdx);
			pNpiTxBuf[wLength - NPI_FCS_LEN] = Cc2662_NpiCalcFcs(&pNpiTxBuf[NPI_LENGTH_LSB_IDX + pCc2662Mgr->uBufSIdx],               /** Calculate the CRC for Request frame **/
			                                        (wLength - (NPI_MSG_START_LEN + NPI_FCS_LEN + pCc2662Mgr->uBufSIdx)));
			pTxTransData->uTxLength = wLength;
            
			(void)ctPeQueueAddData(&pCc2662Mgr->zCmdQueFree, pFrame);  
			(void)ctPeQueueRemoveData(pExecFrameQ);
			pCc2662Mgr->pBswCfg->transfer_req((void*) pNpiTxBuf, wLength, NULL, 0u);                                     /** Transfer the request, No Rx pending Queue for OAD  **/
            
			pTxTransData->uReqPend = FALSE;
			pTxTransData->uReqActive = FALSE;
			EMEM_UPDATE_STATICS(pCc2662Mgr->pEmemStats, qNTxReqs);            
        }
    }
    else
    {
        // CC2662_ASSERT(0);
    }
    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, cc2662_CODE) Cc2662_SyncTransfer(Cc2662_ManagerType *pCc2662Mgr, uint8 *pNpiTxBuf,
 *                                             Cc2662_ReqDataType *pFrame)
 *********************************************************************************************************************/
/*! \brief          Transfer Request data processing
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, cc2662_CODE) Cc2662_SyncTransfer(Cc2662_ManagerType *pCc2662Mgr, Cc2662_TxDataType *pTxTransData,
                                                    Cc2662_ReqDataType *pFrame, ctPeQueueType *pExecFrameQ)
{
    uint8 *pNpiTxBuf;
    uint16 wLength;
    uint8 uDefIdx;
    uint8 uRet = (uint8)COMIF_NOT_OK;

    pNpiTxBuf = pTxTransData->npiTxBuf;
    if((NULL != pNpiTxBuf) && (NULL != pFrame))
    {
        pNpiTxBuf[0u] = 0u;                                                                                             /** Protocol Dummy byte - works in case of SPI or overwritten incase of UART **/
        pNpiTxBuf[pCc2662Mgr->uBufSIdx] = NPI_MSG_SOF;                                                                  /** Start of NPI Message **/
        pNpiTxBuf[pCc2662Mgr->uBufSIdx + NPI_LENGTH_LSB_IDX] = (pFrame->wReqDataLen & 0xFF);                            /** Payload length higher byte **/
        pNpiTxBuf[pCc2662Mgr->uBufSIdx + NPI_LENGTH_MSB_IDX] = ((pFrame->wReqDataLen & 0xFF00) >> 8u);                  /** Payload length lower byte **/
        pNpiTxBuf[pCc2662Mgr->uBufSIdx + NPI_CMDTYPE_IDX] = pFrame->uCmdTyp;                                            /** Command type synchronous or asynchronous **/
        pNpiTxBuf[pCc2662Mgr->uBufSIdx + NPI_CMDID_IDX] = pFrame->uCcReqTyp;                                            /** Command ID **/

        uDefIdx = (pCc2662Mgr->uBufSIdx + NPI_FRAME_FIXED_LEN);
        if(pFrame->wReqDataLen > 0U)
        {
            if(CC2662_APP_DIAG_SETDEVTBLCFG != pFrame->uCcReqTyp)
            {
                (void)memcpy(&pNpiTxBuf[uDefIdx], &pFrame->zTxCmd[0], pFrame->wReqDataLen);
            }
            else
            {
                if(pCc2662Mgr->uDevTblSendIdx < pCc2662Mgr->uDevTblIdx)                                                 /** MAC Table Handling for seperate Memory **/
                {
                    (void)memcpy(&pNpiTxBuf[uDefIdx], pCc2662Mgr->npiGenBuf[(pCc2662Mgr->uDevTblIdx)-1U], pFrame->wReqDataLen);
                    pCc2662Mgr->uDevTblSendIdx++;                                                                           // TODO: Error IN case of wrong Table Index
                }
            }
        }

        pTxTransData->wTxnTimeOut = pCc2662Mgr->wTimeOutCfg;
        if(CC2662_APP_DIAG_STARTNETWORK == pFrame->uCcReqTyp)
        {
            pTxTransData->wTxnTimeOut = CC2662_MANUAL_VOL_STARTUP_TIME;
        }
        else
        {
            if(CC2662_WM_RESET_DEVICE == pFrame->uCcReqTyp)
            {
                pTxTransData->wTxnTimeOut = CC2662_MANUAL_PERS_STARTUP_TIME;
            }
        }

        wLength = (pFrame->wReqDataLen + NPI_PROTO_LEN + pCc2662Mgr->uBufSIdx);
        pNpiTxBuf[wLength - NPI_FCS_LEN] = Cc2662_NpiCalcFcs(&pNpiTxBuf[NPI_LENGTH_LSB_IDX + pCc2662Mgr->uBufSIdx],     /** Calculate the CRC for Request frame **/
                                              (wLength - (NPI_MSG_START_LEN + NPI_FCS_LEN + pCc2662Mgr->uBufSIdx)));
        pTxTransData->uTxLength = wLength;
        if((uint8)QUEUE_E_OK == ctPeQueueAddData(&pTxTransData->zTxPendRespQue, pFrame))
        {
            (void)ctPeQueueRemoveData(pExecFrameQ);
            pTxTransData->uRespPend++;
            pTxTransData->uReqPend = TRUE;

            pCc2662Mgr->pBswCfg->transfer_req((void*) pNpiTxBuf, wLength, NULL, 0u);                                    /** In-case failed for Transfer, retry will take care of this transfer **/
            if(FALSE == pCc2662Mgr->uTimerProg)
            {
                pCc2662Mgr->uTimerProg = TRUE;
                pTxTransData->uReqActive = TRUE;
                pTxTransData->uReqPend = FALSE;
                pCc2662Mgr->pBswCfg->timer_req(TIMER_TRANSFER, pTxTransData->wTxnTimeOut);
            }
            EMEM_UPDATE_STATICS(pCc2662Mgr->pEmemStats, qNTxReqs);
            uRet = (uint8)COMIF_OK;
        }
    }

    return uRet;
}

/*********************************************************************************************************************
 *  External Functions Definition
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  FUNC(uint8, cc2662_CODE) Cc2662_HandleTransfer(void *pComifCtx)
 *********************************************************************************************************************/
/*! \brief          Transfer Request data processing
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, cc2662_CODE) Cc2662_HandleTransfer(void *pComifCtx)
{
    Cc2662_ManagerType *pCc2662Mgr = (Cc2662_ManagerType*) pComifCtx;
    ctPeQueueType *pNextFrameQ;
    Cc2662_ReqDataType *pFrame;
    sint8 xPrioIdxCheck = (uint8)COMIF_CMD_PRIO_CTRL;
    uint8 uReqCheckCont = 0;
    uint8 uReqRetrig = FALSE;
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;

    if(CC2662_INTEGRITY_CHECK(pCc2662Mgr))
    {
        uRet = (uint8)COMIF_CMD_CEQUE_FAILED;
        do
        {
            pFrame = (Cc2662_ReqDataType*) ctPeQueueGetData(&pCc2662Mgr->zCmdQueExec[COMIF_CMD_PRIO_EXCL]);                                 /** Checking Priority Ctrl Packet first **/
            if(NULL != pFrame)
            {
                uRet = Cc2662_SyncTransfer(pCc2662Mgr, &pCc2662Mgr->zTransTd.zSyncTxData, pFrame,                                           /** Not checking Busy since we send the Ctrl Packet again **/
                                           &pCc2662Mgr->zCmdQueExec[COMIF_CMD_PRIO_EXCL]);
                break;                                                                                                                      /** No Need to send any other packet bcz of Ctrl request **/
            }
            else
            {
                pNextFrameQ = GetNextFrameQueue(pCc2662Mgr, xPrioIdxCheck);                                                                 /** Take the Priority Message Out of the QUEUE **/
                pFrame = (Cc2662_ReqDataType*) ctPeQueueGetData(pNextFrameQ);
                if(NULL != pFrame)
                {
                    if((uint8)CC2662_REQ_CHECK > pFrame->uCmd)                                                                                     /* BQ Command Request formation */
                    {
                        if(NPI_ASYNC_REQ == pFrame->uCmdTyp)
                        {
                            if((pFrame->uCcReqTyp == CC2662_APP_DIAG_OADREQ) || (pFrame->uCcReqTyp == CC2662_APP_DIAG_DFUREQ)
                                    || (pFrame->uCcReqTyp == CC2662_WD_STORAGE))
                            {
                                uRet = Cc2662_WdAsyncTransfer(pCc2662Mgr, &pCc2662Mgr->zTransTd.zSyncTxData, pFrame,                         /** Not checking Busy since we send the Ctrl Packet again **/
                                                          &pCc2662Mgr->zCmdQueExec[COMIF_CMD_PRIO_CTRL]);
                                if((uint8)COMIF_OK == uRet)
                                {
                                    uReqCheckCont = 0u;
                                    break;
                                }
                            }
                            else if(FALSE == pCc2662Mgr->uExtDelay)
                            {
                                uRet = Cc2662_AsyncTransfer(pCc2662Mgr);
                                if((uint8)COMIF_OK == uRet)
                                {
                                    uReqRetrig = TRUE;
                                    break;
                                }
                                else
                                {
                                    if(0u == uReqCheckCont++)
                                    {
                                        xPrioIdxCheck = (uint8)COMIF_CMD_PRIO_CTRL;                                                                /** Re-Check if Any Excl WM/WD Messages for sending **/
                                        continue;
                                    }
                                }
                            }
                        }
                        else
                        {
                            if(NPI_SYNC_REQ == pFrame->uCmdTyp)
                            {
                                if((0u == pCc2662Mgr->zTransTd.zSyncTxData.uRespPend) && (FALSE == pCc2662Mgr->uIntDelay))                 /** Only can send One by One Sync Packet **/
                                {
                                    uRet = Cc2662_SyncTransfer(pCc2662Mgr, &pCc2662Mgr->zTransTd.zSyncTxData,
                                                               pFrame, pNextFrameQ);
                                    if((uint8)COMIF_OK == uRet)
                                    {
                                        uReqRetrig = TRUE;
                                        break;
                                    }
                                }
                                else
                                {
                                    if(0u == uReqCheckCont++)
                                    {
                                        xPrioIdxCheck = (uint8)COMIF_CMD_PRIO_PRIO;                                                               /** Re-Check if Any BQ Messages for sending **/
                                        continue;
                                    }
                                }
                            }
                        }

                        if((uint8)COMIF_OK != uRet)
                        {
                            break;
                        }
                    }
                    else
                    {
                        if((uint8)CC2662_REQ_DELAY <= pFrame->uCmd)
                        {
                            (void)Cc2662_HandleDelay(pCc2662Mgr, pFrame, pNextFrameQ);
                        }
                        break;                      /** Delay will be the last packet with or without command Aggregation **/
                    }
                }
                else
                {
                    if(0u == uReqCheckCont++)
                    {
                        xPrioIdxCheck = (uint8)COMIF_CMD_PRIO_EXCL;
                        continue;
                    }
                    else
                    {
                        pCc2662Mgr->uTxDrvState = (uint8)CC2662_STATE_IDLE;
                        break;
                    }
                }
            }
        }
        while(1);

        if((TRUE == uReqRetrig) && ctPeQueueGetNOfElements(&pCc2662Mgr->zTransTd.zMacTxFreeQue))
        {
            pNextFrameQ = GetNextFrameQueue(pCc2662Mgr, COMIF_CMD_PRIO_EXCL);
            if(NULL != pNextFrameQ)
            {
                pCc2662Mgr->pBswCfg->transfer_ctrl(NOTIFY_TX_REQ, NULL, NULL, NULL);
            }
        }
    }

    return (uint8)COMIF_OK;
}

/**********************************************************************************************************************
 *  FUNC(uint8, cc2662_CODE) Cc2662_HandleRecieve(void *pComifCtx)
 *********************************************************************************************************************/
/*! \brief          Recieved Data processing
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, cc2662_CODE) Cc2662_HandleRecieve(void *pComifCtx)
{
    Cc2662_ManagerType *pCc2662Mgr = (Cc2662_ManagerType*) pComifCtx;
    Cc2662_ReqDataType *pCmdFrame;
    uint8 uRet = (uint8)COMIF_CMD_INTEGRITY_FAILED;
    uint16 wNElements;
    uint16 wIndex;

    if(CC2662_INTEGRITY_CHECK(pCc2662Mgr))
    {
        wNElements = ctPeQueueGetNOfElements(&pCc2662Mgr->zCmdReadQueExec);
        for(wIndex = 0; wIndex < wNElements; wIndex++)
        {
            uRet = (uint8)COMIF_CMD_INVALID_READ;
            pCmdFrame = ctPeQueueRemoveData(&pCc2662Mgr->zCmdReadQueExec);
            if(NULL != pCmdFrame)
            {
                if((NULL != pCmdFrame->pRxData) && (pCmdFrame->uBlockSiz > 0U))
                {
                    EMEM_UPDATE_STATICS(pCc2662Mgr->pEmemStats, qNRxResps);
                    if(NULL != pCmdFrame->pServiceReq->cbRxDataSvc)
                    {
                        uRet = pCmdFrame->pServiceReq->cbRxDataSvc(pCmdFrame->pServiceReq, pCmdFrame->pRxData,
                                                                   pCmdFrame->uStatus);
                    }
                }

                if((uint8)QUEUE_E_NOT_OK == ctPeQueueAddData(&pCc2662Mgr->zCmdQueFree, pCmdFrame))
                {
                    uRet = Comif_SetErrorDetails(COMIF_SVCID_CMD_RX_PROC, COMIF_CMD_CFQUE_FAILED, uRet, wIndex);
                }
            }
            else
            {
                uRet = Comif_SetErrorDetails(COMIF_SVCID_CMD_RX_PROC, uRet, wNElements, wIndex);
            }
        }
    }

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_RX_PROC, uRet, wNElements, 0u);
}

/**********************************************************************************************************************
 *  FUNC(uint8, cc2662_CODE) Cc2662_Notify(void *pComifCtx, uint8 uNotifyType)
 *********************************************************************************************************************/
/*! \brief          notification from external processing
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *  \param[in]      uNotifyType: Type of Notification
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of notification process
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, cc2662_CODE) Cc2662_Notify(void *pComifCtx, uint8 uNotifyType)
{
    Cc2662_ManagerType *pCc2662Mgr = (Cc2662_ManagerType*) pComifCtx;
    uint8 uRet = (uint8)COMIF_CMD_INTEGRITY_FAILED;

    if(CC2662_INTEGRITY_CHECK(pCc2662Mgr))
    {
        switch (uNotifyType)
        {
            case (uint8)TRANSFER_COMPLETE_NOTIFY:
            {
                uRet = Cc2662_SeqEndNotify(pCc2662Mgr);                 /** Handling End of Transfer **/
                break;
            }
            case (uint8)TRANSFER_TIMEOUT_NOTIFY:
            {
                uRet = Cc2662_TimeoutNotify(pCc2662Mgr);                /** Handling of Tx Timeout **/
                break;
            }
            case (uint8)TRANSFER_DELAY_NOTIFY:
            {
                uRet = Cc2662_DelayNotify(pCc2662Mgr);                  /** Timeout Notification **/
                break;
            }
            default:
            {
                uRet = (uint8)COMIF_CMD_INVALID_NOTIFY;
                break;
            }
        }
    }

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_NOTIFY, uRet, uNotifyType, 0u);
}

/**********************************************************************************************************************
 *  FUNC(uint8, cc2662_CODE) Cc2662_Control(void *pComifCtx, uint8 uCmd, void *pData)
 *********************************************************************************************************************/
/*! \brief          Input Output control processing
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *  \param[in]      uCmd: command data
 *  \param[in]      pData: pointer to data corresponding to the command
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of input out control request
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, cc2662_CODE) Cc2662_Control(void *pComifCtx, uint8 uCmd, void *pData)
{
    Cc2662_ManagerType *pCc2662Mgr = (Cc2662_ManagerType*) pComifCtx;
    uint8 uRet = (uint8)COMIF_CMD_INTEGRITY_FAILED;
    Comif_ReqCmdType *pReqCmd;
    VersionInfoType *pCmdVerInfo;
    uint32 eData = 0u;
    uint8 uLength;

    if(CC2662_INTEGRITY_CHECK(pCc2662Mgr))
    {
        uRet = (uint8)COMIF_CMD_INVALID_REQDATA;
        switch(uCmd)
        {
            case (uint8)COMIF_CMD_SGLREAD:
            {
                pReqCmd = (Comif_ReqCmdType*) pData;
                if(NULL != pReqCmd)
                {
                    if((NULL != pReqCmd->pData) && (pReqCmd->uDevId <= CC2662_DEV_ADR_RANGE_MAX))
                    {
                        uLength = (uint8) ((uint8) pReqCmd->pData[0] - (uint8) CC2662_READ_LEN_OFFSET);
                        pReqCmd->pData = &uLength;
                        uRet = ScheduleFrameRequest(pCc2662Mgr, pReqCmd);
                    }
                }
                break;
            }
            case (uint8)COMIF_CMD_SGLWRITE:
            {
                pReqCmd = (Comif_ReqCmdType*) pData;
                if(NULL != pReqCmd)
                {
                    if((NULL != pReqCmd->pData) && (pReqCmd->uDevId <= CC2662_DEV_ADR_RANGE_MAX) &&
                       (pReqCmd->wReqCmdSiz <= CC2662_WRITE_DATA_LEN_MAX) &&
                       (pReqCmd->wReqCmdSiz >= CC2662_WRITE_DATA_LEN_MIN))
                    {
                        pReqCmd->uReqType = (uint8)REQTYPE_SINGLE_W;                           /** To Make COMIF CC2662 Compactable **/
                        pReqCmd->uGroupExclude = 0;                                     /** For Broad cast Write */
                        uRet = ScheduleFrameRequest(pCc2662Mgr, pReqCmd);
                    }
                }
                break;
            }
            case (uint8)COMIF_CMD_STKREAD:
            case (uint8)COMIF_CMD_ALLREAD:
            {
                pReqCmd = (Comif_ReqCmdType*) pData;
                if(NULL != pReqCmd)
                {
                    if((NULL != pReqCmd->pData) && (pReqCmd->uDevId <= CC2662_DEV_ADR_RANGE_MAX) &&
                       (pReqCmd->wReqCmdSiz <= CC2662_WRITE_DATA_LEN_MAX) &&
                       (pReqCmd->wReqCmdSiz >= CC2662_WRITE_DATA_LEN_MIN))
                    {
                        uLength = ((uint8) *pReqCmd->pData) - CC2662_READ_LEN_OFFSET;
                        pReqCmd->pData = &uLength;
                        pReqCmd->uReqType = (uint8)REQTYPE_BROADCAST_R;                        /** To Make COMIF CC2662 Compactable **/
                        uRet = ScheduleFrameRequest(pCc2662Mgr, pReqCmd);
                    }
                }
                break;
            }
            case (uint8)COMIF_CMD_STKWRITE:
            case (uint8)COMIF_CMD_ALLWRITE:
            {
                pReqCmd = (Comif_ReqCmdType*) pData;
                if(NULL != pReqCmd)
                {
                    if(NULL != (pReqCmd->pData) && (pReqCmd->wReqCmdSiz <= CC2662_WRITE_DATA_LEN_MAX) &&
                       (pReqCmd->wReqCmdSiz >= CC2662_WRITE_DATA_LEN_MIN))
                    {
                        pReqCmd->uReqType = (uint8)REQTYPE_BROADCAST_W; /** To Make COMIF CC2662 Compactable **/
                        pReqCmd->uGroupExclude = 0;
                        uRet = ScheduleFrameRequest(pCc2662Mgr, pReqCmd);
                    }
                }
                break;
            }
            case (uint8)COMIF_CMD_GNRCREAD:
            case (uint8)COMIF_CMD_GNRCWRITE:
            {
                pReqCmd = (Comif_ReqCmdType*) pData;
                if(NULL != pReqCmd)
                {
                    uRet = ScheduleFrameRequest(pCc2662Mgr, pReqCmd);
                }
                break;
            }
            case (uint8)COMIF_CMD_DELAY:
            {
                pReqCmd = (Comif_ReqCmdType*) pData;
                if(NULL != pReqCmd)
                {
                    uRet = ScheduleFrameRequest(pCc2662Mgr, pReqCmd);
                }
                break;
            }
            case (uint8)COMIF_CMD_WAKEUP:
            {
                if(NULL != pData)
                {
                    eData = (uint32) (*((uint32*) pData));
                    uRet = Cc2662_StartupInit(pCc2662Mgr, eData);
                }
                break;
            }
            case (uint8)COMIF_CMD_ERROR:
            {
                pReqCmd = (Comif_ReqCmdType*) pData;
                if(NULL != pReqCmd)
                {
                    if((NULL != pReqCmd->pData) && (pReqCmd->uDevId <= CC2662_DEV_ADR_RANGE_MAX) &&
                       (pReqCmd->wReqCmdSiz <= CC2662_WRITE_DATA_LEN_MAX) &&
                       (pReqCmd->wReqCmdSiz >= CC2662_WRITE_DATA_LEN_MIN))
                    {
                        uRet = ScheduleFrameRequest(pCc2662Mgr, pReqCmd);
                    }
                }
                break;
            }
            case (uint8)COMIF_CMD_GETVERSION:
            {
                pCmdVerInfo = (VersionInfoType*) pData;
                if(NULL != pCmdVerInfo)
                {
                    pCmdVerInfo->zCmdVersion.vendorID = CC2662_VENDOR_ID;
                    pCmdVerInfo->zCmdVersion.moduleID = CC2662_MODULE_ID;
                    pCmdVerInfo->zCmdVersion.sw_major_version = CC2662_SW_MAJOR_VERSION;
                    pCmdVerInfo->zCmdVersion.sw_minor_version = CC2662_SW_MINOR_VERSION;
                    pCmdVerInfo->zCmdVersion.sw_patch_version = CC2662_SW_PATCH_VERSION;
                    uRet = E_OK;
                }
                break;
            }
            case (uint8)COMIF_CMD_UNKNOWN:
            default:
            {
                uRet = (uint8)COMIF_CMD_INVALID_REQCMD;
                break;
            }
        }
    }

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_CTRL_PROC, uRet, uCmd, eData);
}

/**********************************************************************************************************************
 * FUNC(void*, cc2662_CODE) Cc2662_Init(const void *, Emem_StasticsType *, const ServiceCfgType *)
 *********************************************************************************************************************/
/*! \brief          Initialization of communication device
 *
 *  \param[in]      pComCfgCtx: Communication device configuration
 *  \param[in]      pEStatsCfg: Statics update
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the communication device manager context
 *  \retval         void *
 *
 *  \trace
 *********************************************************************************************************************/

FUNC(void*, cc2662_CODE) Cc2662_Init(const void *pComCfgCtx, Emem_StasticsType *pEStatsCfg,
                                     const ServiceCfgType *pCommCfg)
{
    Cc2662_ConfigType *pCc2662Cfg = (Cc2662_ConfigType*) pComCfgCtx;
    Cc2662_ManagerType *pCc2662MgrRet = NULL;
    Cc2662_ManagerType *pCc2662Mgr;
    uint8 uCmdIfaceId = 0xFF;
    uint8 uRet = (uint8)COMIF_NOT_OK;

    do
    {
        if(NULL == pCc2662Cfg)
        {
            uRet = (uint8)COMIF_CMD_INVALID_CFG;
            break;
        }

        if(pCc2662Cfg->uCmdIfaceCfg >= CC2662_IFACES)
        {
            uRet = (uint8)COMIF_CMD_INVALID_IFACE_CFG;
            break;
        }

        uCmdIfaceId = pCc2662Cfg->uCmdIfaceCfg;
        pCc2662Mgr = &zCc2662Ctx[pCc2662Cfg->uCmdIfaceCfg];
        if(NULL == pCc2662Mgr)
        {
            uRet = (uint8)COMIF_CMD_INVALID_MGR;
            break;
        }

        if(CC2662_INIT == pCc2662Mgr->uInit)
        {
            uRet = (uint8)COMIF_CMD_ALREADY_INUSE;
            break;
        }

        (void)memset(pCc2662Mgr, 0, sizeof(Cc2662_ManagerType));
        if(NULL == pCommCfg)
        {
            uRet = (uint8)COMIF_CMD_INVALID_COMMCFG;
            break;
        }

        pCc2662Mgr->pCommCfg = pCommCfg;
        if(NULL == pCc2662Cfg->pNwCfg)
        {
            uRet = (uint8)COMIF_CMD_INVALID_NWCFG;
            break;
        }

        pCc2662Mgr->pNwCfg = pCc2662Cfg->pNwCfg;
        uRet = Cc2662_QueInit(pCc2662Mgr, pCc2662Cfg);
        if((uint8)COMIF_OK != uRet)
        {
            uRet = (uint8)COMIF_CMD_INVALID_QUECFG;
            break;
        }

        if(NULL == pCc2662Cfg->pNpiTpBufMemCfg)
        {
            uRet = (uint8)COMIF_CMD_INVALID_TPBUF;
            break;
        }
        pCc2662Mgr->pNpiTpBuf = pCc2662Cfg->pNpiTpBufMemCfg;

        if(NULL == pCc2662Cfg->pBswCfg)
        {
            uRet = (uint8)COMIF_CMD_INVALID_BSW;
            break;
        }

        pCc2662Mgr->pBswCfg = pCc2662Cfg->pBswCfg;
        uRet = (uint8)pCc2662Mgr->pBswCfg->init();
        if(E_OK != uRet)
        {
            uRet = (uint8)COMIF_CMD_INVALID_BSW_INIT;
            break;
        }

        pCc2662Mgr->uBufSIdx = pCc2662Mgr->pNwCfg->uProtoSByte;
        pCc2662Mgr->wNpiRxBufLenCfg = pCc2662Mgr->pNwCfg->wNwPktLength;
        pCc2662Mgr->wBqNpiTpBufLen = pCc2662Cfg->wBqNpiTpBufLenCfg;
        pCc2662Mgr->pEmemStats = pEStatsCfg;
        pCc2662Mgr->uCmdIface = pCc2662Cfg->uCmdIfaceCfg;

        pCc2662Mgr->uTxDrvState = (uint8)CC2662_STATE_IDLE;
        pCc2662Mgr->uBQRespStat = (uint8)NPI_RESP_WAIT;
        pCc2662Mgr->uQueLockStat = 0u;

        pCc2662Mgr->qGuardStart = CC2662_GUARD_START;
        pCc2662Mgr->qGuardEnd = CC2662_GUARD_END;
        pCc2662Mgr->uInit = CC2662_INIT;

        pCc2662MgrRet = pCc2662Mgr;
        uRet = (uint8)COMIF_OK;
    }
    while (0);

    (void)Comif_SetErrorDetails(COMIF_SVCID_CMD_INIT, uRet, uCmdIfaceId, 0u);

    return pCc2662MgrRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, cc2662_CODE) Cc2662_Deinit(void *pComifCtx)
 *********************************************************************************************************************/
/*! \brief          De-Initialization of communication device
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of De-init
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, cc2662_CODE) Cc2662_Deinit(void *pComifCtx)
{
    Cc2662_ManagerType *pCc2662Mgr = (Cc2662_ManagerType*) pComifCtx;
    uint8 uRet = (uint8)COMIF_CMD_INTEGRITY_FAILED;

    if(CC2662_INTEGRITY_CHECK(pCc2662Mgr))
    {
        uRet = (uint8)pCc2662Mgr->pBswCfg->deinit();
        (void)memset(pCc2662Mgr, 0, sizeof(Cc2662_ManagerType));
    }

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_DEINIT, uRet, 0u, 0u);
}

#define CC26662_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * End of File: cc2662.c
 *********************************************************************************************************************/
