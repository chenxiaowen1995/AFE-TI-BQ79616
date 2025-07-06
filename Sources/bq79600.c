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
 *  File:       bq79600.c
 *  Project:    TIBMS
 *  Module:     COMIF
 *  Generator:  Code generation tool (ifany)
 *
 *  Description:  Functionalities for Bq79600 Bridge Device
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05May2022    SEM                  0000000000000    Initial version
 * 01.00.01       05April2023  SEM                  0000000000000    Updated Bsw cfg link to invoke BSW calls
 *
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Standard Header Files
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Other Header Files
 *********************************************************************************************************************/

#include "tibms_api.h"
#include "tibms_comif.h"
#include "tibms_bmi.h"
#include "tibms_utils.h"

/**********************************************************************************************************************
 * Version Check (ifrequired)
 *********************************************************************************************************************/

/**	Major Software Config C Version number */
#define BQ79600_FUNC_C_MAJOR_VERSION             (0x01u)

/**	Minor Software Config C Version number */
#define BQ79600_FUNC_C_MINOR_VERSION             (0x00u)

/** Software Patch Config C Version number */
#define BQ79600_FUNC_C_PATCH_VERSION             (0x01u)

#if ((BQ79600_FUNC_C_MAJOR_VERSION != BQ79600_CFG_MAJOR_VERSION) || \
     (BQ79600_FUNC_C_MINOR_VERSION != BQ79600_CFG_MINOR_VERSION) || \
	 (BQ79600_FUNC_C_PATCH_VERSION != BQ79600_CFG_PATCH_VERSION))
#error "tibms: Config version numbers of bq79600.c and bq79600_cfg.h are inconsistent!"
#endif

#if ((BQ79600_SW_MAJOR_VERSION != BQ79600_FUNC_C_MAJOR_VERSION) || \
     (BQ79600_SW_MINOR_VERSION != BQ79600_FUNC_C_MINOR_VERSION) || \
	 (BQ79600_SW_PATCH_VERSION != BQ79600_FUNC_C_PATCH_VERSION))
#error "tibms: Config version numbers of bq79600.c and bq79600.h are inconsistent!"
#endif

#if ((BQ79600_REGS_MAJOR_VERSION != BQ79600_FUNC_C_MAJOR_VERSION) || \
     (BQ79600_REGS_MINOR_VERSION != BQ79600_FUNC_C_MINOR_VERSION) || \
	 (BQ79600_REGS_PATCH_VERSION != BQ79600_FUNC_C_PATCH_VERSION))
#error "tibms: Config version numbers of bq79600.c and bq79600_regs.h are inconsistent!"
#endif

/**********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

#define BQ79600_GUARD_START                                  	(0xCCA1AB1EU)
#define BQ79600_GUARD_END                               		(0xCAFECEEDU)
#define BQ79600_INIT                                            (0xBAU)

#define BQ79600_INTEGRITY_CHECK(pBqComMgr)                      ((NULL != pBqComMgr) && \
                                                                 (BQ79600_GUARD_START == pBqComMgr->qGuardStart) && \
                                                                 (BQ79600_GUARD_END == pBqComMgr->qGuardEnd) && \
                                                                 (BQ79600_INIT == pBqComMgr->uInit))

#define BQ79600_DEV_TX_FIFO_LEN                                 (0x80u)

#define BQ79600_DELAY_CMD_LEN                                   (0x02u)
#define BQ79600_WAKEUP_PING_LEN                                 (0x02u)

#define BQ79600_WRITE_DATA_LEN_MIN                              (0x01u)
#define BQ79600_WRITE_DATA_LEN_MAX                              (0x08u)
#define BQ79600_READ_DATA_LEN_MAX                               (0x80u)
#define BQ79600_READ_DATA_LEN_MIN                               (0x01u)
#define BQ79600_DEV_ADR_RANGE_MAX                               (0x3Fu)

#define BQ79600_READ_LEN_OFFSET                                 (0x01u)

/* SPI baud rate is 2Mbps, CS to CS time needs to be considered */
#define BQ79600_SLP2ACK_DATA_LEN                                (0x34u)
#define BQ79600_WAKEUP_DATA_LEN                                 (0x200u)
#define BQ79600_COMM_CLEAR_DATA_LEN                             (0x01u)
#define BQ79600_REQUEST_BYTE_NUM_MASK                           (0x7Fu)

#define Bq79600_FrameInitpack(type, group, size)     \
        ((uint8)0x80u | (((type) & 7u) << 4) | (((group) & 1u) << 3) | ((((uint8)(size)) - 1u) & 7u))

#define Bq79600_GetCmdType(pData)                               (((pData)[0u] >> 4u) & 0x07u)
#define Bq79600_GetCmdDevAdr(pData)                             ((pData)[1u] & 0x3Fu)
#define Bq79600_GetCmd16bit(pData, Pos)                         (((uint16)((pData)[(Pos)]) << 8u) | ((uint16)((pData)[(Pos) + 1u])))
#define Bq79600_GetReqCmdData(pData, pos)                       (((pData)[(pos)] & 0x7Fu))
#define Bq79600_GetRespCmdData(pData)                           (((pData)[0u] & 0x7Fu) + 1u)
#define Bq79600_GetRespCmdCrc(pData, Pos)                       (((uint16)(pData)[(Pos) + 1u] << 8u) | ((uint16)(pData)[(Pos)]))
#define Bq79600_GetRespCmdDataByte(pData, offset)               ((pData)[4u + (offset)])

/**********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

typedef enum Bq79600_FrameCmdType_Tag
{
    BQ79600_REQ_READ,
    BQ79600_REQ_WRITE,
    BQ79600_REQ_CHECK,
    BQ79600_REQ_COMCLEAR = REQTYPE_CUSTOM_COMCLEAR,
    BQ79600_REQ_WAKEUP = REQTYPE_CUSTOM_WAKEUP,
    BQ79600_REQ_DELAY = REQTYPE_CUSTOM_DELAY,
    BQ79600_REQ_CUSTOM = REQTYPE_CUSTOM_CMD_W
}
Bq79600_FrameCmdType;

typedef enum Bq79600_StatusType_Tag
{
    BQ79600_STATE_IDLE = 0x20,
    BQ79600_STATE_TRANSIENT,
    BQ79600_STATE_WAIT_RESPRDY,
    BQ79600_STATE_WAIT_RESPONSE,
    BQ79600_STATE_WAIT_TX_COMPLETE,
    BQ79600_STATE_WAIT_UART_TX_COMPLETE,
    BQ79600_STATE_WAIT_DELAY,
    BQ79600_STATE_WAIT_COMCLEAR,
    BQ79600_STATE_WAKE_PING,
    BQ79600_STATE_WAKE_PING_DELAY,

    BQ79600_STATE_ERROR
}
Bq79600_StatusType;

typedef struct Bq79600_ContextType_Tag
{
    uint32 qGuardStart;

    const Basic_ConfigType *pBswCfg;
    const ServiceCfgType *pCommCfg;
    Emem_StasticsType *pEmemStats;
    Bq79600_ReqDataType *pCurrReq;
    const Comif_ManagerType *pComifCtx;
    const Bq7971x_ManagerType *pBmcMgr;

    ctPeQueueType zCmdQueFree;
    ctPeQueueType zCmdQueExec[COMIF_CMD_PRIO_MAX];
    ctPeQueueType zReadQueExec;

    uint8 uInit;
    uint8 uDrvState;
    uint8 uCmdIface;
    uint8 uNAFEs;
	uint8 uComTyp;
	uint8 uWakeCnt;

    uint8 uCcProg;
    uint8 uRespRdyProg;
    uint8 uTxInProg;
    uint8 uRxInProg;

    uint8 uDelayProg;
    uint8 uComClearReqExt;
    uint8 uRsrvd[2];

    uint32 qGuardEnd;
}
Bq79600_ManagerType;

/**********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

#define BQ79600_START_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

static Bq79600_ManagerType zBq79600Ctx[BQ79600_IFACES];

#define BQ79600_STOP_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

/***********************************************************************************************************************
 *  Local CONST Object Definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  Local Function Prototypes
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  Local Inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  Local Functions Definition
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  FUNC(uint8, bq79600_CODE) ResponseFrameCrcCheck(const uint8 *pData, uint8 uSFrameLen, uint8 uFrameNum)
 *********************************************************************************************************************/
/*! \brief          This function is used to check the CRC of each frame response data.
 *
 *  \param[in]      pData: Pointer to data
 *  \param[in]      uSFrameLen: single frame length
 *  \param[in]      uFrameNum: Number of Frames
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

#define BQ79600_START_SEC_CODE
#include "Cdd_MemMap.h"
uint8 readInProgress;

STATIC FUNC(uint8, bq79600_CODE) ResponseFrameCrcCheck(const uint8 *pData, uint8 uSFrameLen, uint8 uFrameNum)
{
    uint8 uRet = (uint8)RESP_VALID;
    uint8 uIndex;
    uint16 wCrc;

    for (uIndex = 0u; uIndex < uFrameNum; uIndex ++)
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
 *  FUNC(uint8, bq79600_CODE) Bq79600_TransferComClear(Bq79600_ManagerType *pBqComMgr)
 *********************************************************************************************************************/
/*! \brief          Invoking transfer COM CLEAR REQUEST
 *
 *  \param[in]      pBqComMgr: Communication device Manager
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

STATIC FUNC(uint8, bq79600_CODE) Bq79600_TransferComClear(Bq79600_ManagerType *pBqComMgr)
{
    uint8 uRet = (uint8)COMIF_NOT_OK;

    pBqComMgr->pBswCfg->dio_req(DIO_EDGE_DISABLE);
    pBqComMgr->pBswCfg->transfer_ctrl(CANCEL_TX_REQ, NULL, NULL, NULL);
    pBqComMgr->pBswCfg->transfer_ctrl(CANCEL_RX_REQ, NULL, NULL, NULL);

    if(FALSE == pBqComMgr->uComClearReqExt)
    {
        EMEM_UPDATE_STATICS(pBqComMgr->pEmemStats, qNComRec);
    }

    pBqComMgr->uDrvState = (uint8)BQ79600_STATE_WAIT_COMCLEAR;
    uRet = (uint8)pBqComMgr->pBswCfg->transfer_req(NULL, 1u, NULL, 0u);
    if(E_OK == uRet)
    {
        pBqComMgr->uCcProg = TRUE;
        uRet =(uint8)COMIF_OK;
    }
    else
    {
        uRet = (uint8)COMIF_CMD_SPI_TXERROR;
    }

    pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, BQ79600_WAIT_COMCLEAR_RESP);

    return uRet;
}
/**********************************************************************************************************************
 *  FUNC(uint8, bq79600_CODE) ScheduleFrameRequest(Bq79600_ManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
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
STATIC FUNC(uint8, bq79600_CODE) ScheduleFrameRequest(Bq79600_ManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
{
    Bq79600_ReqDataType *pReqFrame;
    uint8 uRet = (uint8)COMIF_CMD_CFQUE_FAILED;
    uint16 wData;
    uint16 wIndex = 0u;
    uint8 uReqCmdInit = FALSE;
    uint8 uScheduleFrame = FALSE;
    uint8 uSingleReq = FALSE;

    pReqFrame = (Bq79600_ReqDataType *) ctPeQueueRemoveData(&pBqComMgr->zCmdQueFree);
    if(NULL != pReqFrame)
    {
        switch(pReqCmd->uReqType)
        {
            case (uint8)REQTYPE_STACK_W:
            case (uint8)REQTYPE_BROADCAST_W:
            case (uint8)REQTYPE_BROADCAST_W_REV:
            {
                pReqFrame->uCmd = (uint8)BQ79600_REQ_WRITE;
                pReqFrame->pServiceReq = NULL;
                pReqFrame->wReqDataLen = (BQ79600_REQ_FRAME_FIXED_LEN + (pReqCmd->wReqCmdSiz - 1u));
                uReqCmdInit = TRUE;
                break;
            }
            case (uint8)REQTYPE_STACK_R:
            case (uint8)REQTYPE_BROADCAST_R:
            {
                pReqFrame->uCmd = (uint8)BQ79600_REQ_READ;

                pReqFrame->wReqDataLen = BQ79600_REQ_FRAME_FIXED_LEN;
                pReqFrame->wRspDataLen = (uint16) ((pReqCmd->pData[0u] + BQ79600_RSP_FRAME_FIXED_LEN) * pReqCmd->uDevId);

                pReqFrame->wCurrRspDataLen = pReqFrame->wRspDataLen;
                pReqFrame->uBlockSiz = (pReqCmd->pData[0u] & BQ79600_REQUEST_BYTE_NUM_MASK) + BQ79600_RSP_FRAME_FIXED_LEN;
                pReqFrame->pServiceReq = pReqCmd->pServiceCfg;
                pReqFrame->pRxData = (uint8 *) pReqCmd->pServiceCfg->pRawData;
                uReqCmdInit = TRUE;

                if((NULL == pReqFrame->pServiceReq) || (pReqFrame->pServiceReq->wDataLen < pReqFrame->wRspDataLen))
                {
                    (void)ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pReqFrame);
                    uRet = (uint8)COMIF_CMD_INVALID_CMDDATA;
                    uReqCmdInit = FALSE;
                }
                break;
            }
            case (uint8)REQTYPE_SINGLE_W:
            {
                pReqFrame->uCmd = (uint8)BQ79600_REQ_WRITE;
                pReqFrame->pServiceReq = NULL;
                pReqFrame->wReqDataLen = (BQ79600_REQ_FRAME_FIXED_LEN + pReqCmd->wReqCmdSiz);
                uReqCmdInit = TRUE;
                uSingleReq = TRUE;
                break;
            }
            case (uint8)REQTYPE_SINGLE_R:
            {
                pReqFrame->uCmd = (uint8)BQ79600_REQ_READ;
                uSingleReq = TRUE;

                pReqFrame->wReqDataLen = (BQ79600_REQ_FRAME_FIXED_LEN + 1u);
                pReqFrame->wRspDataLen = (uint16) (pReqCmd->pData[0u] + BQ79600_RSP_FRAME_FIXED_LEN);
                pReqFrame->wCurrRspDataLen = pReqFrame->wRspDataLen;
                pReqFrame->uBlockSiz = (pReqCmd->pData[0u] & BQ79600_REQUEST_BYTE_NUM_MASK) + BQ79600_RSP_FRAME_FIXED_LEN;
                pReqFrame->pServiceReq = pReqCmd->pServiceCfg;

                if((NULL != pReqFrame->pServiceReq) && (pReqFrame->pServiceReq->wDataLen >= pReqFrame->wRspDataLen))
                {
                    // uDevId will be the Index
                    //pReqFrame->pRxData = &pReqCmd->pServiceCfg->pRawData[pReqCmd->pServiceCfg->uBlockLen * pReqCmd->uDevId];
                    pReqFrame->pRxData = (uint8 *) pReqCmd->pServiceCfg->pRawData;
                    uReqCmdInit = TRUE;
                }
                else
                {
                    (void)ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pReqFrame);
                    uRet = (uint8)COMIF_CMD_INVALID_CMDDATA;
                }
                break;
            }
            case (uint8)REQTYPE_CUSTOM_ERROR:
            {
                pReqFrame->uCmd = (uint8)BQ79600_REQ_WRITE;
                pReqFrame->pServiceReq = NULL;
                pReqFrame->wReqDataLen = (BQ79600_REQ_FRAME_FIXED_LEN + pReqCmd->wReqCmdSiz+1U);
                pReqFrame->zTxCmd[wIndex++] = 0xFFU;

                uReqCmdInit = TRUE;
                uSingleReq = TRUE;
                break;
            }
            case (uint8)REQTYPE_CUSTOM_DELAY:
            case (uint8)REQTYPE_CUSTOM_WAKEUP:
            case (uint8)REQTYPE_CUSTOM_COMCLEAR:
            {
                pReqFrame->uCmd = pReqCmd->uReqType;
                for (wData = 0; wData < pReqCmd->wReqCmdSiz; wData++)
                {
                    pReqFrame->zTxCmd[wIndex++] = pReqCmd->pData[wData];
                }

                uScheduleFrame = TRUE;
                break;
            }
            default:
            {
                (void)ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pReqFrame);
                uRet = (uint8)COMIF_CMD_INVALID_REQCMD;
                break;
            }
        }
    }
    else
    {
        EMEM_UPDATE_STATICS(pBqComMgr->pEmemStats, qNReqQueFail);
        uRet = (uint8)COMIF_CMD_CFQGET_EMPTY;
    }

    if(0U != uReqCmdInit)
    {
        pReqFrame->zTxCmd[wIndex++] = Bq79600_FrameInitpack(pReqCmd->uReqType, pReqCmd->uGroupExclude,
                                                             pReqCmd->wReqCmdSiz);
        if(TRUE == uSingleReq)
        {
            pReqFrame->zTxCmd[wIndex++] = pReqCmd->uDevId;
        }

        pReqFrame->zTxCmd[wIndex++] = (uint8)(pReqCmd->wRegAdr >> TIBMS_8BIT_OFFSET) & TIBMS_8BIT_MASK;
        pReqFrame->zTxCmd[wIndex++] = (uint8)(pReqCmd->wRegAdr) & TIBMS_8BIT_MASK;

        for (wData = 0; wData < pReqCmd->wReqCmdSiz; wData++)
        {
            pReqFrame->zTxCmd[wIndex++] = pReqCmd->pData[wData];
        }

        wData = CalculateCRC16(pReqFrame->zTxCmd, wIndex);
        pReqFrame->zTxCmd[wIndex++] = (uint8)(wData & TIBMS_8BIT_MASK);
        pReqFrame->zTxCmd[wIndex++] = (uint8)((wData >> TIBMS_8BIT_OFFSET) & TIBMS_8BIT_MASK);

        uScheduleFrame = TRUE;
    }

    if(0U != uScheduleFrame)
    {
        pReqFrame->uCmdInfo = pReqCmd->uCmdInfo;
        uRet = ctPeQueueAddData(&pBqComMgr->zCmdQueExec[COMIF_GET_PRIO(pReqFrame->uCmdInfo)], pReqFrame);
        if((uint8)QUEUE_E_OK == uRet)
        {
            if((uint8)BQ79600_STATE_IDLE == pBqComMgr->uDrvState)
            {
                pBqComMgr->uDrvState = (uint8)BQ79600_STATE_TRANSIENT;
                pBqComMgr->pBswCfg->transfer_ctrl(NOTIFY_TX_REQ, NULL, NULL, NULL);
            }
            uRet = (uint8)COMIF_OK;
        }
    }

    pBqComMgr->zCmdQueFree.nMaxUsage = max(pBqComMgr->zCmdQueFree.nMaxUsage,
                                           (pBqComMgr->zCmdQueFree.wSizeQueue-pBqComMgr->zCmdQueFree.nElements));

    EMEM_UPDATE_STATICS_DATA(pBqComMgr->pEmemStats, qQueMaxUsage, pBqComMgr->zCmdQueFree.nMaxUsage);

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_SCHEDULE, uRet, pReqCmd->uReqType, pBqComMgr->uDrvState);
}


/**********************************************************************************************************************
 *  FUNC(uint8, bq79600_CODE) Bq79600_SendComClear(Bq79600_ManagerType *pBqComMgr)
 *********************************************************************************************************************/
/*! \brief          Adding Delay request between the commands
 *
 *  \param[in]      pBqComMgr: Communication device Manager
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

STATIC FUNC(uint8, bq79600_CODE) Bq79600_SendComClear(Bq79600_ManagerType *pBqComMgr)
{
    Comif_ReqCmdType zReqcmd;
    uint8 uData = BQ79600_COMM_CLEAR_DATA_LEN;

    zReqcmd.uReqType = (uint8)REQTYPE_CUSTOM_COMCLEAR;
    zReqcmd.uGroupExclude = 0u;
    zReqcmd.wReqCmdSiz = BQ79600_COMM_CLEAR_DATA_LEN;
    zReqcmd.uDevId = 0u;
    zReqcmd.wRegAdr = 0u;
    zReqcmd.pData = &uData;
    zReqcmd.pServiceCfg = NULL;
    zReqcmd.uCmdInfo = 0u;

    return ScheduleFrameRequest(pBqComMgr, &zReqcmd);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq79600_CODE) Bq79600_SetDelay(Bq79600_ManagerType *pBqComMgr, uint32 qTimeMs)
 *********************************************************************************************************************/
/*! \brief          Adding Delay request between the commands
 *
 *  \param[in]      pBqComMgr: Communication device Manager
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

STATIC FUNC(uint8, bq79600_CODE) Bq79600_SetDelay(Bq79600_ManagerType *pBqComMgr, uint32 qTimeMs)
{
    Comif_ReqCmdType zReqcmd;
    uint8 uRet;
    uint8 uData[BQ79600_DELAY_CMD_LEN];

    uData[0u] = (uint8)(qTimeMs >> 8u) & TIBMS_8BIT_MASK;
    uData[1u] = (uint8)(qTimeMs >> 0u) & TIBMS_8BIT_MASK;

    zReqcmd.uReqType = (uint8)REQTYPE_CUSTOM_DELAY;
    zReqcmd.uGroupExclude = 0u;
    zReqcmd.wReqCmdSiz = BQ79600_DELAY_CMD_LEN;
    zReqcmd.uDevId = 0u;
    zReqcmd.wRegAdr = 0u;
    zReqcmd.pData = uData;
    zReqcmd.pServiceCfg = NULL;
    zReqcmd.uCmdInfo = COMIF_GET_INFO(qTimeMs);

    uRet = ScheduleFrameRequest(pBqComMgr, &zReqcmd);

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq79600_CODE) Bq79600_WakePing(Bq79600_ManagerType *pBqComMgr, uint32 qTimeMs)
 *********************************************************************************************************************/
/*! \brief          Wakeup Ping during the startup
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *  \param[in]      wTimeMs: Delay time in Millisecond
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

STATIC FUNC(uint8, bq79600_CODE) Bq79600_WakePing(Bq79600_ManagerType *pBqComMgr, uint32 qTimeMs)
{
    uint8 uRet;
    Comif_ReqCmdType zReqcmd;
    uint8 uData[BQ79600_DELAY_CMD_LEN];

    uData[0u] = ((BQ79600_WAKEUP_DATA_LEN >> 8u) & TIBMS_8BIT_MASK);
    uData[1u] = ((BQ79600_WAKEUP_DATA_LEN >> 0u) & TIBMS_8BIT_MASK);

    zReqcmd.uReqType = (uint8)REQTYPE_CUSTOM_WAKEUP;
    zReqcmd.uGroupExclude = 0u;
    zReqcmd.wReqCmdSiz = BQ79600_WAKEUP_PING_LEN;
    zReqcmd.uDevId = 0u;
    zReqcmd.wRegAdr = 0u;
    zReqcmd.pData = uData;
    zReqcmd.pServiceCfg = NULL;
    zReqcmd.uCmdInfo = 0u;

    uRet = ScheduleFrameRequest(pBqComMgr, &zReqcmd);
    if((uint8)COMIF_OK == uRet)
    {
        uRet = Bq79600_SetDelay(pBqComMgr, qTimeMs);
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq79600_CODE) Bq79600_DelayNotify(Bq79600_ManagerType *pBqComMgr)
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

STATIC FUNC(uint8, bq79600_CODE) Bq79600_DelayNotify(Bq79600_ManagerType *pBqComMgr)
{
    uint8 uRet = (uint8)COMIF_OK;

    switch (pBqComMgr->uDrvState)
    {
        case (uint8)BQ79600_STATE_WAIT_RESPONSE:
        {

#if BMI_UART_ENABLE

            if(NULL != pBqComMgr->pCurrReq)
            {
                if(0u == pBqComMgr->pCurrReq->wCurrRspDataLen)
                {
                    //pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, 0u);
                    if((NULL != pBqComMgr->pCurrReq->pRxData) && (pBqComMgr->pCurrReq->uBlockSiz > 0U))
                    {
                        uRet = (uint8)COMIF_CMD_CRC_FAILED;
						uNFrames = (uint8)(pBqComMgr->pCurrReq->wRspDataLen / pBqComMgr->pCurrReq->uBlockSiz);
						pBqComMgr->pCurrReq->uStatus = ResponseFrameCrcCheck(pBqComMgr->pCurrReq->pRxData,
                                                                             pBqComMgr->pCurrReq->uBlockSiz, uNFrames);
                        if((uint8)RESP_VALID == pBqComMgr->pCurrReq->uStatus)
                        {
                            if((uint8)QUEUE_E_OK == ctPeQueueAddData(&pBqComMgr->zReadQueExec, pBqComMgr->pCurrReq))
                            {
                                uRet = (uint8)COMIF_OK;
                                pBqComMgr->pCurrReq = NULL;
                            }
                            pBqComMgr->pBswCfg->transfer_ctrl(NOTIFY_RX_REQ, NULL, NULL, NULL);
			    readInProgress = 0U;
                        }
                        else
                        {
                            EMEM_UPDATE_STATICS(pBqComMgr->pEmemStats, qNCrcFails);
                            pBqComMgr->uDrvState = (uint8) BQ79600_STATE_WAIT_UART_TX_COMPLETE;
                                    pBqComMgr->uRxInProg = FALSE;
                                    readInProgress = 0U;
                            //pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, 1);
                            (void)Bq79600_HandleTransfer(pBqComMgr);
                        }
                    }
                    //pBqComMgr->uRxInProg = FALSE;

                    //pBqComMgr->uDrvState = (uint8)BQ79600_STATE_TRANSIENT;
                    /*pBqComMgr->uDrvState = (uint8) BQ79600_STATE_WAIT_UART_TX_COMPLETE;
                    pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, 1);*/
                    //(void)Bq79600_HandleTransfer(pBqComMgr);
                }

            }

            break;
#endif

        }

        //case (uint8)BQ79600_STATE_WAIT_RESPRDY:

        case (uint8)BQ79600_STATE_WAIT_COMCLEAR:
        {
#if BMI_UART_ENABLE == STD_ON
	    if ( readInProgress == FALSE )
	    {
               Bq79600_HandleTransfer(pBqComMgr);
	    }
            else
	    {
               pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, 1);  
	    }
#else
            uRet = Bq79600_TransferComClear(pBqComMgr);
#endif
            break;
        }
        case (uint8)BQ79600_STATE_WAIT_DELAY:
        {
            if(NULL != pBqComMgr->pCurrReq)
            {
                (void)ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pBqComMgr->pCurrReq);
                pBqComMgr->pCurrReq = NULL;
            }
            pBqComMgr->uDelayProg = FALSE;
            pBqComMgr->uDrvState = (uint8)BQ79600_STATE_TRANSIENT;
            (void)Bq79600_HandleTransfer(pBqComMgr);
            break;
        }
        case (uint8)BQ79600_STATE_WAIT_UART_TX_COMPLETE:
        {
            if(NULL != pBqComMgr->pCurrReq)
                        {
                            (void)ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pBqComMgr->pCurrReq);
                            pBqComMgr->pCurrReq = NULL;
                        }

            Bq79600_HandleTransfer(pBqComMgr);
            break;
        }
		case (uint8)BQ79600_STATE_WAKE_PING:
		{
			if(pBqComMgr->uWakeCnt == WAKEUP_PULSE_CNT)
			{
				if(pBqComMgr->pCurrReq)
				{
					ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pBqComMgr->pCurrReq);
					pBqComMgr->pCurrReq = NULL;
				}
				
			}
			Bq79600_HandleTransfer(pBqComMgr);
			break;
		}
        case (uint8)BQ79600_STATE_WAKE_PING_DELAY:
        {
            uRet = pBqComMgr->pBswCfg->init();
            pBqComMgr->pBswCfg->timer_req(TIMER_DELAY, BQ79600_WAIT_BTW_PING);
            pBqComMgr->uDrvState = BQ79600_STATE_WAKE_PING;
            break;

        }
        default:
        {
            uRet = (uint8)COMIF_OK;
            pBqComMgr->pBswCfg->timer_req(TIMER_DELAY, 1U);
            pBqComMgr->uDrvState = (uint8)BQ79600_STATE_WAIT_UART_TX_COMPLETE;
            break;
        }
    }

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_DELAY_NOTIFY, uRet, pBqComMgr->uDrvState, 0);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq79600_CODE) Bq79600_SeqEndNotify(Bq79600_ManagerType *pBqComMgr)
 *********************************************************************************************************************/
/*! \brief          SPI Sequnce Complete Notification
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

STATIC FUNC(uint8, bq79600_CODE) Bq79600_SeqEndNotify(Bq79600_ManagerType *pBqComMgr)
{
    uint8 uRet = (uint8)COMIF_OK;
    uint8 uNFrames;

    switch (pBqComMgr->uDrvState)
    {
        case (uint8)BQ79600_STATE_WAIT_UART_TX_COMPLETE:
        case (uint8)BQ79600_STATE_WAIT_TX_COMPLETE:
        {
            //pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, 0u);
            if ( readInProgress == FALSE)
            {
            pBqComMgr->uTxInProg = FALSE;
            if(NULL != pBqComMgr->pCurrReq)
            {
                (void)ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pBqComMgr->pCurrReq);
                pBqComMgr->pCurrReq = NULL;
            }

            pBqComMgr->uDrvState = (uint8)BQ79600_STATE_TRANSIENT;
            (void)Bq79600_HandleTransfer(pBqComMgr);
            }
            break;
        }

        case (uint8)BQ79600_STATE_WAIT_RESPONSE:
        {
            if(NULL != pBqComMgr->pCurrReq)
            {
                if(0u == pBqComMgr->pCurrReq->wCurrRspDataLen)
                {
                    pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, 0u);
                    if((NULL != pBqComMgr->pCurrReq->pRxData) && (pBqComMgr->pCurrReq->uBlockSiz > 0U))
                    {
                        uRet = (uint8)COMIF_CMD_CRC_FAILED;
						uNFrames = (uint8)(pBqComMgr->pCurrReq->wRspDataLen / pBqComMgr->pCurrReq->uBlockSiz);
						pBqComMgr->pCurrReq->uStatus = ResponseFrameCrcCheck(pBqComMgr->pCurrReq->pRxData,
                                                                             pBqComMgr->pCurrReq->uBlockSiz, uNFrames);
                        if((uint8)RESP_VALID == pBqComMgr->pCurrReq->uStatus)
                        {
                            if((uint8)QUEUE_E_OK == ctPeQueueAddData(&pBqComMgr->zReadQueExec, pBqComMgr->pCurrReq))
                            {
                                uRet = (uint8)COMIF_OK;
                                pBqComMgr->pCurrReq = NULL;
                            }
                            pBqComMgr->pBswCfg->transfer_ctrl(NOTIFY_RX_REQ, NULL, NULL, NULL);
			    readInProgress = 0U;
                        }
                        else
                        {
                            EMEM_UPDATE_STATICS(pBqComMgr->pEmemStats, qNCrcFails);
                            pBqComMgr->uDrvState = (uint8) BQ79600_STATE_WAIT_UART_TX_COMPLETE;
                                    pBqComMgr->uRxInProg = FALSE;
                            pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, 1);
                        }
                    }
                    //pBqComMgr->uRxInProg = FALSE;

                    //pBqComMgr->uDrvState = (uint8)BQ79600_STATE_TRANSIENT;
                    /*pBqComMgr->uDrvState = (uint8) BQ79600_STATE_WAIT_UART_TX_COMPLETE;
                    pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, 1);*/
                    //(void)Bq79600_HandleTransfer(pBqComMgr);
                }

            }
            break;

        }

        case (uint8)BQ79600_STATE_WAIT_COMCLEAR:
        {
            pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, 0u);
            pBqComMgr->uCcProg = FALSE;
            if(0U != pBqComMgr->uComClearReqExt)
            {
                pBqComMgr->uComClearReqExt = FALSE;
                if(NULL != pBqComMgr->pCurrReq)
                {
                    (void)ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pBqComMgr->pCurrReq);
                    pBqComMgr->pCurrReq = NULL;
                }
            }
            pBqComMgr->uDrvState = (uint8)BQ79600_STATE_TRANSIENT;
            (void)Bq79600_HandleTransfer(pBqComMgr);
            break;
        }
        case (uint8)BQ79600_STATE_WAIT_DELAY:
                {
            pBqComMgr->uTxInProg = FALSE;
                        if(NULL != pBqComMgr->pCurrReq)
                        {
                            (void)ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pBqComMgr->pCurrReq);
                            pBqComMgr->pCurrReq = NULL;
                        }

                        pBqComMgr->uDrvState = (uint8)BQ79600_STATE_TRANSIENT;
                        (void)Bq79600_HandleTransfer(pBqComMgr);
                        break;
                }
        default:
        {
            uRet = (uint8)COMIF_CMD_INVALID_DRVSTATE;
            break;
        }
    }

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_SEQ_NOTIFY, uRet, pBqComMgr->uDrvState, 0u);
}


/**********************************************************************************************************************
 *  FUNC(uint8, bq79600_CODE) Bq79600_RespRdyNotify(Bq79600_ManagerType *pBqComMgr)
 *********************************************************************************************************************/
/*! \brief          SPI RDY Signal Notification
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

STATIC FUNC(uint8, bq79600_CODE) Bq79600_RespRdyNotify(Bq79600_ManagerType *pBqComMgr)
{
    uint8 *pCurrRxData;
    uint8 uRet;
    uint16 wLength;
    uint16 wData;

    switch(pBqComMgr->uDrvState)
    {
        case (uint8)BQ79600_STATE_WAIT_RESPRDY:
        {
            pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, 0u);
            pBqComMgr->uRespRdyProg = FALSE;
            pBqComMgr->uDrvState = (uint8)BQ79600_STATE_TRANSIENT;
            (void)Bq79600_HandleTransfer(pBqComMgr);
            uRet = (uint8)COMIF_OK;
            break;
        }
        case (uint8)BQ79600_STATE_WAIT_RESPONSE:
        {
            uRet = (uint8)COMIF_CMD_INVALID_NOTIFY;
            if(NULL != pBqComMgr->pCurrReq)
            {
                wLength = BQ79600_DEV_TX_FIFO_LEN;
                if(pBqComMgr->pCurrReq->wCurrRspDataLen <= BQ79600_DEV_TX_FIFO_LEN)
                {
                    wLength = pBqComMgr->pCurrReq->wCurrRspDataLen;
                }

                if(wLength > 0U)
                {
                    if(NULL != pBqComMgr->pCurrReq->pRxData)
                    {
                        pCurrRxData = (uint8 *) (pBqComMgr->pCurrReq->pRxData +
                                                (pBqComMgr->pCurrReq->wRspDataLen - pBqComMgr->pCurrReq->wCurrRspDataLen));
                        if(NULL != pCurrRxData)
                        {
                            uRet = (uint8)pBqComMgr->pBswCfg->transfer_req(NULL, 0u, pCurrRxData, wLength);
                            if(E_OK == uRet)
                            {
                                pBqComMgr->pCurrReq->wCurrRspDataLen -= wLength;
                                wData = pBqComMgr->pCurrReq->wCurrRspDataLen;
                                uRet = (uint8)COMIF_OK;
                            }
                            else
                            {
                                uRet = (uint8)COMIF_CMD_SPI_RXERROR;
                            }
                        }
                    }
                    else
                    {
                        uRet = (uint8)COMIF_CMD_INVALID_RXDATABUF;
                    }
                }

                if(0U == pBqComMgr->pCurrReq->wCurrRspDataLen)
                {
                    pBqComMgr->pBswCfg->dio_req(DIO_EDGE_DISABLE);
                }
            }
            break;
        }
        default:
        {
            uRet = (uint8)COMIF_CMD_INVALID_DRVSTATE;
            break;
        }
    }

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_RDY_NOTIFY, uRet, wData, wLength);
}

/**********************************************************************************************************************
 * STATIC FUNC(ctPeQueueType *, bq79600_CODE) GetNextFrameQueue(Bq79600_ManagerType *pBqComMgr)
 *********************************************************************************************************************/
/*! \brief          Get Next Frame from QUEUE
 *
 *  \param[in]      pBqComMgr: Communication device Manager
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

STATIC FUNC(ctPeQueueType *, bq79600_CODE) GetNextFrameQueue(Bq79600_ManagerType *pBqComMgr)
{
    ctPeQueueType *pNextFrameQ = NULL;
    sint8 xPrioIdx = ((uint8)COMIF_CMD_PRIO_MAX-1U);

    do
    {
        pNextFrameQ = &pBqComMgr->zCmdQueExec[xPrioIdx];
        if(NULL != pNextFrameQ)
        {
            if(!ctPeQueueIsEmpty(pNextFrameQ))
            {
                break;
            }
        }
        xPrioIdx--;
    }
    while(xPrioIdx >= 0);

    return pNextFrameQ;
}

/**********************************************************************************************************************
 *  External Functions Definition
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  FUNC(uint8, bq79600_CODE) Bq79600_Control(void *pComifCtx, uint8 uCmd, void *pData)
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

FUNC(uint8, bq79600_CODE) Bq79600_Control(void *pComifCtx, uint8 uCmd, void *pData)
{
    Bq79600_ManagerType *pBqComMgr = (Bq79600_ManagerType *)  pComifCtx;
    uint8 uRet =  (uint8)COMIF_CMD_INTEGRITY_FAILED;
    Comif_ReqCmdType *pReqCmd;
    VersionInfoType *pCmdVer;
    uint8 uLength;
    uint32 qTimeUs;

    if(BQ79600_INTEGRITY_CHECK(pBqComMgr))
    {
        uRet =  (uint8)COMIF_CMD_INVALID_REQDATA;
        switch(uCmd)
        {
            case (uint8)COMIF_CMD_SGLREAD:
            {
                pReqCmd = (Comif_ReqCmdType *) pData;
                if(NULL != pReqCmd)
                {
                    if((NULL != pReqCmd->pData) && (pReqCmd->uDevId <= BQ79600_DEV_ADR_RANGE_MAX) &&
                       (pReqCmd->pData[0] <= BQ79600_READ_DATA_LEN_MAX) &&
                       (pReqCmd->pData[0] >= BQ79600_READ_DATA_LEN_MIN))
                    {
                        uLength = (uint8) ((uint8) pReqCmd->pData[0] - (uint8) BQ79600_READ_LEN_OFFSET);
                        pReqCmd->pData = &uLength;

                        uRet = ScheduleFrameRequest(pBqComMgr, pReqCmd);
                    }
                }
                break;
            }
            case (uint8)COMIF_CMD_SGLWRITE:
            {
                pReqCmd = (Comif_ReqCmdType *) pData;
                if(NULL != pReqCmd)
                {
                    if((NULL != pReqCmd->pData) && (pReqCmd->uDevId <= BQ79600_DEV_ADR_RANGE_MAX) &&
                       (pReqCmd->wReqCmdSiz <= BQ79600_WRITE_DATA_LEN_MAX) &&
                       (pReqCmd->wReqCmdSiz >= BQ79600_WRITE_DATA_LEN_MIN))
                    {
                        uRet = ScheduleFrameRequest(pBqComMgr, pReqCmd);
                    }
                }
                break;
            }
            case (uint8)COMIF_CMD_STKREAD:
            {
                pReqCmd = (Comif_ReqCmdType *) pData;
                if(NULL != pReqCmd)
                {
                    if((NULL != pReqCmd->pData) && (pReqCmd->uDevId <= BQ79600_DEV_ADR_RANGE_MAX) &&
                       (pReqCmd->wReqCmdSiz <= BQ79600_WRITE_DATA_LEN_MAX) &&
                       (pReqCmd->wReqCmdSiz >= BQ79600_WRITE_DATA_LEN_MIN))
                    {
                        uLength = ((uint8) *pReqCmd->pData) - BQ79600_READ_LEN_OFFSET;
                        pReqCmd->pData = &uLength;

                        uRet = ScheduleFrameRequest(pBqComMgr, pReqCmd);
                    }
                }
                break;
            }
            case (uint8)COMIF_CMD_STKWRITE:
            {
                pReqCmd = (Comif_ReqCmdType *) pData;
                if(NULL != pReqCmd)
                {
                    if((NULL != pReqCmd->pData) && (pReqCmd->wReqCmdSiz <= BQ79600_WRITE_DATA_LEN_MAX) &&
                       (pReqCmd->wReqCmdSiz >= BQ79600_WRITE_DATA_LEN_MIN))
                    {
                        uRet = ScheduleFrameRequest(pBqComMgr, pReqCmd);
                    }
                }
                break;
            }
            case (uint8)COMIF_CMD_ALLREAD:
            {
                pReqCmd = (Comif_ReqCmdType *) pData;
                if(NULL != pReqCmd)
                {
                    if((NULL != pReqCmd->pData) && (pReqCmd->uDevId <= BQ79600_DEV_ADR_RANGE_MAX) &&
                       (pReqCmd->wReqCmdSiz <= BQ79600_WRITE_DATA_LEN_MAX) &&
                       (pReqCmd->wReqCmdSiz >= BQ79600_WRITE_DATA_LEN_MIN))
                    {
                        uLength = ((uint8) *pReqCmd->pData) - BQ79600_READ_LEN_OFFSET;
                        pReqCmd->pData = &uLength;

                        uRet = ScheduleFrameRequest(pBqComMgr, pReqCmd);
                    }
                }
                break;
            }
            case (uint8)COMIF_CMD_ALLWRITE:
            {
                pReqCmd = (Comif_ReqCmdType *) pData;
                if(NULL != pReqCmd)
                {
                    if((NULL != pReqCmd->pData) && (pReqCmd->wReqCmdSiz <= BQ79600_WRITE_DATA_LEN_MAX) &&
                       (pReqCmd->wReqCmdSiz >= BQ79600_WRITE_DATA_LEN_MIN))
                    {
                        uRet = ScheduleFrameRequest(pBqComMgr, pReqCmd);
                    }
                }
                break;
            }
            case (uint8)COMIF_CMD_DELAY:
            {
                pReqCmd = (Comif_ReqCmdType*) pData;
                if(NULL != pReqCmd)
                {
                    uRet = ScheduleFrameRequest(pBqComMgr, pReqCmd);
                }
                break;
            }
            case (uint8)COMIF_CMD_WAKEUP:
            {
                if(NULL != pData)
                {
                    qTimeUs = (uint32) (*((uint32 *) pData));
                    uRet = Bq79600_WakePing(pBqComMgr, qTimeUs);
                }
                break;
            }
            case (uint8)COMIF_CMD_ERROR:
            {
                pReqCmd = (Comif_ReqCmdType *) pData;
                if(NULL != pReqCmd)
                {
                    if((NULL != pReqCmd->pData) && (pReqCmd->uDevId <= BQ79600_DEV_ADR_RANGE_MAX) &&
                       (pReqCmd->wReqCmdSiz <= BQ79600_WRITE_DATA_LEN_MAX) &&
                       (pReqCmd->wReqCmdSiz >= BQ79600_WRITE_DATA_LEN_MIN))
                    {
                        uRet = ScheduleFrameRequest(pBqComMgr, pReqCmd);
                    }
                }
                break;
            }
            case (uint8)COMIF_CMD_COMCLEAR:
            {
                uRet = Bq79600_SendComClear(pBqComMgr);
                break;
            }
            case (uint8)COMIF_CMD_GETVERSION:
            {
                pCmdVer = (VersionInfoType *) pData;
                if(NULL != pCmdVer)
                {
                    pCmdVer->zCmdVersion.vendorID = BQ79600_VENDOR_ID;
                    pCmdVer->zCmdVersion.moduleID = BQ79600_MODULE_ID;
                    pCmdVer->zCmdVersion.sw_major_version = BQ79600_SW_MAJOR_VERSION;
                    pCmdVer->zCmdVersion.sw_minor_version = BQ79600_SW_MINOR_VERSION;
                    pCmdVer->zCmdVersion.sw_patch_version = BQ79600_SW_PATCH_VERSION;
                    uRet =  (uint8)COMIF_OK;
                }
                break;
            }
            case (uint8)COMIF_CMD_UNKNOWN:
            default:
            {
                uRet =  (uint8)COMIF_CMD_INVALID_REQCMD;
                break;
            }
        }
    }

    return uRet;
}
/**********************************************************************************************************************
 *  FUNC(uint8, bq79600_CODE) Bq79600_UartHandleTransfer(void *pComifCtx)
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

FUNC(uint8, bq79600_CODE) Bq79600_UartHandleTransfer(Bq79600_ManagerType *pBqComMgr)
{
    uint8 uRet = COMIF_CMD_INTEGRITY_FAILED;
    ctPeQueueType *pNextFrameQ;
    Bq79600_ReqDataType *pReqFrame;
	uint8 *pCurrRxData;
    uint32 qData = 1;
    uint8 uCmd=0;

    if ( readInProgress == TRUE )
    {
        uRet = E_OK;
        pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, qData);
        return Comif_SetErrorDetails(COMIF_SVCID_CMD_TX_PROC, uRet, uCmd, qData);
    }

	if(BQ79600_INTEGRITY_CHECK(pBqComMgr))
    {
        pNextFrameQ = GetNextFrameQueue(pBqComMgr);
        if(pNextFrameQ)
        {
            pReqFrame = ctPeQueueGetData(pNextFrameQ);
            if(pReqFrame)
            {
                uCmd = pReqFrame->uCmd;
                if(BQ79600_REQ_CHECK > pReqFrame->uCmd)
                {
                    
                    if(BQ79600_REQ_READ == pReqFrame->uCmd)
                    {
                                                pBqComMgr->uDrvState = BQ79600_STATE_WAIT_RESPONSE;
                        pBqComMgr->uRxInProg = TRUE;
                        readInProgress = TRUE;
                        qData = (5);                         // TBD: Timings correction

						pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, qData);                                                   // TODO: add timer for Transfer timer for BQ79600
						uRet = pBqComMgr->pBswCfg->transfer_req(pReqFrame->zTxCmd, pReqFrame->wReqDataLen, NULL, 0u);
						if(E_OK == uRet)
						{
							EMEM_UPDATE_STATICS(pBqComMgr->pEmemStats, qNTxReqs);
							if(pReqFrame->wCurrRspDataLen > 0)
							{
								if(pReqFrame->pRxData)
								{
									pCurrRxData = (uint8 *) (pReqFrame->pRxData +
													(pReqFrame->wRspDataLen - pReqFrame->wCurrRspDataLen));
									if(pCurrRxData)
									{
										uRet = pBqComMgr->pBswCfg->transfer_req(NULL, 0u, pCurrRxData, pReqFrame->wCurrRspDataLen);
										pReqFrame->wCurrRspDataLen=0;
									}
								}
							}
							pBqComMgr->pCurrReq = ctPeQueueRemoveData(pNextFrameQ);
							uRet = COMIF_OK;
						}
						else
						{
							uRet = COMIF_CMD_UART_TXERROR;
						}

                    }

                    if(BQ79600_REQ_WRITE == pReqFrame->uCmd)
                    {

                                                  // TODO: add timer for Transfer timer for BQ79600
						//uRet = pBqComMgr->pBswCfg->transfer_req(pReqFrame->zTxCmd, pReqFrame->wReqDataLen, NULL, 0u);
						pBqComMgr->uDrvState = BQ79600_STATE_WAIT_UART_TX_COMPLETE;
                        pBqComMgr->uTxInProg = TRUE;
                                                  // TODO: add timer for Transfer timer for BQ79600
						uRet = pBqComMgr->pBswCfg->transfer_req(pReqFrame->zTxCmd, pReqFrame->wReqDataLen, NULL, 0u);
						//pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, 1);
						//Gpt_StopTimer(1);
						//Gpt_StartTimer(1, 1000);
                        if(E_OK == uRet)
						{
							EMEM_UPDATE_STATICS(pBqComMgr->pEmemStats, qNTxReqs);
							pBqComMgr->pCurrReq = ctPeQueueRemoveData(pNextFrameQ);
							ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pBqComMgr->pCurrReq);
							pBqComMgr->pCurrReq = NULL;
							pBqComMgr->uTxInProg = TRUE;
							uRet = COMIF_OK;
						}
						else
						{
							uRet = COMIF_CMD_UART_TXERROR;
						}

                                         /*       uRet = Bq79600_SetDelay(pBqComMgr, 1U);
                                                Bq79600_HandleTransfer(pBqComMgr); */
					}
                    
                }
                else
                {
                    if(BQ79600_REQ_DELAY == pReqFrame->uCmd)
                    {
                        qData = ((pReqFrame->zTxCmd[0u] << 8u) | (pReqFrame->zTxCmd[1u] << 0u));

                        if ((pBqComMgr->uTxInProg == TRUE) && (pBqComMgr->uDrvState == BQ79600_STATE_WAIT_UART_TX_COMPLETE))
                        {

                            pBqComMgr->uDelayProg = TRUE;
                        }

                        pBqComMgr->uDrvState = BQ79600_STATE_WAIT_DELAY;
                        pBqComMgr->uDelayProg = TRUE;
                        pBqComMgr->pBswCfg->timer_req(TIMER_DELAY, qData);
                        pBqComMgr->pCurrReq = ctPeQueueRemoveData(pNextFrameQ);
                        uRet = COMIF_OK;
                    }

                    if(BQ79600_REQ_WAKEUP == pReqFrame->uCmd)
                    {
                        qData = (((uint32) pReqFrame->zTxCmd[0u] << 8u) | ((uint32) pReqFrame->zTxCmd[1u] << 0u));

                        pBqComMgr->pBswCfg->timer_req(TIMER_DELAY, BQ79600_WAIT_BTW_PING);//wakeping delay

                        uRet = pBqComMgr->pBswCfg->deinit();
						pBqComMgr->uWakeCnt++;
                        if(E_OK == uRet)
                        {
                            pBqComMgr->uDrvState = BQ79600_STATE_WAKE_PING_DELAY;
                            pBqComMgr->uTxInProg = TRUE;
                            if(pBqComMgr->uWakeCnt == WAKEUP_PULSE_CNT)
                            {
                                pBqComMgr->pCurrReq = ctPeQueueRemoveData(pNextFrameQ);
                            }

                            uRet = COMIF_OK;
                        }
                        else
                        {
                            uRet = COMIF_CMD_UART_TXERROR;
                        }
                    }

                    if(BQ79600_REQ_COMCLEAR == pReqFrame->uCmd)
                    {
                        pBqComMgr->uComClearReqExt = TRUE;
                        pBqComMgr->pCurrReq = ctPeQueueRemoveData(pNextFrameQ);
                        uRet = Bq79600_TransferComClear(pBqComMgr);
                    }
                }
            }
            else
            {
                pBqComMgr->uDrvState = BQ79600_STATE_IDLE;
                uRet = COMIF_OK;
            }
        }
    }
    return Comif_SetErrorDetails(COMIF_SVCID_CMD_TX_PROC, uRet, uCmd, qData);
}
/**********************************************************************************************************************
 *  FUNC(uint8, bq79600_CODE) Bq79600_HandleTransfer(void *pComifCtx)
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

FUNC(uint8, bq79600_CODE) Bq79600_HandleTransfer(void *pComifCtx)
{
    Bq79600_ManagerType *pBqComMgr = (Bq79600_ManagerType *)  pComifCtx;
    uint8 uRet = (uint8)COMIF_CMD_INTEGRITY_FAILED;
    ctPeQueueType *pNextFrameQ;
    Bq79600_ReqDataType *pReqFrame;
    uint32 qData = 0;
    uint8 uCmd = 0;
	
	if(pBqComMgr->uComTyp == COMIF_UART)
	{
		uRet = Bq79600_UartHandleTransfer(pBqComMgr);
	}
	else
	{

    if(BQ79600_INTEGRITY_CHECK(pBqComMgr))
    {
        pNextFrameQ = GetNextFrameQueue(pBqComMgr);
        if(NULL != pNextFrameQ)
        {
            pReqFrame = (Bq79600_ReqDataType *)ctPeQueueGetData(pNextFrameQ);
            if(NULL != pReqFrame)
            {
                uCmd = pReqFrame->uCmd;
                if((uint8)BQ79600_REQ_CHECK > pReqFrame->uCmd)
                {
                    if(STD_LOW == pBqComMgr->pBswCfg->dio_req(DIO_READ_CHANNEL))
                    {
                        pBqComMgr->pBswCfg->dio_req(DIO_EDGE_ENABLE);
                        pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, BQ79600_WAIT_SPI_RDY_TIME);
                        pBqComMgr->uDrvState = (uint8)BQ79600_STATE_WAIT_RESPRDY;
                        pBqComMgr->uRespRdyProg = TRUE;
                        uRet = (uint8)COMIF_OK;
                    }
                    else
                    {
                        if((uint8)BQ79600_REQ_READ == pReqFrame->uCmd)
                        {
                            pBqComMgr->pBswCfg->dio_req(DIO_EDGE_ENABLE);
                            pBqComMgr->uDrvState = (uint8)BQ79600_STATE_WAIT_RESPONSE;
                            pBqComMgr->uRxInProg = TRUE;
                            qData = (BQ79600_WAIT_SPI_BIT_TIME);                         // TBD: Timings correction
                        }

                        if((uint8)BQ79600_REQ_WRITE == pReqFrame->uCmd)
                        {
                            pBqComMgr->pBswCfg->dio_req(DIO_EDGE_DISABLE);
                            pBqComMgr->uDrvState = (uint8)BQ79600_STATE_WAIT_TX_COMPLETE;
                            pBqComMgr->uTxInProg = TRUE;
                            qData = (BQ79600_WAIT_SPI_BIT_TIME);                         // TBD: Timings correction
                        }

                        pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, qData);                                                   // TODO: add timer for Transfer timer for BQ79600
                        uRet = (uint8)pBqComMgr->pBswCfg->transfer_req(pReqFrame->zTxCmd, pReqFrame->wReqDataLen, NULL, 0u);
                        if(E_OK == uRet)
                        {
                            EMEM_UPDATE_STATICS(pBqComMgr->pEmemStats, qNTxReqs);
                            pBqComMgr->pCurrReq = (Bq79600_ReqDataType *)ctPeQueueRemoveData(pNextFrameQ);
                            uRet = (uint8)COMIF_OK;
                        }
                        else
                        {
                            uRet = (uint8)COMIF_CMD_SPI_TXERROR;
                        }
                    }
                }
                else
                {
                    pBqComMgr->pBswCfg->dio_req(DIO_EDGE_DISABLE);
                    if((uint8)BQ79600_REQ_DELAY == pReqFrame->uCmd)
                    {
                        qData = ((pReqFrame->zTxCmd[0u] << 8u) | (pReqFrame->zTxCmd[1u] << 0u));

                        pBqComMgr->uDrvState = (uint8)BQ79600_STATE_WAIT_DELAY;
                        pBqComMgr->uDelayProg = TRUE;
                        pBqComMgr->pBswCfg->timer_req(TIMER_DELAY, qData);
                        pBqComMgr->pCurrReq = (Bq79600_ReqDataType *)ctPeQueueRemoveData(pNextFrameQ);
                        uRet = (uint8)COMIF_OK;
                    }

                    if((uint8)BQ79600_REQ_WAKEUP == pReqFrame->uCmd)
                    {
                        qData = (((uint32) pReqFrame->zTxCmd[0u] << 8u) | ((uint32) pReqFrame->zTxCmd[1u] << 0u));

                        pBqComMgr->pBswCfg->timer_req(TIMER_DELAY, BQ79600_WAIT_PING_RESPONSE);
                        uRet = (uint8)pBqComMgr->pBswCfg->transfer_req(NULL, qData, NULL, 0u);
                        if(E_OK == uRet)
                        {
                            pBqComMgr->uDrvState = (uint8)BQ79600_STATE_WAIT_TX_COMPLETE;
                            pBqComMgr->uTxInProg = TRUE;
                            pBqComMgr->pCurrReq = (Bq79600_ReqDataType *)ctPeQueueRemoveData(pNextFrameQ);
                            uRet = (uint8)COMIF_OK;
                        }
                        else
                        {
                            uRet = (uint8)COMIF_CMD_SPI_TXERROR;
                        }
                    }

                    if((uint8)BQ79600_REQ_COMCLEAR == pReqFrame->uCmd)
                    {
                        pBqComMgr->uComClearReqExt = TRUE;
                        pBqComMgr->pCurrReq = (Bq79600_ReqDataType *)ctPeQueueRemoveData(pNextFrameQ);
                        uRet = Bq79600_TransferComClear(pBqComMgr);
                    }
                }
            }
            else
            {
                pBqComMgr->uDrvState = (uint8)BQ79600_STATE_IDLE;
                uRet = (uint8)COMIF_OK;
            }
        }
    }
	}

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_TX_PROC, uRet, uCmd, qData);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq79600_CODE) Bq79600_HandleRecieve(void *pComifCtx)
 *********************************************************************************************************************/
/*! \brief          Recieve data processing
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

FUNC(uint8, bq79600_CODE) Bq79600_HandleRecieve(void *pComifCtx)
{
    Bq79600_ManagerType *pBqComMgr = (Bq79600_ManagerType *)  pComifCtx;
    Bq79600_ReqDataType *pReqFrame;
    uint16 wNElements;
    uint8 uRet = COMIF_CMD_INTEGRITY_FAILED;
    uint8 uIndex;

    if(BQ79600_INTEGRITY_CHECK(pBqComMgr))
    {
        wNElements = ctPeQueueGetNOfElements(&pBqComMgr->zReadQueExec);
        for (uIndex = 0; uIndex < wNElements; uIndex++)
        {
            uRet = COMIF_CMD_INVALID_READ;
            pReqFrame = ctPeQueueRemoveData(&pBqComMgr->zReadQueExec);
            if(pReqFrame)
            {
                if(BQ79600_REQ_READ == pReqFrame->uCmd)
                {
                    if((pReqFrame->pRxData) && (pReqFrame->uBlockSiz > 0))
                    {
                        EMEM_UPDATE_STATICS(pBqComMgr->pEmemStats, qNRxResps);
                        if(pReqFrame->pServiceReq->cbRxDataSvc)
                        {
                            uRet = pReqFrame->pServiceReq->cbRxDataSvc(pReqFrame->pServiceReq, pReqFrame->pRxData,
                                                                       pReqFrame->uStatus);
                        }
                    }
                }

                if(QUEUE_E_OK != ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pReqFrame))
                {
                    uRet = Comif_SetErrorDetails(COMIF_SVCID_CMD_RX_PROC, COMIF_CMD_CFQUE_FAILED, uRet, uIndex);
                }
            }
        }
    }
            pBqComMgr->uRxInProg = FALSE;
            readInProgress = FALSE;
            //(void)Bq79600_HandleTransfer(pBqComMgr);
            pBqComMgr->uDrvState = (uint8)BQ79600_STATE_WAIT_UART_TX_COMPLETE;
            //(void)Bq79600_HandleTransfer(pBqComMgr);
	    //
	    
            pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, 1);
            uRet = E_OK;

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_RX_PROC, uRet, 0, wNElements);
}
/**********************************************************************************************************************
 *  FUNC(uint8, bq79600_CODE) Bq79600_Notify(void *pComifCtx, uint8 uNotifyType)
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

FUNC(uint8, bq79600_CODE)  Bq79600_Notify(void *pComifCtx, uint8 uNotifyType)
{
    Bq79600_ManagerType *pBqComMgr = (Bq79600_ManagerType *)  pComifCtx;
    uint8 uRet = (uint8)COMIF_CMD_INTEGRITY_FAILED;

    if(BQ79600_INTEGRITY_CHECK(pBqComMgr))
    {
        switch (uNotifyType)
        {
            case (uint8)TRANSFER_REQUEST_NOTIFY:
            {
                uRet = Bq79600_RespRdyNotify(pBqComMgr);
                break;
            }
            case (uint8)TRANSFER_COMPLETE_NOTIFY:
            {
                uRet = Bq79600_SeqEndNotify(pBqComMgr);
                break;
            }
            case (uint8)TRANSFER_TIMEOUT_NOTIFY:
            {
                uRet = Bq79600_DelayNotify(pBqComMgr);
                break;
            }

            default:
            {
                uRet = (uint8)COMIF_CMD_INVALID_NOTIFY;
                break;
            }
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(void*, bq79600_CODE) Bq79600_Init(const void *pComCfgCtx, uint8 pBq79600Cfg->uBqIfaceCfg)
 *********************************************************************************************************************/
/*! \brief          Initialization of communication device
 *
 *  \param[in]      pComCfgCtx: Communication device configuration
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

FUNC(void*, bq79600_CODE) Bq79600_Init(const void *pComCfgCtx, Emem_StasticsType *pEmemStatsCfg,
                                       const ServiceCfgType *pCommCfg)
{
    Bq79600_ConfigType *pBq79600Cfg = (Bq79600_ConfigType *) pComCfgCtx;
    Bq79600_ManagerType *pBqComMgr;
    Bq79600_ManagerType *pBq79600CtxRet = NULL;
    uint16 wQueIdx;
    uint8 uRet;

    do
    {
        if(NULL == pBq79600Cfg)
        {
            uRet = (uint8)COMIF_CMD_INVALID_CFG;
            break;
        }

        if(pBq79600Cfg->uCmdIfaceCfg >= BQ79600_IFACES)
        {
            uRet = (uint8)COMIF_CMD_INVALID_IFACE_CFG;
            break;
        }

        pBqComMgr = &zBq79600Ctx[pBq79600Cfg->uCmdIfaceCfg];
        if(NULL == pBqComMgr)
        {
            uRet = (uint8)COMIF_CMD_INVALID_MGR;
            break;
        }

        if(BQ79600_INIT == pBqComMgr->uInit)
        {
            uRet = (uint8)COMIF_ALREADY_INUSE;
            break;
        }

        (void)memset(pBqComMgr, 0, sizeof(Bq79600_ManagerType));
        if(NULL == pBq79600Cfg->pBswCfg)
        {
            uRet = (uint8)COMIF_CMD_INVALID_BSW;
            break;
        }

        pBqComMgr->pBswCfg = pBq79600Cfg->pBswCfg;
        uRet = (uint8)pBqComMgr->pBswCfg->init();
        if(E_OK != uRet)
        {
            uRet = (uint8)COMIF_CMD_INVALID_BSW;
            break;
        }

        if(NULL == pCommCfg)
        {
            uRet = (uint8)COMIF_CMD_INVALID_COMMCFG;
            break;
        }
        pBqComMgr->pCommCfg = pCommCfg;

        // Normal Queue
        if((NULL == pBq79600Cfg->pCmdQueFreeMemCfg) ||
           (NULL == pBq79600Cfg->pCmdDataMemCfg) || (!pBq79600Cfg->wCmdQueCntCfg))
        {
            uRet = (uint8)COMIF_CMD_QUECFG_INVALID;
            break;
        }

        uRet = ctPeQueueCreate(&pBqComMgr->zCmdQueFree, pBqComMgr->pBswCfg->qDriverRes, pBqComMgr->pBswCfg,
                               pBq79600Cfg->pCmdQueFreeMemCfg, pBq79600Cfg->wCmdQueSizeCfg,
                               pBq79600Cfg->wCmdQueCntCfg);
        if((uint8)QUEUE_E_OK != uRet)
        {
            uRet = (uint8)COMIF_CMD_CFQUE_FAILED;
            break;
        }

        for (wQueIdx = 0; wQueIdx < pBq79600Cfg->wCmdQueCntCfg; wQueIdx++)
        {
            uRet = ctPeQueueAddData(&pBqComMgr->zCmdQueFree, (void*) &pBq79600Cfg->pCmdDataMemCfg[wQueIdx]);
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

        if(NULL == pBq79600Cfg->pCmdQueExecMemCfg)
        {
            uRet = (uint8)COMIF_CMD_CFQUE_FAILED;
            break;
        }

        for (wQueIdx = 0; wQueIdx < (uint8)COMIF_CMD_PRIO_MAX; wQueIdx++)
        {
            uRet = ctPeQueueCreate(&pBqComMgr->zCmdQueExec[wQueIdx], pBqComMgr->pBswCfg->qDriverRes, pBqComMgr->pBswCfg,
                                    pBq79600Cfg->pCmdQueExecMemCfg[wQueIdx], pBq79600Cfg->wExecQueSizeCfg,
                                    pBq79600Cfg->wExecQueCntCfg);
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

        // Read QUEUE
        if((NULL == pBq79600Cfg->pReadQueExecMemCfg) || (!pBq79600Cfg->wReadQueCntCfg))
        {
            uRet = (uint8)COMIF_CMD_CFQUE_FAILED;
            break;
        }

        uRet = ctPeQueueCreate(&pBqComMgr->zReadQueExec, pBqComMgr->pBswCfg->qDriverRes, pBqComMgr->pBswCfg,
                               pBq79600Cfg->pReadQueExecMemCfg, pBq79600Cfg->wReadQueSizeCfg,
                               pBq79600Cfg->wReadQueCntCfg);
        if((uint8)QUEUE_E_OK != uRet)
        {
            uRet = (uint8)COMIF_CMD_REQUE_FAILED;
            break;
        }

        pBqComMgr->pEmemStats = pEmemStatsCfg;
        pBqComMgr->uCmdIface = pBq79600Cfg->uCmdIfaceCfg;
		pBqComMgr->uComTyp = pBq79600Cfg->ePeripCfg;
        pBqComMgr->uDrvState = (uint8)BQ79600_STATE_IDLE;
        pBqComMgr->qGuardStart = BQ79600_GUARD_START;
        pBqComMgr->qGuardEnd = BQ79600_GUARD_END;
        pBqComMgr->uInit = BQ79600_INIT;

        pBq79600CtxRet = pBqComMgr;
        uRet = (uint8)COMIF_OK;
    }
    while (0);

    (void)Comif_SetErrorDetails(COMIF_SVCID_CMD_INIT, uRet, 0u, 0u);

    return pBq79600CtxRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, bq79600_CODE) BQ79600_EnableSniff( Bq79600_ManagerType *pBmicMgr , uint8 uDevId)
 *********************************************************************************************************************/
/*! \brief          Function Initializes the Gpio Configuration
 *
 *  \param[in]      pBmicMgr: Manager
 *  \param[in]      uDevId: Device Id.
 *  \param[in]
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Bmi_Init is completed with all basic initialization
 *  \post           The request will queued
 *
 *  \return         uint8
 *  \retval         BMI_OK: Successful completion
 *                  BMI_NOT_OK: Failed to process the notification
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, bq79600_CODE) BQ79600_EnableSniff( Bq79600_ManagerType *pBmcMgr , uint8 uDevId)
{
    uint8 uRet = (uint8)BMI_OK;
    uint8 uCmd;
    if (uDevId > 0u)
    {
        uCmd = BQ79600_DEV_CONF1_POR_VAL | BQ79600_DEV_CONF1_SNIFDET_EN_MSK;
        uRet = Comif_SingleWrite(pBmcMgr->pComifCtx, uDevId,BQ79600_DEV_CONF1_OFFSET,
                                 &uCmd,WRITE_1BYTE);
    }
    else
    {
        uRet = (uint8)E_NOT_OK;
    }
    return uRet;
}
/**********************************************************************************************************************
 *  FUNC(uint8, bq79600_CODE) Bq79600_Deinit(void *pComifCtx)
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

FUNC(uint8, bq79600_CODE) Bq79600_Deinit(void *pComifCtx)
{
    Bq79600_ManagerType *pBqComMgr = (Bq79600_ManagerType *)  pComifCtx;
    uint8 uRet = (uint8)COMIF_CMD_INTEGRITY_FAILED;
    uint8 uCmdIfaceCfg = TIBMS_INVALID_IFACE;

    if(BQ79600_INTEGRITY_CHECK(pBqComMgr))
    {
        // TODO: Implement De-Init steps
        uCmdIfaceCfg = pBqComMgr->uCmdIface;

        uRet = (uint8)pBqComMgr->pBswCfg->deinit();
    }

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_DEINIT, uRet, uCmdIfaceCfg, 0);
}

#define BQ79600_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 * End of File: bq79600.c
 *********************************************************************************************************************/
