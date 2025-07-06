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
 *  File:       bq7973x_com.c
 *  Project:    TIBMS
 *  Module:     COMIF
 *  Generator:  Code generation tool (ifany)
 *
 *  Description:  Functionalities for Bq7973x Bridge Device
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05May2022    SEM                  0000000000000    Initial version
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
#include "tibms_pmi.h"
#include "tibms_utils.h"

/**********************************************************************************************************************
 * Version Check (ifrequired)
 *********************************************************************************************************************/

/**	Major Software Config C Version number */
#define BQ7973X_COM_C_MAJOR_VERSION             (0x01u)

/**	Minor Software Config C Version number */
#define BQ7973X_COM_C_MINOR_VERSION             (0x00u)

/** Software Patch Config C Version number */
#define BQ7973X_COM_C_PATCH_VERSION             (0x00u)

#if ((BQ7973X_COM_C_MAJOR_VERSION != BQ7973X_CFG_MAJOR_VERSION) || \
     (BQ7973X_COM_C_MINOR_VERSION != BQ7973X_CFG_MINOR_VERSION) || \
	 (BQ7973X_COM_C_PATCH_VERSION != BQ7973X_CFG_PATCH_VERSION))
#error "tibms: Config version numbers of bq7973x_com.c and bq7973x_cfg.h are inconsistent!"
#endif

#if ((BQ7973X_SW_MAJOR_VERSION != BQ7973X_COM_C_MAJOR_VERSION) || \
     (BQ7973X_SW_MINOR_VERSION != BQ7973X_COM_C_MINOR_VERSION) || \
	 (BQ7973X_SW_PATCH_VERSION != BQ7973X_COM_C_PATCH_VERSION))
#error "tibms: Config version numbers of bq7973x_com.c and bq7971x.h are inconsistent!"
#endif

#if ((BQ7973X_REGS_MAJOR_VERSION != BQ7973X_COM_C_MAJOR_VERSION) || \
     (BQ7973X_REGS_MINOR_VERSION != BQ7973X_COM_C_MINOR_VERSION) || \
	 (BQ7973X_REGS_PATCH_VERSION != BQ7973X_COM_C_PATCH_VERSION))
#error "tibms: Config version numbers of bq7973x_com.c and bq7973x_regs.h are inconsistent!"
#endif

/**********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

#define BQ7973X_GUARD_START                                  	(0xCCA1AB1EU)
#define BQ7973X_GUARD_END                               		(0xCAFECEEDU)
#define BQ7973X_INIT                                            (0xBAu)

#define BQ7973X_INTEGRITY_CHECK(pBqComMgr)                 (((pBqComMgr) != NULL) && \
                                                                 (BQ7973X_GUARD_START == (pBqComMgr)->qGuardStart) && \
                                                                 (BQ7973X_GUARD_END == (pBqComMgr)->qGuardEnd) && \
                                                                 (BQ7973X_INIT == (pBqComMgr)->uInit))

#define BQ7973X_DEV_TX_FIFO_LEN                                 (0x80u)

#define BQ7973X_DELAY_CMD_LEN                                   (0x02u)
#define BQ7973X_WAKEUP_PING_LEN                                 (0x02u)

#define BQ7973X_WRITE_DATA_LEN_MIN                              (0x01u)
#define BQ7973X_WRITE_DATA_LEN_MAX                              (0x08u)
#define BQ7973X_READ_DATA_LEN_MAX                               (0x80u)
#define BQ7973X_READ_DATA_LEN_MIN                               (0x01u)
#define BQ7973X_DEV_ADR_RANGE_MAX                               (0x3Fu)

#define BQ7973X_READ_LEN_OFFSET                                 (1u)

/* SPI baud rate is 2Mbps, CS to CS time needs to be considered */
#define BQ7973X_SLP2ACK_DATA_LEN                                (0x34u)
#define BQ7973X_WAKEUP_DATA_LEN                                 (0x280u)
#define BQ7973X_COMM_CLEAR_DATA_LEN                             (0x01u)

#define BQ7973X_REQUEST_BYTE_NUM_MASK                           (0x7Fu)

#define Bq7973x_FrameInitpack(type, group, size)     \
        ((uint8)0x80u | (((type) & 7u) << 4) | (((group) & 1u) << 3) | ((((uint8)(size)) - 1u) & 7u))

#define Bq7973x_GetCmdType(pData)                               (((pData)[0u] >> 4u) & 0x07u)
#define Bq7973x_GetCmdDevAdr(pData)                             ((pData)[1u] & 0x3Fu)
#define Bq7973x_GetCmd16bit(pData, Pos)                         (((uint16)((pData)[(Pos)]) << 8u) | ((uint16)((pData)[(Pos) + 1u])))
#define Bq7973x_GetReqCmdData(pData, pos)                       (((pData)[(pos)] & 0x7Fu))
#define Bq7973x_GetRespCmdData(pData)                           (((pData)[0u] & 0x7Fu) + 1u)
#define Bq7973x_GetRespCmdCrc(pData, Pos)                       (((uint16)(pData)[(Pos) + 1u] << 8u) | ((uint16)(pData)[(Pos)]))
#define Bq7973x_GetRespCmdDataByte(pData, offset)               ((pData)[4u + (offset)])

/**********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

typedef enum Bq7973x_FrameCmdType_Tag
{
    BQ7973X_REQ_READ,
    BQ7973X_REQ_WRITE,
    BQ7973X_REQ_CHECK,
    BQ7973X_REQ_COMCLEAR = REQTYPE_CUSTOM_COMCLEAR,
    BQ7973X_REQ_WAKEUP = REQTYPE_CUSTOM_WAKEUP,
    BQ7973X_REQ_DELAY = REQTYPE_CUSTOM_DELAY,
    BQ7973X_REQ_CUSTOM = REQTYPE_CUSTOM_CMD_W
}
Bq7973x_FrameCmdType;

typedef enum Bq7973x_StatusType_Tag
{
    BQ7973X_STATE_IDLE = 0x20,
    BQ7973X_STATE_TRANSIENT,
    BQ7973X_STATE_WAIT_SPIRDY,
    BQ7973X_STATE_WAIT_RESPONSE,
    BQ7973X_STATE_WAIT_TX_COMPLETE,
    BQ7973X_STATE_WAIT_DELAY,
    BQ7973X_STATE_WAIT_COMCLEAR,

    BQ7973X_STATE_ERROR
}
Bq7973x_StatusType;

typedef struct Bq7973x_ComManagerType_Tag
{
    uint32 qGuardStart;

    const Basic_ConfigType *pBswCfg;
    Emem_StasticsType *pEmemStats;

    ctPeQueueType zCmdQueFree;
    ctPeQueueType zCmdQueExec;
    ctPeQueueType zReadQueExec;

    uint8 uDrvState;
    uint8 uCmdIface;
    uint8 uNAFEs;
    uint8 uInit;

    uint8 uCcProg;
    uint8 uSpiRdyProg;
    uint8 uTxInProg;
    uint8 uRxInProg;

    uint8 uDelayProg;
    uint8 uComClearReqExt;
    uint8 uRsrvd[2];

    uint32 qGuardEnd;
}
Bq7973x_ComManagerType;

/**********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

#define BQ7973X_COM_START_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

static Bq7973x_ComManagerType zBq7973xCtx[BQ7973X_COMIF_IFACES];

#define BQ7973X_COM_STOP_SEC_VAR_NOINIT_UNSPECIFIED
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

#define BQ7973X_COM_START_SEC_CODE
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 *  FUNC(uint16, bq7973x_CODE) ResponseFrameCrcCheck(const uint8 *pData, uint16 wDataLen)
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

STATIC FUNC(uint8, bq7973x_CODE) ResponseFrameCrcCheck(const uint8 *pData, uint8 uSFrameLen, uint8 uFrameNum)
{
    uint8 uRet =  (uint8) RESP_VALID;
    uint8 uIndex;
    uint16 wCrc;

    for (uIndex = 0u; uIndex < uFrameNum; uIndex ++)
    {
        wCrc = CalculateCRC16(&pData[uSFrameLen * uIndex], uSFrameLen);
        if(0u != wCrc)
        {
            uRet = (uint8) RESP_INVALID;
            break;
        }
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_TransferComClear(Bq7973x_ComManagerType *pBqComMgr)
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

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_TransferComClear(Bq7973x_ComManagerType *pBqComMgr)
{
    uint8 uRet = (uint8) COMIF_NOT_OK;

    pBqComMgr->pBswCfg->transfer_ctrl(CANCEL_TX_REQ, NULL, NULL, NULL);
    pBqComMgr->pBswCfg->transfer_ctrl(CANCEL_RX_REQ, NULL, NULL, NULL);

    if((uint8) FALSE ==  pBqComMgr->uComClearReqExt)
    {
        EMEM_UPDATE_STATICS(pBqComMgr->pEmemStats, qNComRec);
    }

    pBqComMgr->uDrvState =  (uint8) BQ7973X_STATE_WAIT_COMCLEAR;
    uRet = (uint8)pBqComMgr->pBswCfg->transfer_req(NULL, 1u, NULL, 0u);
    if(E_OK == uRet)
    {
        pBqComMgr->uCcProg = (uint8) TRUE;
        uRet = (uint8) COMIF_OK;
    }
    else
    {
        uRet = (uint8) COMIF_CMD_SPI_TXERROR;
    }

    pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, BQ7973X_WAIT_COMCLEAR_RESP);

    return uRet;
}
/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) ScheduleFrameRequest(Bq7973x_ComManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
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
STATIC FUNC(uint8, bq7973x_CODE) ScheduleFrameRequest(Bq7973x_ComManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
{
    Bq7973x_ReqDataType *pFrame;
    uint8 *pRxDataIn;
    uint8 uRet = COMIF_CMD_CFQUE_FAILED;
    uint16 wData;
    uint16 wIndex = 0u;
    uint8 uReqCmdInit = FALSE;
    uint8 uScheduleFrame = FALSE;
    uint8 uSingleReq = FALSE;

    pFrame = (Bq7973x_ReqDataType *) ctPeQueueRemoveData(&pBqComMgr->zCmdQueFree);
    if(pFrame)
    {
        switch((Comif_CmdReqType)pReqCmd->uReqType) // cast to enum type to avoid violation :Use of underlying enum representation value.
        {
            case REQTYPE_STACK_W:
            case REQTYPE_BROADCAST_W:
            case REQTYPE_BROADCAST_W_REV:
            {
                pFrame->uCmd = BQ7973X_REQ_WRITE;
                pFrame->pServiceReq = NULL;
                pFrame->pRxData = NULL;
                pFrame->wReqDataLen = (BQ7973X_REQ_FRAME_FIXED_LEN + (pReqCmd->wReqCmdSiz - 1u));
                uReqCmdInit = TRUE;
                break;
            }
            case REQTYPE_STACK_R:
            case REQTYPE_BROADCAST_R:
            {
                pReqCmd->uReqType = REQTYPE_BROADCAST_R;
                pFrame->uCmd = BQ7973X_REQ_READ;

                pFrame->wReqDataLen = BQ7973X_REQ_FRAME_FIXED_LEN;
                pFrame->wRspDataLen = (uint16) ((pReqCmd->pData[0u] + BQ7973X_RSP_FRAME_FIXED_LEN) * pReqCmd->uDevId);

                pFrame->wCurrRspDataLen = pFrame->wRspDataLen;
                pFrame->uBlockSiz = (pReqCmd->pData[0u] & BQ7973X_REQUEST_BYTE_NUM_MASK) + BQ7973X_RSP_FRAME_FIXED_LEN;
                pFrame->pServiceReq = pReqCmd->pServiceCfg;
                pFrame->pRxData = (uint8 *) pReqCmd->pServiceCfg->pRawData;
                uReqCmdInit = TRUE;

                if((NULL == pFrame->pServiceReq) || (pFrame->pServiceReq->wDataLen < pFrame->wRspDataLen))
                {
                    ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pFrame);
                    uRet = COMIF_CMD_INVALID_CMDDATA;
                    uReqCmdInit = FALSE;
                }
                break;
            }
            case REQTYPE_SINGLE_W:
            {
                pFrame->uCmd = BQ7973X_REQ_WRITE;
                pFrame->pServiceReq = NULL;
                pFrame->pRxData = NULL;
                pFrame->wReqDataLen = (BQ7973X_REQ_FRAME_FIXED_LEN + pReqCmd->wReqCmdSiz);
                uReqCmdInit = TRUE;
                uSingleReq = TRUE;
                break;
            }
            case REQTYPE_SINGLE_R:
            {
                pFrame->uCmd = BQ7973X_REQ_READ;
                uSingleReq = TRUE;

                pFrame->wReqDataLen = (BQ7973X_REQ_FRAME_FIXED_LEN + 1u);
                pFrame->wRspDataLen = (uint16) (pReqCmd->pData[0u] + BQ7973X_RSP_FRAME_FIXED_LEN);
                pFrame->wCurrRspDataLen = pFrame->wRspDataLen;
                pFrame->uBlockSiz = (pReqCmd->pData[0u] & BQ7973X_REQUEST_BYTE_NUM_MASK) + BQ7973X_RSP_FRAME_FIXED_LEN;
                pFrame->pServiceReq = pReqCmd->pServiceCfg;

                if((pFrame->pServiceReq) && (pFrame->pServiceReq->wDataLen >= \
                   (pFrame->wRspDataLen + (pReqCmd->pServiceCfg->uBlockLen * pReqCmd->uDevId))))
                {
                    pRxDataIn = (uint8 *) pReqCmd->pServiceCfg->pRawData;
                    pFrame->pRxData = &pRxDataIn[pReqCmd->pServiceCfg->uBlockLen * pReqCmd->uDevId]; // uDevId will be the Index
                    uReqCmdInit = TRUE;
                }
                else
                {
                    ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pFrame);
                    uRet = COMIF_CMD_INVALID_CMDDATA;
                    uReqCmdInit = FALSE;
                }
                break;
            }
            case REQTYPE_CUSTOM_ERROR:
            {
                pFrame->uCmd = BQ7973X_REQ_WRITE;
                pFrame->pServiceReq = NULL;
                pFrame->wReqDataLen = (BQ7973X_REQ_FRAME_FIXED_LEN + pReqCmd->wReqCmdSiz+1);
                pFrame->zTxCmd[wIndex++] = 0xFFu;

                uReqCmdInit = TRUE;
                uSingleReq = TRUE;
                break;
            }
            case REQTYPE_CUSTOM_DELAY:
            case REQTYPE_CUSTOM_WAKEUP:
            case REQTYPE_CUSTOM_COMCLEAR:
            {
                pFrame->uCmd = pReqCmd->uReqType;
                for (wData = 0; wData < pReqCmd->wReqCmdSiz; wData++)
                {
                    pFrame->zTxCmd[wIndex++] = pReqCmd->pData[wData];
                }

                uScheduleFrame = TRUE;
                break;
            }
            default:
            {
                ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pFrame);
                uRet = COMIF_CMD_INVALID_REQCMD;
                break;
            }
        }
    }
    else
    {
        EMEM_UPDATE_STATICS(pBqComMgr->pEmemStats, qNReqQueFail);
        uRet = COMIF_CMD_CFQGET_EMPTY;
    }

    if(uReqCmdInit)
    {
        pFrame->zTxCmd[wIndex++] = Bq7973x_FrameInitpack(pReqCmd->uReqType, pReqCmd->uGroupExclude,
                                                             pReqCmd->wReqCmdSiz);
        if(TRUE == uSingleReq)
        {
            pFrame->zTxCmd[wIndex++] = pReqCmd->uDevId;
        }

        pFrame->zTxCmd[wIndex++] = (pReqCmd->wRegAdr >> TIBMS_8BIT_OFFSET) & TIBMS_8BIT_MASK;
        pFrame->zTxCmd[wIndex++] = (pReqCmd->wRegAdr) & TIBMS_8BIT_MASK;

        for (wData = 0; wData < pReqCmd->wReqCmdSiz; wData++)
        {
            pFrame->zTxCmd[wIndex++] = pReqCmd->pData[wData];
        }

        wData = CalculateCRC16(pFrame->zTxCmd, wIndex);
        pFrame->zTxCmd[wIndex++] = (wData & TIBMS_8BIT_MASK);
        pFrame->zTxCmd[wIndex++] = ((wData >> TIBMS_8BIT_OFFSET) & TIBMS_8BIT_MASK);

        uScheduleFrame = TRUE;
    }

    if(uScheduleFrame)
    {
        uRet = (uint8) COMIF_CMD_CEQUE_FAILED;
        if((uint8) QUEUE_E_OK == ctPeQueueAddData(&pBqComMgr->zCmdQueExec, pFrame))
        {
            if((uint8) BQ7973X_STATE_IDLE == pBqComMgr->uDrvState)
            {
                pBqComMgr->uDrvState = (uint8) BQ7973X_STATE_TRANSIENT;
                pBqComMgr->pBswCfg->transfer_ctrl(NOTIFY_TX_REQ, NULL, NULL, NULL);
            }
            uRet = COMIF_OK;
        }
    }

    EMEM_UPDATE_STATICS_DATA(pBqComMgr->pEmemStats, qQueMaxUsage, pBqComMgr->zCmdQueFree.nMaxUsage);

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_SCHEDULE, uRet, pReqCmd->uReqType, pBqComMgr->uDrvState);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_BroadcastRead(Bq7973x_ComManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
 *********************************************************************************************************************/
/*! \brief          Broadcast Read Request
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *  \param[in]      pReqCmd: Request Command
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

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_BroadcastRead(Bq7973x_ComManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
{
    uint8 uRet = (uint8) COMIF_CMD_INVALID_CMDDATA;
    uint8 uLength;

    if(((pReqCmd->pData) != NULL) && (pReqCmd->uDevId <= BQ7973X_DEV_ADR_RANGE_MAX) &&
       (pReqCmd->wReqCmdSiz <= BQ7973X_WRITE_DATA_LEN_MAX) &&
       (pReqCmd->wReqCmdSiz >= BQ7973X_WRITE_DATA_LEN_MIN))
    {
        uLength = ((uint8) *pReqCmd->pData) - BQ7973X_READ_LEN_OFFSET;
        pReqCmd->pData = &uLength;

        uRet = ScheduleFrameRequest(pBqComMgr, pReqCmd);
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_BroadcastWrite(Bq7973x_ComManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
 *********************************************************************************************************************/
/*! \brief          Broadcast Write Request
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *  \param[in]      pReqCmd: Request Command
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

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_BroadcastWrite(Bq7973x_ComManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
{
    uint8 uRet = (uint8)  COMIF_CMD_INVALID_CMDDATA;

    if((pReqCmd->pData != NULL) && (pReqCmd->wReqCmdSiz <= BQ7973X_WRITE_DATA_LEN_MAX) &&
       (pReqCmd->wReqCmdSiz >= BQ7973X_WRITE_DATA_LEN_MIN))
    {
        uRet = ScheduleFrameRequest(pBqComMgr, pReqCmd);
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_StackRead(Bq7973x_ComManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
 *********************************************************************************************************************/
/*! \brief          Stack Read Request
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *  \param[in]      pReqCmd: Request Command
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

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_StackRead(Bq7973x_ComManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
{
    uint8 uRet = (uint8) COMIF_CMD_INVALID_CMDDATA;
    uint8 uLength;

    if((pReqCmd->pData != NULL) && (pReqCmd->uDevId <= BQ7973X_DEV_ADR_RANGE_MAX) &&
       (pReqCmd->wReqCmdSiz <= BQ7973X_WRITE_DATA_LEN_MAX) &&
       (pReqCmd->wReqCmdSiz >= BQ7973X_WRITE_DATA_LEN_MIN))
    {
        uLength = ((uint8) *pReqCmd->pData) - BQ7973X_READ_LEN_OFFSET;
        pReqCmd->pData = &uLength;

        uRet = ScheduleFrameRequest(pBqComMgr, pReqCmd);
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_StackWrite(Bq7973x_ComManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
 *********************************************************************************************************************/
/*! \brief          Stack Write Request
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *  \param[in]      pReqCmd: Request Command
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

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_StackWrite(Bq7973x_ComManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
{
    uint8 uRet = (uint8) COMIF_CMD_INVALID_CMDDATA;

    if((pReqCmd->pData != NULL) && (pReqCmd->wReqCmdSiz <= BQ7973X_WRITE_DATA_LEN_MAX) &&
       (pReqCmd->wReqCmdSiz >= BQ7973X_WRITE_DATA_LEN_MIN))
    {
        uRet = ScheduleFrameRequest(pBqComMgr, pReqCmd);
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_SingleRead(Bq7973x_ComManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
 *********************************************************************************************************************/
/*! \brief          Single Read Request
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *  \param[in]      pReqCmd: Request Command
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

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_SingleRead(Bq7973x_ComManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
{
    uint8 uRet = (uint8) COMIF_CMD_INVALID_CMDDATA;
    uint8 uLength;

    if(((pReqCmd->pData) != NULL) && (pReqCmd->uDevId <= BQ7973X_DEV_ADR_RANGE_MAX) &&
       (pReqCmd->pData[0u] <= BQ7973X_READ_DATA_LEN_MAX) &&
       (pReqCmd->pData[0u] >= BQ7973X_READ_DATA_LEN_MIN))
    {
        uLength = (uint8) ((uint8) pReqCmd->pData[0u] - (uint8) BQ7973X_READ_LEN_OFFSET);
        pReqCmd->pData = &uLength;

        uRet = ScheduleFrameRequest(pBqComMgr, pReqCmd);
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_SingleWrite(Bq7973x_ComManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
 *********************************************************************************************************************/
/*! \brief          Single Write Request
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *  \param[in]      pReqCmd: Request Command
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

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_SingleWrite(Bq7973x_ComManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
{
    uint8 uRet = (uint8) COMIF_CMD_INVALID_CMDDATA;

    if(((pReqCmd->pData) != NULL) && (pReqCmd->uDevId <= BQ7973X_DEV_ADR_RANGE_MAX) &&
       (pReqCmd->wReqCmdSiz <= BQ7973X_WRITE_DATA_LEN_MAX) &&
       (pReqCmd->wReqCmdSiz >= BQ7973X_WRITE_DATA_LEN_MIN))
    {
        uRet = ScheduleFrameRequest(pBqComMgr, pReqCmd);
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_SingleWriteError(Bq7973x_ComManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
 *********************************************************************************************************************/
/*! \brief          Function to simulate NFAULT ERROR
 *
 *  \param[in]      pComifCtx: Communication device Manager
 *  \param[in]      pReqCmd: Request Command
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of recieve data processing
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_SingleWriteError(Bq7973x_ComManagerType *pBqComMgr, Comif_ReqCmdType *pReqCmd)
{
    uint8 uRet = (uint8) COMIF_CMD_INVALID_CMDDATA;

    if(((pReqCmd->pData) != NULL) && (pReqCmd->uDevId <= BQ7973X_DEV_ADR_RANGE_MAX) &&
       (pReqCmd->wReqCmdSiz <= BQ7973X_WRITE_DATA_LEN_MAX) &&
       (pReqCmd->wReqCmdSiz >= BQ7973X_WRITE_DATA_LEN_MIN))
    {
        uRet = ScheduleFrameRequest(pBqComMgr, pReqCmd);
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_SendComClear(Bq7973x_ComManagerType *pBqComMgr)
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

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_SendComClear(Bq7973x_ComManagerType *pBqComMgr)
{
    Comif_ReqCmdType zReqcmd;
    uint8 uData = (uint8) BQ7973X_COMM_CLEAR_DATA_LEN;

    zReqcmd.uReqType = (uint8) REQTYPE_CUSTOM_COMCLEAR;
    zReqcmd.uGroupExclude = 0u;
    zReqcmd.wReqCmdSiz = (uint8) BQ7973X_COMM_CLEAR_DATA_LEN;
    zReqcmd.uDevId = 0u;
    zReqcmd.wRegAdr = 0u;
    zReqcmd.pData = &uData;
    zReqcmd.pServiceCfg = NULL;

    return ScheduleFrameRequest(pBqComMgr, &zReqcmd);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_SetDelay(Bq7973x_ComManagerType *pBqComMgr, uint32 qTimeMs)
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

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_SetDelay(Bq7973x_ComManagerType *pBqComMgr, uint32 qTimeMs)
{
    Comif_ReqCmdType zReqcmd;
    uint8 uRet;
    uint8 uData[BQ7973X_DELAY_CMD_LEN];

    uData[0u] = (uint8) ((qTimeMs >> 8u) & TIBMS_8BIT_MASK);
    uData[1u] = (uint8) ((qTimeMs >> 0u) & TIBMS_8BIT_MASK);

    zReqcmd.uReqType = (uint8) REQTYPE_CUSTOM_DELAY;
    zReqcmd.uGroupExclude = 0u;
    zReqcmd.wReqCmdSiz = BQ7973X_DELAY_CMD_LEN;
    zReqcmd.uDevId = 0u;
    zReqcmd.wRegAdr = 0u;
    zReqcmd.pData = uData;
    zReqcmd.pServiceCfg = NULL;

    uRet = ScheduleFrameRequest(pBqComMgr, &zReqcmd);

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_WakePing(Bq7973x_ComManagerType *pBqComMgr, uint32 qTimeMs)
 *********************************************************************************************************************/
/*! \brief          Wakeup Ping during the startup
 *
 *  \param[in]      pComifCtx: Communication device Manager
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

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_WakePing(Bq7973x_ComManagerType *pBqComMgr, uint32 qTimeMs)
{
    uint8 uRet;
    Comif_ReqCmdType zReqcmd;
    uint8 uData[BQ7973X_DELAY_CMD_LEN];

    uData[0u] = ((BQ7973X_WAKEUP_DATA_LEN >> 8u) & TIBMS_8BIT_MASK);
    uData[1u] = ((BQ7973X_WAKEUP_DATA_LEN >> 0u) & TIBMS_8BIT_MASK);

    zReqcmd.uReqType = (uint8) REQTYPE_CUSTOM_WAKEUP;
    zReqcmd.uGroupExclude = 0u;
    zReqcmd.wReqCmdSiz = BQ7973X_WAKEUP_PING_LEN;
    zReqcmd.uDevId = 0u;
    zReqcmd.wRegAdr = 0u;
    zReqcmd.pData = uData;
    zReqcmd.pServiceCfg = NULL;

    uRet = ScheduleFrameRequest(pBqComMgr, &zReqcmd);
    if((uint8) COMIF_OK == uRet)
    {
        uRet = Bq7973x_SetDelay(pBqComMgr, qTimeMs);
    }

    return uRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_DelayNotify(Bq7973x_ComManagerType *pBqComMgr)
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

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_DelayNotify(Bq7973x_ComManagerType *pBqComMgr)
{
    Bq7973x_ReqDataType *pFrame;
    uint8  uRet = (uint8) COMIF_CMD_INVALID_DRVSTATE;

    switch (pBqComMgr->uDrvState)
    {
        case (uint8) BQ7973X_STATE_WAIT_RESPONSE:
        case (uint8) BQ7973X_STATE_WAIT_SPIRDY:
        case (uint8) BQ7973X_STATE_WAIT_TX_COMPLETE:
        case (uint8) BQ7973X_STATE_WAIT_COMCLEAR:
        {
            uRet = Bq7973x_TransferComClear(pBqComMgr);
            break;
        }
        case (uint8) BQ7973X_STATE_WAIT_DELAY:
        {
            pFrame = (Bq7973x_ReqDataType*) ctPeQueueRemoveData(&pBqComMgr->zCmdQueExec);
            if(pFrame != NULL)
            {
                if((uint8) QUEUE_E_OK != ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pFrame))
                {
                    uRet = (uint8) COMIF_CMD_CFQUE_FAILED;
                }
            }
            pBqComMgr->uDelayProg = (uint8) FALSE;
            pBqComMgr->uDrvState = (uint8) BQ7973X_STATE_TRANSIENT;
            (void) Bq7973x_HandleTransfer(pBqComMgr);
            break;
        }
        default:
        {
            uRet = (uint8) COMIF_CMD_INVALID_DRVSTATE;
            break;
        }
    }

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_DELAY_NOTIFY, uRet, pBqComMgr->uDrvState, 0u);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_SpiSeqNotify(Bq7973x_ComManagerType *pBqComMgr)
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

STATIC FUNC(uint8, bq7973x_CODE) Bq7973x_SpiSeqNotify(Bq7973x_ComManagerType *pBqComMgr)
{
    Bq7973x_ReqDataType *pFrame = NULL;
    uint8 uRet = (uint8) COMIF_OK;
    uint8 uNFrames;

    switch (pBqComMgr->uDrvState)
    {
        case (uint8) BQ7973X_STATE_WAIT_TX_COMPLETE:
        {
            pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, 0u);
            pBqComMgr->uTxInProg = (uint8) FALSE;

            pFrame = (Bq7973x_ReqDataType *) ctPeQueueRemoveData(&pBqComMgr->zCmdQueExec);
            if(pFrame != NULL)
            {
                if((uint8) QUEUE_E_OK != ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pFrame))
                {
                    uRet = (uint8) COMIF_CMD_CFQUE_FAILED;;
                }
            }
            pBqComMgr->uDrvState = (uint8) BQ7973X_STATE_TRANSIENT;
            (void)Bq7973x_HandleTransfer(pBqComMgr);
            break;
        }
        case (uint8) BQ7973X_STATE_WAIT_RESPONSE:
        {
            pFrame = (Bq7973x_ReqDataType *)ctPeQueueGetData(&pBqComMgr->zCmdQueExec);
            if(pFrame != NULL)
            {
                pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, 0u);
                if((pFrame->pRxData != NULL) && (pFrame->uBlockSiz > 0u))
                {
                    uRet = (uint8) COMIF_CMD_CRC_FAILED;
                    uNFrames = (uint8)(pFrame->wRspDataLen / pFrame->uBlockSiz);
                    pFrame->uStatus = ResponseFrameCrcCheck(pFrame->pRxData, pFrame->uBlockSiz, uNFrames);
                    if((uint8) RESP_VALID == pFrame->uStatus)
                    {
                        uRet = (uint8) COMIF_CMD_CEQUE_FAILED;
                        if((uint8) QUEUE_E_OK == ctPeQueueAddData(&pBqComMgr->zReadQueExec, pFrame))
                        {
                            (void)ctPeQueueRemoveData(&pBqComMgr->zCmdQueExec);
                            uRet = (uint8) COMIF_OK;
                        }
                        pBqComMgr->pBswCfg->transfer_ctrl(NOTIFY_RX_REQ, NULL, NULL, NULL);
                    }
                    else
                    {
                        EMEM_UPDATE_STATICS(pBqComMgr->pEmemStats, qNCrcFails);
                    }
                }
                pBqComMgr->uRxInProg = (uint8) FALSE;
                pBqComMgr->uDrvState = (uint8) BQ7973X_STATE_TRANSIENT;
                (void)Bq7973x_HandleTransfer(pBqComMgr);
            }
            break;
        }
        case (uint8)BQ7973X_STATE_WAIT_COMCLEAR:
        {
            pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, 0u);
            pBqComMgr->uCcProg = (uint8)FALSE;
            if(pBqComMgr->uComClearReqExt != 0u)
            {
                pBqComMgr->uComClearReqExt = (uint8)FALSE;
                pFrame = (Bq7973x_ReqDataType *) ctPeQueueRemoveData(&pBqComMgr->zCmdQueExec);
                if(pFrame != NULL)
                {
                    if((uint8)QUEUE_E_OK != ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pFrame))
                    {
                        uRet = (uint8)COMIF_CMD_CFQUE_FAILED;
                    }
                }
            }
            pBqComMgr->uDrvState = (uint8)BQ7973X_STATE_TRANSIENT;
            (void)Bq7973x_HandleTransfer(pBqComMgr);
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
 *  External Functions Definition
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_Control(void *pComifCtx, uint8 uCmd, void *pData)
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

FUNC(uint8, bq7973x_CODE) Bq7973x_Control(void *pComifCtx, uint8 uCmd, void *pData)
{
    Bq7973x_ComManagerType *pBqComMgr = (Bq7973x_ComManagerType *)  pComifCtx;
    uint8 uRet = (uint8)COMIF_CMD_INTEGRITY_FAILED;
    Comif_ReqCmdType *pReqCmd;
    uint32 qTimeUs;

    if(BQ7973X_INTEGRITY_CHECK(pBqComMgr))
    {
        uRet = (uint8)COMIF_CMD_INVALID_REQDATA;
        switch(uCmd)
        {
            case (uint8)COMIF_CMD_SGLREAD:
            {
                pReqCmd = (Comif_ReqCmdType *) pData;
                if(pReqCmd  != NULL )
                {
                    uRet = Bq7973x_SingleRead(pBqComMgr, pReqCmd);
                }
                break;
            }
            case (uint8)COMIF_CMD_SGLWRITE:
            {
                pReqCmd = (Comif_ReqCmdType *) pData;
                if(pReqCmd != NULL )
                {
                    uRet = Bq7973x_SingleWrite(pBqComMgr, pReqCmd);
                }
                break;
            }
            case (uint8)COMIF_CMD_STKREAD:
            {
                pReqCmd = (Comif_ReqCmdType *) pData;
                if(pReqCmd != NULL  )
                {
                    uRet = Bq7973x_StackRead(pBqComMgr, pReqCmd);
                }
                break;
            }
            case (uint8)COMIF_CMD_STKWRITE:
            {
                pReqCmd = (Comif_ReqCmdType *) pData;
                if(pReqCmd != NULL )
                {
                    uRet = Bq7973x_StackWrite(pBqComMgr, pReqCmd);
                }
                break;
            }
            case (uint8)COMIF_CMD_ALLREAD:
            {
                pReqCmd = (Comif_ReqCmdType *) pData;
                if(pReqCmd != NULL )
                {
                    uRet = Bq7973x_BroadcastRead(pBqComMgr, pReqCmd);
                }
                break;
            }
            case (uint8)COMIF_CMD_ALLWRITE:
            {
                pReqCmd = (Comif_ReqCmdType *) pData;
                if(pReqCmd != NULL  )
                {
                    uRet = Bq7973x_BroadcastWrite(pBqComMgr, pReqCmd);
                }
                break;
            }
            case (uint8)COMIF_CMD_DELAY:
            {
                pReqCmd = (Comif_ReqCmdType*) pData;
                if(pReqCmd != NULL )
                {
                    uRet = ScheduleFrameRequest(pBqComMgr, pReqCmd);
                }
                break;
            }
            case (uint8)COMIF_CMD_WAKEUP:
            {
                if(pData != NULL )
                {
                    qTimeUs = (uint32) (*((uint32 *) pData));
                    uRet = Bq7973x_WakePing(pBqComMgr, qTimeUs);
                }
                break;
            }
            case (uint8)COMIF_CMD_ERROR:
            {
                pReqCmd = (Comif_ReqCmdType *) pData;
                if(pReqCmd != NULL )
                {
                    uRet = Bq7973x_SingleWriteError(pBqComMgr, pReqCmd);
                }
                break;
            }
            case (uint8)COMIF_CMD_COMCLEAR:
            {
                uRet = Bq7973x_SendComClear(pBqComMgr);
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

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_CTRL_PROC, uRet, uCmd, 0u);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_HandleTransfer(void *pComifCtx)
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

FUNC(uint8, bq7973x_CODE) Bq7973x_HandleTransfer(void *pComifCtx)
{
    Bq7973x_ComManagerType *pBqComMgr = (Bq7973x_ComManagerType *)  pComifCtx;
    uint8 uRet = (uint8)COMIF_CMD_INTEGRITY_FAILED;
    Bq7973x_ReqDataType *pFrame;
    uint32 qData = 0u;
    uint8 uCmd = 0u;

    if(BQ7973X_INTEGRITY_CHECK(pBqComMgr))
    {
        pFrame = (Bq7973x_ReqDataType *)ctPeQueueGetData(&pBqComMgr->zCmdQueExec);
        if(pFrame != NULL )
        {
            uCmd = pFrame->uCmd;
            if((uint8)BQ7973X_REQ_CHECK > pFrame->uCmd)
            {
                if((uint8)BQ7973X_REQ_READ == pFrame->uCmd)
                {
                    pBqComMgr->uDrvState = (uint8)BQ7973X_STATE_WAIT_RESPONSE;
                    pBqComMgr->uRxInProg = (uint8)TRUE;
                    qData = (BQ7973X_WAIT_SPI_BIT_TIME);                         // TBD: Timings correction

                    pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, qData);
                    uRet = (uint8) pBqComMgr->pBswCfg->transfer_req(pFrame->zTxCmd, pFrame->wReqDataLen,
                                                            pFrame->pRxData, pFrame->wCurrRspDataLen); // TBD: check this
                    if(E_OK == uRet)
                    {
                        EMEM_UPDATE_STATICS(pBqComMgr->pEmemStats, qNRxResps);
                        uRet = (uint8)COMIF_OK;
                    }
                    else
                    {
                        uRet = (uint8)COMIF_CMD_SPI_RXERROR;
                    }
                }

                if((uint8)BQ7973X_REQ_WRITE == pFrame->uCmd)
                {
                    pBqComMgr->uDrvState = (uint8)BQ7973X_STATE_WAIT_TX_COMPLETE;
                    pBqComMgr->uTxInProg = (uint8)TRUE;
                    qData = (BQ7973X_WAIT_SPI_BIT_TIME);                         // TBD: Timings correction

                    pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, qData);
                    uRet = (uint8) pBqComMgr->pBswCfg->transfer_req(pFrame->zTxCmd, pFrame->wReqDataLen, NULL, 0u);
                    if(E_OK == uRet)
                    {
                        EMEM_UPDATE_STATICS(pBqComMgr->pEmemStats, qNTxReqs);
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
                if((uint8)BQ7973X_REQ_DELAY == pFrame->uCmd)
                {
                    qData = (uint32)(( (uint32) pFrame->zTxCmd[0u] << 8u) | ((uint32) pFrame->zTxCmd[1u] << 0u));
                    //  qData = (uint32)(( pFrame->zTxCmd[0u] << 8u) | (pFrame->zTxCmd[1u] << 0u)); OLD Code


                    pBqComMgr->uDrvState = (uint8)BQ7973X_STATE_WAIT_DELAY;
                    pBqComMgr->uDelayProg = (uint8)TRUE;
                    pBqComMgr->pBswCfg->timer_req(TIMER_DELAY, qData);
                    uRet = (uint8)COMIF_OK;
                }

                if((uint8)BQ7973X_REQ_WAKEUP == pFrame->uCmd)
                {
                    qData = (((uint32) pFrame->zTxCmd[0u] << 8u) | ((uint32) pFrame->zTxCmd[1u] << 0u));
                    pBqComMgr->pBswCfg->timer_req(TIMER_TRANSFER, BQ7973X_WAIT_PING_RESPONSE); // TBD: Timiing creates problem in Sitara
                    uRet = (uint8) pBqComMgr->pBswCfg->transfer_req(NULL, qData, NULL, 0u);
                    if(E_OK == uRet)
                    {
                        pBqComMgr->uDrvState = (uint8)BQ7973X_STATE_WAIT_TX_COMPLETE;
                        pBqComMgr->uTxInProg = (uint8)TRUE;
                        uRet = (uint8)COMIF_OK;
                    }
                    else
                    {
                        uRet = (uint8)COMIF_CMD_SPI_TXERROR;
                    }
                }

                if((uint8)BQ7973X_REQ_COMCLEAR == pFrame->uCmd)
                {
                    pBqComMgr->uComClearReqExt = (uint8)TRUE;
                    uRet = Bq7973x_TransferComClear(pBqComMgr);
                }
            }
        }
        else
        {
            pBqComMgr->uDrvState = (uint8)BQ7973X_STATE_IDLE;
            uRet = (uint8)COMIF_OK;
        }
    }

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_TX_PROC, uRet, uCmd, qData);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_HandleRecieve(void *pComifCtx)
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

FUNC(uint8, bq7973x_CODE) Bq7973x_HandleRecieve(void *pComifCtx)
{
    Bq7973x_ComManagerType *pBqComMgr = (Bq7973x_ComManagerType *)  pComifCtx;
    Bq7973x_ReqDataType *pFrame;
    uint16 wNElements;
    uint8 uRet = (uint8)COMIF_CMD_INTEGRITY_FAILED;
    uint8 uIndex;

    if(BQ7973X_INTEGRITY_CHECK(pBqComMgr))
    {
        wNElements = ctPeQueueGetNOfElements(&pBqComMgr->zReadQueExec);
        for (uIndex = 0u; uIndex < wNElements; uIndex++)
        {
            uRet = (uint8)COMIF_CMD_INVALID_READ;
            pFrame = (Bq7973x_ReqDataType *)ctPeQueueRemoveData(&pBqComMgr->zReadQueExec);
            if(pFrame != NULL )
            {
                if((uint8)BQ7973X_REQ_READ == pFrame->uCmd)
                {
                    if((pFrame->pRxData != NULL ) && (pFrame->uBlockSiz > 0u))
                    {
                        EMEM_UPDATE_STATICS(pBqComMgr->pEmemStats, qNRxResps);
                        uRet = Pmi_ProcessRxData(pFrame->pServiceReq, pFrame->pRxData, pFrame->uStatus);
                    }
                }

                if((uint8)QUEUE_E_OK != ctPeQueueAddData(&pBqComMgr->zCmdQueFree, pFrame))
                {
                    uRet = Comif_SetErrorDetails(COMIF_SVCID_CMD_RX_PROC, COMIF_CMD_CFQUE_FAILED, uRet, uIndex);
                }
            }
        }
    }

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_RX_PROC, uRet, 0u, wNElements);
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_Notify(void *pComifCtx, uint8 uNotifyType)
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

FUNC(uint8, bq7973x_CODE) Bq7973x_Notify(void *pComifCtx, uint8 uNotifyType)
{
    Bq7973x_ComManagerType *pBqComMgr = (Bq7973x_ComManagerType *)  pComifCtx;
    uint8 uRet = (uint8)COMIF_CMD_INTEGRITY_FAILED;

    if(BQ7973X_INTEGRITY_CHECK(pBqComMgr))
    {
        switch (uNotifyType)
        {
            case (uint8)TRANSFER_COMPLETE_NOTIFY:
            {
                uRet = Bq7973x_SpiSeqNotify(pBqComMgr);
                break;
            }
            case (uint8)TRANSFER_TIMEOUT_NOTIFY:
            {
                uRet = Bq7973x_DelayNotify(pBqComMgr);
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
 * void* Bq7973x_ComInit(const void *pComCfgCtx, Emem_StasticsType *pEmemStatsCfg, const ServiceCfgType *pCommCfg)
 *********************************************************************************************************************/
/*! \brief          Initialization of communication device
 *
 *  \param[in]      pComCfgCtx: Communication device configuration
 *  \param[in]      pEmemStatsCfg: Error Memory Stastics configuration
 *  \param[in]      pCommCfg: Communication Service Configuration
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

FUNC(void*, bq7973x_CODE) Bq7973x_ComInit(const void *pComCfgCtx, Emem_StasticsType *pEmemStatsCfg,
                                          const ServiceCfgType *pCommCfg)
{
    Bq7973x_ComConfigType *pBq7973xCfg = (Bq7973x_ComConfigType *) pComCfgCtx;
    Bq7973x_ComManagerType *pBqComMgr = NULL;
    Bq7973x_ComManagerType *pBq7973xCtxRet = NULL;
    uint16 wQueIdx;
    uint8 uRet;

    do
    {
        if(NULL == pBq7973xCfg)
        {
            uRet = (uint8)COMIF_CMD_INVALID_CFG;
            break;
        }

        if(pBq7973xCfg->uCmdIfaceCfg >= BQ7973X_COMIF_IFACES)
        {
            uRet = (uint8)COMIF_CMD_INVALID_IFACE_CFG;
            break;
        }

        pBqComMgr = &zBq7973xCtx[pBq7973xCfg->uCmdIfaceCfg];
        if(NULL == pBqComMgr)
        {
            uRet = (uint8)COMIF_CMD_INVALID_MGR;
            break;
        }

        if(BQ7973X_INIT == pBqComMgr->uInit)
        {
            uRet = (uint8)COMIF_ALREADY_INUSE;
            break;
        }

        (void) memset(pBqComMgr, 0u, sizeof(Bq7973x_ComManagerType));
        if(NULL == pBq7973xCfg->pBswCfg)
        {
            uRet = (uint8)COMIF_CMD_INVALID_BSW;
            break;
        }

        pBqComMgr->pBswCfg = pBq7973xCfg->pBswCfg;
        uRet = (uint8)pBqComMgr->pBswCfg->init();
        if(E_OK != uRet)
        {
            uRet = (uint8)COMIF_CMD_INVALID_BSW;
            break;
        }

        if((NULL == pBq7973xCfg->pCmdQueFreeMemCfg) || (NULL == pBq7973xCfg->pCmdDataMemCfg) ||
           (!(pBq7973xCfg->wCmdQueCntCfg)))
        {
            uRet = (uint8)COMIF_CMD_QUECFG_INVALID;
            break;
        }

        uRet = ctPeQueueCreate(&pBqComMgr->zCmdQueFree, 0u, NULL, pBq7973xCfg->pCmdQueFreeMemCfg,
                               pBq7973xCfg->wCmdQueSizeCfg, pBq7973xCfg->wCmdQueCntCfg);
        if((uint8)QUEUE_E_OK != uRet)
        {
            uRet = (uint8)COMIF_CMD_CFQUE_FAILED;
            break;
        }

        for (wQueIdx = 0u; wQueIdx < pBq7973xCfg->wCmdQueCntCfg; wQueIdx++)
        {
            uRet = (uint8)ctPeQueueAddData(&pBqComMgr->zCmdQueFree, (void*) &pBq7973xCfg->pCmdDataMemCfg[wQueIdx]);
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

        if(NULL == pBq7973xCfg->pCmdQueExecMemCfg)
        {
            uRet = (uint8)COMIF_CMD_QUECFG_INVALID;
            break;
        }

        uRet = ctPeQueueCreate(&pBqComMgr->zCmdQueExec, 0u, NULL, pBq7973xCfg->pCmdQueExecMemCfg,
                              pBq7973xCfg->wCmdQueSizeCfg, pBq7973xCfg->wCmdQueCntCfg);
        if((uint8)QUEUE_E_OK != uRet)
        {
            uRet = (uint8)COMIF_CMD_CEQUE_FAILED;
            break;
        }

        if((NULL == pBq7973xCfg->pReadQueExecMemCfg) || (!(pBq7973xCfg->wReadQueCntCfg)))
        {
            uRet = (uint8)COMIF_CMD_QUECFG_INVALID;
            break;
        }

        uRet = ctPeQueueCreate(&pBqComMgr->zReadQueExec, 0u, NULL, pBq7973xCfg->pReadQueExecMemCfg,
                              pBq7973xCfg->wReadQueSizeCfg, pBq7973xCfg->wReadQueCntCfg);
        if((uint8)QUEUE_E_OK != uRet)
        {
            uRet = (uint8)COMIF_CMD_REQUE_FAILED;
            break;
        }

        pBqComMgr->pEmemStats = pEmemStatsCfg;
        pBqComMgr->uCmdIface = pBq7973xCfg->uCmdIfaceCfg;
        pBqComMgr->uDrvState = (uint8)BQ7973X_STATE_IDLE;
        pBqComMgr->qGuardStart = BQ7973X_GUARD_START;
        pBqComMgr->qGuardEnd = BQ7973X_GUARD_END;
        pBqComMgr->uInit = BQ7973X_INIT;

        pBq7973xCtxRet = pBqComMgr;
        uRet = (uint8)COMIF_OK;
    }
    while ((uint8)FALSE);

    (void) Comif_SetErrorDetails(COMIF_SVCID_CMD_INIT, uRet, 0u, wQueIdx);

    return pBq7973xCtxRet;
}

/**********************************************************************************************************************
 *  FUNC(uint8, bq7973x_CODE) Bq7973x_ComDeinit(void *pComifCtx)
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

FUNC(uint8, bq7973x_CODE) Bq7973x_ComDeinit(void *pComifCtx)
{
    Bq7973x_ComManagerType *pBqComMgr = (Bq7973x_ComManagerType *)  pComifCtx;
    uint8 uRet = (uint8)COMIF_CMD_INTEGRITY_FAILED;
    uint8 uCmdIface = TIBMS_INVALID_IFACE;

    if(BQ7973X_INTEGRITY_CHECK(pBqComMgr))
    {
        // TODO: Implement De-Init steps
        uCmdIface = pBqComMgr->uCmdIface;
        uRet = (uint8)COMIF_OK;
    }

    return Comif_SetErrorDetails(COMIF_SVCID_CMD_DEINIT, uRet, uCmdIface, 0u);
}

#define BQ7973X_COM_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 * End of File: bq7973x_com.c
 *********************************************************************************************************************/
