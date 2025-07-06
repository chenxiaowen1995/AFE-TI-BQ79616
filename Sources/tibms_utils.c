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
 *  File:       tibms_utils.c
 *  Project:    TIBMS
 *  Module:     UTILS
 *  Generator:  Code generation tool (ifany)
 *
 *  Description:  Exposed functionalities for TI BMS UTILS
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05May2022    SEM                0000000000000    Initial version
 *
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Standard Header Files
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Other Header Files
 *********************************************************************************************************************/

#include "tibms_api.h"
#include "tibms_utils.h"

/*********************************************************************************************************************
 * Version Check (ifrequired)
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

#define CRC_LOOKUP_SIZE                                     (0x100u)

/*********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  Local CONST Object Definitions
 *********************************************************************************************************************/

#define TIBMS_UTILS_START_SEC_CONST_16BIT
#include "Cdd_MemMap.h"

static const uint16 zCrc16Lkup[CRC_LOOKUP_SIZE] =
{
    0x0000u, 0xC0C1u, 0xC181u, 0x0140u, 0xC301u, 0x03C0u, 0x0280u, 0xC241u, 0xC601u, 0x06C0u, 0x0780u, 0xC741u, 0x0500u,
    0xC5C1u, 0xC481u, 0x0440u, 0xCC01u, 0x0CC0u, 0x0D80u, 0xCD41u, 0x0F00u, 0xCFC1u, 0xCE81u, 0x0E40u, 0x0A00u, 0xCAC1u,
    0xCB81u, 0x0B40u, 0xC901u, 0x09C0u, 0x0880u, 0xC841u, 0xD801u, 0x18C0u, 0x1980u, 0xD941u, 0x1B00u, 0xDBC1u, 0xDA81u,
    0x1A40u, 0x1E00u, 0xDEC1u, 0xDF81u, 0x1F40u, 0xDD01u, 0x1DC0u, 0x1C80u, 0xDC41u, 0x1400u, 0xD4C1u, 0xD581u, 0x1540u,
    0xD701u, 0x17C0u, 0x1680u, 0xD641u, 0xD201u, 0x12C0u, 0x1380u, 0xD341u, 0x1100u, 0xD1C1u, 0xD081u, 0x1040u, 0xF001u,
    0x30C0u, 0x3180u, 0xF141u, 0x3300u, 0xF3C1u, 0xF281u, 0x3240u, 0x3600u, 0xF6C1u, 0xF781u, 0x3740u, 0xF501u, 0x35C0u,
    0x3480u, 0xF441u, 0x3C00u, 0xFCC1u, 0xFD81u, 0x3D40u, 0xFF01u, 0x3FC0u, 0x3E80u, 0xFE41u, 0xFA01u, 0x3AC0u, 0x3B80u,
    0xFB41u, 0x3900u, 0xF9C1u, 0xF881u, 0x3840u, 0x2800u, 0xE8C1u, 0xE981u, 0x2940u, 0xEB01u, 0x2BC0u, 0x2A80u, 0xEA41u,
    0xEE01u, 0x2EC0u, 0x2F80u, 0xEF41u, 0x2D00u, 0xEDC1u, 0xEC81u, 0x2C40u, 0xE401u, 0x24C0u, 0x2580u, 0xE541u, 0x2700u,
    0xE7C1u, 0xE681u, 0x2640u, 0x2200u, 0xE2C1u, 0xE381u, 0x2340u, 0xE101u, 0x21C0u, 0x2080u, 0xE041u, 0xA001u, 0x60C0u,
    0x6180u, 0xA141u, 0x6300u, 0xA3C1u, 0xA281u, 0x6240u, 0x6600u, 0xA6C1u, 0xA781u, 0x6740u, 0xA501u, 0x65C0u, 0x6480u,
    0xA441u, 0x6C00u, 0xACC1u, 0xAD81u, 0x6D40u, 0xAF01u, 0x6FC0u, 0x6E80u, 0xAE41u, 0xAA01u, 0x6AC0u, 0x6B80u, 0xAB41u,
    0x6900u, 0xA9C1u, 0xA881u, 0x6840u, 0x7800u, 0xB8C1u, 0xB981u, 0x7940u, 0xBB01u, 0x7BC0u, 0x7A80u, 0xBA41u, 0xBE01u,
    0x7EC0u, 0x7F80u, 0xBF41u, 0x7D00u, 0xBDC1u, 0xBC81u, 0x7C40u, 0xB401u, 0x74C0u, 0x7580u, 0xB541u, 0x7700u, 0xB7C1u,
    0xB681u, 0x7640u, 0x7200u, 0xB2C1u, 0xB381u, 0x7340u, 0xB101u, 0x71C0u, 0x7080u, 0xB041u, 0x5000u, 0x90C1u, 0x9181u,
    0x5140u, 0x9301u, 0x53C0u, 0x5280u, 0x9241u, 0x9601u, 0x56C0u, 0x5780u, 0x9741u, 0x5500u, 0x95C1u, 0x9481u, 0x5440u,
    0x9C01u, 0x5CC0u, 0x5D80u, 0x9D41u, 0x5F00u, 0x9FC1u, 0x9E81u, 0x5E40u, 0x5A00u, 0x9AC1u, 0x9B81u, 0x5B40u, 0x9901u,
    0x59C0u, 0x5880u, 0x9841u, 0x8801u, 0x48C0u, 0x4980u, 0x8941u, 0x4B00u, 0x8BC1u, 0x8A81u, 0x4A40u, 0x4E00u, 0x8EC1u,
    0x8F81u, 0x4F40u, 0x8D01u, 0x4DC0u, 0x4C80u, 0x8C41u, 0x4400u, 0x84C1u, 0x8581u, 0x4540u, 0x8701u, 0x47C0u, 0x4680u,
    0x8641u, 0x8201u, 0x42C0u, 0x4380u, 0x8341u, 0x4100u, 0x81C1u, 0x8081u, 0x4040u
};

#define TIBMS_UTILS_STOP_SEC_CONST_16BIT
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 *  Local Function Prototypes
 *********************************************************************************************************************/

/*********************************************************************************************************************
 *  Local Inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 *  Local Functions Definition
 *********************************************************************************************************************/

/*********************************************************************************************************************
 *  External Functions Definition
 *********************************************************************************************************************/

#define TIBMS_UTILS_START_SEC_CODE
#include "Cdd_MemMap.h"

/**********************************************************************************************************************
 *  FUNC(uint16, tibms_utils_CODE) CalculateCRC16(const uint8 *pData, uint16 wDataLen)
 *********************************************************************************************************************/
/*! \brief          Function to calculate the CRC
 *
 *  This functions returns the last error stored incase of any issues. Memory dump of array can be take to analyze
 *  further issues
 *
 *  \param[in]      void
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre            None
 *  \post           None
 *  \return         uint16
 *  \retval         CRC
 *  \trace
 *********************************************************************************************************************/

FUNC(uint16, tibms_utils_CODE) CalculateCRC16(const uint8 pData[], uint16 wDataLen)
{
    uint16 wIndex = 0u;
    uint16 wCrc16 = 0xFFFFu;

    for (wIndex = 0u; wIndex < wDataLen; wIndex++)
    {
        wCrc16 ^= (uint16) (pData[wIndex]);
        wCrc16 = zCrc16Lkup[wCrc16 & ((uint16) 0x00FFu)] ^ (wCrc16 >> (uint16) (TIBMS_8BIT_OFFSET));
    }

    return wCrc16;
}

/**********************************************************************************************************************
* void ctPeQueueLock(ctPeQueueType *pctPeQueue)
 *********************************************************************************************************************/
/*! \brief          lock function for protecting the critical sessions
 *
 *  \param[in]      pctPeQueue: Pointer QUE Contenxt
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         returns void
 *  \retval         void
 *
 *  \trace
 *********************************************************************************************************************/

FUNC(void, TIBMS_UTILS) ctPeQueueLock(const ctPeQueueType *pctPeQueue)
{
    if(NULL != pctPeQueue)
    {
        if(NULL != pctPeQueue->pBswCfg)
        {
            pctPeQueue->pBswCfg->resource_req(pctPeQueue->qResNumber, GET_RESOURCE);
        }
    }
}

/**********************************************************************************************************************
* void ctPeQueueUnlock(ctPeQueueType *pctPeQueue)
 *********************************************************************************************************************/
/*! \brief         Unlock function for protecting the critical sessions
 *
 *  \param[in]      pctPeQueue: Pointer QUE Contenxt
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         returns void
 *  \retval         void
 *
 *  \trace
 *********************************************************************************************************************/

FUNC(void, TIBMS_UTILS) ctPeQueueUnlock(const ctPeQueueType *pctPeQueue)
{
    if(NULL != pctPeQueue)
    {
        if(NULL != pctPeQueue->pBswCfg)
        {
            pctPeQueue->pBswCfg->resource_req(pctPeQueue->qResNumber, RELEASE_RESOURCE);
        }
    }
}

/**********************************************************************************************************************
* FUNC(uint8, TIBMS_UTILS) ctPeQueueReset(ctPeQueueType *pctPeQueue)
 *********************************************************************************************************************/
/*! \brief          Resets the QUE
 *
 *  \param[in]      pctPeQueue: Pointer QUE Contenxt
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         returns the status of QUEU
 *  \retval         QUEUE_E_NOT_OK or QUEUE_E_OK
 *
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, TIBMS_UTILS) ctPeQueueReset(ctPeQueueType *pctPeQueue)
{
    uint8 ret = (uint8)QUEUE_E_NOT_OK;

    if(NULL != pctPeQueue)
    {
        ctPeQueueLock(pctPeQueue);

        pctPeQueue->wWriteIdx = 0;
        pctPeQueue->wReadIdx = 0;
        pctPeQueue->nElements = 0;

        ctPeQueueUnlock(pctPeQueue);

        ret = (uint8)QUEUE_E_OK;
    }

    return ret;
}

/**********************************************************************************************************************
* FUNC(uint16, TIBMS_UTILS) ctPeQueueGetSize(ctPeQueueType *pctPeQueue)
 *********************************************************************************************************************/
/*! \brief          Returns the size of the queue
 *
 *  \param[in]      pctPeQueue: Pointer QUE Contenxt
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         returns the status of QUEU
 *  \retval         number
 *
 *  \trace
 *********************************************************************************************************************/

FUNC(uint16, TIBMS_UTILS) ctPeQueueGetSize(const ctPeQueueType *pctPeQueue)
{
    uint16 ret;

    ctPeQueueLock(pctPeQueue);
    ret = pctPeQueue->wSizeQueue;
    ctPeQueueUnlock(pctPeQueue);

    return ret;
}

/**********************************************************************************************************************
* FUNC(uint16, TIBMS_UTILS) ctPeQueueIsEmpty(ctPeQueueType *pctPeQueue)
 *********************************************************************************************************************/
/*! \brief          Get the number of elements in the que
 *
 *  \param[in]      pctPeQueue: Pointer QUE Contenxt
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         returns the status of QUEU
 *  \retval         number
 *
 *  \trace
 *********************************************************************************************************************/

uint16 ctPeQueueGetNOfElements(const ctPeQueueType *pctPeQueue)
{
    uint16 ret;

    ctPeQueueLock(pctPeQueue);
    ret = pctPeQueue->nElements;
    ctPeQueueUnlock(pctPeQueue);

    return ret;
}

/**********************************************************************************************************************
* FUNC(uint16, TIBMS_UTILS) ctPeQueueIsEmpty(ctPeQueueType *pctPeQueue)
 *********************************************************************************************************************/
/*! \brief          Check if queue is empty
 *
 *  \param[in]      pctPeQueue: Pointer QUE Contenxt
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         returns the status of QUEU
 *  \retval         TRUE or FALSE
 *
 *  \trace
 *********************************************************************************************************************/

FUNC(uint16, TIBMS_UTILS) ctPeQueueIsEmpty(const ctPeQueueType *pctPeQueue)
{
    uint16 ret;

    ctPeQueueLock(pctPeQueue);
    ret = (pctPeQueue->wReadIdx == pctPeQueue->wWriteIdx);
    ctPeQueueUnlock(pctPeQueue);

    return ret;
}

/**********************************************************************************************************************
* FUNC(void*, TIBMS_UTILS) ctPeQueueRemoveData(ctPeQueueType *pctPeQueue)
 *********************************************************************************************************************/
/*! \brief          Get the data from the que and remove the data
 *
 *  \param[in]      pctPeQueue: Pointer QUE Contenxt
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         returns the data in the queue
 *  \retval         Data In the QUE
 *                  NULL
 *  \trace
 *********************************************************************************************************************/

FUNC(void*, TIBMS_UTILS) ctPeQueueRemoveData(ctPeQueueType *pctPeQueue)
{
    void *pData = NULL;

    if(NULL != pctPeQueue)
    {
        ctPeQueueLock(pctPeQueue);
        if(pctPeQueue->wReadIdx != pctPeQueue->wWriteIdx)
        {
            pData = pctPeQueue->pQueue[pctPeQueue->wReadIdx];
            pctPeQueue->wReadIdx++;
            if(pctPeQueue->wReadIdx > pctPeQueue->wSizeQueue)
            {
                pctPeQueue->wReadIdx = 0;
            }

            /* Increment counter of free elements */
            pctPeQueue->nElements--;
        }
		ctPeQueueUnlock(pctPeQueue);
    }

    return pData;
}

/**********************************************************************************************************************
* FUNC(void*, TIBMS_UTILS) ctPeQueueGetData(ctPeQueueType *pctPeQueue)
 *********************************************************************************************************************/
/*! \brief          Get the data from queue without incrementing head or tail
 *
 *  \param[in]      pctPeQueue: Pointer QUE Contenxt
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         returns the data in the queue, but not increament the pointers
 *  \retval         Data In the QUE
 *                  NULL
 *  \trace
 *********************************************************************************************************************/

FUNC(void*, TIBMS_UTILS) ctPeQueueGetData(const ctPeQueueType *pctPeQueue)
{
    void *pData = NULL;

    if(NULL != pctPeQueue)
    {
        ctPeQueueLock(pctPeQueue);
        if(pctPeQueue->wReadIdx != pctPeQueue->wWriteIdx)
        {
            pData = pctPeQueue->pQueue[pctPeQueue->wReadIdx];
        }
		ctPeQueueUnlock(pctPeQueue);

    }

    return pData;
}

/**********************************************************************************************************************
* FUNC(uint8, TIBMS_UTILS) ctPeQueueAddData(ctPeQueueType *pctPeQueue, void *pData)
 *********************************************************************************************************************/
/*! \brief          Adding the data to the QUEUE
 *
 *  \param[in]      pctPeQueue: Pointer QUE Contenxt
 *  \param[in]      pData: Data adding to the QUEUE
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         returns Status if QUE FULL -> QUEUE_E_NOT_OK
 *  \retval         QUEUE_E_OK
 *                  QUEUE_E_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, TIBMS_UTILS) ctPeQueueAddData(ctPeQueueType *pctPeQueue, void *pData)
{
    uint8 ret = (uint8)QUEUE_E_OK;

    if(NULL != pctPeQueue)
    {
        ctPeQueueLock(pctPeQueue);
        if((pctPeQueue->nElements+1u) > pctPeQueue->wSizeQueue)
        {
            ret = (uint8)QUEUE_E_NOT_OK;
        }
        else
        {
            pctPeQueue->pQueue[pctPeQueue->wWriteIdx] = pData;
            pctPeQueue->wWriteIdx++;
            if(pctPeQueue->wWriteIdx > pctPeQueue->wSizeQueue)
            {
                pctPeQueue->wWriteIdx = 0;
            }
            pctPeQueue->nElements++;
        }
        ctPeQueueUnlock(pctPeQueue);
    }

    return ret;
}

/**********************************************************************************************************************
 *  FUNC(uint8, TIBMS_UTILS) ctPeQueueCreate(ctPeQueueType *pctPeQueue, uint32 qRes, const Basic_ConfigType *pBswCfgIn,
 *                                           void *pQueue, uint16 wElementSize, uint16 wSizeQueue)
 *********************************************************************************************************************/
/*! \brief          Creating the Pointer QUEUE
 *
 *  \param[in]      pctPeQueue: Pointer QUE Contenxt
 *  \param[in]      qResNumberCfg: Resource used for Protection
 *  \param[in]      pBswCfgIn: Basic Software Configuration
 *  \param[in]      pQueue: Data for the QUEUE
 *  \param[in]      wElementSize: Size of element
 *  \param[in]      wSizeQueue: Total QUEUE Size
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *  \return         returns the status of Que Creation, return not ok if invalid pointers are passed
 *  \retval         QUEUE_E_OK
 *                  QUEUE_E_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, TIBMS_UTILS) ctPeQueueCreate(ctPeQueueType *pctPeQueue, uint32 qRes, const Basic_ConfigType *pBswCfgIn,
                                         void *pQueue, uint16 wElementSize, uint16 wSizeQueue)
{
    uint8 ret = (uint8)QUEUE_E_NOT_OK;

    if((NULL != pctPeQueue) && (NULL != pQueue))
    {
        /* store queue address */
        pctPeQueue->pQueue = pQueue;

        /* Pointer to last element+1 : for modulo operation! */
        pctPeQueue->wWriteIdx = 0;
        pctPeQueue->wReadIdx = 0;
        pctPeQueue->wElementSize = wElementSize;
        pctPeQueue->wSizeQueue = wSizeQueue;

        pctPeQueue->pBswCfg = pBswCfgIn;
        pctPeQueue->qResNumber = qRes;

        /* clear ctPeQueue */
        pctPeQueue->nElements = 0;

        ret = (uint8)QUEUE_E_OK;
    }

    return ret;
}

#define TIBMS_UTILS_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * End of File: tibms_utils.c
 *********************************************************************************************************************/
