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
 *  File:       tibms_utils.h
 *  Project:    TIBMS
 *  Module:     UTILS
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  TI BMS Utils
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

#ifndef TIBMS_UTILS_H
#define TIBMS_UTILS_H

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

#define QUEUE_INVALID_READ_HANDLE           (-1)
#define EMEM_E_OK                           (0u)
#define EMEM_E_NOT_OK                       (1u)

#define EMEM_GUARD                          (0xBE)

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef enum ctPeQueueReturnType_Tag
{
	QUEUE_E_OK = 0u,
	QUEUE_E_NOT_OK,
	QUEUE_E_TRUE,
	QUEUE_E_NO_INIT,
	QUEUE_E_FULL,
	QUEUE_E_NO_DATA,
	QUEUE_E_LOST_DATA,
	QUEUE_E_NO_CONTAINS,
	QUEUE_E_ALREADY_INIT,
	QUEUE_E_FALSE,
	QUEUE_E_NULL
}
ctPeQueueReturnType;

typedef struct ctPeQueueType_Tag
{
    void** pQueue;                                   /* pointer to queue containing pointers to data*/
    const Basic_ConfigType *pBswCfg;                 /* Resource to protect the access to the queue */

    uint32 qResNumber;                               /* qResNumber */

    uint16 wElementSize;                             /* size of one element of the queue */
    uint16 wSizeQueue;                               /* size of the queue */

    uint16 wWriteIdx;                                /* write index */
    uint16 wReadIdx;                                 /* read  index */

    uint16 nElements;                                /* number of contained elements */
    uint16 nMaxUsage;
}
ctPeQueueType;

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

#define TIBMS_UTILS_START_SEC_CODE
#include "Cdd_MemMap.h"

FUNC(uint16, tibms_utils_CODE) CalculateCRC16(const uint8 pData[], uint16 wDataLen);
FUNC(uint8, TIBMS_UTILS) ctPeQueueAddData(ctPeQueueType* pctPeQueue, void *pData);
FUNC(void*, TIBMS_UTILS) ctPeQueueGetData(const ctPeQueueType* pctPeQueue);
FUNC(void*, TIBMS_UTILS) ctPeQueueRemoveData(ctPeQueueType* pctPeQueue);
FUNC(uint16, TIBMS_UTILS) ctPeQueueGetSize(const ctPeQueueType* pctPeQueue);
FUNC(uint16, TIBMS_UTILS) ctPeQueueGetNOfElements(const ctPeQueueType* pctPeQueue );
FUNC(uint16, TIBMS_UTILS) ctPeQueueIsEmpty(const ctPeQueueType* pctPeQueue );
FUNC(void, TIBMS_UTILS) ctPeQueueLock(const ctPeQueueType* pctPeQueue);
FUNC(void, TIBMS_UTILS) ctPeQueueUnlock(const ctPeQueueType* pctPeQueue);
FUNC(uint8, TIBMS_UTILS) ctPeQueueReset(ctPeQueueType* pctPeQueue);
FUNC(uint8, TIBMS_UTILS) ctPeQueueCreate(ctPeQueueType *pctPeQueue, uint32 qRes, const Basic_ConfigType *pBswCfgIn,
                                         void *pQueue, uint16 wElementSize, uint16 wSizeQueue);

#define TIBMS_UTILS_STOP_SEC_CODE
#include "Cdd_MemMap.h"

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

#endif /*TIBMS_UTILS_H*/

/*********************************************************************************************************************
 * End of File: tibms_utils.h
 *********************************************************************************************************************/
