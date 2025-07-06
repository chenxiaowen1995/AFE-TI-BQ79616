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
 *  File:       tibms_comif.c
 *  Project:    TIBMS
 *  Module:     COMIF
 *  Generator:  Code generation tool (ifany)
 *
 *  Description:  Upper layer wrapper for Communication interface, All the communication requests go through this
 *                interface
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05July2022   SEM                  0000000000000    Initial version
 * 01.01.00       04Aug2023    SEM                  0000000000000    Configuration updates
 *
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Standard Header Files
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Other Header Files
 *********************************************************************************************************************/

#include "tibms_api.h"
#include "tibms_comif.h"
#include "tibms_utils.h"

/*********************************************************************************************************************
 * Version Check (if required)
 *********************************************************************************************************************/

/**	Major Software Config C Version number */
#define COMIF_SW_C_MAJOR_VERSION                            (0x01u)

/**	Minor Software Config C Version number */
#define COMIF_SW_C_MINOR_VERSION                            (0x01u)

/** Software Patch Config C Version number */
#define COMIF_SW_C_PATCH_VERSION                            (0x00u)

#if ((COMIF_SW_C_MAJOR_VERSION != TIBMS_CFG_MAJOR_VERSION) || \
     (COMIF_SW_C_MINOR_VERSION != TIBMS_CFG_MINOR_VERSION) || \
	 (COMIF_SW_C_PATCH_VERSION != TIBMS_CFG_PATCH_VERSION))
#error "tibms: Config version numbers of tibms_comif.c and tibms_cfg.h are inconsistent!"
#endif

#if ((COMIF_SW_MAJOR_VERSION != COMIF_SW_C_MAJOR_VERSION) || \
     (COMIF_SW_MINOR_VERSION != COMIF_SW_C_MINOR_VERSION) || \
	 (COMIF_SW_PATCH_VERSION != COMIF_SW_C_PATCH_VERSION))
#error "tibms: Config version numbers of tibms_comif.c and tibms_comif.h are inconsistent!"
#endif

/*********************************************************************************************************************
 * Local Preprocessor #define Constants
 *********************************************************************************************************************/

#define COMIF_GUARD_START                                   (0xCCA1AB1AU)
#define COMIF_GUARD_END                                     (0xCAFECEEAU)
#define COMIF_INIT                                          (0xCAU)

#define COMIF_INTEGRITY_CHECK(pComifMgr)                    ((NULL != pComifMgr) && \
                                                             (COMIF_GUARD_START == pComifMgr->qGuardStart) && \
                                                             (COMIF_GUARD_END == pComifMgr->qGuardEnd) && \
                                                             (NULL != pComifMgr->pComifOps) && \
                                                             (COMIF_INIT == pComifMgr->uInit))

/*********************************************************************************************************************
 * Local Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Type Declarations
 *********************************************************************************************************************/

struct Comif_ManagerType_Tag
{
    uint32 qGuardStart;

    void *pComifHandle;
    const Comif_OperationType *pComifOps;
    const ServiceCfgType *pCommCfg;

    uint8 uInit;
    uint8 uComifIface;
    uint8 uPerip;
    uint8 uRsvd;

    uint32 qGuardEnd;
};

/*********************************************************************************************************************
 * Exported Object Definitions
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Local Object Definitions
 *********************************************************************************************************************/

#define TIBMS_COMIF_START_SEC_VAR_NOINIT_UNSPECIFIED
#include "Cdd_MemMap.h"

static Comif_ManagerType zComifManager[TIBMS_NO_OF_COMIF_IFACES];

#if(TIMBS_COMIF_EMEM_ENABLE == STD_ON)
static Emem_ManagerType zComifEmemMgr;
#endif

#define TIBMS_COMIF_STOP_SEC_VAR_NOINIT_UNSPECIFIED
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

/*********************************************************************************************************************
 *  External Functions Definition
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_BroadcastRead(Comif_ManagerType*,uint8,uint16,
                                                      const uint8*,const ServiceCfgType *, uint8 )
 *********************************************************************************************************************/
/*! \brief          Broadcast read request to Communication interface
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *  \param[in]      uDevId: Device ID for the request
 *  \param[in]      qRegAddr: Register address for the write
 *  \param[in]      pData: Data for the register read
 *  \param[in]      pServiceCfg: Data context for the register read
 *  \param[in]      uReadLen: Length of register data read
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the broadcast read status
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

#define TIBMS_COMIF_START_SEC_CODE
#include "Cdd_MemMap.h"

FUNC(uint8, tibms_comif_CODE) Comif_BroadcastRead(const Comif_ManagerType *pComifMgr, uint8 uDevId, uint32 qRegAddr,
                                                   const ServiceCfgType *pServiceCfg, uint8 uReadLen)
{
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;
    Comif_ReqCmdType zReqcmd;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        zReqcmd.uReqType = (uint8)REQTYPE_BROADCAST_R;
        zReqcmd.uGroupExclude = 0u;
        zReqcmd.wReqCmdSiz = 1u;
        zReqcmd.uDevId = uDevId;
        zReqcmd.wRegAdr = COMIF_GET_REG(qRegAddr);
        zReqcmd.pData = &uReadLen;
        zReqcmd.pServiceCfg = pServiceCfg;
        zReqcmd.uCmdInfo = COMIF_GET_INFO(qRegAddr);

        uRet = pComifMgr->pComifOps->ioctl(pComifMgr->pComifHandle, COMIF_CMD_ALLREAD, (void *) &zReqcmd);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_BroadcastWrite(Comif_ManagerType *, uint8 , uint16 ,const uint8 *, uint8 )
 *********************************************************************************************************************/
/*! \brief          Broadcast read request to Communication interface
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *  \param[in]      uReverseEn: Device ID for the request
 *  \param[in]      qRegAddr: Register address for the write
 *  \param[in]      pData: Data for the register read
 *  \param[in]      uDataLen: Length of register data read
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the broadcast write status
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_BroadcastWrite(const Comif_ManagerType *pComifMgr, uint8 uReverseEn, uint32 qRegAddr,
                                                   const uint8 *pData, uint8 uDataLen)
{
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;
    Comif_ReqCmdType zReqcmd;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        zReqcmd.uReqType = ((uReverseEn == 0u) ? REQTYPE_BROADCAST_W : REQTYPE_BROADCAST_W_REV);
        zReqcmd.uGroupExclude = 0u;
        zReqcmd.wReqCmdSiz = uDataLen;
        zReqcmd.uDevId = 0u;
        zReqcmd.wRegAdr = COMIF_GET_REG(qRegAddr);
        zReqcmd.pData = pData;
        zReqcmd.pServiceCfg = NULL;
        zReqcmd.uCmdInfo = COMIF_GET_INFO(qRegAddr);

        uRet = pComifMgr->pComifOps->ioctl(pComifMgr->pComifHandle, COMIF_CMD_ALLWRITE, (void *) &zReqcmd);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_StackRead(Comif_ManagerType *, uint8, uint16,const ServiceCfgType *, uint8)
 *********************************************************************************************************************/
/*! \brief          Stack Write request to Communication interface
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *  \param[in]      uDevId: Device ID for the request
 *  \param[in]      qRegAddr: Register address for the write
 *  \param[in]      pServiceCfg: Data for the register read
 *  \param[in]      uReadLen: Length of register data read
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the stack read status
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_StackRead(const Comif_ManagerType *pComifMgr, uint8 uDevId, uint32 qRegAddr,
                                              const ServiceCfgType *pServiceCfg, uint8 uReadLen)
{
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;
    Comif_ReqCmdType zReqcmd;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        zReqcmd.uReqType = (uint8)REQTYPE_STACK_R;
        zReqcmd.uGroupExclude = 0u;
        zReqcmd.wReqCmdSiz = 1u;
        zReqcmd.uDevId = uDevId;
        zReqcmd.wRegAdr = COMIF_GET_REG(qRegAddr);
        zReqcmd.pData = &uReadLen;
        zReqcmd.pServiceCfg = pServiceCfg;
        zReqcmd.uCmdInfo = COMIF_GET_INFO(qRegAddr);

        uRet = pComifMgr->pComifOps->ioctl(pComifMgr->pComifHandle, COMIF_CMD_STKREAD, (void *) &zReqcmd);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_StackWrite(Comif_ManagerType *, uint8 , uint16 ,const uint8 *, uint8 )
 *********************************************************************************************************************/
/*! \brief          Stack Write request to Communication interface
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *  \param[in]      uGroupEn: Enabling the group
 *  \param[in]      qRegAddr: Register address for the write
 *  \param[in]      pData: Data for the register write
 *  \param[in]      uReadLen: Length of register data read
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the stack write status
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_StackWrite(const Comif_ManagerType *pComifMgr, uint8 uGroupEn, uint32 qRegAddr,
                                               const uint8 *pData, uint8 uDataLen)
{
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;
    Comif_ReqCmdType zReqcmd;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        zReqcmd.uReqType = (uint8)REQTYPE_STACK_W;
        zReqcmd.uGroupExclude = !(uGroupEn == 0u);
        zReqcmd.wReqCmdSiz = uDataLen;
        zReqcmd.uDevId = 0;
        zReqcmd.wRegAdr = COMIF_GET_REG(qRegAddr);
        zReqcmd.pData = pData;
        zReqcmd.pServiceCfg = NULL;
        zReqcmd.uCmdInfo = COMIF_GET_INFO(qRegAddr);

        uRet = pComifMgr->pComifOps->ioctl(pComifMgr->pComifHandle, COMIF_CMD_STKWRITE, (void *) &zReqcmd);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_SingleRead(Comif_ManagerType *, uint8, uint16,const ServiceCfgType *, uint8)
 *********************************************************************************************************************/
/*! \brief          Single Read request to Communication interface
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *  \param[in]      uDevId: Device ID for the data write
 *  \param[in]      qRegAddr: Register address for the write
 *  \param[in]      pServiceCfg: Data context for the register read
 *  \param[in]      uReadLen: Length of register data read
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the single read status
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_SingleRead(const Comif_ManagerType *pComifMgr, uint8 uDevId, uint32 qRegAddr,
											   const ServiceCfgType *pServiceCfg, uint8 uReadLen)
{
    Comif_ReqCmdType zReqcmd;
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
    	zReqcmd.uReqType = (uint8)REQTYPE_SINGLE_R;
        zReqcmd.uGroupExclude = 0u;
        zReqcmd.wReqCmdSiz = 1u;
        zReqcmd.uDevId = uDevId;
        zReqcmd.wRegAdr = COMIF_GET_REG(qRegAddr);
        zReqcmd.pData = &uReadLen;
        zReqcmd.pServiceCfg = pServiceCfg;
        zReqcmd.uCmdInfo = COMIF_GET_INFO(qRegAddr);

        uRet = pComifMgr->pComifOps->ioctl(pComifMgr->pComifHandle, COMIF_CMD_SGLREAD, (void *) &zReqcmd);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_SingleWrite(Comif_ManagerType *, uint8 , uint16 ,const uint8 *, uint8 )
 *********************************************************************************************************************/
/*! \brief          Single write request to Communication interface
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *  \param[in]      uDevId: Device ID for the data write
 *  \param[in]      qRegAddr: Register address for the write
 *  \param[in]      pData: Data for the register write
 *  \param[in]      uDataLen: Length of register data write
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         Returns the single write status
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_SingleWrite(const Comif_ManagerType *pComifMgr, uint8 uDevId, uint32 qRegAddr,
                                                const uint8 *pData, uint8 uDataLen)
{
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;
    Comif_ReqCmdType zReqcmd;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        zReqcmd.uReqType = (uint8)REQTYPE_SINGLE_W;
        zReqcmd.uGroupExclude = 0u;
        zReqcmd.wReqCmdSiz = uDataLen;
        zReqcmd.uDevId = uDevId;
        zReqcmd.wRegAdr = COMIF_GET_REG(qRegAddr);
        zReqcmd.pData = pData;
        zReqcmd.pServiceCfg = NULL;
        zReqcmd.uCmdInfo = COMIF_GET_INFO(qRegAddr);

        uRet = pComifMgr->pComifOps->ioctl(pComifMgr->pComifHandle, COMIF_CMD_SGLWRITE, (void *) &zReqcmd);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_GenericRead(Comif_ManagerType *, uint8, uint16,const ServiceCfgType *, uint8)
 *********************************************************************************************************************/
/*! \brief          Generic Read request to Communication interface
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *  \param[in]      uDevId: Device ID for the data write
 *  \param[in]      qRegAddr: Register address for the write
 *  \param[in]      pServiceCfg: Data context for the register read
 *  \param[in]      uReadLen: Length of register data read
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the single read status
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_GenericRead(const Comif_ManagerType *pComifMgr, uint8 uDevId, uint32 qData,
                                               const ServiceCfgType *pServiceCfg, const uint8 *pData, uint8 uReadLen)
{
    Comif_ReqCmdType zReqcmd;
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
    	zReqcmd.uReqType = (uint8)REQTYPE_GENERIC_R;
        zReqcmd.uGroupExclude = 0u;
        zReqcmd.wReqCmdSiz = uReadLen;
        zReqcmd.uDevId = uDevId;
        zReqcmd.wRegAdr = COMIF_GET_REG(qData);
        zReqcmd.pData = pData;
        zReqcmd.pServiceCfg = pServiceCfg;
        zReqcmd.uCmdInfo = (uint8)COMIF_CMD_PRIO_CTRL;

        uRet = pComifMgr->pComifOps->ioctl(pComifMgr->pComifHandle, COMIF_CMD_GNRCREAD, (void *) &zReqcmd);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_GenericWrite(Comif_ManagerType *, uint8 , uint16 ,const uint8 *, uint8 )
 *********************************************************************************************************************/
/*! \brief          Generic write request to Communication interface
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *  \param[in]      uDevId: Device ID for the data write
 *  \param[in]      qData: Register address for the write
 *  \param[in]      pData: Data for the register write
 *  \param[in]      uDataLen: Length of register data write
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         Returns the single write status
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_GenericWrite(const Comif_ManagerType *pComifMgr, uint8 uDevId, uint32 qData,
                                                 const uint8 *pData, uint8 uDataLen)
{
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;
    Comif_ReqCmdType zReqcmd;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        zReqcmd.uReqType = (uint8)REQTYPE_GENERIC_W;
        zReqcmd.uGroupExclude = 0u;
        zReqcmd.wReqCmdSiz = uDataLen;
        zReqcmd.uDevId = uDevId;
        zReqcmd.wRegAdr = COMIF_GET_REG(qData);
        zReqcmd.pData = pData;
        zReqcmd.pServiceCfg = NULL;
        zReqcmd.uCmdInfo = (uint8)COMIF_CMD_PRIO_CTRL;

        uRet = pComifMgr->pComifOps->ioctl(pComifMgr->pComifHandle, COMIF_CMD_GNRCWRITE, (void *) &zReqcmd);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_ComClearRequest(Comif_ManagerType *pComifMgr)
 *********************************************************************************************************************/
/*! \brief          Request for the Sending a COM CLEAR Command.
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         Returns the delay request status
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_ComClearRequest(const Comif_ManagerType *pComifMgr)
{
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        uRet = pComifMgr->pComifOps->ioctl(pComifMgr->pComifHandle, COMIF_CMD_COMCLEAR, NULL);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_SingleWriteError(Comif_ManagerType *, uint8 , uint16 ,const uint8 *, uint8 )
 *********************************************************************************************************************/
/*! \brief          Single write Error request to Communication interface
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *  \param[in]      uDevId: Device ID for the data write
 *  \param[in]      qRegAddr: Register address for the write
 *  \param[in]      pData: Data for the register write
 *  \param[in]      uDataLen: Length of register data write
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         Returns the single write status
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_SingleWriteError(const Comif_ManagerType *pComifMgr, uint8 uDevId, uint32 qRegAddr,
                                                     const uint8 *pData, uint8 uDataLen)
{
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;
    Comif_ReqCmdType zReqcmd;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        zReqcmd.uReqType = (uint8)REQTYPE_CUSTOM_ERROR;
        zReqcmd.uGroupExclude = 0u;
        zReqcmd.wReqCmdSiz = uDataLen;
        zReqcmd.uDevId = uDevId;
        zReqcmd.wRegAdr = COMIF_GET_REG(qRegAddr);
        zReqcmd.pData = pData;
        zReqcmd.pServiceCfg = NULL;
        zReqcmd.uCmdInfo = COMIF_GET_INFO(qRegAddr);

        uRet = pComifMgr->pComifOps->ioctl(pComifMgr->pComifHandle, COMIF_CMD_ERROR, (void *) &zReqcmd);
    }

    return uRet;
}


/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_SetDelay(Comif_ManagerType *pComifMgr, uint32 qDelay)
 *********************************************************************************************************************/
/*! \brief          Request for the delay between executing the next command.
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *  \param[in]      qDelay: Delay after the wakeup to the bridge device
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         Returns the delay request status
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_SetDelay(const Comif_ManagerType *pComifMgr, uint32 qDelay)
{
    Comif_ReqCmdType zReqcmd;
    uint8 uData[COMIF_DELAY_CMD_LEN];
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        uData[0u] = (uint8)(qDelay >> 8u) & TIBMS_8BIT_MASK;
        uData[1u] = (uint8)(qDelay >> 0u) & TIBMS_8BIT_MASK;

        zReqcmd.uReqType = (uint8)REQTYPE_CUSTOM_DELAY;
        zReqcmd.uGroupExclude = 0u;
        zReqcmd.wReqCmdSiz = COMIF_DELAY_CMD_LEN;
        zReqcmd.uDevId = 0u;
        zReqcmd.wRegAdr = 0u;
        zReqcmd.pData = uData;
        zReqcmd.pServiceCfg = NULL;
        zReqcmd.uCmdInfo = COMIF_GET_INFO(qDelay);

        uRet = pComifMgr->pComifOps->ioctl(pComifMgr->pComifHandle, COMIF_CMD_DELAY, (void *) &zReqcmd);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comiface_WakePing(Comif_ManagerType *pComifMgr, uint32 qDelay)
 *********************************************************************************************************************/
/*! \brief          Wakeup the base device and stack
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *  \param[in]      qDelay: Delay after the wakeup to the bridge device
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of the request
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comiface_WakePing(const Comif_ManagerType *pComifMgr, uint32 qDelay)
{
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        uRet = pComifMgr->pComifOps->ioctl(pComifMgr->pComifHandle, COMIF_CMD_WAKEUP, (void *) &qDelay);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comiface_HandleRequest(const Comif_ManagerType *pComifMgr, uint8 uCmd,
                                                     const ServiceCfgType *pSvcCfg, void *pData)
 *********************************************************************************************************************/
/*! \brief          Communication Control handler
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *  \param[in]      uCmd: Command Request
 *  \param[in]      pServiceCfg: Service context for the register read
 *  \param[in]      pData: Generic Data
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of the request
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comiface_HandleRequest(const Comif_ManagerType *pComifMgr, uint8 uCmd,
                                                     uint8 uNodeId, void *pData)
{
    const ServiceCfgType *pComSvcCfg = NULL;
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        uRet = (uint8)COMIF_CMD_INVALID_REQCMD;
        if(uCmd < (uint8)COMM_SERVICE_END)
        {
            uRet = (uint8)COMIF_CMD_INVALID_SVCCFG;
            pComSvcCfg = &pComifMgr->pCommCfg[uCmd];
            if((NULL != pComSvcCfg) && (NULL != pComSvcCfg->pSvcStat))
            {
                if(uCmd == pComSvcCfg->uSubId)
                {
                    pComSvcCfg->pSvcStat->uRespStatus = (uint8)RESP_BUSY;
                    uRet = Comif_GenericRead(pComifMgr, uNodeId, uCmd, pComSvcCfg, pData, 0u);
                }
            }
        }
        uRet = Comif_SetErrorDetails(COMIF_SVCID_HANDLE_REQ, uRet, pComifMgr->uComifIface, 0u);
    }

    return uRet;
}

/**********************************************************************************************************************
 * uint8 Comiface_DataProc(const Comif_ManagerType *pComifMgr, const ServiceCfgType *pCommCfg, const uint8 *pRxData)
 *********************************************************************************************************************/
/*! \brief          Communication Startup Control handler
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *  \param[in]      pServiceCfg: Service context for the register read
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of the request
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comiface_DataProc(const Comif_ManagerType *pComifMgr, const ServiceCfgType *pCommCfg,
                                                const uint8 *pRxData)
{
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        // TBD: Add any data processing if required
        if(NULL != pCommCfg->cbApplSvc)
        {
            pCommCfg->cbApplSvc(pCommCfg->uIface, pCommCfg->uServiceId, pCommCfg->uSubId,
                                pCommCfg->pSvcStat->uRespStatus, (void *) pRxData);
        }
        uRet = (uint8)COMIF_OK;
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_HandleTransfer(Comif_ManagerType *pComifMgr)
 *********************************************************************************************************************/
/*! \brief          Handling the transfer
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *
 *  \reentrant      TRUE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the status of the handling request
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_HandleTransfer(const Comif_ManagerType *pComifMgr)
{
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        uRet = pComifMgr->pComifOps->handle_tx(pComifMgr->pComifHandle);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_HandleRecieve(Comif_ManagerType *pComifMgr)
 *********************************************************************************************************************/
/*! \brief          Recieve Data process
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the process state
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_HandleRecieve(const Comif_ManagerType *pComifMgr)
{
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        uRet = pComifMgr->pComifOps->handle_rx(pComifMgr->pComifHandle);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_HandleNotify(Comif_ManagerType *pComifMgr, uint8 uType)
 *********************************************************************************************************************/
/*! \brief          Communication Notification for the status from external
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *  \param[in]      uType: Notification Type
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the notification state
 *  \retval         COMIF_OK
 *                  COMIF_NOT_OK
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_HandleNotify(const Comif_ManagerType *pComifMgr, uint8 uType)
{
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        uRet = pComifMgr->pComifOps->notification(pComifMgr->pComifHandle, uType);
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_SetErrorDetails(uint8 uSvcId, uint8 uErrorId, uint32 qParam1, uint32 qParam2)
 *********************************************************************************************************************/
/*! \brief          This function is Set the Error Memory Details
 *
 *
 *  \param[in]       uBmiIface
 *  \param[in]       uSvcId
 *  \param[in]       uErrorId
 *  \param[in]       param_1
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *
 *  \return         Set the Error Memory Information
 *  \retval         BMI_OK: Successful
 *                  BMI_NOT_OK: Failed
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_SetErrorDetails(uint8 uSvcId, uint8 uErrorId, uint32 qParam1, uint32 qParam2)
{
#if(TIMBS_COMIF_EMEM_ENABLE == STD_ON)
    Emem_EntryType *pEmemEntry;
    uint8 uErrIdx;
    uint8 uEMIdOcpd;

    if((uErrorId > (uint8)COMIF_OK) && (uErrorId < (uint8)COMIF_CMD_CFGINIT_SUCCESS))
    {
        if(ERRMEM_STATE_INIT == zComifEmemMgr.uEmemState)
        {
            uEMIdOcpd = zComifEmemMgr.uEmemSiz;
            (void)zComifEmemMgr.resource_req(zComifEmemMgr.qEmemRes, GET_RESOURCE);
            for(uErrIdx = 0; uErrIdx < zComifEmemMgr.uEmemSiz; uErrIdx++)
            {
                if((uErrorId == zComifEmemMgr.pEmemEntry[uErrIdx].uErrorId) &&
                   (uSvcId == zComifEmemMgr.pEmemEntry[uErrIdx].eServiceId))
                {
                    uEMIdOcpd = uErrIdx;
                }
            }

            if(zComifEmemMgr.uEmemSiz == uEMIdOcpd)
            {
                pEmemEntry = &zComifEmemMgr.pEmemEntry[zComifEmemMgr.uEmemIdx++];
                if(NULL != pEmemEntry)
                {
                    pEmemEntry->eServiceId = uSvcId;
                    pEmemEntry->uErrorId = uErrorId;
                    pEmemEntry->qParam1 = qParam1;
                    pEmemEntry->qParam2 = qParam2;
                }
            }
            else
            {
                pEmemEntry = &zComifEmemMgr.pEmemEntry[uEMIdOcpd];
                if(NULL != pEmemEntry)
                {
                    pEmemEntry->nError++;
                }
            }

            if(zComifEmemMgr.uEmemIdx == zComifEmemMgr.uEmemSiz)
            {
                zComifEmemMgr.uEmemIdx = 0;
            }

            (void)zComifEmemMgr.resource_req(zComifEmemMgr.qEmemRes, RELEASE_RESOURCE);
        }
    }
#endif

    return (uint8)EMEM_ERRORSTATUS2USERSTATUS(uErrorId, ((uErrorId > (uint8)COMIF_OK) && (uErrorId < (uint8)COMIF_CMD_CFGINIT_SUCCESS)));
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_ErrMemoryInit(void)
 *********************************************************************************************************************/
/*! \brief          Error Memory Init
 *
 *  \reentrant      TRUE
 *  \synchronous    TRUE
 *  \pre
 *  \post
 *
 *  \return         status of version information
 *  \retval         BMI_OK: Successful
 *                  BMI_NOT_OK: Failed
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_ErrMemoryInit(void)
{
    uint8 uRet = (uint8)COMIF_NOT_OK;

#if(TIMBS_COMIF_EMEM_ENABLE == STD_ON)
    if(NULL != zComifEmemConfig.pEmemEntry)
    {
        zComifEmemMgr.resource_req = zComifEmemConfig.resource_req;
        zComifEmemMgr.qEmemRes = zComifEmemConfig.qEmemRes;

        zComifEmemMgr.pEmemEntry = zComifEmemConfig.pEmemEntry;
        zComifEmemMgr.uEmemIdx = 0u;
        zComifEmemMgr.uEmemSiz = (uint8)zComifEmemConfig.wEmemEntrySiz;
		zComifEmemMgr.uEmemState = ERRMEM_STATE_INIT;
        uRet = (uint8)COMIF_OK;
    }
#else
    uRet = COMIF_OK;
#endif

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(Comif_ManagerType*, tibms_comif_CODE) Comif_Init(const Comif_ConfigType *pComifCfg, uint8 pComifCfg->uComIfaceCfg)
 *********************************************************************************************************************/
/*! \brief          Communication Inteface Init
 *
 *  \param[in]      pComifCfg: Commmunication Configuration
 *  \param[out]     pNfaultPin: Nfault pin configuration for the COM
 *  \param[out]     pStartupMode: Communication startup mode
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         Pointer to comif context for the intialized
 *  \retval         Comif_ManagerType *
 *  \trace
 *********************************************************************************************************************/

FUNC(Comif_ManagerType*, tibms_comif_CODE) Comif_Init(const Comif_ConfigType *pComifCfg, uint8 *pNfaultPin)
{
    Comif_ManagerType *pComifMgr;
    Comif_ManagerType *pComifMgrRet = NULL;
    uint8 uComIfaceId = 0xFF;
    uint8 uRet;

    do
    {
        #if(TIMBS_COMIF_EMEM_ENABLE == STD_ON)
        uRet = Comif_ErrMemoryInit();
        if((uint8)COMIF_NOT_OK == uRet)
        {
            uRet = (uint8)COMIF_ERROR_MEM_FAILED;
            break;
        }
        #endif

        if((NULL == pComifCfg) && (NULL == pComifCfg->pCommCfg))
        {
            uRet = (uint8)COMIF_INVALID_CONFIG;
            break;
        }

        if(pComifCfg->uComIfaceCfg >= TIBMS_NO_OF_COMIF_IFACES)
        {
            uRet = (uint8)COMIF_INVALID_IFACECFG;
            break;
        }

        uComIfaceId = pComifCfg->uComIfaceCfg;
        pComifMgr = &zComifManager[pComifCfg->uComIfaceCfg];
        if(NULL == pComifMgr)
        {
            uRet = (uint8)COMIF_INVALID_MGR;
            break;
        }

        if(COMIF_INIT == pComifMgr->uInit)
        {
            uRet = (uint8)COMIF_ALREADY_INUSE;
            break;
        }

        (void)memset(pComifMgr, 0, sizeof(Comif_ManagerType));
        pComifMgr->pComifOps = pComifCfg->pComifOpsCfg;
        if(NULL == pComifMgr->pComifOps)
        {
            uRet = (uint8)COMIF_INVALID_OPS_CFG;
            break;
        }

        if(NULL != pNfaultPin)
        {
            *pNfaultPin = pComifCfg->uNfaultPinCfg;
        }

        pComifMgr->pComifHandle = pComifMgr->pComifOps->init(pComifCfg->pComChipCfg,
                                                             &zComifEmemConfig.pEmemStats[pComifCfg->uComIfaceCfg],
                                                             pComifCfg->pCommCfg);
        if(NULL == pComifMgr->pComifHandle)
        {
            uRet = (uint8)COMIF_DEVICE_INIT_FAILED;
            break;
        }

        pComifMgr->pCommCfg = pComifCfg->pCommCfg;
        pComifMgr->uComifIface = pComifCfg->uComIfaceCfg;
        pComifMgr->uInit = COMIF_INIT;

        pComifMgr->qGuardStart = COMIF_GUARD_START;
        pComifMgr->qGuardEnd = COMIF_GUARD_END;
        pComifMgrRet = pComifMgr;

        uRet = (uint8)COMIF_OK;
    }
    while(0);

    (void)Comif_SetErrorDetails(COMIF_SVCID_INIT, uRet, uComIfaceId, 0);

    return pComifMgrRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_DeInit(Comif_ManagerType *pComifMgr)
 *********************************************************************************************************************/
/*! \brief          request for communication interface deinit
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre            Comif_Init is completed with all basic initialization
 *  \post
 *  \return         returns the de-init state
 *  \retval         COMIF_OK: Successful De-init
 *                  COMIF_NOT_OK: Failed to De-init
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_DeInit(Comif_ManagerType *pComifMgr)
{
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        pComifMgr->pComifOps->deinit(pComifMgr->pComifHandle);
        (void)memset(pComifMgr, 0, sizeof(Comif_ManagerType));
        uRet = (uint8)COMIF_OK;
    }

    return uRet;
}

/**********************************************************************************************************************
 * FUNC(uint8, tibms_comif_CODE) Comif_GetVersion(Comif_ManagerType *pComifMgr, VersionInfoType *pVersion)
 *********************************************************************************************************************/
/*! \brief          Function for getting version
 *
 *  \param[in]      pComifMgr: Comif Manager for the corresponding chain
 *  \param[out]     pVersion: comif version
 *
 *  \reentrant      FALSE
 *  \synchronous    FALSE
 *  \pre
 *  \post
 *  \return         returns the de-init state
 *  \retval         COMIF_OK: Successful
 *                  COMIF_NOT_OK: Failed
 *  \trace
 *********************************************************************************************************************/

FUNC(uint8, tibms_comif_CODE) Comif_GetVersion(const Comif_ManagerType *pComifMgr, VersionInfoType *pComifVerInfo)
{
    uint8 uRet = (uint8)COMIF_INTEGRITY_FAILED;

    if(COMIF_INTEGRITY_CHECK(pComifMgr))
    {
        if(NULL != pComifVerInfo)
        {
            pComifVerInfo->zComifVersion.vendorID = COMIF_VENDOR_ID;
            pComifVerInfo->zComifVersion.moduleID = COMIF_MODULE_ID;

            pComifVerInfo->zComifVersion.sw_major_version = COMIF_SW_MAJOR_VERSION;
            pComifVerInfo->zComifVersion.sw_minor_version = COMIF_SW_MINOR_VERSION;
            pComifVerInfo->zComifVersion.sw_patch_version = COMIF_SW_PATCH_VERSION;

            uRet = pComifMgr->pComifOps->ioctl(pComifMgr->pComifHandle, COMIF_CMD_GETVERSION, (void *) pComifVerInfo);
        }
    }

    return uRet;
}

#define TIBMS_COMIF_STOP_SEC_CODE
#include "Cdd_MemMap.h"

/*********************************************************************************************************************
 * End of File: tibms_comif.c
 *********************************************************************************************************************/
