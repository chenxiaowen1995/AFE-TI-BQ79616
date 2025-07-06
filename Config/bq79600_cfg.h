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
 *  File:       bq79600_cfg.h
 *  Project:    TIBMS
 *  Module:     COMIF
 *  Generator:  Code generation tool (if any)
 *
 *  Description: pre-compile configuration for BQ79600
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

#ifndef BQ79600_CFG_H
#define BQ79600_CFG_H

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

#include "bq79600_regs.h"

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

#define BQ79600_CFG_MAJOR_VERSION                   (0x01u)
#define BQ79600_CFG_MINOR_VERSION                   (0x00u)
#define BQ79600_CFG_PATCH_VERSION                   (0x01u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

#define BQ79600_IFACES		                        (1u)

/* INIT(1) + REG(2) + DATA(1) + CRC(2) */
#define BQ79600_REQ_FRAME_FIXED_LEN                 (6u)

/* LEN(1) + ID(1) + REG(2) + DATA(1) + CRC(2) */
#define BQ79600_RSP_FRAME_FIXED_LEN                 (7u)

/* Data_Len(1) + DEV_ID(1) + REG_ADR(2) */
#define BQ79600_RSP_FRAME_PREFIX_LEN                (4u)

#define BQ79600_TX_DATA_MAX_LEN                     (16u)
#define BQ79600_RX_DATA_MAX_LEN                     (128u)

#define BQ79600_WAIT_SPI_RDY_TIME                   (2u)
#define BQ79600_WAIT_SPI_BIT_TIME                   (12u)
#define BQ79600_WAIT_COMCLEAR_RESP                  (5u)
#define BQ79600_WAIT_PING_RESPONSE                  (5u)
#define BQ79600_WAIT_BTW_PING                       (3u)

#define BQ79600_CMD_QUEUE_LEN_CHAIN_0               (93u)
#define BQ79600_EXEC_QUEUE_LEN_CHAIN_0              (93u)
#define BQ79600_READ_QUEUE_LEN_CHAIN_0              (15u)

#define BQ79600_DEV_ADR_RANGE_MAX                   (0x3Fu)

#define BQ79600_MAX_RETRY                           (3u)
#define WAKEUP_PULSE_CNT                            (4u)

/** Register Default Settings **/

#define BQ79600_DIR0_ADDR_POR_VAL                   (0x00u)
#define BQ79600_DIR1_ADDR_POR_VAL                   (0x00u)
#define BQ79600_CONTROL1_POR_VAL                    (0x00u)
#define BQ79600_CONTROL2_POR_VAL                    (0x00u)
#define BQ79600_DIAG_CTRL_POR_VAL                   (0x00u)
#define BQ79600_DEV_CONF1_POR_VAL                   (0x14u)
#define BQ79600_DEV_CONF2_POR_VAL                   (0x00u)
#define BQ79600_TX_HOLD_OFF_POR_VAL                 (0x00u)
#define BQ79600_SLP_TIMEOUT_POR_VAL                 (0x03u)
#define BQ79600_COMM_TIMEOUT_POR_VAL                (0x34u)
#define BQ79600_SPI_FIFO_UNLOCK_POR_VAL             (0x00u)
#define BQ79600_FAULT_MSK_POR_VAL                   (0x00u)
#define BQ79600_FAULT_RST_POR_VAL                   (0x00u)
#define BQ79600_FAULT_SUMMARY_POR_VAL               (0x00u)
#define BQ79600_FAULT_REG_POR_VAL                   (0x00u)
#define BQ79600_FAULT_SYS_POR_VAL                   (0x00u)
#define BQ79600_FAULT_PWR_POR_VAL                   (0x00u)
#define BQ79600_FAULT_COMM1_POR_VAL                 (0x00u)
#define BQ79600_FAULT_COMM2_POR_VAL                 (0x00u)
#define BQ79600_DEV_DIAG_STAT_POR_VAL               (0x00u)
#define BQ79600_PARTID_POR_VAL                      (0x00u)
#define BQ79600_DIE_ID1_POR_VAL                     (0x00u)
#define BQ79600_DIE_ID2_POR_VAL                     (0x00u)
#define BQ79600_DIE_ID3_POR_VAL                     (0x00u)
#define BQ79600_DIE_ID4_POR_VAL                     (0x00u)
#define BQ79600_DIE_ID5_POR_VAL                     (0x00u)
#define BQ79600_DIE_ID6_POR_VAL                     (0x00u)
#define BQ79600_DIE_ID7_POR_VAL                     (0x00u)
#define BQ79600_DIE_ID8_POR_VAL                     (0x00u)
#define BQ79600_DIE_ID9_POR_VAL                     (0x00u)
#define BQ79600_DEBUG_CTRL_UNLOCK_POR_VAL           (0x00u)
#define BQ79600_DEBUG_COMM_CTRL_POR_VAL             (0x78u)
#define BQ79600_DEBUG_COMM_STAT_POR_VAL             (0x13u)
#define BQ79600_DEBUG_SPI_PHY_POR_VAL               (0x00u)
#define BQ79600_DEBUG_SPI_FRAME_POR_VAL             (0x00u)
#define BQ79600_DEBUG_UART_FRAME_POR_VAL            (0x00u)
#define BQ79600_DEBUG_COMH_PHY_POR_VAL              (0x00u)
#define BQ79600_DEBUG_COMH_FRAME_POR_VAL            (0x00u)
#define BQ79600_DEBUG_COML_PHY_POR_VAL              (0x00u)
#define BQ79600_DEBUG_COML_FRAME_POR_VAL            (0x00u)

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef struct Bq79600_ReqDataType_Tag
{
    const ServiceCfgType *pServiceReq;
    uint8 *pRxData;

    uint16 wRspDataLen;
    uint16 wCurrRspDataLen;

    uint16 wReqDataLen;
    uint8 uCmd;
    uint8 uBlockSiz;

    uint8 zTxCmd[BQ79600_TX_DATA_MAX_LEN];

    uint8 uStatus;
    uint8 uRetryStat;
    uint8 uCmdInfo;
}
Bq79600_ReqDataType;

typedef struct Bq79600_ConfigType_Tag
{
    const Basic_ConfigType *pBswCfg;
    Bq79600_ReqDataType *pCmdDataMemCfg;

    void *pCmdQueFreeMemCfg;
    uint16 wCmdQueSizeCfg;
    uint16 wCmdQueCntCfg;

    void *(*pCmdQueExecMemCfg)[(BQ79600_CMD_QUEUE_LEN_CHAIN_0 + 1)];
    uint16 wExecQueSizeCfg;
    uint16 wExecQueCntCfg;

    void *pReadQueExecMemCfg;
    uint16 wReadQueSizeCfg;
    uint16 wReadQueCntCfg;
    uint8 ePeripCfg;

    uint8 uCmdIfaceCfg;
    uint8 uRsvd[3];
}
Bq79600_ConfigType;

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

extern CONST(Comif_ConfigType, TIBMSCFG_CONST) zBmiComifCfgWiredCfg_0;
extern CONST(Comif_ConfigType, TIBMSCFG_CONST) zPmiComifCfgWiredBridgeSpiCfg;

/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

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

#endif /*BQ79600_CFG_H*/

/*********************************************************************************************************************
 * End of File: bq79600_cfg.h
 *********************************************************************************************************************/
