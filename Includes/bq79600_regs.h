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
 *  File:       bq79600_regs.h
 *  Project:    TIBMS
 *  Module:     COMIF
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Register definitions for BQ79600 device
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05July2022    SEM                0000000000000    Initial version
 *
 *********************************************************************************************************************/

#ifndef BQ79600_REGS_H
#define BQ79600_REGS_H

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

#define BQ79600_REGS_MAJOR_VERSION                (0x01u)
#define BQ79600_REGS_MINOR_VERSION                (0x00u)
#define BQ79600_REGS_PATCH_VERSION                (0x01u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/
/* --------------------------------------------------------------------------
 * DIR0_ADDR (R/W):
 * -------------------------------------------------------------------------- */

#define BQ79600_DIR0_ADDR_OFFSET                    (0x306u)

#define BQ79600_DIR0_ADDR_ADDRESS_POS               (0x00u)
#define BQ79600_DIR0_ADDR_ADDRESS_MSK               (0x3Fu)

/* --------------------------------------------------------------------------
 * DIR1_ADDR (R/W):
 * -------------------------------------------------------------------------- */

#define BQ79600_DIR1_ADDR_OFFSET                    (0x307u)

#define BQ79600_DIR1_ADDR_ADDRESS_POS               (0x00u)
#define BQ79600_DIR1_ADDR_ADDRESS_MSK               (0x3Fu)

/* --------------------------------------------------------------------------
 * CONTROL1 (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ79600_CONTROL1_OFFSET                     (0x309u)

#define BQ79600_CONTROL1_DIR_SEL_POS                (0x07u)
#define BQ79600_CONTROL1_DIR_SEL_MSK                (0x80u)
#define BQ79600_CONTROL1_SEND_SHUTDOWN_POS          (0x06u)
#define BQ79600_CONTROL1_SEND_SHUTDOWN_MSK          (0x40u)
#define BQ79600_CONTROL1_SEND_WAKE_POS              (0x05u)
#define BQ79600_CONTROL1_SEND_WAKE_MSK              (0x20u)
#define BQ79600_CONTROL1_SEND_SLPTOACT_POS          (0x04u)
#define BQ79600_CONTROL1_SEND_SLPTOACT_MSK          (0x10u)
#define BQ79600_CONTROL1_GOTO_SHUTDOWN_POS          (0x03u)
#define BQ79600_CONTROL1_GOTO_SHUTDOWN_MSK          (0x08u)
#define BQ79600_CONTROL1_GOTO_SLEEP_POS             (0x02u)
#define BQ79600_CONTROL1_GOTO_SLEEP_MSK             (0x04u)
#define BQ79600_CONTROL1_SOFT_RESET_POS             (0x01u)
#define BQ79600_CONTROL1_SOFT_RESET_MSK             (0x02u)
#define BQ79600_CONTROL1_ADDR_WR_POS                (0x00u)
#define BQ79600_CONTROL1_ADDR_WR_MSK                (0x01u)

/* --------------------------------------------------------------------------
 * CONTROL2 (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ79600_CONTROL2_OFFSET                     (0x30Au)

#define BQ79600_CONTROL2_SEND_HW_RESET_POS          (0x01u)
#define BQ79600_CONTROL2_SEND_HW_RESET_MSK          (0x02u)

/* --------------------------------------------------------------------------
 * DIAG_CTRL (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ79600_DIAG_CTRL_OFFSET                    (0x2000u)

#define BQ79600_DIAG_CTRL_CONF_MON_GO_POS           (0x05u)
#define BQ79600_DIAG_CTRL_CONF_MON_GO_MSK           (0x20u)
#define BQ79600_DIAG_CTRL_PWR_DIAG_GO_POS           (0x04u)
#define BQ79600_DIAG_CTRL_PWR_DIAG_GO_MSK           (0x10u)
#define BQ79600_DIAG_CTRL_SPI_FIFO_DIAG_GO_POS      (0x03u)
#define BQ79600_DIAG_CTRL_SPI_FIFO_DIAG_GO_MSK      (0x08u)
#define BQ79600_DIAG_CTRL_FLIP_FACT_CRC_POS         (0x02u)
#define BQ79600_DIAG_CTRL_FLIP_FACT_CRC_MSK         (0x04u)
#define BQ79600_DIAG_CTRL_FLIP_TR_CRC_POS           (0x01u)
#define BQ79600_DIAG_CTRL_FLIP_TR_CRC_MSK           (0x02u)
#define BQ79600_DIAG_CTRL_INH_SET_GO_POS            (0x00u)
#define BQ79600_DIAG_CTRL_INH_SET_GO_MSK            (0x01u)

/* --------------------------------------------------------------------------
 * DEV_CONF1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ79600_DEV_CONF1_OFFSET                    (0x2001u)

#define BQ79600_DEV_CONF1_SNIFDET_EN_POS            (0x07u)
#define BQ79600_DEV_CONF1_SNIFDET_EN_MSK            (0x80u)
#define BQ79600_DEV_CONF1_SNIFDET_DIS_POS           (0x06u)
#define BQ79600_DEV_CONF1_SNIFDET_DIS_MSK           (0x40u)
#define BQ79600_DEV_CONF1_TONE_RX_EN_POS            (0x05u)
#define BQ79600_DEV_CONF1_TONE_RX_EN_MSK            (0x20u)
#define BQ79600_DEV_CONF1_FCOMM_EN_POS              (0x04u)
#define BQ79600_DEV_CONF1_FCOMM_EN_MSK              (0x10u)
#define BQ79600_DEV_CONF1_TWO_STOP_EN_POS           (0x03u)
#define BQ79600_DEV_CONF1_TWO_STOP_EN_MSK           (0x08u)
#define BQ79600_DEV_CONF1_NFAULT_EN_POS             (0x02u)
#define BQ79600_DEV_CONF1_NFAULT_EN_MSK             (0x04u)
#define BQ79600_DEV_CONF1_RESERVED_POS              (0x01u)
#define BQ79600_DEV_CONF1_RESERVED_MSK              (0x02u)
#define BQ79600_DEV_CONF1_HB_TX_EN_POS              (0x00u)
#define BQ79600_DEV_CONF1_HB_TX_EN_MSK              (0x01u)

/* --------------------------------------------------------------------------
 * DEV_CONF2 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ79600_DEV_CONF2_OFFSET                    (0x2002u)

#define BQ79600_DEV_CONF2_INH_DIS_POS               (0x00u)
#define BQ79600_DEV_CONF2_INH_DIS_MSK               (0x03u)

/* --------------------------------------------------------------------------
 * TX_HOLD_OFF (R/W):
 * -------------------------------------------------------------------------- */

#define BQ79600_TX_HOLD_OFF_OFFSET                  (0x2003u)

#define BQ79600_TX_HOLD_OFF_DLY_POS                 (0x00u)
#define BQ79600_TX_HOLD_OFF_DLY_MSK                 (0xFFu)

/* --------------------------------------------------------------------------
 * SLP_TIMEOUT (R/W):
 * -------------------------------------------------------------------------- */

#define BQ79600_SLP_TIMEOUT_OFFSET                  (0x2004u)

#define BQ79600_SLP_TIMEOUT_SLP_TIME_POS            (0x00u)
#define BQ79600_SLP_TIMEOUT_SLP_TIME_MSK            (0x07u)

/* --------------------------------------------------------------------------
 * COMM_TIMEOUT (R/W):
 * -------------------------------------------------------------------------- */

#define BQ79600_COMM_TIMEOUT_OFFSET                 (0x2005u)

#define BQ79600_COMM_TIMEOUT_CTS_TIME_POS           (0x04u)
#define BQ79600_COMM_TIMEOUT_CTS_TIME_MSK           (0x70u)
#define BQ79600_COMM_TIMEOUT_CTL_ACT_POS            (0x03u)
#define BQ79600_COMM_TIMEOUT_CTL_ACT_MSK            (0x08u)
#define BQ79600_COMM_TIMEOUT_CTL_TIME_POS           (0x00u)
#define BQ79600_COMM_TIMEOUT_CTL_TIME_MSK           (0x07u)

/* --------------------------------------------------------------------------
 * SPI_FIFO_UNLOCK (R/W):
 * -------------------------------------------------------------------------- */

#define BQ79600_SPI_FIFO_UNLOCK_OFFSET              (0x2010u)

#define BQ79600_SPI_FIFO_UNLOCK_CODE_POS            (0x00u)
#define BQ79600_SPI_FIFO_UNLOCK_CODE_MSK            (0x0Fu)

/* --------------------------------------------------------------------------
 * FAULT_MSK (R/W):
 * -------------------------------------------------------------------------- */

#define BQ79600_FAULT_MSK_OFFSET                    (0x2020u)

#define BQ79600_FAULT_MSK_MSK_COML_H_POS            (0x07u)
#define BQ79600_FAULT_MSK_MSK_COML_H_MSK            (0x80u)
#define BQ79600_FAULT_MSK_MSK_UART_SPI_POS          (0x06u)
#define BQ79600_FAULT_MSK_MSK_UART_SPI_MSK          (0x40u)
#define BQ79600_FAULT_MSK_MSK_FCOMM_DET_POS         (0x05u)
#define BQ79600_FAULT_MSK_MSK_FCOMM_DET_MSK         (0x20u)
#define BQ79600_FAULT_MSK_MSK_FTONE_DET_POS         (0x04u)
#define BQ79600_FAULT_MSK_MSK_FTONE_DET_MSK         (0x10u)
#define BQ79600_FAULT_MSK_MSK_HB_POS                (0x03u)
#define BQ79600_FAULT_MSK_MSK_HB_MSK                (0x08u)
#define BQ79600_FAULT_MSK_MSK_REG_POS               (0x02u)
#define BQ79600_FAULT_MSK_MSK_REG_MSK               (0x04u)
#define BQ79600_FAULT_MSK_MSK_SYS_POS               (0x01u)
#define BQ79600_FAULT_MSK_MSK_SYS_MSK               (0x02u)
#define BQ79600_FAULT_MSK_MSK_PWR_POS               (0x00u)
#define BQ79600_FAULT_MSK_MSK_PWR_MSK               (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_RST (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ79600_FAULT_RST_OFFSET                    (0x2030u)

#define BQ79600_FAULT_RST_RST_COML_H_POS            (0x07u)
#define BQ79600_FAULT_RST_RST_COML_H_MSK            (0x80u)
#define BQ79600_FAULT_RST_RST_UART_SPI_POS          (0x06u)
#define BQ79600_FAULT_RST_RST_UART_SPI_MSK          (0x40u)
#define BQ79600_FAULT_RST_RST_FCOMM_DET_POS         (0x05u)
#define BQ79600_FAULT_RST_RST_FCOMM_DET_MSK         (0x20u)
#define BQ79600_FAULT_RST_RST_FTONE_DET_POS         (0x04u)
#define BQ79600_FAULT_RST_RST_FTONE_DET_MSK         (0x10u)
#define BQ79600_FAULT_RST_RST_HB_POS                (0x03u)
#define BQ79600_FAULT_RST_RST_HB_MSK                (0x08u)
#define BQ79600_FAULT_RST_RST_REG_POS               (0x02u)
#define BQ79600_FAULT_RST_RST_REG_MSK               (0x04u)
#define BQ79600_FAULT_RST_RST_SYS_POS               (0x01u)
#define BQ79600_FAULT_RST_RST_SYS_MSK               (0x02u)
#define BQ79600_FAULT_RST_RST_PWR_POS               (0x00u)
#define BQ79600_FAULT_RST_RST_PWR_MSK               (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_SUMMARY (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_FAULT_SUMMARY_OFFSET                (0x2100u)

#define BQ79600_FAULT_SUMMARY_FAULT_COMM_POS        (0x03u)
#define BQ79600_FAULT_SUMMARY_FAULT_COMM_MSK        (0x08u)
#define BQ79600_FAULT_SUMMARY_FAULT_REG_POS         (0x02u)
#define BQ79600_FAULT_SUMMARY_FAULT_REG_MSK         (0x04u)
#define BQ79600_FAULT_SUMMARY_FAULT_SYS_POS         (0x01u)
#define BQ79600_FAULT_SUMMARY_FAULT_SYS_MSK         (0x02u)
#define BQ79600_FAULT_SUMMARY_FAULT_PWR_POS         (0x00u)
#define BQ79600_FAULT_SUMMARY_FAULT_PWR_MSK         (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_REG (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_FAULT_REG_OFFSET                    (0x2101u)

#define BQ79600_FAULT_REG_CONF_MON_ERR_POS          (0x02u)
#define BQ79600_FAULT_REG_CONF_MON_ERR_MSK          (0x04u)
#define BQ79600_FAULT_REG_FACTLDERR_POS             (0x01u)
#define BQ79600_FAULT_REG_FACTLDERR_MSK             (0x02u)
#define BQ79600_FAULT_REG_FACT_CRC_POS              (0x00u)
#define BQ79600_FAULT_REG_FACT_CRC_MSK              (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_SYS (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_FAULT_SYS_OFFSET                    (0x2102u)

#define BQ79600_FAULT_SYS_VALIDATE_DET_POS          (0X07u)
#define BQ79600_FAULT_SYS_VALIDATE_DET_MSK          (0x80u)
#define BQ79600_FAULT_SYS_LFO_POS                   (0X06u)
#define BQ79600_FAULT_SYS_LFO_MSK                   (0x40u)
#define BQ79600_FAULT_SYS_SHUTDOWN_REC_POS          (0X05u)
#define BQ79600_FAULT_SYS_SHUTDOWN_REC_MSK          (0x20u)
#define BQ79600_FAULT_SYS_DRST_POS                  (0X04u)
#define BQ79600_FAULT_SYS_DRST_MSK                  (0x10u)
#define BQ79600_FAULT_SYS_CTL_POS                   (0X03u)
#define BQ79600_FAULT_SYS_CTL_MSK                   (0x08u)
#define BQ79600_FAULT_SYS_CTS_POS                   (0X02u)
#define BQ79600_FAULT_SYS_CTS_MSK                   (0x04u)
#define BQ79600_FAULT_SYS_TSHUT_POS                 (0X01u)
#define BQ79600_FAULT_SYS_TSHUT_MSK                 (0x02u)
#define BQ79600_FAULT_SYS_INH_POS                   (0x00u)
#define BQ79600_FAULT_SYS_INH_MSK                   (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_PWR (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_FAULT_PWR_OFFSET                    (0x2103u)

#define BQ79600_FAULT_PWR_CVDD_UV_DRST_POS          (0X04u)
#define BQ79600_FAULT_PWR_CVDD_UV_DRST_MSK          (0x10u)
#define BQ79600_FAULT_PWR_CVDD_OV_POS               (0X03u)
#define BQ79600_FAULT_PWR_CVDD_OV_MSK               (0x08u)
#define BQ79600_FAULT_PWR_DVDD_OV_POS               (0X02u)
#define BQ79600_FAULT_PWR_DVDD_OV_MSK               (0x04u)
#define BQ79600_FAULT_PWR_AVDDREF_OV_POS            (0X01u)
#define BQ79600_FAULT_PWR_AVDDREF_OV_MSK            (0x02u)
#define BQ79600_FAULT_PWR_AVAO_SW_FAIL_POS          (0x00u)
#define BQ79600_FAULT_PWR_AVAO_SW_FAIL_MSK          (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_COMM1 (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_FAULT_COMM1_OFFSET                  (0x2104u)

#define BQ79600_FAULT_COMM1_FCOMM_DET_POS           (0X06u)
#define BQ79600_FAULT_COMM1_FCOMM_DET_MSK           (0x40u)
#define BQ79600_FAULT_COMM1_FTONE_DET_POS           (0X05u)
#define BQ79600_FAULT_COMM1_FTONE_DET_MSK           (0x20u)
#define BQ79600_FAULT_COMM1_HB_FAIL_POS             (0X04u)
#define BQ79600_FAULT_COMM1_HB_FAIL_MSK             (0x10u)
#define BQ79600_FAULT_COMM1_HB_FAST_POS             (0X03u)
#define BQ79600_FAULT_COMM1_HB_FAST_MSK             (0x08u)
#define BQ79600_FAULT_COMM1_UART_FRAME_POS          (0X02u)
#define BQ79600_FAULT_COMM1_UART_FRAME_MSK          (0x04u)
#define BQ79600_FAULT_COMM1_COMMCLR_DET_POS         (0X01u)
#define BQ79600_FAULT_COMM1_COMMCLR_DET_MSK         (0x02u)
#define BQ79600_FAULT_COMM1_STOP_DET_POS            (0x00u)
#define BQ79600_FAULT_COMM1_STOP_DET_MSK            (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_COMM2 (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_FAULT_COMM2_OFFSET                  (0x2105u)

#define BQ79600_FAULT_COMM2_SPI_FRAME_POS           (0x05u)
#define BQ79600_FAULT_COMM2_SPI_FRAME_MSK           (0x20u)
#define BQ79600_FAULT_COMM2_SPI_PHY_POS             (0x04u)
#define BQ79600_FAULT_COMM2_SPI_PHY_MSK             (0x10u)
#define BQ79600_FAULT_COMM2_COML_FRAME_POS          (0x03u)
#define BQ79600_FAULT_COMM2_COML_FRAME_MSK          (0x08u)
#define BQ79600_FAULT_COMM2_COML_PHY_POS            (0x02u)
#define BQ79600_FAULT_COMM2_COML_PHY_MSK            (0x04u)
#define BQ79600_FAULT_COMM2_COMH_FRAME_POS          (0x01u)
#define BQ79600_FAULT_COMM2_COMH_FRAME_MSK          (0x02u)
#define BQ79600_FAULT_COMM2_COMH_PHY_POS            (0x00u)
#define BQ79600_FAULT_COMM2_COMH_PHY_MSK            (0x01u)

/* --------------------------------------------------------------------------
 * DEV_DIAG_STAT (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DEV_DIAG_STAT_OFFSET                (0x2110u)

#define BQ79600_DEV_DIAG_STAT_PWR_DIAG_RDY_POS      (0x01u)
#define BQ79600_DEV_DIAG_STAT_PWR_DIAG_RDY_MSK      (0x02u)
#define BQ79600_DEV_DIAG_STAT_INH_STAT_POS          (0x00u)
#define BQ79600_DEV_DIAG_STAT_INH_STAT_MSK          (0x01u)

/* --------------------------------------------------------------------------
 * PARTID (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_PARTID_OFFSET                       (0x2120u)

#define BQ79600_PARTID_REV_POS                      (0x00u)
#define BQ79600_PARTID_REV_MSK                      (0xFFu)

/* --------------------------------------------------------------------------
 * DIE_ID1 (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DIE_ID1_OFFSET                      (0x2121u)

#define BQ79600_DIE_ID1_ID_POS                      (0x00u)
#define BQ79600_DIE_ID1_ID_MSK                      (0xFFu)

/* --------------------------------------------------------------------------
 * DIE_ID2 (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DIE_ID2_OFFSET                      (0x2122u)

#define BQ79600_DIE_ID2_ID_POS                      (0x00u)
#define BQ79600_DIE_ID2_ID_MSK                      (0xFFu)

/* --------------------------------------------------------------------------
 * DIE_ID3 (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DIE_ID3_OFFSET                      (0x2123u)

#define BQ79600_DIE_ID3_ID_POS                      (0x00u)
#define BQ79600_DIE_ID3_ID_MSK                      (0xFFu)

/* --------------------------------------------------------------------------
 * DIE_ID4 (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DIE_ID4_OFFSET                      (0x2124u)

#define BQ79600_DIE_ID4_ID_POS                      (0x00u)
#define BQ79600_DIE_ID4_ID_MSK                      (0xFFu)

/* --------------------------------------------------------------------------
 * DIE_ID5 (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DIE_ID5_OFFSET                      (0x2125u)

#define BQ79600_DIE_ID5_ID_POS                      (0x00u)
#define BQ79600_DIE_ID5_ID_MSK                      (0xFFu)

/* --------------------------------------------------------------------------
 * DIE_ID6 (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DIE_ID6_OFFSET                      (0x2126u)

#define BQ79600_DIE_ID6_ID_POS                      (0x00u)
#define BQ79600_DIE_ID6_ID_MSK                      (0xFFu)

/* --------------------------------------------------------------------------
 * DIE_ID7 (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DIE_ID7_OFFSET                      (0x2127u)

#define BQ79600_DIE_ID7_ID_POS                      (0x00u)
#define BQ79600_DIE_ID7_ID_MSK                      (0xFFu)

/* --------------------------------------------------------------------------
 * DIE_ID8 (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DIE_ID8_OFFSET                      (0x2128u)

#define BQ79600_DIE_ID8_ID_POS                      (0x00u)
#define BQ79600_DIE_ID8_ID_MSK                      (0xFFu)

/* --------------------------------------------------------------------------
 * DIE_ID9 (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DIE_ID9_OFFSET                      (0x2129u)

#define BQ79600_DIE_ID9_ID_POS                      (0x00u)
#define BQ79600_DIE_ID9_ID_MSK                      (0xFFu)

/* --------------------------------------------------------------------------
 * DEBUG_CTRL_UNLOCK (R/W):
 * -------------------------------------------------------------------------- */

#define BQ79600_DEBUG_CTRL_UNLOCK_OFFSET            (0x2200u)

#define BQ79600_DEBUG_CTRL_UNLOCK_CODE_POS          (0x00u)
#define BQ79600_DEBUG_CTRL_UNLOCK_CODE_MSK          (0xFFu)

/* --------------------------------------------------------------------------
 * DEBUG_COMM_CTRL (R/W):
 * -------------------------------------------------------------------------- */

#define BQ79600_DEBUG_COMM_CTRL_OFFSET              (0x2201u)

#define BQ79600_DEBUG_COMM_CTRL_COML_TX_EN_POS      (0x06u)
#define BQ79600_DEBUG_COMM_CTRL_COML_TX_EN_MSK      (0x40u)
#define BQ79600_DEBUG_COMM_CTRL_COML_RX_EN_POS      (0x05u)
#define BQ79600_DEBUG_COMM_CTRL_COML_RX_EN_MSK      (0x20u)
#define BQ79600_DEBUG_COMM_CTRL_COMH_TX_EN_POS      (0x04u)
#define BQ79600_DEBUG_COMM_CTRL_COMH_TX_EN_MSK      (0x10u)
#define BQ79600_DEBUG_COMM_CTRL_COMH_RX_EN_POS      (0x03u)
#define BQ79600_DEBUG_COMM_CTRL_COMH_RX_EN_MSK      (0x08u)
#define BQ79600_DEBUG_COMM_CTRL_UART_VIF_BAUD_POS   (0x02u)
#define BQ79600_DEBUG_COMM_CTRL_UART_VIF_BAUD_MSK   (0x04u)
#define BQ79600_DEBUG_COMM_CTRL_USER_UART_EN_POS    (0x01u)
#define BQ79600_DEBUG_COMM_CTRL_USER_UART_EN_MSK    (0x02u)
#define BQ79600_DEBUG_COMM_CTRL_USER_DAISY_EN_POS   (0x00u)
#define BQ79600_DEBUG_COMM_CTRL_USER_DAISY_EN_MSK   (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_COMM_STAT (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DEBUG_COMM_STAT_OFFSET              (0x2300u)

#define BQ79600_DEBUG_COMM_STAT_HW_DAISY_DRV_POS    (0x04u)
#define BQ79600_DEBUG_COMM_STAT_HW_DAISY_DRV_MSK    (0x10u)
#define BQ79600_DEBUG_COMM_STAT_COML_TX_ON_POS      (0x03u)
#define BQ79600_DEBUG_COMM_STAT_COML_TX_ON_MSK      (0x08u)
#define BQ79600_DEBUG_COMM_STAT_COML_RX_ON_POS      (0x02u)
#define BQ79600_DEBUG_COMM_STAT_COML_RX_ON_MSK      (0x04u)
#define BQ79600_DEBUG_COMM_STAT_COMH_TX_ON_POS      (0x01u)
#define BQ79600_DEBUG_COMM_STAT_COMH_TX_ON_MSK      (0x02u)
#define BQ79600_DEBUG_COMM_STAT_COMH_RX_ON_POS      (0x00u)
#define BQ79600_DEBUG_COMM_STAT_COMH_RX_ON_MSK      (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_SPI_PHY (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DEBUG_SPI_PHY_OFFSET                (0x2301u)

#define BQ79600_DEBUG_SPI_PHY_FMT_ERR_POS           (0x06u)
#define BQ79600_DEBUG_SPI_PHY_FMT_ERR_MSK           (0x40u)
#define BQ79600_DEBUG_SPI_PHY_COMCLR_ERR_POS        (0x05u)
#define BQ79600_DEBUG_SPI_PHY_COMCLR_ERR_MSK        (0x20u)
#define BQ79600_DEBUG_SPI_PHY_TXDATA_UNEXP_POS      (0x04u)
#define BQ79600_DEBUG_SPI_PHY_TXDATA_UNEXP_MSK      (0x10u)
#define BQ79600_DEBUG_SPI_PHY_RXDATA_UNEXP_POS      (0x03u)
#define BQ79600_DEBUG_SPI_PHY_RXDATA_UNEXP_MSK      (0x08u)
#define BQ79600_DEBUG_SPI_PHY_TXFIFO_OF_POS         (0x02u)
#define BQ79600_DEBUG_SPI_PHY_TXFIFO_OF_MSK         (0x04u)
#define BQ79600_DEBUG_SPI_PHY_TXFIFO_UF_POS         (0x01u)
#define BQ79600_DEBUG_SPI_PHY_TXFIFO_UF_MSK         (0x02u)
#define BQ79600_DEBUG_SPI_PHY_RXFIFO_OF_POS         (0x00u)
#define BQ79600_DEBUG_SPI_PHY_RXFIFO_OF_MSK         (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_SPI_FRAME (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DEBUG_SPI_FRAME_OFFSET              (0x2302u)

#define BQ79600_DEBUG_SPI_FRAME_TR_SOF_POS          (0x05u)
#define BQ79600_DEBUG_SPI_FRAME_TR_SOF_MSK          (0x20u)
#define BQ79600_DEBUG_SPI_FRAME_TR_WAIT_POS         (0x04u)
#define BQ79600_DEBUG_SPI_FRAME_TR_WAIT_MSK         (0x10u)
#define BQ79600_DEBUG_SPI_FRAME_RC_IERR_POS         (0x03u)
#define BQ79600_DEBUG_SPI_FRAME_RC_IERR_MSK         (0x08u)
#define BQ79600_DEBUG_SPI_FRAME_RC_SOF_POS          (0x02u)
#define BQ79600_DEBUG_SPI_FRAME_RC_SOF_MSK          (0x04u)
#define BQ79600_DEBUG_SPI_FRAME_RC_BYTE_ERR_POS     (0x01u)
#define BQ79600_DEBUG_SPI_FRAME_RC_BYTE_ERR_MSK     (0x02u)
#define BQ79600_DEBUG_SPI_FRAME_RC_CRC_POS          (0x00u)
#define BQ79600_DEBUG_SPI_FRAME_RC_CRC_MSK          (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_UART_FRAME (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DEBUG_UART_FRAME_OFFSET             (0x2303u)

#define BQ79600_DEBUG_UART_FRAME_TR_SOF_POS         (0x05u)
#define BQ79600_DEBUG_UART_FRAME_TR_SOF_MSK         (0x20u)
#define BQ79600_DEBUG_UART_FRAME_TR_WAIT_POS        (0x04u)
#define BQ79600_DEBUG_UART_FRAME_TR_WAIT_MSK        (0x10u)
#define BQ79600_DEBUG_UART_FRAME_RC_IERR_POS        (0x03u)
#define BQ79600_DEBUG_UART_FRAME_RC_IERR_MSK        (0x08u)
#define BQ79600_DEBUG_UART_FRAME_RC_SOF_POS         (0x02u)
#define BQ79600_DEBUG_UART_FRAME_RC_SOF_MSK         (0x04u)
#define BQ79600_DEBUG_UART_FRAME_RC_BYTE_ERR_POS    (0x01u)
#define BQ79600_DEBUG_UART_FRAME_RC_BYTE_ERR_MSK    (0x02u)
#define BQ79600_DEBUG_UART_FRAME_RC_CRC_POS         (0x00u)
#define BQ79600_DEBUG_UART_FRAME_RC_CRC_MSK         (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_COMH_PHY (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DEBUG_COMH_PHY_OFFSET               (0x2304u)

#define BQ79600_DEBUG_COMH_PHY_PERR_POS             (0x04u)
#define BQ79600_DEBUG_COMH_PHY_PERR_MSK             (0x10u)
#define BQ79600_DEBUG_COMH_PHY_BERR_TAG_POS         (0x03u)
#define BQ79600_DEBUG_COMH_PHY_BERR_TAG_MSK         (0x08u)
#define BQ79600_DEBUG_COMH_PHY_SYNC2_POS            (0x02u)
#define BQ79600_DEBUG_COMH_PHY_SYNC2_MSK            (0x04u)
#define BQ79600_DEBUG_COMH_PHY_SYNC1_POS            (0x01u)
#define BQ79600_DEBUG_COMH_PHY_SYNC1_MSK            (0x02u)
#define BQ79600_DEBUG_COMH_PHY_BIT_POS              (0x00u)
#define BQ79600_DEBUG_COMH_PHY_BIT_MSK              (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_COMH_FRAME (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DEBUG_COMH_FRAME_OFFSET             (0x2305u)

#define BQ79600_DEBUG_COMH_FRAME_RR_IERR_POS        (0x04u)
#define BQ79600_DEBUG_COMH_FRAME_RR_IERR_MSK        (0x10u)
#define BQ79600_DEBUG_COMH_FRAME_RR_SOF_POS         (0x03u)
#define BQ79600_DEBUG_COMH_FRAME_RR_SOF_MSK         (0x08u)
#define BQ79600_DEBUG_COMH_FRAME_RR_BYTE_ERR_POS    (0x02u)
#define BQ79600_DEBUG_COMH_FRAME_RR_BYTE_ERR_MSK    (0x04u)
#define BQ79600_DEBUG_COMH_FRAME_RR_UNEXP_POS       (0x01u)
#define BQ79600_DEBUG_COMH_FRAME_RR_UNEXP_MSK       (0x02u)
#define BQ79600_DEBUG_COMH_FRAME_RR_CRC_POS         (0x00u)
#define BQ79600_DEBUG_COMH_FRAME_RR_CRC_MSK         (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_COML_PHY (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DEBUG_COML_PHY_OFFSET               (0x2306u)

#define BQ79600_DEBUG_COML_PHY_PERR_POS             (0x04u)
#define BQ79600_DEBUG_COML_PHY_PERR_MSK             (0x10u)
#define BQ79600_DEBUG_COML_PHY_BERR_TAG_POS         (0x03u)
#define BQ79600_DEBUG_COML_PHY_BERR_TAG_MSK         (0x08u)
#define BQ79600_DEBUG_COML_PHY_SYNC2_POS            (0x02u)
#define BQ79600_DEBUG_COML_PHY_SYNC2_MSK            (0x04u)
#define BQ79600_DEBUG_COML_PHY_SYNC1_POS            (0x01u)
#define BQ79600_DEBUG_COML_PHY_SYNC1_MSK            (0x02u)
#define BQ79600_DEBUG_COML_PHY_BIT_POS              (0x00u)
#define BQ79600_DEBUG_COML_PHY_BIT_MSK              (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_COML_FRAME (R):
 * -------------------------------------------------------------------------- */

#define BQ79600_DEBUG_COML_FRAME_OFFSET             (0x2307u)

#define BQ79600_DEBUG_COML_FRAME_RR_IERR_POS        (0x04u)
#define BQ79600_DEBUG_COML_FRAME_RR_IERR_MSK        (0x10u)
#define BQ79600_DEBUG_COML_FRAME_RR_SOF_POS         (0x03u)
#define BQ79600_DEBUG_COML_FRAME_RR_SOF_MSK         (0x08u)
#define BQ79600_DEBUG_COML_FRAME_RR_BYTE_ERR_POS    (0x02u)
#define BQ79600_DEBUG_COML_FRAME_RR_BYTE_ERR_MSK    (0x04u)
#define BQ79600_DEBUG_COML_FRAME_RR_UNEXP_POS       (0x01u)
#define BQ79600_DEBUG_COML_FRAME_RR_UNEXP_MSK       (0x02u)
#define BQ79600_DEBUG_COML_FRAME_RR_CRC_POS         (0x00u)
#define BQ79600_DEBUG_COML_FRAME_RR_CRC_MSK         (0x01u)

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

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

#endif /*BQ79600_REGS_H*/

/*********************************************************************************************************************
 * End of File: bq79600_regs.h
 *********************************************************************************************************************/
