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
 *  File:       bq7971x_regs.h
 *  Project:    TIBMS
 *  Module:     BMI
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  BQ79716/8 Register Map file
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       05May2022    SEM                0000000000000    Initial version
 * 01.00.01       16May2023    SEM                0000000000000    Initial version
 *
 *********************************************************************************************************************/

#ifndef BQ7971X_REGS_H
#define BQ7971X_REGS_H

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

#define BQ7971X_REGS_MAJOR_VERSION                      (0x01u)
#define BQ7971X_REGS_MINOR_VERSION                      (0x00u)
#define BQ7971X_REGS_PATCH_VERSION                      (0x00u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

#define BQ7971X_GPIO_CONF_REG_NUM                       (0x06u)
#define BQ7971X_ADC_CTRL_REG_NUM                        (0x04u)
#define BQ7971X_BAL_CTRL_REG_NUM                        (0x03u)
#define BQ7971X_DIAG_ADC_DEV_STAT_NUM                   (0x05u)
#define BQ7971X_FAULT_ADC_GPIO_NUM                      (0x02u)
#define BQ7971X_FAULT_ADC_VCELL_NUM                     (0x03u)
#define BQ7971X_FAULT_OVUV_NUM                          (0x03u)
#define BQ7971X_FAULT_OTUT_NUM                          (0x02u)
#define BQ7971X_FAULT_ADC_DIG_NUM                       (0x03u)
#define BQ7971X_DEV_CONF_REG_NUM                        (0x02u)

/* --------------------------------------------------------------------------
 * CUST_CRC_HI          (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CUST_CRC_HI_OFFSET                      (0x000u)
#define BQ7971X_CUST_CRC_HI_POR_VAL                     (0x1Au)

#define BQ7971X_CUST_CRC_HI_CRC_POS                     (0x00u)
#define BQ7971X_CUST_CRC_HI_CRC_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * CUST_CRC_LO          (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CUST_CRC_LO_OFFSET                      (0x001u)
#define BQ7971X_CUST_CRC_LO_POR_VAL                     (0x90u)

#define BQ7971X_CUST_CRC_LO_CRC_POS                     (0x00u)
#define BQ7971X_CUST_CRC_LO_CRC_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * DEV_CONF1            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DEV_CONF1_OFFSET                        (0x002u)
#define BQ7971X_DEV_CONF1_POR_VAL                       (0x0Cu)

#define BQ7971X_DEV_CONF1_PROT_SLP_CYCLE_POS            (0x06u)
#define BQ7971X_DEV_CONF1_PROT_SLP_CYCLE_MSK            (0xc0u)

#define BQ7971X_DEV_CONF1_TWARN_THR_POS                 (0x05u)
#define BQ7971X_DEV_CONF1_TWARN_THR_MSK                 (0x20u)

#define BQ7971X_DEV_CONF1_CBTWARN_DIS_POS               (0x04u)
#define BQ7971X_DEV_CONF1_CBTWARN_DIS_MSK               (0x10u)

#define BQ7971X_DEV_CONF1_NO_ADJ_CB_POS                 (0x03u)
#define BQ7971X_DEV_CONF1_NO_ADJ_CB_MSK                 (0x08u)

#define BQ7971X_DEV_CONF1_NFAULT_EN_POS                 (0x02u)
#define BQ7971X_DEV_CONF1_NFAULT_EN_MSK                 (0x04u)

#define BQ7971X_DEV_CONF1_FTONE_EN_POS                  (0x01u)
#define BQ7971X_DEV_CONF1_FTONE_EN_MSK                  (0x02u)

#define BQ7971X_DEV_CONF1_HB_EN_POS                     (0x00u)
#define BQ7971X_DEV_CONF1_HB_EN_MSK                     (0x01u)

/* --------------------------------------------------------------------------
 * DEV_CONF2            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DEV_CONF2_OFFSET                        (0x003u)
#define BQ7971X_DEV_CONF2_POR_VAL                       (0x00u)

#define BQ7971X_DEV_CONF2_PTC_EN_POS                    (0x07u)
#define BQ7971X_DEV_CONF2_PTC_EN_MSK                    (0x80u)

#define BQ7971X_DEV_CONF2_BB_PIN_EN_POS                 (0x06u)
#define BQ7971X_DEV_CONF2_BB_PIN_EN_MSK                 (0x40u)

#define BQ7971X_DEV_CONF2_BB_PIN_LOC_POS                (0x04u)
#define BQ7971X_DEV_CONF2_BB_PIN_LOC_MSK                (0x30u)

#define BQ7971X_DEV_CONF2_NUM_CELL_POS                  (0x00u)
#define BQ7971X_DEV_CONF2_NUM_CELL_MSK                  (0x0Fu)

/* --------------------------------------------------------------------------
 * COMM_CONF            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_COMM_CONF_OFFSET                        (0x004u)
#define BQ7971X_COMM_CONF_POR_VAL                       (0x00u)

#define BQ7971X_COMM_CONF_RX_RLX_FAULT_POS              (0x05u)
#define BQ7971X_COMM_CONF_RX_RLX_FAULT_MSK              (0x20u)

#define BQ7971X_COMM_CONF_RX_RLX_THRESH_POS             (0x04u)
#define BQ7971X_COMM_CONF_RX_RLX_THRESH_MSK             (0x10u)

#define BQ7971X_COMM_CONF_RX_RLX_SAMP_POS               (0x03u)
#define BQ7971X_COMM_CONF_RX_RLX_SAMP_MSK               (0x08u)

#define BQ7971X_COMM_CONF_STK_RESP_DLY_POS              (0x01u)
#define BQ7971X_COMM_CONF_STK_RESP_DLY_MSK              (0x6u)

#define BQ7971X_COMM_CONF_TX_HOLD_OFF_POS               (0x00u)
#define BQ7971X_COMM_CONF_TX_HOLD_OFF_MSK               (0x01u)

/* --------------------------------------------------------------------------
 * BBVC_POSN1           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_BBVC_POSN1_OFFSET                       (0x005u)
#define BQ7971X_BBVC_POSN1_POR_VAL                      (0x00u)

#define BQ7971X_BBVC_POSN1_CELL18_POS                   (0x01u)
#define BQ7971X_BBVC_POSN1_CELL18_MSK                   (0x02u)

#define BQ7971X_BBVC_POSN1_CELL17_POS                   (0x00u)
#define BQ7971X_BBVC_POSN1_CELL17_MSK                   (0x01u)

/* --------------------------------------------------------------------------
 * BBVC_POSN2           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_BBVC_POSN2_OFFSET                       (0x006u)
#define BQ7971X_BBVC_POSN2_POR_VAL                      (0x00u)

#define BQ7971X_BBVC_POSN2_CELL16_POS                   (0x07u)
#define BQ7971X_BBVC_POSN2_CELL16_MSK                   (0x80u)

#define BQ7971X_BBVC_POSN2_CELL15_POS                   (0x06u)
#define BQ7971X_BBVC_POSN2_CELL15_MSK                   (0x40u)

#define BQ7971X_BBVC_POSN2_CELL14_POS                   (0x05u)
#define BQ7971X_BBVC_POSN2_CELL14_MSK                   (0x20u)

#define BQ7971X_BBVC_POSN2_CELL13_POS                   (0x04u)
#define BQ7971X_BBVC_POSN2_CELL13_MSK                   (0x10u)

#define BQ7971X_BBVC_POSN2_CELL12_POS                   (0x03u)
#define BQ7971X_BBVC_POSN2_CELL12_MSK                   (0x08u)

#define BQ7971X_BBVC_POSN2_CELL11_POS                   (0x02u)
#define BQ7971X_BBVC_POSN2_CELL11_MSK                   (0x04u)

#define BQ7971X_BBVC_POSN2_CELL10_POS                   (0x01u)
#define BQ7971X_BBVC_POSN2_CELL10_MSK                   (0x02u)

#define BQ7971X_BBVC_POSN2_CELL9_POS                    (0x00u)
#define BQ7971X_BBVC_POSN2_CELL9_MSK                    (0x01u)

/* --------------------------------------------------------------------------
 * BBVC_POSN3           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_BBVC_POSN3_OFFSET                       (0x007u)
#define BQ7971X_BBVC_POSN3_POR_VAL                      (0x00u)

#define BQ7971X_BBVC_POSN3_CELL8_POS                    (0x07u)
#define BQ7971X_BBVC_POSN3_CELL8_MSK                    (0x80u)

#define BQ7971X_BBVC_POSN3_CELL7_POS                    (0x06u)
#define BQ7971X_BBVC_POSN3_CELL7_MSK                    (0x40u)

#define BQ7971X_BBVC_POSN3_CELL6_POS                    (0x05u)
#define BQ7971X_BBVC_POSN3_CELL6_MSK                    (0x20u)

#define BQ7971X_BBVC_POSN3_CELL5_POS                    (0x04u)
#define BQ7971X_BBVC_POSN3_CELL5_MSK                    (0x10u)

#define BQ7971X_BBVC_POSN3_CELL4_POS                    (0x03u)
#define BQ7971X_BBVC_POSN3_CELL4_MSK                    (0x08u)

#define BQ7971X_BBVC_POSN3_CELL3_POS                    (0x02u)
#define BQ7971X_BBVC_POSN3_CELL3_MSK                    (0x04u)

#define BQ7971X_BBVC_POSN3_CELL2_POS                    (0x01u)
#define BQ7971X_BBVC_POSN3_CELL2_MSK                    (0x02u)

#define BQ7971X_BBVC_POSN3_CELL1_POS                    (0x00u)
#define BQ7971X_BBVC_POSN3_CELL1_MSK                    (0x01u)

/* --------------------------------------------------------------------------
 * ADC_CONF             (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_ADC_CONF_OFFSET                         (0x008u)
#define BQ7971X_ADC_CONF_POR_VAL                        (0x00u)

#define BQ7971X_ADC_CONF_ADC_DLY_POS                    (0x00u)
#define BQ7971X_ADC_CONF_ADC_DLY_MSK                    (0xFFu)

/* --------------------------------------------------------------------------
 * OV_THRESH            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OV_THRESH_OFFSET                        (0x009u)
#define BQ7971X_OV_THRESH_POR_VAL                       (0x00u)

#define BQ7971X_OV_THRESH_OV_THR_POS                    (0x00u)
#define BQ7971X_OV_THRESH_OV_THR_MSK                    (0xFFu)

/* --------------------------------------------------------------------------
 * UV_THRESH            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_UV_THRESH_OFFSET                        (0x00Au)
#define BQ7971X_UV_THRESH_POR_VAL                       (0x00u)

#define BQ7971X_UV_THRESH_UV_THR_POS                    (0x00u)
#define BQ7971X_UV_THRESH_UV_THR_MSK                    (0x3Fu)

/* --------------------------------------------------------------------------
 * OTUT_THRESH          (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OTUT_THRESH_OFFSET                      (0x00Bu)
#define BQ7971X_OTUT_THRESH_POR_VAL                     (0x00u)

#define BQ7971X_OTUT_THRESH_UT_THR_POS                  (0x05u)
#define BQ7971X_OTUT_THRESH_UT_THR_MSK                  (0xe0u)

#define BQ7971X_OTUT_THRESH_OT_THR_POS                  (0x00u)
#define BQ7971X_OTUT_THRESH_OT_THR_MSK                  (0x1Fu)

/* --------------------------------------------------------------------------
 * UV_DISABLE1          (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_UV_DISABLE1_OFFSET                      (0x00Cu)
#define BQ7971X_UV_DISABLE1_POR_VAL                     (0x00u)

#define BQ7971X_UV_DISABLE1_CELL18_POS                  (0x01u)
#define BQ7971X_UV_DISABLE1_CELL18_MSK                  (0x02u)

#define BQ7971X_UV_DISABLE1_CELL17_POS                  (0x00u)
#define BQ7971X_UV_DISABLE1_CELL17_MSK                  (0x01u)

/* --------------------------------------------------------------------------
 * UV_DISABLE2          (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_UV_DISABLE2_OFFSET                      (0x00Du)
#define BQ7971X_UV_DISABLE2_POR_VAL                     (0x00u)

#define BQ7971X_UV_DISABLE2_CELL16_POS                  (0x07u)
#define BQ7971X_UV_DISABLE2_CELL16_MSK                  (0x80u)

#define BQ7971X_UV_DISABLE2_CELL15_POS                  (0x06u)
#define BQ7971X_UV_DISABLE2_CELL15_MSK                  (0x40u)

#define BQ7971X_UV_DISABLE2_CELL14_POS                  (0x05u)
#define BQ7971X_UV_DISABLE2_CELL14_MSK                  (0x20u)

#define BQ7971X_UV_DISABLE2_CELL13_POS                  (0x04u)
#define BQ7971X_UV_DISABLE2_CELL13_MSK                  (0x10u)

#define BQ7971X_UV_DISABLE2_CELL12_POS                  (0x03u)
#define BQ7971X_UV_DISABLE2_CELL12_MSK                  (0x08u)

#define BQ7971X_UV_DISABLE2_CELL11_POS                  (0x02u)
#define BQ7971X_UV_DISABLE2_CELL11_MSK                  (0x04u)

#define BQ7971X_UV_DISABLE2_CELL10_POS                  (0x01u)
#define BQ7971X_UV_DISABLE2_CELL10_MSK                  (0x02u)

#define BQ7971X_UV_DISABLE2_CELL9_POS                   (0x00u)
#define BQ7971X_UV_DISABLE2_CELL9_MSK                   (0x01u)

/* --------------------------------------------------------------------------
 * UV_DISABLE3          (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_UV_DISABLE3_OFFSET                      (0x00Eu)
#define BQ7971X_UV_DISABLE3_POR_VAL                     (0x00u)

#define BQ7971X_UV_DISABLE3_CELL8_POS                   (0x07u)
#define BQ7971X_UV_DISABLE3_CELL8_MSK                   (0x80u)

#define BQ7971X_UV_DISABLE3_CELL7_POS                   (0x06u)
#define BQ7971X_UV_DISABLE3_CELL7_MSK                   (0x40u)

#define BQ7971X_UV_DISABLE3_CELL6_POS                   (0x05u)
#define BQ7971X_UV_DISABLE3_CELL6_MSK                   (0x20u)

#define BQ7971X_UV_DISABLE3_CELL5_POS                   (0x04u)
#define BQ7971X_UV_DISABLE3_CELL5_MSK                   (0x10u)

#define BQ7971X_UV_DISABLE3_CELL4_POS                   (0x03u)
#define BQ7971X_UV_DISABLE3_CELL4_MSK                   (0x08u)

#define BQ7971X_UV_DISABLE3_CELL3_POS                   (0x02u)
#define BQ7971X_UV_DISABLE3_CELL3_MSK                   (0x04u)

#define BQ7971X_UV_DISABLE3_CELL2_POS                   (0x01u)
#define BQ7971X_UV_DISABLE3_CELL2_MSK                   (0x02u)

#define BQ7971X_UV_DISABLE3_CELL1_POS                   (0x00u)
#define BQ7971X_UV_DISABLE3_CELL1_MSK                   (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_MSK1           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_MSK1_OFFSET                       (0x00Fu)
#define BQ7971X_FAULT_MSK1_POR_VAL                      (0x00u)

#define BQ7971X_FAULT_MSK1_MSK_OC_POS                   (0x07u)
#define BQ7971X_FAULT_MSK1_MSK_OC_MSK                   (0x80u)

#define BQ7971X_FAULT_MSK1_MSK_UT_POS                   (0x06u)
#define BQ7971X_FAULT_MSK1_MSK_UT_MSK                   (0x40u)

#define BQ7971X_FAULT_MSK1_MSK_OT_POS                   (0x05u)
#define BQ7971X_FAULT_MSK1_MSK_OT_MSK                   (0x20u)

#define BQ7971X_FAULT_MSK1_MSK_UV_POS                   (0x04u)
#define BQ7971X_FAULT_MSK1_MSK_UV_MSK                   (0x10u)

#define BQ7971X_FAULT_MSK1_MSK_OV_POS                   (0x03u)
#define BQ7971X_FAULT_MSK1_MSK_OV_MSK                   (0x08u)

#define BQ7971X_FAULT_MSK1_MSK_CB_POS                   (0x02u)
#define BQ7971X_FAULT_MSK1_MSK_CB_MSK                   (0x04u)

#define BQ7971X_FAULT_MSK1_MSK_SYS_POS                  (0x01u)
#define BQ7971X_FAULT_MSK1_MSK_SYS_MSK                  (0x02u)

#define BQ7971X_FAULT_MSK1_MSK_PWR_POS                  (0x00u)
#define BQ7971X_FAULT_MSK1_MSK_PWR_MSK                  (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_MSK2           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_MSK2_OFFSET                       (0x010u)
#define BQ7971X_FAULT_MSK2_POR_VAL                      (0x00u)

#define BQ7971X_FAULT_MSK2_MSK_ADC_POS                  (0x07u)
#define BQ7971X_FAULT_MSK2_MSK_ADC_MSK                  (0x80u)

#define BQ7971X_FAULT_MSK2_MSK_OTP_CRC_POS              (0x06u)
#define BQ7971X_FAULT_MSK2_MSK_OTP_CRC_MSK              (0x40u)

#define BQ7971X_FAULT_MSK2_MSK_OTP_DATA_POS             (0x05u)
#define BQ7971X_FAULT_MSK2_MSK_OTP_DATA_MSK             (0x20u)

#define BQ7971X_FAULT_MSK2_MSK_COMM_FCOMM_POS           (0x04u)
#define BQ7971X_FAULT_MSK2_MSK_COMM_FCOMM_MSK           (0x10u)

#define BQ7971X_FAULT_MSK2_MSK_COMM_FTONE_POS           (0x03u)
#define BQ7971X_FAULT_MSK2_MSK_COMM_FTONE_MSK           (0x08u)

#define BQ7971X_FAULT_MSK2_MSK_COMM_HB_POS              (0x02u)
#define BQ7971X_FAULT_MSK2_MSK_COMM_HB_MSK              (0x04u)

#define BQ7971X_FAULT_MSK2_MSK_COMM_DSY_POS             (0x01u)
#define BQ7971X_FAULT_MSK2_MSK_COMM_DSY_MSK             (0x02u)

#define BQ7971X_FAULT_MSK2_MSK_COMM_UART_POS            (0x00u)
#define BQ7971X_FAULT_MSK2_MSK_COMM_UART_MSK            (0x01u)

/* --------------------------------------------------------------------------
 * CS_ADC_CAL1          (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CS_ADC_CAL1_OFFSET                      (0x011u)
#define BQ7971X_CS_ADC_CAL1_POR_VAL                     (0x00u)

#define BQ7971X_CS_ADC_CAL1_GAINL_POS                   (0x00u)
#define BQ7971X_CS_ADC_CAL1_GAINL_MSK                   (0xFFu)

/* --------------------------------------------------------------------------
 * CS_ADC_CAL2          (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CS_ADC_CAL2_OFFSET                      (0x012u)
#define BQ7971X_CS_ADC_CAL2_POR_VAL                     (0x00u)

#define BQ7971X_CS_ADC_CAL2_GAINH_POS                   (0x05u)
#define BQ7971X_CS_ADC_CAL2_GAINH_MSK                   (0xE0u)

#define BQ7971X_CS_ADC_CAL2_OFFSET_POS                  (0x00u)
#define BQ7971X_CS_ADC_CAL2_OFFSET_MSK                  (0x1Fu)

/* --------------------------------------------------------------------------
 * OC_CONF1             (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OC_CONF1_OFFSET                         (0x013u)
#define BQ7971X_OC_CONF1_POR_VAL                        (0x00u)

#define BQ7971X_OC_CONF1_DEGLITCH_POS                   (0x05u)
#define BQ7971X_OC_CONF1_DEGLITCH_MSK                   (0xE0u)

#define BQ7971X_OC_CONF1_OCD_THR_POS                    (0x02u)
#define BQ7971X_OC_CONF1_OCD_THR_MSK                    (0x1cu)

#define BQ7971X_OC_CONF1_OCC_THR_POS                    (0x00u)
#define BQ7971X_OC_CONF1_OCC_THR_MSK                    (0x03u)

/* --------------------------------------------------------------------------
 * OC_CONF2             (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OC_CONF2_OFFSET                         (0x014u)
#define BQ7971X_OC_CONF2_POR_VAL                        (0x00u)

#define BQ7971X_OC_CONF2_DEGLITCH_POS                   (0x05u)
#define BQ7971X_OC_CONF2_DEGLITCH_MSK                   (0xE0u)

#define BQ7971X_OC_CONF2_OCD_THR_POS                    (0x02u)
#define BQ7971X_OC_CONF2_OCD_THR_MSK                    (0x1cu)

#define BQ7971X_OC_CONF2_OCC_THR_POS                    (0x00u)
#define BQ7971X_OC_CONF2_OCC_THR_MSK                    (0x03u)


/* --------------------------------------------------------------------------
 * CUST_MISC1           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CUST_MISC1_OFFSET                       (0x015u)
#define BQ7971X_CUST_MISC1_POR_VAL                      (0x00u)

#define BQ7971X_CUST_MISC1_DATA_POS                     (0x00u)
#define BQ7971X_CUST_MISC1_DATA_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * CUST_MISC2           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CUST_MISC2_OFFSET                       (0x016u)
#define BQ7971X_CUST_MISC2_POR_VAL                      (0x00u)

#define BQ7971X_CUST_MISC2_DATA_POS                     (0x00u)
#define BQ7971X_CUST_MISC2_DATA_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * CUST_MISC3           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CUST_MISC3_OFFSET                       (0x017u)
#define BQ7971X_CUST_MISC3_POR_VAL                      (0x00u)

#define BQ7971X_CUST_MISC3_DATA_POS                     (0x00u)
#define BQ7971X_CUST_MISC3_DATA_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * CUST_MISC4           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CUST_MISC4_OFFSET                       (0x018u)
#define BQ7971X_CUST_MISC4_POR_VAL                      (0x00u)

#define BQ7971X_CUST_MISC4_DATA_POS                     (0x00u)
#define BQ7971X_CUST_MISC4_DATA_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * CUST_MISC5           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CUST_MISC5_OFFSET                       (0x019u)
#define BQ7971X_CUST_MISC5_POR_VAL                      (0x00u)

#define BQ7971X_CUST_MISC5_DATA_POS                     (0x00u)
#define BQ7971X_CUST_MISC5_DATA_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * CUST_MISC6           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CUST_MISC6_OFFSET                       (0x01Au)
#define BQ7971X_CUST_MISC6_POR_VAL                      (0x00u)

#define BQ7971X_CUST_MISC6_DATA_POS                     (0x00u)
#define BQ7971X_CUST_MISC6_DATA_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * CUST_MISC7           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CUST_MISC7_OFFSET                       (0x01Bu)
#define BQ7971X_CUST_MISC7_POR_VAL                      (0x00u)

#define BQ7971X_CUST_MISC7_DATA_POS                     (0x00u)
#define BQ7971X_CUST_MISC7_DATA_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * CUST_MISC8           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CUST_MISC8_OFFSET                       (0x01Cu)
#define BQ7971X_CUST_MISC8_POR_VAL                      (0x00u)

#define BQ7971X_CUST_MISC8_DATA_POS                     (0x00u)
#define BQ7971X_CUST_MISC8_DATA_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * IDDQ_CONF            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_IDDQ_CONF_OFFSET                        (0x01Du)
#define BQ7971X_IDDQ_CONF_POR_VAL                       (0x00u)

#define BQ7971X_IDDQ_CONF_STATIC_POS                    (0x05u)
#define BQ7971X_IDDQ_CONF_STATIC_MSK                    (0xe0u)

#define BQ7971X_IDDQ_CONF_COMM_POS                      (0x00u)
#define BQ7971X_IDDQ_CONF_COMM_MSK                      (0x1Fu)

/* --------------------------------------------------------------------------
 * GPIO_CONF1           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO_CONF1_OFFSET                       (0x01Eu)
#define BQ7971X_GPIO_CONF1_POR_VAL                      (0x00u)

#define BQ7971X_GPIO_CONF1_SPI_EN_POS                   (0x06u)
#define BQ7971X_GPIO_CONF1_SPI_EN_MSK                   (0x40u)

#define BQ7971X_GPIO_CONF1_GPIO2_POS                    (0x03u)
#define BQ7971X_GPIO_CONF1_GPIO2_MSK                    (0x38u)

#define BQ7971X_GPIO_CONF1_GPIO1_POS                    (0x00u)
#define BQ7971X_GPIO_CONF1_GPIO1_MSK                    (0x07u)

/* --------------------------------------------------------------------------
 * GPIO_CONF2           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO_CONF2_OFFSET                       (0x01Fu)
#define BQ7971X_GPIO_CONF2_POR_VAL                      (0x00u)

#define BQ7971X_GPIO_CONF2_CS_RDY_EN_POS                (0x06u)
#define BQ7971X_GPIO_CONF2_CS_RDY_EN_MSK                (0x40u)

#define BQ7971X_GPIO_CONF2_GPIO4_POS                    (0x03u)
#define BQ7971X_GPIO_CONF2_GPIO4_MSK                    (0x38u)

#define BQ7971X_GPIO_CONF2_GPIO3_POS                    (0x00u)
#define BQ7971X_GPIO_CONF2_GPIO3_MSK                    (0x07u)

/* --------------------------------------------------------------------------
 * GPIO_CONF3           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO_CONF3_OFFSET                       (0x020u)
#define BQ7971X_GPIO_CONF3_POR_VAL                      (0x00u)

#define BQ7971X_GPIO_CONF3_GPIO6_POS                    (0x03u)
#define BQ7971X_GPIO_CONF3_GPIO6_MSK                    (0x38u)

#define BQ7971X_GPIO_CONF3_GPIO5_POS                    (0x00u)
#define BQ7971X_GPIO_CONF3_GPIO5_MSK                    (0x07u)

/* --------------------------------------------------------------------------
 * GPIO_CONF4           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO_CONF4_OFFSET                       (0x021u)
#define BQ7971X_GPIO_CONF4_POR_VAL                      (0x00u)

#define BQ7971X_GPIO_CONF4_GPIO8_POS                    (0x03u)
#define BQ7971X_GPIO_CONF4_GPIO8_MSK                    (0x38u)

#define BQ7971X_GPIO_CONF4_GPIO7_POS                    (0x00u)
#define BQ7971X_GPIO_CONF4_GPIO7_MSK                    (0x07u)

/* --------------------------------------------------------------------------
 * GPIO_CONF5           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO_CONF5_OFFSET                       (0x022u)
#define BQ7971X_GPIO_CONF5_POR_VAL                      (0x00u)

#define BQ7971X_GPIO_CONF5_GPIO10_POS                   (0x03u)
#define BQ7971X_GPIO_CONF5_GPIO10_MSK                   (0x38u)

#define BQ7971X_GPIO_CONF5_GPIO9_POS                    (0x00u)
#define BQ7971X_GPIO_CONF5_GPIO9_MSK                    (0x07u)

/* --------------------------------------------------------------------------
 * GPIO_CONF6           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO_CONF6_OFFSET                       (0x023u)
#define BQ7971X_GPIO_CONF6_POR_VAL                      (0x00u)

#define BQ7971X_GPIO_CONF6_GPIO11_POS                   (0x00u)
#define BQ7971X_GPIO_CONF6_GPIO11_MSK                   (0x07u)

/* --------------------------------------------------------------------------
 * VCELL_OFFSET1            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL_OFFSET1_OFFSET                    (0x024u)
#define BQ7971X_VCELL_OFFSET1_POR_VAL                   (0x00u)

#define BQ7971X_VCELL_OFFSET1_OFFSET2_POS               (0x04u)
#define BQ7971X_VCELL_OFFSET1_OFFSET2_MSK               (0xf0u)

#define BQ7971X_VCELL_OFFSET1_OFFSET1_POS               (0x00u)
#define BQ7971X_VCELL_OFFSET1_OFFSET1_MSK               (0x0Fu)

/* --------------------------------------------------------------------------
 * VCELL_OFFSET2            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL_OFFSET2_OFFSET                    (0x025u)
#define BQ7971X_VCELL_OFFSET2_POR_VAL                   (0x00u)

#define BQ7971X_VCELL_OFFSET2_OFFSET4_POS               (0x04u)
#define BQ7971X_VCELL_OFFSET2_OFFSET4_MSK               (0xf0u)

#define BQ7971X_VCELL_OFFSET2_OFFSET3_POS               (0x00u)
#define BQ7971X_VCELL_OFFSET2_OFFSET3_MSK               (0x0Fu)

/* --------------------------------------------------------------------------
 * VCELL_OFFSET3            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL_OFFSET3_OFFSET                    (0x026u)
#define BQ7971X_VCELL_OFFSET3_POR_VAL                   (0x00u)

#define BQ7971X_VCELL_OFFSET3_OFFSET6_POS               (0x04u)
#define BQ7971X_VCELL_OFFSET3_OFFSET6_MSK               (0xf0u)

#define BQ7971X_VCELL_OFFSET3_OFFSET5_POS               (0x00u)
#define BQ7971X_VCELL_OFFSET3_OFFSET5_MSK               (0x0Fu)

/* --------------------------------------------------------------------------
 * VCELL_OFFSET4            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL_OFFSET4_OFFSET                    (0x027u)
#define BQ7971X_VCELL_OFFSET4_POR_VAL                   (0x00u)

#define BQ7971X_VCELL_OFFSET4_OFFSET8_POS               (0x04u)
#define BQ7971X_VCELL_OFFSET4_OFFSET8_MSK               (0xf0u)

#define BQ7971X_VCELL_OFFSET4_OFFSET7_POS               (0x00u)
#define BQ7971X_VCELL_OFFSET4_OFFSET7_MSK               (0x0Fu)

/* --------------------------------------------------------------------------
 * VCELL_OFFSET5            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL_OFFSET5_OFFSET                    (0x028u)
#define BQ7971X_VCELL_OFFSET5_POR_VAL                   (0x00u)

#define BQ7971X_VCELL_OFFSET5_OFFSET10_POS              (0x04u)
#define BQ7971X_VCELL_OFFSET5_OFFSET10_MSK              (0xf0u)

#define BQ7971X_VCELL_OFFSET5_OFFSET9_POS               (0x00u)
#define BQ7971X_VCELL_OFFSET5_OFFSET9_MSK               (0x0Fu)

/* --------------------------------------------------------------------------
 * VCELL_OFFSET6            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL_OFFSET6_OFFSET                    (0x029u)
#define BQ7971X_VCELL_OFFSET6_POR_VAL                   (0x00u)

#define BQ7971X_VCELL_OFFSET6_OFFSET12_POS              (0x04u)
#define BQ7971X_VCELL_OFFSET6_OFFSET12_MSK              (0xf0u)

#define BQ7971X_VCELL_OFFSET6_OFFSET11_POS              (0x00u)
#define BQ7971X_VCELL_OFFSET6_OFFSET11_MSK              (0x0Fu)

/* --------------------------------------------------------------------------
 * VCELL_OFFSET7            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL_OFFSET7_OFFSET                    (0x02Au)
#define BQ7971X_VCELL_OFFSET7_POR_VAL                   (0x00u)

#define BQ7971X_VCELL_OFFSET7_OFFSET14_POS              (0x04u)
#define BQ7971X_VCELL_OFFSET7_OFFSET14_MSK              (0xf0u)

#define BQ7971X_VCELL_OFFSET7_OFFSET13_POS              (0x00u)
#define BQ7971X_VCELL_OFFSET7_OFFSET13_MSK              (0x0Fu)

/* --------------------------------------------------------------------------
 * VCELL_OFFSET8            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL_OFFSET8_OFFSET                    (0x02Bu)
#define BQ7971X_VCELL_OFFSET8_POR_VAL                   (0x00u)

#define BQ7971X_VCELL_OFFSET8_OFFSET16_POS              (0x04u)
#define BQ7971X_VCELL_OFFSET8_OFFSET16_MSK              (0xf0u)

#define BQ7971X_VCELL_OFFSET8_OFFSET15_POS              (0x00u)
#define BQ7971X_VCELL_OFFSET8_OFFSET15_MSK              (0x0Fu)

/* --------------------------------------------------------------------------
 * VCELL_OFFSET9            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL_OFFSET9_OFFSET                    (0x02Cu)
#define BQ7971X_VCELL_OFFSET9_POR_VAL                   (0x00u)

#define BQ7971X_VCELL_OFFSET9_OFFSET18_POS              (0x04u)
#define BQ7971X_VCELL_OFFSET9_OFFSET18_MSK              (0xf0u)

#define BQ7971X_VCELL_OFFSET9_OFFSET17_POS              (0x00u)
#define BQ7971X_VCELL_OFFSET9_OFFSET17_MSK              (0x0Fu)

/* --------------------------------------------------------------------------
 * OTP_SPARE3           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OTP_SPARE3_OFFSET                       (0x02Du)
#define BQ7971X_OTP_SPARE3_POR_VAL                      (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE2           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OTP_SPARE2_OFFSET                       (0x02Eu)
#define BQ7971X_OTP_SPARE2_POR_VAL                      (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE1           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OTP_SPARE1_OFFSET                       (0x02Fu)
#define BQ7971X_OTP_SPARE1_POR_VAL                      (0x00u)

/* --------------------------------------------------------------------------
 * OTP_PROG_UNLOCK1             (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OTP_PROG_UNLOCK1_OFFSET                 (0x300u)
#define BQ7971X_OTP_PROG_UNLOCK1_POR_VAL                (0x00u)

#define BQ7971X_OTP_PROG_UNLOCK1_CODE_POS               (0x00u)
#define BQ7971X_OTP_PROG_UNLOCK1_CODE_MSK               (0xFFu)

/* --------------------------------------------------------------------------
 * DIR0_ADDR            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIR0_ADDR_OFFSET                        (0x306u)
#define BQ7971X_DIR0_ADDR_POR_VAL                       (0x00u)

#define BQ7971X_DIR0_ADDR_ADDRESS_POS                   (0x00u)
#define BQ7971X_DIR0_ADDR_ADDRESS_MSK                   (0x3Fu)

/* --------------------------------------------------------------------------
 * DIR1_ADDR            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIR1_ADDR_OFFSET                        (0x307u)
#define BQ7971X_DIR1_ADDR_POR_VAL                       (0x00u)

#define BQ7971X_DIR1_ADDR_ADDRESS_POS                   (0x00u)
#define BQ7971X_DIR1_ADDR_ADDRESS_MSK                   (0x3Fu)

/* --------------------------------------------------------------------------
 * COMM_CTRL            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_COMM_CTRL_OFFSET                        (0x308u)
#define BQ7971X_COMM_CTRL_POR_VAL                       (0x00u)

#define BQ7971X_COMM_CTRL_TOP_STACK_POS                 (0x00u)
#define BQ7971X_COMM_CTRL_TOP_STACK_MSK                 (0x01u)

/* --------------------------------------------------------------------------
 * CONTROL1             (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CONTROL1_OFFSET                         (0x309u)
#define BQ7971X_CONTROL1_POR_VAL                        (0x00u)

#define BQ7971X_CONTROL1_DIR_SEL_POS                    (0x07u)
#define BQ7971X_CONTROL1_DIR_SEL_MSK                    (0x80u)

#define BQ7971X_CONTROL1_SEND_SD_HW_RST_POS             (0x06u)
#define BQ7971X_CONTROL1_SEND_SD_HW_RST_MSK             (0x40u)

#define BQ7971X_CONTROL1_SEND_WAKE_POS                  (0x05u)
#define BQ7971X_CONTROL1_SEND_WAKE_MSK                  (0x20u)

#define BQ7971X_CONTROL1_SEND_SLPTOACT_POS              (0x04u)
#define BQ7971X_CONTROL1_SEND_SLPTOACT_MSK              (0x10u)

#define BQ7971X_CONTROL1_GOTO_SHUTDOWN_POS              (0x03u)
#define BQ7971X_CONTROL1_GOTO_SHUTDOWN_MSK              (0x08u)

#define BQ7971X_CONTROL1_GOTO_SLEEP_POS                 (0x02u)
#define BQ7971X_CONTROL1_GOTO_SLEEP_MSK                 (0x04u)

#define BQ7971X_CONTROL1_SOFT_RESET_POS                 (0x01u)
#define BQ7971X_CONTROL1_SOFT_RESET_MSK                 (0x02u)

#define BQ7971X_CONTROL1_ADDR_WR_POS                    (0x00u)
#define BQ7971X_CONTROL1_ADDR_WR_MSK                    (0x01u)

/* --------------------------------------------------------------------------
 * CONTROL2             (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CONTROL2_OFFSET                         (0x30Au)
#define BQ7971X_CONTROL2_POR_VAL                        (0x01u)

#define BQ7971X_CONTROL2_PROG_GO_POS                    (0x03u)
#define BQ7971X_CONTROL2_PROG_GO_MSK                    (0x08u)

#define BQ7971X_CONTROL2_I2C_EN_POS                     (0x02u)
#define BQ7971X_CONTROL2_I2C_EN_MSK                     (0x04u)

#define BQ7971X_CONTROL2_TBD_RSVD_POS                   (0x01u)
#define BQ7971X_CONTROL2_TBD_RSVD_MSK                   (0x02u)

#define BQ7971X_CONTROL2_TSREF_EN_POS                   (0x00u)
#define BQ7971X_CONTROL2_TSREF_EN_MSK                   (0x01u)

/* --------------------------------------------------------------------------
 * ADC_CTRL1            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_ADC_CTRL1_OFFSET                        (0x310u)
#define BQ7971X_ADC_CTRL1_POR_VAL                       (0x00u)

#define BQ7971X_ADC_CTRL1_V_I_SYNC_POS                  (0x07u)
#define BQ7971X_ADC_CTRL1_V_I_SYNC_MSK                  (0x80u)

#define BQ7971X_ADC_CTRL1_LPF_CS_POS                    (0x04u)
#define BQ7971X_ADC_CTRL1_LPF_CS_MSK                    (0x70u)

#define BQ7971X_ADC_CTRL1_LPF_CS_EN_POS                 (0x03u)
#define BQ7971X_ADC_CTRL1_LPF_CS_EN_MSK                 (0x08u)

#define BQ7971X_ADC_CTRL1_CS_DR_POS                     (0x01u)
#define BQ7971X_ADC_CTRL1_CS_DR_MSK                     (0x6u)

#define BQ7971X_ADC_CTRL1_GP_DR_POS                     (0x00u)
#define BQ7971X_ADC_CTRL1_GP_DR_MSK                     (0x01u)

/* --------------------------------------------------------------------------
 * ADC_CTRL2            (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7971X_ADC_CTRL2_OFFSET                        (0x311u)
#define BQ7971X_ADC_CTRL2_POR_VAL                       (0x00u)

#define BQ7971X_ADC_CTRL2_FREEZE_EN_POS                 (0x07u)
#define BQ7971X_ADC_CTRL2_FREEZE_EN_MSK                 (0x80u)

#define BQ7971X_ADC_CTRL2_LPF_VCELL_POS                 (0x04u)
#define BQ7971X_ADC_CTRL2_LPF_VCELL_MSK                 (0x70u)

#define BQ7971X_ADC_CTRL2_LPF_VCELL_EN_POS              (0x03u)
#define BQ7971X_ADC_CTRL2_LPF_VCELL_EN_MSK              (0x08u)

#define BQ7971X_ADC_CTRL2_ADC_MODE_POS                  (0x01u)
#define BQ7971X_ADC_CTRL2_ADC_MODE_MSK                  (0x6u)

#define BQ7971X_ADC_CTRL2_ADC_GO_POS                    (0x00u)
#define BQ7971X_ADC_CTRL2_ADC_GO_MSK                    (0x01u)

/* --------------------------------------------------------------------------
 * ADC_CTRL3            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_ADC_CTRL3_OFFSET                        (0x312u)
#define BQ7971X_ADC_CTRL3_POR_VAL                       (0x00u)

#define BQ7971X_ADC_CTRL3_FLIP_RESET_POS                (0x06u)
#define BQ7971X_ADC_CTRL3_FLIP_RESET_MSK                (0x40u)

#define BQ7971X_ADC_CTRL3_DIAG_VCELL_GPIO_POS           (0x05u)
#define BQ7971X_ADC_CTRL3_DIAG_VCELL_GPIO_MSK           (0x20u)

#define BQ7971X_ADC_CTRL3_DIAG_ANA_SEL_POS              (0x00u)
#define BQ7971X_ADC_CTRL3_DIAG_ANA_SEL_MSK              (0x1Fu)

/* --------------------------------------------------------------------------
 * ADC_CTRL4            (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7971X_ADC_CTRL4_OFFSET                        (0x313u)
#define BQ7971X_ADC_CTRL4_POR_VAL                       (0x00u)

#define BQ7971X_ADC_CTRL4_DIAG_D1D2_SEL_POS             (0x01u)
#define BQ7971X_ADC_CTRL4_DIAG_D1D2_SEL_MSK             (0x0Eu)

#define BQ7971X_ADC_CTRL4_DIAG_MEAS_GO_POS              (0x00u)
#define BQ7971X_ADC_CTRL4_DIAG_MEAS_GO_MSK              (0x01u)

/* --------------------------------------------------------------------------
 * DIAG_ADC_CTRL1           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIAG_ADC_CTRL1_OFFSET                   (0x314u)
#define BQ7971X_DIAG_ADC_CTRL1_POR_VAL                  (0x00u)

#define BQ7971X_DIAG_ADC_CTRL1_VCELL_FILT_POS           (0x07u)
#define BQ7971X_DIAG_ADC_CTRL1_VCELL_FILT_MSK           (0x80u)

#define BQ7971X_DIAG_ADC_CTRL1_BB_THR_POS               (0x05u)
#define BQ7971X_DIAG_ADC_CTRL1_BB_THR_MSK               (0x60u)

#define BQ7971X_DIAG_ADC_CTRL1_VCELL_THR_POS            (0x00u)
#define BQ7971X_DIAG_ADC_CTRL1_VCELL_THR_MSK            (0x1Fu)

/* --------------------------------------------------------------------------
 * DIAG_ADC_CTRL2           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIAG_ADC_CTRL2_OFFSET                   (0x315u)
#define BQ7971X_DIAG_ADC_CTRL2_POR_VAL                  (0x00u)

#define BQ7971X_DIAG_ADC_CTRL2_GPIO_THR_POS             (0x00u)
#define BQ7971X_DIAG_ADC_CTRL2_GPIO_THR_MSK             (0x07u)

/* --------------------------------------------------------------------------
 * DIAG_ADC_CTRL3           (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIAG_ADC_CTRL3_OFFSET                   (0x316u)
#define BQ7971X_DIAG_ADC_CTRL3_POR_VAL                  (0x00u)

#define BQ7971X_DIAG_ADC_CTRL3_ACOMP_MPFLT_INJ_POS      (0x06u)
#define BQ7971X_DIAG_ADC_CTRL3_ACOMP_MPFLT_INJ_MSK      (0x40u)

#define BQ7971X_DIAG_ADC_CTRL3_DCOMP_MPFLT_INJ_POS      (0x05u)
#define BQ7971X_DIAG_ADC_CTRL3_DCOMP_MPFLT_INJ_MSK      (0x20u)

#define BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_POS          (0x04u)
#define BQ7971X_DIAG_ADC_CTRL3_DIAG_DIG_EN_MSK          (0x10u)

#define BQ7971X_DIAG_ADC_CTRL3_BB_MEAS_MODE_POS         (0x03u)
#define BQ7971X_DIAG_ADC_CTRL3_BB_MEAS_MODE_MSK         (0x08u)

#define BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_MODE_POS        (0x01u)
#define BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_MODE_MSK        (0x6u)

#define BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_MODE_BOTH       (0x6u)

#define BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_GO_POS          (0x00u)
#define BQ7971X_DIAG_ADC_CTRL3_DIAG_ANA_GO_MSK          (0x01u)

/* --------------------------------------------------------------------------
 * DIAG_MISC_CTRL1          (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIAG_MISC_CTRL1_OFFSET                  (0x317u)
#define BQ7971X_DIAG_MISC_CTRL1_POR_VAL                 (0x00u)

#define BQ7971X_DIAG_MISC_CTRL1_SPI_LOOPBACK_POS        (0x06u)
#define BQ7971X_DIAG_MISC_CTRL1_SPI_LOOPBACK_MSK        (0x40u)

#define BQ7971X_DIAG_MISC_CTRL1_FLIP_TR_CRC_POS         (0x05u)
#define BQ7971X_DIAG_MISC_CTRL1_FLIP_TR_CRC_MSK         (0x20u)

#define BQ7971X_DIAG_MISC_CTRL1_FLIP_FACT_CRC_POS       (0x04u)
#define BQ7971X_DIAG_MISC_CTRL1_FLIP_FACT_CRC_MSK       (0x10u)

#define BQ7971X_DIAG_MISC_CTRL1_MARGIN_MODE_POS         (0x01u)
#define BQ7971X_DIAG_MISC_CTRL1_MARGIN_MODE_MSK         (0xeu)

#define BQ7971X_DIAG_MISC_CTRL1_MARGIN_GO_POS           (0x00u)
#define BQ7971X_DIAG_MISC_CTRL1_MARGIN_GO_MSK           (0x01u)

/* --------------------------------------------------------------------------
 * DIAG_MISC_CTRL2          (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIAG_MISC_CTRL2_OFFSET                  (0x318u)
#define BQ7971X_DIAG_MISC_CTRL2_POR_VAL                 (0x00u)

#define BQ7971X_DIAG_MISC_CTRL2_CSOW_CURRENT_EN_POS     (0x04u)
#define BQ7971X_DIAG_MISC_CTRL2_CSOW_CURRENT_EN_MSK     (0x10u)

#define BQ7971X_DIAG_MISC_CTRL2_CBFETOW_MPFLT_INJ_POS   (0x03u)
#define BQ7971X_DIAG_MISC_CTRL2_CBFETOW_MPFLT_INJ_MSK   (0x08u)

#define BQ7971X_DIAG_MISC_CTRL2_DIAG_CBFETOW_GO_POS     (0x02u)
#define BQ7971X_DIAG_MISC_CTRL2_DIAG_CBFETOW_GO_MSK     (0x04u)

#define BQ7971X_DIAG_MISC_CTRL2_PWRBIST_NO_RST_POS      (0x01u)
#define BQ7971X_DIAG_MISC_CTRL2_PWRBIST_NO_RST_MSK      (0x02u)

#define BQ7971X_DIAG_MISC_CTRL2_PWRBIST_GO_POS          (0x00u)
#define BQ7971X_DIAG_MISC_CTRL2_PWRBIST_GO_MSK          (0x01u)

/* --------------------------------------------------------------------------
 * CB_CELL18_CTRL           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL18_CTRL_OFFSET                   (0x320u)
#define BQ7971X_CB_CELL18_CTRL_POR_VAL                  (0x00u)

#define BQ7971X_CB_CELL18_CTRL_TIME_POS                 (0x00u)
#define BQ7971X_CB_CELL18_CTRL_TIME_MSK                 (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL17_CTRL           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL17_CTRL_OFFSET                   (0x321u)
#define BQ7971X_CB_CELL17_CTRL_POR_VAL                  (0x00u)

#define BQ7971X_CB_CELL17_CTRL_TIME_POS                 (0x00u)
#define BQ7971X_CB_CELL17_CTRL_TIME_MSK                 (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL16_CTRL           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL16_CTRL_OFFSET                   (0x322u)
#define BQ7971X_CB_CELL16_CTRL_POR_VAL                  (0x00u)

#define BQ7971X_CB_CELL16_CTRL_TIME_POS                 (0x00u)
#define BQ7971X_CB_CELL16_CTRL_TIME_MSK                 (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL15_CTRL           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL15_CTRL_OFFSET                   (0x323u)
#define BQ7971X_CB_CELL15_CTRL_POR_VAL                  (0x00u)

#define BQ7971X_CB_CELL15_CTRL_TIME_POS                 (0x00u)
#define BQ7971X_CB_CELL15_CTRL_TIME_MSK                 (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL14_CTRL           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL14_CTRL_OFFSET                   (0x324u)
#define BQ7971X_CB_CELL14_CTRL_POR_VAL                  (0x00u)

#define BQ7971X_CB_CELL14_CTRL_TIME_POS                 (0x00u)
#define BQ7971X_CB_CELL14_CTRL_TIME_MSK                 (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL13_CTRL           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL13_CTRL_OFFSET                   (0x325u)
#define BQ7971X_CB_CELL13_CTRL_POR_VAL                  (0x00u)

#define BQ7971X_CB_CELL13_CTRL_TIME_POS                 (0x00u)
#define BQ7971X_CB_CELL13_CTRL_TIME_MSK                 (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL12_CTRL           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL12_CTRL_OFFSET                   (0x326u)
#define BQ7971X_CB_CELL12_CTRL_POR_VAL                  (0x00u)

#define BQ7971X_CB_CELL12_CTRL_TIME_POS                 (0x00u)
#define BQ7971X_CB_CELL12_CTRL_TIME_MSK                 (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL11_CTRL           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL11_CTRL_OFFSET                   (0x327u)
#define BQ7971X_CB_CELL11_CTRL_POR_VAL                  (0x00u)

#define BQ7971X_CB_CELL11_CTRL_TIME_POS                 (0x00u)
#define BQ7971X_CB_CELL11_CTRL_TIME_MSK                 (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL10_CTRL           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL10_CTRL_OFFSET                   (0x328u)
#define BQ7971X_CB_CELL10_CTRL_POR_VAL                  (0x00u)

#define BQ7971X_CB_CELL10_CTRL_TIME_POS                 (0x00u)
#define BQ7971X_CB_CELL10_CTRL_TIME_MSK                 (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL9_CTRL            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL9_CTRL_OFFSET                    (0x329u)
#define BQ7971X_CB_CELL9_CTRL_POR_VAL                   (0x00u)

#define BQ7971X_CB_CELL9_CTRL_TIME_POS                  (0x00u)
#define BQ7971X_CB_CELL9_CTRL_TIME_MSK                  (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL8_CTRL            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL8_CTRL_OFFSET                    (0x32Au)
#define BQ7971X_CB_CELL8_CTRL_POR_VAL                   (0x00u)

#define BQ7971X_CB_CELL8_CTRL_TIME_POS                  (0x00u)
#define BQ7971X_CB_CELL8_CTRL_TIME_MSK                  (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL7_CTRL            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL7_CTRL_OFFSET                    (0x32Bu)
#define BQ7971X_CB_CELL7_CTRL_POR_VAL                   (0x00u)

#define BQ7971X_CB_CELL7_CTRL_TIME_POS                  (0x00u)
#define BQ7971X_CB_CELL7_CTRL_TIME_MSK                  (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL6_CTRL            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL6_CTRL_OFFSET                    (0x32Cu)
#define BQ7971X_CB_CELL6_CTRL_POR_VAL                   (0x00u)

#define BQ7971X_CB_CELL6_CTRL_TIME_POS                  (0x00u)
#define BQ7971X_CB_CELL6_CTRL_TIME_MSK                  (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL5_CTRL            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL5_CTRL_OFFSET                    (0x32Du)
#define BQ7971X_CB_CELL5_CTRL_POR_VAL                   (0x00u)

#define BQ7971X_CB_CELL5_CTRL_TIME_POS                  (0x00u)
#define BQ7971X_CB_CELL5_CTRL_TIME_MSK                  (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL4_CTRL            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL4_CTRL_OFFSET                    (0x32Eu)
#define BQ7971X_CB_CELL4_CTRL_POR_VAL                   (0x00u)

#define BQ7971X_CB_CELL4_CTRL_TIME_POS                  (0x00u)
#define BQ7971X_CB_CELL4_CTRL_TIME_MSK                  (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL3_CTRL            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL3_CTRL_OFFSET                    (0x32Fu)
#define BQ7971X_CB_CELL3_CTRL_POR_VAL                   (0x00u)

#define BQ7971X_CB_CELL3_CTRL_TIME_POS                  (0x00u)
#define BQ7971X_CB_CELL3_CTRL_TIME_MSK                  (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL2_CTRL            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL2_CTRL_OFFSET                    (0x330u)
#define BQ7971X_CB_CELL2_CTRL_POR_VAL                   (0x00u)

#define BQ7971X_CB_CELL2_CTRL_TIME_POS                  (0x00u)
#define BQ7971X_CB_CELL2_CTRL_TIME_MSK                  (0x1Fu)

/* --------------------------------------------------------------------------
 * CB_CELL1_CTRL            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CB_CELL1_CTRL_OFFSET                    (0x331u)
#define BQ7971X_CB_CELL1_CTRL_POR_VAL                   (0x00u)

#define BQ7971X_CB_CELL1_CTRL_TIME_POS                  (0x00u)
#define BQ7971X_CB_CELL1_CTRL_TIME_MSK                  (0x1Fu)

/* --------------------------------------------------------------------------
 * VCBDONE_THRESH           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCBDONE_THRESH_OFFSET                   (0x332u)
#define BQ7971X_VCBDONE_THRESH_POR_VAL                  (0x00u)

#define BQ7971X_VCBDONE_THRESH_CB_THR_POS               (0x00u)
#define BQ7971X_VCBDONE_THRESH_CB_THR_MSK               (0x7fu)

/* --------------------------------------------------------------------------
 * OTCB_THRESH          (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OTCB_THRESH_OFFSET                      (0x333u)
#define BQ7971X_OTCB_THRESH_POR_VAL                     (0x00u)

#define BQ7971X_OTCB_THRESH_OTCB_THR_POS                (0x00u)
#define BQ7971X_OTCB_THRESH_OTCB_THR_MSK                (0x0Fu)

/* --------------------------------------------------------------------------
 * OVUV_CTRL1           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OVUV_CTRL1_OFFSET                       (0x334u)
#define BQ7971X_OVUV_CTRL1_POR_VAL                      (0x00u)

#define BQ7971X_OVUV_CTRL1_OVUV_THR_LOCK_POS            (0x05u)
#define BQ7971X_OVUV_CTRL1_OVUV_THR_LOCK_MSK            (0x60u)

#define BQ7971X_OVUV_CTRL1_OVUV_LOCK_POS                (0x00u)
#define BQ7971X_OVUV_CTRL1_OVUV_LOCK_MSK                (0x1Fu)

/* --------------------------------------------------------------------------
 * OVUV_CTRL2           (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OVUV_CTRL2_OFFSET                       (0x335u)
#define BQ7971X_OVUV_CTRL2_POR_VAL                      (0x00u)

#define BQ7971X_OVUV_CTRL2_OVUV_MODE_POS                (0x01u)
#define BQ7971X_OVUV_CTRL2_OVUV_MODE_MSK                (0x6u)

#define BQ7971X_OVUV_CTRL2_OVUV_GO_POS                  (0x00u)
#define BQ7971X_OVUV_CTRL2_OVUV_GO_MSK                  (0x01u)

/* --------------------------------------------------------------------------
 * OTUT_CTRL1           (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OTUT_CTRL1_OFFSET                       (0x336u)
#define BQ7971X_OTUT_CTRL1_POR_VAL                      (0x00u)

#define BQ7971X_OTUT_CTRL1_OTUT_THR_LOCK_POS            (0x04u)
#define BQ7971X_OTUT_CTRL1_OTUT_THR_LOCK_MSK            (0x30u)

#define BQ7971X_OTUT_CTRL1_OTUT_LOCK_POS                (0x00u)
#define BQ7971X_OTUT_CTRL1_OTUT_LOCK_MSK                (0x0Fu)

/* --------------------------------------------------------------------------
 * OTUT_CTRL2           (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OTUT_CTRL2_OFFSET                       (0x337u)
#define BQ7971X_OTUT_CTRL2_POR_VAL                      (0x00u)

#define BQ7971X_OTUT_CTRL2_OTUT_MODE_POS                (0x01u)
#define BQ7971X_OTUT_CTRL2_OTUT_MODE_MSK                (0x6u)

#define BQ7971X_OTUT_CTRL2_OTUT_GO_POS                  (0x00u)
#define BQ7971X_OTUT_CTRL2_OTUT_GO_MSK                  (0x01u)

/* --------------------------------------------------------------------------
 * BAL_CTRL1            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_BAL_CTRL1_OFFSET                        (0x338u)
#define BQ7971X_BAL_CTRL1_POR_VAL                       (0x00u)

#define BQ7971X_BAL_CTRL1_PWM_POS                       (0x00u)
#define BQ7971X_BAL_CTRL1_PWM_MSK                       (0x3Fu)

/* --------------------------------------------------------------------------
 * BAL_CTRL2            (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7971X_BAL_CTRL2_OFFSET                        (0x339u)
#define BQ7971X_BAL_CTRL2_POR_VAL                       (0x00u)

#define BQ7971X_BAL_CTRL2_MASK                          (0x3Fu)

#define BQ7971X_BAL_CTRL2_CB_PAUSE_POS                  (0x05u)
#define BQ7971X_BAL_CTRL2_CB_PAUSE_MSK                  (0x20u)

#define BQ7971X_BAL_CTRL2_FLTSTOP_EN_POS                (0x04u)
#define BQ7971X_BAL_CTRL2_FLTSTOP_EN_MSK                (0x10u)

#define BQ7971X_BAL_CTRL2_OTCB_EN_POS                   (0x03u)
#define BQ7971X_BAL_CTRL2_OTCB_EN_MSK                   (0x08u)

#define BQ7971X_BAL_CTRL2_BAL_ACT_POS                   (0x02u)
#define BQ7971X_BAL_CTRL2_BAL_ACT_MSK                   (0x04u)

#define BQ7971X_BAL_CTRL2_AUTO_BAL_POS                  (0x01u)
#define BQ7971X_BAL_CTRL2_AUTO_BAL_MSK                  (0x02u)

#define BQ7971X_BAL_CTRL2_BAL_GO_POS                    (0x00u)
#define BQ7971X_BAL_CTRL2_BAL_GO_MSK                    (0x01u)

/* --------------------------------------------------------------------------
 * BAL_CTRL3            (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7971X_BAL_CTRL3_OFFSET                        (0x33Au)
#define BQ7971X_BAL_CTRL3_POR_VAL                       (0x00u)

#define BQ7971X_BAL_CTRL3_BAL_TIME_SEL_POS              (0x01u)
#define BQ7971X_BAL_CTRL3_BAL_TIME_SEL_MSK              (0x3eu)

#define BQ7971X_BAL_CTRL3_BAL_TIME_GO_POS               (0x00u)
#define BQ7971X_BAL_CTRL3_BAL_TIME_GO_MSK               (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_RST1           (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_RST1_OFFSET                       (0x340u)
#define BQ7971X_FAULT_RST1_POR_VAL                      (0x00u)

#define BQ7971X_FAULT_RST1_RST_OC_POS                   (0x07u)
#define BQ7971X_FAULT_RST1_RST_OC_MSK                   (0x80u)

#define BQ7971X_FAULT_RST1_RST_UT_POS                   (0x06u)
#define BQ7971X_FAULT_RST1_RST_UT_MSK                   (0x40u)

#define BQ7971X_FAULT_RST1_RST_OT_POS                   (0x05u)
#define BQ7971X_FAULT_RST1_RST_OT_MSK                   (0x20u)

#define BQ7971X_FAULT_RST1_RST_UV_POS                   (0x04u)
#define BQ7971X_FAULT_RST1_RST_UV_MSK                   (0x10u)

#define BQ7971X_FAULT_RST1_RST_OV_POS                   (0x03u)
#define BQ7971X_FAULT_RST1_RST_OV_MSK                   (0x08u)

#define BQ7971X_FAULT_RST1_RST_CB_POS                   (0x02u)
#define BQ7971X_FAULT_RST1_RST_CB_MSK                   (0x04u)

#define BQ7971X_FAULT_RST1_RST_SYS_POS                  (0x01u)
#define BQ7971X_FAULT_RST1_RST_SYS_MSK                  (0x02u)

#define BQ7971X_FAULT_RST1_RST_PWR_POS                  (0x00u)
#define BQ7971X_FAULT_RST1_RST_PWR_MSK                  (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_RST2           (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_RST2_OFFSET                       (0x341u)
#define BQ7971X_FAULT_RST2_POR_VAL                      (0x00u)

#define BQ7971X_FAULT_RST2_RST_ADC_POS                  (0x07u)
#define BQ7971X_FAULT_RST2_RST_ADC_MSK                  (0x80u)

#define BQ7971X_FAULT_RST2_RST_OTP_CRC_POS              (0x06u)
#define BQ7971X_FAULT_RST2_RST_OTP_CRC_MSK              (0x40u)

#define BQ7971X_FAULT_RST2_RST_OTP_DATA_POS             (0x05u)
#define BQ7971X_FAULT_RST2_RST_OTP_DATA_MSK             (0x20u)

#define BQ7971X_FAULT_RST2_RST_COMM_FCOMM_POS           (0x04u)
#define BQ7971X_FAULT_RST2_RST_COMM_FCOMM_MSK           (0x10u)

#define BQ7971X_FAULT_RST2_RST_COMM_FTONE_POS           (0x03u)
#define BQ7971X_FAULT_RST2_RST_COMM_FTONE_MSK           (0x08u)

#define BQ7971X_FAULT_RST2_RST_COMM_HB_POS              (0x02u)
#define BQ7971X_FAULT_RST2_RST_COMM_HB_MSK              (0x04u)

#define BQ7971X_FAULT_RST2_RST_COMM_DSY_POS             (0x01u)
#define BQ7971X_FAULT_RST2_RST_COMM_DSY_MSK             (0x02u)

#define BQ7971X_FAULT_RST2_RST_COMM_UART_POS            (0x00u)
#define BQ7971X_FAULT_RST2_RST_COMM_UART_MSK            (0x01u)

/* --------------------------------------------------------------------------
 * OTP_ECC_TEST             (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OTP_ECC_TEST_OFFSET                     (0x350u)
#define BQ7971X_OTP_ECC_TEST_POR_VAL                    (0x00u)

#define BQ7971X_OTP_ECC_TEST_DED_SEC_POS                (0x02u)
#define BQ7971X_OTP_ECC_TEST_DED_SEC_MSK                (0x04u)

#define BQ7971X_OTP_ECC_TEST_ENC_DEC_POS                (0x01u)
#define BQ7971X_OTP_ECC_TEST_ENC_DEC_MSK                (0x02u)

#define BQ7971X_OTP_ECC_TEST_ENABLE_POS                 (0x00u)
#define BQ7971X_OTP_ECC_TEST_ENABLE_MSK                 (0x01u)

/* --------------------------------------------------------------------------
 * OC_CTRL          (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OC_CTRL_OFFSET                          (0x360u)
#define BQ7971X_OC_CTRL_POR_VAL                         (0x00u)

#define BQ7971X_OC_CTRL_OC_MPFLT_INJ_POS                (0x04u)
#define BQ7971X_OC_CTRL_OC_MPFLT_INJ_MSK                (0x10u)

#define BQ7971X_OC_CTRL_OC_SIGTYPE_POS                  (0x03u)
#define BQ7971X_OC_CTRL_OC_SIGTYPE_MSK                  (0x08u)

#define BQ7971X_OC_CTRL_OC_MODE_POS                     (0x01u)
#define BQ7971X_OC_CTRL_OC_MODE_MSK                     (0x06u)

#define BQ7971X_OC_CTRL_OC_GO_POS                       (0x00u)
#define BQ7971X_OC_CTRL_OC_GO_MSK                       (0x01u)

/* --------------------------------------------------------------------------
 * I2C_WR_DATA          (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_I2C_WR_DATA_OFFSET                      (0x370u)
#define BQ7971X_I2C_WR_DATA_POR_VAL                     (0x00u)

#define BQ7971X_I2C_WR_DATA_DATA_POS                    (0x00u)
#define BQ7971X_I2C_WR_DATA_DATA_MSK                    (0xFFu)

/* --------------------------------------------------------------------------
 * I2C_CTRL             (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7971X_I2C_CTRL_OFFSET                         (0x371u)
#define BQ7971X_I2C_CTRL_POR_VAL                        (0x00u)

#define BQ7971X_I2C_CTRL_SEND_POS                       (0x05u)
#define BQ7971X_I2C_CTRL_SEND_MSK                       (0x20u)

#define BQ7971X_I2C_CTRL_RECEIVE_POS                    (0x04u)
#define BQ7971X_I2C_CTRL_RECEIVE_MSK                    (0x10u)

#define BQ7971X_I2C_CTRL_START_POS                      (0x03u)
#define BQ7971X_I2C_CTRL_START_MSK                      (0x08u)

#define BQ7971X_I2C_CTRL_STOP_POS                       (0x02u)
#define BQ7971X_I2C_CTRL_STOP_MSK                       (0x04u)

#define BQ7971X_I2C_CTRL_NACK_POS                       (0x01u)
#define BQ7971X_I2C_CTRL_NACK_MSK                       (0x02u)

#define BQ7971X_I2C_CTRL_I2C_GO_POS                     (0x00u)
#define BQ7971X_I2C_CTRL_I2C_GO_MSK                     (0x01u)

/* --------------------------------------------------------------------------
 * SPI_CONF             (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_SPI_CONF_OFFSET                         (0x380u)
#define BQ7971X_SPI_CONF_POR_VAL                        (0x00u)

#define BQ7971X_SPI_CONF_CPOL_POS                       (0x06u)
#define BQ7971X_SPI_CONF_CPOL_MSK                       (0x40u)

#define BQ7971X_SPI_CONF_CPHA_POS                       (0x05u)
#define BQ7971X_SPI_CONF_CPHA_MSK                       (0x20u)

#define BQ7971X_SPI_CONF_NUMBIT_POS                     (0x00u)
#define BQ7971X_SPI_CONF_NUMBIT_MSK                     (0x1Fu)

/* --------------------------------------------------------------------------
 * SPI_TX3          (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_SPI_TX3_OFFSET                          (0x381u)
#define BQ7971X_SPI_TX3_POR_VAL                         (0x00u)

#define BQ7971X_SPI_TX3_DATA_POS                        (0x00u)
#define BQ7971X_SPI_TX3_DATA_MSK                        (0xFFu)

/* --------------------------------------------------------------------------
 * SPI_TX2          (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_SPI_TX2_OFFSET                          (0x382u)
#define BQ7971X_SPI_TX2_POR_VAL                         (0x00u)

#define BQ7971X_SPI_TX2_DATA_POS                        (0x00u)
#define BQ7971X_SPI_TX2_DATA_MSK                        (0xFFu)

/* --------------------------------------------------------------------------
 * SPI_TX1          (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_SPI_TX1_OFFSET                          (0x383u)
#define BQ7971X_SPI_TX1_POR_VAL                         (0x00u)

#define BQ7971X_SPI_TX1_DATA_POS                        (0x00u)
#define BQ7971X_SPI_TX1_DATA_MSK                        (0xFFu)

/* --------------------------------------------------------------------------
 * SPI_EXE          (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7971X_SPI_EXE_OFFSET                          (0x384u)
#define BQ7971X_SPI_EXE_POR_VAL                         (0x02u)

#define BQ7971X_SPI_EXE_SS_CTRL_POS                     (0x01u)
#define BQ7971X_SPI_EXE_SS_CTRL_MSK                     (0x02u)

#define BQ7971X_SPI_EXE_SPI_GO_POS                      (0x00u)
#define BQ7971X_SPI_EXE_SPI_GO_MSK                      (0x01u)

/* --------------------------------------------------------------------------
 * OTP_PROG_UNLOCK2             (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OTP_PROG_UNLOCK2_OFFSET                 (0x4FFu)
#define BQ7971X_OTP_PROG_UNLOCK2_POR_VAL                (0x00u)

#define BQ7971X_OTP_PROG_UNLOCK2_CODE_POS               (0x00u)
#define BQ7971X_OTP_PROG_UNLOCK2_CODE_MSK               (0xFFu)

/* --------------------------------------------------------------------------
 * PARTID           (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_PARTID_OFFSET                           (0x500u)
#define BQ7971X_PARTID_POR_VAL                          (0x00u)

#define BQ7971X_PARTID_REV_POS                          (0x00u)
#define BQ7971X_PARTID_REV_MSK                          (0xFFu)

/* --------------------------------------------------------------------------
 * TAPEOUT_REV          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_TAPEOUT_REV_OFFSET                      (0x501u)
#define BQ7971X_TAPEOUT_REV_POR_VAL                     (0x00u)

#define BQ7971X_TAPEOUT_REV_ALL_LAYER_REV_POS           (0x04u)
#define BQ7971X_TAPEOUT_REV_ALL_LAYER_REV_MSK           (0xf0u)

#define BQ7971X_TAPEOUT_REV_METAL_REV_POS               (0x00u)
#define BQ7971X_TAPEOUT_REV_METAL_REV_MSK               (0x0Fu)

/* --------------------------------------------------------------------------
 * DIE_ID1          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIE_ID1_OFFSET                          (0x502u)
#define BQ7971X_DIE_ID1_POR_VAL                         (0x00u)

#define BQ7971X_DIE_ID1_ID_POS                          (0x00u)
#define BQ7971X_DIE_ID1_ID_MSK                          (0xFFu)

/* --------------------------------------------------------------------------
 * DIE_ID2          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIE_ID2_OFFSET                          (0x503u)
#define BQ7971X_DIE_ID2_POR_VAL                         (0x00u)

#define BQ7971X_DIE_ID2_ID_POS                          (0x00u)
#define BQ7971X_DIE_ID2_ID_MSK                          (0xFFu)

/* --------------------------------------------------------------------------
 * DIE_ID3          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIE_ID3_OFFSET                          (0x504u)
#define BQ7971X_DIE_ID3_POR_VAL                         (0x00u)

#define BQ7971X_DIE_ID3_ID_POS                          (0x00u)
#define BQ7971X_DIE_ID3_ID_MSK                          (0xFFu)

/* --------------------------------------------------------------------------
 * DIE_ID4          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIE_ID4_OFFSET                          (0x505u)
#define BQ7971X_DIE_ID4_POR_VAL                         (0x00u)

#define BQ7971X_DIE_ID4_ID_POS                          (0x00u)
#define BQ7971X_DIE_ID4_ID_MSK                          (0xFFu)

/* --------------------------------------------------------------------------
 * DIE_ID5          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIE_ID5_OFFSET                          (0x506u)
#define BQ7971X_DIE_ID5_POR_VAL                         (0x00u)

#define BQ7971X_DIE_ID5_ID_POS                          (0x00u)
#define BQ7971X_DIE_ID5_ID_MSK                          (0xFFu)

/* --------------------------------------------------------------------------
 * DIE_ID6          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIE_ID6_OFFSET                          (0x507u)
#define BQ7971X_DIE_ID6_POR_VAL                         (0x00u)

#define BQ7971X_DIE_ID6_ID_POS                          (0x00u)
#define BQ7971X_DIE_ID6_ID_MSK                          (0xFFu)

/* --------------------------------------------------------------------------
 * DIE_ID7          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIE_ID7_OFFSET                          (0x508u)
#define BQ7971X_DIE_ID7_POR_VAL                         (0x00u)

#define BQ7971X_DIE_ID7_ID_POS                          (0x00u)
#define BQ7971X_DIE_ID7_ID_MSK                          (0xFFu)

/* --------------------------------------------------------------------------
 * DIE_ID8          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIE_ID8_OFFSET                          (0x509u)
#define BQ7971X_DIE_ID8_POR_VAL                         (0x00u)

#define BQ7971X_DIE_ID8_ID_POS                          (0x00u)
#define BQ7971X_DIE_ID8_ID_MSK                          (0xFFu)

/* --------------------------------------------------------------------------
 * CUST_CRC_RSLT_HI             (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CUST_CRC_RSLT_HI_OFFSET                 (0x50Cu)
#define BQ7971X_CUST_CRC_RSLT_HI_POR_VAL                (0x00u)

#define BQ7971X_CUST_CRC_RSLT_HI_CRC_POS                (0x00u)
#define BQ7971X_CUST_CRC_RSLT_HI_CRC_MSK                (0xFFu)

/* --------------------------------------------------------------------------
 * OTP_STAT             (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_OTP_STAT_OFFSET                         (0x510u)
#define BQ7971X_OTP_STAT_POR_VAL                        (0x00u)

#define BQ7971X_OTP_STAT_LOADED_POS                     (0x07u)
#define BQ7971X_OTP_STAT_LOADED_MSK                     (0x80u)

#define BQ7971X_OTP_STAT_UV_OVOK_POS                    (0x06u)
#define BQ7971X_OTP_STAT_UV_OVOK_MSK                    (0x40u)

#define BQ7971X_OTP_STAT_TRY_POS                        (0x05u)
#define BQ7971X_OTP_STAT_TRY_MSK                        (0x20u)

#define BQ7971X_OTP_STAT_UNLOCK_POS                     (0x04u)
#define BQ7971X_OTP_STAT_UNLOCK_MSK                     (0x10u)

#define BQ7971X_OTP_STAT_UV_OVERR_POS                   (0x03u)
#define BQ7971X_OTP_STAT_UV_OVERR_MSK                   (0x08u)

#define BQ7971X_OTP_STAT_SUV_SOVERR_POS                 (0x02u)
#define BQ7971X_OTP_STAT_SUV_SOVERR_MSK                 (0x04u)

#define BQ7971X_OTP_STAT_PROGERR_POS                    (0x01u)
#define BQ7971X_OTP_STAT_PROGERR_MSK                    (0x02u)

#define BQ7971X_OTP_STAT_DONE_POS                       (0x00u)
#define BQ7971X_OTP_STAT_DONE_MSK                       (0x01u)

/* --------------------------------------------------------------------------
 * GPIO_STAT1           (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO_STAT1_OFFSET                       (0x520u)
#define BQ7971X_GPIO_STAT1_POR_VAL                      (0x00u)

#define BQ7971X_GPIO_STAT1_GPIO11_POS                   (0x02u)
#define BQ7971X_GPIO_STAT1_GPIO11_MSK                   (0x04u)

#define BQ7971X_GPIO_STAT1_GPIO10_POS                   (0x01u)
#define BQ7971X_GPIO_STAT1_GPIO10_MSK                   (0x02u)

#define BQ7971X_GPIO_STAT1_GPIO9_POS                    (0x00u)
#define BQ7971X_GPIO_STAT1_GPIO9_MSK                    (0x01u)

/* --------------------------------------------------------------------------
 * GPIO_STAT2           (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO_STAT2_OFFSET                       (0x521u)
#define BQ7971X_GPIO_STAT2_POR_VAL                      (0x00u)

#define BQ7971X_GPIO_STAT2_GPIO8_POS                    (0x07u)
#define BQ7971X_GPIO_STAT2_GPIO8_MSK                    (0x80u)

#define BQ7971X_GPIO_STAT2_GPIO7_POS                    (0x06u)
#define BQ7971X_GPIO_STAT2_GPIO7_MSK                    (0x40u)

#define BQ7971X_GPIO_STAT2_GPIO6_POS                    (0x05u)
#define BQ7971X_GPIO_STAT2_GPIO6_MSK                    (0x20u)

#define BQ7971X_GPIO_STAT2_GPIO5_POS                    (0x04u)
#define BQ7971X_GPIO_STAT2_GPIO5_MSK                    (0x10u)

#define BQ7971X_GPIO_STAT2_GPIO4_POS                    (0x03u)
#define BQ7971X_GPIO_STAT2_GPIO4_MSK                    (0x08u)

#define BQ7971X_GPIO_STAT2_GPIO3_POS                    (0x02u)
#define BQ7971X_GPIO_STAT2_GPIO3_MSK                    (0x04u)

#define BQ7971X_GPIO_STAT2_GPIO2_POS                    (0x01u)
#define BQ7971X_GPIO_STAT2_GPIO2_MSK                    (0x02u)

#define BQ7971X_GPIO_STAT2_GPIO1_POS                    (0x00u)
#define BQ7971X_GPIO_STAT2_GPIO1_MSK                    (0x01u)

/* --------------------------------------------------------------------------
 * BAL_STAT             (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_BAL_STAT_OFFSET                         (0x522u)
#define BQ7971X_BAL_STAT_POR_VAL                        (0x00u)

#define BQ7971X_BAL_STAT_INVALID_CBCONF_POS             (0x05u)
#define BQ7971X_BAL_STAT_INVALID_CBCONF_MSK             (0x20u)

#define BQ7971X_BAL_STAT_OT_PAUSE_DET_POS               (0x04u)
#define BQ7971X_BAL_STAT_OT_PAUSE_DET_MSK               (0x10u)

#define BQ7971X_BAL_STAT_CB_INPAUSE_POS                 (0x03u)
#define BQ7971X_BAL_STAT_CB_INPAUSE_MSK                 (0x08u)

#define BQ7971X_BAL_STAT_CB_RUN_POS                     (0x02u)
#define BQ7971X_BAL_STAT_CB_RUN_MSK                     (0x04u)

#define BQ7971X_BAL_STAT_ABORTFLT_POS                   (0x01u)
#define BQ7971X_BAL_STAT_ABORTFLT_MSK                   (0x02u)

#define BQ7971X_BAL_STAT_CB_DONE_POS                    (0x00u)
#define BQ7971X_BAL_STAT_CB_DONE_MSK                    (0x01u)

/* --------------------------------------------------------------------------
 * BAL_SW_STAT1             (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_BAL_SW_STAT1_OFFSET                     (0x525u)
#define BQ7971X_BAL_SW_STAT1_POR_VAL                    (0x00u)

#define BQ7971X_BAL_SW_STAT1_CELL18_POS                 (0x01u)
#define BQ7971X_BAL_SW_STAT1_CELL18_MSK                 (0x02u)

#define BQ7971X_BAL_SW_STAT1_CELL17_POS                 (0x00u)
#define BQ7971X_BAL_SW_STAT1_CELL17_MSK                 (0x01u)

/* --------------------------------------------------------------------------
 * BAL_SW_STAT2             (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_BAL_SW_STAT2_OFFSET                     (0x526u)
#define BQ7971X_BAL_SW_STAT2_POR_VAL                    (0x00u)

#define BQ7971X_BAL_SW_STAT2_CELL16_POS                 (0x07u)
#define BQ7971X_BAL_SW_STAT2_CELL16_MSK                 (0x80u)

#define BQ7971X_BAL_SW_STAT2_CELL15_POS                 (0x06u)
#define BQ7971X_BAL_SW_STAT2_CELL15_MSK                 (0x40u)

#define BQ7971X_BAL_SW_STAT2_CELL14_POS                 (0x05u)
#define BQ7971X_BAL_SW_STAT2_CELL14_MSK                 (0x20u)

#define BQ7971X_BAL_SW_STAT2_CELL13_POS                 (0x04u)
#define BQ7971X_BAL_SW_STAT2_CELL13_MSK                 (0x10u)

#define BQ7971X_BAL_SW_STAT2_CELL12_POS                 (0x03u)
#define BQ7971X_BAL_SW_STAT2_CELL12_MSK                 (0x08u)

#define BQ7971X_BAL_SW_STAT2_CELL11_POS                 (0x02u)
#define BQ7971X_BAL_SW_STAT2_CELL11_MSK                 (0x04u)

#define BQ7971X_BAL_SW_STAT2_CELL10_POS                 (0x01u)
#define BQ7971X_BAL_SW_STAT2_CELL10_MSK                 (0x02u)

#define BQ7971X_BAL_SW_STAT2_CELL9_POS                  (0x00u)
#define BQ7971X_BAL_SW_STAT2_CELL9_MSK                  (0x01u)

/* --------------------------------------------------------------------------
 * BAL_SW_STAT3             (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_BAL_SW_STAT3_OFFSET                     (0x527u)
#define BQ7971X_BAL_SW_STAT3_POR_VAL                    (0x00u)

#define BQ7971X_BAL_SW_STAT3_CELL8_POS                  (0x07u)
#define BQ7971X_BAL_SW_STAT3_CELL8_MSK                  (0x80u)

#define BQ7971X_BAL_SW_STAT3_CELL7_POS                  (0x06u)
#define BQ7971X_BAL_SW_STAT3_CELL7_MSK                  (0x40u)

#define BQ7971X_BAL_SW_STAT3_CELL6_POS                  (0x05u)
#define BQ7971X_BAL_SW_STAT3_CELL6_MSK                  (0x20u)

#define BQ7971X_BAL_SW_STAT3_CELL5_POS                  (0x04u)
#define BQ7971X_BAL_SW_STAT3_CELL5_MSK                  (0x10u)

#define BQ7971X_BAL_SW_STAT3_CELL4_POS                  (0x03u)
#define BQ7971X_BAL_SW_STAT3_CELL4_MSK                  (0x08u)

#define BQ7971X_BAL_SW_STAT3_CELL3_POS                  (0x02u)
#define BQ7971X_BAL_SW_STAT3_CELL3_MSK                  (0x04u)

#define BQ7971X_BAL_SW_STAT3_CELL2_POS                  (0x01u)
#define BQ7971X_BAL_SW_STAT3_CELL2_MSK                  (0x02u)

#define BQ7971X_BAL_SW_STAT3_CELL1_POS                  (0x00u)
#define BQ7971X_BAL_SW_STAT3_CELL1_MSK                  (0x01u)

/* --------------------------------------------------------------------------
 * BAL_DONE1            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_BAL_DONE1_OFFSET                        (0x529u)
#define BQ7971X_BAL_DONE1_POR_VAL                       (0x00u)

#define BQ7971X_BAL_DONE1_CELL18_POS                    (0x01u)
#define BQ7971X_BAL_DONE1_CELL18_MSK                    (0x02u)

#define BQ7971X_BAL_DONE1_CELL17_POS                    (0x00u)
#define BQ7971X_BAL_DONE1_CELL17_MSK                    (0x01u)

/* --------------------------------------------------------------------------
 * BAL_DONE2            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_BAL_DONE2_OFFSET                        (0x52Au)
#define BQ7971X_BAL_DONE2_POR_VAL                       (0x00u)

#define BQ7971X_BAL_DONE2_CELL16_POS                    (0x07u)
#define BQ7971X_BAL_DONE2_CELL16_MSK                    (0x80u)

#define BQ7971X_BAL_DONE2_CELL15_POS                    (0x06u)
#define BQ7971X_BAL_DONE2_CELL15_MSK                    (0x40u)

#define BQ7971X_BAL_DONE2_CELL14_POS                    (0x05u)
#define BQ7971X_BAL_DONE2_CELL14_MSK                    (0x20u)

#define BQ7971X_BAL_DONE2_CELL13_POS                    (0x04u)
#define BQ7971X_BAL_DONE2_CELL13_MSK                    (0x10u)

#define BQ7971X_BAL_DONE2_CELL12_POS                    (0x03u)
#define BQ7971X_BAL_DONE2_CELL12_MSK                    (0x08u)

#define BQ7971X_BAL_DONE2_CELL11_POS                    (0x02u)
#define BQ7971X_BAL_DONE2_CELL11_MSK                    (0x04u)

#define BQ7971X_BAL_DONE2_CELL10_POS                    (0x01u)
#define BQ7971X_BAL_DONE2_CELL10_MSK                    (0x02u)

#define BQ7971X_BAL_DONE2_CELL9_POS                     (0x00u)
#define BQ7971X_BAL_DONE2_CELL9_MSK                     (0x01u)

/* --------------------------------------------------------------------------
 * BAL_DONE3            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_BAL_DONE3_OFFSET                        (0x52Bu)
#define BQ7971X_BAL_DONE3_POR_VAL                       (0x00u)

#define BQ7971X_BAL_DONE3_CELL8_POS                     (0x07u)
#define BQ7971X_BAL_DONE3_CELL8_MSK                     (0x80u)

#define BQ7971X_BAL_DONE3_CELL7_POS                     (0x06u)
#define BQ7971X_BAL_DONE3_CELL7_MSK                     (0x40u)

#define BQ7971X_BAL_DONE3_CELL6_POS                     (0x05u)
#define BQ7971X_BAL_DONE3_CELL6_MSK                     (0x20u)

#define BQ7971X_BAL_DONE3_CELL5_POS                     (0x04u)
#define BQ7971X_BAL_DONE3_CELL5_MSK                     (0x10u)

#define BQ7971X_BAL_DONE3_CELL4_POS                     (0x03u)
#define BQ7971X_BAL_DONE3_CELL4_MSK                     (0x08u)

#define BQ7971X_BAL_DONE3_CELL3_POS                     (0x02u)
#define BQ7971X_BAL_DONE3_CELL3_MSK                     (0x04u)

#define BQ7971X_BAL_DONE3_CELL2_POS                     (0x01u)
#define BQ7971X_BAL_DONE3_CELL2_MSK                     (0x02u)

#define BQ7971X_BAL_DONE3_CELL1_POS                     (0x00u)
#define BQ7971X_BAL_DONE3_CELL1_MSK                     (0x01u)

/* --------------------------------------------------------------------------
 * BAL_TIME             (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_BAL_TIME_OFFSET                         (0x52Cu)
#define BQ7971X_BAL_TIME_POR_VAL                        (0x00u)

#define BQ7971X_BAL_TIME_TIME_UNIT_POS                  (0x07u)
#define BQ7971X_BAL_TIME_TIME_UNIT_MSK                  (0x80u)

#define BQ7971X_BAL_TIME_TIME_POS                       (0x00u)
#define BQ7971X_BAL_TIME_TIME_MSK                       (0x7fu)

/* --------------------------------------------------------------------------
 * DIAG_STAT1           (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIAG_STAT1_OFFSET                       (0x52Du)
#define BQ7971X_DIAG_STAT1_POR_VAL                      (0x00u)

#define BQ7971X_DIAG_STAT1_FREEZE_ACTIVE_POS            (0x04u)
#define BQ7971X_DIAG_STAT1_FREEZE_ACTIVE_MSK            (0x10u)

#define BQ7971X_DIAG_STAT1_ECC_TEST_OK_POS              (0x03u)
#define BQ7971X_DIAG_STAT1_ECC_TEST_OK_MSK              (0x08u)

#define BQ7971X_DIAG_STAT1_DRDY_OTUT_POS                (0x02u)
#define BQ7971X_DIAG_STAT1_DRDY_OTUT_MSK                (0x04u)

#define BQ7971X_DIAG_STAT1_DRDY_OVUV_POS                (0x01u)
#define BQ7971X_DIAG_STAT1_DRDY_OVUV_MSK                (0x02u)

#define BQ7971X_DIAG_STAT1_DRDY_PWRBIST_POS             (0x00u)
#define BQ7971X_DIAG_STAT1_DRDY_PWRBIST_MSK             (0x01u)

/* --------------------------------------------------------------------------
 * DIAG_STAT2           (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIAG_STAT2_OFFSET                       (0x52Eu)
#define BQ7971X_DIAG_STAT2_POR_VAL                      (0x00u)

#define BQ7971X_DIAG_STAT2_DRDY_CBFETOW_POS             (0x03u)
#define BQ7971X_DIAG_STAT2_DRDY_CBFETOW_MSK             (0x08u)

#define BQ7971X_DIAG_STAT2_DRDY_DIG_POS                 (0x02u)
#define BQ7971X_DIAG_STAT2_DRDY_DIG_MSK                 (0x04u)

#define BQ7971X_DIAG_STAT2_DRDY_ANA_GPIO_POS            (0x01u)
#define BQ7971X_DIAG_STAT2_DRDY_ANA_GPIO_MSK            (0x02u)

#define BQ7971X_DIAG_STAT2_DRDY_ANA_VCELL_POS           (0x00u)
#define BQ7971X_DIAG_STAT2_DRDY_ANA_VCELL_MSK           (0x01u)

/* --------------------------------------------------------------------------
 * ADC_DATA_RDY             (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_ADC_DATA_RDY_OFFSET                     (0x52Fu)
#define BQ7971X_ADC_DATA_RDY_POR_VAL                    (0x00u)

#define BQ7971X_ADC_DATA_RDY_DRDY_CSADC_POS             (0x04u)
#define BQ7971X_ADC_DATA_RDY_DRDY_CSADC_MSK             (0x10u)

#define BQ7971X_ADC_DATA_RDY_DRDY_DIAG_D1D2_POS         (0x03u)
#define BQ7971X_ADC_DATA_RDY_DRDY_DIAG_D1D2_MSK         (0x08u)

#define BQ7971X_ADC_DATA_RDY_DRDY_DIAG_POS              (0x02u)
#define BQ7971X_ADC_DATA_RDY_DRDY_DIAG_MSK              (0x04u)

#define BQ7971X_ADC_DATA_RDY_DRDY_GPADC_POS             (0x01u)
#define BQ7971X_ADC_DATA_RDY_DRDY_GPADC_MSK             (0x02u)

#define BQ7971X_ADC_DATA_RDY_DRDY_VCELLADC_POS          (0x00u)
#define BQ7971X_ADC_DATA_RDY_DRDY_VCELLADC_MSK          (0x01u)

/* --------------------------------------------------------------------------
 * DEV_STAT1            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DEV_STAT1_OFFSET                        (0x530u)
#define BQ7971X_DEV_STAT1_POR_VAL                       (0x00u)

#define BQ7971X_DEV_STAT1_DIAG_MEAS_RUN_POS             (0x07u)
#define BQ7971X_DEV_STAT1_DIAG_MEAS_RUN_MSK             (0x80u)

#define BQ7971X_DEV_STAT1_FACT_CRC_DONE_POS             (0x06u)
#define BQ7971X_DEV_STAT1_FACT_CRC_DONE_MSK             (0x40u)

#define BQ7971X_DEV_STAT1_CUST_CRC_DONE_POS             (0x05u)
#define BQ7971X_DEV_STAT1_CUST_CRC_DONE_MSK             (0x20u)

#define BQ7971X_DEV_STAT1_OTUT_RUN_POS                  (0x04u)
#define BQ7971X_DEV_STAT1_OTUT_RUN_MSK                  (0x10u)

#define BQ7971X_DEV_STAT1_OVUV_RUN_POS                  (0x03u)
#define BQ7971X_DEV_STAT1_OVUV_RUN_MSK                  (0x08u)

#define BQ7971X_DEV_STAT1_DIAG_ANA_RUN_POS              (0x02u)
#define BQ7971X_DEV_STAT1_DIAG_ANA_RUN_MSK              (0x04u)

#define BQ7971X_DEV_STAT1_GPADC_RUN_POS                 (0x01u)
#define BQ7971X_DEV_STAT1_GPADC_RUN_MSK                 (0x02u)

#define BQ7971X_DEV_STAT1_VCELLADC_RUN_POS              (0x00u)
#define BQ7971X_DEV_STAT1_VCELLADC_RUN_MSK              (0x01u)

/* --------------------------------------------------------------------------
 * DEV_STAT2            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DEV_STAT2_OFFSET                        (0x531u)
#define BQ7971X_DEV_STAT2_POR_VAL                       (0x00u)

#define BQ7971X_DEV_STAT2_OC_RUN_POS                    (0x03u)
#define BQ7971X_DEV_STAT2_OC_RUN_MSK                    (0x08u)

#define BQ7971X_DEV_STAT2_CSADC_RUN_POS                 (0x02u)
#define BQ7971X_DEV_STAT2_CSADC_RUN_MSK                 (0x04u)

#define BQ7971X_DEV_STAT2_DIAG_DIG_RUN_POS              (0x01u)
#define BQ7971X_DEV_STAT2_DIAG_DIG_RUN_MSK              (0x02u)

#define BQ7971X_DEV_STAT2_AVDD_ON_POS                   (0x00u)
#define BQ7971X_DEV_STAT2_AVDD_ON_MSK                   (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_SUMMARY            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_SUMMARY_OFFSET                    (0x532u)
#define BQ7971X_FAULT_SUMMARY_POR_VAL                   (0x00u)

#define BQ7971X_FAULT_SUMMARY_FAULT_OC_POS              (0x07u)
#define BQ7971X_FAULT_SUMMARY_FAULT_OC_MSK              (0x80u)

#define BQ7971X_FAULT_SUMMARY_FAULT_ADC_CB_POS          (0x06u)
#define BQ7971X_FAULT_SUMMARY_FAULT_ADC_CB_MSK          (0x40u)

#define BQ7971X_FAULT_SUMMARY_FAULT_OTUT_POS            (0x05u)
#define BQ7971X_FAULT_SUMMARY_FAULT_OTUT_MSK            (0x20u)

#define BQ7971X_FAULT_SUMMARY_FAULT_OVUV_POS            (0x04u)
#define BQ7971X_FAULT_SUMMARY_FAULT_OVUV_MSK            (0x10u)

#define BQ7971X_FAULT_SUMMARY_FAULT_SYS_POS             (0x03u)
#define BQ7971X_FAULT_SUMMARY_FAULT_SYS_MSK             (0x08u)

#define BQ7971X_FAULT_SUMMARY_FAULT_OTP_POS             (0x02u)
#define BQ7971X_FAULT_SUMMARY_FAULT_OTP_MSK             (0x04u)

#define BQ7971X_FAULT_SUMMARY_FAULT_COMM_POS            (0x01u)
#define BQ7971X_FAULT_SUMMARY_FAULT_COMM_MSK            (0x02u)

#define BQ7971X_FAULT_SUMMARY_FAULT_PWR_POS             (0x00u)
#define BQ7971X_FAULT_SUMMARY_FAULT_PWR_MSK             (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_PWR1           (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_PWR1_OFFSET                       (0x535u)
#define BQ7971X_FAULT_PWR1_POR_VAL                      (0x00u)

#define BQ7971X_FAULT_PWR1_PWRBIST_FAIL_POS             (0x07u)
#define BQ7971X_FAULT_PWR1_PWRBIST_FAIL_MSK             (0x80u)

#define BQ7971X_FAULT_PWR1_VSS_OPEN_POS                 (0x06u)
#define BQ7971X_FAULT_PWR1_VSS_OPEN_MSK                 (0x40u)

#define BQ7971X_FAULT_PWR1_TSREF_OSC_POS                (0x05u)
#define BQ7971X_FAULT_PWR1_TSREF_OSC_MSK                (0x20u)

#define BQ7971X_FAULT_PWR1_TSREF_UV_POS                 (0x04u)
#define BQ7971X_FAULT_PWR1_TSREF_UV_MSK                 (0x10u)

#define BQ7971X_FAULT_PWR1_TSREF_OV_POS                 (0x03u)
#define BQ7971X_FAULT_PWR1_TSREF_OV_MSK                 (0x08u)

#define BQ7971X_FAULT_PWR1_DVDD_OV_POS                  (0x02u)
#define BQ7971X_FAULT_PWR1_DVDD_OV_MSK                  (0x04u)

#define BQ7971X_FAULT_PWR1_AVDD_OSC_POS                 (0x01u)
#define BQ7971X_FAULT_PWR1_AVDD_OSC_MSK                 (0x02u)

#define BQ7971X_FAULT_PWR1_AVDD_OV_POS                  (0x00u)
#define BQ7971X_FAULT_PWR1_AVDD_OV_MSK                  (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_PWR2           (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_PWR2_OFFSET                       (0x536u)
#define BQ7971X_FAULT_PWR2_POR_VAL                      (0x00u)

#define BQ7971X_FAULT_PWR2_NEG_CPUMP_POS                (0x00u)
#define BQ7971X_FAULT_PWR2_NEG_CPUMP_MSK                (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_COMM           (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_COMM_OFFSET                       (0x538u)
#define BQ7971X_FAULT_COMM_POR_VAL                      (0x00u)

#define BQ7971X_FAULT_COMM_FCOMM_DET_POS                (0x07u)
#define BQ7971X_FAULT_COMM_FCOMM_DET_MSK                (0x80u)

#define BQ7971X_FAULT_COMM_FTONE_DET_POS                (0x06u)
#define BQ7971X_FAULT_COMM_FTONE_DET_MSK                (0x40u)

#define BQ7971X_FAULT_COMM_HB_FAIL_POS                  (0x05u)
#define BQ7971X_FAULT_COMM_HB_FAIL_MSK                  (0x20u)

#define BQ7971X_FAULT_COMM_COML_POS                     (0x04u)
#define BQ7971X_FAULT_COMM_COML_MSK                     (0x10u)

#define BQ7971X_FAULT_COMM_COMH_POS                     (0x03u)
#define BQ7971X_FAULT_COMM_COMH_MSK                     (0x08u)

#define BQ7971X_FAULT_COMM_UART_FRAME_POS               (0x02u)
#define BQ7971X_FAULT_COMM_UART_FRAME_MSK               (0x04u)

#define BQ7971X_FAULT_COMM_COMMCLR_DET_POS              (0x01u)
#define BQ7971X_FAULT_COMM_COMMCLR_DET_MSK              (0x02u)

#define BQ7971X_FAULT_COMM_STOP_DET_POS                 (0x00u)
#define BQ7971X_FAULT_COMM_STOP_DET_MSK                 (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_OTP            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_OTP_OFFSET                        (0x53Bu)
#define BQ7971X_FAULT_OTP_POR_VAL                       (0x00u)

#define BQ7971X_FAULT_OTP_DED_DET_POS                   (0x05u)
#define BQ7971X_FAULT_OTP_DED_DET_MSK                   (0x20u)

#define BQ7971X_FAULT_OTP_SEC_DET_POS                   (0x04u)
#define BQ7971X_FAULT_OTP_SEC_DET_MSK                   (0x10u)

#define BQ7971X_FAULT_OTP_CUST_CRC_POS                  (0x03u)
#define BQ7971X_FAULT_OTP_CUST_CRC_MSK                  (0x08u)

#define BQ7971X_FAULT_OTP_FACT_CRC_POS                  (0x02u)
#define BQ7971X_FAULT_OTP_FACT_CRC_MSK                  (0x04u)

#define BQ7971X_FAULT_OTP_LOADERR_POS                   (0x01u)
#define BQ7971X_FAULT_OTP_LOADERR_MSK                   (0x02u)

#define BQ7971X_FAULT_OTP_GBLOVERR_POS                  (0x00u)
#define BQ7971X_FAULT_OTP_GBLOVERR_MSK                  (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_SYS            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_SYS_OFFSET                        (0x53Du)
#define BQ7971X_FAULT_SYS_POR_VAL                       (0x00u)

#define BQ7971X_FAULT_SYS_AVDD_ON_POS                   (0x06u)
#define BQ7971X_FAULT_SYS_AVDD_ON_MSK                   (0x40u)

#define BQ7971X_FAULT_SYS_I2C_LOW_POS                   (0x05u)
#define BQ7971X_FAULT_SYS_I2C_LOW_MSK                   (0x20u)

#define BQ7971X_FAULT_SYS_I2C_NACK_POS                  (0x04u)
#define BQ7971X_FAULT_SYS_I2C_NACK_MSK                  (0x10u)

#define BQ7971X_FAULT_SYS_LFO_POS                       (0x03u)
#define BQ7971X_FAULT_SYS_LFO_MSK                       (0x08u)

#define BQ7971X_FAULT_SYS_DRST_POS                      (0x02u)
#define BQ7971X_FAULT_SYS_DRST_MSK                      (0x04u)

#define BQ7971X_FAULT_SYS_TSHUT_POS                     (0x01u)
#define BQ7971X_FAULT_SYS_TSHUT_MSK                     (0x02u)

#define BQ7971X_FAULT_SYS_TWARN_POS                     (0x00u)
#define BQ7971X_FAULT_SYS_TWARN_MSK                     (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_OV1            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_OV1_OFFSET                        (0x540u)
#define BQ7971X_FAULT_OV1_POR_VAL                       (0x00u)

#define BQ7971X_FAULT_OV1_OV18_DET_POS                  (0x01u)
#define BQ7971X_FAULT_OV1_OV18_DET_MSK                  (0x02u)

#define BQ7971X_FAULT_OV1_OV17_DET_POS                  (0x00u)
#define BQ7971X_FAULT_OV1_OV17_DET_MSK                  (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_OV2            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_OV2_OFFSET                        (0x541u)
#define BQ7971X_FAULT_OV2_POR_VAL                       (0x00u)

#define BQ7971X_FAULT_OV2_OV16_DET_POS                  (0x07u)
#define BQ7971X_FAULT_OV2_OV16_DET_MSK                  (0x80u)

#define BQ7971X_FAULT_OV2_OV15_DET_POS                  (0x06u)
#define BQ7971X_FAULT_OV2_OV15_DET_MSK                  (0x40u)

#define BQ7971X_FAULT_OV2_OV14_DET_POS                  (0x05u)
#define BQ7971X_FAULT_OV2_OV14_DET_MSK                  (0x20u)

#define BQ7971X_FAULT_OV2_OV13_DET_POS                  (0x04u)
#define BQ7971X_FAULT_OV2_OV13_DET_MSK                  (0x10u)

#define BQ7971X_FAULT_OV2_OV12_DET_POS                  (0x03u)
#define BQ7971X_FAULT_OV2_OV12_DET_MSK                  (0x08u)

#define BQ7971X_FAULT_OV2_OV11_DET_POS                  (0x02u)
#define BQ7971X_FAULT_OV2_OV11_DET_MSK                  (0x04u)

#define BQ7971X_FAULT_OV2_OV10_DET_POS                  (0x01u)
#define BQ7971X_FAULT_OV2_OV10_DET_MSK                  (0x02u)

#define BQ7971X_FAULT_OV2_OV9_DET_POS                   (0x00u)
#define BQ7971X_FAULT_OV2_OV9_DET_MSK                   (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_OV3            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_OV3_OFFSET                        (0x542u)
#define BQ7971X_FAULT_OV3_POR_VAL                       (0x00u)

#define BQ7971X_FAULT_OV3_OV8_DET_POS                   (0x07u)
#define BQ7971X_FAULT_OV3_OV8_DET_MSK                   (0x80u)

#define BQ7971X_FAULT_OV3_OV7_DET_POS                   (0x06u)
#define BQ7971X_FAULT_OV3_OV7_DET_MSK                   (0x40u)

#define BQ7971X_FAULT_OV3_OV6_DET_POS                   (0x05u)
#define BQ7971X_FAULT_OV3_OV6_DET_MSK                   (0x20u)

#define BQ7971X_FAULT_OV3_OV5_DET_POS                   (0x04u)
#define BQ7971X_FAULT_OV3_OV5_DET_MSK                   (0x10u)

#define BQ7971X_FAULT_OV3_OV4_DET_POS                   (0x03u)
#define BQ7971X_FAULT_OV3_OV4_DET_MSK                   (0x08u)

#define BQ7971X_FAULT_OV3_OV3_DET_POS                   (0x02u)
#define BQ7971X_FAULT_OV3_OV3_DET_MSK                   (0x04u)

#define BQ7971X_FAULT_OV3_OV2_DET_POS                   (0x01u)
#define BQ7971X_FAULT_OV3_OV2_DET_MSK                   (0x02u)

#define BQ7971X_FAULT_OV3_OV1_DET_POS                   (0x00u)
#define BQ7971X_FAULT_OV3_OV1_DET_MSK                   (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_UV1            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_UV1_OFFSET                        (0x543u)
#define BQ7971X_FAULT_UV1_POR_VAL                       (0x00u)

#define BQ7971X_FAULT_UV1_UV18_DET_POS                  (0x01u)
#define BQ7971X_FAULT_UV1_UV18_DET_MSK                  (0x02u)

#define BQ7971X_FAULT_UV1_UV17_DET_POS                  (0x00u)
#define BQ7971X_FAULT_UV1_UV17_DET_MSK                  (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_UV2            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_UV2_OFFSET                        (0x544u)
#define BQ7971X_FAULT_UV2_POR_VAL                       (0x00u)

#define BQ7971X_FAULT_UV2_UV16_DET_POS                  (0x07u)
#define BQ7971X_FAULT_UV2_UV16_DET_MSK                  (0x80u)

#define BQ7971X_FAULT_UV2_UV15_DET_POS                  (0x06u)
#define BQ7971X_FAULT_UV2_UV15_DET_MSK                  (0x40u)

#define BQ7971X_FAULT_UV2_UV14_DET_POS                  (0x05u)
#define BQ7971X_FAULT_UV2_UV14_DET_MSK                  (0x20u)

#define BQ7971X_FAULT_UV2_UV13_DET_POS                  (0x04u)
#define BQ7971X_FAULT_UV2_UV13_DET_MSK                  (0x10u)

#define BQ7971X_FAULT_UV2_UV12_DET_POS                  (0x03u)
#define BQ7971X_FAULT_UV2_UV12_DET_MSK                  (0x08u)

#define BQ7971X_FAULT_UV2_UV11_DET_POS                  (0x02u)
#define BQ7971X_FAULT_UV2_UV11_DET_MSK                  (0x04u)

#define BQ7971X_FAULT_UV2_UV10_DET_POS                  (0x01u)
#define BQ7971X_FAULT_UV2_UV10_DET_MSK                  (0x02u)

#define BQ7971X_FAULT_UV2_UV9_DET_POS                   (0x00u)
#define BQ7971X_FAULT_UV2_UV9_DET_MSK                   (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_UV3            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_UV3_OFFSET                        (0x545u)
#define BQ7971X_FAULT_UV3_POR_VAL                       (0x00u)

#define BQ7971X_FAULT_UV3_UV8_DET_POS                   (0x07u)
#define BQ7971X_FAULT_UV3_UV8_DET_MSK                   (0x80u)

#define BQ7971X_FAULT_UV3_UV7_DET_POS                   (0x06u)
#define BQ7971X_FAULT_UV3_UV7_DET_MSK                   (0x40u)

#define BQ7971X_FAULT_UV3_UV6_DET_POS                   (0x05u)
#define BQ7971X_FAULT_UV3_UV6_DET_MSK                   (0x20u)

#define BQ7971X_FAULT_UV3_UV5_DET_POS                   (0x04u)
#define BQ7971X_FAULT_UV3_UV5_DET_MSK                   (0x10u)

#define BQ7971X_FAULT_UV3_UV4_DET_POS                   (0x03u)
#define BQ7971X_FAULT_UV3_UV4_DET_MSK                   (0x08u)

#define BQ7971X_FAULT_UV3_UV3_DET_POS                   (0x02u)
#define BQ7971X_FAULT_UV3_UV3_DET_MSK                   (0x04u)

#define BQ7971X_FAULT_UV3_UV2_DET_POS                   (0x01u)
#define BQ7971X_FAULT_UV3_UV2_DET_MSK                   (0x02u)

#define BQ7971X_FAULT_UV3_UV1_DET_POS                   (0x00u)
#define BQ7971X_FAULT_UV3_UV1_DET_MSK                   (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_OT1            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_OT1_OFFSET                        (0x547u)
#define BQ7971X_FAULT_OT1_POR_VAL                       (0x00u)

#define BQ7971X_FAULT_OT1_OT11_DET_POS                  (0x02u)
#define BQ7971X_FAULT_OT1_OT11_DET_MSK                  (0x04u)

#define BQ7971X_FAULT_OT1_OT10_DET_POS                  (0x01u)
#define BQ7971X_FAULT_OT1_OT10_DET_MSK                  (0x02u)

#define BQ7971X_FAULT_OT1_OT9_DET_POS                   (0x00u)
#define BQ7971X_FAULT_OT1_OT9_DET_MSK                   (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_OT2            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_OT2_OFFSET                        (0x548u)
#define BQ7971X_FAULT_OT2_POR_VAL                       (0x00u)

#define BQ7971X_FAULT_OT2_OT8_DET_POS                   (0x07u)
#define BQ7971X_FAULT_OT2_OT8_DET_MSK                   (0x80u)

#define BQ7971X_FAULT_OT2_OT7_DET_POS                   (0x06u)
#define BQ7971X_FAULT_OT2_OT7_DET_MSK                   (0x40u)

#define BQ7971X_FAULT_OT2_OT6_DET_POS                   (0x05u)
#define BQ7971X_FAULT_OT2_OT6_DET_MSK                   (0x20u)

#define BQ7971X_FAULT_OT2_OT5_DET_POS                   (0x04u)
#define BQ7971X_FAULT_OT2_OT5_DET_MSK                   (0x10u)

#define BQ7971X_FAULT_OT2_OT4_DET_POS                   (0x03u)
#define BQ7971X_FAULT_OT2_OT4_DET_MSK                   (0x08u)

#define BQ7971X_FAULT_OT2_OT3_DET_POS                   (0x02u)
#define BQ7971X_FAULT_OT2_OT3_DET_MSK                   (0x04u)

#define BQ7971X_FAULT_OT2_OT2_DET_POS                   (0x01u)
#define BQ7971X_FAULT_OT2_OT2_DET_MSK                   (0x02u)

#define BQ7971X_FAULT_OT2_OT1_DET_POS                   (0x00u)
#define BQ7971X_FAULT_OT2_OT1_DET_MSK                   (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_UT1            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_UT1_OFFSET                        (0x54Au)
#define BQ7971X_FAULT_UT1_POR_VAL                       (0x00u)
#define BQ7971X_FAULT_UT1_UT11_DET_POS                  (0x02u)
#define BQ7971X_FAULT_UT1_UT11_DET_MSK                  (0x04u)
#define BQ7971X_FAULT_UT1_UT10_DET_POS                  (0x01u)
#define BQ7971X_FAULT_UT1_UT10_DET_MSK                  (0x02u)
#define BQ7971X_FAULT_UT1_UT9_DET_POS                   (0x00u)
#define BQ7971X_FAULT_UT1_UT9_DET_MSK                   (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_UT2            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_UT2_OFFSET                        (0x54Bu)
#define BQ7971X_FAULT_UT2_POR_VAL                       (0x00u)

#define BQ7971X_FAULT_UT2_UT8_DET_POS                   (0x07u)
#define BQ7971X_FAULT_UT2_UT8_DET_MSK                   (0x80u)

#define BQ7971X_FAULT_UT2_UT7_DET_POS                   (0x06u)
#define BQ7971X_FAULT_UT2_UT7_DET_MSK                   (0x40u)

#define BQ7971X_FAULT_UT2_UT6_DET_POS                   (0x05u)
#define BQ7971X_FAULT_UT2_UT6_DET_MSK                   (0x20u)

#define BQ7971X_FAULT_UT2_UT5_DET_POS                   (0x04u)
#define BQ7971X_FAULT_UT2_UT5_DET_MSK                   (0x10u)

#define BQ7971X_FAULT_UT2_UT4_DET_POS                   (0x03u)
#define BQ7971X_FAULT_UT2_UT4_DET_MSK                   (0x08u)

#define BQ7971X_FAULT_UT2_UT3_DET_POS                   (0x02u)
#define BQ7971X_FAULT_UT2_UT3_DET_MSK                   (0x04u)

#define BQ7971X_FAULT_UT2_UT2_DET_POS                   (0x01u)
#define BQ7971X_FAULT_UT2_UT2_DET_MSK                   (0x02u)

#define BQ7971X_FAULT_UT2_UT1_DET_POS                   (0x00u)
#define BQ7971X_FAULT_UT2_UT1_DET_MSK                   (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_ADC_GPIO1          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_ADC_GPIO1_OFFSET                  (0x54Du)
#define BQ7971X_FAULT_ADC_GPIO1_POR_VAL                 (0x00u)

#define BQ7971X_FAULT_ADC_GPIO1_GPIO11_AFAIL_POS        (0x02u)
#define BQ7971X_FAULT_ADC_GPIO1_GPIO11_AFAIL_MSK        (0x04u)

#define BQ7971X_FAULT_ADC_GPIO1_GPIO10_AFAIL_POS        (0x01u)
#define BQ7971X_FAULT_ADC_GPIO1_GPIO10_AFAIL_MSK        (0x02u)

#define BQ7971X_FAULT_ADC_GPIO1_GPIO9_AFAIL_POS         (0x00u)
#define BQ7971X_FAULT_ADC_GPIO1_GPIO9_AFAIL_MSK         (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_ADC_GPIO2          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_ADC_GPIO2_OFFSET                  (0x54Eu)
#define BQ7971X_FAULT_ADC_GPIO2_POR_VAL                 (0x00u)

#define BQ7971X_FAULT_ADC_GPIO2_GPIO8_AFAIL_POS         (0x07u)
#define BQ7971X_FAULT_ADC_GPIO2_GPIO8_AFAIL_MSK         (0x80u)

#define BQ7971X_FAULT_ADC_GPIO2_GPIO7_AFAIL_POS         (0x06u)
#define BQ7971X_FAULT_ADC_GPIO2_GPIO7_AFAIL_MSK         (0x40u)

#define BQ7971X_FAULT_ADC_GPIO2_GPIO6_AFAIL_POS         (0x05u)
#define BQ7971X_FAULT_ADC_GPIO2_GPIO6_AFAIL_MSK         (0x20u)

#define BQ7971X_FAULT_ADC_GPIO2_GPIO5_AFAIL_POS         (0x04u)
#define BQ7971X_FAULT_ADC_GPIO2_GPIO5_AFAIL_MSK         (0x10u)

#define BQ7971X_FAULT_ADC_GPIO2_GPIO4_AFAIL_POS         (0x03u)
#define BQ7971X_FAULT_ADC_GPIO2_GPIO4_AFAIL_MSK         (0x08u)

#define BQ7971X_FAULT_ADC_GPIO2_GPIO3_AFAIL_POS         (0x02u)
#define BQ7971X_FAULT_ADC_GPIO2_GPIO3_AFAIL_MSK         (0x04u)

#define BQ7971X_FAULT_ADC_GPIO2_GPIO2_AFAIL_POS         (0x01u)
#define BQ7971X_FAULT_ADC_GPIO2_GPIO2_AFAIL_MSK         (0x02u)

#define BQ7971X_FAULT_ADC_GPIO2_GPIO1_AFAIL_POS         (0x00u)
#define BQ7971X_FAULT_ADC_GPIO2_GPIO1_AFAIL_MSK         (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_ADC_VCELL1             (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_ADC_VCELL1_OFFSET                 (0x54Fu)
#define BQ7971X_FAULT_ADC_VCELL1_POR_VAL                (0x00u)

#define BQ7971X_FAULT_ADC_VCELL1_CELL18_AFAIL_POS       (0x01u)
#define BQ7971X_FAULT_ADC_VCELL1_CELL18_AFAIL_MSK       (0x02u)

#define BQ7971X_FAULT_ADC_VCELL1_CELL17_AFAIL_POS       (0x00u)
#define BQ7971X_FAULT_ADC_VCELL1_CELL17_AFAIL_MSK       (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_ADC_VCELL2             (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_ADC_VCELL2_OFFSET                 (0x550u)
#define BQ7971X_FAULT_ADC_VCELL2_POR_VAL                (0x00u)

#define BQ7971X_FAULT_ADC_VCELL2_CELL16_AFAIL_POS       (0x07u)
#define BQ7971X_FAULT_ADC_VCELL2_CELL16_AFAIL_MSK       (0x80u)

#define BQ7971X_FAULT_ADC_VCELL2_CELL15_AFAIL_POS       (0x06u)
#define BQ7971X_FAULT_ADC_VCELL2_CELL15_AFAIL_MSK       (0x40u)

#define BQ7971X_FAULT_ADC_VCELL2_CELL14_AFAIL_POS       (0x05u)
#define BQ7971X_FAULT_ADC_VCELL2_CELL14_AFAIL_MSK       (0x20u)

#define BQ7971X_FAULT_ADC_VCELL2_CELL13_AFAIL_POS       (0x04u)
#define BQ7971X_FAULT_ADC_VCELL2_CELL13_AFAIL_MSK       (0x10u)

#define BQ7971X_FAULT_ADC_VCELL2_CELL12_AFAIL_POS       (0x03u)
#define BQ7971X_FAULT_ADC_VCELL2_CELL12_AFAIL_MSK       (0x08u)

#define BQ7971X_FAULT_ADC_VCELL2_CELL11_AFAIL_POS       (0x02u)
#define BQ7971X_FAULT_ADC_VCELL2_CELL11_AFAIL_MSK       (0x04u)

#define BQ7971X_FAULT_ADC_VCELL2_CELL10_AFAIL_POS       (0x01u)
#define BQ7971X_FAULT_ADC_VCELL2_CELL10_AFAIL_MSK       (0x02u)

#define BQ7971X_FAULT_ADC_VCELL2_CELL9_AFAIL_POS        (0x00u)
#define BQ7971X_FAULT_ADC_VCELL2_CELL9_AFAIL_MSK        (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_ADC_VCELL3             (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_ADC_VCELL3_OFFSET                 (0x551u)
#define BQ7971X_FAULT_ADC_VCELL3_POR_VAL                (0x00u)

#define BQ7971X_FAULT_ADC_VCELL3_CELL8_AFAIL_POS        (0x07u)
#define BQ7971X_FAULT_ADC_VCELL3_CELL8_AFAIL_MSK        (0x80u)

#define BQ7971X_FAULT_ADC_VCELL3_CELL7_AFAIL_POS        (0x06u)
#define BQ7971X_FAULT_ADC_VCELL3_CELL7_AFAIL_MSK        (0x40u)

#define BQ7971X_FAULT_ADC_VCELL3_CELL6_AFAIL_POS        (0x05u)
#define BQ7971X_FAULT_ADC_VCELL3_CELL6_AFAIL_MSK        (0x20u)

#define BQ7971X_FAULT_ADC_VCELL3_CELL5_AFAIL_POS        (0x04u)
#define BQ7971X_FAULT_ADC_VCELL3_CELL5_AFAIL_MSK        (0x10u)

#define BQ7971X_FAULT_ADC_VCELL3_CELL4_AFAIL_POS        (0x03u)
#define BQ7971X_FAULT_ADC_VCELL3_CELL4_AFAIL_MSK        (0x08u)

#define BQ7971X_FAULT_ADC_VCELL3_CELL3_AFAIL_POS        (0x02u)
#define BQ7971X_FAULT_ADC_VCELL3_CELL3_AFAIL_MSK        (0x04u)

#define BQ7971X_FAULT_ADC_VCELL3_CELL2_AFAIL_POS        (0x01u)
#define BQ7971X_FAULT_ADC_VCELL3_CELL2_AFAIL_MSK        (0x02u)

#define BQ7971X_FAULT_ADC_VCELL3_CELL1_AFAIL_POS        (0x00u)
#define BQ7971X_FAULT_ADC_VCELL3_CELL1_AFAIL_MSK        (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_ADC_DIG1           (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_ADC_DIG1_OFFSET                   (0x552u)
#define BQ7971X_FAULT_ADC_DIG1_POR_VAL                  (0x00u)

#define BQ7971X_FAULT_ADC_DIG1_CS_DFAIL_POS             (0x04u)
#define BQ7971X_FAULT_ADC_DIG1_CS_DFAIL_MSK             (0x10u)

#define BQ7971X_FAULT_ADC_DIG1_GP3_DFAIL_POS            (0x03u)
#define BQ7971X_FAULT_ADC_DIG1_GP3_DFAIL_MSK            (0x08u)

#define BQ7971X_FAULT_ADC_DIG1_GP1_DFAIL_POS            (0x02u)
#define BQ7971X_FAULT_ADC_DIG1_GP1_DFAIL_MSK            (0x04u)

#define BQ7971X_FAULT_ADC_DIG1_CELL18_DFAIL_POS         (0x01u)
#define BQ7971X_FAULT_ADC_DIG1_CELL18_DFAIL_MSK         (0x02u)

#define BQ7971X_FAULT_ADC_DIG1_CELL17_DFAIL_POS         (0x00u)
#define BQ7971X_FAULT_ADC_DIG1_CELL17_DFAIL_MSK         (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_ADC_DIG2           (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_ADC_DIG2_OFFSET                   (0x553u)
#define BQ7971X_FAULT_ADC_DIG2_POR_VAL                  (0x00u)

#define BQ7971X_FAULT_ADC_DIG2_CELL16_DFAIL_POS         (0x07u)
#define BQ7971X_FAULT_ADC_DIG2_CELL16_DFAIL_MSK         (0x80u)

#define BQ7971X_FAULT_ADC_DIG2_CELL15_DFAIL_POS         (0x06u)
#define BQ7971X_FAULT_ADC_DIG2_CELL15_DFAIL_MSK         (0x40u)

#define BQ7971X_FAULT_ADC_DIG2_CELL14_DFAIL_POS         (0x05u)
#define BQ7971X_FAULT_ADC_DIG2_CELL14_DFAIL_MSK         (0x20u)

#define BQ7971X_FAULT_ADC_DIG2_CELL13_DFAIL_POS         (0x04u)
#define BQ7971X_FAULT_ADC_DIG2_CELL13_DFAIL_MSK         (0x10u)

#define BQ7971X_FAULT_ADC_DIG2_CELL12_DFAIL_POS         (0x03u)
#define BQ7971X_FAULT_ADC_DIG2_CELL12_DFAIL_MSK         (0x08u)

#define BQ7971X_FAULT_ADC_DIG2_CELL11_DFAIL_POS         (0x02u)
#define BQ7971X_FAULT_ADC_DIG2_CELL11_DFAIL_MSK         (0x04u)

#define BQ7971X_FAULT_ADC_DIG2_CELL10_DFAIL_POS         (0x01u)
#define BQ7971X_FAULT_ADC_DIG2_CELL10_DFAIL_MSK         (0x02u)

#define BQ7971X_FAULT_ADC_DIG2_CELL9_DFAIL_POS          (0x00u)
#define BQ7971X_FAULT_ADC_DIG2_CELL9_DFAIL_MSK          (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_ADC_DIG3           (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_ADC_DIG3_OFFSET                   (0x554u)
#define BQ7971X_FAULT_ADC_DIG3_POR_VAL                  (0x00u)

#define BQ7971X_FAULT_ADC_DIG3_CELL8_DFAIL_POS          (0x07u)
#define BQ7971X_FAULT_ADC_DIG3_CELL8_DFAIL_MSK          (0x80u)

#define BQ7971X_FAULT_ADC_DIG3_CELL7_DFAIL_POS          (0x06u)
#define BQ7971X_FAULT_ADC_DIG3_CELL7_DFAIL_MSK          (0x40u)

#define BQ7971X_FAULT_ADC_DIG3_CELL6_DFAIL_POS          (0x05u)
#define BQ7971X_FAULT_ADC_DIG3_CELL6_DFAIL_MSK          (0x20u)

#define BQ7971X_FAULT_ADC_DIG3_CELL5_DFAIL_POS          (0x04u)
#define BQ7971X_FAULT_ADC_DIG3_CELL5_DFAIL_MSK          (0x10u)

#define BQ7971X_FAULT_ADC_DIG3_CELL4_DFAIL_POS          (0x03u)
#define BQ7971X_FAULT_ADC_DIG3_CELL4_DFAIL_MSK          (0x08u)

#define BQ7971X_FAULT_ADC_DIG3_CELL3_DFAIL_POS          (0x02u)
#define BQ7971X_FAULT_ADC_DIG3_CELL3_DFAIL_MSK          (0x04u)

#define BQ7971X_FAULT_ADC_DIG3_CELL2_DFAIL_POS          (0x01u)
#define BQ7971X_FAULT_ADC_DIG3_CELL2_DFAIL_MSK          (0x02u)

#define BQ7971X_FAULT_ADC_DIG3_CELL1_DFAIL_POS          (0x00u)
#define BQ7971X_FAULT_ADC_DIG3_CELL1_DFAIL_MSK          (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_ADC_MISC           (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_ADC_MISC_OFFSET                   (0x555u)
#define BQ7971X_FAULT_ADC_MISC_POR_VAL                  (0x00u)

#define BQ7971X_FAULT_ADC_MISC_ADC_PFAIL_POS            (0x03u)
#define BQ7971X_FAULT_ADC_MISC_ADC_PFAIL_MSK            (0x08u)

#define BQ7971X_FAULT_ADC_MISC_DIAG_MEAS_PFAIL_POS      (0x02u)
#define BQ7971X_FAULT_ADC_MISC_DIAG_MEAS_PFAIL_MSK      (0x04u)

#define BQ7971X_FAULT_ADC_MISC_DIAG_ANA_PFAIL_POS       (0x01u)
#define BQ7971X_FAULT_ADC_MISC_DIAG_ANA_PFAIL_MSK       (0x02u)

#define BQ7971X_FAULT_ADC_MISC_DIAG_ANA_ABORT_POS       (0x00u)
#define BQ7971X_FAULT_ADC_MISC_DIAG_ANA_ABORT_MSK       (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_CB_FETOW1          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_CB_FETOW1_OFFSET                  (0x557u)
#define BQ7971X_FAULT_CB_FETOW1_POR_VAL                 (0x00u)

#define BQ7971X_FAULT_CB_FETOW1_CB18_FAIL_POS           (0x01u)
#define BQ7971X_FAULT_CB_FETOW1_CB18_FAIL_MSK           (0x02u)

#define BQ7971X_FAULT_CB_FETOW1_CB17_FAIL_POS           (0x00u)
#define BQ7971X_FAULT_CB_FETOW1_CB17_FAIL_MSK           (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_CB_FETOW2          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_CB_FETOW2_OFFSET                  (0x558u)
#define BQ7971X_FAULT_CB_FETOW2_POR_VAL                 (0x00u)

#define BQ7971X_FAULT_CB_FETOW2_CB16_FAIL_POS           (0x07u)
#define BQ7971X_FAULT_CB_FETOW2_CB16_FAIL_MSK           (0x80u)

#define BQ7971X_FAULT_CB_FETOW2_CB15_FAIL_POS           (0x06u)
#define BQ7971X_FAULT_CB_FETOW2_CB15_FAIL_MSK           (0x40u)

#define BQ7971X_FAULT_CB_FETOW2_CB14_FAIL_POS           (0x05u)
#define BQ7971X_FAULT_CB_FETOW2_CB14_FAIL_MSK           (0x20u)

#define BQ7971X_FAULT_CB_FETOW2_CB13_FAIL_POS           (0x04u)
#define BQ7971X_FAULT_CB_FETOW2_CB13_FAIL_MSK           (0x10u)

#define BQ7971X_FAULT_CB_FETOW2_CB12_FAIL_POS           (0x03u)
#define BQ7971X_FAULT_CB_FETOW2_CB12_FAIL_MSK           (0x08u)

#define BQ7971X_FAULT_CB_FETOW2_CB11_FAIL_POS           (0x02u)
#define BQ7971X_FAULT_CB_FETOW2_CB11_FAIL_MSK           (0x04u)

#define BQ7971X_FAULT_CB_FETOW2_CB10_FAIL_POS           (0x01u)
#define BQ7971X_FAULT_CB_FETOW2_CB10_FAIL_MSK           (0x02u)

#define BQ7971X_FAULT_CB_FETOW2_CB9_FAIL_POS            (0x00u)
#define BQ7971X_FAULT_CB_FETOW2_CB9_FAIL_MSK            (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_CB_FETOW3          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_CB_FETOW3_OFFSET                  (0x559u)
#define BQ7971X_FAULT_CB_FETOW3_POR_VAL                 (0x00u)

#define BQ7971X_FAULT_CB_FETOW3_CB8_FAIL_POS            (0x07u)
#define BQ7971X_FAULT_CB_FETOW3_CB8_FAIL_MSK            (0x80u)

#define BQ7971X_FAULT_CB_FETOW3_CB7_FAIL_POS            (0x06u)
#define BQ7971X_FAULT_CB_FETOW3_CB7_FAIL_MSK            (0x40u)

#define BQ7971X_FAULT_CB_FETOW3_CB6_FAIL_POS            (0x05u)
#define BQ7971X_FAULT_CB_FETOW3_CB6_FAIL_MSK            (0x20u)

#define BQ7971X_FAULT_CB_FETOW3_CB5_FAIL_POS            (0x04u)
#define BQ7971X_FAULT_CB_FETOW3_CB5_FAIL_MSK            (0x10u)

#define BQ7971X_FAULT_CB_FETOW3_CB4_FAIL_POS            (0x03u)
#define BQ7971X_FAULT_CB_FETOW3_CB4_FAIL_MSK            (0x08u)

#define BQ7971X_FAULT_CB_FETOW3_CB3_FAIL_POS            (0x02u)
#define BQ7971X_FAULT_CB_FETOW3_CB3_FAIL_MSK            (0x04u)

#define BQ7971X_FAULT_CB_FETOW3_CB2_FAIL_POS            (0x01u)
#define BQ7971X_FAULT_CB_FETOW3_CB2_FAIL_MSK            (0x02u)

#define BQ7971X_FAULT_CB_FETOW3_CB1_FAIL_POS            (0x00u)
#define BQ7971X_FAULT_CB_FETOW3_CB1_FAIL_MSK            (0x01u)

/* --------------------------------------------------------------------------
 * FAULT_OC             (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_FAULT_OC_OFFSET                         (0x55Bu)
#define BQ7971X_FAULT_OC_POR_VAL                        (0x00u)

#define BQ7971X_FAULT_OC_OC_PFAIL_POS                   (0x04u)
#define BQ7971X_FAULT_OC_OC_PFAIL_MSK                   (0x10u)

#define BQ7971X_FAULT_OC_OCC_POS                        (0x03u)
#define BQ7971X_FAULT_OC_OCC_MSK                        (0x08u)

#define BQ7971X_FAULT_OC_OCD_POS                        (0x02u)
#define BQ7971X_FAULT_OC_OCD_MSK                        (0x04u)

#define BQ7971X_FAULT_OC_OCC_DIAGFAIL_POS               (0x01u)
#define BQ7971X_FAULT_OC_OCC_DIAGFAIL_MSK               (0x02u)

#define BQ7971X_FAULT_OC_OCD_DIAGFAIL_POS               (0x00u)
#define BQ7971X_FAULT_OC_OCD_DIAGFAIL_MSK               (0x01u)

/* --------------------------------------------------------------------------
 * VCELL18_HI           (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL18_HI_OFFSET                       (0x574u)
#define BQ7971X_VCELL18_HI_POR_VAL                      (0xFFu)

#define BQ7971X_VCELL18_HI_RESULT_POS                   (0x00u)
#define BQ7971X_VCELL18_HI_RESULT_MSK                   (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL17_HI           (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL17_HI_OFFSET                       (0x576u)
#define BQ7971X_VCELL17_HI_POR_VAL                      (0xFFu)

#define BQ7971X_VCELL17_HI_RESULT_POS                   (0x00u)
#define BQ7971X_VCELL17_HI_RESULT_MSK                   (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL16_HI           (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL16_HI_OFFSET                       (0x578u)
#define BQ7971X_VCELL16_HI_POR_VAL                      (0xFFu)

#define BQ7971X_VCELL16_HI_RESULT_POS                   (0x00u)
#define BQ7971X_VCELL16_HI_RESULT_MSK                   (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL15_HI           (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL15_HI_OFFSET                       (0x57Au)
#define BQ7971X_VCELL15_HI_POR_VAL                      (0xFFu)

#define BQ7971X_VCELL15_HI_RESULT_POS                   (0x00u)
#define BQ7971X_VCELL15_HI_RESULT_MSK                   (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL14_HI           (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL14_HI_OFFSET                       (0x57Cu)
#define BQ7971X_VCELL14_HI_POR_VAL                      (0xFFu)

#define BQ7971X_VCELL14_HI_RESULT_POS                   (0x00u)
#define BQ7971X_VCELL14_HI_RESULT_MSK                   (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL13_HI           (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL13_HI_OFFSET                       (0x57Eu)
#define BQ7971X_VCELL13_HI_POR_VAL                      (0xFFu)

#define BQ7971X_VCELL13_HI_RESULT_POS                   (0x00u)
#define BQ7971X_VCELL13_HI_RESULT_MSK                   (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL12_HI           (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL12_HI_OFFSET                       (0x580u)
#define BQ7971X_VCELL12_HI_POR_VAL                      (0xFFu)

#define BQ7971X_VCELL12_HI_RESULT_POS                   (0x00u)
#define BQ7971X_VCELL12_HI_RESULT_MSK                   (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL11_HI           (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL11_HI_OFFSET                       (0x582u)
#define BQ7971X_VCELL11_HI_POR_VAL                      (0xFFu)

#define BQ7971X_VCELL11_HI_RESULT_POS                   (0x00u)
#define BQ7971X_VCELL11_HI_RESULT_MSK                   (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL10_HI           (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL10_HI_OFFSET                       (0x584u)
#define BQ7971X_VCELL10_HI_POR_VAL                      (0xFFu)

#define BQ7971X_VCELL10_HI_RESULT_POS                   (0x00u)
#define BQ7971X_VCELL10_HI_RESULT_MSK                   (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL9_HI            (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL9_HI_OFFSET                        (0x586u)
#define BQ7971X_VCELL9_HI_POR_VAL                       (0xFFu)

#define BQ7971X_VCELL9_HI_RESULT_POS                    (0x00u)
#define BQ7971X_VCELL9_HI_RESULT_MSK                    (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL8_HI            (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL8_HI_OFFSET                        (0x588u)
#define BQ7971X_VCELL8_HI_POR_VAL                       (0xFFu)

#define BQ7971X_VCELL8_HI_RESULT_POS                    (0x00u)
#define BQ7971X_VCELL8_HI_RESULT_MSK                    (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL7_HI            (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL7_HI_OFFSET                        (0x58Au)
#define BQ7971X_VCELL7_HI_POR_VAL                       (0xFFu)

#define BQ7971X_VCELL7_HI_RESULT_POS                    (0x00u)
#define BQ7971X_VCELL7_HI_RESULT_MSK                    (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL6_HI            (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL6_HI_OFFSET                        (0x58Cu)
#define BQ7971X_VCELL6_HI_POR_VAL                       (0xFFu)

#define BQ7971X_VCELL6_HI_RESULT_POS                    (0x00u)
#define BQ7971X_VCELL6_HI_RESULT_MSK                    (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL5_HI            (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL5_HI_OFFSET                        (0x58Eu)
#define BQ7971X_VCELL5_HI_POR_VAL                       (0xFFu)

#define BQ7971X_VCELL5_HI_RESULT_POS                    (0x00u)
#define BQ7971X_VCELL5_HI_RESULT_MSK                    (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL4_HI            (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL4_HI_OFFSET                        (0x590u)
#define BQ7971X_VCELL4_HI_POR_VAL                       (0xFFu)

#define BQ7971X_VCELL4_HI_RESULT_POS                    (0x00u)
#define BQ7971X_VCELL4_HI_RESULT_MSK                    (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL3_HO            (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL3_HO_OFFSET                        (0x592u)
#define BQ7971X_VCELL3_HO_POR_VAL                       (0xFFu)

#define BQ7971X_VCELL3_HO_RESULT_POS                    (0x00u)
#define BQ7971X_VCELL3_HO_RESULT_MSK                    (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL2_HI            (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL2_HI_OFFSET                        (0x594u)
#define BQ7971X_VCELL2_HI_POR_VAL                       (0xFFu)

#define BQ7971X_VCELL2_HI_RESULT_POS                    (0x00u)
#define BQ7971X_VCELL2_HI_RESULT_MSK                    (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL1_HI            (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL1_HI_OFFSET                        (0x596u)
#define BQ7971X_VCELL1_HI_POR_VAL                       (0xFFu)

#define BQ7971X_VCELL1_HI_RESULT_POS                    (0x00u)
#define BQ7971X_VCELL1_HI_RESULT_MSK                    (0xFFu)

/* --------------------------------------------------------------------------
 * VCELL_ACT_SUM_HI             (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_VCELL_ACT_SUM_HI_OFFSET                 (0x598u)
#define BQ7971X_VCELL_ACT_SUM_HI_POR_VAL                (0xFFu)

#define BQ7971X_VCELL_ACT_SUM_HI_RESULT_POS             (0x00u)
#define BQ7971X_VCELL_ACT_SUM_HI_RESULT_MSK             (0xFFu)

/* --------------------------------------------------------------------------
 * BAT_HI           (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_BAT_HI_OFFSET                           (0x59Au)
#define BQ7971X_BAT_HI_POR_VAL                          (0xFFu)

#define BQ7971X_BAT_HI_RESULT_POS                       (0x00u)
#define BQ7971X_BAT_HI_RESULT_MSK                       (0xFFu)

/* --------------------------------------------------------------------------
 * CURRENT_HI           (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_CURRENT_HI_OFFSET                       (0x59Cu)
#define BQ7971X_CURRENT_HI_POR_VAL                      (0x80u)

#define BQ7971X_CURRENT_HI_RESULT_POS                   (0x00u)
#define BQ7971X_CURRENT_HI_RESULT_MSK                   (0xFFu)

/* --------------------------------------------------------------------------
 * GPIO1_HI             (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO1_HI_OFFSET                         (0x5A8u)
#define BQ7971X_GPIO1_HI_POR_VAL                        (0xFFu)

#define BQ7971X_GPIO1_HI_RESULT_POS                     (0x00u)
#define BQ7971X_GPIO1_HI_RESULT_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * GPIO2_HI             (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO2_HI_OFFSET                         (0x5AAu)
#define BQ7971X_GPIO2_HI_POR_VAL                        (0xFFu)

#define BQ7971X_GPIO2_HI_RESULT_POS                     (0x00u)
#define BQ7971X_GPIO2_HI_RESULT_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * GPIO3_HI             (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO3_HI_OFFSET                         (0x5ACu)
#define BQ7971X_GPIO3_HI_POR_VAL                        (0xFFu)

#define BQ7971X_GPIO3_HI_RESULT_POS                     (0x00u)
#define BQ7971X_GPIO3_HI_RESULT_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * GPIO4_HI             (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO4_HI_OFFSET                         (0x5AEu)
#define BQ7971X_GPIO4_HI_POR_VAL                        (0xFFu)

#define BQ7971X_GPIO4_HI_RESULT_POS                     (0x00u)
#define BQ7971X_GPIO4_HI_RESULT_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * GPIO5_HI             (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO5_HI_OFFSET                         (0x5B0u)
#define BQ7971X_GPIO5_HI_POR_VAL                        (0xFFu)

#define BQ7971X_GPIO5_HI_RESULT_POS                     (0x00u)
#define BQ7971X_GPIO5_HI_RESULT_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * GPIO6_HI             (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO6_HI_OFFSET                         (0x5B2u)
#define BQ7971X_GPIO6_HI_POR_VAL                        (0xFFu)

#define BQ7971X_GPIO6_HI_RESULT_POS                     (0x00u)
#define BQ7971X_GPIO6_HI_RESULT_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * GPIO7_HI             (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO7_HI_OFFSET                         (0x5B4u)
#define BQ7971X_GPIO7_HI_POR_VAL                        (0xFFu)

#define BQ7971X_GPIO7_HI_RESULT_POS                     (0x00u)
#define BQ7971X_GPIO7_HI_RESULT_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * GPIO8_HI             (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO8_HI_OFFSET                         (0x5B6u)
#define BQ7971X_GPIO8_HI_POR_VAL                        (0xFFu)

#define BQ7971X_GPIO8_HI_RESULT_POS                     (0x00u)
#define BQ7971X_GPIO8_HI_RESULT_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * GPIO9_HI             (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO9_HI_OFFSET                         (0x5B8u)
#define BQ7971X_GPIO9_HI_POR_VAL                        (0xFFu)

#define BQ7971X_GPIO9_HI_RESULT_POS                     (0x00u)
#define BQ7971X_GPIO9_HI_RESULT_MSK                     (0xFFu)

/* --------------------------------------------------------------------------
 * GPIO10_HI            (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO10_HI_OFFSET                        (0x5BAu)
#define BQ7971X_GPIO10_HI_POR_VAL                       (0xFFu)

#define BQ7971X_GPIO10_HI_RESULT_POS                    (0x00u)
#define BQ7971X_GPIO10_HI_RESULT_MSK                    (0xFFu)

/* --------------------------------------------------------------------------
 * GPIO11_HI            (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_GPIO11_HI_OFFSET                        (0x5BCu)
#define BQ7971X_GPIO11_HI_POR_VAL                       (0xFFu)

#define BQ7971X_GPIO11_HI_RESULT_POS                    (0x00u)
#define BQ7971X_GPIO11_HI_RESULT_MSK                    (0xFFu)

/* --------------------------------------------------------------------------
 * DIAG_MAIN_HI             (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIAG_MAIN_HI_OFFSET                     (0x5EAu)
#define BQ7971X_DIAG_MAIN_HI_POR_VAL                    (0xFFu)

#define BQ7971X_DIAG_MAIN_HI_RESULT_POS                 (0x00u)
#define BQ7971X_DIAG_MAIN_HI_RESULT_MSK                 (0xFFu)

/* --------------------------------------------------------------------------
 * DIAG_RDNT_HI             (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIAG_RDNT_HI_OFFSET             (0x5EDu)
#define BQ7971X_DIAG_RDNT_HI_POR_VAL            (0xFFu)

#define BQ7971X_DIAG_RDNT_HI_RESULT_POS             (0x00u)
#define BQ7971X_DIAG_RDNT_HI_RESULT_MSK             (0xFFu)

/* --------------------------------------------------------------------------
 * DIETEMP1_HI          (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIETEMP1_HI_OFFSET          (0x5F0u)
#define BQ7971X_DIETEMP1_HI_POR_VAL             (0x80u)

#define BQ7971X_DIETEMP1_HI_RESULT_POS          (0x00u)
#define BQ7971X_DIETEMP1_HI_RESULT_MSK          (0xFFu)

/* --------------------------------------------------------------------------
 * DIETEMP2_HI          (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIETEMP2_HI_OFFSET                      (0x5F2u)
#define BQ7971X_DIETEMP2_HI_POR_VAL                     (0x80u)

#define BQ7971X_DIETEMP2_HI_RESULT_POS                  (0x00u)
#define BQ7971X_DIETEMP2_HI_RESULT_MSK                  (0xFFu)

/* --------------------------------------------------------------------------
 * REF_CAP_HI           (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_REF_CAP_HI_OFFSET                       (0x5F4u)
#define BQ7971X_REF_CAP_HI_POR_VAL                      (0xFFu)

#define BQ7971X_REF_CAP_HI_RESULT_POS                   (0x00u)
#define BQ7971X_REF_CAP_HI_RESULT_MSK                   (0xFFu)

/* --------------------------------------------------------------------------
 * DIAG_D1_HI           (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIAG_D1_HI_OFFSET                       (0x5F6u)
#define BQ7971X_DIAG_D1_HI_POR_VAL                      (0xFFu)

#define BQ7971X_DIAG_D1_HI_RESULT_POS                   (0x00u)
#define BQ7971X_DIAG_D1_HI_RESULT_MSK                   (0xFFu)

/* --------------------------------------------------------------------------
 * DIAG_D2_HI           (RM):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DIAG_D2_HI_OFFSET                       (0x5F8u)
#define BQ7971X_DIAG_D2_HI_POR_VAL                      (0xFFu)

#define BQ7971X_DIAG_D2_HI_RESULT_POS                   (0x00u)
#define BQ7971X_DIAG_D2_HI_RESULT_MSK                   (0xFFu)

/* --------------------------------------------------------------------------
 * REF_CAP_T0_HI            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_REF_CAP_T0_HI_OFFSET                    (0x5FAu)
#define BQ7971X_REF_CAP_T0_HI_POR_VAL                   (0xFFu)

#define BQ7971X_REF_CAP_T0_HI_RESULT_POS                (0x00u)
#define BQ7971X_REF_CAP_T0_HI_RESULT_MSK                (0xFFu)

/* --------------------------------------------------------------------------
 * REF_CAP_T0_LO            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_REF_CAP_T0_LO_OFFSET                    (0x5FBu)
#define BQ7971X_REF_CAP_T0_LO_POR_VAL                   (0xFFu)

#define BQ7971X_REF_CAP_T0_LO_RESULT_POS                (0x00u)
#define BQ7971X_REF_CAP_T0_LO_RESULT_MSK                (0xFFu)

/* --------------------------------------------------------------------------
 * I2C_RD_DATA          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_I2C_RD_DATA_OFFSET                      (0x610u)
#define BQ7971X_I2C_RD_DATA_POR_VAL                     (0x00u)

#define BQ7971X_I2C_RD_DATA_DATA_POS                    (0x00u)
#define BQ7971X_I2C_RD_DATA_DATA_MSK                    (0xFFu)

/* --------------------------------------------------------------------------
 * SPI_RX3          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_SPI_RX3_OFFSET                          (0x620u)
#define BQ7971X_SPI_RX3_POR_VAL                         (0x00u)

#define BQ7971X_SPI_RX3_DATA_POS                        (0x00u)
#define BQ7971X_SPI_RX3_DATA_MSK                        (0xFFu)

/* --------------------------------------------------------------------------
 * SPI_RX2          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_SPI_RX2_OFFSET                          (0x621u)
#define BQ7971X_SPI_RX2_POR_VAL                         (0x00u)

#define BQ7971X_SPI_RX2_DATA_POS                        (0x00u)
#define BQ7971X_SPI_RX2_DATA_MSK                        (0xFFu)

/* --------------------------------------------------------------------------
 * SPI_RX1          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_SPI_RX1_OFFSET                          (0x622u)
#define BQ7971X_SPI_RX1_POR_VAL                         (0x00u)

#define BQ7971X_SPI_RX1_DATA_POS                        (0x00u)
#define BQ7971X_SPI_RX1_DATA_MSK                        (0xFFu)

/* --------------------------------------------------------------------------
 * DEBUG_CTRL_UNLOCK            (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DEBUG_CTRL_UNLOCK_OFFSET                (0x700u)
#define BQ7971X_DEBUG_CTRL_UNLOCK_POR_VAL               (0x00u)

#define BQ7971X_DEBUG_CTRL_UNLOCK_CODE_POS              (0x00u)
#define BQ7971X_DEBUG_CTRL_UNLOCK_CODE_MSK              (0xFFu)

/* --------------------------------------------------------------------------
 * DEBUG_COMM_CTRL1             (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DEBUG_COMM_CTRL1_OFFSET                 (0x701u)
#define BQ7971X_DEBUG_COMM_CTRL1_POR_VAL                (0x02u)

#define BQ7971X_DEBUG_COMM_CTRL1_CT_DIS_POS             (0x05u)
#define BQ7971X_DEBUG_COMM_CTRL1_CT_DIS_MSK             (0x20u)

#define BQ7971X_DEBUG_COMM_CTRL1_TWO_STOP_EN_POS        (0x04u)
#define BQ7971X_DEBUG_COMM_CTRL1_TWO_STOP_EN_MSK        (0x10u)

#define BQ7971X_DEBUG_COMM_CTRL1_UART_BAUD_POS          (0x03u)
#define BQ7971X_DEBUG_COMM_CTRL1_UART_BAUD_MSK          (0x08u)

#define BQ7971X_DEBUG_COMM_CTRL1_UART_MIRROR_EN_POS     (0x02u)
#define BQ7971X_DEBUG_COMM_CTRL1_UART_MIRROR_EN_MSK     (0x04u)

#define BQ7971X_DEBUG_COMM_CTRL1_UART_TX_EN_POS         (0x01u)
#define BQ7971X_DEBUG_COMM_CTRL1_UART_TX_EN_MSK         (0x02u)

#define BQ7971X_DEBUG_COMM_CTRL1_USER_UART_EN_POS       (0x00u)
#define BQ7971X_DEBUG_COMM_CTRL1_USER_UART_EN_MSK       (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_COMM_CTRL2             (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DEBUG_COMM_CTRL2_OFFSET                 (0x702u)
#define BQ7971X_DEBUG_COMM_CTRL2_POR_VAL                (0x1Eu)

#define BQ7971X_DEBUG_COMM_CTRL2_FCOMM_DIS_POS          (0x05u)
#define BQ7971X_DEBUG_COMM_CTRL2_FCOMM_DIS_MSK          (0x20u)

#define BQ7971X_DEBUG_COMM_CTRL2_COML_TX_EN_POS         (0x04u)
#define BQ7971X_DEBUG_COMM_CTRL2_COML_TX_EN_MSK         (0x10u)

#define BQ7971X_DEBUG_COMM_CTRL2_COML_RX_EN_POS         (0x03u)
#define BQ7971X_DEBUG_COMM_CTRL2_COML_RX_EN_MSK         (0x08u)

#define BQ7971X_DEBUG_COMM_CTRL2_COMH_TX_EN_POS         (0x02u)
#define BQ7971X_DEBUG_COMM_CTRL2_COMH_TX_EN_MSK         (0x04u)

#define BQ7971X_DEBUG_COMM_CTRL2_COMH_RX_EN_POS         (0x01u)
#define BQ7971X_DEBUG_COMM_CTRL2_COMH_RX_EN_MSK         (0x02u)

#define BQ7971X_DEBUG_COMM_CTRL2_USER_DAISY_EN_POS      (0x00u)
#define BQ7971X_DEBUG_COMM_CTRL2_USER_DAISY_EN_MSK      (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_COMM_STAT          (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DEBUG_COMM_STAT_OFFSET                  (0x780u)
#define BQ7971X_DEBUG_COMM_STAT_POR_VAL                 (0x33u)

#define BQ7971X_DEBUG_COMM_STAT_HW_UART_DRV_POS         (0x05u)
#define BQ7971X_DEBUG_COMM_STAT_HW_UART_DRV_MSK         (0x20u)

#define BQ7971X_DEBUG_COMM_STAT_HW_DAISY_DRV_POS        (0x04u)
#define BQ7971X_DEBUG_COMM_STAT_HW_DAISY_DRV_MSK        (0x10u)

#define BQ7971X_DEBUG_COMM_STAT_COML_TX_ON_POS          (0x03u)
#define BQ7971X_DEBUG_COMM_STAT_COML_TX_ON_MSK          (0x08u)

#define BQ7971X_DEBUG_COMM_STAT_COML_RX_ON_POS          (0x02u)
#define BQ7971X_DEBUG_COMM_STAT_COML_RX_ON_MSK          (0x04u)

#define BQ7971X_DEBUG_COMM_STAT_COMH_TX_ON_POS          (0x01u)
#define BQ7971X_DEBUG_COMM_STAT_COMH_TX_ON_MSK          (0x02u)

#define BQ7971X_DEBUG_COMM_STAT_COMH_RX_ON_POS          (0x00u)
#define BQ7971X_DEBUG_COMM_STAT_COMH_RX_ON_MSK          (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_UART_RC_TR             (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DEBUG_UART_RC_TR_OFFSET                 (0x781u)
#define BQ7971X_DEBUG_UART_RC_TR_POR_VAL                (0x00u)

#define BQ7971X_DEBUG_UART_RC_TR_TR_SOF_POS             (0x07u)
#define BQ7971X_DEBUG_UART_RC_TR_TR_SOF_MSK             (0x80u)

#define BQ7971X_DEBUG_UART_RC_TR_TR_WAIT_POS            (0x06u)
#define BQ7971X_DEBUG_UART_RC_TR_TR_WAIT_MSK            (0x40u)

#define BQ7971X_DEBUG_UART_RC_TR_RC_IERR_POS            (0x05u)
#define BQ7971X_DEBUG_UART_RC_TR_RC_IERR_MSK            (0x20u)

#define BQ7971X_DEBUG_UART_RC_TR_RC_TXDIS_POS           (0x04u)
#define BQ7971X_DEBUG_UART_RC_TR_RC_TXDIS_MSK           (0x10u)

#define BQ7971X_DEBUG_UART_RC_TR_RC_SOF_POS             (0x03u)
#define BQ7971X_DEBUG_UART_RC_TR_RC_SOF_MSK             (0x08u)

#define BQ7971X_DEBUG_UART_RC_TR_RC_BYTE_ERR_POS        (0x02u)
#define BQ7971X_DEBUG_UART_RC_TR_RC_BYTE_ERR_MSK        (0x04u)

#define BQ7971X_DEBUG_UART_RC_TR_RC_UNEXP_POS           (0x01u)
#define BQ7971X_DEBUG_UART_RC_TR_RC_UNEXP_MSK           (0x02u)

#define BQ7971X_DEBUG_UART_RC_TR_RC_CRC_POS             (0x00u)
#define BQ7971X_DEBUG_UART_RC_TR_RC_CRC_MSK             (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_COMH_BIT           (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DEBUG_COMH_BIT_OFFSET                   (0x782u)
#define BQ7971X_DEBUG_COMH_BIT_POR_VAL                  (0x00u)

#define BQ7971X_DEBUG_COMH_BIT_PERR_POS                 (0x04u)
#define BQ7971X_DEBUG_COMH_BIT_PERR_MSK                 (0x10u)

#define BQ7971X_DEBUG_COMH_BIT_BERR_TAG_POS             (0x03u)
#define BQ7971X_DEBUG_COMH_BIT_BERR_TAG_MSK             (0x08u)

#define BQ7971X_DEBUG_COMH_BIT_SYNC2_POS                (0x02u)
#define BQ7971X_DEBUG_COMH_BIT_SYNC2_MSK                (0x04u)

#define BQ7971X_DEBUG_COMH_BIT_SYNC1_POS                (0x01u)
#define BQ7971X_DEBUG_COMH_BIT_SYNC1_MSK                (0x02u)

#define BQ7971X_DEBUG_COMH_BIT_BIT_POS                  (0x00u)
#define BQ7971X_DEBUG_COMH_BIT_BIT_MSK                  (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_COMH_RC_TR                         (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DEBUG_COMH_RC_TR_OFFSET                 (0x783u)
#define BQ7971X_DEBUG_COMH_RC_TR_POR_VAL                (0x00u)

#define BQ7971X_DEBUG_COMH_RC_TR_TR_WAIT_POS            (0x06u)
#define BQ7971X_DEBUG_COMH_RC_TR_TR_WAIT_MSK            (0x40u)

#define BQ7971X_DEBUG_COMH_RC_TR_RC_IERR_POS            (0x05u)
#define BQ7971X_DEBUG_COMH_RC_TR_RC_IERR_MSK            (0x20u)

#define BQ7971X_DEBUG_COMH_RC_TR_RC_TXDIS_POS           (0x04u)
#define BQ7971X_DEBUG_COMH_RC_TR_RC_TXDIS_MSK           (0x10u)

#define BQ7971X_DEBUG_COMH_RC_TR_RC_SOF_POS             (0x03u)
#define BQ7971X_DEBUG_COMH_RC_TR_RC_SOF_MSK             (0x08u)

#define BQ7971X_DEBUG_COMH_RC_TR_RC_BYTE_ERR_POS        (0x02u)
#define BQ7971X_DEBUG_COMH_RC_TR_RC_BYTE_ERR_MSK        (0x04u)

#define BQ7971X_DEBUG_COMH_RC_TR_RC_UNEXP_POS           (0x01u)
#define BQ7971X_DEBUG_COMH_RC_TR_RC_UNEXP_MSK           (0x02u)

#define BQ7971X_DEBUG_COMH_RC_TR_RC_CRC_POS             (0x00u)
#define BQ7971X_DEBUG_COMH_RC_TR_RC_CRC_MSK             (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_COMH_RR            (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DEBUG_COMH_RR_OFFSET                    (0x784u)
#define BQ7971X_DEBUG_COMH_RR_POR_VAL                   (0x00u)

#define BQ7971X_DEBUG_COMH_RR_RR_TXDIS_POS              (0x04u)
#define BQ7971X_DEBUG_COMH_RR_RR_TXDIS_MSK              (0x10u)

#define BQ7971X_DEBUG_COMH_RR_RR_SOF_POS                (0x03u)
#define BQ7971X_DEBUG_COMH_RR_RR_SOF_MSK                (0x08u)

#define BQ7971X_DEBUG_COMH_RR_RR_BYTE_ERR_POS           (0x02u)
#define BQ7971X_DEBUG_COMH_RR_RR_BYTE_ERR_MSK           (0x04u)

#define BQ7971X_DEBUG_COMH_RR_RR_UNEXP_POS              (0x01u)
#define BQ7971X_DEBUG_COMH_RR_RR_UNEXP_MSK              (0x02u)

#define BQ7971X_DEBUG_COMH_RR_RR_CRC_POS                (0x00u)
#define BQ7971X_DEBUG_COMH_RR_RR_CRC_MSK                (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_COML_BIT (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DEBUG_COML_BIT_OFFSET                   (0x785u)
#define BQ7971X_DEBUG_COML_BIT_POR_VAL                  (0x00u)

#define BQ7971X_DEBUG_COML_BIT_PERR_POS                 (0x04u)
#define BQ7971X_DEBUG_COML_BIT_PERR_MSK                 (0x10u)

#define BQ7971X_DEBUG_COML_BIT_BERR_TAG_POS             (0x03u)
#define BQ7971X_DEBUG_COML_BIT_BERR_TAG_MSK             (0x08u)

#define BQ7971X_DEBUG_COML_BIT_SYNC2_POS                (0x02u)
#define BQ7971X_DEBUG_COML_BIT_SYNC2_MSK                (0x04u)

#define BQ7971X_DEBUG_COML_BIT_SYNC1_POS                (0x01u)
#define BQ7971X_DEBUG_COML_BIT_SYNC1_MSK                (0x02u)

#define BQ7971X_DEBUG_COML_BIT_BIT_POS                  (0x00u)
#define BQ7971X_DEBUG_COML_BIT_BIT_MSK                  (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_COML_RC_TR         (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DEBUG_COML_RC_TR_OFFSET                 (0x786u)
#define BQ7971X_DEBUG_COML_RC_TR_POR_VAL                (0x00u)

#define BQ7971X_DEBUG_COML_RC_TR_TR_WAIT_POS            (0x06u)
#define BQ7971X_DEBUG_COML_RC_TR_TR_WAIT_MSK            (0x40u)

#define BQ7971X_DEBUG_COML_RC_TR_RC_IERR_POS            (0x05u)
#define BQ7971X_DEBUG_COML_RC_TR_RC_IERR_MSK            (0x20u)

#define BQ7971X_DEBUG_COML_RC_TR_RC_TXDIS_POS           (0x04u)
#define BQ7971X_DEBUG_COML_RC_TR_RC_TXDIS_MSK           (0x10u)

#define BQ7971X_DEBUG_COML_RC_TR_RC_SOF_POS             (0x03u)
#define BQ7971X_DEBUG_COML_RC_TR_RC_SOF_MSK             (0x08u)

#define BQ7971X_DEBUG_COML_RC_TR_RC_BYTE_ERR_POS        (0x02u)
#define BQ7971X_DEBUG_COML_RC_TR_RC_BYTE_ERR_MSK        (0x04u)

#define BQ7971X_DEBUG_COML_RC_TR_RC_UNEXP_POS           (0x01u)
#define BQ7971X_DEBUG_COML_RC_TR_RC_UNEXP_MSK           (0x02u)

#define BQ7971X_DEBUG_COML_RC_TR_RC_CRC_POS             (0x00u)
#define BQ7971X_DEBUG_COML_RC_TR_RC_CRC_MSK             (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_COML_RR (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DEBUG_COML_RR_OFFSET                    (0x787u)
#define BQ7971X_DEBUG_COML_RR_POR_VAL                   (0x00u)

#define BQ7971X_DEBUG_COML_RR_RR_TXDIS_POS              (0x04u)
#define BQ7971X_DEBUG_COML_RR_RR_TXDIS_MSK              (0x10u)

#define BQ7971X_DEBUG_COML_RR_RR_SOF_POS                (0x03u)
#define BQ7971X_DEBUG_COML_RR_RR_SOF_MSK                (0x08u)

#define BQ7971X_DEBUG_COML_RR_RR_BYTE_ERR_POS           (0x02u)
#define BQ7971X_DEBUG_COML_RR_RR_BYTE_ERR_MSK           (0x04u)

#define BQ7971X_DEBUG_COML_RR_RR_UNEXP_POS              (0x01u)
#define BQ7971X_DEBUG_COML_RR_RR_UNEXP_MSK              (0x02u)

#define BQ7971X_DEBUG_COML_RR_RR_CRC_POS                (0x00u)
#define BQ7971X_DEBUG_COML_RR_RR_CRC_MSK                (0x01u)

/* --------------------------------------------------------------------------
 * DEBUG_SEC_DED_BLK (R):
 * -------------------------------------------------------------------------- */

#define BQ7971X_DEBUG_SEC_DED_BLK_OFFSET                (0x788u)
#define BQ7971X_DEBUG_SEC_DED_BLK_POR_VAL               (0x00u)

#define BQ7971X_DEBUG_SEC_DED_BLK_SEC_DED_POS           (0x06u)
#define BQ7971X_DEBUG_SEC_DED_BLK_SEC_DED_MSK           (0x40u)
#define BQ7971X_DEBUG_SEC_DED_BLK_BLOCK_POS             (0x00u)
#define BQ7971X_DEBUG_SEC_DED_BLK_BLOCK_MSK             (0x3Fu)



/*             9.1            */
#define BQ7971X_FACT_TM_REG                  (0xE00u)
#define BQ7971X_DIAG_NUM_FF                  ((uint8)0xFFu)
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

#endif /* BQ7971X_REGS_H */

/*********************************************************************************************************************
 * End of File: bq7971x_regs.h
 *********************************************************************************************************************/
