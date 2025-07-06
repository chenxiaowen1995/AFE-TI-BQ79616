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
 *  File:       bq7973x_regs.h
 *  Project:    TIBMS
 *  Module:     PMI
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  BQ7973x Register Map
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

#ifndef BQ7973X_REGS_H
#define BQ7973X_REGS_H

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

#define BQ7973X_REGS_MAJOR_VERSION                  (0x01u)
#define BQ7973X_REGS_MINOR_VERSION                  (0x00u)
#define BQ7973X_REGS_PATCH_VERSION                  (0x00u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/
/* 9.1 */
#define BQ7973X_FACT_TM_REG                  (0xE00u)/* The test mode status register address */



#define BQ7973X_GPIO_CONF1_6_REG_NUM                (6u)
#define BQ7973X_GPIO_CONF7_8_REG_NUM                (2u)
#define BQ7973X_OC_CONF_REG_NUM                     (8u)
#define BQ7973X_ADC_CTRL_REG_NUM                    (4u)
#define BQ7973X_BAL_CTRL_REG_NUM                    (3u)
#define BQ7973X_GPIO_REG_NUM                        (15u)
#define BQ7973X_CURRENT_REG_NUM                     (2u)
#define BQ7973X_VF_REG_NUM                          (2u)
#define BQ7973X_SW_REG_NUM                          (2u)
#define BQ7973X_CC_REG_NUM                          (5u)

/* --------------------------------------------------------------------------
 * CUST_CRC_HI (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CUST_CRC_HI_OFFSET                  (0x000u)
#define BQ7973X_CUST_CRC_HI_POR_VAL                 (0x03u)
#define BQ7973X_CUST_CRC_HI_CRC_POS                 (0u)
#define BQ7973X_CUST_CRC_HI_CRC_MSK                 (0xffu)

/* --------------------------------------------------------------------------
 * CUST_CRC_LO (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CUST_CRC_LO_OFFSET                  (0x001u)
#define BQ7973X_CUST_CRC_LO_POR_VAL                 (0x2Fu)
#define BQ7973X_CUST_CRC_LO_CRC_POS                 (0u)
#define BQ7973X_CUST_CRC_LO_CRC_MSK                 (0xffu)

/* --------------------------------------------------------------------------
 * DEV_CONF1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEV_CONF1_OFFSET                    (0x002u)
#define BQ7973X_DEV_CONF1_POR_VAL                   (0x04u)

#define BQ7973X_DEV_CONF1_NFAULT_EN_POS             (2u)
#define BQ7973X_DEV_CONF1_NFAULT_EN_MSK             (0x4u)

#define BQ7973X_DEV_CONF1_FTONE_EN_POS              (1u)
#define BQ7973X_DEV_CONF1_FTONE_EN_MSK              (0x2u)

#define BQ7973X_DEV_CONF1_HB_EN_POS                 (0u)
#define BQ7973X_DEV_CONF1_HB_EN_MSK                 (0x1u)

/* --------------------------------------------------------------------------
 * OTP_SPARE26 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE26_OFFSET                  (0x003u)
#define BQ7973X_OTP_SPARE26_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * COMM_CONF (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_COMM_CONF_OFFSET                    (0x004u)
#define BQ7973X_COMM_CONF_POR_VAL                   (0x00u)

#define BQ7973X_COMM_CONF_CT_EN_POS                 (7u)
#define BQ7973X_COMM_CONF_CT_EN_MSK                 (0x80u)

#define BQ7973X_COMM_CONF_VIF_DIS_POS               (6u)
#define BQ7973X_COMM_CONF_VIF_DIS_MSK               (0x40u)

#define BQ7973X_COMM_CONF_RX_RLX_FAULT_POS          (5u)
#define BQ7973X_COMM_CONF_RX_RLX_FAULT_MSK          (0x20u)

#define BQ7973X_COMM_CONF_RX_RLX_THRESH_POS         (4u)
#define BQ7973X_COMM_CONF_RX_RLX_THRESH_MSK         (0x10u)

#define BQ7973X_COMM_CONF_RX_RLX_SAMP_POS           (3u)
#define BQ7973X_COMM_CONF_RX_RLX_SAMP_MSK           (0x8u)

#define BQ7973X_COMM_CONF_STK_RESP_DLY_POS          (1u)
#define BQ7973X_COMM_CONF_STK_RESP_DLY_MSK          (0x6u)

#define BQ7973X_COMM_CONF_TX_HOLD_OFF_POS           (0u)
#define BQ7973X_COMM_CONF_TX_HOLD_OFF_MSK           (0x1u)

/* --------------------------------------------------------------------------
 * OTP_SPARE25 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE25_OFFSET                  (0x005u)
#define BQ7973X_OTP_SPARE25_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE24 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE24_OFFSET                  (0x006u)
#define BQ7973X_OTP_SPARE24_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE23 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE23_OFFSET                  (0x007u)
#define BQ7973X_OTP_SPARE23_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * ADC_CONF (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_ADC_CONF_OFFSET                     (0x008u)
#define BQ7973X_ADC_CONF_POR_VAL                    (0x00u)

#define BQ7973X_ADC_CONF_ADC_DLY_POS                (0u)
#define BQ7973X_ADC_CONF_ADC_DLY_MSK                (0xffu)

/* --------------------------------------------------------------------------
 * OTP_SPARE22 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE22_OFFSET                  (0x009u)
#define BQ7973X_OTP_SPARE22_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE21 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE21_OFFSET                  (0x00Au)
#define BQ7973X_OTP_SPARE21_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE20 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE20_OFFSET                  (0x00Bu)
#define BQ7973X_OTP_SPARE20_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE19 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE19_OFFSET                  (0x00Cu)
#define BQ7973X_OTP_SPARE19_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE18 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE18_OFFSET                  (0x00Du)
#define BQ7973X_OTP_SPARE18_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE17 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE17_OFFSET                  (0x00Eu)
#define BQ7973X_OTP_SPARE17_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * FAULT_MSK1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_MSK1_OFFSET                   (0x00Fu)
#define BQ7973X_FAULT_MSK1_POR_VAL                  (0x00u)

#define BQ7973X_FAULT_MSK1_MSK_OC_POS               (7u)
#define BQ7973X_FAULT_MSK1_MSK_OC_MSK               (0x80u)

#define BQ7973X_FAULT_MSK1_MSK_SYS_POS              (1u)
#define BQ7973X_FAULT_MSK1_MSK_SYS_MSK              (0x2u)

#define BQ7973X_FAULT_MSK1_MSK_PWR_POS              (0u)
#define BQ7973X_FAULT_MSK1_MSK_PWR_MSK              (0x1u)

/* --------------------------------------------------------------------------
 * FAULT_MSK2 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_MSK2_OFFSET                   (0x010u)
#define BQ7973X_FAULT_MSK2_POR_VAL                  (0x00u)

#define BQ7973X_FAULT_MSK2_MSK_ADC_POS              (7u)
#define BQ7973X_FAULT_MSK2_MSK_ADC_MSK              (0x80u)

#define BQ7973X_FAULT_MSK2_MSK_OTP_CRC_POS          (6u)
#define BQ7973X_FAULT_MSK2_MSK_OTP_CRC_MSK          (0x40u)

#define BQ7973X_FAULT_MSK2_MSK_OTP_DATA_POS         (5u)
#define BQ7973X_FAULT_MSK2_MSK_OTP_DATA_MSK         (0x20u)

#define BQ7973X_FAULT_MSK2_MSK_COMM_FCOMM_POS       (4u)
#define BQ7973X_FAULT_MSK2_MSK_COMM_FCOMM_MSK       (0x10u)

#define BQ7973X_FAULT_MSK2_MSK_COMM_FTONE_POS       (3u)
#define BQ7973X_FAULT_MSK2_MSK_COMM_FTONE_MSK       (0x8u)

#define BQ7973X_FAULT_MSK2_MSK_COMM_HB_POS          (2u)
#define BQ7973X_FAULT_MSK2_MSK_COMM_HB_MSK          (0x4u)

#define BQ7973X_FAULT_MSK2_MSK_COMM_DSY_POS         (1u)
#define BQ7973X_FAULT_MSK2_MSK_COMM_DSY_MSK         (0x2u)

#define BQ7973X_FAULT_MSK2_MSK_COMM_UART_POS        (0u)
#define BQ7973X_FAULT_MSK2_MSK_COMM_UART_MSK        (0x1u)

/* --------------------------------------------------------------------------
 * CS_ADC1_CAL1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CS_ADC1_CAL1_OFFSET                 (0x011u)
#define BQ7973X_CS_ADC1_CAL1_POR_VAL                (0x00u)

#define BQ7973X_CS_ADC1_CAL1_GAINL_POS              (0u)
#define BQ7973X_CS_ADC1_CAL1_GAINL_MSK              (0xffu)

/* --------------------------------------------------------------------------
 * CS_ADC1_CAL2 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CS_ADC1_CAL2_OFFSET                 (0x012u)
#define BQ7973X_CS_ADC1_CAL2_POR_VAL                (0x00u)

#define BQ7973X_CS_ADC1_CAL2_GAINH_POS              (5u)
#define BQ7973X_CS_ADC1_CAL2_GAINH_MSK              (0xe0u)

#define BQ7973X_CS_ADC1_CAL2_OFFSET_POS             (0u)
#define BQ7973X_CS_ADC1_CAL2_OFFSET_MSK             (0x1fu)

/* --------------------------------------------------------------------------
 * OTP_SPARE16 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE16_OFFSET                  (0x013u)
#define BQ7973X_OTP_SPARE16_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE15 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE15_OFFSET                  (0x014u)
#define BQ7973X_OTP_SPARE15_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * CUST_MISC1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CUST_MISC1_OFFSET                   (0x015u)
#define BQ7973X_CUST_MISC1_POR_VAL                  (0x00u)

#define BQ7973X_CUST_MISC1_DATA_POS                 (0u)
#define BQ7973X_CUST_MISC1_DATA_MSK                 (0xffu)

/* --------------------------------------------------------------------------
 * CUST_MISC2 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CUST_MISC2_OFFSET                   (0x016u)
#define BQ7973X_CUST_MISC2_POR_VAL                  (0x00u)

#define BQ7973X_CUST_MISC2_DATA_POS                 (0u)
#define BQ7973X_CUST_MISC2_DATA_MSK                 (0xffu)

/* --------------------------------------------------------------------------
 * CUST_MISC3 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CUST_MISC3_OFFSET                   (0x017u)
#define BQ7973X_CUST_MISC3_POR_VAL                  (0x00u)

#define BQ7973X_CUST_MISC3_DATA_POS                 (0u)
#define BQ7973X_CUST_MISC3_DATA_MSK                 (0xffu)

/* --------------------------------------------------------------------------
 * CUST_MISC4 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CUST_MISC4_OFFSET                   (0x018u)
#define BQ7973X_CUST_MISC4_POR_VAL                  (0x00u)

#define BQ7973X_CUST_MISC4_DATA_POS                 (0u)
#define BQ7973X_CUST_MISC4_DATA_MSK                 (0xffu)

/* --------------------------------------------------------------------------
 * CUST_MISC5 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CUST_MISC5_OFFSET                   (0x019u)
#define BQ7973X_CUST_MISC5_POR_VAL                  (0x00u)

#define BQ7973X_CUST_MISC5_DATA_POS                 (0u)
#define BQ7973X_CUST_MISC5_DATA_MSK                 (0xffu)

/* --------------------------------------------------------------------------
 * CUST_MISC6 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CUST_MISC6_OFFSET                   (0x01Au)
#define BQ7973X_CUST_MISC6_POR_VAL                  (0x00u)

#define BQ7973X_CUST_MISC6_DATA_POS                 (0u)
#define BQ7973X_CUST_MISC6_DATA_MSK                 (0xffu)

/* --------------------------------------------------------------------------
 * CUST_MISC7 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CUST_MISC7_OFFSET                   (0x01Bu)
#define BQ7973X_CUST_MISC7_POR_VAL                  (0x00u)

#define BQ7973X_CUST_MISC7_DATA_POS                 (0u)
#define BQ7973X_CUST_MISC7_DATA_MSK                 (0xffu)

/* --------------------------------------------------------------------------
 * CUST_MISC8 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CUST_MISC8_OFFSET                   (0x01Cu)
#define BQ7973X_CUST_MISC8_POR_VAL                  (0x00u)

#define BQ7973X_CUST_MISC8_DATA_POS                 (0u)
#define BQ7973X_CUST_MISC8_DATA_MSK                 (0xffu)

/* --------------------------------------------------------------------------
 * OTP_SPARE14 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE14_OFFSET                  (0x01Du)
#define BQ7973X_OTP_SPARE14_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * GPIO_CONF1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO_CONF1_OFFSET                   (0x01Eu)
#define BQ7973X_GPIO_CONF1_POR_VAL                  (0x00u)

#define BQ7973X_GPIO_CONF1_GPIO1_FLT_EN_POS         (7u)
#define BQ7973X_GPIO_CONF1_GPIO1_FLT_EN_MSK         (0x80u)

#define BQ7973X_GPIO_CONF1_GPIO2_POS                (3u)
#define BQ7973X_GPIO_CONF1_GPIO2_MSK                (0x38u)

#define BQ7973X_GPIO_CONF1_GPIO1_POS                (0u)
#define BQ7973X_GPIO_CONF1_GPIO1_MSK                (0x7u)

/* --------------------------------------------------------------------------
 * GPIO_CONF2 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO_CONF2_OFFSET                   (0x01Fu)
#define BQ7973X_GPIO_CONF2_POR_VAL                  (0x00u)

#define BQ7973X_GPIO_CONF2_CS_RDY_EN_POS            (6u)
#define BQ7973X_GPIO_CONF2_CS_RDY_EN_MSK            (0x40u)

#define BQ7973X_GPIO_CONF2_GPIO4_POS                (3u)
#define BQ7973X_GPIO_CONF2_GPIO4_MSK                (0x38u)

#define BQ7973X_GPIO_CONF2_GPIO3_POS                (0u)
#define BQ7973X_GPIO_CONF2_GPIO3_MSK                (0x7u)

/* --------------------------------------------------------------------------
 * GPIO_CONF3 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO_CONF3_OFFSET                   (0x020u)
#define BQ7973X_GPIO_CONF3_POR_VAL                  (0x00u)

#define BQ7973X_GPIO_CONF3_GPIO6_POS                (3u)
#define BQ7973X_GPIO_CONF3_GPIO6_MSK                (0x38u)

#define BQ7973X_GPIO_CONF3_GPIO5_POS                (0u)
#define BQ7973X_GPIO_CONF3_GPIO5_MSK                (0x7u)

/* --------------------------------------------------------------------------
 * GPIO_CONF4 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO_CONF4_OFFSET                   (0x021u)
#define BQ7973X_GPIO_CONF4_POR_VAL                  (0x00u)

#define BQ7973X_GPIO_CONF4_GPIO8_POS                (3u)
#define BQ7973X_GPIO_CONF4_GPIO8_MSK                (0x38u)

#define BQ7973X_GPIO_CONF4_GPIO7_POS                (0u)
#define BQ7973X_GPIO_CONF4_GPIO7_MSK                (0x7u)

/* --------------------------------------------------------------------------
 * GPIO_CONF5 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO_CONF5_OFFSET                   (0x022u)
#define BQ7973X_GPIO_CONF5_POR_VAL                  (0x00u)

#define BQ7973X_GPIO_CONF5_GPIO10_POS               (3u)
#define BQ7973X_GPIO_CONF5_GPIO10_MSK               (0x38u)

#define BQ7973X_GPIO_CONF5_GPIO9_POS                (0u)
#define BQ7973X_GPIO_CONF5_GPIO9_MSK                (0x7u)

/* --------------------------------------------------------------------------
 * GPIO_CONF6 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO_CONF6_OFFSET                   (0x023u)
#define BQ7973X_GPIO_CONF6_POR_VAL                  (0x00u)

#define BQ7973X_GPIO_CONF6_GPIO11_POS               (0u)
#define BQ7973X_GPIO_CONF6_GPIO11_MSK               (0x7u)

/* --------------------------------------------------------------------------
 * OTP_SPARE13 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE13_OFFSET                  (0x024u)
#define BQ7973X_OTP_SPARE13_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE12 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE12_OFFSET                  (0x025u)
#define BQ7973X_OTP_SPARE12_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE11 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE11_OFFSET                  (0x026u)
#define BQ7973X_OTP_SPARE11_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE10 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE10_OFFSET                  (0x027u)
#define BQ7973X_OTP_SPARE10_POR_VAL                 (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE9 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE9_OFFSET                   (0x028u)
#define BQ7973X_OTP_SPARE9_POR_VAL                  (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE8 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE8_OFFSET                   (0x029u)
#define BQ7973X_OTP_SPARE8_POR_VAL                  (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE7 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE7_OFFSET                   (0x02Au)
#define BQ7973X_OTP_SPARE7_POR_VAL                  (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE6 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE6_OFFSET                   (0x02Bu)
#define BQ7973X_OTP_SPARE6_POR_VAL                  (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE5 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE5_OFFSET                   (0x02Cu)
#define BQ7973X_OTP_SPARE5_POR_VAL                  (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE4 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE4_OFFSET                   (0x02Du)
#define BQ7973X_OTP_SPARE4_POR_VAL                  (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE3 (R/W):
 * -------------------------------------------------------------------------- */
#define BQ7973X_OTP_SPARE3_OFFSET                   (0x02Eu)
#define BQ7973X_OTP_SPARE3_POR_VAL                  (0x00u)

/* --------------------------------------------------------------------------
 * OTP_SPARE2 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE2_OFFSET                   (0x02Fu)
#define BQ7973X_OTP_SPARE2_POR_VAL                  (0x00u)

/* --------------------------------------------------------------------------
 * CS_ADC2_CAL1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CS_ADC2_CAL1_OFFSET                 (0x030u)
#define BQ7973X_CS_ADC2_CAL1_POR_VAL                (0x00u)

#define BQ7973X_CS_ADC2_CAL1_GAINL_POS              (0u)
#define BQ7973X_CS_ADC2_CAL1_GAINL_MSK              (0xffu)

/* --------------------------------------------------------------------------
 * CS_ADC2_CAL2 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CS_ADC2_CAL2_OFFSET                 (0x031u)
#define BQ7973X_CS_ADC2_CAL2_POR_VAL                (0x00u)

#define BQ7973X_CS_ADC2_CAL2_GAINH_POS              (5u)
#define BQ7973X_CS_ADC2_CAL2_GAINH_MSK              (0xe0u)

#define BQ7973X_CS_ADC2_CAL2_OFFSET_POS             (0u)
#define BQ7973X_CS_ADC2_CAL2_OFFSET_MSK             (0x1fu)

/* --------------------------------------------------------------------------
 * OC_CONF1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OC_CONF1_OFFSET                     (0x032u)
#define BQ7973X_OC_CONF1_POR_VAL                    (0x00u)

#define BQ7973X_OC_CONF1_OC1_DEG_POS                (3u)
#define BQ7973X_OC_CONF1_OC1_DEG_MSK                (0x78u)

#define BQ7973X_OC_CONF1_OCC1_THRH_POS              (0u)
#define BQ7973X_OC_CONF1_OCC1_THRH_MSK              (0x7u)

/* --------------------------------------------------------------------------
 * OC_CONF2 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OC_CONF2_OFFSET                     (0x033u)
#define BQ7973X_OC_CONF2_POR_VAL                    (0x00u)

#define BQ7973X_OC_CONF2_OCC1_THRL_POS              (0u)
#define BQ7973X_OC_CONF2_OCC1_THRL_MSK              (0xffu)

/* --------------------------------------------------------------------------
 * OC_CONF3 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OC_CONF3_OFFSET                     (0x034u)
#define BQ7973X_OC_CONF3_POR_VAL                    (0x00u)

#define BQ7973X_OC_CONF3_OCD1_THRH_POS              (0u)
#define BQ7973X_OC_CONF3_OCD1_THRH_MSK              (0x7u)

/* --------------------------------------------------------------------------
 * OC_CONF4 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OC_CONF4_OFFSET                     (0x035u)
#define BQ7973X_OC_CONF4_POR_VAL                    (0x00u)

#define BQ7973X_OC_CONF4_OCD1_THRL_POS              (0u)
#define BQ7973X_OC_CONF4_OCD1_THRL_MSK              (0xffu)

/* --------------------------------------------------------------------------
 * OC_CONF5 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OC_CONF5_OFFSET                     (0x036u)
#define BQ7973X_OC_CONF5_POR_VAL                    (0x00u)

#define BQ7973X_OC_CONF5_OC2_INV_POS                (7u)
#define BQ7973X_OC_CONF5_OC2_INV_MSK                (0x80u)

#define BQ7973X_OC_CONF5_OC2_DEG_POS                (3u)
#define BQ7973X_OC_CONF5_OC2_DEG_MSK                (0x78u)

#define BQ7973X_OC_CONF5_OCC2_THRH_POS              (0u)
#define BQ7973X_OC_CONF5_OCC2_THRH_MSK              (0x7u)

/* --------------------------------------------------------------------------
 * OC_CONF6 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OC_CONF6_OFFSET                     (0x037u)
#define BQ7973X_OC_CONF6_POR_VAL                    (0x00u)

#define BQ7973X_OC_CONF6_OCC2_THRL_POS              (0u)
#define BQ7973X_OC_CONF6_OCC2_THRL_MSK              (0xffu)

/* --------------------------------------------------------------------------
 * OC_CONF7 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OC_CONF7_OFFSET                     (0x038u)
#define BQ7973X_OC_CONF7_POR_VAL                    (0x00u)

#define BQ7973X_OC_CONF7_OCD2_THRH_POS              (0u)
#define BQ7973X_OC_CONF7_OCD2_THRH_MSK              (0x7u)

/* --------------------------------------------------------------------------
 * OC_CONF8 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OC_CONF8_OFFSET                     (0x039u)
#define BQ7973X_OC_CONF8_POR_VAL                    (0x00u)

#define BQ7973X_OC_CONF8_OCD2_THRL_POS              (0u)
#define BQ7973X_OC_CONF8_OCD2_THRL_MSK              (0xffu)

/* --------------------------------------------------------------------------
 * FAULT_MSK3 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_MSK3_OFFSET                   (0x03Au)
#define BQ7973X_FAULT_MSK3_POR_VAL                  (0x00u)

#define BQ7973X_FAULT_MSK3_MSK_OC1_POS              (4u)
#define BQ7973X_FAULT_MSK3_MSK_OC1_MSK              (0x10u)

#define BQ7973X_FAULT_MSK3_MSK_OC2_POS              (3u)
#define BQ7973X_FAULT_MSK3_MSK_OC2_MSK              (0x8u)

#define BQ7973X_FAULT_MSK3_MSK_COMM_SPI_POS         (2u)
#define BQ7973X_FAULT_MSK3_MSK_COMM_SPI_MSK         (0x4u)

#define BQ7973X_FAULT_MSK3_MSK_SYS_GPIO_POS         (1u)
#define BQ7973X_FAULT_MSK3_MSK_SYS_GPIO_MSK         (0x2u)

#define BQ7973X_FAULT_MSK3_MSK_CC_POS               (0u)
#define BQ7973X_FAULT_MSK3_MSK_CC_MSK               (0x1u)

/* --------------------------------------------------------------------------
 * GPIO_CONF7 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO_CONF7_OFFSET                   (0x03Bu)
#define BQ7973X_GPIO_CONF7_POR_VAL                  (0x00u)

#define BQ7973X_GPIO_CONF7_GPIO13_POS               (3u)
#define BQ7973X_GPIO_CONF7_GPIO13_MSK               (0x38u)

#define BQ7973X_GPIO_CONF7_GPIO12_POS               (0u)
#define BQ7973X_GPIO_CONF7_GPIO12_MSK               (0x7u)

/* --------------------------------------------------------------------------
 * GPIO_CONF8 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO_CONF8_OFFSET                   (0x03Cu)
#define BQ7973X_GPIO_CONF8_POR_VAL                  (0x00u)

#define BQ7973X_GPIO_CONF8_GPIO15_POS               (3u)
#define BQ7973X_GPIO_CONF8_GPIO15_MSK               (0x38u)

#define BQ7973X_GPIO_CONF8_GPIO14_POS               (0u)
#define BQ7973X_GPIO_CONF8_GPIO14_MSK               (0x7u)

/* --------------------------------------------------------------------------
 * MPSI_CONF (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_MPSI_CONF_OFFSET                    (0x03Du)
#define BQ7973X_MPSI_CONF_POR_VAL                   (0x00u)

#define BQ7973X_MPSI_CONF_MSPI_EN_POS               (7u)
#define BQ7973X_MPSI_CONF_MSPI_EN_MSK               (0x80u)

#define BQ7973X_MPSI_CONF_GPIO9_POS                 (6u)
#define BQ7973X_MPSI_CONF_GPIO9_MSK                 (0x40u)

#define BQ7973X_MPSI_CONF_GPIO8_POS                 (5u)
#define BQ7973X_MPSI_CONF_GPIO8_MSK                 (0x20u)

#define BQ7973X_MPSI_CONF_GPIO7_POS                 (4u)
#define BQ7973X_MPSI_CONF_GPIO7_MSK                 (0x10u)

#define BQ7973X_MPSI_CONF_GPIO6_POS                 (3u)
#define BQ7973X_MPSI_CONF_GPIO6_MSK                 (0x8u)

#define BQ7973X_MPSI_CONF_GPIO5_POS                 (2u)
#define BQ7973X_MPSI_CONF_GPIO5_MSK                 (0x4u)

#define BQ7973X_MPSI_CONF_GPIO4_POS                 (1u)
#define BQ7973X_MPSI_CONF_GPIO4_MSK                 (0x2u)

#define BQ7973X_MPSI_CONF_GPIO3_POS                 (0u)
#define BQ7973X_MPSI_CONF_GPIO3_MSK                 (0x1u)

/* --------------------------------------------------------------------------
 * DEV_CONF2 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEV_CONF2_OFFSET                    (0x03Eu)
#define BQ7973X_DEV_CONF2_POR_VAL                   (0x00u)

#define BQ7973X_DEV_CONF2_PWM_FREQ_POS              (4u)
#define BQ7973X_DEV_CONF2_PWM_FREQ_MSK              (0x10u)

#define BQ7973X_DEV_CONF2_VF2_POS                   (2u)
#define BQ7973X_DEV_CONF2_VF2_MSK                   (0xcu)

#define BQ7973X_DEV_CONF2_VF1_POS                   (0u)
#define BQ7973X_DEV_CONF2_VF1_MSK                   (0x3u)

/* --------------------------------------------------------------------------
 * OTP_SPARE1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_SPARE1_OFFSET                   (0x03Fu)
#define BQ7973X_OTP_SPARE1_POR_VAL                  (0x00u)

/* --------------------------------------------------------------------------
 * OTP_PROG_UNLOCK1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_PROG_UNLOCK1_OFFSET                     (0x300u)
#define BQ7973X_OTP_PROG_UNLOCK1_POR_VAL                    (0x00u)

#define BQ7973X_OTP_PROG_UNLOCK1_CODE_POS                   (0u)
#define BQ7973X_OTP_PROG_UNLOCK1_CODE_MSK                   (0xffu)

/* --------------------------------------------------------------------------
 * OC_UNLOCK1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OC_UNLOCK1_OFFSET                   (0x301u)
#define BQ7973X_OC_UNLOCK1_POR_VAL                  (0x01u)

#define BQ7973X_OC_UNLOCK1_CODE_POS                     (0u)
#define BQ7973X_OC_UNLOCK1_CODE_MSK                     (0xffu)

/* --------------------------------------------------------------------------
 * DIR0_ADDR (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIR0_ADDR_OFFSET                    (0x306u)
#define BQ7973X_DIR0_ADDR_POR_VAL                   (0x00u)

#define BQ7973X_DIR0_ADDR_ADDRESS_POS                   (0u)
#define BQ7973X_DIR0_ADDR_ADDRESS_MSK                   (0x3fu)

/* --------------------------------------------------------------------------
 * DIR1_ADDR (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIR1_ADDR_OFFSET                    (0x307u)
#define BQ7973X_DIR1_ADDR_POR_VAL                   (0x00u)

#define BQ7973X_DIR1_ADDR_ADDRESS_POS                   (0u)
#define BQ7973X_DIR1_ADDR_ADDRESS_MSK                   (0x3fu)

/* --------------------------------------------------------------------------
 * COMM_CTRL (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_COMM_CTRL_OFFSET                    (0x308u)
#define BQ7973X_COMM_CTRL_POR_VAL                   (0x00u)

#define BQ7973X_COMM_CTRL_TOP_STACK_POS                     (0u)
#define BQ7973X_COMM_CTRL_TOP_STACK_MSK                     (0x1u)

/* --------------------------------------------------------------------------
 * CONTROL1 (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CONTROL1_OFFSET                     (0x309u)
#define BQ7973X_CONTROL1_POR_VAL                    (0x00u)

#define BQ7973X_CONTROL1_DIR_SEL_POS                    (7u)
#define BQ7973X_CONTROL1_DIR_SEL_MSK                    (0x80u)

#define BQ7973X_CONTROL1_SEND_SD_HW_RST_POS                     (6u)
#define BQ7973X_CONTROL1_SEND_SD_HW_RST_MSK                     (0x40u)

#define BQ7973X_CONTROL1_SEND_WAKE_POS                  (5u)
#define BQ7973X_CONTROL1_SEND_WAKE_MSK                  (0x20u)

#define BQ7973X_CONTROL1_SEND_SLPTOACT_POS                  (4u)
#define BQ7973X_CONTROL1_SEND_SLPTOACT_MSK                  (0x10u)

#define BQ7973X_CONTROL1_GOTO_SHUTDOWN_POS                  (3u)
#define BQ7973X_CONTROL1_GOTO_SHUTDOWN_MSK                  (0x8u)

#define BQ7973X_CONTROL1_GOTO_SLEEP_POS                     (2u)
#define BQ7973X_CONTROL1_GOTO_SLEEP_MSK                     (0x4u)

#define BQ7973X_CONTROL1_SOFT_RESET_POS                     (1u)
#define BQ7973X_CONTROL1_SOFT_RESET_MSK                     (0x2u)

#define BQ7973X_CONTROL1_ADDR_WR_POS                    (0u)
#define BQ7973X_CONTROL1_ADDR_WR_MSK                    (0x1u)

/* --------------------------------------------------------------------------
 * CONTROL2 (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CONTROL2_OFFSET                     (0x30Au)
#define BQ7973X_CONTROL2_POR_VAL                    (0x01u)

#define BQ7973X_CONTROL2_PROG_GO_POS                    (3u)
#define BQ7973X_CONTROL2_PROG_GO_MSK                    (0x8u)

#define BQ7973X_CONTROL2_I2C_EN_POS                     (2u)
#define BQ7973X_CONTROL2_I2C_EN_MSK                     (0x4u)

#define BQ7973X_CONTROL2_TBD_RSVD_POS                   (1u)
#define BQ7973X_CONTROL2_TBD_RSVD_MSK                   (0x2u)

#define BQ7973X_CONTROL2_TSREF_EN_POS                   (0u)
#define BQ7973X_CONTROL2_TSREF_EN_MSK                   (0x1u)

/* --------------------------------------------------------------------------
 * ADC_CTRL1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_ADC_CTRL1_OFFSET                    (0x310u)
#define BQ7973X_ADC_CTRL1_POR_VAL                   (0x00u)

#define BQ7973X_ADC_CTRL1_V_I_SYNC_POS                  (7u)
#define BQ7973X_ADC_CTRL1_V_I_SYNC_MSK                  (0x80u)

#define BQ7973X_ADC_CTRL1_LPF_CS_POS                    (4u)
#define BQ7973X_ADC_CTRL1_LPF_CS_MSK                    (0x70u)

#define BQ7973X_ADC_CTRL1_LPF_CS_EN_POS                     (3u)
#define BQ7973X_ADC_CTRL1_LPF_CS_EN_MSK                     (0x8u)

#define BQ7973X_ADC_CTRL1_CS_DR_POS                     (1u)
#define BQ7973X_ADC_CTRL1_CS_DR_MSK                     (0x6u)

#define BQ7973X_ADC_CTRL1_VF_GP_DR_POS                  (0u)
#define BQ7973X_ADC_CTRL1_VF_GP_DR_MSK                  (0x1u)

/* --------------------------------------------------------------------------
 * ADC_CTRL2 (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7973X_ADC_CTRL2_OFFSET                    (0x311u)
#define BQ7973X_ADC_CTRL2_POR_VAL                   (0x00u)

#define BQ7973X_ADC_CTRL2_FREEZE_EN_POS                     (7u)
#define BQ7973X_ADC_CTRL2_FREEZE_EN_MSK                     (0x80u)

#define BQ7973X_ADC_CTRL2_LPF_VF_POS                    (4u)
#define BQ7973X_ADC_CTRL2_LPF_VF_MSK                    (0x70u)

#define BQ7973X_ADC_CTRL2_LPF_VF_EN_POS                     (3u)
#define BQ7973X_ADC_CTRL2_LPF_VF_EN_MSK                     (0x8u)

#define BQ7973X_ADC_CTRL2_ADC_MODE_POS                  (1u)
#define BQ7973X_ADC_CTRL2_ADC_MODE_MSK                  (0x6u)

#define BQ7973X_ADC_CTRL2_ADC_GO_POS                    (0u)
#define BQ7973X_ADC_CTRL2_ADC_GO_MSK                    (0x1u)

/* --------------------------------------------------------------------------
 * ADC_CTRL3 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_ADC_CTRL3_OFFSET                    (0x312u)
#define BQ7973X_ADC_CTRL3_POR_VAL                   (0x00u)

#define BQ7973X_ADC_CTRL3_FLIP_RESET_POS                    (6u)
#define BQ7973X_ADC_CTRL3_FLIP_RESET_MSK                    (0x40u)

#define BQ7973X_ADC_CTRL3_DIAG_VF_GPIO_POS                  (5u)
#define BQ7973X_ADC_CTRL3_DIAG_VF_GPIO_MSK                  (0x20u)

#define BQ7973X_ADC_CTRL3_DIAG_ANA_SEL_POS                  (0u)
#define BQ7973X_ADC_CTRL3_DIAG_ANA_SEL_MSK                  (0x1fu)

/* --------------------------------------------------------------------------
 * ADC_CTRL4 (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7973X_ADC_CTRL4_OFFSET                    (0x313u)
#define BQ7973X_ADC_CTRL4_POR_VAL                   (0x00u)

#define BQ7973X_ADC_CTRL4_DIAG_D1D2_SEL_POS                     (1u)
#define BQ7973X_ADC_CTRL4_DIAG_D1D2_SEL_MSK                     (0xeu)

#define BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_POS                  (0u)
#define BQ7973X_ADC_CTRL4_DIAG_MEAS_GO_MSK                  (0x1u)

/* --------------------------------------------------------------------------
 * DIAG_ADC_CTRL1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIAG_ADC_CTRL1_OFFSET                   (0x315u)
#define BQ7973X_DIAG_ADC_CTRL1_POR_VAL                  (0x00u)

#define BQ7973X_DIAG_ADC_CTRL1_VF_GP_THR_POS                    (0u)
#define BQ7973X_DIAG_ADC_CTRL1_VF_GP_THR_MSK                    (0x7u)

/* --------------------------------------------------------------------------
 * DIAG_ADC_CTRL2 (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIAG_ADC_CTRL2_OFFSET                   (0x316u)
#define BQ7973X_DIAG_ADC_CTRL2_POR_VAL                  (0x00u)

#define BQ7973X_DIAG_ADC_CTRL2_ACOMP_MPFLT_INJ_POS                  (6u)
#define BQ7973X_DIAG_ADC_CTRL2_ACOMP_MPFLT_INJ_MSK                  (0x40u)

#define BQ7973X_DIAG_ADC_CTRL2_DCOMP_MPFLT_INJ_POS                  (5u)
#define BQ7973X_DIAG_ADC_CTRL2_DCOMP_MPFLT_INJ_MSK                  (0x20u)

#define BQ7973X_DIAG_ADC_CTRL2_DIAG_DIG_EN_POS                  (4u)
#define BQ7973X_DIAG_ADC_CTRL2_DIAG_DIG_EN_MSK                  (0x10u)

#define BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_MODE_POS                    (1u)
#define BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_MODE_MSK                    (0x6u)

#define BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_GO_POS                  (0u)
#define BQ7973X_DIAG_ADC_CTRL2_DIAG_ANA_GO_MSK                  (0x1u)

/* --------------------------------------------------------------------------
 * DIAG_MISC_CTRL1 (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIAG_MISC_CTRL1_OFFSET                  (0x317u)
#define BQ7973X_DIAG_MISC_CTRL1_POR_VAL                     (0x00u)

#define BQ7973X_DIAG_MISC_CTRL1_SPI_LOOPBACK_POS                    (6u)
#define BQ7973X_DIAG_MISC_CTRL1_SPI_LOOPBACK_MSK                    (0x40u)

#define BQ7973X_DIAG_MISC_CTRL1_FLIP_TR_CRC_POS                     (5u)
#define BQ7973X_DIAG_MISC_CTRL1_FLIP_TR_CRC_MSK                     (0x20u)

#define BQ7973X_DIAG_MISC_CTRL1_FLIP_FACT_CRC_POS                   (4u)
#define BQ7973X_DIAG_MISC_CTRL1_FLIP_FACT_CRC_MSK                   (0x10u)

#define BQ7973X_DIAG_MISC_CTRL1_MARGIN_MODE_POS                     (1u)
#define BQ7973X_DIAG_MISC_CTRL1_MARGIN_MODE_MSK                     (0xeu)

#define BQ7973X_DIAG_MISC_CTRL1_MARGIN_GO_POS                   (0u)
#define BQ7973X_DIAG_MISC_CTRL1_MARGIN_GO_MSK                   (0x1u)

/* --------------------------------------------------------------------------
 * DIAG_MISC_CTRL2 (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIAG_MISC_CTRL2_OFFSET                  (0x318u)
#define BQ7973X_DIAG_MISC_CTRL2_POR_VAL                     (0x00u)

#define BQ7973X_DIAG_MISC_CTRL2_CSOW_CURRENT_EN_POS                     (4u)
#define BQ7973X_DIAG_MISC_CTRL2_CSOW_CURRENT_EN_MSK                     (0x10u)

#define BQ7973X_DIAG_MISC_CTRL2_CBFETOW_MPFLT_INJ_POS                   (3u)
#define BQ7973X_DIAG_MISC_CTRL2_CBFETOW_MPFLT_INJ_MSK                   (0x8u)

#define BQ7973X_DIAG_MISC_CTRL2_DIAG_CBFETOW_GO_POS                     (2u)
#define BQ7973X_DIAG_MISC_CTRL2_DIAG_CBFETOW_GO_MSK                     (0x4u)

#define BQ7973X_DIAG_MISC_CTRL2_PWRBIST_NO_RST_POS                  (1u)
#define BQ7973X_DIAG_MISC_CTRL2_PWRBIST_NO_RST_MSK                  (0x2u)

#define BQ7973X_DIAG_MISC_CTRL2_PWRBIST_GO_POS                  (0u)
#define BQ7973X_DIAG_MISC_CTRL2_PWRBIST_GO_MSK                  (0x1u)

/* --------------------------------------------------------------------------
 * DIAG_OC_CTRL1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIAG_OC_CTRL1_OFFSET                    (0x319u)
#define BQ7973X_DIAG_OC_CTRL1_POR_VAL                   (0x00u)

#define BQ7973X_DIAG_OC_CTRL1_DIAG_OC_LVLH_POS                  (0u)
#define BQ7973X_DIAG_OC_CTRL1_DIAG_OC_LVLH_MSK                  (0x7fu)

/* --------------------------------------------------------------------------
 * DIAG_OC_CTRL2 (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIAG_OC_CTRL2_OFFSET                    (0x31Au)
#define BQ7973X_DIAG_OC_CTRL2_POR_VAL                   (0x00u)

#define BQ7973X_DIAG_OC_CTRL2_DIAG_OC_LVLL_POS                  (3u)
#define BQ7973X_DIAG_OC_CTRL2_DIAG_OC_LVLL_MSK                  (0xf8u)

#define BQ7973X_DIAG_OC_CTRL2_DIAG_OC_SEL_POS                   (2u)
#define BQ7973X_DIAG_OC_CTRL2_DIAG_OC_SEL_MSK                   (0x4u)

#define BQ7973X_DIAG_OC_CTRL2_DIAG_OC_MODE_POS                  (1u)
#define BQ7973X_DIAG_OC_CTRL2_DIAG_OC_MODE_MSK                  (0x2u)

#define BQ7973X_DIAG_OC_CTRL2_DIAG_OC_GO_POS                    (0u)
#define BQ7973X_DIAG_OC_CTRL2_DIAG_OC_GO_MSK                    (0x1u)

/* --------------------------------------------------------------------------
 * FAULT_RST1 (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_RST1_OFFSET                   (0x340u)
#define BQ7973X_FAULT_RST1_POR_VAL                  (0x00u)

#define BQ7973X_FAULT_RST1_RST_DIAG_OC_POS                  (7u)
#define BQ7973X_FAULT_RST1_RST_DIAG_OC_MSK                  (0x80u)

#define BQ7973X_FAULT_RST1_RST_SYS_POS                  (1u)
#define BQ7973X_FAULT_RST1_RST_SYS_MSK                  (0x2u)

#define BQ7973X_FAULT_RST1_RST_PWR_POS                  (0u)
#define BQ7973X_FAULT_RST1_RST_PWR_MSK                  (0x1u)

/* --------------------------------------------------------------------------
 * FAULT_RST2 (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_RST2_OFFSET                   (0x341u)
#define BQ7973X_FAULT_RST2_POR_VAL                  (0x00u)

#define BQ7973X_FAULT_RST2_RST_ADC_POS                  (7u)
#define BQ7973X_FAULT_RST2_RST_ADC_MSK                  (0x80u)

#define BQ7973X_FAULT_RST2_RST_OTP_CRC_POS                  (6u)
#define BQ7973X_FAULT_RST2_RST_OTP_CRC_MSK                  (0x40u)

#define BQ7973X_FAULT_RST2_RST_OTP_DATA_POS                     (5u)
#define BQ7973X_FAULT_RST2_RST_OTP_DATA_MSK                     (0x20u)

#define BQ7973X_FAULT_RST2_RST_COMM_FCOMM_POS                   (4u)
#define BQ7973X_FAULT_RST2_RST_COMM_FCOMM_MSK                   (0x10u)

#define BQ7973X_FAULT_RST2_RST_COMM_FTONE_POS                   (3u)
#define BQ7973X_FAULT_RST2_RST_COMM_FTONE_MSK                   (0x8u)

#define BQ7973X_FAULT_RST2_RST_COMM_HB_POS                  (2u)
#define BQ7973X_FAULT_RST2_RST_COMM_HB_MSK                  (0x4u)

#define BQ7973X_FAULT_RST2_RST_COMM_DSY_POS                     (1u)
#define BQ7973X_FAULT_RST2_RST_COMM_DSY_MSK                     (0x2u)

#define BQ7973X_FAULT_RST2_RST_COMM_UART_POS                    (0u)
#define BQ7973X_FAULT_RST2_RST_COMM_UART_MSK                    (0x1u)

/* --------------------------------------------------------------------------
 * FAULT_RST3 (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_RST3_OFFSET                   (0x342u)
#define BQ7973X_FAULT_RST3_POR_VAL                  (0x00u)

#define BQ7973X_FAULT_RST3_RST_OC1_POS                  (4u)
#define BQ7973X_FAULT_RST3_RST_OC1_MSK                  (0x10u)

#define BQ7973X_FAULT_RST3_RST_OC2_POS                  (3u)
#define BQ7973X_FAULT_RST3_RST_OC2_MSK                  (0x8u)

#define BQ7973X_FAULT_RST3_RST_COMM_SPI_POS                     (2u)
#define BQ7973X_FAULT_RST3_RST_COMM_SPI_MSK                     (0x4u)

#define BQ7973X_FAULT_RST3_RST_SYS_GPIO_POS                     (1u)
#define BQ7973X_FAULT_RST3_RST_SYS_GPIO_MSK                     (0x2u)

#define BQ7973X_FAULT_RST3_RST_CC_POS                   (0u)
#define BQ7973X_FAULT_RST3_RST_CC_MSK                   (0x1u)

/* --------------------------------------------------------------------------
 * OTP_ECC_TEST (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_ECC_TEST_OFFSET                     (0x350u)
#define BQ7973X_OTP_ECC_TEST_POR_VAL                    (0x00u)

#define BQ7973X_OTP_ECC_TEST_DED_SEC_POS                    (2u)
#define BQ7973X_OTP_ECC_TEST_DED_SEC_MSK                    (0x4u)

#define BQ7973X_OTP_ECC_TEST_ENC_DEC_POS                    (1u)
#define BQ7973X_OTP_ECC_TEST_ENC_DEC_MSK                    (0x2u)

#define BQ7973X_OTP_ECC_TEST_ENABLE_POS                     (0u)
#define BQ7973X_OTP_ECC_TEST_ENABLE_MSK                     (0x1u)

/* --------------------------------------------------------------------------
 * OC_CTRL1 (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OC_CTRL1_OFFSET                     (0x360u)
#define BQ7973X_OC_CTRL1_POR_VAL                    (0x00u)

#define BQ7973X_OC_CTRL1_OC1_OUT_DIS_POS                    (6u)
#define BQ7973X_OC_CTRL1_OC1_OUT_DIS_MSK                    (0x40u)

#define BQ7973X_OC_CTRL1_OC2_OUT_DIS_POS                    (5u)
#define BQ7973X_OC_CTRL1_OC2_OUT_DIS_MSK                    (0x20u)

#define BQ7973X_OC_CTRL1_OC_SIGTYPE_POS                     (3u)
#define BQ7973X_OC_CTRL1_OC_SIGTYPE_MSK                     (0x8u)

#define BQ7973X_OC_CTRL1_OC_MODE_POS                    (1u)
#define BQ7973X_OC_CTRL1_OC_MODE_MSK                    (0x2u)

#define BQ7973X_OC_CTRL1_OC_GO_POS                  (0u)
#define BQ7973X_OC_CTRL1_OC_GO_MSK                  (0x1u)

/* --------------------------------------------------------------------------
 * OC_CTRL2 (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OC_CTRL2_OFFSET                     (0x361u)
#define BQ7973X_OC_CTRL2_POR_VAL                    (0x00u)

#define BQ7973X_OC_CTRL2_OC1_FLT_GO_POS                     (1u)
#define BQ7973X_OC_CTRL2_OC1_FLT_GO_MSK                     (0x2u)

#define BQ7973X_OC_CTRL2_OC2_FLT_GO_POS                     (0u)
#define BQ7973X_OC_CTRL2_OC2_FLT_GO_MSK                     (0x1u)

/* --------------------------------------------------------------------------
 * I2C_WR_DATA (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_I2C_WR_DATA_OFFSET                  (0x370u)
#define BQ7973X_I2C_WR_DATA_POR_VAL                     (0x00u)

#define BQ7973X_I2C_WR_DATA_DATA_POS                    (0u)
#define BQ7973X_I2C_WR_DATA_DATA_MSK                    (0xffu)

/* --------------------------------------------------------------------------
 * I2C_CTRL (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7973X_I2C_CTRL_OFFSET                     (0x371u)
#define BQ7973X_I2C_CTRL_POR_VAL                    (0x00u)

#define BQ7973X_I2C_CTRL_SEND_POS                   (5u)
#define BQ7973X_I2C_CTRL_SEND_MSK                   (0x20u)

#define BQ7973X_I2C_CTRL_RECEIVE_POS                    (4u)
#define BQ7973X_I2C_CTRL_RECEIVE_MSK                    (0x10u)

#define BQ7973X_I2C_CTRL_START_POS                  (3u)
#define BQ7973X_I2C_CTRL_START_MSK                  (0x8u)

#define BQ7973X_I2C_CTRL_STOP_POS                   (2u)
#define BQ7973X_I2C_CTRL_STOP_MSK                   (0x4u)

#define BQ7973X_I2C_CTRL_NACK_POS                   (1u)
#define BQ7973X_I2C_CTRL_NACK_MSK                   (0x2u)

#define BQ7973X_I2C_CTRL_I2C_GO_POS                     (0u)
#define BQ7973X_I2C_CTRL_I2C_GO_MSK                     (0x1u)

/* --------------------------------------------------------------------------
 * MPSI_CTRL (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_MPSI_CTRL_OFFSET                    (0x390u)
#define BQ7973X_MPSI_CTRL_POR_VAL                   (0x00u)

#define BQ7973X_MPSI_CTRL_CPOL_POS                  (6u)
#define BQ7973X_MPSI_CTRL_CPOL_MSK                  (0x40u)

#define BQ7973X_MPSI_CTRL_CPHA_POS                  (5u)
#define BQ7973X_MPSI_CTRL_CPHA_MSK                  (0x20u)

#define BQ7973X_MPSI_CTRL_NUMBIT_POS                    (0u)
#define BQ7973X_MPSI_CTRL_NUMBIT_MSK                    (0x1fu)

/* --------------------------------------------------------------------------
 * SPI_TX4 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_SPI_TX4_OFFSET                  (0x391u)
#define BQ7973X_SPI_TX4_POR_VAL                     (0x00u)

#define BQ7973X_SPI_TX4_DATA_POS                    (0u)
#define BQ7973X_SPI_TX4_DATA_MSK                    (0xffu)

/* --------------------------------------------------------------------------
 * SPI_TX3 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_SPI_TX3_OFFSET                  (0x392u)
#define BQ7973X_SPI_TX3_POR_VAL                     (0x00u)

#define BQ7973X_SPI_TX3_DATA_POS                    (0u)
#define BQ7973X_SPI_TX3_DATA_MSK                    (0xffu)

/* --------------------------------------------------------------------------
 * SPI_TX2 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_SPI_TX2_OFFSET                  (0x393u)
#define BQ7973X_SPI_TX2_POR_VAL                     (0x00u)

#define BQ7973X_SPI_TX2_DATA_POS                    (0u)
#define BQ7973X_SPI_TX2_DATA_MSK                    (0xffu)

/* --------------------------------------------------------------------------
 * SPI_TX1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_SPI_TX1_OFFSET                  (0x394u)
#define BQ7973X_SPI_TX1_POR_VAL                     (0x00u)

#define BQ7973X_SPI_TX1_DATA_POS                    (0u)
#define BQ7973X_SPI_TX1_DATA_MSK                    (0xffu)

/* --------------------------------------------------------------------------
 * MSPI_EXE (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7973X_MSPI_EXE_OFFSET                     (0x395u)
#define BQ7973X_MSPI_EXE_POR_VAL                    (0x00u)

#define BQ7973X_MSPI_EXE_SS_ADDR_POS                    (3u)
#define BQ7973X_MSPI_EXE_SS_ADDR_MSK                    (0x38u)

#define BQ7973X_MSPI_EXE_SS_CTRL_POS                    (1u)
#define BQ7973X_MSPI_EXE_SS_CTRL_MSK                    (0x6u)

#define BQ7973X_MSPI_EXE_MSPI_GO_POS                    (0u)
#define BQ7973X_MSPI_EXE_MSPI_GO_MSK                    (0x1u)

/* --------------------------------------------------------------------------
 * CC_CTRL (R/W-AC):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CC_CTRL_OFFSET                  (0x3A0u)
#define BQ7973X_CC_CTRL_POR_VAL                     (0x00u)

#define BQ7973X_CC_CTRL_CC_CLR_GO_POS                   (2u)
#define BQ7973X_CC_CTRL_CC_CLR_GO_MSK                   (0x4u)

#define BQ7973X_CC_CTRL_CC_MODE_POS                     (1u)
#define BQ7973X_CC_CTRL_CC_MODE_MSK                     (0x2u)

#define BQ7973X_CC_CTRL_CC_GO_POS                   (0u)
#define BQ7973X_CC_CTRL_CC_GO_MSK                   (0x1u)

/* --------------------------------------------------------------------------
 * SW_CTRL (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_SW_CTRL_OFFSET                      (0x3B0u)
#define BQ7973X_SW_CTRL_POR_VAL                     (0x00u)

#define BQ7973X_SW_CTRL_SW4_POS                     (6u)
#define BQ7973X_SW_CTRL_SW4_MSK                     (0xc0u)

#define BQ7973X_SW_CTRL_SW3_POS                     (4u)
#define BQ7973X_SW_CTRL_SW3_MSK                     (0x30u)

#define BQ7973X_SW_CTRL_SW2_POS                     (2u)
#define BQ7973X_SW_CTRL_SW2_MSK                     (0xcu)

#define BQ7973X_SW_CTRL_SW1_POS                     (0u)
#define BQ7973X_SW_CTRL_SW1_MSK                     (0x3u)

/* --------------------------------------------------------------------------
 * GPIO_CTRL1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO_CTRL1_OFFSET                   (0x3B3u)
#define BQ7973X_GPIO_CTRL1_POR_VAL                  (0x00u)

#define BQ7973X_GPIO_CTRL1_GPIO15_OUT_POS           (6u)
#define BQ7973X_GPIO_CTRL1_GPIO15_OUT_MSK           (0x40u)

#define BQ7973X_GPIO_CTRL1_GPIO14_OUT_POS           (5u)
#define BQ7973X_GPIO_CTRL1_GPIO14_OUT_MSK           (0x20u)

#define BQ7973X_GPIO_CTRL1_GPIO13_OUT_POS           (4u)
#define BQ7973X_GPIO_CTRL1_GPIO13_OUT_MSK           (0x10u)

#define BQ7973X_GPIO_CTRL1_GPIO12_OUT_POS           (3u)
#define BQ7973X_GPIO_CTRL1_GPIO12_OUT_MSK           (0x8u)

#define BQ7973X_GPIO_CTRL1_GPIO11_OUT_POS           (2u)
#define BQ7973X_GPIO_CTRL1_GPIO11_OUT_MSK           (0x4u)

#define BQ7973X_GPIO_CTRL1_GPIO10_OUT_POS           (1u)
#define BQ7973X_GPIO_CTRL1_GPIO10_OUT_MSK           (0x2u)

#define BQ7973X_GPIO_CTRL1_GPIO9_OUT_POS            (0u)
#define BQ7973X_GPIO_CTRL1_GPIO9_OUT_MSK            (0x1u)

/* --------------------------------------------------------------------------
 * GPIO_CTRL2 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO_CTRL2_OFFSET                   (0x3B4u)
#define BQ7973X_GPIO_CTRL2_POR_VAL                  (0x00u)

#define BQ7973X_GPIO_CTRL2_GPIO8_OUT_POS            (7u)
#define BQ7973X_GPIO_CTRL2_GPIO8_OUT_MSK            (0x80u)

#define BQ7973X_GPIO_CTRL2_GPIO7_OUT_POS            (6u)
#define BQ7973X_GPIO_CTRL2_GPIO7_OUT_MSK            (0x40u)

#define BQ7973X_GPIO_CTRL2_GPIO6_OUT_POS            (5u)
#define BQ7973X_GPIO_CTRL2_GPIO6_OUT_MSK            (0x20u)

#define BQ7973X_GPIO_CTRL2_GPIO5_OUT_POS            (4u)
#define BQ7973X_GPIO_CTRL2_GPIO5_OUT_MSK            (0x10u)

#define BQ7973X_GPIO_CTRL2_GPIO4_OUT_POS            (3u)
#define BQ7973X_GPIO_CTRL2_GPIO4_OUT_MSK            (0x8u)

#define BQ7973X_GPIO_CTRL2_GPIO3_OUT_POS            (2u)
#define BQ7973X_GPIO_CTRL2_GPIO3_OUT_MSK            (0x4u)

#define BQ7973X_GPIO_CTRL2_GPIO2_OUT_POS            (1u)
#define BQ7973X_GPIO_CTRL2_GPIO2_OUT_MSK            (0x2u)

#define BQ7973X_GPIO_CTRL2_GPIO1_OUT_POS            (0u)
#define BQ7973X_GPIO_CTRL2_GPIO1_OUT_MSK            (0x1u)

/* --------------------------------------------------------------------------
 * GPIO_PWM_CTRL (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO_PWM_CTRL_OFFSET                    (0x3B6u)
#define BQ7973X_GPIO_PWM_CTRL_POR_VAL                   (0x00u)

#define BQ7973X_GPIO_PWM_CTRL_GPIO15_PWM_POS                    (6u)
#define BQ7973X_GPIO_PWM_CTRL_GPIO15_PWM_MSK                    (0x40u)

#define BQ7973X_GPIO_PWM_CTRL_GPIO14_PWM_POS                    (5u)
#define BQ7973X_GPIO_PWM_CTRL_GPIO14_PWM_MSK                    (0x20u)

#define BQ7973X_GPIO_PWM_CTRL_GPIO13_PWM_POS                    (4u)
#define BQ7973X_GPIO_PWM_CTRL_GPIO13_PWM_MSK                    (0x10u)

#define BQ7973X_GPIO_PWM_CTRL_GPIO12_PWM_POS                    (3u)
#define BQ7973X_GPIO_PWM_CTRL_GPIO12_PWM_MSK                    (0x8u)

/* --------------------------------------------------------------------------
 * OC_UNLOCK2 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OC_UNLOCK2_OFFSET                   (0x4FEu)
#define BQ7973X_OC_UNLOCK2_POR_VAL                  (0x00u)

#define BQ7973X_OC_UNLOCK2_CODE_POS                     (0u)
#define BQ7973X_OC_UNLOCK2_CODE_MSK                     (0xffu)

/* --------------------------------------------------------------------------
 * OTP_PROG_UNLOCK2 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_PROG_UNLOCK2_OFFSET                     (0x4FFu)
#define BQ7973X_OTP_PROG_UNLOCK2_POR_VAL                    (0x00u)

#define BQ7973X_OTP_PROG_UNLOCK2_CODE_POS                   (0u)
#define BQ7973X_OTP_PROG_UNLOCK2_CODE_MSK                   (0xffu)

/* --------------------------------------------------------------------------
 * PARTID (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_PARTID_OFFSET                   (0x500u)
#define BQ7973X_PARTID_POR_VAL                  (0x00u)

#define BQ7973X_PARTID_REV_POS                  (0u)
#define BQ7973X_PARTID_REV_MSK                  (0xffu)

/* --------------------------------------------------------------------------
 * TAPEOUT_REV (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_TAPEOUT_REV_OFFSET                  (0x501u)
#define BQ7973X_TAPEOUT_REV_POR_VAL                     (0x00u)

#define BQ7973X_TAPEOUT_REV_ALL_LAYER_REV_POS                   (4u)
#define BQ7973X_TAPEOUT_REV_ALL_LAYER_REV_MSK                   (0xf0u)

#define BQ7973X_TAPEOUT_REV_METAL_REV_POS                   (0u)
#define BQ7973X_TAPEOUT_REV_METAL_REV_MSK                   (0xfu)

/* --------------------------------------------------------------------------
 * DIE_ID1 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIE_ID1_OFFSET                  (0x502u)
#define BQ7973X_DIE_ID1_POR_VAL                     (0x00u)

#define BQ7973X_DIE_ID1_ID_POS                  (0u)
#define BQ7973X_DIE_ID1_ID_MSK                  (0xffu)

/* --------------------------------------------------------------------------
 * DIE_ID2 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIE_ID2_OFFSET                  (0x503u)
#define BQ7973X_DIE_ID2_POR_VAL                     (0x00u)

#define BQ7973X_DIE_ID2_ID_POS                  (0u)
#define BQ7973X_DIE_ID2_ID_MSK                  (0xffu)

/* --------------------------------------------------------------------------
 * DIE_ID3 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIE_ID3_OFFSET                  (0x504u)
#define BQ7973X_DIE_ID3_POR_VAL                     (0x00u)

#define BQ7973X_DIE_ID3_ID_POS                  (0u)
#define BQ7973X_DIE_ID3_ID_MSK                  (0xffu)

/* --------------------------------------------------------------------------
 * DIE_ID4 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIE_ID4_OFFSET                  (0x505u)
#define BQ7973X_DIE_ID4_POR_VAL                     (0x00u)

#define BQ7973X_DIE_ID4_ID_POS                  (0u)
#define BQ7973X_DIE_ID4_ID_MSK                  (0xffu)

/* --------------------------------------------------------------------------
 * DIE_ID5 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIE_ID5_OFFSET                  (0x506u)
#define BQ7973X_DIE_ID5_POR_VAL                     (0x00u)

#define BQ7973X_DIE_ID5_ID_POS                  (0u)
#define BQ7973X_DIE_ID5_ID_MSK                  (0xffu)

/* --------------------------------------------------------------------------
 * DIE_ID6 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIE_ID6_OFFSET                  (0x507u)
#define BQ7973X_DIE_ID6_POR_VAL                     (0x00u)

#define BQ7973X_DIE_ID6_ID_POS                  (0u)
#define BQ7973X_DIE_ID6_ID_MSK                  (0xffu)

/* --------------------------------------------------------------------------
 * DIE_ID7 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIE_ID7_OFFSET                  (0x508u)
#define BQ7973X_DIE_ID7_POR_VAL                     (0x00u)

#define BQ7973X_DIE_ID7_ID_POS                  (0u)
#define BQ7973X_DIE_ID7_ID_MSK                  (0xffu)

/* --------------------------------------------------------------------------
 * DIE_ID8 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIE_ID8_OFFSET                  (0x509u)
#define BQ7973X_DIE_ID8_POR_VAL                     (0x00u)

#define BQ7973X_DIE_ID8_ID_POS                  (0u)
#define BQ7973X_DIE_ID8_ID_MSK                  (0xffu)

/* --------------------------------------------------------------------------
 * CUST_CRC_RSLT_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CUST_CRC_RSLT_HI_OFFSET                     (0x50Cu)
#define BQ7973X_CUST_CRC_RSLT_HI_POR_VAL                    (0x00u)

#define BQ7973X_CUST_CRC_RSLT_HI_CRC_POS                    (0u)
#define BQ7973X_CUST_CRC_RSLT_HI_CRC_MSK                    (0xffu)

/* --------------------------------------------------------------------------
 * OTP_STAT (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OTP_STAT_OFFSET                     (0x510u)
#define BQ7973X_OTP_STAT_POR_VAL                    (0x00u)

#define BQ7973X_OTP_STAT_LOADED_POS                     (7u)
#define BQ7973X_OTP_STAT_LOADED_MSK                     (0x80u)

#define BQ7973X_OTP_STAT_UV_OVOK_POS                    (6u)
#define BQ7973X_OTP_STAT_UV_OVOK_MSK                    (0x40u)

#define BQ7973X_OTP_STAT_TRY_POS                    (5u)
#define BQ7973X_OTP_STAT_TRY_MSK                    (0x20u)

#define BQ7973X_OTP_STAT_UNLOCK_POS                     (4u)
#define BQ7973X_OTP_STAT_UNLOCK_MSK                     (0x10u)

#define BQ7973X_OTP_STAT_UV_OVERR_POS                   (3u)
#define BQ7973X_OTP_STAT_UV_OVERR_MSK                   (0x8u)

#define BQ7973X_OTP_STAT_SUV_SOVERR_POS                     (2u)
#define BQ7973X_OTP_STAT_SUV_SOVERR_MSK                     (0x4u)

#define BQ7973X_OTP_STAT_PROGERR_POS                    (1u)
#define BQ7973X_OTP_STAT_PROGERR_MSK                    (0x2u)

#define BQ7973X_OTP_STAT_DONE_POS                   (0u)
#define BQ7973X_OTP_STAT_DONE_MSK                   (0x1u)

/* --------------------------------------------------------------------------
 * GPIO_STAT1 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO_STAT1_OFFSET                   (0x520u)
#define BQ7973X_GPIO_STAT1_POR_VAL                  (0x00u)

#define BQ7973X_GPIO_STAT1_GPIO15_POS                   (6u)
#define BQ7973X_GPIO_STAT1_GPIO15_MSK                   (0x40u)

#define BQ7973X_GPIO_STAT1_GPIO14_POS                   (5u)
#define BQ7973X_GPIO_STAT1_GPIO14_MSK                   (0x20u)

#define BQ7973X_GPIO_STAT1_GPIO13_POS                   (4u)
#define BQ7973X_GPIO_STAT1_GPIO13_MSK                   (0x10u)

#define BQ7973X_GPIO_STAT1_GPIO12_POS                   (3u)
#define BQ7973X_GPIO_STAT1_GPIO12_MSK                   (0x8u)

#define BQ7973X_GPIO_STAT1_GPIO11_POS                   (2u)
#define BQ7973X_GPIO_STAT1_GPIO11_MSK                   (0x4u)

#define BQ7973X_GPIO_STAT1_GPIO10_POS                   (1u)
#define BQ7973X_GPIO_STAT1_GPIO10_MSK                   (0x2u)

#define BQ7973X_GPIO_STAT1_GPIO9_POS                    (0u)
#define BQ7973X_GPIO_STAT1_GPIO9_MSK                    (0x1u)

/* --------------------------------------------------------------------------
 * GPIO_STAT2 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO_STAT2_OFFSET                   (0x521u)
#define BQ7973X_GPIO_STAT2_POR_VAL                  (0x00u)

#define BQ7973X_GPIO_STAT2_GPIO8_POS                    (7u)
#define BQ7973X_GPIO_STAT2_GPIO8_MSK                    (0x80u)

#define BQ7973X_GPIO_STAT2_GPIO7_POS                    (6u)
#define BQ7973X_GPIO_STAT2_GPIO7_MSK                    (0x40u)

#define BQ7973X_GPIO_STAT2_GPIO6_POS                    (5u)
#define BQ7973X_GPIO_STAT2_GPIO6_MSK                    (0x20u)

#define BQ7973X_GPIO_STAT2_GPIO5_POS                    (4u)
#define BQ7973X_GPIO_STAT2_GPIO5_MSK                    (0x10u)

#define BQ7973X_GPIO_STAT2_GPIO4_POS                    (3u)
#define BQ7973X_GPIO_STAT2_GPIO4_MSK                    (0x8u)

#define BQ7973X_GPIO_STAT2_GPIO3_POS                    (2u)
#define BQ7973X_GPIO_STAT2_GPIO3_MSK                    (0x4u)

#define BQ7973X_GPIO_STAT2_GPIO2_POS                    (1u)
#define BQ7973X_GPIO_STAT2_GPIO2_MSK                    (0x2u)

#define BQ7973X_GPIO_STAT2_GPIO1_POS                    (0u)
#define BQ7973X_GPIO_STAT2_GPIO1_MSK                    (0x1u)

/* --------------------------------------------------------------------------
 * DIAG_STAT1 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIAG_STAT1_OFFSET                   (0x52Du)
#define BQ7973X_DIAG_STAT1_POR_VAL                  (0x00u)

#define BQ7973X_DIAG_STAT1_FREEZE_ACTIVE_POS                    (4u)
#define BQ7973X_DIAG_STAT1_FREEZE_ACTIVE_MSK                    (0x10u)

#define BQ7973X_DIAG_STAT1_ECC_TEST_OK_POS                  (3u)
#define BQ7973X_DIAG_STAT1_ECC_TEST_OK_MSK                  (0x8u)

#define BQ7973X_DIAG_STAT1_DRDY_PWRBIST_POS                     (0u)
#define BQ7973X_DIAG_STAT1_DRDY_PWRBIST_MSK                     (0x1u)

/* --------------------------------------------------------------------------
 * DIAG_STAT2 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIAG_STAT2_OFFSET                   (0x52Eu)
#define BQ7973X_DIAG_STAT2_POR_VAL                  (0x00u)

#define BQ7973X_DIAG_STAT2_DRDY_DIG_POS                     (2u)
#define BQ7973X_DIAG_STAT2_DRDY_DIG_MSK                     (0x4u)

#define BQ7973X_DIAG_STAT2_DRDY_ANA_GPIO_POS                    (1u)
#define BQ7973X_DIAG_STAT2_DRDY_ANA_GPIO_MSK                    (0x2u)

#define BQ7973X_DIAG_STAT2_DRDY_ANA_VF_POS                  (0u)
#define BQ7973X_DIAG_STAT2_DRDY_ANA_VF_MSK                  (0x1u)

/* --------------------------------------------------------------------------
 * ADC_DATA_RDY (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_ADC_DATA_RDY_OFFSET                     (0x52Fu)
#define BQ7973X_ADC_DATA_RDY_POR_VAL                    (0x00u)

#define BQ7973X_ADC_DATA_RDY_DRDY_CSADC_POS                     (4u)
#define BQ7973X_ADC_DATA_RDY_DRDY_CSADC_MSK                     (0x10u)

#define BQ7973X_ADC_DATA_RDY_DRDY_DIAG_D1D2_POS                     (3u)
#define BQ7973X_ADC_DATA_RDY_DRDY_DIAG_D1D2_MSK                     (0x8u)

#define BQ7973X_ADC_DATA_RDY_DRDY_DIAG_POS                  (2u)
#define BQ7973X_ADC_DATA_RDY_DRDY_DIAG_MSK                  (0x4u)

#define BQ7973X_ADC_DATA_RDY_DRDY_GPADC_POS                     (1u)
#define BQ7973X_ADC_DATA_RDY_DRDY_GPADC_MSK                     (0x2u)

#define BQ7973X_ADC_DATA_RDY_DRDY_VFADC_POS                     (0u)
#define BQ7973X_ADC_DATA_RDY_DRDY_VFADC_MSK                     (0x1u)

/* --------------------------------------------------------------------------
 * DEV_STAT1 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEV_STAT1_OFFSET                    (0x530u)
#define BQ7973X_DEV_STAT1_POR_VAL                   (0x00u)

#define BQ7973X_DEV_STAT1_DIAG_MEAS_RUN_POS                     (7u)
#define BQ7973X_DEV_STAT1_DIAG_MEAS_RUN_MSK                     (0x80u)

#define BQ7973X_DEV_STAT1_FACT_CRC_DONE_POS                     (6u)
#define BQ7973X_DEV_STAT1_FACT_CRC_DONE_MSK                     (0x40u)

#define BQ7973X_DEV_STAT1_CUST_CRC_DONE_POS                     (5u)
#define BQ7973X_DEV_STAT1_CUST_CRC_DONE_MSK                     (0x20u)

#define BQ7973X_DEV_STAT1_DIAG_ANA_RUN_POS                  (2u)
#define BQ7973X_DEV_STAT1_DIAG_ANA_RUN_MSK                  (0x4u)

#define BQ7973X_DEV_STAT1_GPADC_RUN_POS                     (1u)
#define BQ7973X_DEV_STAT1_GPADC_RUN_MSK                     (0x2u)

#define BQ7973X_DEV_STAT1_VFADC_RUN_POS                     (0u)
#define BQ7973X_DEV_STAT1_VFADC_RUN_MSK                     (0x1u)

/* --------------------------------------------------------------------------
 * DEV_STAT2 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEV_STAT2_OFFSET                    (0x531u)
#define BQ7973X_DEV_STAT2_POR_VAL                   (0x00u)

#define BQ7973X_DEV_STAT2_CC_RUN_POS                    (5u)
#define BQ7973X_DEV_STAT2_CC_RUN_MSK                    (0x20u)

#define BQ7973X_DEV_STAT2_DIAG_OC_RUN_POS                   (4u)
#define BQ7973X_DEV_STAT2_DIAG_OC_RUN_MSK                   (0x10u)

#define BQ7973X_DEV_STAT2_OC_RUN_POS                    (3u)
#define BQ7973X_DEV_STAT2_OC_RUN_MSK                    (0x8u)

#define BQ7973X_DEV_STAT2_CSADC_RUN_POS                     (2u)
#define BQ7973X_DEV_STAT2_CSADC_RUN_MSK                     (0x4u)

#define BQ7973X_DEV_STAT2_DIAG_DIG_RUN_POS                  (1u)
#define BQ7973X_DEV_STAT2_DIAG_DIG_RUN_MSK                  (0x2u)

/* --------------------------------------------------------------------------
 * FAULT_SUMMARY (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_SUMMARY_OFFSET                    (0x532u)
#define BQ7973X_FAULT_SUMMARY_POR_VAL                   (0x00u)

#define BQ7973X_FAULT_SUMMARY_FAULT_OC_POS                  (7u)
#define BQ7973X_FAULT_SUMMARY_FAULT_OC_MSK                  (0x80u)

#define BQ7973X_FAULT_SUMMARY_FAULT_ADC_CC_POS                  (6u)
#define BQ7973X_FAULT_SUMMARY_FAULT_ADC_CC_MSK                  (0x40u)

#define BQ7973X_FAULT_SUMMARY_FAULT_SYS_POS                     (3u)
#define BQ7973X_FAULT_SUMMARY_FAULT_SYS_MSK                     (0x8u)

#define BQ7973X_FAULT_SUMMARY_FAULT_OTP_POS                     (2u)
#define BQ7973X_FAULT_SUMMARY_FAULT_OTP_MSK                     (0x4u)

#define BQ7973X_FAULT_SUMMARY_FAULT_COMM_POS                    (1u)
#define BQ7973X_FAULT_SUMMARY_FAULT_COMM_MSK                    (0x2u)

#define BQ7973X_FAULT_SUMMARY_FAULT_PWR_POS                     (0u)
#define BQ7973X_FAULT_SUMMARY_FAULT_PWR_MSK                     (0x1u)

/* --------------------------------------------------------------------------
 * FAULT_PWR1 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_PWR1_OFFSET                   (0x535u)
#define BQ7973X_FAULT_PWR1_POR_VAL                  (0x00u)

#define BQ7973X_FAULT_PWR1_PWRBIST_FAIL_POS                     (7u)
#define BQ7973X_FAULT_PWR1_PWRBIST_FAIL_MSK                     (0x80u)

#define BQ7973X_FAULT_PWR1_VSS_OPEN_POS                     (6u)
#define BQ7973X_FAULT_PWR1_VSS_OPEN_MSK                     (0x40u)

#define BQ7973X_FAULT_PWR1_TSREF_OSC_POS                    (5u)
#define BQ7973X_FAULT_PWR1_TSREF_OSC_MSK                    (0x20u)

#define BQ7973X_FAULT_PWR1_TSREF_UV_POS                     (4u)
#define BQ7973X_FAULT_PWR1_TSREF_UV_MSK                     (0x10u)

#define BQ7973X_FAULT_PWR1_TSREF_OV_POS                     (3u)
#define BQ7973X_FAULT_PWR1_TSREF_OV_MSK                     (0x8u)

#define BQ7973X_FAULT_PWR1_DVDD_OV_POS                  (2u)
#define BQ7973X_FAULT_PWR1_DVDD_OV_MSK                  (0x4u)

#define BQ7973X_FAULT_PWR1_AVDD_OSC_POS                     (1u)
#define BQ7973X_FAULT_PWR1_AVDD_OSC_MSK                     (0x2u)

#define BQ7973X_FAULT_PWR1_AVDD_OV_POS                  (0u)
#define BQ7973X_FAULT_PWR1_AVDD_OV_MSK                  (0x1u)

/* --------------------------------------------------------------------------
 * FAULT_PWR2 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_PWR2_OFFSET                   (0x536u)
#define BQ7973X_FAULT_PWR2_POR_VAL                  (0x00u)

#define BQ7973X_FAULT_PWR2_CP_OV_POS                    (2u)
#define BQ7973X_FAULT_PWR2_CP_OV_MSK                    (0x4u)

#define BQ7973X_FAULT_PWR2_CP_UV_POS                    (1u)
#define BQ7973X_FAULT_PWR2_CP_UV_MSK                    (0x2u)

/* --------------------------------------------------------------------------
 * FAULT_COMM1 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_COMM1_OFFSET                  (0x538u)
#define BQ7973X_FAULT_COMM1_POR_VAL                     (0x00u)

#define BQ7973X_FAULT_COMM1_FCOMM_DET_POS                   (7u)
#define BQ7973X_FAULT_COMM1_FCOMM_DET_MSK                   (0x80u)

#define BQ7973X_FAULT_COMM1_FTONE_DET_POS                   (6u)
#define BQ7973X_FAULT_COMM1_FTONE_DET_MSK                   (0x40u)

#define BQ7973X_FAULT_COMM1_HB_FAIL_POS                     (5u)
#define BQ7973X_FAULT_COMM1_HB_FAIL_MSK                     (0x20u)

#define BQ7973X_FAULT_COMM1_COML_POS                    (4u)
#define BQ7973X_FAULT_COMM1_COML_MSK                    (0x10u)

#define BQ7973X_FAULT_COMM1_COMH_POS                    (3u)
#define BQ7973X_FAULT_COMM1_COMH_MSK                    (0x8u)

#define BQ7973X_FAULT_COMM1_UART_FRAME_POS                  (2u)
#define BQ7973X_FAULT_COMM1_UART_FRAME_MSK                  (0x4u)

#define BQ7973X_FAULT_COMM1_COMMCLR_DET_POS                     (1u)
#define BQ7973X_FAULT_COMM1_COMMCLR_DET_MSK                     (0x2u)

#define BQ7973X_FAULT_COMM1_STOP_DET_POS                    (0u)
#define BQ7973X_FAULT_COMM1_STOP_DET_MSK                    (0x1u)

/* --------------------------------------------------------------------------
 * FAULT_COMM2 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_COMM2_OFFSET                  (0x539u)
#define BQ7973X_FAULT_COMM2_POR_VAL                     (0x00u)

#define BQ7973X_FAULT_COMM2_VIF_DIS_POS                     (1u)
#define BQ7973X_FAULT_COMM2_VIF_DIS_MSK                     (0x2u)

#define BQ7973X_FAULT_COMM2_SPI_POS                     (0u)
#define BQ7973X_FAULT_COMM2_SPI_MSK                     (0x1u)

/* --------------------------------------------------------------------------
 * FAULT_OTP (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_OTP_OFFSET                    (0x53Bu)
#define BQ7973X_FAULT_OTP_POR_VAL                   (0x00u)

#define BQ7973X_FAULT_OTP_DED_DET_POS                   (5u)
#define BQ7973X_FAULT_OTP_DED_DET_MSK                   (0x20u)

#define BQ7973X_FAULT_OTP_SEC_DET_POS                   (4u)
#define BQ7973X_FAULT_OTP_SEC_DET_MSK                   (0x10u)

#define BQ7973X_FAULT_OTP_CUST_CRC_POS                  (3u)
#define BQ7973X_FAULT_OTP_CUST_CRC_MSK                  (0x8u)

#define BQ7973X_FAULT_OTP_FACT_CRC_POS                  (2u)
#define BQ7973X_FAULT_OTP_FACT_CRC_MSK                  (0x4u)

#define BQ7973X_FAULT_OTP_LOADERR_POS                   (1u)
#define BQ7973X_FAULT_OTP_LOADERR_MSK                   (0x2u)

#define BQ7973X_FAULT_OTP_GBLOVERR_POS                  (0u)
#define BQ7973X_FAULT_OTP_GBLOVERR_MSK                  (0x1u)

/* --------------------------------------------------------------------------
 * FAULT_SYS1 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_SYS1_OFFSET                   (0x53Du)
#define BQ7973X_FAULT_SYS1_POR_VAL                  (0x00u)

#define BQ7973X_FAULT_SYS1_GPIO_POS                     (7u)
#define BQ7973X_FAULT_SYS1_GPIO_MSK                     (0x80u)

#define BQ7973X_FAULT_SYS1_I2C_LOW_POS                  (5u)
#define BQ7973X_FAULT_SYS1_I2C_LOW_MSK                  (0x20u)

#define BQ7973X_FAULT_SYS1_I2C_NACK_POS                     (4u)
#define BQ7973X_FAULT_SYS1_I2C_NACK_MSK                     (0x10u)

#define BQ7973X_FAULT_SYS1_LFO_POS                  (3u)
#define BQ7973X_FAULT_SYS1_LFO_MSK                  (0x8u)

#define BQ7973X_FAULT_SYS1_DRST_POS                     (2u)
#define BQ7973X_FAULT_SYS1_DRST_MSK                     (0x4u)

#define BQ7973X_FAULT_SYS1_TSHUT_POS                    (1u)
#define BQ7973X_FAULT_SYS1_TSHUT_MSK                    (0x2u)

/* --------------------------------------------------------------------------
 * FAULT_SYS2 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_SYS2_OFFSET                   (0x53Eu)
#define BQ7973X_FAULT_SYS2_POR_VAL                  (0x00u)

#define BQ7973X_FAULT_SYS2_I2C_BUSY_POS                     (2u)
#define BQ7973X_FAULT_SYS2_I2C_BUSY_MSK                     (0x4u)

#define BQ7973X_FAULT_SYS2_SPI_BUSY_POS                     (1u)
#define BQ7973X_FAULT_SYS2_SPI_BUSY_MSK                     (0x2u)

#define BQ7973X_FAULT_SYS2_MSPI_SS_POS                  (0u)
#define BQ7973X_FAULT_SYS2_MSPI_SS_MSK                  (0x1u)

/* --------------------------------------------------------------------------
 * FAULT_ADC_GPIO1 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_ADC_GPIO1_OFFSET                  (0x54Du)
#define BQ7973X_FAULT_ADC_GPIO1_POR_VAL                     (0x00u)

#define BQ7973X_FAULT_ADC_GPIO1_GPIO15_AFAIL_POS                    (6u)
#define BQ7973X_FAULT_ADC_GPIO1_GPIO15_AFAIL_MSK                    (0x40u)

#define BQ7973X_FAULT_ADC_GPIO1_GPIO14_AFAIL_POS                    (5u)
#define BQ7973X_FAULT_ADC_GPIO1_GPIO14_AFAIL_MSK                    (0x20u)

#define BQ7973X_FAULT_ADC_GPIO1_GPIO13_AFAIL_POS                    (4u)
#define BQ7973X_FAULT_ADC_GPIO1_GPIO13_AFAIL_MSK                    (0x10u)

#define BQ7973X_FAULT_ADC_GPIO1_GPIO12_AFAIL_POS                    (3u)
#define BQ7973X_FAULT_ADC_GPIO1_GPIO12_AFAIL_MSK                    (0x8u)

#define BQ7973X_FAULT_ADC_GPIO1_GPIO11_AFAIL_POS                    (2u)
#define BQ7973X_FAULT_ADC_GPIO1_GPIO11_AFAIL_MSK                    (0x4u)

#define BQ7973X_FAULT_ADC_GPIO1_GPIO10_AFAIL_POS                    (1u)
#define BQ7973X_FAULT_ADC_GPIO1_GPIO10_AFAIL_MSK                    (0x2u)

#define BQ7973X_FAULT_ADC_GPIO1_GPIO9_AFAIL_POS                     (0u)
#define BQ7973X_FAULT_ADC_GPIO1_GPIO9_AFAIL_MSK                     (0x1u)

/* --------------------------------------------------------------------------
 * FAULT_ADC_GPIO2 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_ADC_GPIO2_OFFSET                  (0x54Eu)
#define BQ7973X_FAULT_ADC_GPIO2_POR_VAL                     (0x00u)

#define BQ7973X_FAULT_ADC_GPIO2_GPIO8_AFAIL_POS                     (7u)
#define BQ7973X_FAULT_ADC_GPIO2_GPIO8_AFAIL_MSK                     (0x80u)

#define BQ7973X_FAULT_ADC_GPIO2_GPIO7_AFAIL_POS                     (6u)
#define BQ7973X_FAULT_ADC_GPIO2_GPIO7_AFAIL_MSK                     (0x40u)

#define BQ7973X_FAULT_ADC_GPIO2_GPIO6_AFAIL_POS                     (5u)
#define BQ7973X_FAULT_ADC_GPIO2_GPIO6_AFAIL_MSK                     (0x20u)

#define BQ7973X_FAULT_ADC_GPIO2_GPIO5_AFAIL_POS                     (4u)
#define BQ7973X_FAULT_ADC_GPIO2_GPIO5_AFAIL_MSK                     (0x10u)

#define BQ7973X_FAULT_ADC_GPIO2_GPIO4_AFAIL_POS                     (3u)
#define BQ7973X_FAULT_ADC_GPIO2_GPIO4_AFAIL_MSK                     (0x8u)

#define BQ7973X_FAULT_ADC_GPIO2_GPIO3_AFAIL_POS                     (2u)
#define BQ7973X_FAULT_ADC_GPIO2_GPIO3_AFAIL_MSK                     (0x4u)

#define BQ7973X_FAULT_ADC_GPIO2_GPIO2_AFAIL_POS                     (1u)
#define BQ7973X_FAULT_ADC_GPIO2_GPIO2_AFAIL_MSK                     (0x2u)

#define BQ7973X_FAULT_ADC_GPIO2_GPIO1_AFAIL_POS                     (0u)
#define BQ7973X_FAULT_ADC_GPIO2_GPIO1_AFAIL_MSK                     (0x1u)

/* --------------------------------------------------------------------------
 * FAULT_ADC_VF (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_ADC_VF_OFFSET                     (0x551u)
#define BQ7973X_FAULT_ADC_VF_POR_VAL                    (0x00u)

#define BQ7973X_FAULT_ADC_VF_VF2_AFAIL_POS                  (1u)
#define BQ7973X_FAULT_ADC_VF_VF2_AFAIL_MSK                  (0x2u)

#define BQ7973X_FAULT_ADC_VF_VF1_AFAIL_POS                  (0u)
#define BQ7973X_FAULT_ADC_VF_VF1_AFAIL_MSK                  (0x1u)

/* --------------------------------------------------------------------------
 * FAULT_ADC_DIG1 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_ADC_DIG1_OFFSET                   (0x552u)
#define BQ7973X_FAULT_ADC_DIG1_POR_VAL                  (0x00u)

#define BQ7973X_FAULT_ADC_DIG1_GP3_DFAIL_POS                    (3u)
#define BQ7973X_FAULT_ADC_DIG1_GP3_DFAIL_MSK                    (0x8u)

#define BQ7973X_FAULT_ADC_DIG1_GP1_DFAIL_POS                    (2u)
#define BQ7973X_FAULT_ADC_DIG1_GP1_DFAIL_MSK                    (0x4u)

/* --------------------------------------------------------------------------
 * FAULT_ADC_DIG2 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_ADC_DIG2_OFFSET                   (0x554u)
#define BQ7973X_FAULT_ADC_DIG2_POR_VAL                  (0x00u)

#define BQ7973X_FAULT_ADC_DIG2_VF2_DFAIL_POS                    (1u)
#define BQ7973X_FAULT_ADC_DIG2_VF2_DFAIL_MSK                    (0x2u)

#define BQ7973X_FAULT_ADC_DIG2_VF1_DFAIL_POS                    (0u)
#define BQ7973X_FAULT_ADC_DIG2_VF1_DFAIL_MSK                    (0x1u)

/* --------------------------------------------------------------------------
 * FAULT_ADC_MISC (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_ADC_MISC_OFFSET                   (0x555u)
#define BQ7973X_FAULT_ADC_MISC_POR_VAL                  (0x00u)

#define BQ7973X_FAULT_ADC_MISC_CC_OVF_POS                   (4u)
#define BQ7973X_FAULT_ADC_MISC_CC_OVF_MSK                   (0x10u)

#define BQ7973X_FAULT_ADC_MISC_ADC_PFAIL_POS                    (3u)
#define BQ7973X_FAULT_ADC_MISC_ADC_PFAIL_MSK                    (0x8u)

#define BQ7973X_FAULT_ADC_MISC_DIAG_MEAS_PFAIL_POS                  (2u)
#define BQ7973X_FAULT_ADC_MISC_DIAG_MEAS_PFAIL_MSK                  (0x4u)

#define BQ7973X_FAULT_ADC_MISC_DIAG_ANA_PFAIL_POS                   (1u)
#define BQ7973X_FAULT_ADC_MISC_DIAG_ANA_PFAIL_MSK                   (0x2u)

#define BQ7973X_FAULT_ADC_MISC_DIAG_ANA_ABORT_POS                   (0u)
#define BQ7973X_FAULT_ADC_MISC_DIAG_ANA_ABORT_MSK                   (0x1u)

/* --------------------------------------------------------------------------
 * FAULT_OC (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_FAULT_OC_OFFSET                     (0x55Bu)
#define BQ7973X_FAULT_OC_POR_VAL                    (0x00u)

#define BQ7973X_FAULT_OC_DIAG_OC_ABORT_POS                  (7u)
#define BQ7973X_FAULT_OC_DIAG_OC_ABORT_MSK                  (0x80u)

#define BQ7973X_FAULT_OC_OCC2_POS                   (6u)
#define BQ7973X_FAULT_OC_OCC2_MSK                   (0x40u)

#define BQ7973X_FAULT_OC_OCD2_POS                   (5u)
#define BQ7973X_FAULT_OC_OCD2_MSK                   (0x20u)

#define BQ7973X_FAULT_OC_OC_PFAIL_POS                   (4u)
#define BQ7973X_FAULT_OC_OC_PFAIL_MSK                   (0x10u)

#define BQ7973X_FAULT_OC_OCC1_POS                   (3u)
#define BQ7973X_FAULT_OC_OCC1_MSK                   (0x8u)

#define BQ7973X_FAULT_OC_OCD1_POS                   (2u)
#define BQ7973X_FAULT_OC_OCD1_MSK                   (0x4u)

/* --------------------------------------------------------------------------
 * VF2_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_VF2_HI_OFFSET                   (0x594u)
#define BQ7973X_VF2_HI_POR_VAL                  (0xFFu)

#define BQ7973X_VF2_HI_RESULT_POS                   (0u)
#define BQ7973X_VF2_HI_RESULT_MSK                   (0xffu)

/* --------------------------------------------------------------------------
 * VF1_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_VF1_HI_OFFSET                   (0x596u)
#define BQ7973X_VF1_HI_POR_VAL                  (0xFFu)

#define BQ7973X_VF1_HI_RESULT_POS                   (0u)
#define BQ7973X_VF1_HI_RESULT_MSK                   (0xffu)

/* --------------------------------------------------------------------------
 * CP_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CP_HI_OFFSET                    (0x598u)
#define BQ7973X_CP_HI_POR_VAL                   (0xFFu)

#define BQ7973X_CP_HI_RESULT_POS                    (0u)
#define BQ7973X_CP_HI_RESULT_MSK                    (0xffu)

/* --------------------------------------------------------------------------
 * CURRENT1_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CURRENT1_HI_OFFSET                  (0x59Cu)
#define BQ7973X_CURRENT1_HI_POR_VAL                     (0x80u)

#define BQ7973X_CURRENT1_HI_RESULT_POS                  (0u)
#define BQ7973X_CURRENT1_HI_RESULT_MSK                  (0xffu)

/* --------------------------------------------------------------------------
 * CURRENT2_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CURRENT2_HI_OFFSET                  (0x59Fu)
#define BQ7973X_CURRENT2_HI_POR_VAL                     (0x80u)

#define BQ7973X_CURRENT2_HI_RESULT_POS                  (0u)
#define BQ7973X_CURRENT2_HI_RESULT_MSK                  (0xffu)

/* --------------------------------------------------------------------------
 * OC1_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OC1_HI_OFFSET                   (0x5A2u)
#define BQ7973X_OC1_HI_POR_VAL                  (0x80u)

#define BQ7973X_OC1_HI_RESULT_POS                   (0u)
#define BQ7973X_OC1_HI_RESULT_MSK                   (0xffu)

/* --------------------------------------------------------------------------
 * OC2_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_OC2_HI_OFFSET                   (0x5A4u)
#define BQ7973X_OC2_HI_POR_VAL                  (0x80u)

#define BQ7973X_OC2_HI_RESULT_POS                   (0u)
#define BQ7973X_OC2_HI_RESULT_MSK                   (0xffu)

/* --------------------------------------------------------------------------
 * GPIO1_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO1_HI_OFFSET                     (0x5A8u)
#define BQ7973X_GPIO1_HI_POR_VAL                    (0xFFu)

#define BQ7973X_GPIO1_HI_RESULT_POS                     (0u)
#define BQ7973X_GPIO1_HI_RESULT_MSK                     (0xffu)

/* --------------------------------------------------------------------------
 * GPIO2_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO2_HI_OFFSET                     (0x5AAu)
#define BQ7973X_GPIO2_HI_POR_VAL                    (0xFFu)

#define BQ7973X_GPIO2_HI_RESULT_POS                     (0u)
#define BQ7973X_GPIO2_HI_RESULT_MSK                     (0xffu)

/* --------------------------------------------------------------------------
 * GPIO3_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO3_HI_OFFSET                     (0x5ACu)
#define BQ7973X_GPIO3_HI_POR_VAL                    (0xFFu)

#define BQ7973X_GPIO3_HI_RESULT_POS                     (0u)
#define BQ7973X_GPIO3_HI_RESULT_MSK                     (0xffu)

/* --------------------------------------------------------------------------
 * GPIO4_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO4_HI_OFFSET                     (0x5AEu)
#define BQ7973X_GPIO4_HI_POR_VAL                    (0xFFu)

#define BQ7973X_GPIO4_HI_RESULT_POS                     (0u)
#define BQ7973X_GPIO4_HI_RESULT_MSK                     (0xffu)

/* --------------------------------------------------------------------------
 * GPIO5_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO5_HI_OFFSET                     (0x5B0u)
#define BQ7973X_GPIO5_HI_POR_VAL                    (0xFFu)

#define BQ7973X_GPIO5_HI_RESULT_POS                     (0u)
#define BQ7973X_GPIO5_HI_RESULT_MSK                     (0xffu)

/* --------------------------------------------------------------------------
 * GPIO6_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO6_HI_OFFSET                     (0x5B2u)
#define BQ7973X_GPIO6_HI_POR_VAL                    (0xFFu)

#define BQ7973X_GPIO6_HI_RESULT_POS                     (0u)
#define BQ7973X_GPIO6_HI_RESULT_MSK                     (0xffu)

/* --------------------------------------------------------------------------
 * GPIO7_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO7_HI_OFFSET                     (0x5B4u)
#define BQ7973X_GPIO7_HI_POR_VAL                    (0xFFu)

#define BQ7973X_GPIO7_HI_RESULT_POS                     (0u)
#define BQ7973X_GPIO7_HI_RESULT_MSK                     (0xffu)

/* --------------------------------------------------------------------------
 * GPIO8_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO8_HI_OFFSET                     (0x5B6u)
#define BQ7973X_GPIO8_HI_POR_VAL                    (0xFFu)

#define BQ7973X_GPIO8_HI_RESULT_POS                     (0u)
#define BQ7973X_GPIO8_HI_RESULT_MSK                     (0xffu)

/* --------------------------------------------------------------------------
 * GPIO9_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO9_HI_OFFSET                     (0x5B8u)
#define BQ7973X_GPIO9_HI_POR_VAL                    (0xFFu)

#define BQ7973X_GPIO9_HI_RESULT_POS                 (0u)
#define BQ7973X_GPIO9_HI_RESULT_MSK                 (0xffu)

/* --------------------------------------------------------------------------
 * GPIO10_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO10_HI_OFFSET                    (0x5BAu)
#define BQ7973X_GPIO10_HI_POR_VAL                   (0xFFu)

#define BQ7973X_GPIO10_HI_RESULT_POS                (0u)
#define BQ7973X_GPIO10_HI_RESULT_MSK                (0xffu)

/* --------------------------------------------------------------------------
 * GPIO11_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO11_HI_OFFSET                    (0x5BCu)
#define BQ7973X_GPIO11_HI_POR_VAL                   (0xFFu)

#define BQ7973X_GPIO11_HI_RESULT_POS                (0u)
#define BQ7973X_GPIO11_HI_RESULT_MSK                (0xffu)

/* --------------------------------------------------------------------------
 * GPIO12_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO12_HI_OFFSET                    (0x5BEu)
#define BQ7973X_GPIO12_HI_POR_VAL                   (0xFFu)

#define BQ7973X_GPIO12_HI_RESULT_POS                (0u)
#define BQ7973X_GPIO12_HI_RESULT_MSK                (0xffu)

/* --------------------------------------------------------------------------
 * GPIO13_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO13_HI_OFFSET                    (0x5C0u)
#define BQ7973X_GPIO13_HI_POR_VAL                   (0xFFu)

#define BQ7973X_GPIO13_HI_RESULT_POS                (0u)
#define BQ7973X_GPIO13_HI_RESULT_MSK                (0xffu)

/* --------------------------------------------------------------------------
 * GPIO14_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO14_HI_OFFSET                    (0x5C2u)
#define BQ7973X_GPIO14_HI_POR_VAL                   (0xFFu)

#define BQ7973X_GPIO14_HI_RESULT_POS                (0u)
#define BQ7973X_GPIO14_HI_RESULT_MSK                (0xffu)

/* --------------------------------------------------------------------------
 * GPIO15_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_GPIO15_HI_OFFSET                    (0x5C4u)
#define BQ7973X_GPIO15_HI_POR_VAL                   (0xFFu)

#define BQ7973X_GPIO15_HI_RESULT_POS                (0u)
#define BQ7973X_GPIO15_HI_RESULT_MSK                (0xffu)

/* --------------------------------------------------------------------------
 * CC_ACC_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CC_ACC_HI_OFFSET                    (0x5C6u)
#define BQ7973X_CC_ACC_HI_POR_VAL                   (0x00u)

#define BQ7973X_CC_ACC_HI_RESULT_POS                (0u)
#define BQ7973X_CC_ACC_HI_RESULT_MSK                (0xffu)

/* --------------------------------------------------------------------------
 * CC_CNT (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_CC_CNT_OFFSET                       (0x5CAu)
#define BQ7973X_CC_CNT_POR_VAL                      (0x00u)

#define BQ7973X_CC_CNT_RESULT_POS                   (0u)
#define BQ7973X_CC_CNT_RESULT_MSK                   (0xffu)

/* --------------------------------------------------------------------------
 * DIAG_MAIN_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIAG_MAIN_HI_OFFSET                 (0x5EAu)
#define BQ7973X_DIAG_MAIN_HI_POR_VAL                (0xFFu)

#define BQ7973X_DIAG_MAIN_HI_RESULT_POS             (0u)
#define BQ7973X_DIAG_MAIN_HI_RESULT_MSK             (0xffu)

/* --------------------------------------------------------------------------
 * DIAG_RDNT_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIAG_RDNT_HI_OFFSET                 (0x5EDu)
#define BQ7973X_DIAG_RDNT_HI_POR_VAL                (0xFFu)

#define BQ7973X_DIAG_RDNT_HI_RESULT_POS             (0u)
#define BQ7973X_DIAG_RDNT_HI_RESULT_MSK             (0xffu)

/* --------------------------------------------------------------------------
 * DIETEMP1_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIETEMP1_HI_OFFSET                  (0x5F0u)
#define BQ7973X_DIETEMP1_HI_POR_VAL                 (0x80u)

#define BQ7973X_DIETEMP1_HI_RESULT_POS              (0u)
#define BQ7973X_DIETEMP1_HI_RESULT_MSK              (0xffu)

/* --------------------------------------------------------------------------
 * DIETEMP2_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIETEMP2_HI_OFFSET                  (0x5F2u)
#define BQ7973X_DIETEMP2_HI_POR_VAL                 (0x80u)

#define BQ7973X_DIETEMP2_HI_RESULT_POS              (0u)
#define BQ7973X_DIETEMP2_HI_RESULT_MSK              (0xffu)

/* --------------------------------------------------------------------------
 * REF_CAP_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_REF_CAP_HI_OFFSET                   (0x5F4u)
#define BQ7973X_REF_CAP_HI_POR_VAL                  (0xFFu)

#define BQ7973X_REF_CAP_HI_RESULT_POS               (0u)
#define BQ7973X_REF_CAP_HI_RESULT_MSK               (0xffu)

/* --------------------------------------------------------------------------
 * DIAG_D1_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIAG_D1_HI_OFFSET                   (0x5F6u)
#define BQ7973X_DIAG_D1_HI_POR_VAL                  (0xFFu)

#define BQ7973X_DIAG_D1_HI_RESULT_POS               (0u)
#define BQ7973X_DIAG_D1_HI_RESULT_MSK               (0xffu)

/* --------------------------------------------------------------------------
 * DIAG_D2_HI (RM):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DIAG_D2_HI_OFFSET                   (0x5F8u)
#define BQ7973X_DIAG_D2_HI_POR_VAL                  (0xFFu)

#define BQ7973X_DIAG_D2_HI_RESULT_POS               (0u)
#define BQ7973X_DIAG_D2_HI_RESULT_MSK               (0xffu)

/* --------------------------------------------------------------------------
 * REF_CAP_T0_HI (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_REF_CAP_T0_HI_OFFSET                (0x5FAu)
#define BQ7973X_REF_CAP_T0_HI_POR_VAL               (0xFFu)

#define BQ7973X_REF_CAP_T0_HI_RESULT_POS            (0u)
#define BQ7973X_REF_CAP_T0_HI_RESULT_MSK            (0xffu)

/* --------------------------------------------------------------------------
 * REF_CAP_T0_LO (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_REF_CAP_T0_LO_OFFSET                (0x5FBu)
#define BQ7973X_REF_CAP_T0_LO_POR_VAL               (0xFFu)

#define BQ7973X_REF_CAP_T0_LO_RESULT_POS            (0u)
#define BQ7973X_REF_CAP_T0_LO_RESULT_MSK            (0xffu)

/* --------------------------------------------------------------------------
 * I2C_RD_DATA (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_I2C_RD_DATA_OFFSET                  (0x610u)
#define BQ7973X_I2C_RD_DATA_POR_VAL                 (0x00u)

#define BQ7973X_I2C_RD_DATA_DATA_POS                (0u)
#define BQ7973X_I2C_RD_DATA_DATA_MSK                (0xffu)

/* --------------------------------------------------------------------------
 * MSPI_RX4 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_MSPI_RX4_OFFSET                     (0x630u)
#define BQ7973X_MSPI_RX4_POR_VAL                    (0x00u)

#define BQ7973X_MSPI_RX4_DATA_POS                   (0u)
#define BQ7973X_MSPI_RX4_DATA_MSK                   (0xffu)

/* --------------------------------------------------------------------------
 * MSPI_RX3 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_MSPI_RX3_OFFSET                     (0x631u)
#define BQ7973X_MSPI_RX3_POR_VAL                    (0x00u)

#define BQ7973X_MSPI_RX3_DATA_POS                   (0u)
#define BQ7973X_MSPI_RX3_DATA_MSK                   (0xffu)

/* --------------------------------------------------------------------------
 * MSPI_RX2 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_MSPI_RX2_OFFSET                     (0x632u)
#define BQ7973X_MSPI_RX2_POR_VAL                    (0x00u)

#define BQ7973X_MSPI_RX2_DATA_POS                   (0u)
#define BQ7973X_MSPI_RX2_DATA_MSK                   (0xffu)

/* --------------------------------------------------------------------------
 * MSPI_RX1 (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_MSPI_RX1_OFFSET                     (0x633u)
#define BQ7973X_MSPI_RX1_POR_VAL                    (0x00u)

#define BQ7973X_MSPI_RX1_DATA_POS                   (0u)
#define BQ7973X_MSPI_RX1_DATA_MSK                   (0xffu)

/* --------------------------------------------------------------------------
 * DEBUG_CTRL_UNLOCK (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEBUG_CTRL_UNLOCK_OFFSET            (0x700u)
#define BQ7973X_DEBUG_CTRL_UNLOCK_POR_VAL           (0x00u)

#define BQ7973X_DEBUG_CTRL_UNLOCK_CODE_POS          (0u)
#define BQ7973X_DEBUG_CTRL_UNLOCK_CODE_MSK          (0xffu)

/* --------------------------------------------------------------------------
 * DEBUG_COMM_CTRL1 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEBUG_COMM_CTRL1_OFFSET             (0x701u)
#define BQ7973X_DEBUG_COMM_CTRL1_POR_VAL            (0x02u)

#define BQ7973X_DEBUG_COMM_CTRL1_TWO_STOP_EN_POS    (4u)
#define BQ7973X_DEBUG_COMM_CTRL1_TWO_STOP_EN_MSK    (0x10u)

#define BQ7973X_DEBUG_COMM_CTRL1_UART_BAUD_POS      (3u)
#define BQ7973X_DEBUG_COMM_CTRL1_UART_BAUD_MSK      (0x8u)

#define BQ7973X_DEBUG_COMM_CTRL1_UART_MIRROR_EN_POS (2u)
#define BQ7973X_DEBUG_COMM_CTRL1_UART_MIRROR_EN_MSK (0x4u)

#define BQ7973X_DEBUG_COMM_CTRL1_UART_TX_EN_POS     (1u)
#define BQ7973X_DEBUG_COMM_CTRL1_UART_TX_EN_MSK     (0x2u)

#define BQ7973X_DEBUG_COMM_CTRL1_USER_UART_EN_POS   (0u)
#define BQ7973X_DEBUG_COMM_CTRL1_USER_UART_EN_MSK   (0x1u)

/* --------------------------------------------------------------------------
 * DEBUG_COMM_CTRL2 (R/W):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEBUG_COMM_CTRL2_OFFSET             (0x702u)
#define BQ7973X_DEBUG_COMM_CTRL2_POR_VAL            (0x1Eu)

#define BQ7973X_DEBUG_COMM_CTRL2_FCOMM_DIS_POS      (5u)
#define BQ7973X_DEBUG_COMM_CTRL2_FCOMM_DIS_MSK      (0x20u)

#define BQ7973X_DEBUG_COMM_CTRL2_COML_TX_EN_POS     (4u)
#define BQ7973X_DEBUG_COMM_CTRL2_COML_TX_EN_MSK     (0x10u)

#define BQ7973X_DEBUG_COMM_CTRL2_COML_RX_EN_POS     (3u)
#define BQ7973X_DEBUG_COMM_CTRL2_COML_RX_EN_MSK     (0x8u)

#define BQ7973X_DEBUG_COMM_CTRL2_COMH_TX_EN_POS     (2u)
#define BQ7973X_DEBUG_COMM_CTRL2_COMH_TX_EN_MSK     (0x4u)

#define BQ7973X_DEBUG_COMM_CTRL2_COMH_RX_EN_POS     (1u)
#define BQ7973X_DEBUG_COMM_CTRL2_COMH_RX_EN_MSK     (0x2u)

#define BQ7973X_DEBUG_COMM_CTRL2_USER_DAISY_EN_POS  (0u)
#define BQ7973X_DEBUG_COMM_CTRL2_USER_DAISY_EN_MSK  (0x1u)

/* --------------------------------------------------------------------------
 * DEBUG_COMM_STAT (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEBUG_COMM_STAT_OFFSET              (0x780u)
#define BQ7973X_DEBUG_COMM_STAT_POR_VAL             (0x33u)

#define BQ7973X_DEBUG_COMM_STAT_HW_UART_DRV_POS     (5u)
#define BQ7973X_DEBUG_COMM_STAT_HW_UART_DRV_MSK     (0x20u)

#define BQ7973X_DEBUG_COMM_STAT_HW_DAISY_DRV_POS    (4u)
#define BQ7973X_DEBUG_COMM_STAT_HW_DAISY_DRV_MSK    (0x10u)

#define BQ7973X_DEBUG_COMM_STAT_COML_TX_ON_POS      (3u)
#define BQ7973X_DEBUG_COMM_STAT_COML_TX_ON_MSK      (0x8u)

#define BQ7973X_DEBUG_COMM_STAT_COML_RX_ON_POS      (2u)
#define BQ7973X_DEBUG_COMM_STAT_COML_RX_ON_MSK      (0x4u)

#define BQ7973X_DEBUG_COMM_STAT_COMH_TX_ON_POS      (1u)
#define BQ7973X_DEBUG_COMM_STAT_COMH_TX_ON_MSK      (0x2u)

#define BQ7973X_DEBUG_COMM_STAT_COMH_RX_ON_POS      (0u)
#define BQ7973X_DEBUG_COMM_STAT_COMH_RX_ON_MSK      (0x1u)

/* --------------------------------------------------------------------------
 * DEBUG_UART_RC_TR (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEBUG_UART_RC_TR_OFFSET             (0x781u)
#define BQ7973X_DEBUG_UART_RC_TR_POR_VAL            (0x00u)

#define BQ7973X_DEBUG_UART_RC_TR_TR_SOF_POS         (7u)
#define BQ7973X_DEBUG_UART_RC_TR_TR_SOF_MSK         (0x80u)

#define BQ7973X_DEBUG_UART_RC_TR_TR_WAIT_POS        (6u)
#define BQ7973X_DEBUG_UART_RC_TR_TR_WAIT_MSK        (0x40u)

#define BQ7973X_DEBUG_UART_RC_TR_RC_IERR_POS        (5u)
#define BQ7973X_DEBUG_UART_RC_TR_RC_IERR_MSK        (0x20u)

#define BQ7973X_DEBUG_UART_RC_TR_RC_TXDIS_POS       (4u)
#define BQ7973X_DEBUG_UART_RC_TR_RC_TXDIS_MSK       (0x10u)

#define BQ7973X_DEBUG_UART_RC_TR_RC_SOF_POS         (3u)
#define BQ7973X_DEBUG_UART_RC_TR_RC_SOF_MSK         (0x8u)

#define BQ7973X_DEBUG_UART_RC_TR_RC_BYTE_ERR_POS    (2u)
#define BQ7973X_DEBUG_UART_RC_TR_RC_BYTE_ERR_MSK    (0x4u)

#define BQ7973X_DEBUG_UART_RC_TR_RC_UNEXP_POS       (1u)
#define BQ7973X_DEBUG_UART_RC_TR_RC_UNEXP_MSK       (0x2u)

#define BQ7973X_DEBUG_UART_RC_TR_RC_CRC_POS         (0u)
#define BQ7973X_DEBUG_UART_RC_TR_RC_CRC_MSK         (0x1u)

/* --------------------------------------------------------------------------
 * DEBUG_COMH_BIT (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEBUG_COMH_BIT_OFFSET               (0x782u)
#define BQ7973X_DEBUG_COMH_BIT_POR_VAL              (0x00u)

#define BQ7973X_DEBUG_COMH_BIT_PERR_POS             (4u)
#define BQ7973X_DEBUG_COMH_BIT_PERR_MSK             (0x10u)

#define BQ7973X_DEBUG_COMH_BIT_BERR_TAG_POS         (3u)
#define BQ7973X_DEBUG_COMH_BIT_BERR_TAG_MSK         (0x8u)

#define BQ7973X_DEBUG_COMH_BIT_SYNC2_POS            (2u)
#define BQ7973X_DEBUG_COMH_BIT_SYNC2_MSK            (0x4u)

#define BQ7973X_DEBUG_COMH_BIT_SYNC1_POS            (1u)
#define BQ7973X_DEBUG_COMH_BIT_SYNC1_MSK            (0x2u)

#define BQ7973X_DEBUG_COMH_BIT_BIT_POS              (0u)
#define BQ7973X_DEBUG_COMH_BIT_BIT_MSK              (0x1u)

/* --------------------------------------------------------------------------
 * DEBUG_COMH_RC_TR (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEBUG_COMH_RC_TR_OFFSET             (0x783u)
#define BQ7973X_DEBUG_COMH_RC_TR_POR_VAL            (0x00u)

#define BQ7973X_DEBUG_COMH_RC_TR_TR_WAIT_POS        (6u)
#define BQ7973X_DEBUG_COMH_RC_TR_TR_WAIT_MSK        (0x40u)

#define BQ7973X_DEBUG_COMH_RC_TR_RC_IERR_POS        (5u)
#define BQ7973X_DEBUG_COMH_RC_TR_RC_IERR_MSK        (0x20u)

#define BQ7973X_DEBUG_COMH_RC_TR_RC_TXDIS_POS       (4u)
#define BQ7973X_DEBUG_COMH_RC_TR_RC_TXDIS_MSK       (0x10u)

#define BQ7973X_DEBUG_COMH_RC_TR_RC_SOF_POS         (3u)
#define BQ7973X_DEBUG_COMH_RC_TR_RC_SOF_MSK         (0x8u)

#define BQ7973X_DEBUG_COMH_RC_TR_RC_BYTE_ERR_POS    (2u)
#define BQ7973X_DEBUG_COMH_RC_TR_RC_BYTE_ERR_MSK    (0x4u)

#define BQ7973X_DEBUG_COMH_RC_TR_RC_UNEXP_POS       (1u)
#define BQ7973X_DEBUG_COMH_RC_TR_RC_UNEXP_MSK       (0x2u)

#define BQ7973X_DEBUG_COMH_RC_TR_RC_CRC_POS         (0u)
#define BQ7973X_DEBUG_COMH_RC_TR_RC_CRC_MSK         (0x1u)

/* --------------------------------------------------------------------------
 * DEBUG_COMH_RR (R):
 * -------------------------------------------------------------------------- */


#define BQ7973X_DEBUG_COMH_RR_OFFSET                (0x784u)
#define BQ7973X_DEBUG_COMH_RR_POR_VAL               (0x00u)

#define BQ7973X_DEBUG_COMH_RR_RR_TXDIS_POS          (4u)
#define BQ7973X_DEBUG_COMH_RR_RR_TXDIS_MSK          (0x10u)

#define BQ7973X_DEBUG_COMH_RR_RR_SOF_POS            (3u)
#define BQ7973X_DEBUG_COMH_RR_RR_SOF_MSK            (0x8u)

#define BQ7973X_DEBUG_COMH_RR_RR_BYTE_ERR_POS       (2u)
#define BQ7973X_DEBUG_COMH_RR_RR_BYTE_ERR_MSK       (0x4u)

#define BQ7973X_DEBUG_COMH_RR_RR_UNEXP_POS          (1u)
#define BQ7973X_DEBUG_COMH_RR_RR_UNEXP_MSK          (0x2u)

#define BQ7973X_DEBUG_COMH_RR_RR_CRC_POS            (0u)
#define BQ7973X_DEBUG_COMH_RR_RR_CRC_MSK            (0x1u)

/* --------------------------------------------------------------------------
 * DEBUG_COML_BIT (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEBUG_COML_BIT_OFFSET               (0x785u)
#define BQ7973X_DEBUG_COML_BIT_POR_VAL              (0x00u)

#define BQ7973X_DEBUG_COML_BIT_PERR_POS             (4u)
#define BQ7973X_DEBUG_COML_BIT_PERR_MSK             (0x10u)

#define BQ7973X_DEBUG_COML_BIT_BERR_TAG_POS         (3u)
#define BQ7973X_DEBUG_COML_BIT_BERR_TAG_MSK         (0x8u)

#define BQ7973X_DEBUG_COML_BIT_SYNC2_POS            (2u)
#define BQ7973X_DEBUG_COML_BIT_SYNC2_MSK            (0x4u)

#define BQ7973X_DEBUG_COML_BIT_SYNC1_POS            (1u)
#define BQ7973X_DEBUG_COML_BIT_SYNC1_MSK            (0x2u)

#define BQ7973X_DEBUG_COML_BIT_BIT_POS              (0u)
#define BQ7973X_DEBUG_COML_BIT_BIT_MSK              (0x1u)

/* --------------------------------------------------------------------------
 * DEBUG_COML_RC_TR (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEBUG_COML_RC_TR_OFFSET             (0x786u)
#define BQ7973X_DEBUG_COML_RC_TR_POR_VAL            (0x00u)

#define BQ7973X_DEBUG_COML_RC_TR_TR_WAIT_POS        (6u)
#define BQ7973X_DEBUG_COML_RC_TR_TR_WAIT_MSK        (0x40u)

#define BQ7973X_DEBUG_COML_RC_TR_RC_IERR_POS        (5u)
#define BQ7973X_DEBUG_COML_RC_TR_RC_IERR_MSK        (0x20u)

#define BQ7973X_DEBUG_COML_RC_TR_RC_TXDIS_POS       (4u)
#define BQ7973X_DEBUG_COML_RC_TR_RC_TXDIS_MSK       (0x10u)

#define BQ7973X_DEBUG_COML_RC_TR_RC_SOF_POS         (3u)
#define BQ7973X_DEBUG_COML_RC_TR_RC_SOF_MSK         (0x8u)

#define BQ7973X_DEBUG_COML_RC_TR_RC_BYTE_ERR_POS    (2u)
#define BQ7973X_DEBUG_COML_RC_TR_RC_BYTE_ERR_MSK    (0x4u)

#define BQ7973X_DEBUG_COML_RC_TR_RC_UNEXP_POS       (1u)
#define BQ7973X_DEBUG_COML_RC_TR_RC_UNEXP_MSK       (0x2u)

#define BQ7973X_DEBUG_COML_RC_TR_RC_CRC_POS         (0u)
#define BQ7973X_DEBUG_COML_RC_TR_RC_CRC_MSK         (0x1u)

/* --------------------------------------------------------------------------
 * DEBUG_COML_RR (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEBUG_COML_RR_OFFSET                (0x787u)
#define BQ7973X_DEBUG_COML_RR_POR_VAL               (0x00u)

#define BQ7973X_DEBUG_COML_RR_RR_TXDIS_POS          (4u)
#define BQ7973X_DEBUG_COML_RR_RR_TXDIS_MSK          (0x10u)

#define BQ7973X_DEBUG_COML_RR_RR_SOF_POS            (3u)
#define BQ7973X_DEBUG_COML_RR_RR_SOF_MSK            (0x8u)

#define BQ7973X_DEBUG_COML_RR_RR_BYTE_ERR_POS       (2u)
#define BQ7973X_DEBUG_COML_RR_RR_BYTE_ERR_MSK       (0x4u)

#define BQ7973X_DEBUG_COML_RR_RR_UNEXP_POS          (1u)
#define BQ7973X_DEBUG_COML_RR_RR_UNEXP_MSK          (0x2u)

#define BQ7973X_DEBUG_COML_RR_RR_CRC_POS            (0u)
#define BQ7973X_DEBUG_COML_RR_RR_CRC_MSK            (0x1u)

/* --------------------------------------------------------------------------
 * DEBUG_SEC_DED_BLK (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEBUG_SEC_DED_BLK_OFFSET            (0x788u)
#define BQ7973X_DEBUG_SEC_DED_BLK_POR_VAL           (0x00u)

#define BQ7973X_DEBUG_SEC_DED_BLK_SEC_DED_POS       (7u)
#define BQ7973X_DEBUG_SEC_DED_BLK_SEC_DED_MSK       (0x80u)

#define BQ7973X_DEBUG_SEC_DED_BLK_BLOCK_POS         (0u)
#define BQ7973X_DEBUG_SEC_DED_BLK_BLOCK_MSK         (0x7fu)

/* --------------------------------------------------------------------------
 * DEBUG_SPI_BIT (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEBUG_SPI_BIT_OFFSET                (0x789u)
#define BQ7973X_DEBUG_SPI_BIT_POR_VAL               (0x00u)

#define BQ7973X_DEBUG_SPI_BIT_FMT_ERR_POS           (4u)
#define BQ7973X_DEBUG_SPI_BIT_FMT_ERR_MSK           (0x10u)

#define BQ7973X_DEBUG_SPI_BIT_COMMCLR_ERR_POS       (3u)
#define BQ7973X_DEBUG_SPI_BIT_COMMCLR_ERR_MSK       (0x8u)

#define BQ7973X_DEBUG_SPI_BIT_RSDATA_UNEXP_POS      (2u)
#define BQ7973X_DEBUG_SPI_BIT_RSDATA_UNEXP_MSK      (0x4u)

#define BQ7973X_DEBUG_SPI_BIT_TXFIFO_UF_POS         (1u)
#define BQ7973X_DEBUG_SPI_BIT_TXFIFO_UF_MSK         (0x2u)

#define BQ7973X_DEBUG_SPI_BIT_RXFIFO_OF_POS         (0u)
#define BQ7973X_DEBUG_SPI_BIT_RXFIFO_OF_MSK         (0x1u)

/* --------------------------------------------------------------------------
 * DEBUG_SPI_RC_TR (R):
 * -------------------------------------------------------------------------- */

#define BQ7973X_DEBUG_SPI_RC_TR_OFFSET              (0x78Au)
#define BQ7973X_DEBUG_SPI_RC_TR_POR_VAL             (0x00u)

#define BQ7973X_DEBUG_SPI_RC_TR_TR_SOF_POS          (3u)
#define BQ7973X_DEBUG_SPI_RC_TR_TR_SOF_MSK          (0x8u)

#define BQ7973X_DEBUG_SPI_RC_TR_RC_IERR_POS         (2u)
#define BQ7973X_DEBUG_SPI_RC_TR_RC_IERR_MSK         (0x4u)

#define BQ7973X_DEBUG_SPI_RC_TR_RC_SOF_POS          (1u)
#define BQ7973X_DEBUG_SPI_RC_TR_RC_SOF_MSK          (0x2u)
#define BQ7973X_DEBUG_SPI_RC_TR_RC_CRC_POS          (0u)
#define BQ7973X_DEBUG_SPI_RC_TR_RC_CRC_MSK          (0x1u)

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

#endif /* BQ7973X_REGS_H */

/*********************************************************************************************************************
 * End of File: bq7973x_regs.h
 *********************************************************************************************************************/
