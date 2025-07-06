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
 *  File:       bq7973x_cfg.h
 *  Project:    TIBMS
 *  Module:     PMI
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed functionalities for PMI interface
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

#ifndef BQ7973X_CFG_H
#define BQ7973X_CFG_H

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

#include "bq7973x_regs.h"

/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

#define BQ7973X_CFG_MAJOR_VERSION                   (0x01u)
#define BQ7973X_CFG_MINOR_VERSION                   (0x00u)
#define BQ7973X_CFG_PATCH_VERSION                   (0x00u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

#define BQ7973X_WAKEUP_PING_DELAY                   (4u)   	/* 4.5ms 	*/
#define BQ7973X_WAKEUP_TONE_DELAY                   (6u)    /* 5.5ms */

#define BQ7973X_INVALDVOLVALUE                      (0x8000u)
#define BQ7973X_INVALDVOLDIEVALUE                   ((sint16)0x8000)
#define BQ7973X_INVALDVOLCSVALUE                    (0x800000U)
#define BQ7973X_CLIPVOLCSVALUE                      (0x7fffffU)
#define BQ7973X_TEMPDIFFPLAUVALUE                   (50u)  /* 5 degrees Celsius */
#define BQ7973X_MAXBGPLAUVALUE                      (14000u)
#define BQ7973X_MINBGPLAUVALUE                      (8000u)
#define BQ7973X_MAXREFCAPPLAUVALUE                  (50000u)
#define BQ7973X_MINREFCAPPLAUVALUE                  (30000u)
#define BQ7973X_LSB_DIETEMP                         (0.1f)
#define BQ7973X_LSB_CS                              (0.0001f)  /* 0.1uV/LSB */
#define BQ7973X_R_SHUNT                             (10.0f)
#define BQ7973X_CS_REFRESH                          (1.024f)
#define BQ7973X_INSUL_RES1                          (499.0f * 4) /* kiloohm */
#define BQ7973X_INSUL_RES2                          (499.0f * 4) /* kiloohm */
#define BQ7973X_INSUL_RES3                          (60.4f / 3)  /* kiloohm */
#define BQ7973X_COMIF_IFACES		                (1u)
#define BQ7973X_IFACES		                        (1u)
#define BQ7973X_MAXSWADJSHRTVALUE                   (25000u)
#define BQ7973X_MINSWADJSHRTVALUE                   (10000u)

#define BQ7973X_DIO_NFAULT_PIN                  DioChannel_N2HET1_22 /* Connect NFAULT to MUC pin PA6. */

/* INIT(1) + REG(2) + DATA(1) + CRC(2) */
#define BQ7973X_REQ_FRAME_FIXED_LEN                 (6u)

/* LEN(1) + ID(1) + REG(2) + DATA(1) + CRC(2) */
#define BQ7973X_RSP_FRAME_FIXED_LEN                 (7u)

/* Data_Len(1) + DEV_ID(1) + REG_ADR(2) */
#define BQ7973X_RSP_FRAME_PREFIX_LEN                (4u)

#define BQ7973X_WAIT_SPI_RDY_TIME                   (2u)
#define BQ7973X_WAIT_SPI_BIT_TIME                   (12u)
#define BQ7973X_WAIT_COMCLEAR_RESP                  (5u)
#define BQ7973X_WAIT_PING_RESPONSE                  (5u)

#define BQ7973X_TX_DATA_MAX_LEN                     (14u)
#define BQ7973X_RX_DATA_MAX_LEN                     (128u)

#define BQ7973X_DEV_ADR_RANGE_MAX                   (0x3Fu)

#define BQ7973X_MAX_RETRY                           (3u)

#define BQ7973X_HW_RESET_DELAY                      (45u)  /* 45ms */
#define BQ7973X_GOTO_SLEEP_DELAY                    (1u)    /* 1000us */
#define BQ7973X_SOFT_RESET_DELAY                    (1u)   /* 1ms */

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef enum BQ7973x_SiliconRevType_Tag
{
    BQ7973X_SILICON_REVA,
    BQ7973X_SILICON_REVB,
    BQ7973X_SILICON_REVC,
    BQ7973X_SILICON_REVD
}
BQ7973x_SiliconRevType;

typedef struct Bq7973x_ReqDataType_Tag
{
    const ServiceCfgType *pServiceReq;
    uint8 *pRxData;

    uint16 wRspDataLen;
    uint16 wCurrRspDataLen;

    uint16 wReqDataLen;
    uint8 uCmd;
    uint8 uBlockSiz;

    uint8 zTxCmd[BQ7973X_TX_DATA_MAX_LEN];
    uint8 uStatus;
    uint8 uRetryStat;
}
Bq7973x_ReqDataType;

typedef struct Bq7973x_ComConfigType_Tag
{
    const Basic_ConfigType *pBswCfg;
    Bq7973x_ReqDataType *pCmdDataMemCfg;

    void *pCmdQueFreeMemCfg;
    void *pCmdQueExecMemCfg;
    void *pReadQueExecMemCfg;

    uint16 wCmdQueSizeCfg;
    uint16 wCmdQueCntCfg;

    uint16 wReadQueSizeCfg;
    uint16 wReadQueCntCfg;

    uint8 uCmdIfaceCfg;
    uint8 uRsvd[3];
}
Bq7973x_ComConfigType;

typedef struct Bq7973x_NVMRegsType_Tag
{
    uint8 uDev_Conf[2u];      /* 0x002, 0x03E */
    uint8 uComm_Conf;         /* 0x004 */
    uint8 uAdc_Conf;          /* 0x008 */
    uint8 uFault_Msk[3u];     /* 0x00F, 0x010, 0x03A */
    uint8 uCS_Adc1_Cal[2u];   /* 0x011, 0x012 */
    uint8 uGpio_Conf[BQ7973X_GPIO_CONF1_6_REG_NUM + BQ7973X_GPIO_CONF7_8_REG_NUM];     /* 0x01E - 0x023, 0x03B - 0x03C */
    uint8 uCS_Adc2_Cal[2u];   /* 0x030, 0x031 */
    uint8 uOC_Conf[8u];       /* 0x032 - 0x039 */
    uint8 uMspi_Conf;         /* 0x03D */
    /* Diag_Update_Start check if this reg is updated somewhere and add the corrsponding value */
    uint8 uOVUV_Thresh[2u]; /* 0x009 - 0x00A */
    uint8 uOTUT_Thresh;      /* 0x00B */
    /* Diag_Update_END */
}

Bq7973x_NVMRegsType;

typedef struct Bq7973x_CtrlRegsType_Tag
{
    uint8 uControl2;             /* 0x30A */
    uint8 uAdc_Ctrl[4u];         /* 0x310 - 0x313 */
    uint8 uDiag_Adc_Ctrl[2u];    /* 0x315 - 0x316 */
    uint8 uDiag_Misc_Ctrl[2u];   /* 0x317 - 0x318 */
    uint8 uDiag_OC_Ctrl[2u];     /* 0x319 - 0x31A */
    uint8 uFault_RST[3u];        /* 0x340 - 0x342 */
    uint8 uOC_Ctrl[2u];          /* 0x360 - 0x361 */
    uint8 uI2C_WR_Data;          /* 0x370 */
    uint8 uI2C_Ctrl;             /* 0x371 */
    uint8 uMspi_Ctrl;            /* 0x390 */
    uint8 uMspi_Tx[4u];          /* 0x391 - 0x394 */
    uint8 uMspi_Exe;             /* 0x395 */
    uint8 uCC_Ctrl;              /* 0x3A0 */
    uint8 uSW_Ctrl;              /* 0x3B0 */
    uint8 uGpio_Ctrl[2u];        /* 0x03B3 - 0x03B4 */
    uint8 uGPIO_PWM_CTRL;        /* 0x03B6 */
}
Bq7973x_CtrlRegsType;

typedef struct Bq7973x_RegsType_Tag
{
    Bq7973x_NVMRegsType zRegNVM;
    Bq7973x_CtrlRegsType zRegCtrl;
}
Bq7973x_RegsType;

typedef struct Bq7973x_DiagReqType_Tag
{
    void *pUserData;
    const ServiceCfgType *pDiagSvcCfg;
    uint8 uDiagReqId;
    uint8 uDiagStep;
    uint8 uDiagStat;
    uint8 uDiagRetry;
}
Bq7973x_DiagReqType;

typedef struct Bq7973x_ConfigType_Tag
{
    const Bq7973x_RegsType *p_rcBq7973xRegsCfg;

/* Diag_Update_Start */
    Bq7973x_DiagReqType *pDiagReqDataMemCfg;
    void *pDiagCmdQueFreeMemCfg;
    void *pDiagCmdQueExecMemCfg;

    uint16 uDiagCmdQueSizeCfg;
    uint16 uDiagCmdQueCntCfg;

   // uint8 eComIf;
   // uint8 uDefDevId;

   // uint16 uRsvd;
   /* Diag_Update_END */

}
Bq7973x_ConfigType;


typedef struct DiagPMIFdtiStatType_Tag
{
    uint8                      uSM_FACTCRC_STAT;    /* FACT CRC status bit */
    uint8                      uSM_OTP_UNLCKSTAT;   /* OTP program unlock indication */

}DiagPMIFdtiStatType;
typedef struct DiagPMIFdtiResultType_Tag
{
    uint8                      uSM_DCOMP_FLTINJ_VF; /* VF Digital comparison Fault Injection */
    uint8                      uSM_DCOMP_FLTINJ_GP; /* GP Digital comparison Fault Injection */
    uint8                      uSM_ACOMP_FLTINJ_VF; /* VF Analog comparison Fault Injection */
    uint16                     uSM_ACOMP_FLTINJ_GP; /* GP Analog comparison Fault Injection */
    uint8                      uSM_BG1_DIAG;        /* BG1 Reference Diagnostic */
    uint8                      uSM_BG2_DIAG;        /* BG2 Reference Diagnostic */
    uint8                      uSM_PWR_BIST;        /* Power Supply BIST */
    uint8                      uSM_REFCAP_DIAG;     /* REF_CAP Voltage Diagnostic */
    uint8                      uSM_COMM_FLTDIAG_TXDIS; /* MCU communication fault diagnostics */
    uint8                      uSM_OTP_ECC;         /* OTP ECC detection */
    uint8                      uSM_FLIP_RESET;      /* Flip Reset ADC Dignostic */
    uint8                      uSM_OC_DET_DIAG;     /* Over Current Detection diagnostic */
    uint8                      uSM_OC_UNLOCK;       /* Unlock Over Current Fault Enable Access */
    uint8                      uSM_FACT_CRCDIAG;    /* FACT CRC diagnostic */
    uint8                      uSM_SW_ADJSHRT;      /* SW Adjacent Short diagnostic */
    uint8                      uSM_VIF_CRC;         /* Daisy Chain CRC Error detection */

    uint8                      uSM_COMM_FLTDIAG_CRC;   /* MCU communication fault diagnostics */
    uint8                      uSM_COMM_FLTDIAG_INIT;  /* MCU communication fault diagnostics */

    uint8                      uSM_NFAULT_DIAG;     /* NFAULT pin diagnostic */
    uint8                      uSM_VF_ACOMP;        /* VF Voltage Analog Comparison */
    uint8                      uSM_VF_DCOMP;        /* VF Voltage Digital Comparison */
    uint16                     uSM_ACOMP_GPIO;        /* Vn_GPIOn Analog Comparison */
    uint8                      uSM_GP_DCOMP;        /* Vn_GPIOn Digital Comparison */
    uint8                      uSM_AVDD_OV;         /* AVDD OV Detection */
    uint8                      uSM_AVDD_OSC;        /* AVDD OSC Detection */
    uint8                      uSM_DVDD_OV;         /* DVDD OV Detection */
    uint8                      uSM_DVDD_UV;         /* DVDD UV Detection */
    uint8                      uSM_TSREF_OV;        /* TSREF OV Detection */
    uint8                      uSM_TSREF_UV;        /* TSREF UV Detection */
    uint8                      uSM_TSREF_OSC;       /* TSREF OSC Detection */
    uint8                      uSM_CP_OV;           /* Charge Pump UV Detection */
    uint8                      uSM_CP_UV;           /* Charge Pump OV Detection */
    uint8                      uSM_REFVSS_OPEN;     /* REF_VSS Open Detection */
    uint8                      uSM_VSS_OPEN;        /* AVSS Open Detection */
    uint8                      uSM_VF_PLAU;         /* VF Input Plausibility */
    uint16                     xSM_GP_PLAU;         /* Vn_GPIOn Input Plausibility */
    uint8                      uSM_DRDY_VF;         /* VF ADC Data Ready status bit */
    uint8                      uSM_DRDY_GP;         /* GP ADC Data Ready status bit */
    uint8                      uSM_DRDY_DIAGD1D2;   /* D1D2 ADC DIAG Data Ready status bit */
    uint8                      uSM_DRDY_ANA_VF;     /* VF Comp Data Ready status bit */
    uint8                      uSM_DRDY_ANA_GP;     /* GP Comp Data Ready status bit */
    uint8                      uSM_DRDY_DIG;        /* Digital Comp Data Ready status bit */
    uint8                      uSM_MAIN_PFAIL;      /* Main ADC Parity Fail Detection */
    uint8                      uSM_DIAG_PFAIL;      /* DIAG ADC Parity Fail Detection */
    uint8                      uSM_ANA_PFAIL;       /* ACOMP Parity Fail Detection */
    uint8                      uSM_DIAG_ERR;        /* ADC COMP Diagnostic Error */
    uint8                      uSM_VF_OPEN;         /* VF Open Wire detection */
    uint16                     xSM_GP_OPEN;         /* Vn_GPIOn Open Wire detection */
    uint16                     xSM_GP_ADJSHRT;      /* Vn_GPIOn Adjacent Short diagnostic */
    uint8                      uSM_GPVF_ADJSHRT;    /* V15_GPIO15,VF2 Adjacent Short diagnostic */
    uint8                      uSM_SW_MON;          /* SW Output Monitor (optional) */
    uint8                      uSM_CS_PLAU;         /* Current Sense Input Plausibility */
    uint8                      uSM_CS_COMP;         /* Current Sense Input Difference */
    uint8                      uSM_DRDY_CS;         /* CS ADC Data Ready status bit */
    uint8                      uSM_CS_OCC1;         /* Over Current Detection during charge */
    uint8                      uSM_CS_OCC2;         /* Over Current Detection during charge */
    uint8                      uSM_CS_OCD1;         /* Over Current Detection during discharge */
    uint8                      uSM_CS_OCD2;         /* Over Current Detection during discharge */
    uint8                      uSM_OC_PLAU;         /* Over Current Sense Input Plausibility */
    uint8                      uSM_OC_COMP;         /* Over Current Sense Input Difference */
    uint8                      uSM_OC_PFAIL;        /* OC ADC Parity Fail Detection */
    uint8                      uSM_OC_ERR;          /* Over Current Detection Abort */
    uint8                      uSM_OC_OS;           /* OC1, OC2 Output Open/Short */

    uint8                      uSM_COMM_BERR;       /* Byte error detection */
    uint8                      uSM_COMM_IERR;       /* IERR detection */
    uint8                      uSM_COMM_SOF;        /* Start of Frame error detection */
    uint8                      uSM_COMM_SYNC1;      /* Daisy Chain SYNC1 Error detection */
    uint8                      uSM_COMM_SYNC2;      /* Daisy Chain SYNC2 Error detection */
    uint8                      uSM_COMM_TIMEOUT;    /* Comm Timeout detection */
    uint8                      uSM_COMM_TXDIS;      /* TXDIS error detection */
    uint8                      uSM_COMM_UNEXP;      /* UNEXP communication detection */
    uint8                      uSM_COMM_WAIT;       /* WAIT error detection */


    uint8                      uSM_COMM_FLTDIAG_DELAY; /* MCU communication fault diagnostics */
    uint8                      uSM_SLEEP_FAULT;     /* Sleep Mode Fault Tone */
    uint8                      uSM_SLEEP_HB;        /* Sleep Mode Heartbeat */
    uint8                      uSM_SLEEP_FSTHB;     /* Fast heartbeat detection */
    uint8                      uSM_VIF_BIT;         /* Daisy Chain BIT Error detection */
    uint8                      uSM_VIF_BYTE;        /* Daisy Chain BYTE Error detection */
    uint8                      uSM_VIF_CRCDIAG;     /* Daisy Chain CRC diagnostic */
    uint8                      uSM_UART_STOPBIT;    /* UART STOP Bit Error detection */
    uint8                      uSM_UART_CRC;        /* UART CRC Error detection */
    uint8                      uSM_LFOSC_FREQ;      /* LFOSC frequency mismatch detection */
    uint8                      uSM_DIE_TEMP;        /* Die Temp1/Temp2 plausibility */
    uint8                      uSM_DIE_TEMP_DIFF;   /* Die Temp1/Temp2 difference */
    uint8                      uSM_TSHUT_STAT;      /* Thermal Shutdown detection */
    uint8                      uSM_CUST_CRC;        /* Customer Register CRC detection */
    uint8                      uSM_FACT_CRC;        /* Factory Register CRC detection */

    uint8                      uSM_OTP_UNLCK;       /* OTP program unlock */
    uint8                      uSM_OTP_SUV_SOVERR;  /* OTP program voltage lock out */
    uint8                      uSM_OTP_UVOVERR;     /* OTP programming voltage error detection */
    uint8                      uSM_OTP_PERR;        /* OTP Program Error detection */
    uint8                      uSM_OTP_ECCDIAG;     /* OTP ECC Internal diagnostic */
    uint8                      uSM_OTP_LOAD_ERR;    /* OTP Customer/Factory Load Error */
    uint8                      uSM_OTP_OVERR;       /* OTP Over Voltage Error */
    uint8                      uSM_OTP_MARGIN;      /* OTP Read Margin Test */
    uint8                      uSM_FACT_TM;         /* Factory Testmode Detection */
    uint8                      uSM_OTP_UNLCKSTAT;    /* OTP State // TBD added in 8.2 and may need used from DiagPMIFdtiStatType */
// TBD
    uint32                     uSM_ACOMP_VCELL;     /* SM_ACOMP CELL Analog comparison Fault Injection */
    uint32                     uSM_NFAULT;          /* NFAULT*/

} DiagPMIFdtiResultType;

/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

extern CONST(Pmi_ConfigType, TIBMSCFG_CONST) zPmiCfg_0;                            // Both do not work together for now
extern CONST(Pmi_ConfigType, TIBMSCFG_CONST) zPmiCfgDaisyChain;

extern CONST(ServiceCfgType, TIBMSCFG_CONST) zPmiFuncCfg_1[];
extern CONST(ServiceCfgType, TIBMSCFG_CONST) zPmiDiagCfg_0[];
extern CONST(ServiceCfgType, TIBMSCFG_CONST) zPmiComCfg_0[];
extern CONST(Basic_ConfigType, TIBMSCFG_CONST) zPmiBswCfg_0;
extern CONST(ServiceCfgType, TIBMSCFG_CONST) zPmiDiagCfg_1[];
extern CONST(Comif_ConfigType, TIBMSCFG_CONST) zPmiComifCfgDirectSpiCfg;

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

#endif /*BQ7973X_CFG_H*/

/*********************************************************************************************************************
 * End of File: bq7973x_cfg.h
 *********************************************************************************************************************/
