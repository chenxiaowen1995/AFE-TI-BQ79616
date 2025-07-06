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
 *  File:       tibms_diag.h
 *  Project:    TIBMS
 *  Module:     DIAG
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  Exposed API's and configurations for the TI BMS Diag functionalities
 *
 *--------------------------------------------------------------------------------------------------------------------
 * Author:  SEM
 *--------------------------------------------------------------------------------------------------------------------
 * Revision History (top to bottom: first revision to last revision)
 *--------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------
 * 01.00.00       08April2023   SEM                 0000000000000    Initial version
 * 01.01.00       08June2023    SEM                 0000000000000    Updated Diagnostic for WBMS Retry safe
 *
 *********************************************************************************************************************/

#ifndef TIBMS_DIAG_H
#define TIBMS_DIAG_H

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

#include "tibms_cfg.h"
#include "bq7973x_cfg.h"
/*********************************************************************************************************************
 * Version (Check if required)
 *********************************************************************************************************************/

#define TIBMS_DIAG_API_MAJOR_VERSION            (0x01u)
#define TIBMS_DIAG_API_MINOR_VERSION            (0x01u)
#define TIBMS_DIAG_API_PATCH_VERSION            (0x00u)

/*********************************************************************************************************************
 * Exported Preprocessor #define Macros
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

#define DIAGNOERROR                             (0u)
#define DIAGERROR                               (1u)

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef enum Bq7971x_DiagStatype_Tag
{
    DIAG_STAT_NOERROR = 0x0u,

    DIAG_STAT_PENDING = 0x9u,
    DIAG_STAT_ERROR = 0xAu,
    DIAG_STAT_RUN_ERROR = 0xBu,
    DIAG_STAT_DRDY_ERROR = 0xCu,
    DIAG_STAT_EXEC_ERROR = 0xDu,
    DIAG_STAT_ABORT = 0xEu,

    DIAG_STAT_NOT_READY = 0xFu,
}
Bq7971x_DiagStatype;

typedef struct DiagMpfdtiResultType_Tag
{
    uint32 zCBFETDiagResult;                /* CB FET Diagnostic Result 3 bytes */
    uint32 zDcompFltInj;                    /* SM_DCOMP_FLTINJ Digital comparison Fault Injection */
    uint32 zVcellACompFltInj;               /* SM_ACOMP_FLTINJ CELL Analog comparison Fault Injection */

    uint32 zGpioAcompFltInj:16;             /* SM_ACOMP_FLTINJ GPIO Analog comparison Fault Injection */

    uint32 zBG1Voltage:16;                  /* BG1 Voltage Value */
    uint32 zBG2Voltage:16;                  /* BG2 Voltage Value */
    uint32 zRefCapVoltage:16;               /* REF_CAP Voltage */

    uint16 zOvVoltage;                      /* OV Voltage Value */
    uint16 zUvVoltage;                      /* UV Voltage Value */
    uint16 zOtVoltage;                      /* OT Voltage Value */
    uint16 zUtVoltage;                      /* UT Voltage Value */

    uint32 zSM_CB_FAULT;                    /* CB Open Wire/CB FET Short threshold diagnostics */
    uint32 zSM_CBFET_OPN;                   /* SM_CBFET_DIAG - CB FET Diagnostic */
    uint32 zSM_CBOW_FLTINJ;                 /* CBOW FLTINJ Diagnostic */

    uint32 zSM_STARTUP:1;                   /* SM_STARTUP */
    uint32 zSM_BG1_DIAG:1;                  /* SM_BG1_DIAG - BG1 Reference Diagnostic */
    uint32 zSM_BG2_DIAG:1;                  /* SM_BG2_DIAG - BG2 Reference Diagnostic */
    uint32 zSM_REFCAP_DIAG:1;               /* SM_REFCAP_DIAG - REF_CAP Voltage Diagnostic */
    uint32 zSM_PWR_BIST:1;                  /* SM_PWR_BIST - Power Supply BIST */

    uint32 zSM_OV_DAC:1;                    /* UV Threshold DAC diagnostic */
    uint32 zSM_UV_DAC:1;                    /* OV Threshold DAC diagnostic */
    uint32 zSM_OT_DAC:1;                    /* OT Threshold DAC diagnostic */
    uint32 zSM_UT_DAC:1;                    /* UT Threshold DAC diagnostic */

    uint32 zSM_ACOMP_FLTINJ_VCELL:1;        /* SM_ACOMP_FLTINJ CELL Analog comparison Fault Injection */
    uint32 zSM_ACOMP_FLTINJ_GPIO:1;         /* SM_ACOMP_FLTINJ GPIO Analog comparison Fault Injection */
    uint32 zSM_DCOMP_FLTINJ:1;              /* SM_DCOMP_FLTINJ Digital comparison Fault Injection */
    uint32 zSM_OV_SC;                       /* OV Detection Single Channel */
    uint32 zSM_UV_SC;                       /* UV Detection Single Channel */
    uint16 zSM_OT_SC;                       /* OT Detection Single Channel */
    uint16 zSM_UT_SC;                       /* UT Detection Single Channel */
    uint16 zSM_OT_RR;                       /* OT Detection Round Robin */
    uint16 zSM_UT_RR;                       /* UT Detection Round Robin */
    uint32 Rsvd:20;                         /* RSVD */
    uint8 uSM_SLEEP_FAULT;                  /* Sleep Mode Fault Tone */
    uint8 uSM_SLEEP_HB;                     /* Sleep Mode Heartbeat */
    uint8 uSM_SLEEP_FSTHB;                  /* Fast heartbeat detection */
    uint8 uSM_PWR_BIST;                     /* Power Supply BIST */
    uint8 uDrdyPwrBist;
    uint8 uDrdyPwrBistErr;
    uint8 uSM_FLIP_RESET;                   /* Flip Reset ADC Dignostic */
}
DiagMpfdtiResultType;

typedef struct BQ7971X_DiagResultComb_Struct_Tag
{
    uint32  SM_AVDD_OV             :1;  /*  0:AVDD OV Detection */
    uint32  SM_AVDD_UV             :1;  /*  1:AVDD UV Detection */
    uint32  SM_AVDD_OSC            :1;  /*  2:AVDD OSC Detection */
    uint32  SM_DVDD_OV             :1;  /*  3:DVDD OV Detection */
    uint32  SM_DVDD_UV             :1;  /*  4:DVDD UV Detection */
    uint32  SM_TSREF_OV            :1;  /*  5:TSREF OV Detection */
    uint32  SM_TSREF_UV            :1;  /*  6:TSREF UV Detection */
    uint32  SM_TSREF_OSC           :1;  /*  7:TSREF OSC Detection */
    uint32  SM_NCPUMP_UV           :1;  /*  8:Neg CPUMP UV Detection */
    uint32  SM_VSS_OPEN            :1;  /*  9:VSS Open Detection */
    uint32  SM_REFVSS_OPEN         :1;  /* 10:REF_VSS Open Detection */
    uint32  SM_BG1_DIAG            :1;  /* 11:BG1 Reference Diagnostic */
    uint32  SM_BG2_DIAG            :1;  /* 12:BG2 Reference Diagnostic */
    uint32  SM_REFCAP_DIAG         :1;  /* 13:REF_CAP Voltage Diagnostic */
    uint32  SM_PWR_BIST            :1;  /* 14:Power Supply BIST */
    uint32  SM_BB_PLAU             :1;  /* 15:Bus Bar Input Plausibility */
    uint32  SM_CB_PLAU             :1;  /* CB Input Plausibility */
    uint32  SM_DRDY_VCELL          :1;  /* 16:Main ADC Data Ready status bit */
    uint32  SM_DRDY_DIAG           :1;  /* 17:ADC DIAG Data Ready status bit */
    uint32  SM_DRDY_GPIO           :1;  /* 18:GPIO ADC Data Ready status bit */
    uint32  SM_DRDY_DIAGD1D2       :1;  /* 19:GP ADC DIAG Data Ready status bit */
    uint32  SM_DRDY_ANA_VCELL      :1;  /* 20:Vcell Comp Data Ready status bit */
    uint32  SM_DRDY_ANA_GPIO       :1;  /* 21:GPIO Comp Data Ready status bit */
    uint32  SM_DRDY_DIG            :1;  /* 22:Digital Comp Data Ready status bit */
    uint32  SM_MAIN_PFAIL          :1;  /* 23:Main ADC Parity Fail Detection */
    uint32  SM_DIAG_PFAIL          :1;  /* 24:DIAG ADC Parity Fail Detection */
    uint32  SM_ANA_PFAIL           :1;  /* 25:ACOMP Parity Fail Detection */
    uint32  SM_DIAG_ABORT          :1;  /* 26:ADC COMP Diagnostic Abort */
    uint32  SM_FLIP_RESET          :1;  /* 27:Flip Reset ADC Dignostic */
    uint32  SM_DRDY_CBFETOW        :1;  /* 28:CB Open Wire diagnostic ready status bit */
    uint32  SM_BB_OPN_DET          :1;  /* 29:BB Open Wire threshold detection */
    uint32  SM_OV_DAC              :1;  /* 30:OV Threshold DAC diagnostic */
    uint32  SM_UV_DAC              :1;  /* 31:UV Threshold DAC diagnostic */
    uint32  SM_OT_DAC              :1;  /* 32:OT Threshold DAC diagnostic */
    uint32  SM_UT_DAC              :1;  /* 33:UT Threshold DAC diagnostic */
    uint32  SM_DEV_ADDR            :1;  /* 34:MCU Device Address Diagnostic */
    uint32  RSVD35                 :1;  /* 35:  */
    uint32  SM_NFAULT_DIAG         :2;  /* 36-37:NFAULT pin diagnostic */
    uint32  SM_COMM_BERR           :1;  /* 38:Byte error detection */
    uint32  SM_COMM_IERR           :1;  /* 39:IERR detection */
    uint32  SM_COMM_SOF            :1;  /* 40:Start of Frame error detection */
    uint32  SM_COMM_SYNC1          :1;  /* 41:Daisy Chain SYNC1 Error detection */
    uint32  SM_COMM_SYNC2          :1;  /* 42:Daisy Chain SYNC2 Error detection */
    uint32  SM_COMM_TIMEOUT        :1;  /* 43:Short Comm Timeout detection */
    uint32  SM_COMM_TXDIS          :1;  /* 44:TXDIS error detection */
    uint32  SM_COMM_UNEXP          :1;  /* 45:UNEXP communication detection */
    uint32  SM_COMM_WAIT           :1;  /* 46:WAIT error detection */
    uint32  SM_COMM_FLTDIAG_CRC    :1;  /* 47:MCU communication fault diagnostics */
    uint32  SM_COMM_FLTDIAG_INIT   :1;  /* 48:MCU communication fault diagnostics */
    uint32  SM_COMM_FLTDIAG_DELAY  :1;  /* 49:MCU communication fault diagnostics */
    uint32  SM_COMM_FLTDIAG_TXDIS  :1;  /* 50:MCU communication fault diagnostics */
    uint32  SM_SLEEP_FAULT         :1;  /* 51:Sleep Mode Fault Tone */
    uint32  SM_SLEEP_HB            :1;  /* 52:Sleep Mode Heartbeat */
    uint32  SM_SLEEP_FSTHB         :1;  /* 53:Fast heartbeat detection */
    uint32  SM_VIF_CRC             :1;  /* 54:Daisy Chain CRC Error detection */
    uint32  SM_VIF_BIT             :1;  /* 55:Daisy Chain BIT Error detection */
    uint32  SM_VIF_BYTE            :1;  /* 56:Daisy Chain BYTE Error detection */
    uint32  SM_VIF_FAULT           :1;  /* 57:Daisy Chain Fault Signal diagnostic */
    uint32  SM_VIF_CRCDIAG         :1;  /* 58:Daisy Chain CRC diagnostic */
    uint32  SM_UART_CCLR           :1;  /* 59:UART Comm Clear detection */
    uint32  SM_UART_STOPBIT        :1;  /* 60:UART STOP Bit Error detection */
    uint32  SM_LFOSC_FREQ          :1;  /* 61:LFOSC frequency mismatch detection */
    uint32  SM_DIE_TEMP            :2;  /* 62-63:Die Temp1/Temp2 plausibility */
    uint32  SM_DIE_TEMP_DIFF       :1;  /* 64:Die Temp1/Temp2 difference */
    uint32  SM_TWARN_DIE           :1;  /* 65:Thermal Warning */
    uint32  SM_TSHUT_STAT          :1;  /* 66:Thermal Shutdown Status */
    uint32  SM_HW_RST              :1;  /* 67:HW Reset Diagnostic */
    uint32  SM_CUST_CRC            :1;  /* 68:Customer Register CRC detection */
    uint32  SM_FACT_CRC            :1;  /* 69:Factory Register CRC detection */
    uint32  SM_CUSTCRC_STAT        :1;  /* 70:CUST CRC status bit */
    uint32  SM_FACTCRC_STAT        :1;  /* 71:FACT CRC status bit */
    uint32  SM_FACT_CRCDIAG        :1;  /* 72:FACT CRC diagnostic */
    uint32  SM_OTP_UNLCK           :1;  /* 73:OTP program unlock */
    uint32  SM_OTP_UNLCKSTAT       :1;  /* 74:OTP program unlock indication */
    uint32  SM_OTP_SUV_SOVERROR    :1;  /* 75:OTP Program Voltage Lock Out */
    uint32  SM_OTP_UVOVERROR       :1;  /* 76:OTP programming voltage error detection */
    uint32  SM_OTP_PERR            :1;  /* 77:OTP Program Error detection */
    uint32  SM_OTP_ECCDIAG         :2;  /* 78-79:OTP ECC detection */
    uint32  SM_OTP_LOADERR         :1;  /* 80:OTP Customer/Factory Load Error */
    uint32  SM_OTP_OVERR           :1;  /* 81:OTP OverVoltage Error */
    uint32  SM_FACT_TM             :1;  /* 82:Factory Testmode Detection */
    uint32  RSVD83                 :1;  /* 83: */
    uint32  SPI_FAULT_DIAG         :3;  /* 84-86: */
    uint32  RSVD87                 :1;  /* 87: */
    uint32  RSVD88_95              :8;  /* 88-95: */
} BQ7971X_DiagResultComb_Struct_Type;


typedef struct BQ7973X_DiagResult_Struct_Tag
{
    uint8                      SM_AVDD_OV;         /* AVDD OV Detection */
    uint8                      SM_AVDD_OSC;        /* AVDD OSC Detection */
    uint8                      SM_DVDD_OV;         /* DVDD OV Detection */
    uint8                      SM_DVDD_UV;         /* DVDD UV Detection */
    uint8                      SM_TSREF_OV;        /* TSREF OV Detection */
    uint8                      SM_TSREF_UV;        /* TSREF UV Detection */
    uint8                      SM_TSREF_OSC;       /* TSREF OSC Detection */
    uint8                      SM_CP_OV;           /* Charge Pump UV Detection */
    uint8                      SM_CP_UV;           /* Charge Pump OV Detection */
    uint8                      SM_REFVSS_OPEN;     /* REF_VSS Open Detection */
    uint8                      SM_VSS_OPEN;        /* AVSS Open Detection */
    uint8                      SM_BG1_DIAG;        /* BG1 Reference Diagnostic */
    uint8                      SM_BG2_DIAG;        /* BG2 Reference Diagnostic */
    uint8                      SM_REFCAP_DIAG;     /* REF_CAP Voltage Diagnostic */
    uint8                      SM_PWR_BIST;        /* Power Supply BIST */
    uint8                      SM_VF_PLAU;         /* VF Input Plausibility */
    uint8                      SM_VF_ACOMP;        /* VF Voltage Analog Comparison */
    uint8                      SM_VF_DCOMP;        /* VF Voltage Digital Comparison */
    uint16                     SM_GP_PLAU;         /* Vn_GPIOn Input Plausibility */
    uint16                     SM_GP_ACOMP;        /* Vn_GPIOn Analog Comparison */
    uint8                      SM_GP_DCOMP;        /* Vn_GPIOn Digital Comparison */
    uint8                      SM_DRDY_VF;         /* VF ADC Data Ready status bit */
    uint8                      SM_DRDY_GP;         /* GP ADC Data Ready status bit */
    uint8                      SM_DRDY_DIAGD1D2;   /* D1D2 ADC DIAG Data Ready status bit */
    uint8                      SM_DRDY_ANA_VF;     /* VF Comp Data Ready status bit */
    uint8                      SM_DRDY_ANA_GP;     /* GP Comp Data Ready status bit */
    uint8                      SM_DRDY_DIG;        /* Digital Comp Data Ready status bit */
    uint8                      SM_MAIN_PFAIL;      /* Main ADC Parity Fail Detection */
    uint8                      SM_DIAG_PFAIL;      /* DIAG ADC Parity Fail Detection */
    uint8                      SM_ANA_PFAIL;       /* ACOMP Parity Fail Detection */
    uint8                      SM_DIAG_ERR;        /* ADC COMP Diagnostic Error */
    uint8                      SM_VF_OPEN;         /* VF Open Wire detection */
    uint16                     SM_GP_OPEN;         /* Vn_GPIOn Open Wire detection */
    uint16                     SM_GP_ADJSHRT;      /* Vn_GPIOn Adjacent Short diagnostic */
    uint8                      SM_GPVF_ADJSHRT;    /* V15_GPIO15,VF2 Adjacent Short diagnostic */
    uint8                      SM_SW_MON;          /* SW Output Monitor (optional) */
    uint8                      SM_SW_ADJSHRT;      /* SW Adjacent Short diagnostic */
    uint8                      SM_DCOMP_FLTINJ_VF; /* VF Digital comparison Fault Injection */
    uint8                      SM_DCOMP_FLTINJ_GP; /* GP Digital comparison Fault Injection */
    uint8                      SM_ACOMP_FLTINJ_VF; /* VF Analog comparison Fault Injection */
    uint16                     SM_ACOMP_FLTINJ_GP; /* GP Analog comparison Fault Injection */
    uint8                      SM_FLIP_RESET;      /* Flip Reset ADC Dignostic */
    uint8                      SM_CS_PLAU;         /* Current Sense Input Plausibility */
    uint8                      SM_CS_COMP;         /* Current Sense Input Difference */
    uint8                      SM_DRDY_CS;         /* CS ADC Data Ready status bit */
    uint8                      SM_CS_OCC1;         /* Over Current Detection during charge */
    uint8                      SM_CS_OCC2;         /* Over Current Detection during charge */
    uint8                      SM_CS_OCD1;         /* Over Current Detection during discharge */
    uint8                      SM_CS_OCD2;         /* Over Current Detection during discharge */
    uint8                      SM_OC_PLAU;         /* Over Current Sense Input Plausibility */
    uint8                      SM_OC_COMP;         /* Over Current Sense Input Difference */
    uint8                      SM_OC_PFAIL;        /* OC ADC Parity Fail Detection */
    uint8                      SM_OC_ERR;          /* Over Current Detection Abort */
    uint8                      SM_OC_OS;           /* OC1, OC2 Output Open/Short */
    uint8                      SM_OC_UNLOCK;       /* Unlock Over Current Fault Enable Access */
    uint8                      SM_OC_DET_DIAG;     /* Over Current Detection diagnostic */
    uint8                      SM_NFAULT_DIAG;     /* NFAULT pin diagnostic */
    uint8                      SM_COMM_BERR;       /* Byte error detection */
    uint8                      SM_COMM_IERR;       /* IERR detection */
    uint8                      SM_COMM_SOF;        /* Start of Frame error detection */
    uint8                      SM_COMM_SYNC1;      /* Daisy Chain SYNC1 Error detection */
    uint8                      SM_COMM_SYNC2;      /* Daisy Chain SYNC2 Error detection */
    uint8                      SM_COMM_TIMEOUT;    /* Comm Timeout detection */
    uint8                      SM_COMM_TXDIS;      /* TXDIS error detection */
    uint8                      SM_COMM_UNEXP;      /* UNEXP communication detection */
    uint8                      SM_COMM_WAIT;       /* WAIT error detection */
    uint8                      SM_COMM_FLTDIAG_CRC;   /* MCU communication fault diagnostics */
    uint8                      SM_COMM_FLTDIAG_INIT;  /* MCU communication fault diagnostics */
    uint8                      SM_COMM_FLTDIAG_DELAY; /* MCU communication fault diagnostics */
    uint8                      SM_COMM_FLTDIAG_TXDIS; /* MCU communication fault diagnostics */
    uint8                      SM_SLEEP_FAULT;     /* Sleep Mode Fault Tone */
    uint8                      SM_SLEEP_HB;        /* Sleep Mode Heartbeat */
    uint8                      SM_SLEEP_FSTHB;     /* Fast heartbeat detection */
    uint8                      SM_VIF_CRC;         /* Daisy Chain CRC Error detection */
    uint8                      SM_VIF_BIT;         /* Daisy Chain BIT Error detection */
    uint8                      SM_VIF_BYTE;        /* Daisy Chain BYTE Error detection */
    uint8                      SM_VIF_CRCDIAG;     /* Daisy Chain CRC diagnostic */
    uint8                      SM_UART_STOPBIT;    /* UART STOP Bit Error detection */
    uint8                      SM_UART_CRC;        /* UART CRC Error detection */
    uint8                      SM_LFOSC_FREQ;      /* LFOSC frequency mismatch detection */
    uint8                      SM_DIE_TEMP;        /* Die Temp1/Temp2 plausibility */
    uint8                      SM_DIE_TEMP_DIFF;   /* Die Temp1/Temp2 difference */
    uint8                      SM_TSHUT_STAT;      /* Thermal Shutdown detection */
    uint8                      SM_CUST_CRC;        /* Customer Register CRC detection */
    uint8                      SM_FACT_CRC;        /* Factory Register CRC detection */
    uint8                      SM_FACTCRC_STAT;    /* FACT CRC status bit */
    uint8                      SM_FACT_CRCDIAG;    /* FACT CRC diagnostic */
    uint8                      SM_OTP_UNLCK;       /* OTP program unlock */
    uint8                      SM_OTP_UNLCKSTAT;   /* OTP program unlock indication */
    uint8                      SM_OTP_SUV_SOVERR;  /* OTP program voltage lock out */
    uint8                      SM_OTP_UVOVERR;     /* OTP programming voltage error detection */
    uint8                      SM_OTP_PERR;        /* OTP Program Error detection */
    uint8                      SM_OTP_ECC;         /* OTP ECC detection */
    uint8                      SM_OTP_ECCDIAG;     /* OTP ECC Internal diagnostic */
    uint8                      SM_OTP_LOAD_ERR;    /* OTP Customer/Factory Load Error */
    uint8                      SM_OTP_OVERR;       /* OTP Over Voltage Error */
    uint8                      SM_OTP_MARGIN;      /* OTP Read Margin Test */
    uint8                      SM_FACT_TM;         /* Factory Testmode Detection */
} BQ7973X_DiagResult_Struct_Type;

typedef struct DiagFdtiStatResultType_Tag
{
    uint16 zDieTemp1;                       /* Die Temp1 read value as PCB temp */
    uint16 zDieTemp2;                       /* Die Temp2 read value  */
    uint16 zDieTempPrev1;                   /* Die Temp1 Prev read value as PCB temp */
    uint16 zDieTempPrev2;                   /* Die Temp2 Prev read value  */

    uint32 zDieTemp1Stuck:8;                /* Die Temp1 and Temp2 are same */
    uint32 zDieTemp2Stuck:8;                /* Die Temp1 and Temp2 are same */
    uint32 zGpioPullSel:16;                 /* GPIO Voltage PULL SEL Configuration */

    uint32 zSM_VCELL_PLAU;                  /* CELL Voltage Plausibility, 0 for valid, 1 for invalid */
    uint32 zSM_VC_OPN_DET;                  /* SM_VC_OPN_DET Status */
    uint32 zSM_VCCB_SHRT;                   /* SM_VCCB_SHRT Status */
    uint32 zSM_CABLE_RES_AB;                /* CABLE Resistance */

    uint32 zSM_ACOMP_VCELL;                 /* SM_ACOMP CELL Analog comparison Fault Injection */
    uint32 zSM_ACOMP_GPIO;                  /* SM_ACOMP GPIO Analog comparison Fault Injection */
    uint32 zSM_DCOMP;                       /* SM_DCOMP Digital comparison Fault Injection */

    uint16 zSM_GPIO_PLAU;                   /* GPIO Plausibility, 0 for valid, 1 for invalid */
    uint16 zSM_GPIO_OPNWR;                  /* SM_GPIO_OPNWR, GPIO Open Wire detection */
    uint16 zSM_GPIO_ADJSHRT;                /* SM_GPIO_ADJSHRT, GPIO Adjacent Short diagnostic */

    uint32 zSM_FAULT_SUMMARY:8;             /* SM_FAULT_SUMMARY */
    uint32 zSM_DIE_TEMP1:1;                 /* SM_DIE_TEMP1 */
    uint32 zSM_DIE_TEMP2:1;                 /* SM_DIE_TEMP2 */
    uint32 zSM_DIE_TEMP_DIFF:1;             /* SM_DIE_TEMP_DIFF */
    uint32 zSM_NFAULT;                  /* NFAULT*/
    BQ7971X_DiagResultComb_Struct_Type Comb;
    BQ7973X_DiagResult_Struct_Type PMI_Comb;
    uint32 Rsvd:21;                         /* RSVD */
}
DiagFdtiResultType;

typedef union DiagMpfdtiStatType_Tag
{
    struct
    {
        uint32 sInit:4;                     /* Diag Startup Status */
        uint32 sBG1:4;                      /* BG1 Diag Status */
        uint32 sBG2:4;                      /* BG2 Diag  Status */
        uint32 sRefCap:4;                   /* Ref Diag Status */
        uint32 sPwrBist:4;                  /* Power Bist Status */
        uint32 sCbFet:4;                    /* Status of CB FET Diagnostic */

        uint32 sOvStat:4;                   /* Status of OV Diag */
        uint32 sUvStat:4;                   /* Status of UV Diag */
        uint32 sOtStat:4;                   /* Status of OT Diag */
        uint32 sUtStat:4;                   /* Status of UT Diag */

        uint32 sVcellAFltInjStat:4;         /* Status of VCELL ACOMP Fault Injection */
        uint32 sVcellACompStat:4;           /* Status of VCELL ACOMP Status */
        uint32 sGpioAFltInjStat:4;          /* Status of GPIO ACOMP Fault Injection */
        uint32 sGpioACompStat:4;            /* Status of GPIO ACOMP Status */
        uint32 sDCompFltInjStat:4;          /* Status of DCOMP Fault Injection */
        uint32 sDCompStat:4;                /* Status of DCOMP Status */
        uint32 sCbOwFltInjStat:4;           /* Status of CB OW Diagnostic */
    }
    zMpfdtiOut;
    uint32 qMpfdtiOut;
}
DiagMpfdtiStatType;

typedef union DiagFdtiStatType_Tag
{
    struct
    {
        uint32 sDiagStat:4;                 /* Status of ADC Status */
        uint32 sACompDiag:4;                /* Status of ACOMP DIAG */
        uint32 sDCompDiag:4;                /* Status of SM_DCOMP  */

        uint32 sVcPlau:4;                   /* VCELL PLAU Status */
        uint32 sVcOpen:4;                   /* VCELL Open Detection Status */
        uint32 sVccbShrt:4;                 /* VCCB SHORT Detection Status */

        uint32 sGpioPlau:4;                 /* GPIO PLAU Status */
        uint32 sGpioOpnWr:4;                /* GPIO OPNWR Execution Status */
        uint32 sGpioAdjShrt:4;              /* GPIO ADJSHRT Execution Status */

        uint32 sDieTemp:4;                  /* Status of Die Temperature */
        uint32 sDieTempDiff:4;              /* Status of Die Temperature Difference */
    }
    zFdtiOut;
    uint32 qFdtiOut;
}
DiagFdtiStatType;

typedef struct DiagStatusType_Tag
{
    DiagMpfdtiStatType zMpfdtiStat;
    DiagFdtiStatType zFdtiStat;
}
DiagStatusType;

typedef struct DiagResultType_Tag
{
    DiagMpfdtiResultType zMpfdtiResult;
    DiagFdtiResultType zFdtiResult;
}
DiagResultType;

typedef enum DiagServiceType_PMI_Tag
{
    PMI_MPFDI_STARTUPINIT       = 0u,
    PMI_MPFDI_BG1_DIAG          = 1u,
    PMI_MPFDI_BG2_DIAG          = 2u,
    PMI_MPFDI_ACOMP_FLTINJ_DIAG = 3u,
    PMI_MPFDI_DCOMP_FLTINJ_DIAG = 4u,
    PMI_MPFDI_OC_UNLOCK_DIAG    = 5u,
    PMI_MPFDI_OC_DET_DIAG       = 6u,
    PMI_MPFDI_SW_ADJ_DIAG       = 7u,
    PMI_MPFDI_REFCAP_DIAG       = 8u,
    PMI_MPFDI_SW_OTP_PROG       = 9u,
    PMI_MPFDI_VIF_CRC_DIAG      = 10u,
    PMI_MPFDI_SLEEP_FAULT_DIAG  = 11u,
    PMI_MPFDI_FACT_CRC_DIAG     = 12u,
    PMI_MPFDI_PWR_BIST_DIAG     = 13u,
    PMI_MFDI_FLIP_RESET_DIAG    = 14u,
    
    PMI_FDTI_CYCLE_INIT         = 15u,           /* SM_CYCLE INIT - Checking Diagnostics Status Every Cycle */
    PMI_FDTI_ACOMP_DIAG         = 16u,
    PMI_FDTI_DCOMP_DIAG         = 17u,
    PMI_FDTI_GP_PLAU_DIAG       = 18u,
    PMI_FDTI_FAULT_ADC_MISC     = 19u,
    PMI_FDTI_GPIO_OPNWR_DIAG    = 20u,
    PMI_FDTI_GPIO_ADJSHRT       = 21u,
    PMI_FDTI_UPDATE_CUST_CRC    = 22u,
    PMI_FDTI_FLT_OTC_DIAG       = 23u,
    PMI_FDTI_CS_PLAU_DIAG       = 24u,
    PMI_FDTI_FAULT_PWR_DIAG     = 25u,
    PMI_FDTI_FAULT_COMM_DIAG    = 26u,
    PMI_FDTI_SW_MON_DIAG        = 27u,
    PMI_FDTI_OC_PLAU_DIAG       = 28u,
    PMI_FDTI_COMM_FLT_MSK_DIAG  = 29u,
    PMI_FDTI_NFAULT             = 30u,
    PMI_FDTI_DIETMEP_DIAG       = 31u,
    PMI_FDTI_FACTORY_CRC_DIAG   = 32u,
    PMI_FDTI_COMM_DEBUG_DIAG    = 33u,
    PMI_FDTI_FAULT_SYS_DIAG     = 34u,
    PMI_FDTI_FAULT_OTP_DIAG     = 35u,
    PMI_FDTI_OTP_STATE_DIAG     = 36u,
    PMI_FDTI_VF_OPEN_DIAG       = 37u,
    PMI_FDTI_VF_PLAU_DIAG       = 38u,
    PMI_FDTI_GET_CS1_VREF       = 39u,
    PMI_FDTI_INSULAT_DET        = 40u,
    PMI_FDTI_READ_SPI_DATA      = 41u,
    PMI_FDTI_SET_VISYNC         = 42u,
    PMI_FDTI_GET_VAVDD          = 43u,
    PMI_FDTI_DRY_VF_DIAG        = 44u,
    PMI_FDTI_DRY_GPIO_DIAG      = 45u,

    PMI_FDTI_FACT_TM_DIAG       = 46u,
    PMI_FDTI_OTP_MARGIN_DIAG    = 47u,

    //TBD , This Service is not a Diag Service
    PMI_FDTI_DRY_DIG_DIAG       = 48u,
    PMI_FDTI_DRY_DIG_D1_D2_DIAG = 49u,
    PMI_FDTI_DRY_CS_DIAG        = 50u,
    PMI_FDTI_DRDY_ANA_VF_DIAG   = 51u,
    PMI_FDTI_DRDY_ANA_GPIO_DIAG = 52u,
    PMI_FDTI_DRDY_VF_DIAG       = 53u,


    PMI_DIAGSERVICE_END
}PMI_DiagServiceType;


typedef enum DiagServiceType_Tag
{

    BMI_SM_MPFDI_STARTUPINIT                = (5u ),          /* SM_DCOMP_INIT - Initialization of BMC Comparison */
    BMI_SM_MPFDI_BGX_DIAG                   = (6u ),          /* SM_BG1_DIAG, SM_BG2_DIAG - BG1, BG2 Reference Diagnostic*/
    BMI_SM_MPFDI_REFCAP_DIAG                = (7u ),          /* SM_REFCAP_DIAG - REF_CAP Voltage Diagnostic*/
    BMI_SM_MPFDI_PWR_BIST                   = (8u ),          /* SM_PWR_BIST - Power Supply BIST*/
    BMI_SM_MPFDI_ACOMP_FLTINJ               = (9u ),          /* SM_ACOMP_FLTINJ - Analog comparison Fault Injection */
    BMI_SM_MPFDI_DCOMP_FLTINJ               = (10u),         /* SM_DCOMP_FLTINJ - Digital comparison Fault Injection */
    BMI_SM_MPFDI_CBFET_DIAG                 = (11u),         /* SM_CBFET_DIAG - CB FET Diagnostic */
    BMI_SM_MPFDI_OV_DAC                     = (12u),         /* SM_OV_DAC - OV/UV Threshold DAC diagnostic */
    BMI_SM_MPFDI_UV_DAC                     = (13u),         /* SM_UV_DAC - OV/UV Threshold DAC diagnostic */
    BMI_SM_MPFDI_OT_DAC                     = (14u),         /* SM_OT_DAC - OT OT_CB Threshold DAC diagnost */
    BMI_SM_MPFDI_UT_DAC                     = (15u),         /* SM_UT_DAC - OT/UT Threshold DAC diagnostic */
    BMI_SM_CBOW_FLTINJ                      = (16u),         /* SM_CBOW_FLTINJ - Multipoint fault injection for CBFET OW Check */
    BMI_SM_MPFDI_OTP_PROGRAM                = (17u),
    BMI_SM_MPFDI_OTUT_SC_DIAG               = (18u),
    BMI_SM_MPFDI_OVUV_SC_DIAG               = (19u),
    BMI_SM_MPFDI_OTUT_RR_DIAG               = (20u),
    BMI_SM_MPFDI_SLEEP_FAULT_DIAG           = (21u),
    BMI_SM_MPFDI_HW_RESET_DIAG              = (22u),
    BMI_SM_MPFDI_DEV_ADDR_DIAG              = (23u),

    BMI_SM_FDTI_CYCLE_INIT                  = (18u+6u),         /* SM_CYCLE INIT - Checking Diagnostics Status Every Cycle */
    BMI_SM_FDTI_GPIO_DIAG                   = (19u+6u),         /* SM_GPIO_OPNWR and SM_GPIO_ADJSHRT - GPIO Open Wire/Adjshort detection */
    BMI_SM_FDTI_GPIO_OPNWR                  = (20u+6u),         /* SM_GPIO_OPNWR - GPIO Open Wire detection */
    BMI_SM_FDTI_GPIO_ADJSHRT                = (21u+6u),         /* SM_GPIO_ADJSHRT - GPIO Adjacent Short diagnostic */
    BMI_SM_FDTI_VC_OPN_DET                  = (22u+6u),         /* SM_VC_OPN_DET - VC Open Wire threshold detection */
    BMI_SM_FDTI_VCCB_SHRT                   = (23u+6u),         /* SM_VCCB_SHRT - CBn to VCn Short Diagnostic */
    BMI_SM_FDTI_VCELLGPIO_ACOMP             = (24u+6u),         /* SM_VCELL_ACOMP, SM_GPIO_ACOMP - VCELL/GPIO Analog Comparison*/
    BMI_SM_FDTI_DCOMP_DIAG                  = (25u+6u),         /* SM_DCOMP, SM_GPIO_DCOMP - ADC Digital Comparison*/
    BMI_SM_FDTI_ADC_COMP_DIAG               = (26u+6u),         /* SM_DCOMP, SM_GPIO_DCOMP - ADC Digital Comparison*/
    BMI_SM_FDTI_FAULT_SUMMARY               = (27u+6u),         /* SM_FAULT_SUMMARY - Checking Fault Summary */
    BMI_SM_FDTI_DIETMEP_DIAG                = (28u+6u),         /* SM_DCOMP, SM_GPIO_DCOMP - ADC Digital Comparison*/
    BMI_SM_FTDI_NFAULT                      = (29u+6u),          /* SM  NFAULT pin diagnostic */
    BMI_SM_COMM_FAULT                       = (30u+6u),          /* SM  communication fault diagnostics */
    BMI_SM_ECC_DIAG                         = (31u+6u),          /* SM  communication fault diagnostics */
    BMI_SM_COMM_FLT_MSK_DIAG                = (32u+6u),          /* SM  verify the register bit setting */
    BMI_SM_FACTORY_CRC_DIAG                 = (33u+6u),          /* SM  factory CRC diagnostic. */
    BMI_SM_VLF_CRC_DIAG                     = (34u+6u),          /* SM  daisy chain CRC diagnostic */
    BMI_SM_SPI_FAULT_DIAG                   = (35u+6u),          /* SM  Check the sending and receiving functions of SPI. */
    BMI_SM_FAULT_ADC_MISC                   = (36u+6u),          /* SM FAULT_ADC_MISC diagnosis.. */
    BMI_SM_VLF_FAULT_DIAG                   = (37u+6u),          /* SM  daisy chain CRC diagnostic */
    BMI_SM_GET_DIAG_D1                      = (38u+6u),
    BMI_SM_GET_DIAG_D2                      = (39u+6u),

    BMI_SET_SOFT_RESET                      = (40u+6u),
	BMI_SET_HW_RESET                        = (41u+6u),
	BMI_GOTO_SHUTDOWN                       = (42u+6u),
	BMI_SET_DEV_SLEEP                       = (43u+6u),
	BMI_SET_DEV_SHUT2WAKEUP                 = (44u+6u),
	BMI_SET_DEV_SLP2WAKEUP                  = (45u+6u),
	BMI_DIAG_ABORT_DIAG                     = (46u+6u),
	BMI_SM_BB_PLAU_DIAG                     = (47u+6u),
	BMI_SM_CB_PLAU_DIAG                     = (48u+6u),
	BMI_SM_FACR_TM                          = (49u+6u),
	BMI_SM_FAULT_OTP_ERR_DIAG               = (50u+6u),
	BMI_SM_OTP_STATE_DIAG                   = (51u+6u),
	BMI_SM_CRC_STATE_DIAG                   = (52u+6u),
	BMI_FDTI_FAULT_SYS_DIAG                 = (53u+6u),
    BMI_FDTI_FLIP_RESET_DIAG                = (54u+6u),
	BMI_SM_FTDI_COMM_DEBUG_DIAG             = (55u+6u),
	BMI_SM_GET_DIAG_MAIN_RDNT               = (56u+6u),
	BMI_SM_FAULT_PWR_DIAG                   = (57u+6u),
    BMI_DIAGSERVICE_END
}
DiagServiceType;

typedef struct PMIDiagStatusType_Tag
{
    DiagMpfdtiStatType zMpfdtiStat;
    DiagFdtiStatType zFdtiStat;
}
DiagPmiStatusType;

typedef struct PMIDiagResultType_Tag
{
    DiagMpfdtiResultType zPMIMpfdtiResult;
    DiagPMIFdtiResultType zPMIFdtiResult;
}
DiagPmiResultType;

typedef struct Std_CddVersionInfoType_Tag
{
    uint16 vendorID;
    uint16 moduleID;
    uint8 instanceID;
    uint8 sw_major_version;
    uint8 sw_minor_version;
    uint8 sw_patch_version;
}
Std_CddVersionInfoType;



/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

FUNC(uint8, tibms_bmi_CODE) Bmi_DiagServiceRequest(uint8 uBmiIfaceIn, uint8 uCmd, void *pData , uint8 uChannel);
FUNC(uint8, tibms_bmi_CODE) Pmi_DiagServiceRequest(uint8 uPmiIfaceIn, uint8 uCmd, void *pData);
/*********************************************************************************************************************
 * Exported Inline Function Definitions and Function-Like Macros
 *********************************************************************************************************************/

// FDTI Routines
#define Bmi_DiagCycleInit(uIface)                  Bmi_DiagServiceRequest(uIface, BMI_SM_FDTI_CYCLE_INIT, NULL, 0)
#define Bmi_GpioDiag(uIface)                       Bmi_DiagServiceRequest(uIface, BMI_SM_FDTI_GPIO_DIAG, NULL, 0)
#define Bmi_GpioOpnWrDiag(uIface)                  Bmi_DiagServiceRequest(uIface, BMI_SM_FDTI_GPIO_OPNWR, NULL, 0)
#define Bmi_GpioAdjShrtDiag(uIface)                Bmi_DiagServiceRequest(uIface, BMI_SM_FDTI_GPIO_ADJSHRT, NULL, 0)
#define Bmi_VcOpenDiag(uIface)                     Bmi_DiagServiceRequest(uIface, BMI_SM_FDTI_VC_OPN_DET, NULL, 0)
#define Bmi_VCCBShrtDiag(uIface)                   Bmi_DiagServiceRequest(uIface, BMI_SM_FDTI_VCCB_SHRT, NULL, 0)
#define Bmi_AcompDiag(uIface)                      Bmi_DiagServiceRequest(uIface, BMI_SM_FDTI_VCELLGPIO_ACOMP, NULL, 0)
#define Bmi_DcompDiag(uIface)                      Bmi_DiagServiceRequest(uIface, BMI_SM_FDTI_DCOMP_DIAG, NULL, 0)
#define Bmi_FaultSummary(uIface)                   Bmi_DiagServiceRequest(uIface, BMI_SM_FDTI_FAULT_SUMMARY, NULL, 0)
#define Bmi_DieTempDiag(uIface)                    Bmi_DiagServiceRequest(uIface, BMI_SM_FDTI_DIETMEP_DIAG, NULL, 0)
#define Bmi_GetDiagD1(uIface)                      Bmi_DiagServiceRequest(uIface, BMI_SM_GET_DIAG_D1, NULL, 0)
#define Bmi_GetDiagD2(uIface)                      Bmi_DiagServiceRequest(uIface, BMI_SM_GET_DIAG_D2, NULL, 0)
                            /* 8.2 */
#define Bmi_DiagAbortDiag(uIface)                  Bmi_DiagServiceRequest(uIface, BMI_DIAG_ABORT_DIAG, NULL, 0)
#define Bmi_FaultAdcMiscDiag(uIface)               Bmi_DiagServiceRequest(uIface, BMI_SM_FAULT_ADC_MISC, NULL, 0)
#define Bmi_SPIFaultDiag(uIface)                   Bmi_DiagServiceRequest(uIface, BMI_SM_SPI_FAULT_DIAG, NULL, 0)
#define Bmi_BBOpenDiag(uIface)                     Bmi_DiagServiceRequest(uIface, BMI_SM_FDTI_VC_OPN_DET, NULL, 0)
#define Bmi_CbOpenDetDiag(uIface)                  Bmi_DiagServiceRequest(uIface, BMI_SM_FDTI_VCELLGPIO_ACOMP, NULL, 0)
#define Bmi_VIFCrcDiag(uIface)                     Bmi_DiagServiceRequest(uIface, BMI_SM_VLF_CRC_DIAG, NULL, 0)
#define Bmi_NfaultDiag(uIface)                     Bmi_DiagServiceRequest(uIface, BMI_SM_FTDI_NFAULT, NULL, 0)
#define Bmi_CommDebugDiag(uIface)                  Bmi_DiagServiceRequest(uIface, BMI_SM_FTDI_COMM_DEBUG_DIAG, NULL, 0)
#define Bmi_BBPlauDiag(uIface)                     Bmi_DiagServiceRequest(uIface, BMI_SM_BB_PLAU_DIAG, NULL, 0)
#define Bmi_CBPlauDiag(uIface , uChannel)          Bmi_DiagServiceRequest(uIface, BMI_SM_CB_PLAU_DIAG, NULL, uChannel)
#define Bmi_GetDiagMainRdnt(uIface,uChannel)       Bmi_DiagServiceRequest(uIface, BMI_SM_GET_DIAG_MAIN_RDNT,  NULL,uChannel)
#define Bmi_FaultPwrDiag(uIface)                   Bmi_DiagServiceRequest(uIface, BMI_SM_FAULT_PWR_DIAG, NULL, 0)
                           /* 9.1   */
#define Bmi_FactTmDiag(uIface)                     Bmi_DiagServiceRequest(uIface, BMI_SM_FACR_TM, NULL, 0)
#define Bmi_OtpErrDiag(uIface)                     Bmi_DiagServiceRequest(uIface, BMI_SM_FAULT_OTP_ERR_DIAG, NULL, 0)
#define Bmi_OtpStatDiag(uIface)                    Bmi_DiagServiceRequest(uIface, BMI_SM_OTP_STATE_DIAG, NULL, 0)
#define Bmi_FactCrcDiag(uIface)                    Bmi_DiagServiceRequest(uIface, BMI_SM_FACTORY_CRC_DIAG, NULL, 0)
#define Bmi_CrcStatDiag(uIface)                    Bmi_DiagServiceRequest(uIface, BMI_SM_CRC_STATE_DIAG, NULL, 0)
#define Bmi_FaultSysDiag(uIface)                   Bmi_DiagServiceRequest(uIface, BMI_FDTI_FAULT_SYS_DIAG, NULL, 0)
#define Bmi_FlipResetDiag(uIface)                  Bmi_DiagServiceRequest(uIface, BMI_FDTI_FLIP_RESET_DIAG, NULL, 0)



// MPFDI Routines
#define Bmi_BGxDiag(uIface)                        Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_BGX_DIAG, NULL, 0)
#define Bmi_RefCapDiag(uIface)                     Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_REFCAP_DIAG, NULL, 0)
#define Bmi_PwrBistDiag(uIface)                    Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_PWR_BIST, NULL, 0)
#define Bmi_AcompFltinjDiag(uIface)                Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_ACOMP_FLTINJ, NULL, 0)
#define Bmi_DcompFltinjDiag(uIface)                Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_DCOMP_FLTINJ, NULL, 0)
#define Bmi_CbfetDiag(uIface)                      Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_CBFET_DIAG, NULL, 0)
#define Bmi_OvDacDiag(uIface)                      Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_OV_DAC, NULL, 0)
#define Bmi_UvDacDiag(uIface)                      Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_UV_DAC, NULL, 0)
#define Bmi_OtDacDiag(uIface)                      Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_OT_DAC, NULL, 0)
#define Bmi_UtDacDiag(uIface)                      Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_UT_DAC, NULL, 0)
#define Bmi_CBOW_FltInjDiag(uIface)                Bmi_DiagServiceRequest(uIface, BMI_SM_CBOW_FLTINJ, NULL, 0)
/*  8.2 */
#define Bmi_OTPProgram(uIface)                     Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_OTP_PROGRAM, NULL)
/* 9.1 */
#define Bmi_OtUtSCDiag(uIface , uChannel)         Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_OTUT_SC_DIAG, NULL, uChannel)
#define Bmi_OvUvSCDiag(uIface , uChannel)         Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_OVUV_SC_DIAG, NULL, uChannel)
#define Bmi_OtUtRRDiag(uIface)                    Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_OTUT_RR_DIAG, NULL, 0)
#define Bmi_SleepFaultDiag(uIface)                Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_SLEEP_FAULT_DIAG, NULL, 0)
#define Bmi_HWResetDiag(uIface)                   Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_HW_RESET_DIAG, NULL, 0)
#define Bmi_DevAddrDiag(uIface)                   Bmi_DiagServiceRequest(uIface, BMI_SM_MPFDI_DEV_ADDR_DIAG, NULL, 0)

// MPFDI Routines
#define Pmi_AcompFltinjDiag(uIface)                Pmi_DiagServiceRequest(uIface, PMI_MPFDI_ACOMP_FLTINJ_DIAG, NULL)
#define Pmi_DcompFltinjDiag(uIface)                Pmi_DiagServiceRequest(uIface, PMI_MPFDI_DCOMP_FLTINJ_DIAG, NULL)
#define Pmi_Bg1Diag(uIface)                        Pmi_DiagServiceRequest(uIface, PMI_MPFDI_BG1_DIAG, NULL)
#define Pmi_Bg2Diag(uIface)                        Pmi_DiagServiceRequest(uIface, PMI_MPFDI_BG2_DIAG, NULL)
#define Pmi_RefCapDiag(uIface)                     Pmi_DiagServiceRequest(uIface, PMI_MPFDI_REFCAP_DIAG, NULL)
#define Pmi_OCUnlockDiag(uIface)                   Pmi_DiagServiceRequest(uIface, PMI_MPFDI_OC_UNLOCK_DIAG, NULL)
#define Pmi_OCDetDiag(uIface)                      Pmi_DiagServiceRequest(uIface, PMI_MPFDI_OC_DET_DIAG, NULL)
#define Pmi_SWAdjShortDiag(uIface)                 Pmi_DiagServiceRequest(uIface, PMI_MPFDI_SW_ADJ_DIAG, NULL)
#define Pmi_OTPProgram(uIface)                     Pmi_DiagServiceRequest(uIface, PMI_MPFDI_SW_OTP_PROG, NULL)
#define Pmi_VFPlauDiag(uIface)                     Pmi_DiagServiceRequest(uIface, PMI_FDTI_VF_PLAU_DIAG, NULL)


// PMI FDTI
#define Pmi_AcompDiag(uIface)                      Pmi_DiagServiceRequest(uIface, PMI_FDTI_ACOMP_DIAG, NULL)
#define Pmi_DcompDiag(uIface)                      Pmi_DiagServiceRequest(uIface, PMI_FDTI_DCOMP_DIAG, NULL)
#define Pmi_CommDebugDiag(uIface)                  Pmi_DiagServiceRequest(uIface, PMI_FDTI_COMM_DEBUG_DIAG, NULL)
#define Pmi_CSPlauDiag(uIface)                     Pmi_DiagServiceRequest(uIface, PMI_FDTI_CS_PLAU_DIAG, NULL)
#define Pmi_DieTempDiag(uIface)                    Pmi_DiagServiceRequest(uIface, PMI_FDTI_DIETMEP_DIAG, NULL)
#define Pmi_FactCrcStatDiag(uIface)                Pmi_DiagServiceRequest(uIface, PMI_FDTI_FACTORY_CRC_DIAG, NULL)
#define Pmi_FaultAdcMiscDiag(uIface)               Pmi_DiagServiceRequest(uIface, PMI_FDTI_FAULT_ADC_MISC, NULL)
#define Pmi_FaultCommDiag(uIface)                  Pmi_DiagServiceRequest(uIface, PMI_FDTI_FAULT_COMM_DIAG, NULL)
#define Pmi_FaultOCDiag(uIface)                    Pmi_DiagServiceRequest(uIface, PMI_FDTI_FLT_OTC_DIAG, NULL)
#define Pmi_FaultOtpDiag(uIface)                   Pmi_DiagServiceRequest(uIface, PMI_FDTI_FAULT_OTP_DIAG, NULL)
#define Pmi_FaultPwrDiag(uIface)                   Pmi_DiagServiceRequest(uIface, PMI_FDTI_FAULT_PWR_DIAG, NULL)
#define Pmi_FaultSysDiag(uIface)                   Pmi_DiagServiceRequest(uIface, PMI_FDTI_FAULT_SYS_DIAG, NULL)
#define Pmi_GpioAdjShortDiag(uIface)               Pmi_DiagServiceRequest(uIface, PMI_FDTI_GPIO_ADJSHRT, NULL)
#define Pmi_GpioOpenDiag(uIface)                   Pmi_DiagServiceRequest(uIface, PMI_FDTI_GPIO_OPNWR_DIAG, NULL)
#define Pmi_GPPlauDiag(uIface)                     Pmi_DiagServiceRequest(uIface, PMI_FDTI_GP_PLAU_DIAG, NULL)
#define Pmi_NfaultDiag(uIface)                     Pmi_DiagServiceRequest(uIface, PMI_FDTI_NFAULT, NULL)
#define Pmi_OCPlauDiag(uIface)                     Pmi_DiagServiceRequest(uIface, PMI_FDTI_OC_PLAU_DIAG, NULL)
#define Pmi_OtpStatDiag(uIface)                    Pmi_DiagServiceRequest(uIface, PMI_FDTI_OTP_STATE_DIAG, NULL)
#define Pmi_SWMonDiag(uIface)                      Pmi_DiagServiceRequest(uIface, PMI_FDTI_SW_MON_DIAG, NULL)
#define Pmi_UpdateCustCrc(uIface)                  Pmi_DiagServiceRequest(uIface, PMI_FDTI_UPDATE_CUST_CRC, NULL)
#define Pmi_VFOpenDiag(uIface)                     Pmi_DiagServiceRequest(uIface, PMI_FDTI_VF_OPEN_DIAG, NULL)
#define Pmi_GetCs1Vref(uIface)                     Pmi_DiagServiceRequest(uIface, PMI_FDTI_GET_CS1_VREF, NULL)
#define Pmi_InsulationDet(uIface)                  Pmi_DiagServiceRequest(uIface, PMI_FDTI_INSULAT_DET, NULL)
#define Pmi_ReadSPIData (uIface)                   Pmi_DiagServiceRequest(uIface, PMI_FDTI_READ_SPI_DATA, NULL)
#define Pmi_SetVIsync(uIface)                      Pmi_DiagServiceRequest(uIface,PMI_FDTI_SET_VISYNC, NULL)
#define Pmi_GetVAvdd(uIface)                       Pmi_DiagServiceRequest(uIface,PMI_FDTI_GET_VAVDD, NULL)

/*   9.1   */
#define Pmi_FactTmDiag(uIface)                     Pmi_DiagServiceRequest(uIface,PMI_FDTI_FACT_TM_DIAG, NULL)
#define Pmi_OtpMarginDiag(uIface)                  Pmi_DiagServiceRequest(uIface,PMI_FDTI_OTP_MARGIN_DIAG, NULL)
#define Pmi_VIFCrcDiag(uIface)                     Pmi_DiagServiceRequest(uIface,PMI_MPFDI_VIF_CRC_DIAG, NULL)
#define Pmi_SleepFaultDiag(uIface)                 Pmi_DiagServiceRequest(uIface,PMI_MPFDI_SLEEP_FAULT_DIAG, NULL)
#define Pmi_FactCrcDiag(uIface)                    Pmi_DiagServiceRequest(uIface,PMI_MPFDI_FACT_CRC_DIAG, NULL)
#define Pmi_PwrBistDiag(uIface)                    Pmi_DiagServiceRequest(uIface,PMI_MPFDI_PWR_BIST_DIAG, NULL)
#define Pmi_FlipResetDiag(uIface)                  Pmi_DiagServiceRequest(uIface,PMI_MFDI_FLIP_RESET_DIAG, NULL)


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /*TIBMS_DIAG_H*/

/*********************************************************************************************************************
 * End of File: tibms_diag.h
 *********************************************************************************************************************/
