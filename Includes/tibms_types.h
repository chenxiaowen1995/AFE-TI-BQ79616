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
 *  File:       tibms_types.h
 *  Project:    TIBMS
 *  Module:     CONFIG
 *  Generator:  Code generation tool (if any)
 *
 *  Description:  TI BMS type definitions
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

#ifndef TIBMS_TYPES_H
#define TIBMS_TYPES_H

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

/**
 * Macro for setting value to 1 of the bit given by the mask.
 *
 * @param reg Register to be modified.
 * @param mask Bit selection in register.
 */
#define TIBMS_SET_BIT_VALUE(val, mask) ((val) | (mask))

/**
 * Macro for setting value to 0 of the bit given by the mask.
 *
 * @param reg Register to be modified.
 * @param mask Bit selection in register.
 */
#define TIBMS_UNSET_BIT_VALUE(val, mask) ((val) & ~(mask))

/**
 * Macro for setting value of bits given by the mask.
 *
 * @param reg Register value to be modified.
 * @param mask Bits selection in register.
 * @param shift Bits shift in register.
 * @param val Value to be applied.
 * @param range Admissible range of value.
 */
#define TIBMS_SET_BITS_VALUE(reg, mask, shift, val, range) (((reg) & ~(mask)) | (((val) & (range)) << (shift)))

/**
 * Macro for getting value of bits given by the mask.
 *
 * @param reg Register to be read.
 * @param mask Bits selection in register.
 * @param shift Bits shift in register.
 */
#define TIBMS_GET_BITS_VALUE(val, mask, shift)  (((val) & (mask)) >> (shift))

#define TIBMS_GetValue(typ, buf)                (*((typ *)buf))
#define TIBMS_SetValue(typ, buf, value)         (*((typ *)buf)) = (typ) value

#define TIBMS_ALIGN(x, b)                       (((x) + (b) - 1u) & ~((b) - 1u))
#define TIBMS_BYTE_ALIGN(x)                     TIBMS_ALIGN(x, 8u)

#define TIBMS_BitSet(arg, pos)                  ((arg) |= (1UL << (pos)))
#define TIBMS_BitClear(arg, pos)                ((arg) &= ~(1UL << (pos)))
#define TIBMS_BitBool(x)                        (!(!(x)))
#define TIBMS_BitTest(arg, pos)                 ((arg) & (1UL << (pos)))
#define TIBMS_BitFlip(arg, pos)                 ((arg) ^ (1UL << (pos)))
#define TIBMS_BitFlagInclude(arg, flag)         (arg |= flag)
#define TIBMS_BitFlagExclude(arg, flag)         (arg &= (~flag))

#define TIBMS_LoPart8(x)                        ((uint8)(x) & 0x0F)
#define TIBMS_HiPart8(x)                        (((uint8)(x) & 0xF0) >> 4)
#define TIBMS_LoPart16(x)                       ((uint8)(x))
#define TIBMS_HiPart16(x)                       ((uint8)((VosByte2)(x) >> 8))
#define TIBMS_LoPart32(x)                       ((uint16)(x))
#define TIBMS_HiPart32(x)                       ((uint16)((VosByte4)(x) >> 16))

#define itemsInArray(array)                     (sizeof(array)/sizeof*(array))
#define max(a,b)                                (((a) > (b)) ? (a) : (b))
#define min(a,b)                                (((a) < (b)) ? (a) : (b))

#define TIBMS_LIB_EXPORT                        extern

#define TIBMS_API                               (TIBMS_LIB_EXPORT)

/*********************************************************************************************************************
 * Exported Preprocessor #define Constants
 *********************************************************************************************************************/

#ifndef E_OK
#define E_OK                                    (0x00U)
#endif

#ifndef E_NOT_OK
#define E_NOT_OK                                (0x01U)
#endif

#ifndef E_PENDING
#define E_PENDING                               (0x02U)
#endif

#ifndef STD_ON
#define STD_ON                                  (0x01U)
#endif

#ifndef STD_OFF
#define STD_OFF                                 (0x00U)
#endif

#ifndef NULL
#define NULL                                    ((void *) 0x00U)
#endif

#ifndef NULL_PTR
#define NULL_PTR                                ((void *) 0x00U)
#endif

#ifndef TRUE
#define TRUE                                    (0x01u)
#endif

#ifndef FALSE
#define FALSE                                   (0x00u)
#endif

#define TIBMS_8BIT_MASK                         (0xFFu)
#define TIBMS_16BIT_MASK                        (0xFFFFu)
#define TIBMS_32BIT_MASK                        (0xFFFFFFFFu)

#define TIBMS_0BIT_OFFSET                       (0x00u)
#define TIBMS_8BIT_OFFSET                       (0x08u)
#define TIBMS_16BIT_OFFSET                      (0x10u)
#define TIBMS_24BIT_OFFSET                      (0x18u)

#define TIBMS_NO_ENDIAN                         (0x00u)
#define TIBMS_SML_ENDIAN                        (0x01u)
#define TIBMS_BIG_ENDIAN                        (0x02u)

#define TIBMS_INVALID_IFACE                     (0xFFu)

/*********************************************************************************************************************
 * Exported Type Declarations
 *********************************************************************************************************************/

typedef struct Comif_ManagerType_Tag Comif_ManagerType;
typedef struct InterfaceCfgType_Tag InterfaceCfgType;
typedef struct ServiceCfgType_Tag ServiceCfgType;
typedef struct ServiceStatusType_Tag ServiceStatusType;
typedef struct Emem_StasticsType_Tag Emem_StasticsType;

typedef void (*ApplCbType)(uint8 uIface, uint8 uServiceId, uint8 uSubId, uint8 uStatus, void *pRxBuf);
typedef uint8 (*ServiceCbType)(const ServiceCfgType *pServiceReq, const uint8 *pRxData, uint8 uRespStatus);

typedef enum RespStateType_Tag
{
    RESP_INVALID = 0u,
    RESP_BUSY,
    RESP_VALID,
    RESP_DECODED,
    RESP_USED
}
RespStateType;

struct ServiceStatusType_Tag
{
    uint32 cReqTimeMs;
    uint16 nReqMsgs;
    uint16 nRespMsgs;

    uint8 uInfo;
    uint8 uRespStatus;
    uint8 uFailure;
    uint8 cRespMaxTimeMs;
};

struct ServiceCfgType_Tag
{
    uint8 uServiceId;
    uint8 uSubId;
    uint8 uIface;
    uint8 uDataOffset;

    uint8 *pRawData;
    void *pDecodeBuf0;
    void *pDecodeBuf1;
    void *pDecodeBuf2;

    uint16 wDataLen;
    uint8 uBlockLen;
    uint8 uDecodeLen;

    ServiceStatusType *pSvcStat;
    ServiceCbType cbRxDataSvc;
    ApplCbType cbApplSvc;
};

typedef enum Basic_OpType_Tag
{
    DIO_READ_CHANNEL,
    DIO_WRITE_CHANNEL,
    DIO_EDGE_ENABLE,
    DIO_EDGE_DISABLE,

    GET_RESOURCE,
    RELEASE_RESOURCE,

    CANCEL_TX_REQ,
    CANCEL_RX_REQ,
    NOTIFY_TX_REQ,
    NOTIFY_RX_REQ,
    NOTIFY_RX_END_REQ,
    GET_RX_INFO,
    GET_TX_INFO,

    TIMER_DELAY,
    TIMER_TRANSFER
}
Basic_OpType;

typedef struct Bsw_ConfigType_Tag
{
    uint32 (*init)(void);
    uint32 (*transfer_req)(void *pTxBuf, uint16 wTxLength, void *pRxBuf, uint16 wRxLength);
    uint32 (*transfer_ctrl)(uint32 qReq, void *pData, uint16 *pOutData, uint8 *uPendingRx);
    uint32 (*timer_req)(uint32 qTimerReq, uint32 qTimeMs);
    uint32 (*dio_req)(uint32 qCtrlReq);
    uint32 (*resource_req)(uint32 qResId, uint32 qCtrlReq);
    uint32 (*timestamp_req)(uint32 *Value, uint32 *ElapsedValue);
    uint32 (*deinit)(void);

    uint32 qDriverRes;
    uint32 qApplRes;
    uint32 qDiagRes;
}
Basic_ConfigType;

/** Communication Interface Configuration **/

typedef enum Comif_WiredChipType_Tag
{
    COMIF_BQ79600_CHIP = 0x1u,
    COMIF_BQ7971X_CHIP = 0x2u,
    COMIF_BQ7973X_CHIP = 0x3u,
    COMIF_BQ79800_CHIP = 0x4u
}
Comif_WiredChipType;

typedef enum Comif_WirelessChipType_Tag
{
    COMIF_CC2662_CHIP = 0x1u,
    COMIF_CC2762_CHIP = 0x2u
}
Comif_WirelessChipType;

typedef enum Comif_IfaceType_Tag
{
    COMIF_WIRED = 0x1u,
    COMIF_WIRELESS = 0x2u,
    COMIF_DIRECT = 0x3u
}
Comif_CommIfaceType;

typedef enum Comif_PeripheralType_Tag
{
    COMIF_UART = 0x0u,
    COMIF_SPI = 0x1u,
    COMIF_CAN = 0x2u,
    COMIF_ETH = 0x3u
}
Comif_PeripheralType;

typedef struct Comif_OperationType_Tag
{
   void* (*init)(const void *cfg, Emem_StasticsType *pESCfg, const ServiceCfgType *pCommCfg);
   uint8 (*handle_tx)(void *ctx);
   uint8 (*handle_rx)(void *ctx);
   uint8 (*notification)(void *ctx, uint8 uNotifyType);
   uint8 (*ioctl)(void *ctx, uint8 uCmd, void *pData);
   uint8 (*deinit)(void *ctx);
}
Comif_OperationType;

typedef struct Comif_ConfigType_Tag
{
    const void *pComChipCfg;
    const Comif_OperationType *pComifOpsCfg;
    const ServiceCfgType *pCommCfg;

    uint8 uComIfaceCfg;
    uint8 eComifTypeCfg;
    uint8 eComifChipCfg;
    uint8 ePeripCfg;

    uint8 uNfaultPinCfg;
    uint8 uRsvd[3];
}
Comif_ConfigType;

/** Cell Monitor Interface Configuration **/

typedef enum Bmi_BmcType_Tag
{
    BMI_BQ79712_CHIP = 0x1u,
    BMI_BQ79716_CHIP = 0x2u,
    BMI_BQ79718_CHIP = 0x3u,
    BMI_BQ79812_CHIP = 0x4u,
    BMI_BQ79816_CHIP = 0x5u,
    BMI_BQ79818_CHIP = 0x6u,
    BMI_BQ79824_CHIP = 0x7u,
    BMI_BQ79828_CHIP = 0x8u
}
Bmi_BmcType;

typedef enum Bmi_ControlType_Tag
{
    BMI_GET_DRIVER_STATE = 0,
    BMI_GET_DIAG_STATE,
    BMI_GET_DIAG_STATUS,
    BMI_GET_DIAG_RESULT,
    BMI_GET_VERSION,
    BMI_SET_ADC_CTRL1,
    BMI_SET_ADC_CTRL2,
    BMI_SET_ADC_CTRL3,
    BMI_CONTROL_MAX
}
Bmi_ControlType;

typedef enum Bmi_ServiceType_Tag
{
    BMI_SERVICE_CONFIG = 0,
    BMI_SERVICE_BALANCING,
    BMI_SERVICE_CELL_VOLT,
    BMI_SERVICE_GPIO_READ,
    BMI_SERVICE_DIAGNOSTICS,
    BMI_SERVICE_COMMUNICATION,
    BMI_SERVICE_USER,
    BMI_SERVICE_VCAP_REF,
    BMI_SERVICE_VBAT,
    BMI_SERVICE_POWER_CONTROL,
    BMI_SERVICE_RESET,
    BMI_SERVICE_SET_COMM_CONF,
    BMI_SERVICE_SET_BB_CONF,
    BMI_SERVICE_SET_ENB_SPI,
    BMI_SERVICE_SET_SPI_CONF,
    BMI_SERVICE_SET_SPI_EXE,
    BMI_SERVICE_WRITE_SPI_DATA,
    BMI_SERVICE_READ_SPI_DATA,
    BMI_SERVICE_SET_ENB_I2C,
    BMI_SERVICE_SET_I2C_CTRL,
    BMI_SERVICE_SET_I2C_DATA,
    BMI_SERVICE_GET_I2C_DATA,
    BMI_SERVICE_GET_I2C_FAULT,
    BMI_SERVICE_GET_DIAG_RDNT,
    BMI_SERVICE_GET_DIAG_MAIN,
    BMI_SERVICE_SET_GPIO_CONF,
    BMI_SERVICE_SET_GPIO_DOWN,
    BMI_SERVICE_GET_VCELL_ACTSUM,
    BMI_SERVICE_GET_DIE_TEMP,
    BMI_SERVICE_EN_ADC_FREEZE,
    BMI_SERVICE_DIS_ADC_FREEZE,
    BMI_SERVICE_SET_ADC_DELAY,
    BMI_SERVICE_MAX
}
Bmi_ServiceType;

typedef struct Bmi_OperationType_Tag
{
   void* (*init)(const InterfaceCfgType *ctx, Comif_ManagerType *pComif, uint8 uNfault, uint8 uStartup, uint8 uWup);
   uint8 (*handle_tx)(void *ctx);
   uint8 (*handle_rx)(void *ctx);
   uint8 (*process_rx)(void *ctx, const ServiceCfgType *pSvc, const uint8 *pRxData, uint8 uStatus);
   uint8 (*decode_rx)(void *ctx, uint8 uService, uint8 uSubId, void *buffer, uint16 wLength);
   uint8 (*service)(void *ctx, uint8 uServiceTyp, uint8 uCmd, uint8 uNodeId, void *pData);
   uint8 (*notify)(void *ctx, uint8 uType);
   uint8 (*ioctl)(void *ctx, uint8 uCmd, void *pData, uint16 wLength);
   uint8 (*deinit)(void *ctx);
}
Bmi_OperationType;

typedef struct Bmi_ConfigType_Tag
{
    void *pBmicCfg;
    const Basic_ConfigType *pBswCfg;
    const Bmi_OperationType *pBmcOpsCfg;
    const ServiceCfgType *pFuncCfg;
    const ServiceCfgType *pCellBalCfg;
    const ServiceCfgType *pDiagCfg;

    uint32 eBmiChipCfg;
}
Bmi_ConfigType;

/** Pack Monitor Interface Configuration **/

typedef enum Pmi_PmcType_Tag
{
    PMI_BQ79731_CHIP = 0x1u,
    PMI_BQ79735_CHIP = 0x2u
}
Pmi_PmcType;

typedef enum Pmi_ControlType_Tag
{
    PMI_GET_DRIVER_STATE,
    PMI_GET_DIAG_STATE,
    PMI_GET_VERSION,
    PMI_CONTROL_MAX
}
Pmi_ControlType;

typedef enum Pmi_ServiceType_Tag
{
    PMI_SERVICE_STARTUP,
    PMI_SERVICE_CONFIG,
    PMI_SERVICE_PACK_CURRENT,
    PMI_SERVICE_VF_VOLT,
    PMI_SERVICE_CP_VOLT,
    PMI_SERVICE_CC_ACC_CNT,
    PMI_SERVICE_VGPIO,
    PMI_SERVICE_DIETEMP,

    PMI_SERVICE_GET_CCOVF,
    PMI_SERVICE_GET_I2C_DATA,
    PMI_SERVICE_RESET,
    PMI_SERVICE_DIAGNOSTICS,
    PMI_SERVICE_CC_CLEAR,
    PMI_SERVICE_SET_I2C_DATA,
    PMI_SERVICE_SET_I2C_CTRL,
    PMI_SERVICE_ENABLE_I2C ,
    PMI_SERVICE_WRITE_SPI_DATA,
    PMI_SERVICE_SET_SPI_EXE,
    PMI_SERVICE_SET_SPI_CTRL,
    PMI_SERVICE_MSPI_CONF,
    PMI_SERVICE_EN_ADC_FREEZE,
    PMI_SERVICE_DIS_ADC_FREEZE,
    PMI_SERVICE_SET_ADC_DELAY,
    PMI_SERVICE_ENTER_COMM_DEBUG,
    PMI_SERVICE_SET_OC_OUTPUT,
    PMI_SERVICE_COMMUNICATION,
    PMI_SERVICE_GET_VRF_CAP,
    PMI_SERVICE_USER,
    PMI_SERVICE_MAX
}
Pmi_ServiceType;

typedef struct Pmi_OperationType_Tag
{
   void* (*init)(const InterfaceCfgType *ctx, Comif_ManagerType *pComfig, uint8 uWkup);
   uint8 (*handle_tx)(void *ctx);
   uint8 (*handle_rx)(void *ctx);
   uint8 (*process_rx)(void *ctx, const ServiceCfgType *pSvc, const uint8 *pRxData, uint8 uStatus);
   uint8 (*decode_rx)(void *ctx, uint8 uService, void *buffer, uint16 wLength);
   uint8 (*service)(void *ctx, uint8 uServiceTyp, uint8 uCmd, void *pData);
   uint8 (*notify)(void *ctx, uint8 uType);
   uint8 (*ioctl)(void *ctx, uint8 uCmd, void *pData);
   uint8 (*deinit)(void *ctx);
}
Pmi_OperationType;

typedef struct Pmi_ConfigType_Tag
{
    void *pPmicCfg;
    const Basic_ConfigType *pBswCfg;
    const Pmi_OperationType *pPmcOpsCfg;
    const ServiceCfgType *pFuncCfg;
    const ServiceCfgType *pDiagCfg;
    const ServiceCfgType *pCommCfg;

    uint16 ePmiChipCfg;
    uint16 uPmiIfaceCfg;
}
Pmi_ConfigType;

/** Common Configurations **/

struct InterfaceCfgType_Tag
{
    const Comif_ConfigType *pComIfCfg;
    const Bmi_ConfigType *pBmiCfg;
    const Pmi_ConfigType *pPmiCfg;

    uint8 uIfaceCfg;
    uint8 eComTypeCfg;
    uint8 eReqComDirCfg;
    uint8 uNDevsCfg;

    uint8 uNBmicsCfg;
    uint8 uNPmicsCfg;
    uint8 uTopOfDevCfg;

    uint8 uIsPmicInIfCfg;
    uint8 uPmicDevIdCfg;
    uint8 uRsvd[3];
};

typedef struct Bms_ConfigType_Tag
{
    uint32 qStartSig;
    const InterfaceCfgType *pIfaceCfg;
    uint32 qEndSig;
}
Bms_ConfigType;

typedef enum
{
    GPIO_DISABLED = 0,
    GPIO_ADC_RATIOMETRIC,
    GPIO_ADC_ABSOLUTE,
    GPIO_DIGITAL_INPUT,
    GPIO_DIGITAL_OUTPUT_HIGH,
    GPIO_DIGITAL_OUTPUT_LOW,
    GPIO_ADC_WEAK_PULL_UP,
    GPIO_ADC_WEAK_PULL_DOWN
}Gpio_val;
typedef struct
{
    uint8 gpio_conf_addr;
    uint8 devId;
    uint8 gpio_conf_val;
}Gpio_conf;
/*********************************************************************************************************************
 * Exported Object Declarations
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Exported Function Prototypes
 *********************************************************************************************************************/

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

#endif /*TIBMS_TYPES_H*/

/*********************************************************************************************************************
 * End of File: tibms_types.h
 *********************************************************************************************************************/
