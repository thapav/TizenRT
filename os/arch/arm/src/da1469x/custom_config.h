/**
 ****************************************************************************************
 *
 * @file custom_config.h
 *
 * @brief Custom configuration file for non-FreeRTOS applications executing from RAM.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef CUSTOM_CONFIG_TIZENRT_H_
#define CUSTOM_CONFIG_TIZENRT_H_

#include "bsp_definitions.h"
#include "sw_version.h"

#define OS_BAREMETAL
//#define SEC_MODEN
#define dg_configDEVICE DEVICE_DA1469x

////partition of 16M flash
//#define USE_PARTITION_TABLE_16MB_WITH_SUOTA
//partition of 32M flash
#define USE_PARTITION_TABLE_32MB_WITH_SUOTA

#define ARM_MATH_CM33
#define CONFIG_RETARGET
#define CONFIG_RETARGET_UART    HW_UART2
#define CONFIG_RETARGET_UART_TX_PORT    HW_GPIO_PORT_0
#define CONFIG_RETARGET_UART_TX_PIN     HW_GPIO_PIN_9
#define CONFIG_RETARGET_UART_TX_MODE    HW_GPIO_MODE_OUTPUT
#define CONFIG_RETARGET_UART_TX_FUNC    HW_GPIO_FUNC_UART2_TX

#define CONFIG_RETARGET_UART_RX_PORT    HW_GPIO_PORT_0
#define CONFIG_RETARGET_UART_RX_PIN     HW_GPIO_PIN_8
#define CONFIG_RETARGET_UART_RX_MODE    HW_GPIO_MODE_INPUT_PULLUP
#define CONFIG_RETARGET_UART_RX_FUNC    HW_GPIO_FUNC_UART2_RX
#if (dg_configIMAGE_SETUP == DEVELOPMENT_MODE)
#define dg_configSKIP_MAGIC_CHECK_AT_START      ( 0 )
#else
#define dg_configSKIP_MAGIC_CHECK_AT_START      ( 1 )
#endif

/*************************************************************************************************\
 * Board revision
 EMUL rev0.3 - 0x00
 MAIN rev0.0 / 0.1 - 0x01
 */
#define dg_configBoardRev                       (0x03)

#define dg_configUSE_USB_ENUMERATION            ( 0 )
#define dg_configOPTIMAL_RETRAM                 ( 0 )
//#define __HEAP_SIZE                             0x2000
//#define __HEAP_SIZE                             ( 0x0c00 )

/*************************************************************************************************\
 * System configuration
 */
#define dg_configUSE_LP_CLK                     ( LP_CLK_32768 )
#define dg_configEXEC_MODE                      ( MODE_IS_CACHED )
#define dg_configCODE_LOCATION                  ( NON_VOLATILE_IS_FLASH )
#define dg_configEMULATE_OTP_COPY               ( 0 )
#define dg_configUSER_CAN_USE_TIMER1            (1)

#define dg_configMEM_RETENTION_MODE             (0x1F)
#define dg_configSHUFFLING_MODE                 (0x3)

#define dg_configUSE_WDOG                       (0)
#define dg_configUSE_BOD                        (0)

#define dg_configUSE_DCDC                       (1)

#define dg_configFLASH_CONNECTED_TO             (FLASH_CONNECTED_TO_1V8)
#define dg_configFLASH_POWER_DOWN               (0)
#define dg_configFLASH_AUTODETECT               (1)

#define dg_configPOWER_1V8_ACTIVE               (1)
#define dg_configPOWER_1V8_SLEEP                (1)

#define dg_configBATTERY_TYPE                   (BATTERY_TYPE_LIMN2O4)
#define dg_configBATTERY_CHARGE_CURRENT         2       // 30mA
#define dg_configBATTERY_PRECHARGE_CURRENT      20      // 2.1mA
#define dg_configBATTERY_CHARGE_NTC             1       // disabled

#define dg_configUSE_HW_UART                    (1)
#define dg_configUART_ADAPTER                   (1)


#define dg_configUSE_HW_QSPI2                   ( 1 )
#define dg_configQSPIC2_DEV_HEADER_FILE         "psram_aps1604jsq.h"
#define dg_configQSPIC2_DEV_CONFIG              psram_aps1604jsq_config

#if (dg_configUSE_HW_QSPI2!=1)
#define dg_configFAULT_DEBUG_DUMP               ( 1 )
#error "Cannot have the RAM DUMP enabled for FAULTS, without external RAM and enabled the QSPI2 controller"
#endif
#define dg_configUSE_USB                        0
#define dg_configUSE_USB_CHARGER                0
#define dg_configALLOW_CHARGING_NOT_ENUM        1

#define dg_configUSE_ProDK                      ( 1 )
#define dg_configUSE_SW_CURSOR                  (1)
#define dg_configUSE_AUTO_CHIP_DETECTION        (1)

#define dg_configPOWER_1V8P                     (1)

#define dg_configDISABLE_BACKGROUND_FLASH_OPS   (1)

#define dg_configCRYPTO_ADAPTER                 (0)

#define dg_configUSE_HW_WKUP                    (0)

#define dg_configSUPPRESS_HelloMsg              (0)

#define dg_configVERIFY_QSPI_WRITE              (1)

#define dg_configFLASH_ADAPTER                  (1)
#define dg_configNVMS_ADAPTER                   (1)
#define dg_configNVMS_VES                       (1)
#define dg_configFLASH_GIGA32

#if (dg_configBoardRev == 0x00)
#define dg_configFLASH_DEVICE_TYPE              GD25LQ_SERIES
#define dg_configFLASH_DENSITY                  GD25LQ256_SIZE
#define dg_configFLASH_CONFIG                   flash_gd25lq256_config
#define dg_configFLASH_HEADER_FILE              "qspi_gd25lq256.h"
#else
#ifdef dg_configFLASH_GIGA32
#define dg_configFLASH_DEVICE_TYPE              GD25LQ_SERIES
#define dg_configFLASH_DENSITY                  GD25LQ256_SIZE
#define dg_configFLASH_CONFIG                   flash_gd25lq256_config
#define dg_configFLASH_HEADER_FILE              "qspi_gd25lq256.h"
#else
#define dg_configFLASH_DEVICE_TYPE              W25Q256JWPM
#define dg_configFLASH_DENSITY                  W25Q_256Mb_SIZE
#define dg_configFLASH_CONFIG                   flash_w25q256jwpm_config
#define dg_configFLASH_HEADER_FILE              "qspi_w25q256jw_pm.h"
#endif
#endif

/*
#ifdef dg_configFLASH_GIGA32
#if (dg_configFLASH_POWER_OFF==1 )
#undef dg_configFLASH_POWER_OFF
#define dg_configFLASH_POWER_OFF (0)
#endif
#endif
*/

//partition of 16M flash
//#define USE_PARTITION_TABLE_16MB_WITH_SUOTA
//partition of 32M flash
#define USE_PARTITION_TABLE_32MB_WITH_SUOTA

#define dg_configCACHEABLE_QSPI_AREA_LEN        ( NVMS_WORKOUT_HISTORY_PART_start - MEMORY_QSPIF_BASE )
#define dg_configQSPI_CODE_SIZE_AA              ( 1024 * 1024 * 2 )
#define dg_configQSPI_MAX_IMAGE_SIZE            ( 1024 * 1024 * 2 )

/*************************************************************************************************\
* QSPI Ext Ram configuration
*/
#define dg_configUSE_HW_QSPI2                   ( 1 )
#define dg_configQSPIC2_DEV_HEADER_FILE         "psram_aps1604jsq.h"
#define dg_configQSPIC2_DEV_CONFIG              psram_aps1604jsq_config

#define dg_configENABLE_PARAM_PART_SC           (1)

/*************************************************************************************************\
 * User GPIO config
 */
#ifdef INCLUDE_USER_GPIO_CONFIG

/* SENSOR I2C */
#define SENSOR_I2C_SDA_PORT          (HW_GPIO_PORT_1)
#define SENSOR_I2C_SDA_PIN           (HW_GPIO_PIN_15)
#define SENSOR_I2C_SCL_PORT          (HW_GPIO_PORT_1)
#define SENSOR_I2C_SCL_PIN           (HW_GPIO_PIN_16)

/* HRM EN */
#define SENSOR_HRM_EN_PORT           (HW_GPIO_PORT_1)
#define SENSOR_HRM_EN_PIN            (HW_GPIO_PIN_13)

/* ACC/HRM INT */
#define SENSOR_INT_PORT              (HW_GPIO_PORT_1)
#define SENSOR_HRM_INT_PIN           (HW_GPIO_PIN_14)
#define SENSOR_ACC_INT_PIN           (HW_GPIO_PIN_17)

/* DISPLAY TE */
#define DISP_TE_PORT               (HW_GPIO_PORT_0)
#define DISP_TE_PIN                (HW_GPIO_PIN_24)

/* DISPLAY SPI */
#define DISP_SPI_CS_PORT             (HW_GPIO_PORT_0)
#define DISP_SPI_CS_PIN              (HW_GPIO_PIN_28)
#define DISP_SPI_SCK_PORT            (HW_GPIO_PORT_1)
#define DISP_SPI_SCK_PIN             (HW_GPIO_PIN_4)
#define DISP_SPI_MOSI_PORT           (HW_GPIO_PORT_0)
#define DISP_SPI_MOSI_PIN            (HW_GPIO_PIN_18)
#define DISP_SPI_DCX_PORT            (HW_GPIO_PORT_0)
#define DISP_SPI_DCX_PIN             (HW_GPIO_PIN_27)

/* DISPLAY EN */
#define DISP_EN_PORT                 (HW_GPIO_PORT_0)
#define DISP_EN_PIN                  (HW_GPIO_PIN_19)
/* DISPLAY RST */
#define DISP_RST_PORT                (HW_GPIO_PORT_0)
#define DISP_RST_PIN                 (HW_GPIO_PIN_20)

/* WIRELESS_CHARGING */
#define WLC_I2C_SDA_PORT          (HW_GPIO_PORT_1)
#define WLC_I2C_SDA_PIN           (HW_GPIO_PIN_18)
#define WLC_I2C_SCL_PORT          (HW_GPIO_PORT_1)
#define WLC_I2C_SCL_PIN           (HW_GPIO_PIN_19)
#define WLC_RECT_INT_PORT  		  (HW_GPIO_PORT_1)
#define WLC_RECT_INT_PIN          (HW_GPIO_PIN_10)
#define WLC_NTAG_FD_PORT          (HW_GPIO_PORT_1)
#define WLC_NTAG_FD_PIN           (HW_GPIO_PIN_20)
#define WLC_NTAG_EN_PORT          (HW_GPIO_PORT_1)
#define WLC_NTAG_EN_PIN           (HW_GPIO_PIN_21)
#define WLC_RECT_SLEEP_PORT       (HW_GPIO_PORT_1)
#define WLC_RECT_SLEEP_PIN        (HW_GPIO_PIN_22)

#ifdef INCLUDE_NFC
#define NFC_EN_PORT                (HW_GPIO_PORT_0)
#define NFC_EN_PIN                 (HW_GPIO_PIN_26)
/* NFC_FIRMWARE */
#define NFC_FIRMWARE_PORT          (HW_GPIO_PORT_1)
#define NFC_FIRMWARE_PIN           (HW_GPIO_PIN_1)
/* NFC_IRQ */
#define NFC_IRQ_PORT               (HW_GPIO_PORT_1)
#define NFC_IRQ_PIN                (HW_GPIO_PIN_0)

#define NFC_I2C_SDA_PORT          (HW_GPIO_PORT_0)
#define NFC_I2C_SDA_PIN           (HW_GPIO_PIN_17)
#define NFC_I2C_SCL_PORT          (HW_GPIO_PORT_0)
#define NFC_I2C_SCL_PIN           (HW_GPIO_PIN_16)

#endif

/* TOUCH I2C*/
#define TOUCH_I2C_SCL_PORT           (HW_GPIO_PORT_0)
#define TOUCH_I2C_SCL_PIN            (HW_GPIO_PIN_29)
#define TOUCH_I2C_SDA_PORT           (HW_GPIO_PORT_0)
#define TOUCH_I2C_SDA_PIN            (HW_GPIO_PIN_31)
#define TOUCH_I2C_RST_PORT           (HW_GPIO_PORT_0)
#define TOUCH_I2C_RST_PIN            (HW_GPIO_PIN_30)
#define TOUCH_I2C_INT_PORT           (HW_GPIO_PORT_0)
#define TOUCH_I2C_INT_PIN            (HW_GPIO_PIN_21)

/* SIDE KEY */
#define SIDE_KEY_INT_PORT          (HW_GPIO_PORT_0)
#define SIDE_KEY_INT_PIN           (HW_GPIO_PIN_7)

/* MOTOR */
#define MOTOR_HWREV00_EN_GPIO_PORT (HW_GPIO_PORT_0)
#define MOTOR_HWREV00_EN_GPIO_PIN  (HW_GPIO_PIN_15)
#define MOTOR_EN_GPIO_PORT         (HW_GPIO_PORT_1)
#define MOTOR_EN_GPIO_PIN          (HW_GPIO_PIN_7)

/* DM HASH */
#define dg_configUSE_HW_AES_HASH                                (1)
#define dg_configAES_USE_OTP_KEY                                (1)

/* HW REV Check GPIOS*/
#define HW_REV0_GPIO_PORT         (HW_GPIO_PORT_0)
#define HW_REV0_GPIO_PIN          (HW_GPIO_PIN_14)
#define HW_REV1_GPIO_PORT         (HW_GPIO_PORT_0)
#define HW_REV1_GPIO_PIN          (HW_GPIO_PIN_6)

#endif
#include "bsp_defaults.h"
/* Include middleware default values */
#include "middleware_defaults.h"
/* Include memory layout */
#include "bsp_memory_layout.h"

#endif /* CUSTOM_CONFIG_RAM_H_ */
