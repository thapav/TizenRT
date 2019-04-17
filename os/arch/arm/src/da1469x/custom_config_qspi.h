/**
 ****************************************************************************************
 *
 * @file custom_config_qspi.h
 *
 * @brief Board Support Package. User Configuration file for cached QSPI mode.
 *
 * Copyright (C) 2015-2018 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */
#ifndef CUSTOM_CONFIG_QSPI_H_
#define CUSTOM_CONFIG_QSPI_H_

#define SEC_MODEN

#include "sw_version.h"
#include "bsp_definitions.h"
#include "custom_config_psram.h"

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

#define CONFIG_USE_BLE

#if defined(SEC_MODEN)
#define CONFIG_MODEN_SEC_RESET
//#define CONFIG_LOG_TIMESTAMP
#endif

#if (dg_configIMAGE_SETUP == DEVELOPMENT_MODE)
#define dg_configFAULT_DEBUG_DUMP               ( 1 )
#define dg_configSYS_DUMP                       ( 1 )
#define CONFIG_WD_MONITOR

#define dg_configSKIP_MAGIC_CHECK_AT_START      ( 0 )

#else

#define dg_configFAULT_DEBUG_DUMP               ( 0 )
#define dg_configSYS_DUMP                       ( 0 )

#define dg_configSKIP_MAGIC_CHECK_AT_START      ( 1 )
#endif

/*************************************************************************************************\
 * Board revision
 EMUL rev0.3 - 0x00
 MAIN rev0.0 / 0.1 - 0x01
 */
#define dg_configBoardRev                       (0x03)

/*************************************************************************************************\
 * System configuration
 */
#define dg_configUSE_LP_CLK                     ( LP_CLK_32768 )
#define dg_configEXEC_MODE                      ( MODE_IS_CACHED )
#define dg_configCODE_LOCATION                  ( NON_VOLATILE_IS_FLASH )
#define dg_configEMULATE_OTP_COPY               ( 0 )
#define dg_configUSER_CAN_USE_TIMER1            ( 0 )
#define CONFIG_USE_SEC_PM
#define CONFIG_USE_SEC_FAULT
#define CONFIG_USE_SEC_MPU
#define dg_configUSE_HW_MPU                     ( 1 )

// Suota function is supported if this config is set 1
#define dg_configSUOTA_SUPPORT                  ( 1 )
#define dg_configDISABLE_BACKGROUND_FLASH_OPS   ( 0 )
#define dg_configUSE_WDOG                       ( 1 )

#define dg_configWDOG_EXTENDED                  ( 1 )
#if (dg_configWDOG_EXTENDED == 1)
#define dg_configWDOG_MAX_TASKS_CNT             ( 64 )
#else
#define dg_configWDOG_MAX_TASKS_CNT             ( 32 )
#endif

#define dg_configFLASH_CONNECTED_TO             ( FLASH_CONNECTED_TO_1V8 )
#define dg_configFLASH_POWER_DOWN               ( 1 )
#define dg_configPOWER_1V8_ACTIVE               ( 1 )
#define dg_configPOWER_1V8_SLEEP                ( 1 )
#define dg_configPOWER_1V8p_ACTIVE              ( 1 )
#define dg_configPOWER_1V8p_SLEEP               ( 1 )

#define dg_configBATTERY_TYPE                   ( BATTERY_TYPE_LIMN2O4 )
#define dg_configBATTERY_CHARGE_CURRENT         ( 2 )    // 30mA
#define dg_configBATTERY_PRECHARGE_CURRENT      ( 20 )   // 2.1mA
#define dg_configBATTERY_CHARGE_NTC             ( 1 )    // disabled

#define dg_configUSE_HW_UART                    (1)
#define dg_configUART_ADAPTER                   (1)


#define dg_configUSE_HW_QSPI2                   ( 1 )
#define dg_configQSPIC2_DEV_HEADER_FILE         "psram_aps1604jsq.h"
#define dg_configQSPIC2_DEV_CONFIG              psram_aps1604jsq_config

#if (dg_configUSE_HW_QSPI2!=1)
#define dg_configFAULT_DEBUG_DUMP               ( 1 )
#error "Cannot have the RAM DUMP enabled for FAULTS, without external RAM and enabled the QSPI2 controller"
#endif

#define dg_configUSE_USB                        ( 1 )
#define dg_configUSE_USB_CHARGER                ( 1 )
#define dg_configALLOW_CHARGING_NOT_ENUM        ( 1 )

#define dg_configUSE_ProDK                      ( 1 )

#define dg_configUSE_SW_CURSOR                  ( 1 )

#define dg_configTESTMODE_MEASURE_SLEEP_CURRENT ( 0 )

/* CHARGER */
#define dg_configUSE_HW_CHARGER                 ( 1 )
#define dg_configUSE_HW_USB_CHARGER             ( 1 )
#define dg_configUSE_SYS_CHARGER                ( 1 )
#define dg_configUSE_SYS_USB                    ( 1 )
#define dg_configUSE_USB_ENUMERATION            ( 0 )
#define dg_configCHG_TEMP_CONTROL               ( 0 )
#define dg_configCHARGER_TEST_ONLY              ( 0 ) // INCLUDE_SCREEN_TIMEOUT off
#define dg_configFMP_TEMP_                      ( 0 )

/* BATTERY */
#define INCLUDE_BATTERY_LEVEL                   ( 1 )

/* MOTOR */
#define INCLUDE_MOTOR_ENABLE                    ( 1 )

/*************************************************************************************************\
 * FreeRTOS configuration
 */
#define OS_FREERTOS                              /* Define this to use FreeRTOS */
#define configTOTAL_HEAP_SIZE                    ( 76 * 1024 )   /* 96KB FreeRTOS Total Heap Size */

/*************************************************************************************************\
 * PSRAM Heap Configuration
 */
#define configUSE_PSRAM_HEAP                     ( 1 )
#define configUSE_PSRAM_STACK_WHITELIST			 ( 1 )
#define configTOTAL_PSRAM_HEAP_SIZE              ( CONFIG_PSRAM_HEAP_SIZE )   /* 128KB PSRAM Heap Size */
#define configPSRAM_HEAP_BASE                    ( CONFIG_PSRAM_HEAP_BASE )   /* PSRAM address */

/*************************************************************************************************\
 * segger systemview configuration
 */
#define dg_configSYSTEMVIEW                     ( 0 )
#define dg_configPERFORMANCE                    ( 0 )

#define configWRIST_UP_PROFILING                ( 0 )

/*************************************************************************************************\
 * Peripherals configuration
 */
#define dg_configFLASH_ADAPTER                  ( 1 )
#define dg_configNVMS_ADAPTER                   ( 1 )
#define dg_configNVMS_VES                       ( 1 )
#if dg_configSUOTA_SUPPORT
#define dg_configNVPARAM_ADAPTER                ( 1 )
#endif
#define dg_configNVPARAM_ADAPTER                ( 1 )
#define dg_configGPADC_ADAPTER                  ( 1 )
#define dg_configUSE_HW_SENSOR_NODE             ( 1 )
#define dg_configUSE_SNC_HW_GPADC               ( 1 )
#define dg_configSNC_ADAPTER                    ( 1 )
#define dg_configUSE_NFC                        ( 1 )
#define dg_configUSE_HW_LCDC                    ( 1 )
#define dg_configLCDC_ADAPTER                   ( 1 )
#define dg_configI2C_ADAPTER                    ( 1 )
#define dg_configUSE_HW_I2C                     ( 1 )
#define dg_configUSE_SNC_HW_I2C                 ( 0 )
#define dg_configACCEL_LSM6DSL
#define dg_configHRM_AFE4410

#if dg_configSUOTA_SUPPORT
//#define defaultBLE_PPCP_INTERVAL_MIN            (BLE_CONN_INTERVAL_FROM_MS(500))    // 500 ms
//#define defaultBLE_PPCP_INTERVAL_MAX            (BLE_CONN_INTERVAL_FROM_MS(750))    // 750 ms
//#define defaultBLE_PPCP_SUP_TIMEOUT             (BLE_SUPERVISION_TMO_FROM_MS(6000)) // 6000 ms

#define BLE_MAX_MISSES_ALLOWED                  (3)
#define BLE_MAX_DELAYS_ALLOWED                  (3)

/*
 * SUOTA loader configuration:
 * - To enable SUOTA over GATT only, set SUOTA_VERSION to any version >= SUOTA_VERSION_1_1
 *      and leave SUOTA_PSM undefined.
 * - To enable SUOTA over GATT and L2CAP CoC, set SUOTA_VERSION to any version >= SUOTA_VERSION_1_2
 *      and also define SUOTA_PSM to match the desired PSM. In this case the central device
 *      can use either of both according to its preference.
 */
#define SUOTA_VERSION   SUOTA_VERSION_1_3
#define SUOTA_PSM       0x81

/*************************************************************************************************\
 * FreeRTOS specific config
 */
#define OS_FREERTOS                              /* Define this to use FreeRTOS */

#if SUOTA_PSM
        #define SUOTA_HEAP_OVERHEAD     (8192)// SUOTA_HEAP_OVERHEAD is changed from 3200 to 8192 for increasing SUOTA speed
#else
        #define SUOTA_HEAP_OVERHEAD     (0)
#endif
#endif // End configs of suota support

/*************************************************************************************************\
* QSPI Flash memory configuration
*/
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

/*************************************************************************************************\
 * BLE configuration
 */
#define CONFIG_USE_BLE_SERVICES

#define CONFIG_USE_BLE_CLIENTS

#define dg_configBLE_CENTRAL                    ( 0 )
#define dg_configBLE_GATT_CLIENT                ( 1 )
#define dg_configBLE_OBSERVER                   ( 0 )
#define dg_configBLE_BROADCASTER                ( 0 )
// If SUOTA_PSM, set dg_configBLE_L2CAP_COC is 1
#ifndef SUOTA_PSM
#define dg_configBLE_L2CAP_COC                  ( 0 )
#endif
#define dg_configASSIGN_RANDOM_BD_ADDRESS  (1)
#define CONFIG_BLE_STORAGE                      ( 1 )

/*************************************************************************************************\
 * User Feature config
 */
#define FEATURE_SAMSUNG
#define INCLUDE_DISPLAY_DRIVER
#define INCLUDE_SCREEN_TIMEOUT
#define INCLUDE_USER_GPIO_CONFIG
#define INCLUDE_WLC /* NFC Wireless Charging*/
#define INCLUDE_UI_TASK
#define INCLUDE_NFC
#define INCLUDE_TOUCH_DRIVER
#define INCLUDE_FLASH_LOG_ENABLE
#define INCLUDE_DISPLAY_COLOR_8RGB888
#define I2C_TIMEOUT_ERROR
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


#define SAP_ENABLED 1

// Optimizations:
#define dg_configBLE_STACK_DB_HEAP_SIZE         ( 5120 )  // 5KB
#define defaultBLE_MAX_BONDED                   ( 2 )
#define defaultBLE_MAX_CONNECTIONS              ( 8 )

// Default Bluetooth Device Address
#define defaultBLE_STATIC_ADDRESS            { 0x03, 0x00, 0x80, 0xCA, 0xEA, 0x80 }

/* Include bsp default values */
#include "bsp_defaults.h"
/* Include middleware default values */
#include "middleware_defaults.h"
/* Include memory layout */
#include "bsp_memory_layout.h"

#endif /* CUSTOM_CONFIG_QSPI_H_ */
