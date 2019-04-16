/**
 * \addtogroup BSP
 * \{
 * \addtogroup SYSTEM
 * \{
 * \addtogroup MEMORY
 *
 * \{
 */

/**
 ****************************************************************************************
 *
 * @file qspi_gd25lq256.h
 *
 * @brief QSPI flash driver for the GigaDevice GD25LQ256
 *
 * Copyright (C) 2016-2018 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */
#ifndef _QSPI_GD25LQ256_H_
#define _QSPI_GD25LQ256_H_

#ifndef GIGADEVICE_ID
#define GIGADEVICE_ID  0xC8
#endif

#ifndef GD25LQ_SERIES
#define GD25LQ_SERIES  0x60
#endif

#define GD25LQ256_SIZE  0x19

#include "qspi_common.h"
#include "sdk_defs.h"

#include "qspi_gigadevice.h"

#if dg_configFLASH_POWER_OFF == 1
#if FLASH_AUTODETECT == 1
#pragma message "Please note that QSPI Flash GD25LQ256 will NOT work properly in FLASH_POWER_OFF mode"
#else
#error "QSPI Flash GD25LQ256 will NOT work properly in FLASH_POWER_OFF mode"
#endif
#endif

// Flash power up/down timings
#define GD25LQ256_POWER_DOWN_DELAY_US          20

#define GD25LQ256_RELEASE_POWER_DOWN_DELAY_US  20

#define GD25LQ256_POWER_UP_DELAY_US            10000

#if (dg_configFLASH_POWER_DOWN == 1)
/**
 * \brief uCode for handling the QSPI FLASH release from power-down.
 */
        /*
         * 0x09   // CMD_NBYTES = 1, CMD_TX_MD = 0 (Single), CMD_VALID = 1
         * 0xD0   // CMD_WT_CNT_LS = 0xD0 --> 45000 / 62.5 = 720   // 45usec for worst case
         * 0x02   // CMD_WT_CNT_MS = 0x02
         * 0xAB   // Release Power Down
         * (up to 16 words)
         */
        const uint32_t GD25LQ256_ucode_wakeup[] = {
                0xAB000009 | (((uint16_t)(GD25LQ256_RELEASE_POWER_DOWN_DELAY_US*1000/31.25) & 0xFFFF) << 8),
        };
#else
/**
 * \brief uCode for handling the QSPI FLASH exit from the "Continuous Read Mode".
 */
        /*
         * 0x45   // CMD_NBYTES = 8, CMD_TX_MD = 2 (Quad), CMD_VALID = 1
         * 0x00   // CMD_WT_CNT_LS = 0
         * 0x00   // CMD_WT_CNT_MS = 0
         * 0xFF
         * 0xFF
         * 0xFF
         * 0xFF
         * 0xFF
         * 0xFF
         * 0xFF
         * 0xFF
         */
        const uint32_t GD25LQ256_ucode_wakeup[] = {
                0xFF000045,
                0x00FFFFFF,
        };
#endif


static void flash_gd25lq256_initialize(HW_QSPIC_ID id);
static void flash_gd25lq256_sys_clock_cfg(HW_QSPIC_ID id, sys_clk_t sys_clk);
static uint8_t flash_gd25lq256_get_dummy_bytes(HW_QSPIC_ID id);

static qspi_flash_config_t flash_gd25lq256_config = {
        .manufacturer_id               = GIGADEVICE_ID,
        .device_type                   = GD25LQ_SERIES,
        .device_density                = GD25LQ256_SIZE,
        .is_suspended                  = flash_gd_is_suspended,
        .initialize                    = flash_gd25lq256_initialize,
        .sys_clk_cfg                   = flash_gd25lq256_sys_clock_cfg,
        .get_dummy_bytes               = flash_gd25lq256_get_dummy_bytes,
        .break_seq_size                = HW_QSPI_BREAK_SEQ_SIZE_2B,
        .address_size                  = HW_QSPI_ADDR_SIZE_32,
        .read_instr                    = CMD_FAST_READ_QUAD,
        .read_addr_phaze               = HW_QSPI_BUS_MODE_QUAD,
        .page_program_opcode           = CMD_QUAD_PAGE_PROGRAM,
        .page_qpi_program_opcode       = CMD_QPI_PAGE_PROGRAM,
        .erase_opcode                  = CMD_SECTOR_ERASE,
        .erase_suspend_opcode          = GD_ERASE_PROGRAM_SUSPEND,
        .erase_resume_opcode           = GD_ERASE_PROGRAM_RESUME,
        .quad_page_program_address     = false,
        .read_erase_progress_opcode    = CMD_READ_STATUS_REGISTER,
        .erase_in_progress_bit         = FLASH_STATUS_BUSY_BIT,
        .erase_in_progress_bit_high_level = true,
#if GIGADEVICE_PERFORMANCE_MODE
        .send_once                     = 1,
        .extra_byte                    = 0x20,
#else
        .send_once                     = 0,
        .extra_byte                    = 0x00,
#endif
        .send_extra_byte               = true,
        .ucode_wakeup                  = {GD25LQ256_ucode_wakeup, sizeof(GD25LQ256_ucode_wakeup)},
        .power_down_delay              = GD25LQ256_POWER_DOWN_DELAY_US,
        .release_power_down_delay      = GD25LQ256_RELEASE_POWER_DOWN_DELAY_US,
        .power_up_delay                = GD25LQ256_POWER_UP_DELAY_US,
        .is_ram                        = false,
        .qpi_mode                      = false,
        .enter_qpi_opcode              = CMD_ENTER_QPI_MODE,
        .memory_size                   = MEMORY_SIZE_256Mb, /* 256M-bit Serial Flash */

};

__RETAINED_CODE static void flash_gd25lq256_initialize(HW_QSPIC_ID id)
{
        uint8_t cmd_32bit_addressing[1];

        flash_gd_enable_quad_mode(id);

        if (flash_gd25lq256_config.address_size == HW_QSPI_ADDR_SIZE_32) {
                cmd_32bit_addressing[0] = 0xB7; // enable 4B (32-bit) addressing
        } else {
                cmd_32bit_addressing[0] = 0xE9; // disable 4B (32-bit) addressing (24-bit default)
        }

        flash_write_enable(id);
        flash_write(id, cmd_32bit_addressing, 1);
        /* Wait for the Flash to process the command */
        while (flash_is_busy(id));
}

__RETAINED_CODE static void flash_gd25lq256_sys_clock_cfg(HW_QSPIC_ID id, sys_clk_t sys_clk)
{
}

__RETAINED_CODE static uint8_t flash_gd25lq256_get_dummy_bytes(HW_QSPIC_ID id)
{
        return 2;
}

#endif /* _QSPI_GD25LQ256_H_ */
/**
 * \}
 * \}
 * \}
 */
