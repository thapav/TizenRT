/**
 * \addtogroup PLA_BSP_SYSTEM
 * \{
 * \addtogroup PLA_MEMORY
 * 
 * \{
 */

/**
 ****************************************************************************************
 *
 * @file qspi_w25q256jw_pm.h
 *
 * @brief QSPI flash driver for the Winbond W25Q80JQPIG
 *
 * Copyright (C) 2016-2018 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */
#ifndef _QSPI_W25Q256JWPM_H_
#define _QSPI_W25Q256JWPM_H_

#ifndef WINBOND_ID
#define WINBOND_ID      0xEF
#endif

// Device type using command 0x9F
#define W25Q256JWPM        0x80

#ifndef W25Q_256Mb_SIZE
#define W25Q_256Mb_SIZE   0x19
#endif

#include "qspi_common.h"
#include "sdk_defs.h"

#include "qspi_winbond.h"

#define W25Q256JWPM_POWER_DOWN_DELAY_US          3
#define W25Q256JWPM_RELEASE_POWER_DOWN_DELAY_US  200
#define W25Q256JWPM_POWER_UP_DELAY_US            20

static void flash_w25q256jwpm_sys_clock_cfg(HW_QSPIC_ID id, sys_clk_t sys_clk);
static uint8_t flash_w25q256jwpm_get_dummy_bytes(HW_QSPIC_ID id);
static void flash_w25q256jwpm_initialize(HW_QSPIC_ID id);

#if (dg_configFLASH_POWER_OFF == 1)
/**
 * \brief uCode for handling the QSPI FLASH activation from power off.
 */
        /*
         * Should work with all Winbond flashes -- verified with W25Q80EW.
         *
         * Delay 10usec
         * 0x01   // CMD_NBYTES = 0, CMD_TX_MD = 0 (Single), CMD_VALID = 1
         * 0xA0   // CMD_WT_CNT_LS = 160 --> 10000 / 62.5 = 160 = 10usec
         * 0x00   // CMD_WT_CNT_MS = 0
         * Exit from Fast Read mode
         * 0x09   // CMD_NBYTES = 1, CMD_TX_MD = 0 (Single), CMD_VALID = 1
         * 0x00   // CMD_WT_CNT_LS = 0
         * 0x00   // CMD_WT_CNT_MS = 0
         * 0xFF   // Enable Reset
         * (up to 16 words)
         */
        const uint32_t w25q256jwpm_ucode_wakeup[] = {
                0x09000001 | (((uint16_t)(W25Q256JWPM_POWER_UP_DELAY_US*1000/31.25) & 0xFFFF) << 8),
                0x00FF0000,
        };
#elif (dg_configFLASH_POWER_DOWN == 1)
/**
 * \brief uCode for handling the QSPI FLASH release from power-down.
 */
        /*
         * Should work with all Winbond flashes -- verified with W25Q80EW.
         *
         * 0x09   // CMD_NBYTES = 1, CMD_TX_MD = 0 (Single), CMD_VALID = 1
         * 0x30   // CMD_WT_CNT_LS = 3000 / 62.5 = 48 // 3usec
         * 0x00   // CMD_WT_CNT_MS = 0
         * 0xAB   // Release Power Down
         * (up to 16 words)
         */
        const uint32_t w25q256jwpm_ucode_wakeup[] = {
                0xAB000009 | (((uint16_t)(W25Q256JWPM_RELEASE_POWER_DOWN_DELAY_US*1000/31.25) & 0xFFFF) << 8),
        };
#else
/**
 * \brief uCode for handling the QSPI FLASH exit from the "Continuous Read Mode".
 */
        /*
         * 0x2D   // CMD_NBYTES = 5, CMD_TX_MD = 2 (Quad), CMD_VALID = 1
         * 0x00   // CMD_WT_CNT_LS = 0
         * 0x00   // CMD_WT_CNT_MS = 0
         * 0x55   // Clocks 0-1 (A31-24)
         * 0x55   // Clocks 0-1 (A23-16)
         * 0x55   // Clocks 2-3 (A15-8)
         * 0x55   // Clocks 4-5 (A7-0)
         * 0xFF   // Clocks 6-7 (M7-0) : M5-4 != '11' ==> Disable "Continuous Read Mode"
         * (up to 16 words)
         */
        const uint32_t w25q256jwpm_ucode_wakeup[] = {
                0x5500002D,
                0x55555555,
                0x000000FF,
        };
#endif


static const qspi_flash_config_t flash_w25q256jwpm_config = {
        .manufacturer_id                  = WINBOND_ID,
        .device_type                      = W25Q256JWPM,
        .device_density                   = W25Q_256Mb_SIZE,
        .is_suspended                     = flash_w25q_is_suspended,
        .initialize                       = flash_w25q256jwpm_initialize,
        .sys_clk_cfg                      = flash_w25q256jwpm_sys_clock_cfg,
        .get_dummy_bytes                  = flash_w25q256jwpm_get_dummy_bytes,
        .break_seq_size                   = HW_QSPI_BREAK_SEQ_SIZE_2B,
        .address_size                     = HW_QSPI_ADDR_SIZE_32,
        .read_instr                       = CMD_FAST_READ_QUAD_4B,
        .read_addr_phaze                  = HW_QSPI_BUS_MODE_QUAD,
        .page_program_opcode              = CMD_QUAD_PAGE_PROGRAM_4B,
        .page_qpi_program_opcode          = CMD_QPI_PAGE_PROGRAM,
        .quad_page_program_address        = false,
        .erase_opcode                     = CMD_SECTOR_ERASE_4B,
        .erase_suspend_opcode             = W25Q_ERASE_PROGRAM_SUSPEND,
        .erase_resume_opcode              = W25Q_ERASE_PROGRAM_RESUME,
        .read_erase_progress_opcode       = CMD_READ_STATUS_REGISTER,
        .erase_in_progress_bit            = FLASH_STATUS_BUSY_BIT,
        .erase_in_progress_bit_high_level = true,
        .send_once                        = 1,
        .extra_byte                       = 0xA0,
        .send_extra_byte                  = true,
        .ucode_wakeup                     = {w25q256jwpm_ucode_wakeup, sizeof(w25q256jwpm_ucode_wakeup)},
        .power_down_delay                 = W25Q256JWPM_POWER_DOWN_DELAY_US,
        .release_power_down_delay         = W25Q256JWPM_RELEASE_POWER_DOWN_DELAY_US,
        .power_up_delay                   = W25Q256JWPM_POWER_UP_DELAY_US,
        .is_ram                           = false,
        .qpi_mode                         = false,
        .enter_qpi_opcode                 = CMD_ENTER_QPI_MODE,
        .memory_size                      = MEMORY_SIZE_256Mb,
};

__RETAINED_CODE static void flash_w25q256jwpm_initialize(HW_QSPIC_ID id)
{
        uint8_t cmd[2];

        flash_w25q_initialize(id);

        /* clear protection in register-1 */
        flash_write_enable(id);
        flash_write_status_register(id, flash_read_status_register(id) & 0x03);

        /* Set QE bit & clear any protection in register-2 */
        flash_write_enable(id);
        flash_w25q_write_status_register_2(id, 0x02);

        /* clear protection in register-3 */
        cmd[0] = 0x11;
        cmd[1] = 0x0;
        flash_write_enable(id);
        flash_write(id, cmd, 2);
        /* Wait for the Flash to process the command */
        while (flash_is_busy(id));

        /* set address mode */
        if (flash_w25q256jwpm_config.address_size == HW_QSPI_ADDR_SIZE_32) {
                cmd[0] = 0xB7; // enable 4B (32-bit) addressing
        } else {
                cmd[0] = 0xE9; // disable 4B (32-bit) addressing (24-bit default)
        }
        flash_write_enable(id);
        flash_write(id, cmd, 1);
        /* Wait for the Flash to process the command */
        while (flash_is_busy(id));
}

__RETAINED_CODE static void flash_w25q256jwpm_sys_clock_cfg(HW_QSPIC_ID id, sys_clk_t sys_clk)
{
        if (sys_clk == sysclk_PLL96){

              hw_qspi_set_read_pipe_clock_delay(id, 7);

              hw_qspi_set_div(id, HW_QSPI_DIV_4);

        } else {

              hw_qspi_set_read_pipe_clock_delay(id, 2);

              hw_qspi_set_div(id, HW_QSPI_DIV_2);

        }
}

__RETAINED_CODE static uint8_t flash_w25q256jwpm_get_dummy_bytes(HW_QSPIC_ID id)
{
        return 2;
}

#endif /* _QSPI_W25Q245JWPM_H_ */
/**
 * \}
 * \}
 */
