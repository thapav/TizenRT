/**
\addtogroup BSP
\{
\addtogroup DEVICES
\{
\addtogroup SYS
\{
*/

/**
****************************************************************************************
*
* @file hw_sys_da1469x.c
*
* @brief System Driver
*
* Copyright (C) 2017-2018 Dialog Semiconductor.
* This computer program includes Confidential, Proprietary Information
* of Dialog Semiconductor. All Rights Reserved.
*
****************************************************************************************
*/

#if dg_configUSE_HW_SYS

#include <stdint.h>
#include "hw_cpm.h"
#include "hw_clk.h"
#include "hw_pd.h"
#include "hw_sys.h"
#include "hw_gpio.h"
#include "sys_tcs_da1469x.h"

#define SW_CURSOR_GPIO                  *(SW_CURSOR_PORT == 0 ? \
                                                (SW_CURSOR_PIN == 0 ? &(GPIO->P0_00_MODE_REG) : \
                                                (SW_CURSOR_PIN == 1 ? &(GPIO->P0_01_MODE_REG) : \
                                                (SW_CURSOR_PIN == 2 ? &(GPIO->P0_02_MODE_REG) : \
                                                (SW_CURSOR_PIN == 3 ? &(GPIO->P0_03_MODE_REG) : \
                                                (SW_CURSOR_PIN == 4 ? &(GPIO->P0_04_MODE_REG) : \
                                                (SW_CURSOR_PIN == 5 ? &(GPIO->P0_05_MODE_REG) : \
                                                (SW_CURSOR_PIN == 6 ? &(GPIO->P0_06_MODE_REG) : \
                                                (SW_CURSOR_PIN == 7 ? &(GPIO->P0_07_MODE_REG) : \
                                                (SW_CURSOR_PIN == 8 ? &(GPIO->P0_08_MODE_REG) : \
                                                (SW_CURSOR_PIN == 9 ? &(GPIO->P0_09_MODE_REG) : \
                                                (SW_CURSOR_PIN == 10 ? &(GPIO->P0_10_MODE_REG) : \
                                                (SW_CURSOR_PIN == 11 ? &(GPIO->P0_11_MODE_REG) : \
                                                (SW_CURSOR_PIN == 12 ? &(GPIO->P0_12_MODE_REG) : \
                                                (SW_CURSOR_PIN == 13 ? &(GPIO->P0_13_MODE_REG) : \
                                                (SW_CURSOR_PIN == 14 ? &(GPIO->P0_14_MODE_REG) : \
                                                (SW_CURSOR_PIN == 15 ? &(GPIO->P0_15_MODE_REG) : \
                                                (SW_CURSOR_PIN == 16 ? &(GPIO->P0_16_MODE_REG) : \
                                                (SW_CURSOR_PIN == 17 ? &(GPIO->P0_17_MODE_REG) : \
                                                (SW_CURSOR_PIN == 18 ? &(GPIO->P0_18_MODE_REG) : \
                                                (SW_CURSOR_PIN == 19 ? &(GPIO->P0_19_MODE_REG) : \
                                                (SW_CURSOR_PIN == 20 ? &(GPIO->P0_20_MODE_REG) : \
                                                (SW_CURSOR_PIN == 21 ? &(GPIO->P0_21_MODE_REG) : \
                                                (SW_CURSOR_PIN == 22 ? &(GPIO->P0_22_MODE_REG) : \
                                                (SW_CURSOR_PIN == 23 ? &(GPIO->P0_23_MODE_REG) : \
                                                (SW_CURSOR_PIN == 24 ? &(GPIO->P0_24_MODE_REG) : \
                                                (SW_CURSOR_PIN == 25 ? &(GPIO->P0_25_MODE_REG) : \
                                                (SW_CURSOR_PIN == 26 ? &(GPIO->P0_26_MODE_REG) : \
                                                (SW_CURSOR_PIN == 27 ? &(GPIO->P0_27_MODE_REG) : \
                                                (SW_CURSOR_PIN == 28 ? &(GPIO->P0_28_MODE_REG) : \
                                                (SW_CURSOR_PIN == 29 ? &(GPIO->P0_29_MODE_REG) : \
                                                (SW_CURSOR_PIN == 30 ? &(GPIO->P0_30_MODE_REG) : \
                                                                       &(GPIO->P0_31_MODE_REG)))))))))))))))))))))))))))))))) \
                                                : \
                                                (SW_CURSOR_PIN == 0 ? &(GPIO->P1_00_MODE_REG) : \
                                                (SW_CURSOR_PIN == 1 ? &(GPIO->P1_01_MODE_REG) : \
                                                (SW_CURSOR_PIN == 2 ? &(GPIO->P1_02_MODE_REG) : \
                                                (SW_CURSOR_PIN == 3 ? &(GPIO->P1_03_MODE_REG) : \
                                                (SW_CURSOR_PIN == 4 ? &(GPIO->P1_04_MODE_REG) : \
                                                (SW_CURSOR_PIN == 5 ? &(GPIO->P1_05_MODE_REG) : \
                                                (SW_CURSOR_PIN == 6 ? &(GPIO->P1_06_MODE_REG) : \
                                                (SW_CURSOR_PIN == 7 ? &(GPIO->P1_07_MODE_REG) : \
                                                (SW_CURSOR_PIN == 8 ? &(GPIO->P1_08_MODE_REG) : \
                                                (SW_CURSOR_PIN == 9 ? &(GPIO->P1_09_MODE_REG) : \
                                                (SW_CURSOR_PIN == 10 ? &(GPIO->P1_10_MODE_REG) : \
                                                (SW_CURSOR_PIN == 11 ? &(GPIO->P1_11_MODE_REG) : \
                                                (SW_CURSOR_PIN == 12 ? &(GPIO->P1_12_MODE_REG) : \
                                                (SW_CURSOR_PIN == 13 ? &(GPIO->P1_13_MODE_REG) : \
                                                (SW_CURSOR_PIN == 14 ? &(GPIO->P1_14_MODE_REG) : \
                                                (SW_CURSOR_PIN == 15 ? &(GPIO->P1_15_MODE_REG) : \
                                                (SW_CURSOR_PIN == 16 ? &(GPIO->P1_16_MODE_REG) : \
                                                (SW_CURSOR_PIN == 17 ? &(GPIO->P1_17_MODE_REG) : \
                                                (SW_CURSOR_PIN == 18 ? &(GPIO->P1_18_MODE_REG) : \
                                                (SW_CURSOR_PIN == 19 ? &(GPIO->P1_19_MODE_REG) : \
                                                (SW_CURSOR_PIN == 20 ? &(GPIO->P1_20_MODE_REG) : \
                                                (SW_CURSOR_PIN == 21 ? &(GPIO->P1_21_MODE_REG) : \
                                                                       &(GPIO->P1_22_MODE_REG))))))))))))))))))))))))

#define SW_CURSOR_SET                   *(SW_CURSOR_PORT == 0 ? &(GPIO->P0_SET_DATA_REG) :    \
                                                                &(GPIO->P1_SET_DATA_REG))

#define SW_CURSOR_RESET                 *(SW_CURSOR_PORT == 0 ? &(GPIO->P0_RESET_DATA_REG) :  \
                                                                &(GPIO->P1_RESET_DATA_REG))

#define SW_CURSOR_SET_PAD_LATCH         *(SW_CURSOR_PORT == 0 ? &(CRG_TOP->P0_SET_PAD_LATCH_REG) :  \
                                                                &(CRG_TOP->P1_SET_PAD_LATCH_REG))

#define SW_CURSOR_RESET_PAD_LATCH       *(SW_CURSOR_PORT == 0 ? &(CRG_TOP->P0_RESET_PAD_LATCH_REG) :  \
                                                                &(CRG_TOP->P1_RESET_PAD_LATCH_REG))

#define NUM_OF_REG_CONFIG_ENTRIES       5

/*
 * Indicates which master has currently access to a certain peripheral.
 * hw_sys_sw_bsr[periph_id] == BSR_MASTER_X: Indicates that master X has currently access to that peripheral.
 * This allows for more efficient calculations on SNC side.
 */
__RETAINED uint32_t hw_sys_sw_bsr[BSR_PERIPH_ID_MAX];
__RETAINED static uint8_t hw_sys_sw_bsr_cnt[BSR_PERIPH_ID_MAX];

__RETAINED static hw_sys_reg_config_t hw_sys_reg_config[NUM_OF_REG_CONFIG_ENTRIES];
__RETAINED static uint32_t hw_sys_reg_num_of_config_entries;

__RETAINED uint32_t hw_sys_pd_com_acquire_cnt;
__RETAINED uint32_t hw_sys_pd_periph_acquire_cnt;

void hw_sys_set_preferred_values(void)
{
#if (dg_configENABLE_DA1469x_AA_SUPPORT)
        /* Workaround for bug2522AA_074: LDO output drift upwards on hold mode
         * Set BG_REFRESH_INTERVAL to 0x80 to overcome VDD_LDO_RET issue
         */
        REG_SETF(CRG_TOP, PMU_SLEEP_REG, BG_REFRESH_INTERVAL, 0x80);
#endif
        /* Apply default values*/
        /* PD_AON. Apply if no CS value is set by the booter */
        if (CRG_TOP->PMU_TRIM_REG == 0) {
                CRG_TOP->PMU_TRIM_REG = DEFAULT_PMU_TRIM_REG;
        }

        /* Apply preferred settings */
        REG_SET_BIT(CRG_TOP, BANDGAP_REG, BANDGAP_ENABLE_CLAMP);
        CRG_TOP->BIAS_VREF_SEL_REG = 0x000000CA;
        REG_SETF(CRG_TOP, BOD_LVL_CTRL0_REG, BOD_LVL_V30, 0x11F);
}

void hw_sys_setup_sw_cursor(void)
{
        if (dg_configUSE_SW_CURSOR == 1) {
                hw_sys_pd_com_enable();
                hw_gpio_set_pin_function(SW_CURSOR_PORT, SW_CURSOR_PIN, HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO);
                hw_gpio_pad_latch_enable(SW_CURSOR_PORT, SW_CURSOR_PIN);
                hw_gpio_pad_latch_disable(SW_CURSOR_PORT, SW_CURSOR_PIN);
                hw_sys_pd_com_disable();
        }
}

void hw_sys_trigger_sw_cursor(void)
{
        if (dg_configUSE_SW_CURSOR == 1) {
                hw_sys_pd_com_enable();
                hw_gpio_configure_pin(SW_CURSOR_PORT, SW_CURSOR_PIN, HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO, true);
                hw_gpio_pad_latch_enable(SW_CURSOR_PORT, SW_CURSOR_PIN);
                hw_clk_delay_usec(50);
                hw_gpio_set_pin_function(SW_CURSOR_PORT, SW_CURSOR_PIN, HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO);
                hw_gpio_pad_latch_disable(SW_CURSOR_PORT, SW_CURSOR_PIN);
                hw_sys_pd_com_disable();
        }
}

void hw_sys_assert_trigger_gpio(void)
{
        if (EXCEPTION_DEBUG == 1) {
                if (dg_configLP_CLK_SOURCE == LP_CLK_IS_DIGITAL) {
                        hw_clk_configure_ext32k_pins();
                }

                hw_pd_power_up_com();
                hw_gpio_pad_latch_enable_all();

                DBG_SET_HIGH(EXCEPTION_DEBUG, EXCEPTIONDBG);
        }
}


bool hw_sys_hw_bsr_try_lock(HW_BSR_MASTER_ID hw_bsr_master_id, HW_BSR_POS pos)
{
        ASSERT_ERROR((hw_bsr_master_id & SW_BSR_HW_BSR_MASK) == hw_bsr_master_id);
        ASSERT_WARNING((pos % 2) == 0);
        ASSERT_WARNING(pos < 31);

        MEMCTRL->BUSY_SET_REG = hw_bsr_master_id << pos;
        if (((MEMCTRL->BUSY_STAT_REG >> pos) & SW_BSR_HW_BSR_MASK) == hw_bsr_master_id) {
                return true;
        } else {
                return false;
        }
}

static HW_BSR_MASTER_ID hw_sys_sw_bsr_to_hw_bsr(SW_BSR_MASTER_ID sw_bsr_master_id) {
        switch (sw_bsr_master_id) {
        case SW_BSR_MASTER_SNC:
                return HW_BSR_MASTER_SNC;
                break;
        case SW_BSR_MASTER_SYSCPU:
                return HW_BSR_MASTER_SYSCPU;
                break;
        case SW_BSR_MASTER_CMAC:
                return HW_BSR_MASTER_CMAC;
                break;
        default:
                ASSERT_ERROR(0);
                return HW_BSR_MASTER_NONE;
                break;
        };
}

void hw_sys_hw_bsr_unlock(HW_BSR_MASTER_ID hw_bsr_master_id, HW_BSR_POS pos)
{
        ASSERT_ERROR((hw_bsr_master_id & SW_BSR_HW_BSR_MASK) == hw_bsr_master_id);
        ASSERT_ERROR(((MEMCTRL->BUSY_STAT_REG >> pos) & SW_BSR_HW_BSR_MASK) == hw_bsr_master_id);
        ASSERT_WARNING((pos % 2) == 0);
        ASSERT_WARNING(pos < 31);

        MEMCTRL->BUSY_RESET_REG = hw_bsr_master_id << pos;
}

void hw_sys_sw_bsr_init(void)
{
        int i;

        for (i = 0; i < sizeof(hw_sys_sw_bsr) / sizeof(hw_sys_sw_bsr[0]); i++) {
                hw_sys_sw_bsr[i] = SW_BSR_MASTER_NONE;
                hw_sys_sw_bsr_cnt[i] = 0;
        }

        MEMCTRL->BUSY_RESET_REG = MEMCTRL->BUSY_STAT_REG;
}

bool hw_sys_sw_bsr_try_acquire(SW_BSR_MASTER_ID sw_bsr_master_id, uint32_t periph_id)
{
        HW_BSR_MASTER_ID hw_bsr_master_id;
        bool acquired = false;

        ASSERT_ERROR (periph_id < BSR_PERIPH_ID_MAX);

        /*
         * Since hw_sys_hw_bsr_try_lock() / hw_sys_hw_bsr_unlock() sequence
         * can be executed from different contexts, the caller must
         * either disable interrupts or use a mutex to protect against
         * the following scenario:
         * -  Context1: hw_sys_hw_bsr_try_lock()
         * -  Context2: hw_sys_hw_bsr_try_lock()
         * -  Context2: hw_sys_hw_bsr_unlock()
         * -  Context1: HW BSR has been released before calling hw_sys_hw_bsr_unlock()
         */

        hw_bsr_master_id = hw_sys_sw_bsr_to_hw_bsr(sw_bsr_master_id);

        while (!hw_sys_hw_bsr_try_lock(hw_bsr_master_id, HW_BSR_SW_POS)) {}

        if ((hw_sys_sw_bsr[periph_id] == SW_BSR_MASTER_NONE) || (hw_sys_sw_bsr[periph_id] == sw_bsr_master_id)) {
                /* Update SW BSR */
                hw_sys_sw_bsr[periph_id] = sw_bsr_master_id;
                hw_sys_sw_bsr_cnt[periph_id]++;
                acquired = true;
        }

        hw_sys_hw_bsr_unlock(hw_bsr_master_id, HW_BSR_SW_POS);

        return acquired;
}

bool hw_sys_sw_bsr_acquired(SW_BSR_MASTER_ID sw_bsr_master_id, uint32_t periph_id)
{
        HW_BSR_MASTER_ID hw_bsr_master_id;
        bool acquired = false;

        hw_bsr_master_id = hw_sys_sw_bsr_to_hw_bsr(sw_bsr_master_id);

        while (!hw_sys_hw_bsr_try_lock(hw_bsr_master_id, HW_BSR_SW_POS)) {}

        if (hw_sys_sw_bsr[periph_id] == sw_bsr_master_id) {
                acquired = true;
        }

        hw_sys_hw_bsr_unlock(hw_bsr_master_id, HW_BSR_SW_POS);

        return acquired;
}

void hw_sys_sw_bsr_release(SW_BSR_MASTER_ID sw_bsr_master_id, uint32_t periph_id)
{
        HW_BSR_MASTER_ID hw_bsr_master_id;

        hw_bsr_master_id = hw_sys_sw_bsr_to_hw_bsr(sw_bsr_master_id);

        while (!hw_sys_hw_bsr_try_lock(hw_bsr_master_id, HW_BSR_SW_POS)) {}

        ASSERT_ERROR (hw_sys_sw_bsr[periph_id] == sw_bsr_master_id);
        ASSERT_ERROR (hw_sys_sw_bsr_cnt[periph_id]);

        if (--hw_sys_sw_bsr_cnt[periph_id] == 0) {
                hw_sys_sw_bsr[periph_id] = SW_BSR_MASTER_NONE;
        }

        hw_sys_hw_bsr_unlock(hw_bsr_master_id, HW_BSR_SW_POS);
}

__RETAINED_CODE void hw_sys_pd_com_enable(void)
{
        GLOBAL_INT_DISABLE();
        ASSERT_ERROR((!hw_sys_pd_com_acquire_cnt) || !REG_GETF(CRG_TOP, PMU_CTRL_REG, COM_SLEEP));
        ASSERT_ERROR((hw_sys_pd_com_acquire_cnt) || REG_GETF(CRG_TOP, PMU_CTRL_REG, COM_SLEEP));
        if (++hw_sys_pd_com_acquire_cnt == 1) {
                hw_pd_power_up_com();
        }
        GLOBAL_INT_RESTORE();

        ASSERT_ERROR(REG_GETF(CRG_TOP, SYS_STAT_REG, COM_IS_UP));
}

__RETAINED_CODE void hw_sys_pd_com_disable(void)
{
        ASSERT_ERROR(!REG_GETF(CRG_TOP, PMU_CTRL_REG, COM_SLEEP));

        GLOBAL_INT_DISABLE();
        ASSERT_ERROR(hw_sys_pd_com_acquire_cnt);
        if (--hw_sys_pd_com_acquire_cnt == 0) {
                hw_pd_power_down_com();
        }
        GLOBAL_INT_RESTORE();
}

__RETAINED_CODE void hw_sys_pd_periph_enable(void)
{
        GLOBAL_INT_DISABLE();
        if (++hw_sys_pd_periph_acquire_cnt == 1) {
                hw_pd_power_up_periph();
        }
        GLOBAL_INT_RESTORE();

        ASSERT_ERROR(REG_GETF(CRG_TOP, SYS_STAT_REG, PER_IS_UP));
}

__RETAINED_CODE void hw_sys_pd_periph_disable(void)
{
        ASSERT_ERROR(!REG_GETF(CRG_TOP, PMU_CTRL_REG, PERIPH_SLEEP));

        GLOBAL_INT_DISABLE();
        ASSERT_ERROR(hw_sys_pd_periph_acquire_cnt);
        if (--hw_sys_pd_periph_acquire_cnt == 0) {
                hw_pd_power_down_periph();
        }
        GLOBAL_INT_RESTORE();
}

uint32_t hw_sys_reg_add_config(hw_sys_reg_config_t *config, uint32_t num_of_entries)
{
       ASSERT_ERROR(hw_sys_reg_num_of_config_entries + num_of_entries <= NUM_OF_REG_CONFIG_ENTRIES);

       uint32_t ret = hw_sys_reg_num_of_config_entries;

       for (uint32_t i = 0; i < num_of_entries; i++) {
               hw_sys_reg_config[hw_sys_reg_num_of_config_entries + i] = config[i];
       }
       hw_sys_reg_num_of_config_entries += num_of_entries;

       return ret;
}

hw_sys_reg_config_t *hw_sys_reg_get_config(uint32_t index)
{
        ASSERT_WARNING(index == 0 || index < hw_sys_reg_num_of_config_entries);

        return &hw_sys_reg_config[index];
}

void hw_sys_reg_modify_config(uint32_t index, __IO uint32_t *addr, uint32_t value)
{
        ASSERT_ERROR(index < hw_sys_reg_num_of_config_entries);

        hw_sys_reg_config[index].value = value;

        // Address must be written after value to prevent race condition with other hosts
        hw_sys_reg_config[index].addr = addr;
}

uint32_t *hw_sys_reg_get_num_of_config_entries(void)
{
        return &hw_sys_reg_num_of_config_entries;
}

__RETAINED_CODE void hw_sys_reg_apply_config(void)
{
        uint32_t *p = (uint32_t *)&hw_sys_reg_config;

        while (p < (uint32_t *)&hw_sys_reg_config + 2 * hw_sys_reg_num_of_config_entries) {
                *(uint32_t *)(*p) = *(p+1);
                p += 2;
        }
}

#endif /* dg_configUSE_HW_SYS */

/**
\}
\}
\}
*/
