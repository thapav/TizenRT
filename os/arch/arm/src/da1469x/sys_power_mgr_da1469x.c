/**
 ****************************************************************************************
 *
 * @file sys_power_mgr_da1469x.c
 *
 * @brief Power Manager
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/**
 \addtogroup BSP
 \{
 \addtogroup SYSTEM
 \{
 \addtogroup POWER_MANAGER
 \{
 */


#include "sdk_defs.h"
#include "hw_clk.h"
#if dg_configUSE_HW_PMU
#       include "hw_bod.h"
#       include "hw_pmu.h"
#endif
#include "qspi_automode.h"
#include "hw_cpm.h"
#include "hw_otpc.h"
#include "hw_sys.h"
#include "hw_pd.h"
#include "hw_pdc.h"
#include "hw_rtc.h"
#include "sys_tcs.h"
#include "hw_watchdog.h"
#include "sys_clock_mgr.h"
#include "sys_clock_mgr_internal.h"
#include "sys_power_mgr.h"
#include "sys_power_mgr_internal.h"
#include "sys_watchdog.h"
#include "sys_watchdog_internal.h"
#ifdef SEC_MODEN
#include "hw_gpio.h"
#include "hw_wkup_da1469x.h"
#include "system.h"
#include "boot_info.h"
#endif

#if (dg_configUSE_HW_MPU == 1)
#include "hw_mpu.h"
typedef struct __MPU_status
{
        __IOM uint32_t CTRL;                   /* MPU Control Register */
        __IOM uint32_t RNR;                    /* MPU Region Number Register */
        __IOM uint32_t RBAR;                   /* MPU Region Base Address Register */
        __IOM uint32_t RLAR;                   /* MPU Region Limit Address Register */
        __IOM uint32_t RBAR_A1;                /* MPU Region Base Address Register Alias 1 */
        __IOM uint32_t RLAR_A1;                /* MPU Region Limit Address Register Alias 1 */
        __IOM uint32_t RBAR_A2;                /* MPU Region Base Address Register Alias 2 */
        __IOM uint32_t RLAR_A2;                /* MPU Region Limit Address Register Alias 2 */
        __IOM uint32_t RBAR_A3;                /* MPU Region Base Address Register Alias 3 */
        __IOM uint32_t RLAR_A3;                /* MPU Region Limit Address Register Alias 3 */
        __IOM uint32_t MAIR0;                  /* MPU Memory Attribute Indirection Register 0 */
        __IOM uint32_t MAIR1;                  /* MPU Memory Attribute Indirection Register 1 */
}MPU_status_save;

__RETAINED_UNINIT MPU_status_save MPU_status;
#endif

#undef OS_FREERTOS
#ifdef OS_FREERTOS
#       include "osal.h"
#       include "sys_charger.h"
#       include "sys_tcs.h"
#       include "sys_trng.h"
#       include "sys_socf.h"
#       include "sys_timer.h"
#       include "sys_timer_internal.h"
#       include "sys_bsr.h"
#       include "hw_gpadc.h"
#       include "hw_lcdc.h"
#       include "hw_usb.h"
#       include "hw_usb_charger.h"
#       include "hw_dma.h"
#       include "resmgmt.h"
#       if dg_configPMU_ADAPTER
#               include "ad_pmu.h"
#       endif
#       if (USE_BLE_SLEEP == 1)
#               include "ad_ble.h"
#       endif
#endif

#define PM_ENABLE_PD_COM_WHILE_ACTIVE   (1)
#define PM_ENABLE_SLEEP_DIAGNOSTICS     (0)
#define PM_SLEEP_MODE_REQUEST_THRESHOLD (64)
#define PM_SLEEP_MODE_CNT_OFFSET        (5)

#undef dg_configUSE_HW_GPADC
#define dg_configUSE_HW_GPADC 0

#ifdef OS_FREERTOS
        bool goto_deepsleep(void);

#       if (PWR_MGR_DEBUG == 1)
#               pragma message "Power Manager: Debugging mode is on!"
#       endif

#       if ((PWR_MGR_FUNCTIONAL_DEBUG == 1) || (PWR_MGR_USE_TIMING_DEBUG == 1))
#               pragma message "Power Manager: GPIO Debugging is on!"
#       endif

#       if (FLASH_DEBUG == 1)
#               pragma message "Flash: Flash Debugging is on!"
#       endif

#       define GUARD_TIME                              64      // in slots

// Make sure that sleep time can be measured by the sleep timer
#       if (dg_configUSE_LP_CLK == LP_CLK_RCX)
#               define MAX_TIMER_IDLE_COUNT  (LP_CNT_NATIVE_MASK + 1 - GUARD_TIME * 32)
#       else
#               define MAX_TIMER_IDLE_COUNT  (LP_CNT_NATIVE_MASK + 1 - GUARD_TIME * TICK_PERIOD)
#       endif

// Watchdog is active during sleep. The system must wake-up before the watchdog expires to reload its counter
#       if (dg_configUSE_LP_CLK == LP_CLK_RCX)
                // When LP clock is RCX, the watchdog is clocked by RCX clock divided by 320
#               define MAX_WDOG_TIMER_IDLE_COUNT  (dg_configWDOG_IDLE_RESET_VALUE * 320 - GUARD_TIME * 32)
#       else
                // When LP clock is not RCX, the watchdog is clocked by RC32K divided by 320. In this case the IDLE count
                // must be calculated according to dg_configXTAL32K_FREQ and dg_configRC32K_FREQ.
                // The multiplier of 320 has been split to 16 and 20 to avoid overflow during the execution of the operations.
#               define MAX_WDOG_TIMER_IDLE_COUNT  ((dg_configWDOG_IDLE_RESET_VALUE * \
                                                    16 * dg_configXTAL32K_FREQ / dg_configRC32K_FREQ) * 20 - \
                                                    GUARD_TIME * TICK_PERIOD)
#       endif

#       if (dg_configUSE_LP_CLK != LP_CLK_RCX)
#               define MAX_IDLE_TICKS_ALLOWED        ((MAX_TIMER_IDLE_COUNT / TICK_PERIOD) + 5)
#       endif

/*
 * Global and / or retained variables
 */
        __RETAINED static uint8_t sleep_mode_cnt2[pm_mode_sleep_max];
        __RETAINED static periph_init_cb periph_init;
        __RETAINED static OS_MUTEX xSemaphorePM;
        __RETAINED static system_state_t system_sleeping;
        __RETAINED static bool sleep_is_blocked;

        __RETAINED static sleep_mode_t current_sleep_mode;
        __RETAINED static sleep_mode_t user_sleep_mode;
        __RETAINED static uint8_t sleep_mode_cnt[pm_mode_sleep_max];
        __RETAINED static bool wakeup_mode_is_XTAL32;   // false: RC32, true: XTAL32M/PLL
        __RETAINED static adapter_call_backs_t *adapters_cb[dg_configPM_MAX_ADAPTERS_CNT];
        __RETAINED static uint8_t sleep_mode_cnt3[pm_mode_sleep_max];

#       if (PWR_MGR_DEBUG == 1)
        __RETAINED uint32_t low_power_periods_ret;
        __RETAINED uint32_t sleep_period_ret;
        __RETAINED uint32_t trigger_setting_ret;
        __RETAINED uint32_t lp_time1_ret;  // Power-up: start
        __RETAINED uint32_t lp_time2_ret;  // Power-up: after the activation of the Power Domains
        __RETAINED uint32_t lp_time3_ret;  // Power-up: after the clock setting - finish
#       endif

#endif /* OS_FREERTOS */

#ifdef OS_FREERTOS
/*
 * Local variables
 */

static uint32_t ulTimeSpentSleepingInTicks;                    // Counts in ticks

static bool adapters_wake_up_ind_called = false;
static bool call_adapters_xtal16m_ready_ind = false;

static uint64_t sleep_blocked_until = 0;

#if dg_configENABLE_DEBUGGER
/* Contains the index of the combo trigger PDC LUT entry */
__RETAINED static uint32_t jtag_wkup_combo_pdc_entry_idx;
/* Indicates when sleep blocking due to wake-up from JTAG ends. */
static uint64_t jtag_wkup_sleep_blocked_until = 0;
#endif

/*
 * Forward declarations
 */

/*
 * \brief When resuming the OS, check if we were sleeping and calculate the time we slept.
 *
 * \warning Must be called with interrupts disabled!
 */
__RETAINED_CODE static void sleep_exit(void);

#if dg_configENABLE_DEBUGGER
static void jtag_wkup_discover_combo_pdc_entry_idx(void);
__RETAINED_CODE static void jtag_wkup_check(bool sys_cpu_entered_sleep);
__STATIC_INLINE bool jtag_wkup_delay_has_expired(void);
#endif

/*
 * Function definitions
 */

static void init_component(const comp_init_tree_t **done, int *done_cnt,
                                                                const comp_init_tree_t *for_init)
{
        int i;

        /* Look in done array to see if for_init component was already initialized. */
        for (i = 0; i < *done_cnt; ++i) {
                if (done[i] == for_init) {
                        return;
                }
        }

        /* If not found, component was not initialized yet */

        /* Initialized dependencies first */
        for (i = 0; for_init->depend && for_init->depend[i]; ++i) {
                init_component(done, done_cnt, for_init->depend[i]);
        }
        /* Mark as initialized and call initialization function */
        done[*done_cnt] = for_init;
        ++(*done_cnt);
        if (for_init->init_fun) {
                for_init->init_fun(for_init->init_arg);
        }
}

static void init_components(const comp_init_tree_t **init, int num)
{
        /* Array of already initialized components */
        const comp_init_tree_t *done[num];
        /* Number of already initialized components */
        int done_cnt = 0;

        while (num) {
                init_component(done, &done_cnt, *init);
                ++init;
                --num;
        }
}

/* Symbols generated by linker */
extern const comp_init_tree_t *__start_adapter_init_section;
extern const comp_init_tree_t *__stop_adapter_init_section;
/*
 * In case there are no adapters defined make sure that 0 size linker section is generated.
 */
static const int adapter_init_section_marker[0] __attribute__((section("adapter_init_section")))
                                                                        __USED = {};

static void init_adapters(void)
{
        const comp_init_tree_t **init = &__start_adapter_init_section;
        /* Linker will provide those two symbols so number of adapter can be computed */
        const int num = &__stop_adapter_init_section - &__start_adapter_init_section;

        /*
         * Make sure there is no error in configuration. If this assert fires it probably
         * means that there are too many adapters declared with ADAPTER_INIT macros or
         * there is something else put in adapter_init_section.
         */
        ASSERT_ERROR(num <= dg_configPM_MAX_ADAPTERS_CNT);

        init_components(init, num);
}

/* Symbols generated by linker */
extern const comp_init_tree_t *__start_bus_init_section;
extern const comp_init_tree_t *__stop_bus_init_section;
/*
 * In case there are no buses defined make sure that 0 size linker section is generated.
 */
static const int bus_init_section_marker[0] __attribute__((section("bus_init_section")))
                                                                        __USED = {};

static void init_buses(void)
{
        init_components(&__start_bus_init_section,
                                        &__stop_bus_init_section - &__start_bus_init_section);
}

/* Symbols generated by linker */
extern const comp_init_tree_t *__start_device_init_section;
extern const comp_init_tree_t *__stop_device_init_section;
/*
 * In case there are no devices defined make sure that 0 size linker section is generated.
 */
static const int device_init_section_marker[0] __attribute__((section("device_init_section")))
                                                                        __USED = {};

static void init_devices(void)
{
        init_components(&__start_device_init_section,
                                &__stop_device_init_section - &__start_device_init_section);
}

static void xtalm_ready_cb(void)
{
        // Inform Adapters.
        adapter_call_backs_t *p_Ad;

        if (adapters_wake_up_ind_called) {
                for (int i = 0; i < dg_configPM_MAX_ADAPTERS_CNT; i++) {
                        p_Ad = adapters_cb[i];
                        if ((p_Ad != NULL) && (p_Ad->ad_xtalm_ready_ind != NULL)) {
                                p_Ad->ad_xtalm_ready_ind();
                        }
                }
        } else {
                call_adapters_xtal16m_ready_ind = true;
        }
}

#if (dg_configSYS_MANUAL_DUMP == 1)
extern void Manual_HandlerC(void);
#endif

#if (PM_ENABLE_SLEEP_DIAGNOSTICS == 1)
static void pm_enable_sleep_diagnostics(void)
{
        /*
         * Enable mapping of sleep diagnostic signals to GPIOs
         *      BANDGAP_ENABLE  -> P0_25
         *      WOKENUP         -> P0_16
         */
        hw_sys_pd_com_enable();

        hw_gpio_set_pin_function(HW_GPIO_PORT_0, HW_GPIO_PIN_16, HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);
        hw_gpio_set_pin_function(HW_GPIO_PORT_0, HW_GPIO_PIN_25, HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);

        // Enable sleep diagnostics
        REG_SETF(CRG_TOP, PMU_CTRL_REG, MAP_BANDGAP_EN, 1);

        hw_gpio_pad_latch_enable(HW_GPIO_PORT_0, HW_GPIO_PIN_16);
        hw_gpio_pad_latch_enable(HW_GPIO_PORT_0, HW_GPIO_PIN_25);
        hw_gpio_pad_latch_disable(HW_GPIO_PORT_0, HW_GPIO_PIN_16);
        hw_gpio_pad_latch_disable(HW_GPIO_PORT_0, HW_GPIO_PIN_25);

        hw_sys_pd_com_disable();
}
#endif /* (PM_ENABLE_SLEEP_DIAGNOSTICS == 1) */

void pm_system_init(periph_init_cb peripherals_initialization)
{
        /* Time to copy the image must be less than the minimum sleep time */
        ASSERT_WARNING(dg_configIMAGE_COPY_TIME < dg_configMIN_SLEEP_TIME);

        periph_init = peripherals_initialization;

#if (PM_ENABLE_PD_COM_WHILE_ACTIVE == 1)
        hw_sys_pd_com_enable();
        sys_tcs_apply_reg_pairs(SYS_TCS_GROUP_PD_COMM);
#endif
//        if ((sys_tcs_xtal32m_settling_time == 0) || (dg_configLP_CLK_SOURCE == LP_CLK_IS_ANALOG) && (dg_configUSE_LP_CLK == LP_CLK_RCX)) {
//                pm_wakeup_xtal16m_time = dg_configWAKEUP_XTAL32_TIME;
//        } else {
//                pm_wakeup_xtal16m_time = sys_tcs_xtal32m_settling_time + dg_configWAKEUP_RC32_TIME;
//        }

        hw_sys_setup_retmem();                          // Set retmem mode

#if dg_configUSE_HW_OTPC
        if (dg_configCODE_LOCATION != NON_VOLATILE_IS_OTP) {
                hw_otpc_disable();                      // Make sure OTPC is off
        }
#endif

#if (PM_ENABLE_SLEEP_DIAGNOSTICS == 1)
        pm_enable_sleep_diagnostics();
#endif /* (PM_ENABLE_SLEEP_DIAGNOSTICS == 1) */

        hw_sys_set_cache_retained();                    // Set cache in retained mode

        if (dg_configUSE_SW_CURSOR == 1) {
                hw_sys_setup_sw_cursor();
        }

        if (periph_init != NULL) {
                periph_init();                          // Power on peripherals and GPIOs
        }

#if CONFIG_LONG_PRESS_POWER_ON == ( 1 )
        check_long_press_power_on();
#endif

#if (dg_configSYS_MANUAL_DUMP == 1)
        hw_watchdog_freeze();
        char result=check_manual_dump();
        printf("Manual Dump=%d\r\n",result);
        if ( get_dump_mode() ) {
            printf("call manual dump but reset was already happened!\r\n");
        } else {
            if (result) {
                set_dump_mode(1);
                Manual_HandlerC();
                printf("set RESET_REASON_FAULT_MANUAL\r\n");
            }
        }
        hw_watchdog_unfreeze();
#endif

        DBG_CONFIGURE_HIGH(PWR_MGR_FUNCTIONAL_DEBUG, PWRDBG_POWERUP);
        DBG_CONFIGURE_HIGH(PWR_MGR_USE_TIMING_DEBUG, PWRDBG_SLEEP_EXIT);

        xSemaphorePM = xSemaphoreCreateMutex();         // Create Mutex
        ASSERT_WARNING(xSemaphorePM != NULL);

        /* Init system BSR */
        sys_sw_bsr_init();

        qspi_operations_init();
        resource_init();
        init_adapters();
        init_buses();
        init_devices();

        if (dg_configUSE_WDOG == 1) {
                hw_watchdog_unfreeze();                 // Start watchdog
        }
        else {
                hw_watchdog_freeze();                   // Stop watchdog
                sys_watchdog_set_pos_val(dg_configWDOG_IDLE_RESET_VALUE);
        }

        cm_register_xtal_ready_callback(xtalm_ready_cb);

#if dg_configENABLE_DEBUGGER
        jtag_wkup_discover_combo_pdc_entry_idx();
#endif

        hw_gpio_set_pin_function(NFC_I2C_SDA_PORT, NFC_I2C_SDA_PIN, HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO);
		hw_gpio_pad_latch_enable(NFC_I2C_SDA_PORT, NFC_I2C_SDA_PIN);
        hw_gpio_pad_latch_disable(NFC_I2C_SDA_PORT, NFC_I2C_SDA_PIN);
}

void pm_wait_debugger_detach(sleep_mode_t mode)
{
        volatile bool loop = true;

        if (dg_configUSE_WDOG == 1) {
                hw_watchdog_freeze();                   // Stop watchdog
        }
        // Make sure that the debugger is detached else sleep won't be possible
        if (configUSE_TICKLESS_IDLE != 0) {
                if (mode != pm_mode_active) {
                        while (loop && hw_sys_is_debugger_attached()) {
                        }
                }
        }

        if (dg_configUSE_WDOG == 1) {
                hw_watchdog_unfreeze();                 // Start watchdog
        }
}

#if dg_configPMU_ADAPTER
void pm_set_1v8_state(bool state)
{
        ad_pmu_set_1v8_state(state);
}

bool pm_get_1v8_state(void)
{
        return ad_pmu_get_1v8_state();
}

#endif

void pm_set_wakeup_mode(bool wait_for_xtalm)
{
        ASSERT_WARNING(xSemaphorePM != NULL);

        OS_EVENT_WAIT(xSemaphorePM, OS_EVENT_FOREVER);    // Block forever

        wakeup_mode_is_XTAL32 = wait_for_xtalm;

        OS_EVENT_SIGNAL(xSemaphorePM);
}

bool pm_get_wakeup_mode(void)
{
        bool mode;

        ASSERT_WARNING(xSemaphorePM != NULL);

        OS_EVENT_WAIT(xSemaphorePM, OS_EVENT_FOREVER);    // Block forever

        mode = wakeup_mode_is_XTAL32;

        OS_EVENT_SIGNAL(xSemaphorePM);

        return mode;
}

static void pm_sleep_mode_update(void)
{
        /* Select the power mode with the highest priority */
        if ((sleep_mode_cnt[pm_mode_active] > PM_SLEEP_MODE_CNT_OFFSET) || (user_sleep_mode == pm_mode_active)) {
                current_sleep_mode = pm_mode_active;
        } else if ((sleep_mode_cnt[pm_mode_idle] > PM_SLEEP_MODE_CNT_OFFSET) || (user_sleep_mode == pm_mode_idle)) {
                current_sleep_mode = pm_mode_idle;
        } else if ((sleep_mode_cnt[pm_mode_extended_sleep] > PM_SLEEP_MODE_CNT_OFFSET) || (user_sleep_mode == pm_mode_extended_sleep)) {
                current_sleep_mode = pm_mode_extended_sleep;
        } else {
                current_sleep_mode = user_sleep_mode;
        }
}

sleep_mode_t pm_sleep_mode_set(sleep_mode_t mode)
{
        sleep_mode_t previous_mode;

        ASSERT_ERROR(mode < pm_mode_sleep_max);
        GLOBAL_INT_DISABLE();

        previous_mode = current_sleep_mode;
        user_sleep_mode = mode;
        pm_sleep_mode_update();

        GLOBAL_INT_RESTORE();

        return previous_mode;
}

sleep_mode_t pm_sleep_mode_get(void)
{
        sleep_mode_t mode;

        GLOBAL_INT_DISABLE();

        mode = current_sleep_mode;

        GLOBAL_INT_RESTORE();

        return mode;
}

#ifdef CONFIG_USE_SEC_PM
#define SLEEP_MODE_CNT_WORKAROUND
void pm_sleep_mode_request(sleep_mode_t mode, wakeup_client_t client)
{
	int8_t cnt_diff = 0;

	ASSERT_ERROR(mode < pm_mode_sleep_max);
#ifndef SLEEP_MODE_CNT_WORKAROUND
	ASSERT_ERROR(sleep_mode_cnt[mode] < PM_SLEEP_MODE_REQUEST_THRESHOLD + PM_SLEEP_MODE_CNT_OFFSET &&
		sleep_mode_cnt[mode] == sleep_mode_cnt2[mode] + PM_SLEEP_MODE_CNT_OFFSET &&
		sleep_mode_cnt[mode] == sleep_mode_cnt3[mode] + PM_SLEEP_MODE_CNT_OFFSET);
#endif

	GLOBAL_INT_DISABLE();

	if (sleep_mode_cnt[mode] == 0)
		sleep_mode_cnt[mode] = PM_SLEEP_MODE_CNT_OFFSET;

#ifdef SLEEP_MODE_CNT_WORKAROUND
	// W/A: fix broken mode count
	if (sleep_mode_cnt2[mode] == sleep_mode_cnt3[mode] &&
			sleep_mode_cnt[mode] != sleep_mode_cnt2[mode] + PM_SLEEP_MODE_CNT_OFFSET) {
		if (sleep_mode_cnt[mode] > sleep_mode_cnt2[mode] + PM_SLEEP_MODE_CNT_OFFSET)
			cnt_diff = sleep_mode_cnt[mode] - (sleep_mode_cnt2[mode] + PM_SLEEP_MODE_CNT_OFFSET);
		else
			cnt_diff = sleep_mode_cnt2[mode] + PM_SLEEP_MODE_CNT_OFFSET - sleep_mode_cnt[mode];
		sleep_mode_cnt[mode] = sleep_mode_cnt2[mode] + PM_SLEEP_MODE_CNT_OFFSET;
	}
	if (sleep_mode_cnt[mode] >= PM_SLEEP_MODE_REQUEST_THRESHOLD + PM_SLEEP_MODE_CNT_OFFSET) {
		cnt_diff = sleep_mode_cnt[mode] - (PM_SLEEP_MODE_REQUEST_THRESHOLD + PM_SLEEP_MODE_CNT_OFFSET - 1);
		sleep_mode_cnt[mode] = PM_SLEEP_MODE_REQUEST_THRESHOLD + PM_SLEEP_MODE_CNT_OFFSET - 1;
	}
	sleep_mode_cnt2[mode] = sleep_mode_cnt[mode] - PM_SLEEP_MODE_CNT_OFFSET;
	sleep_mode_cnt3[mode] = sleep_mode_cnt[mode] - PM_SLEEP_MODE_CNT_OFFSET;
#endif

	sleep_mode_cnt2[mode]++;
	sleep_mode_cnt[mode]++;
	sleep_mode_cnt3[mode]++;
	pm_sleep_mode_update();

	pm_increase_client_cnt(mode, client, cnt_diff);

	GLOBAL_INT_RESTORE();

#ifndef SLEEP_MODE_CNT_WORKAROUND
	ASSERT_ERROR(sleep_mode_cnt[mode] == sleep_mode_cnt2[mode] &&
		sleep_mode_cnt[mode] == sleep_mode_cnt3[mode]);
#endif
}

void pm_sleep_mode_release(sleep_mode_t mode, wakeup_client_t client)
{
	int8_t cnt_diff = 0;

	ASSERT_ERROR(mode < pm_mode_sleep_max);
#ifndef SLEEP_MODE_CNT_WORKAROUND
	ASSERT_ERROR(sleep_mode_cnt[mode] > PM_SLEEP_MODE_CNT_OFFSET &&
		sleep_mode_cnt[mode] == sleep_mode_cnt2[mode] + PM_SLEEP_MODE_CNT_OFFSET &&
		sleep_mode_cnt[mode] == sleep_mode_cnt3[mode] + PM_SLEEP_MODE_CNT_OFFSET);
#endif

	GLOBAL_INT_DISABLE();

#ifdef SLEEP_MODE_CNT_WORKAROUND
	// W/A: fix broken mode count
	if (sleep_mode_cnt2[mode] == sleep_mode_cnt3[mode] &&
			sleep_mode_cnt[mode] != sleep_mode_cnt2[mode] + PM_SLEEP_MODE_CNT_OFFSET) {
		if (sleep_mode_cnt[mode] > sleep_mode_cnt2[mode] + PM_SLEEP_MODE_CNT_OFFSET)
			cnt_diff = sleep_mode_cnt[mode] - (sleep_mode_cnt2[mode] + PM_SLEEP_MODE_CNT_OFFSET);
		else
			cnt_diff = sleep_mode_cnt2[mode] + PM_SLEEP_MODE_CNT_OFFSET - sleep_mode_cnt[mode];
		sleep_mode_cnt[mode] = sleep_mode_cnt2[mode] + PM_SLEEP_MODE_CNT_OFFSET;
	}
	if (sleep_mode_cnt[mode] <= PM_SLEEP_MODE_CNT_OFFSET) {
		cnt_diff = PM_SLEEP_MODE_CNT_OFFSET + 1 - sleep_mode_cnt[mode];
		sleep_mode_cnt[mode] = PM_SLEEP_MODE_CNT_OFFSET + 1;
	}
	sleep_mode_cnt2[mode] = sleep_mode_cnt[mode] - PM_SLEEP_MODE_CNT_OFFSET;
	sleep_mode_cnt3[mode] = sleep_mode_cnt[mode] - PM_SLEEP_MODE_CNT_OFFSET;
#endif

	sleep_mode_cnt2[mode]--;
	sleep_mode_cnt[mode]--;
	sleep_mode_cnt3[mode]--;
	pm_sleep_mode_update();

	pm_decrease_client_cnt(mode, client, cnt_diff);

	GLOBAL_INT_RESTORE();

#ifndef SLEEP_MODE_CNT_WORKAROUND
	ASSERT_ERROR(sleep_mode_cnt[mode] == sleep_mode_cnt2[mode] &&
		sleep_mode_cnt[mode] == sleep_mode_cnt3[mode]);
#endif
}
#else
void pm_sleep_mode_request(sleep_mode_t mode)
{
        ASSERT_ERROR(mode < pm_mode_sleep_max);
        ASSERT_ERROR(sleep_mode_cnt[mode] < PM_SLEEP_MODE_REQUEST_THRESHOLD);

        GLOBAL_INT_DISABLE();

        sleep_mode_cnt[mode]++;
        pm_sleep_mode_update();

        GLOBAL_INT_RESTORE();
}

void pm_sleep_mode_release(sleep_mode_t mode)
{
        ASSERT_ERROR(mode < pm_mode_sleep_max);
        ASSERT_ERROR(sleep_mode_cnt[mode] != 0);

        GLOBAL_INT_DISABLE();

        sleep_mode_cnt[mode]--;
        pm_sleep_mode_update();

        GLOBAL_INT_RESTORE();
}
#endif
pm_id_t pm_register_adapter(const adapter_call_backs_t *cb)
{
        pm_id_t ret = -1;
        int i = 0;

        ASSERT_WARNING(xSemaphorePM != NULL);
        ASSERT_WARNING(cb != NULL);

        OS_EVENT_WAIT(xSemaphorePM, OS_EVENT_FOREVER);    // Block forever

        while ((i < dg_configPM_MAX_ADAPTERS_CNT) && (adapters_cb[i] != NULL)) {
                i++;
        }

        if (i < dg_configPM_MAX_ADAPTERS_CNT) {
                adapters_cb[i] = (adapter_call_backs_t *)cb;
                ret = i;
        }

        ASSERT_WARNING(ret != -1);                      // Increase dg_configPM_MAX_ADAPTERS_CNT

        OS_EVENT_SIGNAL(xSemaphorePM);

        return ret;
}

void pm_unregister_adapter(pm_id_t id)
{
        ASSERT_WARNING(xSemaphorePM != NULL);

        OS_EVENT_WAIT(xSemaphorePM, OS_EVENT_FOREVER);    // Block forever

        ASSERT_WARNING((id >= 0) && (id < dg_configPM_MAX_ADAPTERS_CNT));     // Is id valid?
        ASSERT_WARNING(adapters_cb[id] != NULL);     // Is it registered?

        adapters_cb[id] = NULL;

        OS_EVENT_SIGNAL(xSemaphorePM);
}

void pm_defer_sleep_for(pm_id_t id, uint32_t time_in_LP_cycles)
{
        uint64_t rtc_time;
        uint64_t lp_block_time;

        ASSERT_WARNING((id >= 0) && (id < dg_configPM_MAX_ADAPTERS_CNT));
        ASSERT_WARNING(adapters_cb[id] != NULL);                     // Is it registered?
        ASSERT_WARNING(time_in_LP_cycles <= dg_configPM_MAX_ADAPTER_DEFER_TIME);

        /*
         * Update Real Time Clock value
         */
        rtc_time = sys_timer_get_timestamp_fromISR();

        lp_block_time = rtc_time + time_in_LP_cycles;

        if (sleep_is_blocked == false) {
                sleep_blocked_until = lp_block_time;
                sleep_is_blocked = true;
        } else {
                // Update only if the new block time is after the previous one
                if (sleep_blocked_until < lp_block_time) {
                        sleep_blocked_until = lp_block_time;
                }
        }
}

/**
 * \brief Initialize the system after wake-up.
 *
 * \warning Called in ISR context after the settling of the XTAL.
 *
 */
static void sys_init_wake_up(void)
{
         int i;
         uint32_t iser = 0, iser2 = 0;
         adapter_call_backs_t *p_Ad;

 //        if (dg_configLP_CLK_SOURCE == LP_CLK_IS_DIGITAL) {
 //                 hw_clk_configure_ext32k_pins();
 //        }

         // No need to configure pins when XTAL32K is used
         //sys_tcs_apply(tcs_system);

         /*
          * Restore AHB, APB. If sys_clk is not RC32, it will be restored when the XTAL32
          * settles.
          */
         cm_sys_clk_sleep(false);

#if (dg_configUSE_HW_GPADC)
         uint64_t rtc;

         hw_gpadc_init(NULL);
         hw_gpadc_set_ldo_delay(0x0B);           // RC32 is the ADC clock (DIVN is 1:1).
         hw_gpadc_enable();
         hw_gpadc_set_ldo_constant_current(true);
         hw_gpadc_set_ldo_dynamic_current(true);
         hw_gpadc_set_input_mode(HW_GPADC_INPUT_MODE_SINGLE_ENDED);
         hw_gpadc_set_input(HW_GPADC_INPUT_SE_VDD);

         rtc = sys_timer_get_timestamp_fromISR();                // 22 usec delay is required!
#endif

         GLOBAL_INT_DISABLE();

         bool xtal32_ready = cm_poll_xtalm_ready();

         if (wakeup_mode_is_XTAL32 && !xtal32_ready) {
                 /* Mask interrupts again, since the Adapters or the periph_init() may call
                  * NVIC_EnableIRQ(). This is to ensure that no code other than the power manager
                  * code will be executed until the XTAL32M has settled, as the wake-up mode defines.
                  */
                 iser  = NVIC->ISER[0];                   // Log interrupt enable status (IRQs 00-31)
                 iser2 = NVIC->ISER[1];                   // Log interrupt enable status (IRQs 32-63)
         }

         // Inform Adapters
         adapters_wake_up_ind_called = true;

         if (periph_init != NULL) {
                 periph_init();                          // Power on peripherals and GPIOs
         }

         for (i = 0; i < dg_configPM_MAX_ADAPTERS_CNT; i++) {
                 p_Ad = adapters_cb[i];
                 if ((p_Ad != NULL) && (p_Ad->ad_wake_up_ind != NULL)) {
                         p_Ad->ad_wake_up_ind(false);
                 }
         }

         if (call_adapters_xtal16m_ready_ind) {
                 for (i = 0; i < dg_configPM_MAX_ADAPTERS_CNT; i++) {
                         p_Ad = adapters_cb[i];
                         if ((p_Ad != NULL) && (p_Ad->ad_xtalm_ready_ind != NULL)) {
                                 p_Ad->ad_xtalm_ready_ind();
                         }
                 }
         }

         if (wakeup_mode_is_XTAL32 && !xtal32_ready) {
                 /* Disable all interrupts except for the ones that were active when this function
                  * was entered. No adapter should activate an interrupt that is not normally active
                  * in the ad_wake_up_ind() call-back. All interrupts will be properly restored to
                  * the state they were before the system entered into the sleep mode.
                  */
                 NVIC->ICER[0] = ~iser;
                 NVIC->ICER[1] = ~iser2;
                 NVIC->ISER[0] = iser;
                 NVIC->ISER[1] = iser2;
         }

         GLOBAL_INT_RESTORE();

         DBG_CONFIGURE_HIGH(PWR_MGR_FUNCTIONAL_DEBUG, PWRDBG_TICK);
         DBG_CONFIGURE_LOW(PWR_MGR_FUNCTIONAL_DEBUG, PWRDBG_POWERUP);
         DBG_CONFIGURE_LOW(EXCEPTION_DEBUG, EXCEPTIONDBG);

         // A user definable macro that allows application code to be added.
         configPOST_SLEEP_PROCESSING();

#if (dg_configUSE_HW_GPADC)
         while ((rtc + 2) > sys_timer_get_timestamp_fromISR()) {
                 // 2 LP clocks are max ~60usec (XTAL32K) or 200usec (RCX)!
         }
         hw_gpadc_offset_calibrate();
#endif
}

#if (dg_configDISABLE_BACKGROUND_FLASH_OPS == 0)

__RETAINED_CODE static void execute_active_wfi(void)
{
        bool skip_wfi;

        // If an interrupt is already pending, do not sleep.
        if ((NVIC->ISER[0] & NVIC->ISPR[0]) || (NVIC->ISER[1] & NVIC->ISPR[1])) {
                DBG_SET_LOW(PWR_MGR_USE_TIMING_DEBUG, PWRDBG_SLEEP_ENTER);
                DBG_SET_HIGH(PWR_MGR_USE_TIMING_DEBUG, PWRDBG_SLEEP_EXIT);
                return;
        }

        skip_wfi = qspi_process_operations();

        /* Wait for an interrupt
         *
         * Any interrupt will cause an exit from WFI(). This is not a problem since even
         * if an interrupt other that the tick interrupt occurs before the next tick comes,
         * the only thing that should be done is to resume the scheduler. Since no tick has
         * occurred, the OS time will be the same.
         */
        DBG_SET_LOW(PWR_MGR_USE_TIMING_DEBUG, PWRDBG_SLEEP_ENTER);
        if (!skip_wfi) {
                __WFI();
        }
        DBG_SET_HIGH(PWR_MGR_USE_TIMING_DEBUG, PWRDBG_SLEEP_EXIT);

        qspi_check_and_suspend_operations();
}
#endif

/*
 * \brief When continuing from the WFI(), check if we were sleeping and, if so, power-up the system.
 *
 * \warning Must be called with interrupts disabled!
 *
 */
static __RETAINED_CODE void system_wake_up(void)
{
        /*
         * Check if it is a wake-up.
         */
        if (system_sleeping == sys_powered_down) {
                uint32_t iser, iser2;

                /*
                 * Disable all IRQs except for Timer2 and XTAL32M_RDY. Other ISRs may call OS routines
                 * that require the system timing to have been restored. Thus, their execution must
                 * be deferred until the call to the vTaskStepTick().
                 */
                iser  = NVIC->ISER[0];                   // Log interrupt enable status (IRQs 00-31)
                iser2 = NVIC->ISER[1];                   // Log interrupt enable status (IRQs 32-63)

                /*
                 * Disable all interrupts except those of Timer2 and XTAL32M
                 * XTAL32M_RDY_IRQn and TIMER2_IRQn are assumed to be in NVIC->ICER[0].
                 * If not then the following code must be modified accordingly.
                 */
                ASSERT_WARNING(TIMER2_IRQn < 32 && XTAL32M_RDY_IRQn < 32);

                NVIC->ICER[0] = iser & ~((uint32_t)(1 << TIMER2_IRQn) | (uint32_t)(1 << XTAL32M_RDY_IRQn));

                /*
                 * If the code stops at this point then the interrupts were enabled while they
                 * shouldn't be so.
                 */
                ASSERT_WARNING(__get_PRIMASK() == 1  || __get_BASEPRI() != 0);

                cm_enable_xtalm_if_required();

                cm_sys_clk_wakeup();

                /* Exit the critical section. */
                __enable_irq();

#if (PWR_MGR_DEBUG == 1)
                sys_timer_get_timestamp_fromCPM(&lp_time1_ret);
#endif

                sys_init_wake_up();

#if (PWR_MGR_DEBUG == 1)
                sys_timer_get_timestamp_fromCPM(&lp_time2_ret);
#endif

                cm_switch_to_xtalm_if_settled();

                if (wakeup_mode_is_XTAL32) {
                        /* Wait for XTAL32 and PLL, if required. */
                        cm_halt_until_sysclk_ready();
                }

#if (PWR_MGR_DEBUG == 1)
                sys_timer_get_timestamp_fromCPM(&lp_time3_ret);
#endif

                DBG_SET_LOW(PWR_MGR_FUNCTIONAL_DEBUG, PWRDBG_TICK);

                /*
                 * Determine how long the microcontroller was actually in a low power
                 * state. This time will be less than xExpectedIdleTime if the
                 * microcontroller was brought out of low power mode by an interrupt
                 * other than the sleep timer interrupt.
                 *
                 * >>> This information is provided by the Power Manager. <<<
                 *
                 * Note that the scheduler is suspended before
                 * portSUPPRESS_TICKS_AND_SLEEP() is called, and resumed when
                 * portSUPPRESS_TICKS_AND_SLEEP() returns.  Therefore no other tasks will
                 * execute until this function completes.
                 */

                /*
                 * Correct the kernel's tick count to account for the time the
                 * microcontroller spent in its low power state.
                 */
                sleep_exit();

                DBG_SET_LOW(PWR_MGR_USE_TIMING_DEBUG, PWRDBG_SLEEP_EXIT);

                /* Restore all interrupts */
                NVIC->ISER[0] = iser;
                NVIC->ISER[1] = iser2;
        } else {
                /* Correct the kernel's tick count to account for the time the
                 * microcontroller spent in its low power state.
                 */
                sleep_exit();

                DBG_SET_LOW(PWR_MGR_USE_TIMING_DEBUG, PWRDBG_SLEEP_EXIT);

                system_sleeping = sys_active;

                __enable_irq();
        }

        /* Notify blocked tasks, if appropriate. */
        qspi_process_completed_operations();
}

static void sleep_exit(void)
{
        if (system_sleeping != sys_active) {
                /*
                 * Calculate how long we've been idle or sleeping and update OS
                 */
                ulTimeSpentSleepingInTicks = sys_timer_update_slept_time();
		pm_update_sleep_time(ulTimeSpentSleepingInTicks);

#if (dg_configUSE_LP_CLK != LP_CLK_RCX)
                /* No more that MAX_IDLE_TICKS_ALLOWED ticks should have passed */
                ASSERT_WARNING(ulTimeSpentSleepingInTicks < MAX_IDLE_TICKS_ALLOWED);
#endif

                /*
                 * If the RCX is used and we've been idle for too long then force a
                 * re-calibration of the RCX to make sure that we keep track of the clock's changes.
                 */
                if ((dg_configLP_CLK_SOURCE == LP_CLK_IS_ANALOG) && (dg_configUSE_LP_CLK == LP_CLK_RCX)) {
                        if (system_sleeping == sys_idle) {
                                GLOBAL_INT_DISABLE();
                                if ((ulTimeSpentSleepingInTicks > 250) /*&& cm_lp_clk_is_avail()*/) {
                                        cm_calibrate_rcx_start();
                                }
                                GLOBAL_INT_RESTORE();
                        }
                }
                system_sleeping = sys_active;
        }
}

__RETAINED_CODE static void apply_wfi(bool allow_entering_sleep, uint32_t sleep_period)
{
        // XTAL32M_RDY_IRQn is assumed to be in NVIC->ISER[0].
        // If not then the following code must be modified accordingly.
        ASSERT_WARNING(XTAL32M_RDY_IRQn < 32);

        // If an interrupt (other than XTAL32M_RDY_IRQn) is already pending then do not sleep.
        if ((NVIC->ISER[0] & NVIC->ISPR[0] & ~(1UL << XTAL32M_RDY_IRQn)) || (NVIC->ISER[1] & NVIC->ISPR[1])) {
                if (allow_entering_sleep) {
                        int i;
                        adapter_call_backs_t *p_Ad;
                        /*
                         * Inform Adapters about the aborted sleep because pm_system_sleeping
                         * will be left to "sys_idle" and the "wake-up" path will not be followed.
                         */
                        for (i = dg_configPM_MAX_ADAPTERS_CNT - 1; i >= 0; i--) {
                                p_Ad = adapters_cb[i];
                                if ((p_Ad != NULL) && (p_Ad->ad_sleep_canceled != NULL)) {
                                        p_Ad->ad_sleep_canceled();
                                }
                        }
						if (!allow_entering_sleep) {
							pm_update_blocker_cnt(pm_sleep_blocker_pending_int);
						}
                }

                DBG_SET_LOW(PWR_MGR_USE_TIMING_DEBUG, PWRDBG_SLEEP_ENTER);
                DBG_SET_HIGH(PWR_MGR_USE_TIMING_DEBUG, PWRDBG_SLEEP_EXIT);

                return;
        }

        if (allow_entering_sleep) {
                /*
                 * Hooks
                 */
                if (sleep_period == 0) {
                        // A user definable macro that allows application code to be added.
                        configPRE_STOP_PROCESSING();
                } else {
                        // A user definable macro that allows application code to be added.
                        configPRE_SLEEP_PROCESSING( sleep_period );
                }

                /*
                 * Mark that a wake-up interrupt will be treated as such and not as a
                 * typical interrupt.
                 */
                system_sleeping = sys_powered_down;

                /*
                 * Switch to 32MHz system clock. AHB/APB dividers are set to 1.
                 */
                cm_sys_clk_sleep(true);

                DBG_SET_LOW(PWR_MGR_FUNCTIONAL_DEBUG, PWRDBG_POWERUP);

                // Watchdog is always active during sleep. Check if watchdog will expire during sleep
#if (dg_configUSE_LP_CLK == LP_CLK_RCX)
                ASSERT_WARNING(sleep_period < (sys_watchdog_get_val() * 320 - 10 * 32));
#else
                ASSERT_WARNING(sleep_period < ((sys_watchdog_get_val() * 16 * dg_configXTAL32K_FREQ / dg_configRC32K_FREQ) * 20 - 10 * TICK_PERIOD));
#endif

                pm_prepare_sleep(current_sleep_mode);
        } else {

                __UNUSED bool is_any_lld_active = false;
#if (dg_configUSE_HW_SENSOR_NODE == 1)
                is_any_lld_active = true;
#endif
#if (dg_configUSE_HW_DMA == 1)
                is_any_lld_active |= hw_dma_channel_active();
#endif
#if (dg_configUSE_HW_USB == 1)
                is_any_lld_active |= hw_usb_active();
#endif
#if (dg_configUSE_HW_LCDC == 1)
                is_any_lld_active |= hw_lcdc_is_active();
#endif
                if (!is_any_lld_active) {
                        // Lower the clocks to reduce power consumption.
                        cm_lower_all_clocks();
                }

                /*
                 * Hook to add any application specific code before entering Idle state.
                 */
                configPRE_IDLE_ENTRY( sleep_period );
        }

        DBG_CONFIGURE_LOW(PWR_MGR_USE_TIMING_DEBUG, PWRDBG_SLEEP_ENTER);

#if (dg_configIMAGE_SETUP == DEVELOPMENT_MODE)
        /*
         * If the code stops at this point then the interrupts were enabled while they
         * shouldn't be so.
         */
        ASSERT_WARNING(__get_PRIMASK() == 1 || __get_BASEPRI() != 0);
#endif

        /*
         * Sleep
         */
        if (allow_entering_sleep) {
                bool system_entered_sleep;

                if (NVIC_GetPendingIRQ(XTAL32M_RDY_IRQn)) {
                        /*
                         * If there is a pending XTAL32M_RDY_IRQnn clear it now to allow the system
                         * to go to sleep. When the system wakes-up, it will check if XTAL32M is
                         * settled and switch to XTAL32M without waiting for the interrupt.
                         */
                        NVIC_ClearPendingIRQ(XTAL32M_RDY_IRQn);
                }

#if (PM_ENABLE_PD_COM_WHILE_ACTIVE == 1)
                /* Disable PD COM if it is not needed during sleep */
                hw_sys_pd_com_disable();
#endif /* (PM_ENABLE_PD_COM_WHILE_ACTIVE == 1) */
                /* Disable PD PERIPH if it is not needed during sleep */
                hw_sys_pd_periph_disable();

#if (dg_configUSE_HW_MPU == 1)
                /* Save MPU state */
                MPU_status.CTRL = MPU->CTRL;
                MPU_status.RNR = MPU->RNR;
                MPU_status.RBAR = MPU->RBAR;
                MPU_status.RLAR = MPU->RLAR;
                MPU_status.RBAR_A1 = MPU->RBAR_A1;
                MPU_status.RLAR_A1 = MPU->RLAR_A1;
                MPU_status.RBAR_A2 = MPU->RBAR_A2;
                MPU_status.RLAR_A2 = MPU->RLAR_A2;
                MPU_status.RLAR_A3 = MPU->RLAR_A3;
                MPU_status.RBAR_A3 = MPU->RBAR_A3;
                MPU_status.MAIR0 = MPU->MAIR0;
                MPU_status.MAIR1 = MPU->MAIR1;
#endif

                system_entered_sleep = goto_deepsleep();

                if (system_entered_sleep) {
                        /*
                         *  Apply system non-retained register configuration.
                         *  DCDC configuration must be applied immediately after wake-up to reduce
                         *  current consumption.
                         */
                        hw_sys_reg_apply_config();

#if (dg_configUSE_HW_MPU == 1)
                        /* restore MPU */
                        hw_mpu_disable();
                        MPU->RNR = MPU_status.RNR;
                        MPU->RBAR = MPU_status.RBAR;
                        MPU->RLAR = MPU_status.RLAR;
                        MPU->RBAR_A1 = MPU_status.RBAR_A1;
                        MPU->RLAR_A1 = MPU_status.RLAR_A1;
                        MPU->RBAR_A2 = MPU_status.RBAR_A2;
                        MPU->RLAR_A2 = MPU_status.RLAR_A2;
                        MPU->RBAR_A3 = MPU_status.RBAR_A3;
                        MPU->RLAR_A3 = MPU_status.RLAR_A3;
                        MPU->MAIR0 = MPU_status.MAIR0;
                        MPU->MAIR1 = MPU_status.MAIR1;
                        MPU->CTRL = MPU_status.CTRL;
#endif
                }

#if dg_configENABLE_DEBUGGER
                /* Detect if woken-up from JTAG and delay sleep if needed. */
                jtag_wkup_check(system_entered_sleep);
#endif

                /*
                 * If sleep was cancelled and the sleep mode was pm_mode_hibernation or
                 * pm_mode_deep_sleep, the device must be rebooted.
                 */
                if (!system_entered_sleep &&
                    (current_sleep_mode == pm_mode_hibernation ||
                     current_sleep_mode == pm_mode_deep_sleep) ) {
                        if (dg_configENABLE_DEBUGGER) {
                                ENABLE_DEBUGGER;
                        }
#ifdef SEC_MODEN
                        sys_set_reset_reason(RESET_REASON_REBOOT_WIFI);
#endif
                        hw_cpm_reboot_system();
                }
        }
        else {
                __WFI();
                /* Make sure that the code will be executed at the fastest possible speed. */
                hw_clk_set_hclk_div(ahb_div1);
        }

        /* Make sure that Timer2 is not programmed to hit within the next few usec. */
        sys_timer_invalidate_trigger();

        if (!allow_entering_sleep) {
                /* Restore clocks that may have been lowered before entering IDLE. */
                cm_restore_all_clocks();
        }

        if (system_sleeping == sys_powered_down) {
                /* Enable PD PERIPH */
                hw_sys_pd_periph_enable();
#if (PM_ENABLE_PD_COM_WHILE_ACTIVE == 1)
                /* Enable PD COM */
                hw_sys_pd_com_enable();
#endif /* (PM_ENABLE_PD_COM_WHILE_ACTIVE == 1) */
        }

        DBG_SET_HIGH(PWR_MGR_USE_TIMING_DEBUG, PWRDBG_SLEEP_EXIT);

        /*
         * Initial wake-up handling of the system.
         */
        if (system_sleeping == sys_powered_down) {
                /*
                 * ad_wake_up_ind() and ad_xtalm_ready_ind() have not been called yet
                 */
                adapters_wake_up_ind_called = false;
                call_adapters_xtal16m_ready_ind = false;

                pm_resume_from_sleep();

                /* flash is now on,so you can call TCS */
                sys_tcs_apply_reg_pairs(SYS_TCS_GROUP_PD_PER);
                sys_tcs_apply_reg_pairs(SYS_TCS_GROUP_PD_COMM);
                sys_tcs_apply_reg_pairs(SYS_TCS_GROUP_PD_SYS);
                hw_sys_set_preferred_values(HW_PD_SYS);
        }
}

__RETAINED_CODE void pm_execute_wfi(void)
{
#if (dg_configDISABLE_BACKGROUND_FLASH_OPS == 0)
        /* If an interrupt is already pending, do not sleep. */
        if ((NVIC->ISER[0] & NVIC->ISPR[0]) || (NVIC->ISER[1] & NVIC->ISPR[1])) {
                DBG_SET_LOW(PWR_MGR_USE_TIMING_DEBUG, PWRDBG_SLEEP_ENTER);
                DBG_SET_HIGH(PWR_MGR_USE_TIMING_DEBUG, PWRDBG_SLEEP_EXIT);

                __enable_irq();
                return;
        }

        if (qspi_process_operations() == false) {
                /*
                 * Wait for an interrupt
                 *
                 * Any interrupt will cause an exit from WFI(). This is not a problem since even
                 * if an interrupt other that the tick interrupt occurs before the next tick comes,
                 * the only thing that should be done is to resume the scheduler. Since no tick has
                 * occurred, the OS time will be the same.
                 */
                DBG_SET_LOW(PWR_MGR_USE_TIMING_DEBUG, PWRDBG_SLEEP_ENTER);
                __WFI();
                DBG_SET_HIGH(PWR_MGR_USE_TIMING_DEBUG, PWRDBG_SLEEP_EXIT);
        }

        qspi_check_and_suspend_operations();

        __enable_irq();

        /* Notify blocked tasks, if appropriate. */
        qspi_process_completed_operations();
#else
        __WFI();
        __enable_irq();
#endif
}

void pm_sleep_enter(uint32_t low_power_periods)
{
        uint64_t rtc_time;
        uint32_t lp_current_time;
        uint32_t trigger;
        uint32_t sleep_period;
        bool allow_stopping_tick = false;
        bool allow_entering_sleep = false;
        bool abort_sleep = false;

 #if (PWR_MGR_DEBUG == 1)
        low_power_periods_ret = low_power_periods;
 #endif

#if (dg_configDISABLE_BACKGROUND_FLASH_OPS == 0)
        if (qspi_is_op_pending()) {
                abort_sleep = true;
                pm_update_blocker_cnt(pm_sleep_blocker_abort_sleep1);
        }
#endif

        /* Check that TRNG service is not in the process of generating random numbers */
        if (sys_trng_producing_numbers() != 0) {
                abort_sleep = true;
                pm_update_blocker_cnt(pm_sleep_blocker_abort_sleep2);

        }

#if dg_configENABLE_DEBUGGER
        /* If the debugger is attached then sleep is not allowed */
        if (hw_sys_is_debugger_attached() || !jtag_wkup_delay_has_expired()) {
                abort_sleep = true;
                pm_update_blocker_cnt(pm_sleep_blocker_abort_sleep3);
        }
#endif /* dg_configENABLE_DEBUGGER */

        /*
         * Update Real Time Clock value
         */
        rtc_time = sys_timer_get_timestamp_fromCPM(&lp_current_time);

		if (current_sleep_mode == pm_mode_active) {
			pm_update_blocker_cnt(pm_sleep_blocker_pm_mode_active);
		}

		if (current_sleep_mode == pm_mode_idle) {
			pm_update_blocker_cnt(pm_sleep_blocker_pm_mode_idle);
		}

        /*
         * Check if sleep is allowed.
         */
        if (!abort_sleep && (current_sleep_mode != pm_mode_active)) {
                uint32_t lp_tick_offset;
                uint32_t os_sleep_time = 0;
                uint32_t rtc_offset;
                uint32_t sleep_time_reduction = 0;
                bool is_infinite = true;
                uint32_t wakeup_time = pm_get_sys_wakeup_cycles();

                if (wakeup_mode_is_XTAL32) {
                        wakeup_time += cm_get_xtalm_settling_lpcycles();
                }

                /* We plan to stop Tick */
                allow_stopping_tick = true;

                if (current_sleep_mode != pm_mode_idle) {
                        /* We plan to enter sleep */
                        allow_entering_sleep = true;
                }

                /*
                 * Compute when the earliest wake-up time is.
                 *
                 * lp_prescaled_time - lp_last_trigger : offset in this tick period (in prescaled LP
                 *         cycles, for the calculation of the sleep_period of the OS tasks, this
                 *         offset must be subtracted).
                 * rtc_time : absolute time (in LP cycles)
                 *
                 * sleep_period holds the result (in prescaled LP cycles).
                 */

                // 1. Check OS first!
                if (low_power_periods) {
                        // 1a. Offset in this tick period.
                        lp_tick_offset = sys_timer_get_tick_offset();

                        // 1c. Calculate time of wake-up as an offset of the current RTC value.
                        if (lp_tick_offset > low_power_periods) {
                                // We are already late! The tick interrupt may already be pending...
                                allow_entering_sleep = false;
                                allow_stopping_tick = false;
                        }
                        else {
                                os_sleep_time = low_power_periods - lp_tick_offset;
                        }
                        // 1d. Subtract power-up time.
                        if (allow_entering_sleep) {
                                if (os_sleep_time > wakeup_time) {
                                        sleep_time_reduction = wakeup_time;
                                        os_sleep_time -= sleep_time_reduction;
                                }
                                else {
                                        allow_entering_sleep = false;
                                }
                        }

                        if (!allow_entering_sleep) {
                                pm_update_blocker_cnt(pm_sleep_blocker_wakeup_time);
                        }

                        // 1e. Initially, wake-up time is set for the OS case.
                        sleep_period = os_sleep_time;
                        is_infinite = false;
                }
                else {
                        // 1f. Sleep period is infinite for the OS.
                        sleep_period = -1;
                }

                if (allow_entering_sleep) {                     // Power-down is possible.
                        do {
                                int i;
                                adapter_call_backs_t *p_Ad;

                                // 2. Check if sleep is blocked for some short period
                                if (sleep_is_blocked) {
                                        rtc_offset = (uint32_t)(sleep_blocked_until - rtc_time);
                                        if (rtc_offset < dg_configPM_MAX_ADAPTER_DEFER_TIME) {
                                                // Still valid ==> Block is ON.
                                                allow_entering_sleep = false;
                                                break;
                                        }
                                        else {
                                                // Time has passed! Reset flag!
                                                sleep_is_blocked = false;
                                        }
                                }

                                // 3. Calculate the overhead added from the Adapters.
                                if (!is_infinite) {
                                        for (i = 0; i < dg_configPM_MAX_ADAPTERS_CNT; i++) {
                                                p_Ad = adapters_cb[i];
                                                if (p_Ad != NULL) {
                                                        if (sleep_period > p_Ad->ad_sleep_preparation_time) {
                                                                sleep_time_reduction += p_Ad->ad_sleep_preparation_time;
                                                                sleep_period -= p_Ad->ad_sleep_preparation_time;
                                                        }
                                                        else {
                                                                sleep_period = 0;
                                                                allow_entering_sleep = false;
                                                                pm_update_bad_adapter_cnt(i);
                                                                break;
                                                        }
                                                }
                                        }
                                }
                        } while (0);

                        if (!allow_entering_sleep) {
                                pm_update_blocker_cnt(pm_sleep_blocker_adapters1);
                        }
                }

                // 4. Calculate sleep_period and check if power-down is possible.
                if (is_infinite) {
                        sleep_period = 0;
                }
                else {
                         // 4b. Abort power down if sleep_period is too short!
                         if (sleep_period < dg_configMIN_SLEEP_TIME) {
                                 allow_entering_sleep = false;
                         }

                         // 4c. Restore sleep time if power-down is not possible.
                         if (!allow_entering_sleep) {
                                 /* Since the system will not be powered-down, restore the sleep time
                                  * by inverting any reductions that have been made to account for
                                  * delays relevant to power-down / wake-up.
                                  */
                                 sleep_period += sleep_time_reduction;

                                 /* If the CPU clock is too slow, wake-up earlier in order to be able
                                  * to resume the OS in time.
                                  */
                                 if (cm_cpu_clk_get_fromISR() < cpuclk_16M) {
                                         sleep_period -= dg_configWAKEUP_RC32_TIME_SLOW;
                                 }
                         }

                         // 4d. Check if sleep period is too small!
                         if (sleep_period <= (TICK_PERIOD)) {
                                 allow_stopping_tick = false;
                         }

                         if (!allow_entering_sleep) {
                                 pm_update_blocker_cnt(pm_sleep_blocker_sleep_period);
                         }
                 }

#if (PWR_MGR_DEBUG == 1)
                 sleep_period_ret = sleep_period;
#endif
         }

         if (allow_stopping_tick) {
                 /*
                  * Mark that a wake-up interrupt will be treated as such and not as a typical interrupt
                  */
                 system_sleeping = sys_idle;

                 if (((current_sleep_mode == pm_mode_hibernation) || current_sleep_mode == pm_mode_deep_sleep)
                                 && (sleep_period == 0)) {
                         /*
                          * Interrupt is already disabled!
                          * No LP clock will be available during sleep and no wake up has been
                          * requested! The system will wake up only from an external event!
                          * No trigger will be programmed.
                          */
                 } else {
                         uint32_t lp_latest_time;
                         uint32_t computational_delay = 0;
                         uint32_t max_sleep_period;

                         /*
                          * If no "deep sleep" mode and sleep_period is infinite then wake up
                          * periodically to implement RTC.
                          * If "deep sleep" and wake-up has been requested after a sleep_period then
                          * schedule the wake up.
                          * In both cases, the lp clock will stay alive to allow the timer to wake
                          * the system up in time. The lp_last_trigger already holds the last trigger
                          * that hit since the prvStopTickInterrupt() has been called!
                          */
                         if ((dg_configUSE_WDOG == 1 && sys_watchdog_monitor_mask_empty())) {
                                 /*
                                  * Reload the watchdog counter to prevent it from expiring during sleep
                                  * or while waiting for an interrupt.
                                  */
                                 sys_watchdog_set_pos_val(dg_configWDOG_IDLE_RESET_VALUE);
                         }

                         if (dg_configUSE_WDOG == 0 && !allow_entering_sleep) {
                                 /*
                                  * The watchdog is frozen. The system must wake-up just before the
                                  * OS timer wraps.
                                  */
                                 max_sleep_period = MAX_TIMER_IDLE_COUNT;
                         }
                         else {
                                 /*
                                  * The watchdog is active, even when System PD is off.
                                  * The value of MAX_WDOG_TIMER_IDLE_COUNT ensures that the system will
                                  * wake-up before the watchdog expires.
                                  */
                                 max_sleep_period = MAX_WDOG_TIMER_IDLE_COUNT;
                         }

                         if (sleep_period == 0 || sleep_period > max_sleep_period) {
                                 sleep_period = max_sleep_period;
                         }
                         else {
                                 /*
                                  * Configure OS timer to raise interrupt at the requested time.
                                  */
                                 rtc_time = sys_timer_get_timestamp_fromCPM(&lp_latest_time);

                                 computational_delay = (lp_latest_time - lp_current_time) & LP_CNT_NATIVE_MASK;

                                 /* Make sure computational_delay is less than 10 prescaled cycles (else is too big!). */
                                 ASSERT_WARNING(computational_delay < 10);

                                 sleep_period -= computational_delay;
                         }
                         // Set wake-up trigger (minus any computational delays).
                         trigger = (lp_current_time + sleep_period) & LP_CNT_NATIVE_MASK;

 #if (PWR_MGR_DEBUG == 1)
                         trigger_setting_ret = trigger;
 #endif
                         sys_timer_set_trigger(trigger);


                 }

                 /**********************************************************************************/

                 if (allow_entering_sleep) {
                         if (hw_dma_channel_active()) {
                                 allow_entering_sleep = false;
                                 pm_update_blocker_cnt(pm_sleep_blocker_hw_dma);
                         }
                 }

                 if (allow_entering_sleep) {
                         int i;
                         adapter_call_backs_t *p_Ad;

                         /*
                          * Inquiry Adapters about the forthcoming sleep entry
                          */

                         // 1. Inquiry Adapters
                         for (i = dg_configPM_MAX_ADAPTERS_CNT - 1; i >= 0; i--) {
                                 p_Ad = adapters_cb[i];
                                 if ((p_Ad != NULL) && (p_Ad->ad_prepare_for_sleep != NULL)) {
                                         if (!p_Ad->ad_prepare_for_sleep()) {
                                                 break;
                                         }
                                 }
                         }

                         // 2. If an Adapter rejected sleep, resume any Adapters that have already accepted it.
                         if (i >= 0) {
                                 allow_entering_sleep = false;   // Sleep has been canceled.

                                pm_update_bad_adapter_cnt(i);
                                pm_update_blocker_cnt(pm_sleep_blocker_adapters2);
                                 i++;
                                 while (i < dg_configPM_MAX_ADAPTERS_CNT) {
                                         p_Ad = adapters_cb[i];
                                         if ((p_Ad != NULL) && (p_Ad->ad_sleep_canceled != NULL)) {
                                                 p_Ad->ad_sleep_canceled();
                                         }
                                         i++;
                                 }
                         }
                 }

                 apply_wfi(allow_entering_sleep, sleep_period);

                 if (system_sleeping == sys_powered_down) {
                         if (dg_configIMAGE_SETUP == DEVELOPMENT_MODE) {
                                 rtc_time = sys_timer_get_timestamp_fromISR();
                         }
                 }

                 if (!allow_entering_sleep) {
                         /*
                          * Hook to add any application specific code after exiting Idle state.
                          * Note: the System and the Peripheral Power Domains are active.
                          */
                         configPOST_IDLE_ENTRY( sleep_period );
                 }
         } else {
                 /*
                  * Make sure that it's not vTaskStepTick() that advances time but the OS timer ISR, in
                  * this case!
                  */
                ulTimeSpentSleepingInTicks = 0;

 #if (dg_configDISABLE_BACKGROUND_FLASH_OPS == 1)
                /*
                 * Wait for an interrupt
                 *
                 * Any interrupt will cause an exit from WFI(). This is not a problem since even
                 * if an interrupt other that the tick interrupt occurs before the next tick comes,
                 * the only thing that should be done is to resume the scheduler. Since no tick has
                 * occurred, the OS time will be the same.
                 */
                __WFI();
 #else
                execute_active_wfi();
 #endif
         }

         ASSERT_WARNING(__get_PRIMASK() == 1);

         /* Wake-up! */
         system_wake_up();
}

system_state_t pm_get_system_sleep_state(void)
{
        return system_sleeping;
}

#if dg_configENABLE_DEBUGGER


__STATIC_INLINE bool jtag_wkup_is_combo_trigger(uint32_t idx)
{
        const uint32_t MASK = HW_PDC_LUT_ENTRY_FIELD_MASK(TRIG_SELECT)
                            | HW_PDC_LUT_ENTRY_FIELD_MASK(TRIG_ID)
                            | HW_PDC_LUT_ENTRY_FIELD_MASK(PDC_MASTER);

        const uint32_t VALUE = HW_PDC_LUT_ENTRY_VAL(HW_PDC_TRIG_SELECT_PERIPHERAL,
                                                    HW_PDC_PERIPH_TRIG_ID_COMBO,
                                                    HW_PDC_MASTER_CM33, 0);

        return (hw_pdc_read_entry(idx) & MASK) == (VALUE & MASK);
}

static void jtag_wkup_discover_combo_pdc_entry_idx(void)
{
        jtag_wkup_combo_pdc_entry_idx = HW_PDC_INVALID_LUT_INDEX;

        for (int i = 0; i < HW_PDC_LUT_SIZE; ++i) {
                if (jtag_wkup_is_combo_trigger(i)) {
                        jtag_wkup_combo_pdc_entry_idx = i;
                        break;
                }
        }
}

__STATIC_INLINE bool jtag_wkup_combo_source_irq_is_pending(void) __attribute__((always_inline));
__STATIC_INLINE bool jtag_wkup_combo_source_irq_is_pending(void)
{
        ASSERT_ERROR( (CMAC2SYS_IRQn < 32)
                        && (KEY_WKUP_GPIO_IRQn < 32)
                        && (VBUS_IRQn < 32) );

        uint32_t non_jtag_combo_irq_pending = NVIC->ISPR[0]
                                                 & ( (1 << CMAC2SYS_IRQn)
                                                   | (1 << KEY_WKUP_GPIO_IRQn)
                                                   | (1 << VBUS_IRQn)
                                                   );
        return non_jtag_combo_irq_pending != 0;
}

__STATIC_INLINE bool jtag_wkup_detect(void) __attribute__((always_inline));
__STATIC_INLINE bool jtag_wkup_detect(void)
{
        const bool combo_pdc_entry_exists = (jtag_wkup_combo_pdc_entry_idx
                                                != HW_PDC_INVALID_LUT_INDEX);

        const bool combo_is_pending = (combo_pdc_entry_exists
                                        && hw_pdc_is_pending(jtag_wkup_combo_pdc_entry_idx));

        return combo_is_pending && !jtag_wkup_combo_source_irq_is_pending();
}

__STATIC_INLINE void jtag_wkup_disable_sleep_delay(void) __attribute__((always_inline));
__STATIC_INLINE void jtag_wkup_disable_sleep_delay(void)
{
        jtag_wkup_sleep_blocked_until = 0;
}

__STATIC_INLINE void jtag_wkup_set_sleep_delay_ms(uint32_t millis) __attribute__((always_inline));
__STATIC_INLINE void jtag_wkup_set_sleep_delay_ms(uint32_t millis)
{
        const uint32_t delay_sleep_lp_cycles = (millis * configSYSTICK_CLOCK_HZ) / 1000;
        const uint64_t now = sys_timer_get_timestamp_fromISR();

        jtag_wkup_sleep_blocked_until = now + delay_sleep_lp_cycles;
}

__RETAINED_CODE static void jtag_wkup_check(bool sys_cpu_entered_sleep)
{
        jtag_wkup_disable_sleep_delay();

        /* If the system CPU (CM33) did not enter sleep then we are done. */
        if (!sys_cpu_entered_sleep) {
                return;
        }

        if (jtag_wkup_detect()) {
                jtag_wkup_set_sleep_delay_ms(50);
        }
}

__STATIC_INLINE bool jtag_wkup_delay_has_expired(void) __attribute__((always_inline));
__STATIC_INLINE bool jtag_wkup_delay_has_expired(void)
{
        uint32_t unused;
        const uint64_t now = sys_timer_get_timestamp_fromCPM(&unused);

        return (now > jtag_wkup_sleep_blocked_until);
}

#endif /* dg_configENABLE_DEBUGGER */

#endif /* OS_FREERTOS */

void pm_set_sys_wakeup_mode(sys_wakeup_mode_t mode)
{
        uint32_t val = CRG_TOP->PMU_SLEEP_REG;
        switch (mode) {
        case pm_sys_wakeup_mode_slow:
                REG_SET_FIELD(CRG_TOP, PMU_SLEEP_REG, ULTRA_FAST_WAKEUP, val, 0);
                REG_SET_FIELD(CRG_TOP, PMU_SLEEP_REG, FAST_WAKEUP, val, 0);
                break;
        case pm_sys_wakeup_mode_fast:
                REG_SET_FIELD(CRG_TOP, PMU_SLEEP_REG, ULTRA_FAST_WAKEUP, val, 0);
                REG_SET_FIELD(CRG_TOP, PMU_SLEEP_REG, FAST_WAKEUP, val, 1);
                break;
        case pm_sys_wakeup_mode_ultra_fast:
        {
#if dg_configUSE_HW_PMU && dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH
                ASSERT_WARNING(dg_configFLASH_POWER_OFF != 1);

                HW_PMU_1V8P_RAIL_CONFIG rail_config_1v8p;
                ASSERT_WARNING((dg_configFLASH_CONNECTED_TO != FLASH_CONNECTED_TO_1V8P  &&
                                dg_configFLASH_CONNECTED_TO != FLASH_CONNECTED_TO_1V8F) ||
                                hw_pmu_get_1v8p_onsleep_config(&rail_config_1v8p) == POWER_RAIL_ENABLED);

                HW_PMU_1V8_RAIL_CONFIG rail_config_1v8;
                ASSERT_WARNING(dg_configFLASH_CONNECTED_TO != FLASH_CONNECTED_TO_1V8 ||
                        hw_pmu_get_1v8_onsleep_config(&rail_config_1v8) == POWER_RAIL_ENABLED);

#endif
#if dg_configUSE_HW_PMU
                HW_PMU_1V2_SLEEP_VOLTAGE rail_sleep_voltage;
                ASSERT_ERROR(hw_pmu_get_1v2_onsleep_config(&rail_sleep_voltage) == POWER_RAIL_ENABLED);
                // VDD voltage during sleep must be 0.9V
                ASSERT_ERROR(rail_sleep_voltage == HW_PMU_1V2_SLEEP_VOLTAGE_0V9);

                HW_PMU_1V2_RAIL_CONFIG rail_config;
                ASSERT_ERROR(hw_pmu_get_1v2_onwakeup_config(&rail_config) == POWER_RAIL_ENABLED);
                // LDO_VDD_RET must be used for sleep and wakeup
                ASSERT_ERROR(rail_config.current == HW_PMU_1V2_MAX_LOAD_1);
                // VDD voltage on wakeup must be 0.9V
                ASSERT_ERROR(rail_config.voltage == HW_PMU_1V2_VOLTAGE_0V9);
#endif
                REG_SET_FIELD(CRG_TOP, PMU_SLEEP_REG, ULTRA_FAST_WAKEUP, val , 1);
                REG_SET_FIELD(CRG_TOP, PMU_SLEEP_REG, FAST_WAKEUP, val, 1);
                break;
        }
        default:
                ASSERT_WARNING(0);
        }

        CRG_TOP->PMU_SLEEP_REG = val;

#if (USE_BLE_SLEEP == 1)
        ad_ble_update_wakeup_time();
#endif
}

sys_wakeup_mode_t pm_get_sys_wakeup_mode(void)
{
        uint32_t val = CRG_TOP->PMU_SLEEP_REG;

        if (REG_GET_FIELD(CRG_TOP, PMU_SLEEP_REG, ULTRA_FAST_WAKEUP, val) == 1) {
                return pm_sys_wakeup_mode_ultra_fast;
        }
        else if (REG_GET_FIELD(CRG_TOP, PMU_SLEEP_REG, FAST_WAKEUP, val) == 1) {
                return pm_sys_wakeup_mode_fast;
        }

        return pm_sys_wakeup_mode_slow;
}

uint8_t pm_get_sys_wakeup_cycles(void)
{
        uint8_t wakeup_cycles = 0;

        switch (pm_get_sys_wakeup_mode()) {
        case pm_sys_wakeup_mode_slow:
                wakeup_cycles = dg_configWAKEUP_RC32_TIME_SLOW;

#if dg_configUSE_HW_PMU
                HW_PMU_1V8_RAIL_CONFIG rail_1v8_config;

                if (hw_pmu_get_1v8_onwakeup_config(&rail_1v8_config) == POWER_RAIL_DISABLED ||
                   hw_pmu_get_1v8_onsleep_config(&rail_1v8_config) == POWER_RAIL_ENABLED) {
                        // If 1V8 is off or is on during sleep, then there is no need to wait to settle
                        wakeup_cycles -= dg_configVOLTAGE_CHECK_LP_CYCLES;
                }

                HW_PMU_1V8P_RAIL_CONFIG rail_1v8p_config;

                if (hw_pmu_get_1v8p_onwakeup_config(&rail_1v8p_config) == POWER_RAIL_DISABLED ||
                   hw_pmu_get_1v8p_onsleep_config(&rail_1v8p_config) == POWER_RAIL_ENABLED) {
                        // If 1V8P is off or is on during sleep, then there is no need to wait to settle
                        wakeup_cycles -= dg_configVOLTAGE_CHECK_LP_CYCLES;
                }
#endif
                break;
        case pm_sys_wakeup_mode_fast:
                wakeup_cycles = dg_configWAKEUP_RC32_TIME_FAST;
                break;
        case pm_sys_wakeup_mode_ultra_fast:
                wakeup_cycles = dg_configWAKEUP_RC32_TIME_ULTRA_FAST;
                break;
        default:
                ASSERT_WARNING(0);
        }
        return wakeup_cycles;
}

__RETAINED_CODE void pm_prepare_sleep(sleep_mode_t sleep_mode)
{
        if (sleep_mode == pm_mode_hibernation || sleep_mode == pm_mode_deep_sleep) {
#ifdef OS_FREERTOS
                // Hibernation Sleep mode! Wake-up only from external button!
                // Deep sleep mode! Wake-up only from external button! Keep RTC.
                sys_timer_stop();            // Stop OS timer
#endif

                // Switch system clock to RC32M
                hw_clk_set_sysclk(SYS_CLK_IS_RC32);

                /*
                 * Clear all PDC entries and make sure SYS_SLEEP is 0
                 */
                REG_CLR_BIT(CRG_TOP, PMU_CTRL_REG, SYS_SLEEP);

                hw_pdc_lut_reset();

                // Add one PDC entry for M33 to wake-up from GPIO and set SYS_SLEEP to 1
                uint32_t pdc_entry_index = hw_pdc_add_entry( HW_PDC_LUT_ENTRY_VAL(HW_PDC_TRIG_SELECT_PERIPHERAL,
                                                                                  HW_PDC_PERIPH_TRIG_ID_COMBO,
                                                                                  HW_PDC_MASTER_CM33, 0));

                hw_pdc_set_pending(pdc_entry_index);
                hw_pdc_acknowledge(pdc_entry_index);
                REG_SET_BIT(CRG_TOP, PMU_CTRL_REG, SYS_SLEEP);

                if (sleep_mode == pm_mode_deep_sleep) {
                        // Add one PDC entry for M33 to wake-up from RTC timer
                        pdc_entry_index = hw_pdc_add_entry( HW_PDC_LUT_ENTRY_VAL(HW_PDC_TRIG_SELECT_PERIPHERAL,
                                                                                 HW_PDC_PERIPH_TRIG_ID_RTC_TIMER,
                                                                                 HW_PDC_MASTER_CM33, 0));

                }

                // Turn off LP clocks. Leave LP clock unchanged in pm_mode_deep_sleep for RTC

                if (sleep_mode == pm_mode_hibernation) {
#if dg_configUSE_HW_RTC
                        hw_rtc_stop();
#endif
                        // Switch LP clock to to RC32K to allow waking-up from hibernation
                        hw_clk_enable_lpclk(LP_CLK_IS_RC32K);
                        hw_clk_set_lpclk(LP_CLK_IS_RC32K);

                        if ((dg_configUSE_LP_CLK == LP_CLK_32000)
                                        || (dg_configUSE_LP_CLK == LP_CLK_32768)) {
                                hw_clk_disable_lpclk(LP_CLK_IS_XTAL32K);
                        }

                        if (dg_configUSE_LP_CLK == LP_CLK_RCX) {
                                hw_clk_disable_lpclk(LP_CLK_IS_RCX);
                        }

                        /* Prepare for clockless sleep */
                        hw_sys_enable_clockless();
                }
                else {
                        /* Only XTAL32M can be used as LP clock in deep sleep mode */
                        ASSERT_WARNING((dg_configUSE_LP_CLK == LP_CLK_32000)
                                                || (dg_configUSE_LP_CLK == LP_CLK_32768));

                        /* Clockless mode cannot be used. Disable RC32K to disable watchdog. */
                        hw_clk_disable_lpclk(LP_CLK_IS_RC32K);
                }

                hw_sys_no_retmem();
                hw_sys_enable_reset_on_wup();

                hw_pd_power_down_tim();

                REG_CLR_BIT(CRG_TOP, PMU_SLEEP_REG, ULTRA_FAST_WAKEUP);
                REG_CLR_BIT(CRG_TOP, PMU_SLEEP_REG, FAST_WAKEUP);

#if dg_configUSE_HW_PMU
                HW_PMU_ERROR_CODE error_code;

                hw_bod_deactivate();

                /*
                 * Turn off 1V8 and 1V8P since the system will
                 * resume either by recharging the battery (where a
                 * reset/reboot will be issued on the occurrence of
                 * VBUS_IRQn) or by replacing the battery.
                 */
                hw_pmu_1v8_onsleep_disable();

                error_code = hw_pmu_1v8p_onsleep_disable();
                ASSERT_WARNING(error_code == HW_PMU_ERROR_NOERROR);

                error_code = hw_pmu_1v2_onwakeup_enable(HW_PMU_1V2_MAX_LOAD_50);
                ASSERT_WARNING(error_code == HW_PMU_ERROR_NOERROR);

                if (sleep_mode == pm_mode_hibernation) {
                        // Use VDD clamp can be used for hibernation
                        error_code = hw_pmu_set_vdd_clamp(HW_PMU_VDD_VOLTAGE_828);
                        ASSERT_WARNING(error_code == HW_PMU_ERROR_NOERROR);

                        error_code = hw_pmu_1v2_onsleep_disable();
                        ASSERT_WARNING(error_code == HW_PMU_ERROR_NOERROR);

#if (dg_configENABLE_DA1469x_AA_SUPPORT)
                        /* Workaround for bug2522AA_074: LDO output drift upwards on hold mode
                         * Leave BG_REFRESH_INTERVAL to 0x80 in deep sleep mode
                         */
                        REG_SETF(CRG_TOP, PMU_SLEEP_REG , BG_REFRESH_INTERVAL, 0);
#endif
                } else {
                        // RET_LDO must be used for deep sleep
                        error_code = hw_pmu_1v2_onsleep_enable(HW_PMU_1V2_SLEEP_VOLTAGE_0V75);
                        ASSERT_WARNING(error_code == HW_PMU_ERROR_NOERROR);
                }

#if (dg_configENABLE_DA1469x_AA_SUPPORT == 0)
                REG_SETF(CRG_TOP, PMU_SLEEP_REG , BG_REFRESH_INTERVAL, 0);
#endif
                error_code = hw_pmu_3v0_onsleep_config(HW_PMU_3V0_MAX_LOAD_1);

                ASSERT_WARNING(error_code == HW_PMU_ERROR_NOERROR);     // Use VSYS clamp
#endif

                if (dg_configENABLE_DEBUGGER) {
                        DISABLE_DEBUGGER;
                }
                REG_CLR_BIT(CRG_TOP, SYS_CTRL_REG, CMAC_DEBUGGER_ENABLE);
        } else {
                /*
                 * Put the Flash in Power-Down.
                 */
                if (dg_configFLASH_POWER_DOWN == 1) {
                        qspi_automode_flash_power_down();
                }

#if dg_configUSE_HW_PMU
                if (dg_configFLASH_CONNECTED_TO == FLASH_CONNECTED_TO_1V8F && dg_configFLASH_POWER_OFF == 1) {
#if dg_configUSE_BOD
                        hw_bod_deactivate_channel(BOD_CHANNEL_1V8F);
#endif
                        HW_PMU_ERROR_CODE error_code;
                        error_code = hw_pmu_1v8f_disable();
                        ASSERT_WARNING(error_code == HW_PMU_ERROR_NOERROR);
                }
#endif /* dg_configUSE_HW_PMU */
        }

        /*
         * Put the Flash in Power-Down.
         */
        qspi_automode_flash_power_down();

#if dg_configUSE_HW_PMU
        if (dg_configFLASH_CONNECTED_TO == FLASH_CONNECTED_TO_1V8F && dg_configFLASH_POWER_OFF == 1) {
#if dg_configUSE_BOD
                hw_bod_deactivate_channel(BOD_CHANNEL_1V8F);
#endif
                HW_PMU_ERROR_CODE error_code;

                error_code = hw_pmu_1v8f_disable();
                ASSERT_WARNING(error_code == HW_PMU_ERROR_NOERROR);
        }
#endif /* dg_configUSE_HW_PMU */
}

__RETAINED_CODE void pm_resume_from_sleep(void)
{
#if dg_configUSE_HW_PMU
        if (dg_configFLASH_CONNECTED_TO == FLASH_CONNECTED_TO_1V8F && dg_configFLASH_POWER_OFF == 1) {
                hw_pmu_1v8f_enable();
#if dg_configUSE_BOD
                hw_bod_activate_channel(BOD_CHANNEL_1V8F);
#endif
        }
#endif /* dg_configUSE_HW_PMU */

        /*
         * Power-up Flash if dg_configFLASH_POWER_DOWN is 1
         * Re-initialize Flash if dg_configFLASH_POWER_OFF is 1
         */
        qspi_automode_flash_power_up();

#if dg_configUSE_HW_OTPC
        // Put the OTP in DSTBY mode to minimize current consumption
        if (dg_configCODE_LOCATION != NON_VOLATILE_IS_OTP) {
                hw_otpc_disable();
        }
#endif

#if (dg_configUSE_WDOG == 0)
        hw_watchdog_freeze();                   // Stop watchdog
        /*
         * Reload the watchdog counter to prevent it from expiring during sleep.
         */
        sys_watchdog_set_pos_val(dg_configWDOG_IDLE_RESET_VALUE);
#else
        // Restore watchdog reset value
        sys_watchdog_set_pos_val(dg_configWDOG_RESET_VALUE);
#endif
}

#if defined(CONFIG_MODEN_SEC_RESET)
extern __RETAINED_CODE void hw_cpm_reset_system(void);

#define REG_GET(addr)				(*(volatile unsigned int *)(addr))
#define REG_SET(addr, value)		((*(volatile unsigned int *)(addr)) = (value))

#define PMU_CTRL_REG					0x50000020UL
#define ENABLE_CLKLESS				(1 << 8)
#define RETAIN_CACHE					(1 << 7)
#define SYS_SLEEP						(1 << 6)
#define RESET_ON_WAKEUP				(1 << 5)
#define MAP_BANDGAP_EN				(1 << 4)
#define COM_SLEEP						(1 << 3)
#define TIM_SLEEP						(1 << 2)
#define RADIO_SLEEP					(1 << 1)
#define PERIPH_SLEEP					(1 << 0)

#define SYS_CTRL_REG					0x50000024UL
#define SW_RESET						(1 << 15)
#define CACHRAM_MUX					(1 << 10)
#define TIMEOUT_DISABLE				(1 << 9)
#define DEBUGGER_ENABLE				(1 << 7)
#define QSPI_INT						(1 << 4)
#define REMAP_INTVECT				(1 << 3)
#define REMAP_ADR0_QSPI_FLASH		(1 << 2)
#define REMAP_ADR0_OTP				(1 << 1)
#define REMAP_ADR0_ROM				(1 << 0)

#define POR_PIN_REG					0x50000098UL
#define POR_PIN_POLARITY				(1 << 7)
#define POR_GENERATION_DISABLED		0x3F
#define POR_GPIO_P0_07				0x07

/*
 * Time = POR_TIME * 4096 * RC32 clock period
 */
#define POR_TIMER_REG			0x5000009CUL

/*
 * Reset State
 */
#define RESET_STAT_REG			0x500000BCUL
#define CMAC_WDOGRESET_STAT		(1 << 5)
#define SWD_HWRESET_STAT			(1 << 4)
#define WDOGRESET_STAT			(1 << 3)
#define SWRESET_STAT				(1 << 2)
#define HWRESET_STAT				(1 << 1)
#define PORESET_STAT				(1 << 0)

/* to be revised */
void display_reset_state(void) {
	uint8_t reg_value = 0;

	reg_value = REG_GET(RESET_STAT_REG);
	printf("\r\n\r\nReset Mask = 0x%x\r\n", reg_value);

	// POR_RESET
	if ( !!(reg_value & PORESET_STAT) ) {
		printf("	POR_RESET\r\n");
	}
	// HW_RESET
	if ( !!(reg_value & HWRESET_STAT) ) {
			printf("	HWRESET_STAT\r\n");
	}
	// SW_RESET
	if ( !!(reg_value & SWRESET_STAT) ) {
			printf("	SWRESET_STAT\r\n");
	}
	// WDOGRESET_STAT
	if ( !!(reg_value & WDOGRESET_STAT) ) {
			printf("	WDOGRESET_STAT\r\n");
	}
	// SWD_HWRESET_STAT
	if ( !!(reg_value & SWD_HWRESET_STAT) ) {
			printf("	SWD_HWRESET_STAT\r\n");
	}
	// CMAC_WDOGRESET_STAT
	if ( !!(reg_value & CMAC_WDOGRESET_STAT) ) {
			printf("	CMAC_WDOGRESET_STAT\r\n");
	}
}


/**
 * \brief Perform soft reset
 *
 * This function is called in order to soft reset.
 * All of timer and task should be deleted
 * All of data that need to be saved should be written to flash memory.
 * At the end of process above, system will be rebooted by watch dog.
 */
void pm_system_sw_reset(void) {
	printf("--> %s <---\r\n",__func__);

#if 1
	// Method 1:
	//    Dialog?�ì„�? recommendation?? ë°©ë²??. reset clock & power module
	//
	sys_set_reset_reason(RESET_REASON_REBOOT_SOFT);
	hw_cpm_reset_system();
#endif

#if 0
	// Method 2:
	//     System Control Registerë¥? ?œì?��???ëŠ?? ë°©ë²??
	//     SW_RESET bitë¥? setting??ë©´ ì¦?�ì‹�? reset ??
	uint16_t reg = 0;
	reg = REG_GET(SYS_CTRL_REG);
	printf("SYS_CTRL_REG=0x%04x\r\n", reg);
	reg |= SW_RESET;
	REG_SET(SYS_CTRL_REG, reg);
#endif

	while(1);
}

/**
 * \brief Perform POR(Power on reset : Hard) reset
 *
 * This function is called in order to soft reset.
 * All of timer and task should be deleted
 * All of data that need to be saved should be written to flash memory.
 * At the end of process above, system will be rebooted.
 */
void pm_system_hw_reset(void) {
	uint16_t reg = 0;

#if 0 // PMU control. ??��?��? ??��?�� ??��?��???ì? ?Šì��?
	reg = REG_GET(PMU_CTRL_REG);
	printf("PMU_REG=0x%04x\r\n",reg);

	// ??��?��? bitë§? ?¤ì ????ë©´ reset ??ë��?
	//reg |=  RESET_ON_WAKEUP;
	//REG_SET(PMU_CTRL_REG, reg);

	reg = RESET_ON_WAKEUP;
	REG_SET(PMU_CTRL_REG, reg);

	reg = REG_GET(PMU_CTRL_REG);
	printf("PMU_REG=0x%04x\r\n",reg);
#endif


#if 1 // POR control
	/* read current register */
	reg = REG_GET(POR_PIN_REG);
	printf("POR_PIN_REG=0x%04x\n", reg);

	reg = 0;
	/* set polarity */
	reg |= POR_PIN_POLARITY;

	/* set gpio */
	reg |= POR_GPIO_P0_07;

	REG_SET(POR_PIN_REG, reg);

	/* set timer */
	reg = 1;
	reg = REG_SET(POR_TIMER_REG, reg);
#endif

	while(1);
}

#if 0
/**
 * \brief Perform POR(Power on reset : Hard) reset
 *
 * This function set POR registers to set POR timer
 * when side-key have been pressed for 7 seconds, POR will occur
 */

void pm_system_7sec_hw_reset(void) {
	uint16_t reg = 0;

#if 1 // POR control
	/* read current register */
	reg = REG_GET(POR_PIN_REG);
	printf("POR_PIN_REG=0x%04x\n", reg);

	reg = 0;
	/* set polarity */
	//reg |= POR_PIN_POLARITY; //active low, pressed side-key

	/* set gpio */
	reg |= POR_GPIO_PO_07;

	REG_SET(POR_PIN_REG, reg);

	/* set timer */
	reg = 56; //about 7 secconds
	reg = REG_SET(POR_TIMER_REG, reg);
#endif

}
#endif

#endif

/**
 \}
 \}
 \}
 */
