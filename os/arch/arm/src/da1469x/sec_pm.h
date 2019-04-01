/**
 ****************************************************************************************
 *
 * @file sec_pm.h
 *
 * @brief Samsung Power Manager header file.
 *
 * Copyright (C) 2018-2019 Samsung Electronics Co. Ltd.
 * This computer program includes Confidential, Proprietary Information
 * of Samsung Electronics Co. Ltd.. All Rights Reserved.
 *
 ****************************************************************************************
 */


#ifndef SEC_PM_H_
#define SEC_PM_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	pm_wc_crypto = 0,
	pm_wc_nvms,
	pm_wc_usb,
	pm_wc_socf,
	pm_wc_clk,
	pm_wc_gdi,
	pm_wc_factory,
	pm_wc_adc,
	pm_wc_uart,
	pm_wc_lcdc,
	pm_wc_i2c_1,
	pm_wc_i2c_2,
	pm_wc_i2c_3,
	pm_wc_i2c_4,
	pm_wc_i2c_5,
	pm_wc_i2c_6,
	pm_wc_max
} wakeup_client_t;

typedef enum {
	pm_fail_i2c_max = 10
} sleep_fail_i2c_t;

typedef enum {
	pm_sleep_blocker_try_sleep = 0,
	pm_sleep_blocker_os_abort_sleep,
	pm_sleep_blocker_abort_sleep1,
	pm_sleep_blocker_abort_sleep2,
	pm_sleep_blocker_abort_sleep3,
	pm_sleep_blocker_pm_mode_active,
	pm_sleep_blocker_pm_mode_idle,
	pm_sleep_blocker_wakeup_time,
	pm_sleep_blocker_adapters1,
	pm_sleep_blocker_sleep_period,
	pm_sleep_blocker_hw_dma,
	pm_sleep_blocker_adapters2,
	pm_sleep_blocker_pending_int,
	pm_sleep_blocker_max
} sleep_blocker_t;

typedef enum {
	clk_sysclk_default,
	clk_sysclk_max,
	clk_max
} clk_t;

#ifdef CONFIG_USE_SEC_PM

/* Replaces same function with no argument in sys_power_mgr.h */
void pm_stay_active(wakeup_client_t client);
void pm_stay_idle(wakeup_client_t client);
void pm_resume_sleep(wakeup_client_t client);
/* End of replacements */

void pm_update_motor(bool onoff);

__RETAINED_CODE void pm_update_sensor(int sensor_id, uint32_t type, uint32_t rate, uint8_t mode);
void pm_update_sensor_algo(int algo, int enable);
void pm_update_sensor_algo_dm(int algo, int enable);
__RETAINED_CODE void pm_update_sensor_algo_evt(int algo);
__RETAINED_CODE void pm_profile_sensor(int start, int normal);
__RETAINED_CODE void pm_profile_algo(int algo, int start, int normal);

void pm_update_clk_time(clk_t clk);

__RETAINED_CODE void pm_save_pinmux_setting(int port, int pin, int mode, int function);
__RETAINED_CODE void pm_save_pin_state(int port, int pin, int state);
void pm_show_pins_state(void);

void pm_sap_snd(const uint8_t reserved_service_id);
void pm_sap_rcv(const uint8_t reserved_service_id);

void pm_hook_ui_event(int ui_event);
void pm_hook_after_ui_event(int ui_event);
void pm_increase_client_cnt(wakeup_client_t client);
void pm_decrease_client_cnt(wakeup_client_t client);
__RETAINED_CODE void pm_increase_i2c_cnt(unsigned char addr);
__RETAINED_CODE void pm_increase_dma_cnt(int channel_flag);

__RETAINED_CODE void pm_update_blocker_cnt(sleep_blocker_t blocker);
__RETAINED_CODE void pm_update_bad_adapter_cnt(int adapter);
__RETAINED_CODE void pm_update_active_time(int update);
__RETAINED_CODE void pm_update_sleep_time(uint32_t sleep_ticks);

__RETAINED_CODE void prvConfigureTimerForRunTimeStats(void);
__RETAINED_CODE uint32_t prvGetRunTimeCounterValue(void);

void pm_show_run_time_stat(void);

#else /* CONFIG_USE_SEC_PM */

static inline void pm_update_motor(bool onoff) {}

static inline void pm_update_sensor(int sensor_id, uint32_t type, uint32_t rate, uint8_t mode) {}
static inline void pm_update_sensor_algo(int algo, int enable) {}
static inline void pm_update_sensor_algo_dm(int algo, int enable) {}
static inline void pm_update_sensor_algo_evt(int algo) {}
static inline void pm_profile_sensor(int start, int normal) {}
static inline void pm_profile_algo(int algo, int start, int normal) {}

static inline void pm_update_clk_time(clk_t clk) {}

static inline void pm_save_pinmux_setting(int port, int pin, int mode, int function) {}
static inline void pm_save_pin_state(int port, int pin, int state) {}
static inline void pm_show_pins_state(void) {}

static inline void pm_sap_snd(const uint8_t reserved_service_id) {}
static inline void pm_sap_rcv(const uint8_t reserved_service_id) {}

static inline void pm_hook_ui_event(int ui_event) {}
static inline void pm_hook_after_ui_event(int ui_event) {}
static inline void pm_increase_client_cnt(wakeup_client_t client) {}
static inline void pm_decrease_client_cnt(wakeup_client_t client) {}
static inline void pm_increase_i2c_cnt(unsigned char addr) {}
static inline void pm_increase_dma_cnt(int channel_flag) {}

static inline void pm_update_blocker_cnt(sleep_blocker_t blocker) {}
static inline void pm_update_bad_adapter_cnt(int adapter) {}
static inline void pm_update_active_time(int update) {}
static inline void pm_update_sleep_time(uint32_t sleep_ticks) {}

static inline void prvConfigureTimerForRunTimeStats(void) {}
static inline uint32_t prvGetRunTimeCounterValue(void) { return 0; }

static inline void pm_show_run_time_stat(void) {}

#endif /* CONFIG_USE_SEC_PM */

#endif /* SEC_PM_H_ */
