/**
 ****************************************************************************************
 *
 * @file sec_pm.c
 *
 * @brief Samsung Power Manager
 *
 * Copyright (C) 2018-2019 Samsung Electronics Co. Ltd.
 * This computer program includes Confidential, Proprietary Information
 * of Samsung Electronics Co. Ltd.. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include <inttypes.h>
#include <stdarg.h>
#include "sdk_defs.h"
#include "sys_clock_mgr.h"
#include "sys_clock_mgr_internal.h"
#include "sys_power_mgr.h"
#include "sys_power_mgr_internal.h"
#include "sys_timer.h"
#include "sys_timer_internal.h"
#include "sys_watchdog.h"
#include "sec_pm.h"
#include "sec_pm_data.h"

//#define SEC_PM_SENSORTASK_PROFILE

#define PM_SENSOR_MAX	(SENSOR_MAX)

#define PM_BLE_CORE_ACTIVE      0 /* BLE_ACTIVE */ 
#define PM_BLE_CORE_SLEEPING    1 /* BLE_SLEEPING */
#define PM_BLE_CORE_WAKING_UP   2 /* BLE_WAKING_UP */

PRIVILEGED_DATA t_pm_data pm_data = {0,};

PRIVILEGED_DATA uint32_t uLLSleepBlocker[pm_sleep_blocker_max];
PRIVILEGED_DATA uint32_t uLLBadAdapters[dg_configPM_MAX_ADAPTERS_CNT];
PRIVILEGED_DATA uint32_t ulFailI2c[pm_fail_i2c_max][2];
PRIVILEGED_DATA uint32_t ulFailDma[HW_DMA_CHANNEL_INVALID];

PRIVILEGED_DATA uint32_t ulPinStateChgReqCnt[HW_GPIO_NUM_PORTS][32];
PRIVILEGED_DATA uint32_t ulPinFunc[HW_GPIO_NUM_PORTS]; // 1 : gpio output, 0 : other functions 
PRIVILEGED_DATA uint32_t ulPinState[HW_GPIO_NUM_PORTS];

PRIVILEGED_DATA uint32_t ulSensorRefCnt[PM_SENSOR_MAX][2];
PRIVILEGED_DATA uint32_t ulSensorRate[PM_SENSOR_MAX][2];
PRIVILEGED_DATA uint32_t ulSensorUpdateRate;
PRIVILEGED_DATA uint32_t ulRegSensorAlgo;
PRIVILEGED_DATA uint32_t ulRegSensorAlgoDM;

PRIVILEGED_DATA char sec_pm_buf[512];


/*
 * Function for ble core statistics
 */
void pm_update_ble_core_sleep_time(int state)
{
    uint64_t rtc_now; 

    //MSG_WARN(DBG_PM|DBG_SLEEP, "pm_update_ble_core_sleep_time: %d\r\n", state);

    rtc_now = sys_timer_get_timestamp();

    if (state == PM_BLE_CORE_ACTIVE) {        
        if (pm_data.ulBLECoreCurrentState == PM_BLE_CORE_SLEEPING)
            pm_data.uLLTotalBLECoreSleepTicks += (rtc_now - pm_data.uLLBLECoreSleepTicksNow);
        pm_data.ulBLECoreCurrentState = (uint32_t)state;
    } else if (state == PM_BLE_CORE_SLEEPING) {
        if (pm_data.ulBLECoreCurrentState != PM_BLE_CORE_SLEEPING) {
            pm_data.uLLBLECoreSleepTicksNow = rtc_now;
            pm_data.uLLBLECoreSleepCnt++;
            pm_data.ulBLECoreCurrentState = (uint32_t)state;
        }
    } else if (state == PM_BLE_CORE_WAKING_UP) {
        if (pm_data.ulBLECoreCurrentState == PM_BLE_CORE_SLEEPING)
            pm_data.uLLTotalBLECoreSleepTicks += (rtc_now - pm_data.uLLBLECoreSleepTicksNow);
        pm_data.ulBLECoreCurrentState = (uint32_t)state;
    }   
}

/*
 * Function for motor_onoff
 */
void pm_update_motor(bool onoff)
{
	uint64_t uLLRTCTickNow;

	if (onoff) {
		pm_data.uLLMotorTicksNow = sys_timer_get_timestamp();
		pm_data.ulMotorCnt++;
	} else {
		uLLRTCTickNow = sys_timer_get_timestamp();
		pm_data.uLLTotalMotorTicks += (uLLRTCTickNow - pm_data.uLLMotorTicksNow);
	}
}

/*
 * Function for monitoring sensor algorithms
 */ 
__RETAINED_CODE void pm_update_sensor(int sensor_id, uint32_t type, uint32_t rate, uint8_t mode)
{
	configASSERT(sensor_id < PM_SENSOR_MAX);
	configASSERT(mode < 2);

	if (type == 1)
		ulSensorRefCnt[sensor_id][mode]++;
	else if (type == 2 && ulSensorRefCnt[sensor_id][mode] > 0)
		ulSensorRefCnt[sensor_id][mode]--;
	else if (type == 4)
		ulSensorRate[sensor_id][mode] = rate;
	else if (type == 8)
		ulSensorUpdateRate = rate;
}

void pm_update_sensor_algo(int algo, int enable)
{
	if (enable)
		ulRegSensorAlgo |= (uint32_t) (1 << algo);
	else
		ulRegSensorAlgo &= (uint32_t) ~(1 << algo);

	//MSG_WARN(DBG_PM|DBG_SLEEP, "pm_u_s_a: %d %d\r\n",
		//	algo, enable);
}

void pm_update_sensor_algo_dm(int algo, int enable)
{
	if (enable)
		ulRegSensorAlgoDM |= (uint32_t) (1 << algo);
	else
		ulRegSensorAlgoDM &= (uint32_t) ~(1 << algo);

	//MSG_WARN(DBG_PM|DBG_SLEEP, "pm_u_s_a_m: %d %d\r\n",
		//		algo, enable);
}

__RETAINED_CODE void pm_update_sensor_algo_evt(int algo)
{
	configASSERT(algo <= PM_SENSOR_ALG_MAX);

	pm_data.ulSensorAlgoEvt[algo]++;
}

#ifdef SEC_PM_SENSORTASK_PROFILE
__RETAINED_CODE void pm_profile_sensor(int start, int normal)
{
	uint64_t uLLRTCTickNow, uLLDelta;

	uLLRTCTickNow = sys_timer_get_timestamp();

	if (normal) {
	} else {
		if (start) {
			pm_data.uLLSensorStartTick = uLLRTCTickNow;
		} else {
			uLLDelta = uLLRTCTickNow - pm_data.uLLSensorStartTick;
			pm_data.uLLTotalSensorTick += uLLDelta;
			pm_data.uLLLastSensorTick = uLLDelta;
			pm_data.ulSensorUpdateCnt++;
		}
	}
}

__RETAINED_CODE void pm_profile_algo(int algo, int start, int normal)
{
	uint64_t uLLRTCTickNow, uLLDelta;

	uLLRTCTickNow = sys_timer_get_timestamp();

	if (normal) {
	} else {
		if (start) {
			pm_data.uLLAlgoStartTick = uLLRTCTickNow;
		} else {
			uLLDelta = uLLRTCTickNow - pm_data.uLLAlgoStartTick;
			pm_data.uLLTotalAlgoTick[algo] += uLLDelta;
			pm_data.uLLLastAlgoTick[algo] = uLLDelta;
		}
	}
}
#else
static inline void pm_profile_sensor(int start, int normal) {}
static inline void pm_profile_algo(int algo, int start, int normal) {}
#endif


/*
 * Function for clk 
 */
void pm_update_clk_time(clk_t clk)
{
	uint64_t uLLRTCTickNow, uLLDelta;

	uLLRTCTickNow = sys_timer_get_timestamp();
	uLLDelta = uLLRTCTickNow - pm_data.uLLClkChgTicksNow;
	pm_data.uLLTotalClkTicks[pm_data.eCurrentClk] += uLLDelta;

	//MSG_WARN(DBG_PM|DBG_SLEEP, "pm_c: %d %d %d %d\r\n",
		//		pm_data.eCurrentClk, clk, (uint32_t)uLLDelta, pm_data.ulClkChgReqCnt);

	pm_data.eCurrentClk = clk;
	pm_data.uLLClkChgTicksNow = uLLRTCTickNow;
	pm_data.ulClkChgReqCnt[clk]++;
}


/*
 * Function for pin control
 */
__RETAINED_CODE void pm_save_pinmux_setting(int port, int pin, int mode, int function)
{
	int output_mode = HW_GPIO_MODE_OUTPUT|HW_GPIO_MODE_OUTPUT_PUSH_PULL|HW_GPIO_MODE_OUTPUT_OPEN_DRAIN;
	//uint32_t ulPrevPinFunc = ulPinFunc[port];

 	if ((function == HW_GPIO_FUNC_GPIO) && (mode & output_mode))
		ulPinFunc[port] |= (uint32_t) (1 << pin);
	else
		ulPinFunc[port] &= (uint32_t) ~(1 << pin);

	//if (ulPrevPinFunc != ulPinFunc[port])
		//MSG_WARN(DBG_PM|DBG_SLEEP, "pm: %d %d %d 0x%x 0x%x 0x%x\r\n",
			//		port, pin, function, mode, ulPrevPinFunc, ulPinFunc[port]);
}

__RETAINED_CODE void pm_save_pin_state(int port, int pin, int state)
{
	//uint32_t ulPrevPinState;

	//ulPrevPinState = ulPinState[port];

	if (state)
		ulPinState[port] |= (uint32_t) (1 << pin);
	else
		ulPinState[port] &= (uint32_t) ~(1 << pin);

	ulPinStateChgReqCnt[port][pin]++;

	//if (pin == 19 && (ulPrevPinState != ulPinFunc[port]))
		//MSG_WARN(DBG_PM|DBG_SLEEP, "pm_p: %d %d %d 0x%x 0x%x 0x%x\r\n",
			//		port, pin, state, ulPrevPinState, ulPinState[port]);
}

void pm_show_pins_state(void)
{
	//int i;

	MSG_WARN(DBG_PM|DBG_SLEEP, "0x%08x 0x%08x\r\n", ulPinFunc[0], ulPinFunc[1]);
	MSG_WARN(DBG_PM|DBG_SLEEP, "0x%08x 0x%08x\r\n", ulPinState[0], ulPinState[1]);

	//for (i = 0; i < 32; i++)
		//MSG_WARN(DBG_PM|DBG_SLEEP, "%d\r\n", ulPinStateChgReqCnt[0][i]);

	//for (i = 0; i < 24; i++)
		//MSG_WARN(DBG_PM|DBG_SLEEP, "%d\r\n", ulPinStateChgReqCnt[1][i]);
}


/*
 * Function for sap framework
 */
void pm_sap_snd(const uint8_t reserved_service_id)
{
	if (reserved_service_id >= MAX_NONITOR_SAP_ID)
		return;
	pm_data.ulSapSndCnt[reserved_service_id]++;
}

void pm_sap_rcv(const uint8_t reserved_service_id)
{
	if (reserved_service_id >= MAX_NONITOR_SAP_ID)
		return;
	pm_data.ulSapRcvCnt[reserved_service_id]++;
}

/*
 * Hook Function for UI Event
 */
void pm_hook_ui_event(int ui_event)
{
	int e = ui_event;
	uint64_t uLLRTCTickNow;
	uint32_t ulChargeVoltNow, ulChargeSocNow;
	
	switch (e) {	
	case UI_EVENT_SCREEN_TURN_ON:
		pm_data.uLLScreenOnTicksNow = sys_timer_get_timestamp();
		pm_data.ulScreenOnCnt++;
		//cm_sys_clk_set(sysclk_PLL96);
		break;
	/*case UI_EVENT_SCREEN_TURN_OFF:
		uLLRTCTickNow = sys_timer_get_timestamp();
		pm_data.uLLTotalScreenOnTicks += (uLLRTCTickNow - pm_data.uLLScreenOnTicksNow);
		pm_show_run_time_stat();
		break;*/
	case UI_EVENT_INPUT_WRIST_UP:
		pm_data.ulWristUpCnt++;
		break;
#ifdef UI_EVENT_SHOW_FMB
	case UI_EVENT_SHOW_FMB:
		pm_data.ulShowFindMyBandCnt++;
		break;
#endif
#ifdef UI_EVENT_SHOW_HEALTH_ALERT
	case UI_EVENT_SHOW_HEALTH_ALERT:
		pm_data.ulShowHealthAlertCnt++;
		break;
#endif
	case UI_EVENT_INPUT_TOUCH_PRESSED:
		pm_data.ulTouchCnt++;
		break;
	case UI_EVENT_SINGLE_KEY_PRESS:
		pm_data.ulKeyCnt++;
		if (!gdi_display_is_enabled()) {
			pm_show_run_time_stat();
		}
		break;
	case UI_EVENT_CHG_ATTACH:
		pm_data.uLLChargeTicksNow = sys_timer_get_timestamp();
		pm_data.lLastChargeVolt = ad_battery_get_voltage();
		pm_data.lLastChargeSoc = sec_batt_get_soc();
		break;
	case UI_EVENT_CHG_DETACH:
		uLLRTCTickNow = sys_timer_get_timestamp();
		pm_data.uLLTotalChargeTicks += uLLRTCTickNow - pm_data.uLLChargeTicksNow;
		ulChargeVoltNow = ad_battery_get_voltage();
		pm_data.lTotalChargeVolt += ulChargeVoltNow - pm_data.lLastChargeVolt;
		ulChargeSocNow = sec_batt_get_soc();
		pm_data.lTotalChargeSoc += ulChargeSocNow - pm_data.lLastChargeSoc;
		break;
	default:
		break;
	}
}

void pm_hook_after_ui_event(int ui_event)
{
	int e = ui_event;
	uint64_t uLLRTCTickNow;
	
	switch (e) {		
	case UI_EVENT_SCREEN_TURN_ON:
		pm_show_pins_state();
		break;
	case UI_EVENT_SCREEN_TURN_OFF:
	{
		//cm_sys_clk_set_status_t err;

		//err = cm_sys_clk_set(sysclk_XTAL32M);
		//if (err < cm_sysclk_success)
			//MSG_WARN(DBG_PM|DBG_SLEEP, "pm_c: %d\r\n", err);
		pm_show_pins_state();
		uLLRTCTickNow = sys_timer_get_timestamp();
		pm_data.uLLTotalScreenOnTicks += (uLLRTCTickNow - pm_data.uLLScreenOnTicksNow);
		pm_show_run_time_stat();
		break;
	}
	default:
		break;
	}
}

/*
 * Function for pm_stay_xxx
 */
void pm_increase_client_cnt(int mode, wakeup_client_t client, int diff)
{
	uint32_t unused;

	OS_ASSERT(mode < SLEEP_MODE_MAX);
	OS_ASSERT(client < pm_wc_max);

	pm_data.ulWakeupRequestClient[mode][client]++;
	pm_data.uLLLastChangeTick[mode][client] = sys_timer_get_timestamp_fromCPM(&unused); 
	pm_data.ulWakeupDiffSum += diff;
}

void pm_decrease_client_cnt(int mode, wakeup_client_t client, int diff)
{
	uint32_t unused;
	uint64_t uLLTickNow, uLLDelta;

	OS_ASSERT(mode < SLEEP_MODE_MAX);
	OS_ASSERT(client < pm_wc_max);

	pm_data.ulWakeupReleaseClient[mode][client]++;
	pm_data.ulWakeupDiffSum += diff;

	if (pm_data.ulWakeupRequestClient[mode][client] - pm_data.ulWakeupReleaseClient[mode][client] > 0)
		return;

	uLLTickNow = sys_timer_get_timestamp_fromCPM(&unused);
	uLLDelta = uLLTickNow - pm_data.uLLLastChangeTick[mode][client];
	pm_data.uLLTotalPreventTick[mode][client] += uLLDelta;
}

/*
 * Function for ad_i2c_prepare_for_sleep
 */
__RETAINED_CODE void pm_increase_i2c_cnt(unsigned char addr)
{
	int i;

	for (i = 0; i < pm_fail_i2c_max; i++) {
		if (ulFailI2c[i][0] == addr) {
			ulFailI2c[i][1]++;
			break;
		} else if (ulFailI2c[i][0] == 0) {
			ulFailI2c[i][0] = addr;
			ulFailI2c[i][1]++;
			break;
		}
	}
}

/*
 * Function for hw_dma_channel_active
 */
__RETAINED_CODE void pm_increase_dma_cnt(int channel_flag)
{
	int i;

	for (i = 0; i < HW_DMA_CHANNEL_INVALID; i++) {
		if ((channel_flag >> i) & 1)
			ulFailDma[i]++;
	}
}

/*
 * Function for FreeRTOS macro
 */
__RETAINED_CODE void prvConfigureTimerForRunTimeStats(void)
{
	pm_data.ulSysTickNow = hw_timer_get_count(HW_TIMER2);
}

__RETAINED_CODE uint32_t prvGetRunTimeCounterValue(void)
{
	uint32_t ulSysTickPrev = pm_data.ulSysTickNow;
	
	pm_data.ulSysTickNow = hw_timer_get_count(HW_TIMER2);
	pm_data.ulRunTimeTick += (pm_data.ulSysTickNow - ulSysTickPrev) & LP_CNT_NATIVE_MASK;

	return pm_data.ulRunTimeTick;
}


/*
 * External Function definitions
 */
__RETAINED_CODE void pm_update_blocker_cnt(sleep_blocker_t blocker)
{
	if (blocker < pm_sleep_blocker_max) {
		uLLSleepBlocker[blocker]++;
	}
}

__RETAINED_CODE void pm_update_bad_adapter_cnt(int adapter)
{
	if (adapter < dg_configPM_MAX_ADAPTERS_CNT) {
		uLLBadAdapters[adapter]++;
	}
}

__RETAINED_CODE void pm_update_active_time(int update)
{
	uint32_t unused, rtc_now; 

	rtc_now = sys_timer_get_timestamp_fromCPM(&unused);

	if (update) {
		pm_data.ulTimeSpentActiveInTicks += (rtc_now - pm_data.ulActiveStartTicks);
		pm_data.ulTrySleepStartTicks = rtc_now;
	} else {
		pm_data.ulActiveStartTicks = rtc_now;
		pm_data.ulTimeSpentTrySleepInTicks += (rtc_now - pm_data.ulTrySleepStartTicks);	
	}	
}

__RETAINED_CODE void pm_update_sleep_time(uint32_t sleep_ticks)
{
	system_state_t system_state;

	system_state = pm_get_system_sleep_state();

	if (system_state == sys_powered_down) {
		pm_data.ulTimeSpentSysPoweredDownInTicks += sleep_ticks;
	} else {
		pm_data.ulTimeSpentSysIdleInTicks += sleep_ticks;
	}
}

void pm_get_data(t_pm_data *data)
{
	uint16_t i;
	enum charge_status chg;
	uint64_t uLLRTCTickNow;
	uint32_t ulChargeVoltNow, ulChargeSocNow;

	OS_ASSERT(data != NULL);

	pm_data.ulTaskNum = task_stat_update_and_get_num();
	for (i = 0; i < pm_data.ulTaskNum; i++) {
		task_stat_get_name(i, pm_data.stTaskName[i], PM_TASK_NAME_MAX);
		pm_data.stTaskName[i][PM_TASK_NAME_MAX] = '\0';
		pm_data.ulTaskRunTimeCounter[i] = task_stat_get_tick(i);
	}

	chg = sec_batt_get_chg_status();
	if (chg != SEC_BATT_DISCHG) {
		uLLRTCTickNow = sys_timer_get_timestamp();
		pm_data.uLLTotalChargeTicks += uLLRTCTickNow - pm_data.uLLChargeTicksNow;
		ulChargeVoltNow = ad_battery_get_voltage();
		pm_data.lTotalChargeVolt += ulChargeVoltNow - pm_data.lLastChargeVolt;
		ulChargeSocNow = sec_batt_get_soc();
		pm_data.lTotalChargeSoc += ulChargeSocNow - pm_data.lLastChargeSoc;

		pm_data.uLLChargeTicksNow = uLLRTCTickNow;
		pm_data.lLastChargeVolt = ulChargeVoltNow;
		pm_data.lLastChargeSoc = ulChargeSocNow;
	}

	memcpy(data, &pm_data, sizeof(t_pm_data));
}

#define PM_PRINT_TARGET_UART	1
#define PM_PRINT_TARGET_BUFFER	2
static void _pm_print(int target, char *dst, char *src, int32_t *offset, size_t max_offset)
{
	OS_ASSERT(src);

	if (target == PM_PRINT_TARGET_UART)
		MSG_WARN(DBG_PM|DBG_SLEEP, "%s", src);
	else {
		OS_ASSERT(dst);
		OS_ASSERT(offset);
		if (*offset < max_offset) {
			memcpy(dst + *offset, src, strlen(src));
			*offset += strlen(src);
		}
	}
}

static char str_ull[22];
static char *_ull_to_str(uint64_t data)
{
	uint64_t temp = data;
	int i;

	memset(str_ull, 0, sizeof(str_ull));

	if (temp == 0) {
		str_ull[0] = '0';
	} else {
		while (temp > 0 && strlen(str_ull) < sizeof(str_ull) - 1) {
			for (i = strlen(str_ull); i > 0; i--) {
				str_ull[i] = str_ull[i - 1];
			}
			str_ull[0] = '0' + (temp % 10);
			temp /= 10;
			//MSG_WARN(DBG_PM|DBG_SLEEP, "%s (%d)\r\n", str_ull, temp);
		}
	}
	return str_ull;
}

static int32_t _pm_print_run_time_stat(int target, char *buf, size_t length)
{
	uint32_t ulTotalRunTime, ulSumRunTime, ulRTCTime;
	uint16_t i, j;
	char cTempBuf[256];
	int ret;
	int32_t offset = 0;

	OS_ASSERT(target == PM_PRINT_TARGET_UART || target == PM_PRINT_TARGET_BUFFER);
	if (target == PM_PRINT_TARGET_BUFFER)
		OS_ASSERT(buf);

	task_stat_update_and_get_num();
	ulSumRunTime = task_stat_get_sum_tick();
	ulTotalRunTime = task_stat_get_total_tick();

	if( ulTotalRunTime == 0 && target == PM_PRINT_TARGET_UART ) {
		MSG_WARN(DBG_PM|DBG_SLEEP, "sec_pm ulTotalRunTime error(0)\n");
		return 0;
	}

	ulRTCTime = (uint32_t)sys_timer_get_timestamp();

	MSG_WARN(DBG_PM|DBG_SLEEP, "=====================================\r\n");
	
	/* ---------------------------------------- */
	/* Active */
	ret = 0;
	memset(cTempBuf, 0, sizeof(cTempBuf));
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%lu %lu %lu ", ulSumRunTime, ulRTCTime, ulTotalRunTime);
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%s ", _ull_to_str(pm_data.uLLTotalScreenOnTicks));
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%s ", _ull_to_str(pm_data.uLLTotalBLECoreSleepTicks));
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%s ", _ull_to_str(pm_data.uLLBLECoreSleepCnt));
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%s\r\n", _ull_to_str(pm_data.uLLTotalChargeTicks));
	_pm_print(target, buf, cTempBuf, &offset, length);

	ret = 0;
	memset(cTempBuf, 0, sizeof(cTempBuf));
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%lu %lu %lu %lu %lu\r\n",
		pm_data.ulTimeSpentActiveInTicks + pm_data.ulTimeSpentTrySleepInTicks, pm_data.ulTimeSpentActiveInTicks,
		pm_data.ulTimeSpentTrySleepInTicks, pm_data.ulTimeSpentSysIdleInTicks * TICK_PERIOD,
		pm_data.ulTimeSpentSysPoweredDownInTicks * TICK_PERIOD);	
	_pm_print(target, buf, cTempBuf, &offset, length);

	ret = 0;
	memset(cTempBuf, 0, sizeof(cTempBuf));
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%s ", _ull_to_str(pm_data.uLLTotalClkTicks[0]));
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%s ", _ull_to_str(pm_data.uLLTotalClkTicks[1]));
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%d %lu %lu\r\n",
		pm_data.eCurrentClk, pm_data.ulClkChgReqCnt[0], pm_data.ulClkChgReqCnt[1]);
	_pm_print(target, buf, cTempBuf, &offset, length);

	/* ---------------------------------------- */
	/* Sleep */
	ret = 0;
	memset(cTempBuf, 0, sizeof(cTempBuf));
	for (i = 0; i < pm_sleep_blocker_max; i++)
		ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%s ", _ull_to_str(uLLSleepBlocker[i]));
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "\r\n");
	_pm_print(target, buf, cTempBuf, &offset, length);

	ret = 0;
	memset(cTempBuf, 0, sizeof(cTempBuf));
	for (i = 0; i < dg_configPM_MAX_ADAPTERS_CNT; i++)
		ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%s ", _ull_to_str(uLLBadAdapters[i]));
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "\r\n");
	_pm_print(target, buf, cTempBuf, &offset, length);

	for (j = 0; j < SLEEP_MODE_MAX; j++) {
		ret = 0;
		memset(cTempBuf, 0, sizeof(cTempBuf));
		for (i = 0; i < pm_wc_max; i++)
			ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%lu ", pm_data.ulWakeupRequestClient[j][i]);
		ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "\r\n");
		_pm_print(target, buf, cTempBuf, &offset, length);

		ret = 0;
		memset(cTempBuf, 0, sizeof(cTempBuf));
		for (i = 0; i < pm_wc_max; i++)
			ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%lu ", pm_data.ulWakeupReleaseClient[j][i]);
		ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "\r\n");
		_pm_print(target, buf, cTempBuf, &offset, length);

		ret = 0;	
		memset(cTempBuf, 0, sizeof(cTempBuf));
		for (i = 0; i < pm_wc_max; i++)
			ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%s ", _ull_to_str(pm_data.uLLTotalPreventTick[j][i]));
		ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "\r\n");
		_pm_print(target, buf, cTempBuf, &offset, length);
	}

	ret = 0;
	memset(cTempBuf, 0, sizeof(cTempBuf));
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%lu\r\n", pm_data.ulWakeupDiffSum);
	_pm_print(target, buf, cTempBuf, &offset, length);

	ret = 0;
	memset(cTempBuf, 0, sizeof(cTempBuf));
	for (i = 0; i < pm_fail_i2c_max; i++) {
		if (ulFailI2c[i][0] == 0)
			break;
		ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "0x%lx %lu ", ulFailI2c[i][0], ulFailI2c[i][1]);
	}
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "\r\n");
	_pm_print(target, buf, cTempBuf, &offset, length);

	ret = 0;
	memset(cTempBuf, 0, sizeof(cTempBuf));
	for (i = 0; i < HW_DMA_CHANNEL_INVALID; i++)
		ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%lu ", ulFailDma[i]);
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "\r\n\r\n");
	_pm_print(target, buf, cTempBuf, &offset, length);

	/* ---------------------------------------- */
	/* Sensor */
	ret = 0;
	memset(cTempBuf, 0, sizeof(cTempBuf));
	for (i = 0; i < PM_SENSOR_MAX; i++)
		ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%lu %lu ", ulSensorRefCnt[i][0], ulSensorRefCnt[i][1]);
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "\r\n");
	_pm_print(target, buf, cTempBuf, &offset, length);

	ret = 0;
	memset(cTempBuf, 0, sizeof(cTempBuf));
	for (i = 0; i < PM_SENSOR_MAX; i++)
		ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%lu %lu ", ulSensorRate[i][0], ulSensorRate[i][1]);
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%d 0x%x 0x%x\r\n", ulSensorUpdateRate, ulRegSensorAlgo, ulRegSensorAlgoDM);
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "\r\n");
	_pm_print(target, buf, cTempBuf, &offset, length);

	ret = 0;
	memset(cTempBuf, 0, sizeof(cTempBuf));
	for (i = 0; i < PM_SENSOR_ALG_MAX; i++)
		ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%lu ", pm_data.ulSensorAlgoEvt[i]);
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "\r\n");
	_pm_print(target, buf, cTempBuf, &offset, length);

	ret = 0;
	memset(cTempBuf, 0, sizeof(cTempBuf));
	for (i = 0; i < PM_SENSOR_ALG_MAX; i++)
		ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%s ", _ull_to_str(pm_data.uLLTotalAlgoTick[i]));
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "\r\n");
	_pm_print(target, buf, cTempBuf, &offset, length);

	ret = 0;
	memset(cTempBuf, 0, sizeof(cTempBuf));
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%s ", _ull_to_str(pm_data.uLLTotalSensorTick));
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%lu\r\n\r\n", pm_data.ulSensorUpdateCnt);
	_pm_print(target, buf, cTempBuf, &offset, length);

	/* ---------------------------------------- */
	/* SAP */
	ret = 0;
	memset(cTempBuf, 0, sizeof(cTempBuf));
	for (i = 0; i < MAX_NONITOR_SAP_ID; i++)
		ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%lu ", pm_data.ulSapSndCnt[i]);
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "\r\n");
	_pm_print(target, buf, cTempBuf, &offset, length);

	ret = 0;
	memset(cTempBuf, 0, sizeof(cTempBuf));
	for (i = 0; i < MAX_NONITOR_SAP_ID; i++)
		ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "%lu ", pm_data.ulSapRcvCnt[i]);
	ret += snprintf(cTempBuf + ret, sizeof(cTempBuf) - ret, "\r\n");
	_pm_print(target, buf, cTempBuf, &offset, length);

	if (target == PM_PRINT_TARGET_UART) {
		task_stat_print();
	}

	MSG_WARN(DBG_PM|DBG_SLEEP, "=====================================\r\n");

	if (target == PM_PRINT_TARGET_BUFFER) {
		*(buf + offset) = '\0';
		offset++;
	}

	return offset;
}

void pm_show_run_time_stat(void)
{
	_pm_print_run_time_stat(PM_PRINT_TARGET_UART, NULL, 0);
}

int32_t pm_dump_run_time_stat(char *buf, int32_t length)
{
	return _pm_print_run_time_stat(PM_PRINT_TARGET_BUFFER, buf, length);
}
