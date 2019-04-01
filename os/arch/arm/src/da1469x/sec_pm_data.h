/**
 ****************************************************************************************
 *
 * @file sec_pm_data.h
 *
 * @brief Samsung Power Manager data header file.
 *
 * Copyright (C) 2018-2019 Samsung Electronics Co. Ltd.
 * This computer program includes Confidential, Proprietary Information
 * of Samsung Electronics Co. Ltd.. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef SEC_PM_DATA_H_
#define SEC_PM_DATA_H_

#include "hw_dma.h"
#include "FreeRTOSConfig.h"
#if SAP_ENABLED == 1
#include "sap_reserved_services.h"
#endif
#include "hw_gpio.h"
#include "algmanager.h"
#include "sec_pm.h"

#if SAP_ENABLED == 1
#define MAX_NONITOR_SAP_ID	(BIGDATA_SERVICE_ID + 1)
#else
#define MAX_NONITOR_SAP_ID	0
#endif
#define PM_SENSOR_ALG_MAX	(ALG_MAX)

#define MAX_NONITOR_TASK	50
#define PM_TASK_NAME_MAX	10

/**
 * \brief Power manager data structure
 *
 */
typedef struct {
	uint32_t ulSysTickNow;
	uint32_t ulRunTimeTick; 

	uint32_t ulActiveStartTicks;
	uint32_t ulTimeSpentActiveInTicks;
	uint32_t ulTrySleepStartTicks;
	uint32_t ulTimeSpentTrySleepInTicks; 
	uint32_t ulTimeSpentSysPoweredDownInTicks;
	uint32_t ulTimeSpentSysIdleInTicks;

	uint32_t uLLSleepBlocker[pm_sleep_blocker_max];
	uint32_t uLLBadAdapters[dg_configPM_MAX_ADAPTERS_CNT];
	uint32_t cWakeupClient[pm_wc_max]; // wakeup active count	
	uint32_t cWakeupClientDeactiveCnt[pm_wc_max]; // wakeup deactive count
	uint64_t uLLLastChangeTick[pm_wc_max]; // wakeup client last change
	uint64_t uLLTotalPreventTick[pm_wc_max]; // total prevent time
	uint32_t ulFailI2c[pm_fail_i2c_max][2];
	uint32_t ulFailDma[HW_DMA_CHANNEL_INVALID];

	uint64_t uLLScreenOnTicksNow;
	uint64_t uLLTotalScreenOnTicks;
	uint32_t ulScreenOnCnt;

	uint32_t ulTouchCnt;
	uint32_t ulKeyCnt;
	uint32_t ulWristUpCnt;
	uint32_t ulShowNotiCnt;
	uint32_t ulShowFindMyBandCnt;
	uint32_t ulShowAlarmCnt;
	uint32_t ulShowHealthAlertCnt;
	uint32_t ulShowCallCnt;
	uint32_t ulShowReminderCnt;

	uint32_t ulSapSndCnt[MAX_NONITOR_SAP_ID];
	uint32_t ulSapRcvCnt[MAX_NONITOR_SAP_ID];

	uint32_t ulTaskNum;
	char stTaskName[MAX_NONITOR_TASK][PM_TASK_NAME_MAX + 1];
	uint32_t ulTaskRunTimeCounter[MAX_NONITOR_TASK];
	
	uint64_t uLLTotalClkTicks[clk_max];

	uint64_t uLLTotalChargeTicks;
	uint32_t ulChargeState;

	uint64_t uLLTotalMotorTicks;
	uint32_t ulMotorCnt;

	uint64_t uLLTotalSensorTick;
	uint32_t ulSensorUpdateCnt;
	uint64_t uLLTotalAlgoTick[PM_SENSOR_ALG_MAX];
	uint32_t ulSensorAlgoEvt[PM_SENSOR_ALG_MAX];
} t_pm_data;


#ifdef CONFIG_USE_SEC_PM

void pm_get_data(t_pm_data *data);

#else /* CONFIG_USE_SEC_PM */

static inline void pm_get_data(t_pm_data *data) {}

#endif /* CONFIG_USE_SEC_PM */

#endif /* SEC_PM_DATA_H_ */
