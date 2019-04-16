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
#if SAP_ENABLED == 1
#include "sap_reserved_services.h"
#endif
#include "hw_gpio.h"
#include "sys_power_mgr.h"
//#include "algmanager.h"
#include "sec_pm.h"

#define SLEEP_MODE_MAX		2

#if SAP_ENABLED == 1
#define MAX_NONITOR_SAP_ID	(BIGDATA_SERVICE_ID + 1)
#else
#define MAX_NONITOR_SAP_ID	0
#endif
//#define PM_SENSOR_ALG_MAX	(ALG_MAX)

#define MAX_NONITOR_TASK	50
#define PM_TASK_NAME_MAX	10

/**
 * \brief Power manager data structure
 *
 */
typedef struct {
	uint32_t ulSysTickNow;
	uint32_t ulRunTimeTick; 

	uint64_t uLLChargeTicksNow;
	uint64_t uLLTotalChargeTicks;
	int32_t lLastChargeVolt;
	int32_t lTotalChargeVolt;
	int32_t lLastChargeSoc;
	int32_t lTotalChargeSoc;

	uint32_t ulActiveStartTicks;
	uint32_t ulTimeSpentActiveInTicks;
	uint32_t ulTrySleepStartTicks;
	uint32_t ulTimeSpentTrySleepInTicks; 
	uint32_t ulTimeSpentSysPoweredDownInTicks;
	uint32_t ulTimeSpentSysIdleInTicks;

	uint64_t uLLLastChangeTick[pm_mode_idle + 1][pm_wc_max]; // wakeup client last change
	uint64_t uLLTotalPreventTick[pm_mode_idle + 1][pm_wc_max]; // total prevent time
	uint32_t ulWakeupRequestClient[SLEEP_MODE_MAX][pm_wc_max]; // wakeup request count	
	uint32_t ulWakeupReleaseClient[SLEEP_MODE_MAX][pm_wc_max]; // wakeup release count
	uint32_t ulWakeupDiffSum; 				// wakeup cnt mismatch

	uint64_t uLLScreenOnTicksNow;
	uint64_t uLLTotalScreenOnTicks;
	uint32_t ulScreenOnCnt;
	
	int eCurrentClk;
	uint32_t ulClkChgReqCnt[clk_max];
	uint64_t uLLClkChgTicksNow;
	uint64_t uLLTotalClkTicks[clk_max];
	
	uint64_t uLLBLECoreSleepTicksNow;
	uint64_t uLLTotalBLECoreSleepTicks;
	uint64_t uLLBLECoreSleepCnt;
	uint32_t ulBLECoreCurrentState;
	
	uint64_t uLLMotorTicksNow;
	uint64_t uLLTotalMotorTicks;
	uint32_t ulMotorCnt;

	uint32_t ulTouchCnt;
	uint32_t ulKeyCnt;
	uint32_t ulWristUpCnt;
	uint32_t ulShowNotiCnt;
	uint32_t ulShowFindMyBandCnt;
	uint32_t ulShowAlarmCnt;
	uint32_t ulShowHealthAlertCnt;
	uint32_t ulShowCallCnt;
	uint32_t ulShowReminderCnt;

	uint32_t ulTaskNum;
	char stTaskName[MAX_NONITOR_TASK][PM_TASK_NAME_MAX + 1];
	uint32_t ulTaskRunTimeCounter[MAX_NONITOR_TASK];

	uint32_t ulSapSndCnt[MAX_NONITOR_SAP_ID];
	uint32_t ulSapRcvCnt[MAX_NONITOR_SAP_ID];

	uint64_t uLLSensorStartTick;
	uint64_t uLLLastSensorTick;
	uint64_t uLLTotalSensorTick;
	uint32_t ulSensorUpdateCnt;
	uint64_t uLLAlgoStartTick;
//	uint64_t uLLLastAlgoTick[PM_SENSOR_ALG_MAX];
//	uint64_t uLLTotalAlgoTick[PM_SENSOR_ALG_MAX];
//	uint32_t ulSensorAlgoEvt[PM_SENSOR_ALG_MAX];
} t_pm_data;


#ifdef CONFIG_USE_SEC_PM

void pm_get_data(t_pm_data *data);

#else /* CONFIG_USE_SEC_PM */

static inline void pm_get_data(t_pm_data *data) {}

#endif /* CONFIG_USE_SEC_PM */

#endif /* SEC_PM_DATA_H_ */
