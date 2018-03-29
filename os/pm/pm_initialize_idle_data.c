/****************************************************************************
 *
 * Copyright 2018 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 *
 * os/pm/pm_initialize_idle_data.c
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <queue.h>
#include <assert.h>
#include <debug.h>

#include <tinyara/pm/pm.h>
#include "pm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_initialize_idle_data
 *
 * Description:
 *   Map the CPU IDLE power states & their transition timings to the TizenRT
 *   power management framework. If threshold counts are not provided by SOC,
 *   initialize them with default PM values of thresholds.
 *
 ****************************************************************************/

void pm_initialize_idle_data(void)
{
	int index = 0;
	bool assign_defaultvalue = true;

#ifdef CONFIG_PM_STATE_SWITCH_THRESH
	/* Initialize state threshold value to be read from thresh_str */
	int threshold = 0;
	const char *thresh_str = CONFIG_PM_STATE_SWITCH_THRESH;

	/* Variable to track number of tick count threshold provided by the SOC */
	int check_count = 0;

	/* Normal state tick count threshold is NA, hence initialized to 0 */
	pm_sleep_state_switch_thresholds[index++] = 0;

	/* Parse the sleep state tick count thresholds from the Kconfig */

	while(*thresh_str) {

		/* Extract thresholds from string input in config as integer value */
		threshold = strtoul(thresh_str, NULL, 0);

		while(*thresh_str != ',' && *thresh_str) {
			thresh_str++;
		}

		if(*thresh_str == ',') {
			thresh_str++;
		}

		check_count++;

		/* Case when more state threshold values are provided by PM_STATE_COUNT_THRESH*/
		if(check_count > (CONFIG_PM_NSTATE - 1)) {
			DEBUGASSERT(!(check_count > (CONFIG_PM_NSTATE - 1)));
		}

		/* Update sleep state threshold array with extracted integer value */
		pm_sleep_state_switch_thresholds[index++] = threshold;
	}

	/* Check if all the state entering tick count thresholds are provided by the SOC */

	if(check_count == (CONFIG_PM_NSTATE - 1)) {
	/* Case when all state threshold values are provided by PM_STATE_COUNT_THRESH*/

		assign_defaultvalue = false;

	} else if((check_count > 0 && check_count < (CONFIG_PM_NSTATE - 1))) {
	/* Case when less state threshold values are provided by PM_STATE_COUNT_THRESH*/

		DEBUGASSERT(!(check_count > 0 && check_count < CONFIG_PM_NSTATE))

	} else {
	/* Case when no state threshold values are provided by PM_STATE_COUNT_THRESH*/

		assign_defaultvalue = true;

	}
#endif
	/* If assign_defaultvalue is true (any of the state threshold values are not provided),
	 * give default threshold values (in steps of 10).
	 */

	if(assign_defaultvalue) {

		int incremental_tick_count = PM_DEFAULT_STATE_SWITCH_THRESH;

		for(index = 1; index < CONFIG_PM_NSTATE; index++) {
			pm_sleep_state_switch_thresholds[index] = incremental_tick_count;
			incremental_tick_count = incremental_tick_count + PM_DEFAULT_STATE_SWITCH_THRESH;
		}
	}

	/* Reset flag to assign default threshold values */
	assign_defaultvalue = true;

#ifdef CONFIG_PM_STATE_ENTER_THRESH

	/* Initialize state threshold value to be read from thresh_enter */
	int threshold_enter = 0;
	const char *thresh_enter = CONFIG_PM_STATE_ENTER_THRESH;

	/* Variable to track number of activity threshold provided by the SOC */
	check_count = 0;

	/* Normal state entering threshold is NA, hence initialized to 0 */
	index = 0;
	pm_sleep_enter_activity_thresholds[index++] = 0;

	/* Parse the sleep state enter thresholds from the Kconfig */

	while(*thresh_enter) {

		/* Extract thresholds from string input in config as integer value */
		threshold_enter = strtoul(thresh_enter, NULL, 0);

		while(*thresh_enter != ',' && *thresh_enter) {
			thresh_enter++;
		}

		if(*thresh_enter == ',') {
			thresh_enter++;
		}

		check_count++;

		/* Case when more state threshold values are provided by PM_STATE_ENTER_THRESH*/
		if(check_count > (CONFIG_PM_NSTATE - 1)) {
			DEBUGASSERT(!(check_count > (CONFIG_PM_NSTATE - 1)));
		}

		/* Update sleep state threshold array with extracted integer value */
		pm_sleep_enter_activity_thresholds[index++] = threshold_enter;
	}

	/* Check if all the state entering tick count thresholds are provided by the SOC */

	if(check_count == (CONFIG_PM_NSTATE - 1)) {
	/* Case when all state threshold values are provided by PM_STATE_ENTER_THRESH*/

		assign_defaultvalue = false;

	} else if((check_count > 0 && check_count < (CONFIG_PM_NSTATE - 1))) {
	/* Case when less state threshold values are provided by PM_STATE_ENTER_THRESH*/

		DEBUGASSERT(!(check_count > 0 && check_count < CONFIG_PM_NSTATE))

	} else {
	/* Case when no state threshold values are provided by PM_STATE_ENTER_THRESH*/

		assign_defaultvalue = true;

	}
#endif

	/* If assign_defaultvalue is true (any of the state enter threshold values are not provided),
	 * give default threshold values (in steps of 10).
	 */

	if(assign_defaultvalue) {

		for(index = 1; index < CONFIG_PM_NSTATE; index++) {
			pm_sleep_enter_activity_thresholds[index] = PM_DEFAULT_STATE_ENTER_THRESH;
		}
	}

	/* Reset flag to assign default threshold values */
	assign_defaultvalue = true;

#ifdef CONFIG_PM_STATE_EXIT_THRESH

	/* Initialize state threshold value to be read from thresh_exit */
	int threshold_exit = 0;
	const char *thresh_exit = CONFIG_PM_STATE_EXIT_THRESH;

	/* Variable to track number of activity threshold provided by the SOC */
	check_count = 0;

	/* Normal state entering threshold is NA, hence initialized to 0 */
	index = 0;
	pm_sleep_enter_activity_thresholds[index++] = 0;

	/* Parse the sleep state enter thresholds from the Kconfig */

	while(*thresh_exit) {

		/* Extract thresholds from string input in config as integer value */
		threshold_exit = strtoul(thresh_exit, NULL, 0);

		while(*thresh_exit != ',' && *thresh_exit) {
			thresh_exit++;
		}

		if(*thresh_exit == ',') {
			thresh_exit++;
		}

		check_count++;

		/* Case when more state threshold values are provided by PM_STATE_EXIT_THRESH*/
		if(check_count > (CONFIG_PM_NSTATE - 1)) {
			DEBUGASSERT(!(check_count > (CONFIG_PM_NSTATE - 1)));
		}

		/* Update sleep state threshold array with extracted integer value */
		pm_sleep_exit_activity_thresholds[index++] = threshold_exit;
	}

	/* Check if all the state entering tick count thresholds are provided by the SOC */

	if(check_count == (CONFIG_PM_NSTATE - 1)) {
	/* Case when all state threshold values are provided by PM_STATE_EXIT_THRESH*/

		assign_defaultvalue = false;

	} else if((check_count > 0 && check_count < (CONFIG_PM_NSTATE - 1))) {
	/* Case when less state threshold values are provided by PM_STATE_EXIT_THRESH*/

		DEBUGASSERT(!(check_count > 0 && check_count < CONFIG_PM_NSTATE))

	} else {
	/* Case when no state threshold values are provided by PM_STATE_EXIT_THRESH*/

		assign_defaultvalue = true;

	}
#endif

	/* If assign_defaultvalue is true (any of the state exit threshold values are not provided),
	 * give default threshold values (in steps of 10).
	 */

	if(assign_defaultvalue) {

		for(index = 1; index < CONFIG_PM_NSTATE; index++) {
			pm_sleep_exit_activity_thresholds[index] = PM_DEFAULT_STATE_EXIT_THRESH;
		}
	}
}
