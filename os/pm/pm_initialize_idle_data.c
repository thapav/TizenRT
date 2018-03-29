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
 *   power management framework. If tick counts are not provided by SOC,
 *   initialize them with default PM values of thresholds.
 *
 ****************************************************************************/

void pm_initialize_idle_data(void)
{
	int i;
	int incremental_tick_count;
	bool assign_defaultvalue = true;
	struct pm_idle_state_info *pdat = get_pm_idle_data();

	dbg("DBG: sleep_states_count =  %d\n", sleep_states_count);

	/* Check if all the state entering tick count values provided by SOC are 0 */

	if(sleep_states_count > 0) {
		for(i = 1; i <= sleep_states_count; i++) {
			if((pdat+i)->sleep_threshold != 0) {
				assign_defaultvalue = false;
				break;
			}
		}
	}

	/* Check if all the state entering tick count values provided by SOC are 0 */

	if(assign_defaultvalue) {

		incremental_tick_count = CONFIG_PM_ENTER_COUNT;

		for(i = 1; i <= sleep_states_count; i++) {
			(pdat+i)->sleep_threshold = incremental_tick_count;
			incremental_tick_count = incremental_tick_count + CONFIG_PM_ENTER_COUNT;
		}
	}

}
