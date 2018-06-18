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
 *  arch/arm/src/s5j/s5j_pm_idle_init_data.c
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <errno.h>

#include <tinyara/config.h>
#include <tinyara/arch.h>
#include <tinyara/board.h>
#include <tinyara/irq.h>
#include <tinyara/pm/pm.h>

#include "s5j_clock.h"
#include "s5j_pm.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Threshold time slice count to enter the next low power consdumption
 * state. Indexing is next state 0:IDLE, 1: STANDBY, 2: SLEEP.
 */

#define CONFIG_SOC_STATE1_ENTER_COUNT   30	/* 30 time slice counts: 30 * 100msec = 3sec */
#define CONFIG_SOC_STATE2_ENTER_COUNT   50	/* 50 time slice counts: 50 * 100msec = 5sec */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s5j_idle_pmnormal
 *
 * Description:
 *   Enter NORMAL mode.
 *
 ****************************************************************************/
static int s5j_idle_pmnormal(void)
{
        s5j_clk_pll_select_mux(true);

	return errno;
}
/****************************************************************************
 * Name: s5j_idle_pmstop
 *
 * Description:
 *   Enter STOP mode.
 *
 ****************************************************************************/
static int s5j_idle_pmstandby(void)
{
        s5j_clk_pll_select_mux(false);

	return errno;
}

/****************************************************************************
 * Name: s5j_idle_pmstandby
 *
 * Description:
 *   Enter STANDBY mode.
 *
 * Input Parameters:
 *   None
 *
 ****************************************************************************/
static int s5j_idle_pmstop(void)
{
        /* FIXME: implement me */
        asm("wfi");

        /* won't get here */

	return errno;
}

/****************************************************************************
 * Public Prototypes
 ****************************************************************************/
/* Provide sleep state function callback addresses & timer thresholds if any */

struct pm_idle_state_info pm_idle_table[] = {
	{&s5j_idle_pmnormal, 0},
	{&s5j_idle_pmstandby, CONFIG_SOC_STATE1_ENTER_COUNT},
	{&s5j_idle_pmstop, CONFIG_SOC_STATE2_ENTER_COUNT}
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/* Provide sleep state's data pointer */

struct pm_idle_state_info *get_pm_idle_data(void)
{
	return pm_idle_table;
}

/* Provide sleep state count */

int get_pm_sleep_state_count(void)
{
	/* Divide size of pm_idle_table by size of pm_idle_table[0] (one element) to get the sleep state count */
	int count = sizeof(pm_idle_table) / sizeof(struct pm_idle_state_info);

	/* Exclude normal state from the state count */
	count = count - 1;

	return count;
}
