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
dbg("DBG: entered\n");
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
dbg("DBG: entered\n");
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
dbg("DBG: entered\n");
        /* FIXME: implement me */
        asm("wfi");

        /* won't get here */

	return errno;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Switch to the new sleep state suggested by the PM framework */
int up_pmidle_switch_state(int state_ID)
{
	int ret = OK;

	switch(state_ID) {
	case S5J_NORMAL:
		ret = s5j_pmnormal();
		break;

	case S5J_STOP:
		ret = s5j_pmstop();
		break;

	case S5J_STANDBY:
		ret = s5j_pmstandby();
		break;

	default:
		break;
	}

	return ret;
}
