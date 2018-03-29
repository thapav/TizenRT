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
 *  arch/arm/src/s5j/s5j_pm_idle_table.c
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>
#include <debug.h>

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
#define PM_IDLE_DOMAIN 0 /* Revisit */
#define CONFIG_SOC_STATE1_ENTER_COUNT   30
#define CONFIG_SOC_STATE2_ENTER_COUNT   50

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: s5j_idle_pmnormal
 *
 * Description:
 *   Enter NORMAL mode.
 *
 ****************************************************************************/
static void s5j_idle_pmnormal(void)
{
        s5j_clk_pll_select_mux(true);
}
/****************************************************************************
 * Name: s5j_idle_pmstop
 *
 * Description:
 *   Enter STOP mode.
 *
 ****************************************************************************/
static void s5j_idle_pmstop(void)
{
        s5j_clk_pll_select_mux(false);
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
static void s5j_idle_pmstandby(void)
{
        /* FIXME: implement me */
        asm("wfi");

        /* won't get here */
}

struct pm_idle_state_info pm_idle_table[] = {
	{&s5j_idle_pmnormal, 0},
	{&s5j_idle_pmstop, CONFIG_SOC_STATE1_ENTER_COUNT},
	{&s5j_idle_pmstandby, CONFIG_SOC_STATE2_ENTER_COUNT}
};

int get_pm_idle_table_size(void)
{
	int count;
	int a = sizeof(pm_idle_table);
	int b = sizeof(struct pm_idle_state_info);

	count = a / b;
	/* Exclude normal state from the state count */
	count = count -1;

	return count;
}
