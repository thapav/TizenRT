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
 * os/pm/pm_idle.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <debug.h>

#include <tinyara/board.h>
#include <tinyara/pm/pm.h>

#include <tinyara/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PM_IDLE_DOMAIN	0

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: pm_idle_process
 *
 * Description:
 *   Perform IDLE state power management.
 *
 ****************************************************************************/
static void pm_idle_process(void)
{
	static int oldstate = PM_NORMAL;
	int newstate;
	irqstate_t flags;
	int ret;

	/* Decide, which power saving level can be obtained */
	newstate = pm_checkstate(PM_IDLE_DOMAIN);

	/* Check for state changes */
	if (newstate != oldstate) {
		flags = irqsave();

		/* Then force the global state change */
		ret = pm_changestate(PM_IDLE_DOMAIN, newstate);
		if (ret < 0) {
			/* The new state change failed, revert to the preceding state */
			pm_changestate(PM_IDLE_DOMAIN, oldstate);
		} else {
			/* Save the new state */
			oldstate = newstate;
		}

		/* SOC-specific power management logic */
		ret = up_pm_switch_state(newstate);
		if (ret < 0) {
			/* Staying in same state as SOC returned error */
			if((newstate - 1) >= 0) {
				up_pm_switch_state(newstate - 1);
			}
		}

		irqrestore(flags);
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_idle
 *
 * Description:
 *   pm_idle() is the logic that will be executed when their is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 ****************************************************************************/
void pm_idle(void)
{

#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
	/*
	 * If the system is idle and there are no timer interrupts, then process
	 * "fake" timer interrupts. Hopefully, something will wake up.
	 */
	sched_process_timer();
#else
	/* Perform IDLE mode power management */
	pm_idle_process();

	/* Sleep until an interrupt occurs to save power. */
	asm("WFI");
#endif
}
