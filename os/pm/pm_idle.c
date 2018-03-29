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
 *   Copyright (C) 2011-2012, 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
			/* SOC-specific power management logic */
			ret = up_pm_switch_state(newstate);
			if (ret < 0) {
				/* Staying in same state as SOC returned error */
				ret = up_pm_switch_state(oldstate);
				ret = pm_changestate(PM_IDLE_DOMAIN, oldstate);
			} else {
				/* Save the new state */
				oldstate = newstate;
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
	/* Perform IDLE mode power management */
	pm_idle_process();
}
