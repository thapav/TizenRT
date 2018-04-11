/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
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
 * Included Files
 ****************************************************************************/

#include <apps/shell/tash.h>
#include <pm_metrics.h>
#include <tinyara/config.h>
#include <debug.h>
#include <unistd.h>
#include <tinyara/kthread.h>
#include <sched.h>
#include <time.h>
#include <pm.h>
#include "pm.h"
#include "pm_debug.h"
#include "pm_metrics.h"
#include "pm_test.h"

/****************************************************************************
 * Global Variables
 ****************************************************************************/

struct pm_callback_s pmtest_cbarray[PMTEST_DEVICES];

/****************************************************************************
 * Local functions
 ****************************************************************************/

#ifndef CONFIG_IDLE_PM
void pmtest_dev_notify(struct pm_callback_s *cb, int domain, enum pm_state_e state)
{
	pmvdbg("Recevied notify call for %s device in %d domain for %d state \n", cb->name, domain, state);
}

int pmtest_dev_prepare(struct pm_callback_s *cb, int domain, enum pm_state_e state)
{
	pmvdbg("Recevied prepare call for %s device in %d domain for %d state \n", cb->name, domain, state);
	return OK;
}

/* PM test routine */

static int pmtest_kthread(int argc, char *argv[])
{
	enum pm_state_e s = PM_IDLE;
#ifdef CONFIG_PM_METRICS
	struct pm_time_in_each_s m;
#endif
	struct timespec start_time;
	struct timespec elapsed_time;

	pmvdbg("This test runs for 7200 secs\n");
	clock_gettime(CLOCK_REALTIME, &start_time);
	elapsed_time.tv_sec = start_time.tv_sec;

	while (elapsed_time.tv_sec <= (start_time.tv_sec + PMTEST_DURATION_IN_SECS)) {
		sleep(PMTEST_THREAD_SLEEP_TIME);
		pm_changestate(PMTEST_DOMAIN, s);
		pm_dumpstates();
#ifdef CONFIG_PM_METRICS
		pm_get_domainmetrics(0, &m);
		pmvdbg("Normal:%d Idle:%d standby:%d sleep:%d \n", m.normal, m.idle, m.standby, m.sleep);
#endif
		clock_gettime(CLOCK_REALTIME, &elapsed_time);
	}
	return 0;
}
#endif

/* PM IDLE test routine */
static int pmidletest_kthread(int argc, char *argv[])
{
	int ss_count;
	int suggested_state;
	int idle_test_sleep_time = 0;

	struct timespec start_time;
	struct timespec elapsed_time;
	struct pm_idle_state_info *pdat = get_pm_idle_data();

	ss_count = get_pm_sleep_state_count();

	if((ss_count != sleep_states_count) && (ss_count == 0)) {
		pmvdbg("Error: invalid number of sleep states\n");
		return 0;
	}
	dbg("sleep state count = %d\n", ss_count);

	/* Test for last sleep state */
	suggested_state = ss_count;
	dbg("Suggested sleep state = %d\n", suggested_state);

	/* Calculate total sleep time idle_test_sleep_time for last sleep state */
	for(i = 1; i <= suggested_state; i++) {
		idle_test_sleep_time += (pdat+i)->sleep_threshold;
	}
	dbg("Calculated sleep time for the last sleep state = %d\n", idle_test_sleep_time);

	pmvdbg("This test runs for 7200 secs\n");
	clock_gettime(CLOCK_REALTIME, &start_time);
	elapsed_time.tv_sec = start_time.tv_sec;

	while (elapsed_time.tv_sec <= (start_time.tv_sec + PMTEST_DURATION_IN_SECS)) {
		sleep(idle_test_sleep_time);
		pm_changestate(PMTEST_DOMAIN, suggested_state);
		pm_dumpstates();
		clock_gettime(CLOCK_REALTIME, &elapsed_time);
	}

	return 0;
}

/* Launch pm test thread */

void pmtest_launch_kthread(void)
{
#ifndef CONFIG_IDLE_PM
	if (kernel_thread("pmtest", PMTEST_THREAD_PRIORITY, PMTEST_THREAD_STACKSIZE, pmtest_kthread, (char *const *)NULL) < 0) {
		pmvdbg("pmtest kthread launch failed\n");
	}
#endif
	/* Check PM Idle functionality */
	if (kernel_thread("pmidletest", PMTEST_THREAD_PRIORITY, PMTEST_THREAD_STACKSIZE, pmidletest_kthread, (char *const *)NULL) < 0) {
		pmvdbg("pmidletest kthread launch failed\n");
	}

}

/* init function  */

void pmtest_init(void)
{
#ifndef CONFIG_IDLE_PM
	int i = 0;
	char pmtest_dev_names[PMTEST_DEVICES][CONFIG_PM_DEVNAME_LEN] = { "dev0", "dev1", "dev2" };

	for (i = 0; i < PMTEST_DEVICES; i++) {
		pmtest_cbarray[i].notify = &pmtest_dev_notify;
		pmtest_cbarray[i].prepare = &pmtest_dev_prepare;
		strncpy(pmtest_cbarray[i].name, pmtest_dev_names[i], CONFIG_PM_DEVNAME_LEN - 1);
	}
	for (i = 0; i < PMTEST_DEVICES; i++) {
		pm_register(0, &pmtest_cbarray[i]);
	}
#endif
	/* Launch test thead */
	pmtest_launch_kthread();
}
