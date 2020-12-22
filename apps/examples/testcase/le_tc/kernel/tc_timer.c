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

/// @file timer.c
/// @brief Test Case Example for Timer API
#include <tinyara/config.h>
#include <stdio.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <tinyara/kernel_test_drv.h>
#include "../../os/kernel/timer/timer.h"
#include "tc_internal.h"

#define USECINT 10000000

static int sig_no = SIGRTMIN;

#ifndef CONFIG_BUILD_PROTECTED
/**
* @fn                   :tc_timer_timer_create_delete
* @brief                :Create and delete a POSIX per-process timer
* @scenario             :Create and delete a POSIX per-process timer
* API's covered         :timer_create, timer_delete
* Preconditions         :none
* Postconditions        :none
* @return               :void
*/
static void tc_timer_timer_create_delete(void)
{
	int ret_chk = 0;
	timer_t timer_id;
	timer_t gtimer_id;
	clockid_t clockid;
#ifdef CONFIG_CLOCK_MONOTONIC
	clockid = CLOCK_MONOTONIC;
#endif
	struct sigevent st_sigevent;
	struct posix_timer_s *st_ret_prt;
	struct itimerspec st_timer_spec_val;

	/* Set and enable alarm */

	st_sigevent.sigev_notify = SIGEV_SIGNAL;
	st_sigevent.sigev_signo = sig_no;
	st_sigevent.sigev_value.sival_ptr = &timer_id;

	/* Check for clock ID != CLOCK_REALTIME */

#ifdef CONFIG_CLOCK_MONOTONIC
	ret_chk = timer_create(clockid, &st_sigevent, &timer_id);
	TC_ASSERT_EQ("timer_create", ret_chk, ERROR);
	TC_ASSERT_EQ("timer_create", errno, EINVAL);
#endif

	clockid = CLOCK_REALTIME;

	/* Check for NULL timer ID */

	ret_chk = timer_create(clockid, &st_sigevent, NULL);
	TC_ASSERT_EQ("timer_create", ret_chk, ERROR);
	TC_ASSERT_EQ("timer_create", errno, EINVAL);

	/* Check for NULL timer ID */

	ret_chk = timer_delete(NULL);
	TC_ASSERT_EQ("timer_delete", ret_chk, ERROR);
	TC_ASSERT_EQ("timer_delete", errno, EINVAL);

	ret_chk = timer_create(clockid, &st_sigevent, &timer_id);
	TC_ASSERT_NEQ("timer_create", ret_chk, ERROR);
	TC_ASSERT_NEQ("timer_create", timer_id, NULL);

	st_ret_prt = (struct posix_timer_s *)timer_id;
	TC_ASSERT_EQ_CLEANUP("timer_create", st_ret_prt->pt_value.sival_ptr, st_sigevent.sigev_value.sival_ptr, timer_delete(timer_id));
	TC_ASSERT_EQ_CLEANUP("timer_create", st_ret_prt->pt_signo, st_sigevent.sigev_signo, timer_delete(timer_id));
	TC_ASSERT_EQ_CLEANUP("timer_create", st_ret_prt->pt_crefs, 1, timer_delete(timer_id));
	TC_ASSERT_EQ_CLEANUP("timer_create", st_ret_prt->pt_owner, getpid(), timer_delete(timer_id));
	TC_ASSERT_EQ_CLEANUP("timer_create", st_ret_prt->pt_delay, 0, timer_delete(timer_id));

	st_timer_spec_val.it_value.tv_sec = 1;
	st_timer_spec_val.it_value.tv_nsec = 0;

	st_timer_spec_val.it_interval.tv_sec = 1;
	st_timer_spec_val.it_interval.tv_nsec = 0;

	ret_chk = timer_settime(timer_id, 0, &st_timer_spec_val, NULL);
	TC_ASSERT_EQ_ERROR_CLEANUP("timer_settime", ret_chk, OK, errno, timer_delete(timer_id));

	ret_chk = timer_delete(timer_id);
	TC_ASSERT_NEQ("timer_delete", ret_chk, ERROR);

	ret_chk = timer_create(CLOCK_REALTIME, NULL, &gtimer_id);
	TC_ASSERT_NEQ("timer_create", ret_chk, ERROR);

	st_ret_prt = (struct posix_timer_s *)gtimer_id;

	TC_ASSERT_EQ_CLEANUP("timer_create", st_ret_prt->pt_value.sival_ptr, st_ret_prt, timer_delete(gtimer_id));
	TC_ASSERT_EQ_CLEANUP("timer_create", st_ret_prt->pt_crefs, 1, timer_delete(gtimer_id));
	TC_ASSERT_EQ_CLEANUP("timer_create", st_ret_prt->pt_owner, getpid(), timer_delete(gtimer_id));
	TC_ASSERT_EQ_CLEANUP("timer_create", st_ret_prt->pt_delay, 0, timer_delete(gtimer_id));

	ret_chk = timer_delete(gtimer_id);
	TC_ASSERT_NEQ("timer_delete", ret_chk, ERROR);

	TC_SUCCESS_RESULT();
}
#endif	/* CONFIG_BUILD_PROTECTED */

#ifndef CONFIG_DISABLE_POSIX_TIMERS
/**
* @fn                   :tc_timer_timer_getoverrun
* @brief                :queue a signal to the process for a given timer at any point in time
* @scenario             :If the timer_getoverrun() function succeeds, it will return the timer expiration overrun count
* @scenario              EINVAL if The timerid argument does not correspond to an ID returned by timer_create() but not yet deleted by timer_delete().
* API's covered         :timer_create, timer_getoverrun
* Preconditions         :Creation of timer_id(timer_create)
* Postconditions        :none
* @return               :void
*/
static void tc_timer_timer_getoverrun(void)
{
	int ret_chk = 0;
	timer_t timer_id;
	clockid_t clockid = CLOCK_REALTIME;
	struct sigevent st_sigevent;

	/* Set and enable alarm */

	st_sigevent.sigev_notify = SIGEV_SIGNAL;
	st_sigevent.sigev_signo = sig_no;
	st_sigevent.sigev_value.sival_ptr = &timer_id;

	/* Check for NULL clock ID != CLOCK_REALTIME */

	ret_chk = timer_create(clockid, &st_sigevent, &timer_id);
	TC_ASSERT_NEQ("timer_create", ret_chk, ERROR);
	TC_ASSERT_NEQ("timer_create", timer_id, NULL);

	ret_chk = timer_getoverrun(timer_id);
	TC_ASSERT_EQ_CLEANUP("timer_getoverrun", ret_chk, ERROR, timer_delete(timer_id));
	TC_ASSERT_EQ_CLEANUP("timer_getoverrun", errno, ENOSYS, timer_delete(timer_id));

	ret_chk = timer_delete(timer_id);
	TC_ASSERT_NEQ("timer_delete", ret_chk, ERROR);

	TC_SUCCESS_RESULT();
}
#endif

/**
* @fn                   :tc_timer_timer_set_get_time
* @brief                :arm/disarm and fetch state of POSIX per-process timer
* @scenario             :arm/disarm and fetch state of POSIX per-process timer
* API's covered         :timer_create, timer_settime, timer_gettime
* Preconditions         :Creation of timer_id(timer_create)
* Postconditions        :none
* @return               :void
*/
static void tc_timer_timer_set_get_time(void)
{
	int ret_chk = ERROR;
	clockid_t clockid = CLOCK_REALTIME;
	struct sigevent st_sigevent;
	struct itimerspec st_timer_spec_set;
	struct itimerspec st_timer_spec_get;
	timer_t timer_id;

	/* Set and enable alarm */

	st_sigevent.sigev_notify = SIGEV_SIGNAL;
	st_sigevent.sigev_signo = sig_no;
	st_sigevent.sigev_value.sival_ptr = &timer_id;

	ret_chk = timer_create(clockid, &st_sigevent, &timer_id);
	TC_ASSERT_NEQ("timer_create", ret_chk, ERROR);
	TC_ASSERT_NEQ("timer_create", timer_id, NULL);

	st_timer_spec_set.it_interval.tv_sec = 1;
	st_timer_spec_set.it_interval.tv_nsec = 0;	/* interval; */
	st_timer_spec_set.it_value.tv_sec = 1;
	st_timer_spec_set.it_value.tv_nsec = 0;	/* expire; */

	/* Null timer ID check for timer_settime */

	ret_chk = timer_settime(NULL, 0, &st_timer_spec_set, NULL);
	TC_ASSERT_EQ_CLEANUP("timer_settime", ret_chk, ERROR, timer_delete(timer_id));
	TC_ASSERT_EQ_CLEANUP("timer_settime", errno, EINVAL, timer_delete(timer_id));

	/* Null timer value check for timer_settime */

	ret_chk = timer_settime(timer_id, 0, NULL, NULL);
	TC_ASSERT_EQ_CLEANUP("timer_settime", ret_chk, ERROR, timer_delete(timer_id));
	TC_ASSERT_EQ_CLEANUP("timer_settime", errno, EINVAL, timer_delete(timer_id));

	/* Null it_value parameter check for timer_settime */

	st_timer_spec_set.it_value.tv_sec = 0;
	ret_chk = timer_settime(timer_id, 0, &st_timer_spec_set, NULL);
	TC_ASSERT_EQ_CLEANUP("timer_settime", ret_chk, OK, timer_delete(timer_id));

	/* Null it_value parameter check for timer_settime */

	st_timer_spec_set.it_value.tv_sec = -1;
	ret_chk = timer_settime(timer_id, 0, &st_timer_spec_set, NULL);
	TC_ASSERT_EQ_CLEANUP("timer_settime", ret_chk, OK, timer_delete(timer_id));
	st_timer_spec_set.it_value.tv_sec = 1;

	/* Check if TIMER_ABSTIME (Flag = 1) is selected */

	ret_chk = timer_settime(timer_id, 1, &st_timer_spec_set, NULL);
	TC_ASSERT_EQ_CLEANUP("timer_settime", ret_chk, OK, timer_delete(timer_id));

	ret_chk = timer_settime(timer_id, 0, &st_timer_spec_set, NULL);	/* Flag =1 :TIMER_ABSTIME */
	TC_ASSERT_EQ_ERROR_CLEANUP("timer_settime", ret_chk, OK, errno, timer_delete(timer_id));

	usleep(USECINT);

	/* Null timer ID check for timer_gettime */

	ret_chk = timer_gettime(NULL, &st_timer_spec_get);
	TC_ASSERT_EQ("timer_gettime", ret_chk, ERROR);
	TC_ASSERT_EQ("timer_gettime", errno, EINVAL);

	ret_chk = timer_gettime(timer_id, &st_timer_spec_get);
	TC_ASSERT_EQ_ERROR_CLEANUP("timer_gettime", ret_chk, OK, errno, timer_delete(timer_id));
	TC_ASSERT_GEQ_CLEANUP("timer_gettime", st_timer_spec_get.it_interval.tv_nsec, st_timer_spec_set.it_interval.tv_nsec, timer_delete(timer_id));
	TC_ASSERT_GEQ_CLEANUP("timer_gettime", st_timer_spec_get.it_interval.tv_sec, st_timer_spec_set.it_interval.tv_sec, timer_delete(timer_id));
	TC_ASSERT_GEQ_CLEANUP("timer_gettime", st_timer_spec_get.it_value.tv_sec, st_timer_spec_set.it_value.tv_sec, timer_delete(timer_id));
	TC_ASSERT_GEQ_CLEANUP("timer_gettime", st_timer_spec_get.it_value.tv_nsec, st_timer_spec_set.it_value.tv_nsec, timer_delete(timer_id));

	timer_delete(timer_id);
	TC_SUCCESS_RESULT();
}

/**
* @fn                   :tc_timer_timer_initialize
* @brief                :Boot up configuration of the POSIX timer facility.
* @brief                :Boot up configuration of the POSIX timer facility.
* API's covered         :timer_initialize
* Preconditions         :Creation of timer_id(timer_create)
* Postconditions        :none
* @return               :void
*/
static void tc_timer_timer_initialize(void)
{
	int fd;
	int ret_chk;
	fd = tc_get_drvfd();

	ret_chk = ioctl(fd, TESTIOC_TIMER_INITIALIZE_TEST, 0);
	TC_ASSERT_EQ("timer_initialize", ret_chk, OK);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: timer
 ****************************************************************************/

int timer_tc_main(void)
{
#ifndef CONFIG_BUILD_PROTECTED
	tc_timer_timer_create_delete();
#endif	

#ifndef CONFIG_DISABLE_POSIX_TIMERS
	tc_timer_timer_getoverrun();
#endif                     /* CONFIG_DISABLE_POSIX_TIMERS */
	tc_timer_timer_set_get_time();
	tc_timer_timer_initialize();

	return 0;
}
