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

/// @file pm_test.c
/// @brief Test Case Example for PM API

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include <sys/ioctl.h>

#include <tinyara/pm/pm.h>
#include <tinyara/testcase_drv.h>
/**************************************************************************
* Private Definitions
**************************************************************************/
#define PM_TEST_PATH "/dev/testcase"

/**************************************************************************
* Private Types
**************************************************************************/

/**************************************************************************
* Private Function Prototypes
**************************************************************************/

/**************************************************************************
* Global Variables
**************************************************************************/

/**************************************************************************
* Private Variables
**************************************************************************/

/**************************************************************************
* Private Functions
**************************************************************************/

/**************************************************************************
* Public Functions
**************************************************************************/

/**************************************************************************
* Local Functions
**************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int pm_test_main(int argc, FAR char *argv[])
#endif
{
	int fd;
	int ret;

	fd = open(PM_TEST_PATH, O_RDONLY);
	if (fd < 0) {
		printf("open Failed : %s\n", PM_TEST_PATH);
		return ERROR;
	}

	ret = ioctl(fd, TESTIOC_DRIVER_PM);
	if (ret < 0) {
		printf("PM module test result: FAIL");
		return ERROR;
	}

	close(fd);
	printf("PM module test result: PASS");

	return 0;
}
