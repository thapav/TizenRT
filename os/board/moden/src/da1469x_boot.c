/*****************************************************************************
 *
 * Copyright 2019 Samsung Electronics All Rights Reserved.
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
 * board/da1469x/src/da1469x_boot.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>

//#include <stdio.h>

#include <tinyara/board.h>
#include <arch/board/board.h>
//#include <chip.h>

//#include "sdk_defs.h"
#include "sys_clock_mgr.h"
#include "sys_clock_mgr_internal.h"
#include "hw_clk_da1469x.h"
#include "hw_clk.h"
#include "sec_mpu.h"
#include "sys_watchdog.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

extern void retarget_init(void);
/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
/****************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/
void board_initialize(void)
{

#if defined CONFIG_RETARGET
	extern void retarget_init(void);
#endif
	/* Prepare clocks. Note: cm_cpu_clk_set() and cm_sys_clk_set() can be called only from a
	 * task since they will suspend the task until the XTAL16M has settled and, maybe, the PLL
	 * is locked.
	 */
	cm_sys_clk_init(sysclk_XTAL32M);
	cm_apb_set_clock_divider(apb_div1);
	cm_ahb_set_clock_divider(ahb_div1);
	cm_lp_clk_init();

//	cm_sys_clk_set(sysclk_PLL96);

	/* Initialize bss on ext ram */
//	sec_mpu_init_bss_ext_ram();

	/*
	 * Initialize platform watchdog
	 */
//	sys_watchdog_init();
#if defined CONFIG_RETARGET
	retarget_init();
#endif

}
#endif							/* CONFIG_BOARD_INITIALIZE */
