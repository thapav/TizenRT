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
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <tinyara/arch.h>
#include <tinyara/board.h>
#include <tinyara/userspace.h>

#include <arch/board/board.h>

#if CONFIG_MM_REGIONS > 1
#include <tinyara/kmalloc.h>
#endif

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
#include <string.h>
#endif

#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   - For the normal "flat" build, this function returns the size of the
 *     single heap.
 *   - For the protected build (CONFIG_BUILD_PROTECTED=y) with both kernel-
 *     and user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function
 *     provides the size of the unprotected, user-space heap.
 *   - For the kernel build (CONFIG_BUILD_KERNEL=y), this function provides
 *     the size of the protected, kernel-space heap.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
 *
 *   The following memory map is assumed for the flat build:
 *
 *     .data region.  Size determined at link time.
 *     .bss  region  Size determined at link time.
 *     IDLE thread stack.  Size determined by CONFIG_IDLETHREAD_STACKSIZE.
 *     Heap.  Extends to the end of SRAM.
 *
 *   The following memory map is assumed for the kernel build:
 *
 *     Kernel .data region.  Size determined at link time.
 *     Kernel .bss  region  Size determined at link time.
 *     Kernel IDLE thread stack.  Size determined by CONFIG_IDLETHREAD_STACKSIZE.
 *     Padding for alignment
 *     User .data region.  Size determined at link time.
 *     User .bss region  Size determined at link time.
 *     Kernel heap.  Size determined by CONFIG_MM_KERNEL_HEAPSIZE.
 *     User heap.  Extends to the end of SRAM.
 *
 ****************************************************************************/

void up_allocate_heap(FAR void **heap_start, size_t *heap_size)
{
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
	/* Get the unaligned size and position of the user-space heap.
	 * This heap begins after the user-space .bss section at an offset
	 * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
	 */

	uintptr_t ubss_start = (uintptr_t) USERSPACE->us_bssstart;
	uintptr_t ubase = (uintptr_t) USERSPACE->us_bssend;
	size_t usize = CONFIG_RAM_END - ubase;

	DEBUGASSERT(ubase < (uintptr_t) CONFIG_RAM_END);

	/* zero initialize the user space bss section */
	memset((void *)ubss_start, 0, (ubase - ubss_start));
	/* Return the user-space heap settings */

	board_led_on(LED_HEAPALLOCATE);
	*heap_start = (FAR void *)ubase;
	*heap_size = usize;
#else

	/* Return the heap settings */

	board_led_on(LED_HEAPALLOCATE);
	*heap_start = (FAR void *)(g_idle_topstack & ~(0x7));
	*heap_size = CONFIG_RAM_END - (uint32_t)(*heap_start);
#endif
}

/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   For the kernel build (CONFIG_BUILD_PROTECTED/KERNEL=y) with both kernel-
 *   and user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function allocates
 *   the kernel-space heap.  A custom version of this function is needed if
 *   memory protection of the kernel heap is required.
 *
 ****************************************************************************/

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
void up_allocate_kheap(FAR void **heap_start, size_t *heap_size)
{
	/* Return the kernel heap settings (i.e., the part of the heap region
	 * that was not dedicated to the user heap).
	 */

	*heap_start = (FAR void *)(g_idle_topstack & ~(0x7));
	*heap_size = (uint32_t)((uintptr_t)__usram_segment_start__) - (uint32_t)(*heap_start);
}
#endif

/****************************************************************************
 * Name: up_addregion
 ****************************************************************************/
#if CONFIG_MM_REGIONS > 1
void up_addregion(void)
{
	/* TODO : Implement as per requirement */
	/* kumm_addregion(...); */
}
#endif
