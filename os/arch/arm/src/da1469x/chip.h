/****************************************************************************
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
#ifndef __ARCH_ARM_INCLUDE_DA1469X_CHIP_H
#define __ARCH_ARM_INCLUDE_DA1469X_CHIP_H

/*******************************************************************************
 * Included Files
 ******************************************************************************/

#include <tinyara/config.h>

#define ARMV8M_PERIPHERAL_INTERRUPTS 32

/*******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xe0	/* Bits [5:7] set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80	/* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00	/* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x20	/* Three bits of interrupt priority used */

#define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#define NVIC_SYSH_HIGH_PRIORITY       NVIC_SYSH_PRIORITY_MAX
#define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_MAXNORMAL_PRIORITY
#define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX

/*******************************************************************************
 * Public Types
 ******************************************************************************/

/*******************************************************************************
 * Public Data
 ******************************************************************************/

/*******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_DA1469X_CHIP_H */
