/**************************************************************************//**
 * @file     system_D2522.h
 * @brief    CMSIS Device System Header File for
 *           DA1469x Device Series
 * @version  V5.00
 * @date     21. October 2016
 ******************************************************************************/
/*
 * Copyright (c) 2009-2016 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/* (c) 2017 Modified by Dialog Semiconductor */


#ifndef SYSTEM_D2522_H
#define SYSTEM_D2522_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock) */


/**
  \brief Setup the microcontroller system.

   Initialize the System and update the SystemCoreClock variable.
 */
extern void SystemInit (void);


/**
  \brief  Update SystemCoreClock variable.
   Updates the SystemCoreClock with current core Clock retrieved from cpu registers.
 */
extern void SystemCoreClockUpdate (void);

/**
 * \brief Convert a CPU address to a physical address
 *
 * To calculate the physical address, the current remapping (SYS_CTRL_REG.REMAP_ADR0)
 * is used.
 *
 * \param [in] addr address seen by CPU
 *
 * \return physical address (for DMA, AES/HASH etc.) -- can be same or different as addr
 *
 */
extern uint32_t black_orca_phy_addr(uint32_t addr);


#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_D2522_H */
