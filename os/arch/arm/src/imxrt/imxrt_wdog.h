/****************************************************************************
 *
 * Copyright 2019 NXP Semiconductors All Rights Reserved.
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
 * os/arch/arm/src/imxrt/imxrt_wdog.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Janne Rosberg <janne@offcode.fi>
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_WDOG_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <tinyara/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "up_internal.h"
#include "chip.h"
#include "chip/imxrt_wdog.h"

#if defined(CONFIG_ARCH_CHIP_FAMILY_IMXRT102x)
#include <chip/imxrt102x_features.h>
#include "chip/imxrt102x_config.h"
#elif defined(CONFIG_ARCH_CHIP_FAMILY_IMXRT105x)
#include <chip/imxrt105x_features.h>
#include "chip/imxrt105x_config.h"
#else
#error Unrecognized i.MX RT architecture
#endif

#include "imxrt_config.h"

/*!
 * @addtogroup wdog
 * @{
 */
/*******************************************************************************
 * Definitions
 *******************************************************************************/
/*! @name Driver version */
/*@{*/
/*! @brief Defines WDOG driver version */
#define FSL_WDOG_DRIVER_VERSION (MAKE_VERSION(2, 1, 0))
/*@}*/
/*! @name Refresh sequence */
/*@{*/
#define WDOG_REFRESH_KEY (0xAAAA5555U)
/*@}*/

enum imxrt_wdog_type_e {
	IMXRT_WDOG1,
	IMXRT_WDOG2,
	IMXRT_WDOG_MAX
};

/* @breif IOCTL commands list */
enum imxrt_wdog_cmd_e {
	IMXRT_WDOG_CMD_PAUSE,
	IMXRT_WDOG_CMD_RESUME,
	IMXRT_WDOG_CMD_MAX
};

/*! @brief Defines WDOG work mode. */
typedef struct _wdog_work_mode {
	bool enableWait;  /*!< continue or suspend WDOG in wait mode  */
	bool enableStop;  /*!< continue or suspend WDOG in stop mode  */
	bool enableDebug; /*!< continue or suspend WDOG in debug mode */
} wdog_work_mode_t;

/*! @brief Describes WDOG configuration structure. */
typedef struct _wdog_config {
	bool enableWdog;             /*!< Enables or disables WDOG */
	wdog_work_mode_t workMode;   /*!< Configures WDOG work mode in debug stop and wait mode */
	bool enableInterrupt;        /*!< Enables or disables WDOG interrupt */
	uint16_t timeoutValue;       /*!< Timeout value */
	uint16_t interruptTimeValue; /*!< Interrupt count timeout value */
	bool softwareResetExtension; /*!< software reset extension */
	bool enablePowerDown;        /*!< power down enable bit */
	bool enableTimeOutAssert;    /*!< Enable WDOG_B timeout assertion. */
} wdog_config_t;

/*!
 * @brief WDOG interrupt configuration structure, default settings all disabled.
 *
 * This structure contains the settings for all of the WDOG interrupt configurations.
 */
enum _wdog_interrupt_enable {
	kWDOG_InterruptEnable = WDOG_WICR_WIE_MASK /*!< WDOG timeout generates an interrupt before reset*/
};

/*!
 * @brief WDOG status flags.
 *
 * This structure contains the WDOG status flags for use in the WDOG functions.
 */
enum _wdog_status_flags {
	kWDOG_RunningFlag = WDOG_WCR_WDE_MASK,         /*!< Running flag, set when WDOG is enabled*/
	kWDOG_PowerOnResetFlag = WDOG_WRSR_POR_MASK,   /*!< Power On flag, set when reset is the result of a powerOnReset*/
	kWDOG_TimeoutResetFlag = WDOG_WRSR_TOUT_MASK,  /*!< Timeout flag, set when reset is the result of a timeout*/
	kWDOG_SoftwareResetFlag = WDOG_WRSR_SFTW_MASK, /*!< Software flag, set when reset is the result of a software*/
	kWDOG_InterruptFlag = WDOG_WICR_WTIS_MASK      /*!< interrupt flag,whether interrupt has occurred or not*/
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name WDOG Initialization and De-initialization.
 * @{
 */

/*!
 * @brief Initializes the WDOG configuration sturcture.
 *
 * This function initializes the WDOG configuration structure to default values. The default
 * values are as follows.
 * @code
 *   wdogConfig->enableWdog = true;
 *   wdogConfig->workMode.enableWait = true;
 *   wdogConfig->workMode.enableStop = false;
 *   wdogConfig->workMode.enableDebug = false;
 *   wdogConfig->enableInterrupt = false;
 *   wdogConfig->enablePowerdown = false;
 *   wdogConfig->resetExtension = flase;
 *   wdogConfig->timeoutValue = 0xFFU;
 *   wdogConfig->interruptTimeValue = 0x04u;
 * @endcode
 *
 * @param config Pointer to the WDOG configuration structure.
 * @see wdog_config_t
 */
void imxrt_wdog_getdefaultconfig(wdog_config_t *config);

/*!
 * @brief Initializes the WDOG.
 *
 * This function initializes the WDOG. When called, the WDOG runs according to the configuration.
 *
 * This is an example.
 * @code
 *   wdog_config_t config;
 *   WDOG_GetDefaultConfig(&config);
 *   config.timeoutValue = 0xffU;
 *   config->interruptTimeValue = 0x04u;
 *   WDOG_Init(wdog_base,&config);
 * @endcode
 *
 * @param base   WDOG peripheral base address
 * @param config The configuration of WDOG
 */
void imxrt_wdog_init(WDOG_Type *base, const wdog_config_t *config);

/*!
 * @brief Shuts down the WDOG.
 *
 * This function shuts down the WDOG.
 * Watchdog Enable bit is a write one once only bit. It is not
 * possible to clear this bit by a software write, once the bit is set.
 * This bit(WDE) can be set/reset only in debug mode(exception).
 */
void imxrt_wdog_deinit(WDOG_Type *base);

/*!
 * @brief Enables the WDOG module.
 *
 * This function writes a value into the WDOG_WCR register to enable the WDOG.
 * This is a write one once only bit. It is not possible to clear this bit by a software write,
 * once the bit is set. only debug mode exception.
 * @param base WDOG peripheral base address
 */
static inline void imxrt_wdog_enable(WDOG_Type *base)
{
	base->WCR |= WDOG_WCR_WDE_MASK;
}

/*!
 * @brief Disables the WDOG module.
 *
 * This function writes a value into the WDOG_WCR register to disable the WDOG.
 * This is a write one once only bit. It is not possible to clear this bit by a software write,once the bit is set.
 * only debug mode exception
 * @param base WDOG peripheral base address
 */
static inline void imxrt_wdog_disable(WDOG_Type *base)
{
	base->WCR &= ~WDOG_WCR_WDE_MASK;
}

/*!
 * @brief Trigger the system software reset.
 *
 * This function will write to the WCR[SRS] bit to trigger a software system reset.
 * This bit will automatically resets to "1" after it has been asserted to "0".
 * Note: Calling this API will reset the system right now, please using it with more attention.
 *
 * @param base WDOG peripheral base address
 */
static inline void imxrt_wdog_triggersystemsoftwarereset(WDOG_Type *base)
{
	base->WCR &= ~WDOG_WCR_SRS_MASK;
}

/*!
 * @brief Trigger an output assertion.
 *
 * This function will write to the WCR[WDA] bit to trigger WDOG_B signal assertion.
 * The WDOG_B signal can be routed to external pin of the chip, the output pin will turn to
 * assertion along with WDOG_B signal.
 * Note: The WDOG_B signal will remain assert until a power on reset occurred, so, please
 * take more attention while calling it.
 *
 * @param base WDOG peripheral base address
 */
static inline void imxrt_wdog_triggersoftwaresignal(WDOG_Type *base)
{
	base->WCR &= ~WDOG_WCR_WDA_MASK;
}

/*!
 * @brief Enables the WDOG interrupt.
 *
 *This bit is a write once only bit. Once the software does a write access to this bit, it will get
 *locked and cannot be reprogrammed until the next system reset assertion
 *
 * @param base WDOG peripheral base address
 * @param mask The interrupts to enable
 * The parameter can be combination of the following source if defined.
 * @arg kWDOG_InterruptEnable
 */
static inline void imxrt_wdog_enableinterrupts(WDOG_Type *base, uint16_t mask)
{
	base->WICR |= mask;
}

/*!
 * @brief Gets the WDOG all reset status flags.
 *
 * This function gets all reset status flags.
 *
 * @code
 * uint16_t status;
 * status = WDOG_GetStatusFlags (wdog_base);
 * @endcode
 * @param base        WDOG peripheral base address
 * @return            State of the status flag: asserted (true) or not-asserted (false).@see _wdog_status_flags
 *                    - true: a related status flag has been set.
 *                    - false: a related status flag is not set.
 */
uint16_t imxrt_wdog_getstatusflags(WDOG_Type *base);

/*!
 * @brief Clears the WDOG flag.
 *
 * This function clears the WDOG status flag.
 *
 * This is an example for clearing the interrupt flag.
 * @code
 *   WDOG_ClearStatusFlags(wdog_base,KWDOG_InterruptFlag);
 * @endcode
 * @param base        WDOG peripheral base address
 * @param mask        The status flags to clear.
 *                    The parameter could be any combination of the following values.
 *                    kWDOG_TimeoutFlag
 */
void imxrt_wdog_clearinterruptstatus(WDOG_Type *base, uint16_t mask);

/*!
 * @brief Sets the WDOG timeout value.
 *
 * This function sets the timeout value.
 * This function writes a value into WCR registers.
 * The time-out value can be written at any point of time but it is loaded to the counter at the time
 * when WDOG is enabled or after the service routine has been performed.
 *
 * @param base WDOG peripheral base address
 * @param timeoutCount WDOG timeout value; count of WDOG clock tick.
 */
static inline void imxrt_wdog_settimeoutvalue(WDOG_Type *base, uint16_t timeoutCount)
{
	base->WCR = (base->WCR & ~WDOG_WCR_WT_MASK) | WDOG_WCR_WT(timeoutCount);
}

/*!
 * @brief Sets the WDOG interrupt count timeout value.
 *
 * This function sets the interrupt count timeout value.
 * This function writes a value into WIC registers which are wirte-once.
 * This field is write once only. Once the software does a write access to this field, it will get locked
 * and cannot be reprogrammed until the next system reset assertion.
 * @param base WDOG peripheral base address
 * @param timeoutCount WDOG timeout value; count of WDOG clock tick.
 */
static inline void imxrt_wdog_setinterrupttimeoutvalue(WDOG_Type *base, uint16_t timeoutCount)
{
	base->WICR = (base->WICR & ~WDOG_WICR_WICT_MASK) | WDOG_WICR_WICT(timeoutCount);
}

/*!
 * @brief Disable the WDOG power down enable bit.
 *
 * This function disable the WDOG power down enable(PDE).
 * This function writes a value into WMCR registers which are wirte-once.
 * This field is write once only. Once software sets this bit it cannot be reset until the next system reset.
 * @param base WDOG peripheral base address
 */
static inline void imxrt_wdog_disablepowerdownenable(WDOG_Type *base)
{
	base->WMCR &= ~WDOG_WMCR_PDE_MASK;
}

/*!
 * @brief Refreshes the WDOG timer.
 *
 * This function feeds the WDOG.
 * This function should be called before the WDOG timer is in timeout. Otherwise, a reset is asserted.
 *
 * @param base WDOG peripheral base address
 */
void imxrt_wdog_refresh(WDOG_Type *base);
/*@}*/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_wdog_disable
 *
 * Description:
 *   Called at the very beginning of _start.  Disables all watchdogs
 *
 ****************************************************************************/

void imxrt_wdog_disable_all(void);

/****************************************************************************
 * Name: imxrt_wdog_hwreset
 *
 * Description:
 *   Called to do hw reset in source code
 *
 ****************************************************************************/

void imxrt_wdog_hwreset(void);

/****************************************************************************
 * Name: imxrt_wdog_initialize
 *
 * Description:
 *   Initialize and register the imxrt watchdog. Watchdog timer is registered
 *   as 'devpath'.
 *
 ****************************************************************************/
int imxrt_wdog_initialize(const char *devpath, int wdog_base);

#if defined(__cplusplus)
}
#endif

#endif							/* __ARCH_ARM_SRC_IMXRT_IMXRT_WDOG_H */
