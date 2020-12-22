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
/*
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <tinyara/clock.h>
#include <tinyara/timer.h>
#include <tinyara/irq.h>

#include "imxrt_clock.h"
#include "imxrt_qtmr.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Gets the instance from the base address to be used to gate or ungate the module clock
 *
 * @param base Quad Timer peripheral base address
 *
 * @return The Quad Timer instance
 */
static uint32_t imxrt_qtmr_getinstance(TMR_Type *base);
static int imxrt_qtmr_handler(int irq, void *context, void *arg);
static void imxrt_qtmr_setisr(struct imxrt_qtmr_chipinfo_s *qtmr, xcpt_t handler, void *arg);
static void imxrt_qtmr_setcallback(struct timer_lowerhalf_s *lower, tccb_t callback, void *arg);
static int imxrt_qtmr_settimeout(struct timer_lowerhalf_s *lower, uint32_t timeout);
static int imxrt_qtmr_getstatus(struct timer_lowerhalf_s *lower, struct timer_status_s *status);
static int imxrt_qtmr_ioctl(struct timer_lowerhalf_s *lower, int cmd, unsigned long arg);
static int imxrt_qtmr_stop(struct timer_lowerhalf_s *lower);
static int imxrt_qtmr_start(struct timer_lowerhalf_s *lower);
int imxrt_qtmr_initialize(const char *devpath, int timer);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to Quad Timer bases for each instance. */
static TMR_Type *const s_qtmrBases[] = TMR_BASE_PTRS;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to Quad Timer clocks for each instance. */
static const clock_ip_name_t s_qtmrClocks[] = TMR_CLOCKS;
#endif							/* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

static struct imxrt_qtmr_chipinfo_s imxrt_qtmr0_chipinfo = {
	.base = TMR1,
	.irq_id = IMXRT_IRQ_QTIMER1,
};

static struct imxrt_qtmr_chipinfo_s imxrt_qtmr1_chipinfo = {
	.base = TMR2,
	.irq_id = IMXRT_IRQ_QTIMER2,

};

#ifdef CONFIG_ARCH_CHIP_FAMILY_IMXRT105x
static struct imxrt_qtmr_chipinfo_s imxrt_qtmr2_chipinfo = {
	.base = TMR3,
	.irq_id = IMXRT_IRQ_QTIMER3,
};

static struct imxrt_qtmr_chipinfo_s imxrt_qtmr3_chipinfo = {
	.base = TMR4,
	.irq_id = IMXRT_IRQ_QTIMER4,
};
#endif

struct imxrt_qtmr_lowerhalf_s {
	const struct timer_ops_s *ops;		/* Lowerhalf operations */
	struct imxrt_qtmr_chipinfo_s *qtmr;
	qtmr_config_t config;
	bool started;				/* True: Timer has been started */
	tccb_t callback;
	void *arg;				/* Argument passed to upper half callback */
	qtmr_channel_selection_t channel;
	qtmr_selection_t tmr;
};

static const struct timer_ops_s qtmr_timer_ops = {
	.start = imxrt_qtmr_start,
	.stop = imxrt_qtmr_stop,
	.getstatus = imxrt_qtmr_getstatus,
	.settimeout = imxrt_qtmr_settimeout,
	.setcallback = imxrt_qtmr_setcallback,
	.ioctl = imxrt_qtmr_ioctl,
};

static struct imxrt_qtmr_lowerhalf_s g_qtmr0_0_lowerhalf = {
	.ops = &qtmr_timer_ops,
	.channel = kQTMR_Channel_0,
	.tmr = kQTMR_0,
};

static struct imxrt_qtmr_lowerhalf_s g_qtmr0_1_lowerhalf = {
	.ops = &qtmr_timer_ops,
	.channel = kQTMR_Channel_1,
	.tmr = kQTMR_0,
};

static struct imxrt_qtmr_lowerhalf_s g_qtmr0_2_lowerhalf = {
	.ops = &qtmr_timer_ops,
	.channel = kQTMR_Channel_2,
	.tmr = kQTMR_0,
};

static struct imxrt_qtmr_lowerhalf_s g_qtmr0_3_lowerhalf = {
	.ops = &qtmr_timer_ops,
	.channel = kQTMR_Channel_3,
	.tmr = kQTMR_0,
};

static struct imxrt_qtmr_lowerhalf_s g_qtmr1_0_lowerhalf = {
	.ops = &qtmr_timer_ops,
	.channel = kQTMR_Channel_0,
	.tmr = kQTMR_1,
};

static struct imxrt_qtmr_lowerhalf_s g_qtmr1_1_lowerhalf = {
	.ops = &qtmr_timer_ops,
	.channel = kQTMR_Channel_1,
	.tmr = kQTMR_1,
};

static struct imxrt_qtmr_lowerhalf_s g_qtmr1_2_lowerhalf = {
	.ops = &qtmr_timer_ops,
	.channel = kQTMR_Channel_2,
	.tmr = kQTMR_1,
};

static struct imxrt_qtmr_lowerhalf_s g_qtmr1_3_lowerhalf = {
	.ops = &qtmr_timer_ops,
	.channel = kQTMR_Channel_3,
	.tmr = kQTMR_1,
};

#ifdef CONFIG_ARCH_CHIP_FAMILY_IMXRT105x
static struct imxrt_qtmr_lowerhalf_s g_qtmr2_0_lowerhalf = {
	.ops = &qtmr_timer_ops,
	.channel = kQTMR_Channel_0,
	.tmr = kQTMR_2,
};

static struct imxrt_qtmr_lowerhalf_s g_qtmr2_1_lowerhalf = {
	.ops = &qtmr_timer_ops,
	.channel = kQTMR_Channel_1,
	.tmr = kQTMR_2,
};

static struct imxrt_qtmr_lowerhalf_s g_qtmr2_2_lowerhalf = {
	.ops = &qtmr_timer_ops,
	.channel = kQTMR_Channel_2,
	.tmr = kQTMR_2,
};

static struct imxrt_qtmr_lowerhalf_s g_qtmr2_3_lowerhalf = {
	.ops = &qtmr_timer_ops,
	.channel = kQTMR_Channel_3,
	.tmr = kQTMR_2,
};

static struct imxrt_qtmr_lowerhalf_s g_qtmr3_0_lowerhalf = {
	.ops = &qtmr_timer_ops,
	.channel = kQTMR_Channel_0,
	.tmr = kQTMR_3,
};

static struct imxrt_qtmr_lowerhalf_s g_qtmr3_1_lowerhalf = {
	.ops = &qtmr_timer_ops,
	.channel = kQTMR_Channel_1,
	.tmr = kQTMR_3,
};

static struct imxrt_qtmr_lowerhalf_s g_qtmr3_2_lowerhalf = {
	.ops = &qtmr_timer_ops,
	.channel = kQTMR_Channel_2,
	.tmr = kQTMR_3,
};

static struct imxrt_qtmr_lowerhalf_s g_qtmr3_3_lowerhalf = {
	.ops = &qtmr_timer_ops,
	.channel = kQTMR_Channel_3,
	.tmr = kQTMR_3,
};
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t imxrt_qtmr_getinstance(TMR_Type *base)
{
	uint32_t instance;

	/* Find the instance index from base address mappings. */
	for (instance = 0; instance < ARRAY_SIZE(s_qtmrBases); instance++) {
		if (s_qtmrBases[instance] == base) {
			break;
		}
	}

	assert(instance < ARRAY_SIZE(s_qtmrBases));

	return instance;
}

/*!
 * brief Ungates the Quad Timer clock and configures the peripheral for basic operation.
 *
 * note This API should be called at the beginning of the application using the Quad Timer driver.
 *
 * param base     Quad Timer peripheral base address
 * param channel  Quad Timer channel number
 * param config   Pointer to user's Quad Timer config structure
 */
void imxrt_qtmr_init(TMR_Type *base, qtmr_channel_selection_t channel, const qtmr_config_t *config)
{
	assert(config);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
	/* Enable the module clock */
	imxrt_clock_enableclock(s_qtmrClocks[imxrt_qtmr_getinstance(base)]);
#endif							/* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

	/* Setup the counter sources */
	base->CHANNEL[channel].CTRL = (TMR_CTRL_PCS(config->primarySource) | TMR_CTRL_SCS(config->secondarySource));

	/* Setup the master mode operation */
	base->CHANNEL[channel].SCTRL = (TMR_SCTRL_EEOF(config->enableExternalForce) | TMR_SCTRL_MSTR(config->enableMasterMode));

	/* Setup debug mode */
	base->CHANNEL[channel].CSCTRL = TMR_CSCTRL_DBG_EN(config->debugMode);

	base->CHANNEL[channel].FILT &= ~(TMR_FILT_FILT_CNT_MASK | TMR_FILT_FILT_PER_MASK);
	/* Setup input filter */
	base->CHANNEL[channel].FILT = (TMR_FILT_FILT_CNT(config->faultFilterCount) | TMR_FILT_FILT_PER(config->faultFilterPeriod));
}

/*!
 * brief Stops the counter and gates the Quad Timer clock
 *
 * param base     Quad Timer peripheral base address
 * param channel  Quad Timer channel number
 */
void imxrt_qtmr_deinit(TMR_Type *base, qtmr_channel_selection_t channel)
{
	/* Stop the counter */
	base->CHANNEL[channel].CTRL &= ~TMR_CTRL_CM_MASK;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
	/* Disable the module clock */
	imxrt_clock_disableclock(s_qtmrClocks[imxrt_qtmr_getinstance(base)]);
#endif							/* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

/*!
 * brief  Fill in the Quad Timer config struct with the default settings
 *
 * The default values are:
 * code
 *    config->debugMode = kQTMR_RunNormalInDebug;
 *    config->enableExternalForce = false;
 *    config->enableMasterMode = false;
 *    config->faultFilterCount = 0;
 *    config->faultFilterPeriod = 0;
 *    config->primarySource = kQTMR_ClockDivide_2;
 *    config->secondarySource = kQTMR_Counter0InputPin;
 * endcode
 * param config Pointer to user's Quad Timer config structure.
 */
void imxrt_qtmr_getdefaultconfig(qtmr_config_t *config)
{
	assert(config);

	/* Initializes the configure structure to zero. */
	memset(config, 0, sizeof(*config));

	/* Halt counter during debug mode */
	config->debugMode = kQTMR_RunNormalInDebug;

	/* Another counter cannot force state of OFLAG signal */
	config->enableExternalForce = false;

	/* Compare function's output from this counter is not broadcast to other counters */
	config->enableMasterMode = false;

	/* Fault filter count is set to 0 */
	config->faultFilterCount = 0;

	/* Fault filter period is set to 0 which disables the fault filter */
	config->faultFilterPeriod = 0;

	/* Primary count source is IP bus clock divide by 2 */
	config->primarySource = kQTMR_ClockDivide_2;

	/* Secondary count source is counter 0 input pin */
	config->secondarySource = kQTMR_Counter0InputPin;
}

/*!
 * brief Sets up Quad timer module for PWM signal output.
 *
 * The function initializes the timer module according to the parameters passed in by the user. The
 * function also sets up the value compare registers to match the PWM signal requirements.
 *
 * param base             Quad Timer peripheral base address
 * param channel          Quad Timer channel number
 * param pwmFreqHz        PWM signal frequency in Hz
 * param dutyCyclePercent PWM pulse width, value should be between 0 to 100
 *                         0=inactive signal(0% duty cycle)...
 *                         100=active signal (100% duty cycle)
 * param outputPolarity   true: invert polarity of the output signal, false: no inversion
 * param srcClock_Hz      Main counter clock in Hz.
 *
 * return Returns an error if there was error setting up the signal.
 */
status_t imxrt_qtmr_setuppwm(TMR_Type *base, qtmr_channel_selection_t channel, uint32_t pwmFreqHz, uint8_t dutyCyclePercent, bool outputPolarity, uint32_t srcClock_Hz)
{
	uint32_t periodCount, highCount, lowCount, reg;

	if (dutyCyclePercent > 100) {
		/* Invalid dutycycle */
		return kStatus_Fail;
	}

	/* Set OFLAG pin for output mode and force out a low on the pin */
	base->CHANNEL[channel].SCTRL |= (TMR_SCTRL_FORCE_MASK | TMR_SCTRL_OEN_MASK);

	/* Counter values to generate a PWM signal */
	periodCount = (srcClock_Hz / pwmFreqHz);
	highCount = (periodCount * dutyCyclePercent) / 100;
	lowCount = periodCount - highCount;

	/* Setup the compare registers for PWM output */
	base->CHANNEL[channel].COMP1 = lowCount;
	base->CHANNEL[channel].COMP2 = highCount;

	/* Setup the pre-load registers for PWM output */
	base->CHANNEL[channel].CMPLD1 = lowCount;
	base->CHANNEL[channel].CMPLD2 = highCount;

	reg = base->CHANNEL[channel].CSCTRL;
	/* Setup the compare load control for COMP1 and COMP2.
	 * Load COMP1 when CSCTRL[TCF2] is asserted, load COMP2 when CSCTRL[TCF1] is asserted
	 */
	reg &= ~(TMR_CSCTRL_CL1_MASK | TMR_CSCTRL_CL2_MASK);
	reg |= (TMR_CSCTRL_CL1(kQTMR_LoadOnComp2) | TMR_CSCTRL_CL2(kQTMR_LoadOnComp1));
	base->CHANNEL[channel].CSCTRL = reg;

	if (outputPolarity) {
		/* Invert the polarity */
		base->CHANNEL[channel].SCTRL |= TMR_SCTRL_OPS_MASK;
	} else {
		/* True polarity, no inversion */
		base->CHANNEL[channel].SCTRL &= ~TMR_SCTRL_OPS_MASK;
	}

	reg = base->CHANNEL[channel].CTRL;
	reg &= ~(TMR_CTRL_OUTMODE_MASK);

	/* Count until compare value is reached and re-initialize the counter, toggle OFLAG output
	 * using alternating compare register
	 */
	reg |= (TMR_CTRL_LENGTH_MASK | TMR_CTRL_OUTMODE(kQTMR_ToggleOnAltCompareReg));
	base->CHANNEL[channel].CTRL = reg;

	return kStatus_Success;
}

/*!
 * brief Allows the user to count the source clock cycles until a capture event arrives.
 *
 * The count is stored in the capture register.
 *
 * param base            Quad Timer peripheral base address
 * param channel         Quad Timer channel number
 * param capturePin      Pin through which we receive the input signal to trigger the capture
 * param inputPolarity   true: invert polarity of the input signal, false: no inversion
 * param reloadOnCapture true: reload the counter when an input capture occurs, false: no reload
 * param captureMode     Specifies which edge of the input signal triggers a capture
 */
void imxrt_qtmr_setupinputcapture(TMR_Type *base, qtmr_channel_selection_t channel, qtmr_input_source_t capturePin, bool inputPolarity, bool reloadOnCapture, qtmr_input_capture_edge_t captureMode)
{
	uint16_t reg;

	/* Clear the prior value for the input source for capture */
	reg = base->CHANNEL[channel].CTRL & (~TMR_CTRL_SCS_MASK);

	/* Set the new input source */
	reg |= TMR_CTRL_SCS(capturePin);
	base->CHANNEL[channel].CTRL = reg;

	/* Clear the prior values for input polarity, capture mode. Set the external pin as input */
	reg = base->CHANNEL[channel].SCTRL & (~(TMR_SCTRL_IPS_MASK | TMR_SCTRL_CAPTURE_MODE_MASK | TMR_SCTRL_OEN_MASK));

	/* Set the new values */
	reg |= (TMR_SCTRL_IPS(inputPolarity) | TMR_SCTRL_CAPTURE_MODE(captureMode));
	base->CHANNEL[channel].SCTRL = reg;

	/* Setup if counter should reload when a capture occurs */
	if (reloadOnCapture) {
		base->CHANNEL[channel].CSCTRL |= TMR_CSCTRL_ROC_MASK;
	} else {
		base->CHANNEL[channel].CSCTRL &= ~TMR_CSCTRL_ROC_MASK;
	}
}

/*!
 * brief Enables the selected Quad Timer interrupts
 *
 * param base      Quad Timer peripheral base address
 * param channel   Quad Timer channel number
 * param mask      The interrupts to enable. This is a logical OR of members of the
 *                  enumeration ::qtmr_interrupt_enable_t
 */
void imxrt_qtmr_enableinterrupts(TMR_Type *base, qtmr_channel_selection_t channel, uint32_t mask)
{
	uint16_t reg;

	reg = base->CHANNEL[channel].SCTRL;

	/* Compare interrupt */
	if (mask & kQTMR_CompareInterruptEnable) {
		reg |= TMR_SCTRL_TCFIE_MASK;
	}

	/* Overflow interrupt */
	if (mask & kQTMR_OverflowInterruptEnable) {
		reg |= TMR_SCTRL_TOFIE_MASK;
	}
	/* Input edge interrupt */
	if (mask & kQTMR_EdgeInterruptEnable) {
		/* Restriction: Do not set both SCTRL[IEFIE] and DMA[IEFDE] */
		base->CHANNEL[channel].DMA &= ~TMR_DMA_IEFDE_MASK;
		reg |= TMR_SCTRL_IEFIE_MASK;
	}

	base->CHANNEL[channel].SCTRL = reg;

	reg = base->CHANNEL[channel].CSCTRL;

	/* Compare 1 interrupt */
	if (mask & kQTMR_Compare1InterruptEnable) {
		reg |= TMR_CSCTRL_TCF1EN_MASK;
	}

	/* Compare 2 interrupt */
	if (mask & kQTMR_Compare2InterruptEnable) {
		reg |= TMR_CSCTRL_TCF2EN_MASK;
	}

	base->CHANNEL[channel].CSCTRL = reg;
}

/*!
 * brief Disables the selected Quad Timer interrupts
 *
 * param base     Quad Timer peripheral base addres
 * param channel  Quad Timer channel number
 * param mask The interrupts to enable. This is a logical OR of members of the
 *             enumeration ::qtmr_interrupt_enable_t
 */
void imxrt_qtmr_disableinterrupts(TMR_Type *base, qtmr_channel_selection_t channel, uint32_t mask)
{
	uint16_t reg;

	reg = base->CHANNEL[channel].SCTRL;

	/* Compare interrupt */
	if (mask & kQTMR_CompareInterruptEnable) {
		reg &= ~TMR_SCTRL_TCFIE_MASK;
	}

	/* Overflow interrupt */
	if (mask & kQTMR_OverflowInterruptEnable) {
		reg &= ~TMR_SCTRL_TOFIE_MASK;
	}

	/* Input edge interrupt */
	if (mask & kQTMR_EdgeInterruptEnable) {
		reg &= ~TMR_SCTRL_IEFIE_MASK;
	}

	base->CHANNEL[channel].SCTRL = reg;

	reg = base->CHANNEL[channel].CSCTRL;

	/* Compare 1 interrupt */
	if (mask & kQTMR_Compare1InterruptEnable) {
		reg &= ~TMR_CSCTRL_TCF1EN_MASK;
	}

	/* Compare 2 interrupt */
	if (mask & kQTMR_Compare2InterruptEnable) {
		reg &= ~TMR_CSCTRL_TCF2EN_MASK;
	}

	base->CHANNEL[channel].CSCTRL = reg;
}

/*!
 * brief Gets the enabled Quad Timer interrupts
 *
 * param base    Quad Timer peripheral base address
 * param channel Quad Timer channel number
 *
 * return The enabled interrupts. This is the logical OR of members of the
 *         enumeration ::qtmr_interrupt_enable_t
 */
uint32_t imxrt_qtmr_getenabledinterrupts(TMR_Type *base, qtmr_channel_selection_t channel)
{
	uint32_t enabledInterrupts = 0;
	uint16_t reg;

	reg = base->CHANNEL[channel].SCTRL;

	/* Compare interrupt */
	if (reg & TMR_SCTRL_TCFIE_MASK) {
		enabledInterrupts |= kQTMR_CompareFlag;
	}

	/* Overflow interrupt */
	if (reg & TMR_SCTRL_TOFIE_MASK) {
		enabledInterrupts |= kQTMR_OverflowInterruptEnable;
	}

	/* Input edge interrupt */
	if (reg & TMR_SCTRL_IEFIE_MASK) {
		enabledInterrupts |= kQTMR_EdgeInterruptEnable;
	}

	reg = base->CHANNEL[channel].CSCTRL;

	/* Compare 1 interrupt */
	if (reg & TMR_CSCTRL_TCF1EN_MASK) {
		enabledInterrupts |= kQTMR_Compare1InterruptEnable;
	}

	/* Compare 2 interrupt */
	if (reg & TMR_CSCTRL_TCF2EN_MASK) {
		enabledInterrupts |= kQTMR_Compare2InterruptEnable;
	}

	return enabledInterrupts;
}

/*!
 * brief Gets the Quad Timer status flags
 *
 * param base     Quad Timer peripheral base address
 * param channel  Quad Timer channel number
 *
 * return The status flags. This is the logical OR of members of the
 *         enumeration ::qtmr_status_flags_t
 */
uint32_t imxrt_qtmr_getstatusflags(TMR_Type *base, qtmr_channel_selection_t channel)
{
	uint32_t statusFlags = 0;
	uint16_t reg;

	reg = base->CHANNEL[channel].SCTRL;

	/* Timer compare flag */
	if (reg & TMR_SCTRL_TCF_MASK) {
		statusFlags |= kQTMR_CompareFlag;
	}

	/* Timer overflow flag */
	if (reg & TMR_SCTRL_TOF_MASK) {
		statusFlags |= kQTMR_OverflowFlag;
	}

	/* Input edge flag */
	if (reg & TMR_SCTRL_IEF_MASK) {
		statusFlags |= kQTMR_EdgeFlag;
	}

	reg = base->CHANNEL[channel].CSCTRL;

	/* Compare 1 flag */
	if (reg & TMR_CSCTRL_TCF1_MASK) {
		statusFlags |= kQTMR_Compare1Flag;
	}

	/* Compare 2 flag */
	if (reg & TMR_CSCTRL_TCF2_MASK) {
		statusFlags |= kQTMR_Compare2Flag;
	}

	return statusFlags;
}

/*!
 * brief Clears the Quad Timer status flags.
 *
 * param base     Quad Timer peripheral base address
 * param channel  Quad Timer channel number
 * param mask The status flags to clear. This is a logical OR of members of the
 *             enumeration ::qtmr_status_flags_t
 */
void imxrt_qtmr_clearstatusflags(TMR_Type *base, qtmr_channel_selection_t channel, uint32_t mask)
{
	uint16_t reg;

	reg = base->CHANNEL[channel].SCTRL;

	/* Timer compare flag */
	if (mask & kQTMR_CompareFlag) {
		reg &= ~TMR_SCTRL_TCF_MASK;
	}

	/* Timer overflow flag */
	if (mask & kQTMR_OverflowFlag) {
		reg &= ~TMR_SCTRL_TOF_MASK;
	}

	/* Input edge flag */
	if (mask & kQTMR_EdgeFlag) {
		reg &= ~TMR_SCTRL_IEF_MASK;
	}

	base->CHANNEL[channel].SCTRL = reg;

	reg = base->CHANNEL[channel].CSCTRL;
	
	/* Compare 1 flag */
	if (mask & kQTMR_Compare1Flag) {
		reg &= ~TMR_CSCTRL_TCF1_MASK;
	}

	/* Compare 2 flag */
	if (mask & kQTMR_Compare2Flag) {
		reg &= ~TMR_CSCTRL_TCF2_MASK;
	}

	base->CHANNEL[channel].CSCTRL = reg;
}

/*!
 * brief Sets the timer period in ticks.
 *
 * Timers counts from initial value till it equals the count value set here. The counter
 * will then reinitialize to the value specified in the Load register.
 *
 * note
 * 1. This function will write the time period in ticks to COMP1 or COMP2 register
 *    depending on the count direction
 * 2. User can call the utility macros provided in fsl_common.h to convert to ticks
 * 3. This function supports cases, providing only primary source clock without secondary source clock.
 *
 * param base     Quad Timer peripheral base address
 * param channel  Quad Timer channel number
 * param ticks Timer period in units of ticks
 */
void imxrt_qtmr_settimerperiod(TMR_Type *base, qtmr_channel_selection_t channel, uint16_t ticks)
{
	uint32_t reg;
	/* Set the length bit to reinitialize the counters on a match */
	base->CHANNEL[channel].CTRL |= TMR_CTRL_LENGTH_MASK;

	if (base->CHANNEL[channel].CTRL & TMR_CTRL_DIR_MASK) {
		tmrvdbg("TMR Counting down\n");
		/* Counting down */
		base->CHANNEL[channel].COMP2 = ticks;
		/* Setting CMPLD2 register as well */
		base->CHANNEL[channel].CMPLD2 = ticks;
	} else {
		tmrvdbg("TMR Counting up\n");
		/* Counting up */
		base->CHANNEL[channel].COMP1 = ticks;
		/* Setting CMPLD1 register as well */
		base->CHANNEL[channel].CMPLD1 = ticks;
	}
}

uint32_t imxrt_qtmr_gettimerperiod(TMR_Type *base, qtmr_channel_selection_t channel)
{
	if (base->CHANNEL[channel].CTRL & TMR_CTRL_DIR_MASK) {
		/* Counting down */
		return base->CHANNEL[channel].COMP2;
	} else {
		/* Counting up */
		return base->CHANNEL[channel].COMP1;
	}
}

/*!
 * brief Enable the Quad Timer DMA.
 *
 * param base     Quad Timer peripheral base address
 * param channel  Quad Timer channel number
 * param mask     The DMA to enable. This is a logical OR of members of the
 *                  enumeration ::qtmr_dma_enable_t
 */
void imxrt_qtmr_enabledma(TMR_Type *base, qtmr_channel_selection_t channel, uint32_t mask)
{
	uint16_t reg;

	reg = base->CHANNEL[channel].DMA;
	/* Input Edge Flag DMA Enable */
	if (mask & kQTMR_InputEdgeFlagDmaEnable) {
		/* Restriction: Do not set both DMA[IEFDE] and SCTRL[IEFIE] */
		base->CHANNEL[channel].SCTRL &= ~TMR_SCTRL_IEFIE_MASK;
		reg |= TMR_DMA_IEFDE_MASK;
	}
	/* Comparator Preload Register 1 DMA Enable */
	if (mask & kQTMR_ComparatorPreload1DmaEnable) {
		reg |= TMR_DMA_CMPLD1DE_MASK;
	}
	/* Comparator Preload Register 2 DMA Enable */
	if (mask & kQTMR_ComparatorPreload2DmaEnable) {
		reg |= TMR_DMA_CMPLD2DE_MASK;
	}
	base->CHANNEL[channel].DMA = reg;
}

/*!
 * brief Disable the Quad Timer DMA.
 *
 * param base     Quad Timer peripheral base address
 * param channel  Quad Timer channel number
 * param mask     The DMA to enable. This is a logical OR of members of the
 *                  enumeration ::qtmr_dma_enable_t
 */
void imxrt_qtmr_disabledma(TMR_Type *base, qtmr_channel_selection_t channel, uint32_t mask)
{
	uint16_t reg;

	reg = base->CHANNEL[channel].DMA;
	/* Input Edge Flag DMA Enable */
	if (mask & kQTMR_InputEdgeFlagDmaEnable) {
		reg &= ~TMR_DMA_IEFDE_MASK;
	}
	/* Comparator Preload Register 1 DMA Enable */
	if (mask & kQTMR_ComparatorPreload1DmaEnable) {
		reg &= ~TMR_DMA_CMPLD1DE_MASK;
	}
	/* Comparator Preload Register 2 DMA Enable */
	if (mask & kQTMR_ComparatorPreload2DmaEnable) {
		reg &= ~TMR_DMA_CMPLD2DE_MASK;
	}
	base->CHANNEL[channel].DMA = reg;
}

struct imxrt_qtmr_chipinfo_s *imxrt_qtmr_configure(int timer, qtmr_config_t *config, qtmr_channel_selection_t channel)
{
	struct imxrt_qtmr_chipinfo_s *qtmr = NULL;
	switch (timer) {
	case kQTMR_0:
		qtmr = &imxrt_qtmr0_chipinfo;
		break;

	case kQTMR_1:
		qtmr = &imxrt_qtmr1_chipinfo;
		break;
#ifdef CONFIG_ARCH_CHIP_FAMILY_IMXRT105x
	case kQTMR_2:
		qtmr = &imxrt_qtmr2_chipinfo;
		break;

	case kQTMR_3:
		qtmr = &imxrt_qtmr3_chipinfo;
		break;
#endif
	}

	imxrt_qtmr_init(qtmr->base, channel, config);
	return qtmr;
}

static int imxrt_qtmr_handler(int irq, void *context, void *arg)
{
	struct imxrt_qtmr_lowerhalf_s *priv = (struct imxrt_qtmr_lowerhalf_s *)arg;
	struct imxrt_qtmr_chipinfo_s *qtmr = priv->qtmr;
	uint32_t next_interval_us = 0;
	uint32_t freq;
	uint64_t ticks;

	imxrt_qtmr_clearstatusflags(qtmr->base, priv->channel, kQTMR_Compare1Flag);

	if (priv->callback(&next_interval_us, priv->arg)) {
		if (next_interval_us > 0) {
			freq = imxrt_clock_getperclkfreq();
			ticks = ((uint64_t) freq * (uint64_t) next_interval_us) / USEC_PER_SEC;
			imxrt_qtmr_settimerperiod(priv->qtmr->base, priv->channel, ticks);
		}
	} else {
		imxrt_qtmr_stop((struct timer_lowerhalf_s *)priv);
	}

	return OK;
}

static void imxrt_qtmr_setisr(struct imxrt_qtmr_chipinfo_s *qtmr, xcpt_t handler, void *arg)
{
	irq_attach(qtmr->irq_id, handler, arg);
	up_enable_irq(qtmr->irq_id);
}

static void imxrt_qtmr_setcallback(struct timer_lowerhalf_s *lower, tccb_t callback, void *arg)
{
	struct imxrt_qtmr_lowerhalf_s *priv = (struct imxrt_qtmr_lowerhalf_s *)lower;
	irqstate_t flags = irqsave();

	/* Save the new callback */
	priv->callback = callback;
	priv->arg = arg;

	if (priv->callback != NULL) {
		imxrt_qtmr_setisr(priv->qtmr, imxrt_qtmr_handler, priv);
		imxrt_qtmr_enableinterrupts(priv->qtmr->base, priv->channel, kQTMR_Compare1InterruptEnable);
	} else {
		imxrt_qtmr_disableinterrupts(priv->qtmr->base, priv->channel, kQTMR_Compare1InterruptEnable);
		imxrt_qtmr_setisr(priv->qtmr, NULL, NULL);
	}

	irqrestore(flags);
}

static int imxrt_qtmr_settimeout(struct timer_lowerhalf_s *lower, uint32_t timeout)
{
	uint32_t freq;
	uint64_t ticks;
	struct imxrt_qtmr_lowerhalf_s *priv = (struct imxrt_qtmr_lowerhalf_s *)lower;

	if (priv->started) {
		return -EPERM;
	}

	freq = imxrt_clock_getperclkfreq();

	ticks = ((uint64_t) freq * (uint64_t) timeout) / USEC_PER_SEC;
	tmrdbg("freq:%d ticks:%llu, timeout = %lu\n", freq, ticks, timeout);

	if (ticks < USHRT_MAX) {
		imxrt_qtmr_settimerperiod(priv->qtmr->base, priv->channel, ticks);
	} else {
		tmrdbg("Ticks [%d] exceeds limit for this 16-bit QTMR timer counter [limit = %u]\n", ticks, USHRT_MAX);
		return -EINVAL;
	}

	imxrt_qtmr_settimerperiod(priv->qtmr->base, priv->channel, ticks);

	return OK;
}

static int imxrt_qtmr_getstatus(struct timer_lowerhalf_s *lower, struct timer_status_s *status)
{
	struct imxrt_qtmr_lowerhalf_s *priv = (struct imxrt_qtmr_lowerhalf_s *)lower;
	uint32_t ticks, timeout_ticks, cur_ticks;
	uint32_t freq;

	freq = imxrt_clock_getperclkfreq();

	/* Return the status bit */
	status->flags = 0;
	if (priv->started) {
		status->flags |= TCFLAGS_ACTIVE;
	}

	if (priv->callback) {
		status->flags |= TCFLAGS_HANDLER;
	}

	/* Return the actual timeout in microseconds */
	timeout_ticks = imxrt_qtmr_gettimerperiod(priv->qtmr->base, priv->channel);
	status->timeout = ((uint64_t)USEC_PER_SEC * (uint64_t)timeout_ticks) / freq;

	/* Get the time remaining until the timer expires (in microseconds). */
	cur_ticks = imxrt_qtmr_getcurrenttimercount(priv->qtmr->base, priv->channel);
	ticks = timeout_ticks - cur_ticks;
	status->timeleft = ((uint64_t)USEC_PER_SEC * (uint64_t)ticks) / freq;

	return OK;
}

static int imxrt_qtmr_ioctl(struct timer_lowerhalf_s *lower, int cmd, unsigned long arg)
{
	return -ENOTTY;
}

static int imxrt_qtmr_stop(struct timer_lowerhalf_s *lower)
{
	struct imxrt_qtmr_lowerhalf_s *priv = (struct imxrt_qtmr_lowerhalf_s *)lower;
	tmrvdbg("Stop timer [%d] channel [%d]\n", priv->tmr, priv->channel);

	if (priv->started) {
		imxrt_qtmr_disableinterrupts(priv->qtmr->base, priv->channel, kQTMR_Compare1InterruptEnable);

		imxrt_qtmr_stoptimer(priv->qtmr->base, priv->channel);

		imxrt_qtmr_setisr(priv->qtmr, NULL, NULL);
		priv->started = false;

		tmrvdbg("QTMR Timer %d channel %d is stopped\n", priv->tmr, priv->channel);
		return OK;
	}

	/* Return ENODEV to indicate that the timer was not running */
	tmrdbg("Error!! QTMR Timer %d channel %d is not running\n", priv->tmr, priv->channel);
	return -ENODEV;
}

static int imxrt_qtmr_start(struct timer_lowerhalf_s *lower)
{
	struct imxrt_qtmr_lowerhalf_s *priv = (struct imxrt_qtmr_lowerhalf_s *)lower;
	uint32_t enabled_interrupts;
	uint16_t counter;
	tmrvdbg("Start for timer[%d] channel [%d]\n", priv->tmr, priv->channel);

	if (!priv->started) {
		if (priv->callback != NULL) {
			imxrt_qtmr_enableinterrupts(priv->qtmr->base, priv->channel, kQTMR_Compare1InterruptEnable);
		}

		enabled_interrupts = imxrt_qtmr_getenabledinterrupts(priv->qtmr->base, priv->channel);
		enabled_interrupts &= kQTMR_Compare1InterruptEnable;
		if (enabled_interrupts) {
			tmrvdbg("enabled interrupt CMP1\n");
		}

		imxrt_qtmr_setisr(priv->qtmr, imxrt_qtmr_handler, priv);

		imxrt_qtmr_starttimer(priv->qtmr->base, priv->channel, kQTMR_PriSrcRiseEdge);

		priv->started = true;

		tmrvdbg("QTMR Timer %d Channel %d is started, callback %s\n", priv->tmr, priv->channel, priv->callback == NULL ? "none" : "set");

		return OK;
	}

	/* Return EBUSY to indicate that the timer was already running */
	tmrdbg("Error!! QTMR Timer %d Channel %d was already running\n", priv->tmr, priv->channel);
	return -EBUSY;
}

int imxrt_qtmr_initialize(const char *devpath, int timer)
{
	qtmr_config_t config;
	struct imxrt_qtmr_lowerhalf_s *lower = NULL;

	/* At present, only one channel is being utilised */
	int channel = 0;

	switch (timer) {
	case kQTMR_0:
		lower = &g_qtmr0_0_lowerhalf;
		break;

	case kQTMR_1:
		lower = &g_qtmr1_0_lowerhalf;
		break;

#ifdef CONFIG_ARCH_CHIP_FAMILY_IMXRT105x
	case kQTMR_2:
		lower = &g_qtmr2_0_lowerhalf;
		break;

	case kQTMR_3:
		lower = &g_qtmr3_0_lowerhalf;
		break;
#endif
	}

	if (!lower) {
		return -ENODEV;
	}

	/* Initialize the elements of lower half state structure */
	lower->started = false;
	lower->callback = NULL;

	imxrt_qtmr_getdefaultconfig(&config);
	memcpy(&(lower->config), &config, sizeof(qtmr_config_t));
	lower->qtmr = imxrt_qtmr_configure(timer, &config, channel);
	lower->tmr = timer;
	lower->channel = channel;

	if (lower->qtmr == NULL) {
		return -EINVAL;
	}

	/*
	 * Register the timer driver as /dev/timerX.  The returned value from
	 * timer_register is a handle that could be used with timer_unregister().
	 * REVISIT: The returned handle is discard here.
	 */
	if (!timer_register(devpath, (struct timer_lowerhalf_s *)lower)) {
		/*
		 * The actual cause of the failure may have been a failure to allocate
		 * perhaps a failure to register the timer driver (such as if the
		 * 'depath' were not unique).  We know here but we return EEXIST to
		 * indicate the failure (implying the non-unique devpath).
		 */
		return -EEXIST;
	}

	return OK;
}
