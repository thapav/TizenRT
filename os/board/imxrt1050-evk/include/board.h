/* ****************************************************************
*
* Copyright 2019 NXP Semiconductors All Rights Reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************/
/************************************************************************************
 * os/configs/imxrt1050/include/board.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name TinyARA nor the names of its contributors may be
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
 ************************************************************************************/

#ifndef __CONFIGS_IMXRT1050_EVK_INCLUDE_BOARD_H
#define __CONFIGS_IMXRT1050_EVK_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <tinyara/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

/* Set VDD_SOC to 1.5V */

#define IMXRT_VDD_SOC (0x12)

/* Set Arm PLL (PLL1) to  fOut    = (24Mhz * ARM_PLL_DIV_SELECT/2) / ARM_PODF_DIVISOR
 *                        600Mhz  = (24Mhz * ARM_PLL_DIV_SELECT/2) / ARM_PODF_DIVISOR
 *                        ARM_PLL_DIV_SELECT = 100
 *                        ARM_PODF_DIVISOR   = 2
 *                        600Mhz  = (24Mhz * 100/2) / 2
 *
 *     AHB_CLOCK_ROOT             = PLL1fOut / IMXRT_AHB_PODF_DIVIDER
 *     1Hz to 600 Mhz             = 600Mhz / IMXRT_ARM_CLOCK_DIVIDER
 *                        IMXRT_ARM_CLOCK_DIVIDER = 1
 *                        600Mhz  = 600Mhz / 1
 *
 *     PRE_PERIPH_CLK_SEL         = PRE_PERIPH_CLK_SEL_PLL1
 *     PERIPH_CLK_SEL             = 1 (0 select PERIPH_CLK2_PODF, 1 select PRE_PERIPH_CLK_SEL_PLL1)
 *     PERIPH_CLK                 = 600Mhz
 *
 *     IPG_CLOCK_ROOT             = AHB_CLOCK_ROOT / IMXRT_IPG_PODF_DIVIDER
 *                       IMXRT_IPG_PODF_DIVIDER = 4
 *                       150Mhz = 600Mhz / 4
 *
 *     PRECLK_CLOCK_ROOT          = IPG_CLOCK_ROOT / IMXRT_PERCLK_PODF_DIVIDER
 *                       IMXRT_PERCLK_PODF_DIVIDER = 1
 *                       150Mhz = 150Mhz / 1
 *
 *     SEMC_CLK_ROOT              = 600Mhz / IMXRT_SEMC_PODF_DIVIDER (labeled AIX_PODF in 18.2)
 *                       IMXRT_SEMC_PODF_DIVIDER = 8
 *                       75Mhz    = 600Mhz / 8
 *
 * Set Sys PLL (PLL2) to  fOut    = (24Mhz * (20+(2*(DIV_SELECT)))
 *                        528Mhz  = (24Mhz * (20+(2*(1)))
 *
 * Set USB1 PLL (PLL3) to fOut    = (24Mhz * 20)
 *                         480Mhz = (24Mhz * 20)
 */

#define BOARD_XTAL_FREQUENCY      24000000
#define IMXRT_PRE_PERIPH_CLK_SEL  CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL1
#define IMXRT_PERIPH_CLK_SEL      CCM_CBCDR_PERIPH_CLK_SEL_PRE_PERIPH
#define IMXRT_ARM_PLL_DIV_SELECT  100
#define IMXRT_ARM_PODF_DIVIDER    2
#define IMXRT_AHB_PODF_DIVIDER    1
#define IMXRT_IPG_PODF_DIVIDER    4
#define IMXRT_PERCLK_CLK_SEL      CCM_CSCMR1_PERCLK_CLK_SEL_IPG_CLK_ROOT
#define IMXRT_PERCLK_PODF_DIVIDER 9
#define IMXRT_SEMC_PODF_DIVIDER   8
#define IMXRT_LPSPI_CLK_SELECT    CCM_CBCMR_LPSPI_CLK_SEL_PLL3_PFD0
#define IMXRT_LSPI_PODF_DIVIDER   8
#define IMXRT_USDHC1_CLK_SELECT   CCM_CSCMR1_USDHC1_CLK_SEL_PLL2_PFD0
#define IMXRT_USDHC1_PODF_DIVIDER 2
#define IMXRT_SYS_PLL_SELECT      CCM_ANALOG_PLL_SYS_DIV_SELECT_22

#define BOARD_CPU_FREQUENCY \
	(BOARD_XTAL_FREQUENCY * (IMXRT_ARM_PLL_DIV_SELECT / 2)) / IMXRT_ARM_PODF_DIVIDER

/* LED definitions ******************************************************************/
/* There are four LED status indicators located on the EVK Board.  The functions of
 * these LEDs include:
 *
 *   - Main Power Supply(D3)
 *     Green: DC 5V main supply is normal.
 *     Red:   J2 input voltage is over 5.6V.
 *     Off:   The board is not powered.
 *   - Reset RED LED(D15)
 *   - OpenSDA LED(D16)
 *   - USER LED(D18)
 *
 * Only a single LED, D18, is under software control.
 */

/* LED index values for use with board_userled() */

#define BOARD_USERLED     0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_USERLED_BIT (1 << BOARD_USERLED)

/* This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/imxrt_autoleds.c. The LED is used to encode
 * OS-related events as follows:
 *
 *   -------------------- ----------------------------- ------
 *   SYMBOL                   Meaning                   LED
 *   -------------------- ----------------------------- ------ */

#define LED_STARTED       0		/* TinyARA has been started  OFF    */
#define LED_HEAPALLOCATE  0		/* Heap has been allocated OFF    */
#define LED_IRQSENABLED   0		/* Interrupts enabled      OFF    */
#define LED_STACKCREATED  1		/* Idle stack created      ON     */
#define LED_INIRQ         2		/* In an interrupt         N/C    */
#define LED_SIGNAL        2		/* In a signal handler     N/C    */
#define LED_ASSERTION     2		/* An assertion failed     N/C    */
#define LED_PANIC         3		/* The system has crashed  FLASH  */
#undef  LED_IDLE				/* Not used                       */

/* Thus if the LED is statically on, TinyARA has successfully  booted and is,
 * apparently, running normally.  If the LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/* Button definitions ***************************************************************/

/* The IMXRT board has one external user button
 *
 * 1. SW8 (IRQ88)   GPIO5-00
 */

#define BUTTON_SW8        0

#define BUTTON_SW8_BIT    (1 << BUTTON_SW8)

/* PIO Disambiguation ***************************************************************/
/* LPUARTs
 *
 * Virtual console port provided by OpenSDA:
 *
 *          UART1_TXD   GPIO_AD_B0_12  LPUART1_TX
 *          UART1_RXD   GPIO_AD_B0_13  LPUART1_RX
 *
 *   NOTE: There are no alternative pin configurations for LPUART1.
 *
 * Arduino RS-232 Shield:
 *
 *   J22 D0 UART_RX/D0  GPIO_AD_B1_07  LPUART3_RX
 *   J22 D1 UART_TX/D1  GPIO_AD_B1_06  LPUART3_TX
 */

#define GPIO_LPUART3_RX   GPIO_LPUART3_RX_1	/* GPIO_AD_B1_07 */
#define GPIO_LPUART3_TX   GPIO_LPUART3_TX_1	/* GPIO_AD_B1_06 */

/* LPI2Cs
 *
 * Arduino Connector
 *
 *   J23 A4 A4/ADC4/SDA  GPIO_AD_B1_01  LPI2C1_SDA
 *   J23 A5 A5/ADC5/SCL  GPIO_AD_B1_00  LPI2C1_SCL
 */

#define GPIO_LPI2C1_SDA   GPIO_LPI2C1_SDA_2	/* GPIO_AD_B1_01 */
#define GPIO_LPI2C1_SCL   GPIO_LPI2C1_SCL_2	/* GPIO_AD_B1_00 */

#define GPIO_LPI2C2_SDA   GPIO_LPI2C2_SDA_2	/* GPIO_AD_B1_01 */
#define GPIO_LPI2C2_SCL   GPIO_LPI2C2_SCL_2	/* GPIO_AD_B1_00 */

#define GPIO_LPI2C3_SDA   GPIO_LPI2C3_SDA_2	/* GPIO_AD_B1_01 */
#define GPIO_LPI2C3_SCL   GPIO_LPI2C3_SCL_2	/* GPIO_AD_B1_00 */

#define GPIO_LPI2C4_SDA   GPIO_LPI2C4_SDA_2	/* GPIO_AD_B1_01 */
#define GPIO_LPI2C4_SCL   GPIO_LPI2C4_SCL_2	/* GPIO_AD_B1_00 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif							/* __ASSEMBLY__ */
#endif							/* __CONFIGS_IMXRT1050_EVK_INCLUDE_BOARD_H */
