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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <tinyara/irq.h>
#include <tinyara/arch.h>
#include <tinyara/serial/serial.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/*
 * If we are not using the serial driver for the console, then we
 * still must provide some minimal implementation of up_putc.
 */

#undef TTYS0_DEV
#undef TTYS1_DEV
#undef TTYS2_DEV

#undef UART0_ASSIGNED
#undef UART1_ASSIGNED
#undef UART2_ASSIGNED

/* Which UART with be ttyS0/console and which ttyS1? ttyS2 */

/* First pick the console and ttys0. This could be any of UART0-2 */
#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#define CONSOLE_DEV             g_uart0port             /* UART0 is console */
#define TTYS0_DEV               g_uart0port             /* UART0 is ttyS0 */
#define UART0_ASSIGNED  1
#define HAVE_SERIAL_CONSOLE
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#define CONSOLE_DEV             g_uart1port             /* UART1 is console */
#define TTYS0_DEV               g_uart1port             /* UART1 is ttyS0 */
#define UART1_ASSIGNED  1
#define HAVE_SERIAL_CONSOLE
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#define CONSOLE_DEV             g_uart2port             /* UART2 is console */
#define TTYS0_DEV               g_uart2port             /* UART2 is ttyS0 */
#define UART2_ASSIGNED  1
#define HAVE_SERIAL_CONSOLE
#else
#undef CONSOLE_DEV                                              /* No console */
#if defined(CONFIG_DA1469X_UART0)
#define TTYS0_DEV               g_uart0port             /* UART0 is ttyS0 */
#define UART0_ASSIGNED  1
#elif defined(CONFIG_DA1469X_UART1)
#define TTYS0_DEV               g_uart1port             /* UART1 is ttyS0 */
#define UART1_ASSIGNED  1
#elif defined(CONFIG_DA1469X_UART2)
#define TTYS0_DEV               g_uart2port             /* UART2 is ttyS0 */
#define UART2_ASSIGNED  1
#endif
#endif

/* Pick ttyS1. This could be any of UART0-2 excluding the console UART. */
#if defined(CONFIG_DA1469X_UART0) && !defined(UART0_ASSIGNED)
#define TTYS1_DEV               g_uart0port             /* UART0 is ttyS1 */
#define UART0_ASSIGNED  1
#elif defined(CONFIG_DA1469X_UART1) && !defined(UART1_ASSIGNED)
#define TTYS1_DEV               g_uart1port             /* UART1 is ttyS1 */
#define UART1_ASSIGNED  1
#elif defined(CONFIG_DA1469X_UART2) && !defined(UART2_ASSIGNED)
#define TTYS1_DEV               g_uart2port             /* UART2 is ttyS1 */
#define UART2_ASSIGNED  1
#endif

/*
 * Pick ttyS2. This could be one of UART1-2. It can't be UART0 because that
 * was either assigned as ttyS0 or ttyS1. One of these could also be the
 * console
 */
#if defined(CONFIG_DA1469X_UART1) && !defined(UART1_ASSIGNED)
#define TTYS2_DEV               g_uart1port             /* UART1 is ttyS2 */
#define UART1_ASSIGNED  1
#elif defined(CONFIG_DA1469X_UART2) && !defined(UART2_ASSIGNED)
#define TTYS2_DEV               g_uart2port             /* UART2 is ttyS2 */
#define UART2_ASSIGNED  1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct da1469x_up_dev_s {
	uint32_t port;				/* UART port */
	uint32_t im;				/* Saved IM value */
	uint32_t baud;				/* Configured baud */
	uint32_t irq;				/* IRQ associated with this UART */
	uint32_t parity;			/* 0=none, 1=odd, 2=even */
	uint32_t bits;				/* 5: 5bits    6: 6bits    7: 7bits    8: 8bits */
	uint32_t stopbits2;			/* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Variables
 ****************************************************************************/

#ifdef CONFIG_DA1469X_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_DA1469X_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_DA1469X_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif

#define DA1469X_UART0_IRQ (UART0_IRQn+16)
#define DA1469X_UART1_IRQ (UART1_IRQn+16)
#define DA1469X_UART2_IRQ (UART2_IRQn+16)

/*
static struct da1469x_up_dev_s g_uart0priv = {
	.port = HAL_UART_0,
	.baud = CONFIG_UART0_BAUD,
	.irq = DA1469X_UART0_IRQ,
	.parity = CONFIG_UART0_PARITY,
	.bits = CONFIG_UART0_BITS,
	.stopbits2 = CONFIG_UART0_2STOP,
};
*/

#ifdef CONFIG_DA1469X_UART0
static uart_dev_t g_uart0port = {
	.recv = {
		.size = CONFIG_UART0_RXBUFSIZE,
		.buffer = g_uart0rxbuffer,
	},
	.xmit = {
		.size = CONFIG_UART0_TXBUFSIZE,
		.buffer = g_uart0txbuffer,
	},
//	.ops = &g_uart_ops,
//	.priv = &g_uart0priv,
};
#endif

/*
static struct da1469x_up_dev_s g_uart1priv = {
	.port = HAL_UART_1,
	.baud = CONFIG_UART1_BAUD,
	.irq = DA1469X_UART1_IRQ,
	.parity = CONFIG_UART1_PARITY,
	.bits = CONFIG_UART1_BITS,
	.stopbits2 = CONFIG_UART1_2STOP,
};
*/

#ifdef CONFIG_DA1469X_UART1
static uart_dev_t g_uart1port = {
	.recv = {
		.size = CONFIG_UART1_RXBUFSIZE,
		.buffer = g_uart1rxbuffer,
	},
	.xmit = {
		.size = CONFIG_UART1_TXBUFSIZE,
		.buffer = g_uart1txbuffer,
	},
//	.ops = &g_uart_ops,
//	.priv = &g_uart1priv,
};
#endif

/*
static struct da1469x_up_dev_s g_uart2priv = {
	.port = HAL_UART_2,
	.baud = CONFIG_UART2_BAUD,
	.irq = DA1469X_UART2_IRQ,
	.parity = CONFIG_UART2_PARITY,
	.bits = CONFIG_UART2_BITS,
	.stopbits2 = CONFIG_UART2_2STOP,
};
*/

#ifdef CONFIG_DA1469X_UART2
static uart_dev_t g_uart2port = {
	.recv = {
		.size = CONFIG_UART2_RXBUFSIZE,
		.buffer = g_uart2rxbuffer,
	},
	.xmit = {
		.size = CONFIG_UART2_TXBUFSIZE,
		.buffer = g_uart2txbuffer,
	},
//	.ops = &g_uart_ops,
//	.priv = &g_uart2priv,
};
#endif

//extern UART_REGISTER_T *const g_uart_regbase[];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: da1469x_up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/
static int da1469x_up_setup(struct uart_dev_s *dev)
{
	return OK;
}
/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
	CONSOLE_DEV.isconsole = true;
	da1469x_up_setup(&CONSOLE_DEV);

	(void)uart_register("/dev/console", &CONSOLE_DEV);

	/* Register all UARTs */
#ifdef TTYS0_DEV
	uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
#ifdef TTYS1_DEV
	uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
	uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif
}

/**************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 **************************************************************************/

void up_lowputc(char ch)
{
//	hal_uart_put_char(DA1469X_CONSOLE_UART_PORT, ch);
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
	/* Check for LF */

	if (ch == '\n') {
		/* Add CR */

		up_lowputc('\r');
	}

	up_lowputc(ch);
	return ch;
}
