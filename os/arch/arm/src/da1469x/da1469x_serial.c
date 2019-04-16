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

#include "hw_uart.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define UART_IS_CONFIGURED_FLAG        (0x15)

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
#if 0
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
/* UART configuration */
static uart_config uart_init = {
	.baud_rate              = HW_UART_BAUDRATE_115200,	//CHECK VALUES ONCE
	.data                   = HW_UART_DATABITS_8,
	.parity                 = HW_UART_PARITY_NONE,
	.stop                   = HW_UART_STOPBITS_1,
	.auto_flow_control      = 0,
	.use_dma                = 0,
	.use_fifo               = 1,
	.tx_dma_channel         = HW_DMA_CHANNEL_1,
	.rx_dma_channel         = HW_DMA_CHANNEL_0,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int da1469x_up_setup(struct uart_dev_s *dev);
static void da1469x_up_shutdown(struct uart_dev_s *dev);
static int da1469x_up_attach(struct uart_dev_s *dev);
static void da1469x_up_detach(struct uart_dev_s *dev);
//static int da1469x_up_interrupt(int irq, void *context, FAR void *arg);
static int da1469x_up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int da1469x_up_receive(struct uart_dev_s *dev, uint32_t *status);
static void da1469x_up_rxint(struct uart_dev_s *dev, bool enable);
static bool da1469x_up_rxavailable(struct uart_dev_s *dev);
static void da1469x_up_send(struct uart_dev_s *dev, int ch);
static void da1469x_up_txint(struct uart_dev_s *dev, bool enable);
static bool da1469x_up_txready(struct uart_dev_s *dev);
static bool da1469x_up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Serial driver UART operations */

static const struct uart_ops_s g_uart_ops = {
        .setup = da1469x_up_setup,
        .shutdown = da1469x_up_shutdown,
        .attach = da1469x_up_attach,
        .detach = da1469x_up_detach,
        .ioctl = da1469x_up_ioctl,
        .receive = da1469x_up_receive,
        .rxint = da1469x_up_rxint,
        .rxavailable = da1469x_up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
        .rxflowcontrol = NULL,
#endif
        .send = da1469x_up_send,
        .txint = da1469x_up_txint,
        .txready = da1469x_up_txready,
        .txempty = da1469x_up_txempty,
};

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

#ifdef CONFIG_DA1469X_UART0
static uart_config g_uart0priv = {
	.baud_rate              = CONFIG_UART0_BAUD,	//CHECK VALUES ONCE
	.data                   = CONFIG_UART0_BITS,    ///CHECK VALUES ONCE currently = 3
	.parity                 = CONFIG_UART0_PARITY,
	.stop                   = CONFIG_UART0_2STOP,
	.auto_flow_control      = 0,
	.use_dma                = 0,
	.use_fifo               = 1,
	.tx_dma_channel         = 0,
	.rx_dma_channel         = 0,
};
//	.port = HW_UART1,
//	.irq = DA1469X_UART0_IRQ,

static uart_dev_t g_uart0port = {
	.recv = {
		.size = CONFIG_UART0_RXBUFSIZE,
		.buffer = g_uart0rxbuffer,
	},
	.xmit = {
		.size = CONFIG_UART0_TXBUFSIZE,
		.buffer = g_uart0txbuffer,
	},
	.ops = &g_uart_ops,
	.priv = &g_uart0priv,
};
#endif

#ifdef CONFIG_DA1469X_UART1
static uart_config g_uart1priv = {
	.baud_rate              = CONFIG_UART1_BAUD,	//CHECK VALUES ONCE
	.data                   = CONFIG_UART1_BITS,
	.parity                 = CONFIG_UART1_PARITY,
	.stop                   = CONFIG_UART1_2STOP,
	.auto_flow_control      = 0,
	.use_dma                = 0,
	.use_fifo               = 1,
	.tx_dma_channel         = 0,
	.rx_dma_channel         = 0,
};
//	.port = HW_UART2,
//	.irq = DA1469X_UART1_IRQ,

static uart_dev_t g_uart1port = {
	.recv = {
		.size = CONFIG_UART1_RXBUFSIZE,
		.buffer = g_uart1rxbuffer,
	},
	.xmit = {
		.size = CONFIG_UART1_TXBUFSIZE,
		.buffer = g_uart1txbuffer,
	},
	.ops = &g_uart_ops,
	.priv = &g_uart1priv,
};
#endif

#ifdef CONFIG_DA1469X_UART2
static uart_config g_uart2priv = {
	.baud_rate              = CONFIG_UART2_BAUD,	//CHECK VALUES ONCE
	.data                   = CONFIG_UART1_BITS,
	.parity                 = CONFIG_UART1_PARITY,
	.stop                   = CONFIG_UART1_2STOP,
	.auto_flow_control      = 0,
	.use_dma                = 0,
	.use_fifo               = 1,
	.tx_dma_channel         = 0,
	.rx_dma_channel         = 0,
};
//	.data                   = HW_UART_DATABITS_8,
//	.port = HW_UART3,
//	.irq = DA1469X_UART2_IRQ,

static uart_dev_t g_uart2port = {
	.recv = {
		.size = CONFIG_UART2_RXBUFSIZE,
		.buffer = g_uart2rxbuffer,
	},
	.xmit = {
		.size = CONFIG_UART2_TXBUFSIZE,
		.buffer = g_uart2txbuffer,
	},
	.ops = &g_uart_ops,
	.priv = &g_uart2priv,
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
	uart_config *priv = (uart_config *)dev->priv;
	uart_config uart_setup;
	DEBUGASSERT(priv);
	// deinit
//	hw_uart_deinit();
	//HW_UART_ID id = ad_uart_data->ctrl->id; >>> DA1469X_CONSOLE_UART_PORT

	uart_setup.baud_rate = CONFIG_UART0_BAUD;    //CHECK VALUES ONCE
	uart_setup.data = CONFIG_UART0_BITS;    ///CHECK VALUES ONCE currently = 3
	uart_setup.parity = CONFIG_UART0_PARITY;
	uart_setup.stop = CONFIG_UART0_2STOP;
	uart_setup.auto_flow_control = 0;
	uart_setup.use_dma = 0;
	uart_setup.use_fifo = 1;
	uart_setup.tx_dma_channel = 0;
	uart_setup.rx_dma_channel = 0;

	return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void da1469x_up_shutdown(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int da1469x_up_attach(struct uart_dev_s *dev)
{
// TO DO
	uart_config *priv = (uart_config *)dev->priv;
	int ret = 0;
	DEBUGASSERT(priv);
	/* Attach and enable the IRQ */

//	ret = irq_attach(priv->irq, da1469x_up_interrupt, NULL);
	if (ret == OK) {
		/* Enable the interrupt (RX and TX interrupts are still disabled
		 * in the UART
		 */

//		up_enable_irq(priv->irq);
	}

	return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void da1469x_up_detach(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: da1469x_up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked
 *   when an interrupt received on the 'irq'  It should call
 *   uart_transmitchars or uart_receivechar to perform the
 *   appropriate data transfers.  The interrupt handling logic\
 *   must be able to map the 'irq' number into the approprite
 *   uart_dev_s structure in order to call these functions.
 *
static int da1469x_up_interrupt(int irq, void *context, FAR void *arg)
{
	return OK;
}
 ****************************************************************************/

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int da1469x_up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	return OK;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int da1469x_up_receive(struct uart_dev_s *dev, uint32_t *status)
{
	return OK;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void da1469x_up_rxint(struct uart_dev_s *dev, bool enable)
{
	uart_config *priv = (uart_config *)dev->priv;
	DEBUGASSERT(priv);
//	hw_uart_rec_data_int_set(, enable);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool da1469x_up_rxavailable(struct uart_dev_s *dev)
{
	return true;
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void da1469x_up_send(struct uart_dev_s *dev, int ch)
{
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void da1469x_up_txint(struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool da1469x_up_txready(struct uart_dev_s *dev)
{
	return true;
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool da1469x_up_txempty(struct uart_dev_s *dev)
{
	return true;
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
 * Pre-processor Definitions
 **************************************************************************/
/* Configuration **********************************************************/

/* Select UART parameters for the selected console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#define DA1469X_CONSOLE_UART_PORT HW_UART1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#define DA1469X_CONSOLE_UART_PORT HW_UART2
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#define DA1469X_CONSOLE_UART_PORT HW_UART3
#else
 #error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#endif

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

/**************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 **************************************************************************/

void up_lowsetup(void)
{
	hw_uart_init(DA1469X_CONSOLE_UART_PORT, &uart_init);
//	hw_uart_write_scr(DA1469X_CONSOLE_UART_PORT, UART_IS_CONFIGURED_FLAG);
}
/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/
#endif
int up_putc(int ch)
{
	return ch;
}
