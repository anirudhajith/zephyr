/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief UART driver for the shakti uart v2
 */

#define DT_DRV_COMPAT shakti_uart0

#include <kernel.h>
#include <arch/cpu.h>
#include <drivers/uart.h>
#define asm __asm__

#define UART_STS_RX_THRESHOLD		1 << 8
#define UART_STS_BREAK_ERROR	        1 << 7
#define UART_STS_FRAME_ERROR	        1 << 6
#define UART_STS_OVERRUN	        1 << 5
#define UART_STS_PARITY_ERROR		1 << 4
#define UART_STS_RX_FULL		1 << 3
#define UART_STS_RX_NOT_EMPTY		1 << 2
#define UART_STS_TX_FULL		1 << 1
#define UART_STS_TX_EMPTY		1 << 0

/*! UART Interrupt Enable bits description */
#define UART_ENABLE_RX_THRESHOLD	1 << 8
#define UART_ENABLE_BREAK_ERROR		1 << 7
#define UART_ENABLE_FRAME_ERROR		1 << 6
#define UART_ENABLE_OVERRUN		1 << 5
#define UART_ENABLE_PARITY_ERROR	1 << 4
#define UART_ENABLE_RX_FULL		1 << 3
#define UART_ENABLE_RX_NOT_EMPTY	1 << 2
#define UART_ENABLE_TX_FULL		1 << 1
#define UART_ENABLE_TX_EMPTY		1 << 0
#define UART_TX_BUFFER_SIZE       	10000

struct uart_shakti_regs_t {
	uint16_t baud;	 		/*! Baud rate configuration Register -- 16 bits*/
	uint16_t reserv0;	 	/*! reserved */
	uint32_t  tx;	 		/*! Transmit register -- the value that needs to be tranmitted needs to be written here-32 bits*/
	uint32_t  rx;	 		/*! Receive register -- the value that received from uart can be read from here --32 bits*/
	char  status;	 		/*! Status register -- Reads various transmit and receive status - 8 bits*/
	char  reserv1;	 		/*! reserved */
	uint16_t  reserv2; 		/*! reserved */
	uint16_t delay;   		/*! Delays the transmit with specified clock - 16bits*/
	uint16_t reserv3;  		/*! reserved */
	uint16_t control;   	/*! Control Register -- Configures the no. of bits used, stop bits, parity enabled or not - 16bits*/
	uint16_t reserv5;  		/*! reserved */
	char ie;	     		/*! Enables the required interrupts - 8 bits*/
	char reserv6;   		/*! reserved */
	uint16_t reserv7;  		/*! reserved */
	char  iqcycles; 		/*! 8-bit register that indicates number of input qualification cycles - 8 bits*/
};


struct uart_shakti_device_config {
	uintptr_t   port;
	uint32_t       sys_clk_freq;
	uint32_t       baud_rate;
	uint32_t       rxcnt_irq;
	uint32_t       txcnt_irq;
};

#define DEV_CFG(dev)						\
	((const struct uart_shakti_device_config * const)	\
	 (dev)->config)
#define DEV_UART(dev)						\
	((struct uart_shakti_regs_t *)(DEV_CFG(dev))->port)
#define DEV_DATA(dev)						\
	((struct uart_shakti_data * const)(dev)->data)

/**
 * @brief Output a character in polled mode.
 *
 * Writes data to tx register if transmitter is not full.
 *
 * @param dev UART device struct
 * @param c Character to send
 */
static void uart_shakti_poll_out(const struct device *dev,
		unsigned char c)
{
	volatile struct uart_shakti_regs_t *uart = DEV_UART(dev);

	/* Wait while TX FIFO is full */
	while (uart->status & UART_STS_TX_FULL);
	uart->tx = (int)c;
}

/**
 * @brief Poll the device for input.
 *
 * @param dev UART device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */
static int uart_shakti_poll_in(const struct device *dev, unsigned char *c)
{
	volatile struct uart_shakti_regs_t *uart = DEV_UART(dev);

	if (uart->status & UART_STS_RX_NOT_EMPTY) {
		*c = (uart->rx);
		return 0;
	}

	return -1;
}


static int uart_shakti_init(const struct device *dev)
{
	const struct uart_shakti_device_config * const cfg = DEV_CFG(dev);
	volatile struct uart_shakti_regs_t *uart = DEV_UART(dev);

	/* Enable TX and RX channels */

	/* Set baud rate */
	uart->baud = cfg->sys_clk_freq / (16 * cfg->baud_rate);
//	uart->baud = 162; 

	printk("clk freq = %d\n",cfg->sys_clk_freq);
	printk("baudrate = %d\n",cfg->baud_rate);

	return 0;
}

static const struct uart_driver_api uart_shakti_driver_api = {
	.poll_in          = uart_shakti_poll_in,
	.poll_out         = uart_shakti_poll_out,
	.err_check        = NULL,
};

