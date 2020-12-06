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
	uint16_t baud;	 /*! Baud rate configuration Register -- 16 bits*/
	uint16_t reserv0;	 /*! reserved */
	uint32_t  tx;	 /*! Transmit register -- the value that needs to be tranmitted needs to be written here-32 bits*/
	uint32_t  rx;	 /*! Receive register -- the value that received from uart can be read from here --32 bits*/
	char  status;	 /*! Status register -- Reads various transmit and receive status - 8 bits*/
	char  reserv1;	 /*! reserved */
	uint16_t  reserv2; /*! reserved */
	uint16_t delay;    /*! Delays the transmit with specified clock - 16bits*/
	uint16_t reserv3;  /*! reserved */
	uint16_t control;   /*! Control Register -- Configures the no. of bits used, stop bits, parity enabled or not - 16bits*/
	uint16_t reserv5;  /*! reserved */
	char ie;	     /*! Enables the required interrupts - 8 bits*/
	char reserv6;   /*! reserved */
	uint16_t reserv7;  /*! reserved */
	char  iqcycles; /*! 8-bit register that indicates number of input qualification cycles - 8 bits*/
	char reserv8;   /*! reserved */
	uint16_t reserv9;  /*! reserved */
#ifdef USE_RX_THRESHOLD /*! This is to be used only when support is there. */
	char rx_threshold;	/*! RX FIFO size configuration register - 8 bits*/
	char reserv10;    /*! reserved */ 
	uint16_t reserv11;    /*! reserved */
#endif	
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
typedef void (*irq_cfg_func_t)(void);
#endif

struct uart_shakti_device_config {
	uintptr_t   port;
	uint32_t       sys_clk_freq;
	uint32_t       baud_rate;
	uint32_t       rxcnt_irq;
	uint32_t       txcnt_irq;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	irq_cfg_func_t cfg_func;
#endif
};

struct uart_shakti_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
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

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

/**
 * @brief Fill FIFO with data
 *
 * @param dev UART device struct
 * @param tx_data Data to transmit
 * @param size Number of bytes to send
 *
 * @return Number of bytes sent
 */
static int uart_shakti_fifo_fill(const struct device *dev,
		const uint8_t *tx_data,
		int size)
{
	volatile struct uart_shakti_regs_t *uart = DEV_UART(dev);
	int i;

	for (i = 0; i < size && ; i++)
	{
		while(uart->status & UART_STS_TX_FULL) ;
		uart->tx = tx_data[i];
	}

	return i;
}

/**
 * @brief Read data from FIFO
 *
 * @param dev UART device struct
 * @param rxData Data container
 * @param size Container size
 *
 * @return Number of bytes read
 */
static int uart_shakti_fifo_read(const struct device *dev,
		uint8_t *rx_data,
		const int size)
{
	volatile struct uart_shakti_regs_t *uart = DEV_UART(dev);
	int i = 0;
	uint32_t val;
	if(uart->status & UART_STS_RX_FULL || uart->status & UART_STS_RX_NOT_EMPTY )
	{
		for (i = 0; i < size; i++) {
			if(uart->status & UART_STS_RX_NOT_EMPTY)
				rx_data[i] = (uint8_t)(uart->rx);
			else
				break;
		}
	}
	return i;
}

/**
 * @brief Enable TX interrupt in ie register
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_shakti_irq_tx_enable(const struct device *dev)
{
	volatile struct uart_shakti_regs_t *uart = DEV_UART(dev);

	uart->ie |= UART_ENABLE_TX_EMPTY;
}

/**
 * @brief Disable TX interrupt in ie register
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_shakti_irq_tx_disable(const struct device *dev)
{
	volatile struct uart_shakti_regs_t *uart = DEV_UART(dev);

	uart->ie &= ~UART_ENABLE_TX_EMPTY;
}

/**
 * @brief Check if Tx IRQ has been raised
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static int uart_shakti_irq_tx_ready(const struct device *dev)
{
	volatile struct uart_shakti_regs_t *uart = DEV_UART(dev);

	return (uart->status & UART_STS_TX_EMPTY);
}

/**
 * @brief Check if nothing remains to be transmitted
 *
 * @param dev UART device struct
 *
 * @return 1 if nothing remains to be transmitted, 0 otherwise
 */
static int uart_shakti_irq_tx_complete(const struct device *dev)
{
	volatile struct uart_shakti_regs_t *uart = DEV_UART(dev);

	/*
	 * No TX EMPTY flag for this controller,
	 * just check if TX FIFO is not full
	 */
	return (uart->status & UART_STS_TX_EMPTY);
}

/**
 * @brief Enable RX interrupt in ie register
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_shakti_irq_rx_enable(const struct device *dev)
{
	volatile struct uart_shakti_regs_t *uart = DEV_UART(dev);

	uart->ie |= UART_ENABLE_RX_NOT_EMPTY;
}

/**
 * @brief Disable RX interrupt in ie register
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_shakti_irq_rx_disable(const struct device *dev)
{
	volatile struct uart_shakti_regs_t *uart = DEV_UART(dev);

	uart->ie &= ~UART_ENABLE_RX_NOT_EMPTY;
}

/**
 * @brief Check if Rx IRQ has been raised
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static int uart_shakti_irq_rx_ready(const struct device *dev)
{
	volatile struct uart_shakti_regs_t *uart = DEV_UART(dev);

	return !!(uart->status & UART_STS_RX_NOT_EMPTY);
}

/* No error interrupt for this controller */
static void uart_shakti_irq_err_enable(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static void uart_shakti_irq_err_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
}

/**
 * @brief Check if any IRQ is pending
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is pending, 0 otherwise
 */
static int uart_shakti_irq_is_pending(const struct device *dev)
{
	volatile struct uart_shakti_regs_t *uart = DEV_UART(dev);

	return !!(uart->status & (!UART_STS_TX_EMPTY | UART_STS_RX_NOT_EMPTY));
}

static int uart_shakti_irq_update(const struct device *dev)
{
	return 1;
}

/**
 * @brief Set the callback function pointer for IRQ.
 *
 * @param dev UART device struct
 * @param cb Callback function pointer.
 *
 * @return N/A
 */
static void uart_shakti_irq_callback_set(const struct device *dev,
		uart_irq_callback_user_data_t cb,
		void *cb_data)
{
	struct uart_shakti_data *data = DEV_DATA(dev);

	data->callback = cb;
	data->cb_data = cb_data;
}

static void uart_shakti_irq_handler(const struct device *dev)
{
	struct uart_shakti_data *data = DEV_DATA(dev);

	if (data->callback)
		data->callback(dev, data->cb_data);
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */


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

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	/* Ensure that uart IRQ is disabled initially */
	uart->ie = 0U;

	/* Setup IRQ handler */
	cfg->cfg_func();
#endif

	return 0;
}

static const struct uart_driver_api uart_shakti_driver_api = {
	.poll_in          = uart_shakti_poll_in,
	.poll_out         = uart_shakti_poll_out,
	.err_check        = NULL,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill        = uart_shakti_fifo_fill,
	.fifo_read        = uart_shakti_fifo_read,
	.irq_tx_enable    = uart_shakti_irq_tx_enable,
	.irq_tx_disable   = uart_shakti_irq_tx_disable,
	.irq_tx_ready     = uart_shakti_irq_tx_ready,
	.irq_tx_complete  = uart_shakti_irq_tx_complete,
	.irq_rx_enable    = uart_shakti_irq_rx_enable,
	.irq_rx_disable   = uart_shakti_irq_rx_disable,
	.irq_rx_ready     = uart_shakti_irq_rx_ready,
	.irq_err_enable   = uart_shakti_irq_err_enable,
	.irq_err_disable  = uart_shakti_irq_err_disable,
	.irq_is_pending   = uart_shakti_irq_is_pending,
	.irq_update       = uart_shakti_irq_update,
	.irq_callback_set = uart_shakti_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_SHAKTI_PORT_0

static struct uart_shakti_data uart_shakti_data_0;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_shakti_irq_cfg_func_0(void);
#endif

static const struct uart_shakti_device_config uart_shakti_dev_cfg_0 = {
	.port         = DT_INST_REG_ADDR(0),
	.sys_clk_freq = DT_INST_PROP(0, clock_frequency),
	.baud_rate    = DT_INST_PROP(0, current_speed),
	.rxcnt_irq    = CONFIG_UART_SHAKTI_PORT_0_RXCNT_IRQ,
	.txcnt_irq    = CONFIG_UART_SHAKTI_PORT_0_TXCNT_IRQ,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.cfg_func     = uart_shakti_irq_cfg_func_0,
#endif
};

DEVICE_AND_API_INIT(uart_shakti_0, DT_INST_LABEL(0),
		uart_shakti_init,
		&uart_shakti_data_0, &uart_shakti_dev_cfg_0,
		PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		(void *)&uart_shakti_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_shakti_irq_cfg_func_0(void)
{
	IRQ_CONNECT(DT_INST_IRQN(0),
			CONFIG_UART_SHAKTI_PORT_0_IRQ_PRIORITY,
			uart_shakti_irq_handler, DEVICE_GET(uart_shakti_0),
			0);

	irq_enable(DT_INST_IRQN(0));
}
#endif

#endif /* CONFIG_UART_SHAKTI_PORT_0 */

#ifdef CONFIG_UART_SHAKTI_PORT_1

static struct uart_shakti_data uart_shakti_data_1;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_shakti_irq_cfg_func_1(void);
#endif

static const struct uart_shakti_device_config uart_shakti_dev_cfg_1 = {
	.port         = DT_INST_REG_ADDR(1),
	.sys_clk_freq = DT_INST_PROP(1, clock_frequency),
	.baud_rate    = DT_INST_PROP(1, current_speed),
	.rxcnt_irq    = CONFIG_UART_SHAKTI_PORT_1_RXCNT_IRQ,
	.txcnt_irq    = CONFIG_UART_SHAKTI_PORT_1_TXCNT_IRQ,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.cfg_func     = uart_shakti_irq_cfg_func_1,
#endif
};

DEVICE_AND_API_INIT(uart_shakti_1, DT_INST_LABEL(1),
		uart_shakti_init,
		&uart_shakti_data_1, &uart_shakti_dev_cfg_1,
		PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		(void *)&uart_shakti_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_shakti_irq_cfg_func_1(void)
{
	IRQ_CONNECT(DT_INST_IRQN(1),
			CONFIG_UART_SHAKTI_PORT_1_IRQ_PRIORITY,
			uart_shakti_irq_handler, DEVICE_GET(uart_shakti_1),
			0);

	irq_enable(DT_INST_IRQN(1));
}
#endif

#endif /* CONFIG_UART_SHAKTI_PORT_1 */

#ifdef CONFIG_UART_SHAKTI_PORT_2

static struct uart_shakti_data uart_shakti_data_2;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_shakti_irq_cfg_func_2(void);
#endif

static const struct uart_shakti_device_config uart_shakti_dev_cfg_2 = {
	.port         = DT_INST_REG_ADDR(1),
	.sys_clk_freq = DT_INST_PROP(1, clock_frequency),
	.baud_rate    = DT_INST_PROP(1, current_speed),
	.rxcnt_irq    = CONFIG_UART_SHAKTI_PORT_2_RXCNT_IRQ,
	.txcnt_irq    = CONFIG_UART_SHAKTI_PORT_2_TXCNT_IRQ,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.cfg_func     = uart_shakti_irq_cfg_func_2,
#endif
};

DEVICE_AND_API_INIT(uart_shakti_2, DT_INST_LABEL(1),
		uart_shakti_init,
		&uart_shakti_data_2, &uart_shakti_dev_cfg_2,
		PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		(void *)&uart_shakti_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_shakti_irq_cfg_func_1(void)
{
	IRQ_CONNECT(DT_INST_IRQN(1),
			CONFIG_UART_SHAKTI_PORT_2_IRQ_PRIORITY,
			uart_shakti_irq_handler, DEVICE_GET(uart_shakti_2),
			0);

	irq_enable(DT_INST_IRQN(1));
}
#endif

#endif /* CONFIG_UART_SHAKTI_PORT_2 */
