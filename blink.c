#include <stdint.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/lptimer.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/dbgmcu.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/assert.h>
#include <libopencm3/cm3/itm.h>

volatile int cnt = 0;
volatile int tempo = 1000;

void sys_tick_handler(void)
{
	if (cnt++ == tempo) {
		cnt = 0;
		gpio_toggle(GPIOE, GPIO8);
		gpio_toggle(GPIOB, GPIO2);
	}
}

void dbgmcu_set_trace_mode(uint32_t trace_mode);
void dbgmcu_enable_trace(bool enable);

void dbgmcu_set_trace_mode(uint32_t trace_mode)
{
	uint32_t cr = DBGMCU_CR & ~DBGMCU_CR_TRACE_MODE_MASK;
	DBGMCU_CR = cr | trace_mode;
}

void dbgmcu_enable_trace(bool enable)
{
	if (enable)
		DBGMCU_CR |= DBGMCU_CR_TRACE_IOEN;
	else
		DBGMCU_CR &= ~DBGMCU_CR_TRACE_IOEN;
}

void trace_init(void);
void trace_wait_fifo(uint8_t channel);
void trace_fast(uint8_t channel, char msg[], size_t len);
void trace(uint8_t channel, char msg[], size_t len);

void trace_init(void)
{
	dbgmcu_set_trace_mode(DBGMCU_CR_TRACE_MODE_ASYNC);
	dbgmcu_enable_trace(true);

	tpiu_set_sspr(TPIU_SPPR_ASYNC_MANCHESTER);
	TPUI_DEVID_NRZ_SUPPORTED
}

void trace_wait_fifo(uint8_t channel) {
	while (!(ITM_STIM8(channel) & ITM_STIM_FIFOREADY));
}

void trace_fast(uint8_t channel, char msg[], size_t len)
{
	int i = 0;

	if (len > sizeof(uint32_t)) {
		// start on 32b aligned buf
		while ((uint32_t)&msg[i] & sizeof(uint32_t)) {
			trace_wait_fifo(channel);
			ITM_STIM8(channel) = msg[i];
			i++;
			len--;
		}

		// copy remaining until less than 4bytes
		while (len > sizeof(uint32_t)) {
			trace_wait_fifo(channel);
			ITM_STIM32(channel) = *(uint32_t *) &msg[i];
			i+=sizeof(uint32_t);
			len-=sizeof(uint32_t);
		}
	}

	while (len > sizeof(uint32_t)) {
		trace_wait_fifo(channel);
		ITM_STIM8(channel) = msg[i];
		i++;
		len--;
	}
}

void trace(uint8_t channel, char msg[], size_t len)
{
	int i = 0;
	while (len--) {
		trace_wait_fifo(channel);
		ITM_STIM8(channel) = msg[i++];
	}
}

int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART2, '\r');
			}
			usart_send_blocking(USART2, ptr[i]);
		}
		trace(0, ptr, len);
		return i;
	}
	errno = EIO;
	return -1;
}

int main(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOE);

	/* led on B2 */
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2);
	/* led on E8 */
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);

	rcc_clock_setup(&rcc_clock_config[1]);

	rcc_set_mco(RCC_HSI16);
	// XXX add mcopre helper

	/* systick at 1000hz, clocked on ahb */
	systick_set_frequency(1000, rcc_ahb_frequency);
	systick_interrupt_enable();
	systick_counter_enable();

	rcc_periph_clock_enable(RCC_USART2);
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);
	gpio_set_af(GPIOD, GPIO_AF7, GPIO6);
	gpio_set_af(GPIOD, GPIO_AF7, GPIO5);

	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART2, USART_MODE_TX);
	usart_enable(USART2);

	trace_init();

	int c = 0;
	while (1) {
		fprintf(stderr, "%s:%d : %d\n", __func__, __LINE__, c++);
	}

	return 0;
}

