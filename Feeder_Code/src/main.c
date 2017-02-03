/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2013 Stephen Dwyer <scdwyer@ualberta.ca>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
//#include <libopencm3/usb/usbd.h>
//#include <libopencm3/usb/cdc.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "defines.h"

// maximum is at about 4000
#define LED_COUNT 0x1
// minimum ID offset is 0x100 (first ID byte mustn't be 0x00)
#define ID_OFFSET 0xA000

int _write(int file, char *ptr, int len);

static void clock_setup(void)
{
	rcc_clock_setup_in_hsi_out_48mhz();

	/* Enable clocks for GPIO ports */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Enable TIM3 Periph */
	rcc_periph_clock_enable(RCC_TIM3);

	/* Enable DMA1 clock */
	rcc_periph_clock_enable(RCC_DMA);

	rcc_periph_clock_enable(RCC_USART2);
}

static void uart_setup(void) {
	/* Enable the USART3 interrupt. */
//	nvic_enable_irq(NVIC_USART3_IRQ);

	/* setup GPIO: tx on PB10, rx on PB11 */
	//gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
	//	GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);
	gpio_set_af(GPIOA, GPIO_AF2, GPIO15);
//	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_MODE_AF, GPIO_USART2_RX);

        /* Setup UART parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, 1);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART2, USART_MODE_RX); // no tx for now

	/* Enable USART3 Receive interrupt. */
	USART_CR1(USART2) |= USART_CR1_RXNEIE;

	/* Finally enable the USART. */
	usart_enable(USART2);
}

#define TICK_NS (1000/48)
#define WS0 (350 / TICK_NS)
#define WS1 (800 / TICK_NS)
#define WSP (1300 / TICK_NS)
#define WSL (20000 / TICK_NS)

#define DMA_BANK_SIZE (40 * 8 * 3)
#define DMA_SIZE (DMA_BANK_SIZE*2)
static uint8_t dma_data[DMA_SIZE];
static volatile uint32_t led_data[LED_COUNT];
static volatile uint32_t led_cur = 0;

static void pwm_setup(void) {
	/* Configure GPIOs: OUT=PA7 */
	gpio_set_output_options(GPIOA, GPIO_AF2, GPIO_OSPEED_HIGH, GPIO6);

	timer_reset(TIM3);

	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	timer_disable_oc_output(TIM3, TIM_OC2);
	timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
	timer_disable_oc_clear(TIM3, TIM_OC2);
	timer_set_oc_value(TIM3, TIM_OC2, 0);
	timer_enable_oc_preload(TIM3, TIM_OC2);
	timer_set_oc_polarity_high(TIM3, TIM_OC2);
	timer_enable_oc_output(TIM3, TIM_OC2);

	timer_set_dma_on_update_event(TIM3);
	timer_enable_irq(TIM3, TIM_DIER_UDE); // in fact, enable DMA on update

	timer_enable_preload(TIM3);
	timer_continuous_mode(TIM3);
	timer_set_period(TIM3, WSP);

	timer_enable_counter(TIM3);
}

static void populate_dma_data(uint8_t *dma_data_bank) {
	for(int i=0; i<DMA_BANK_SIZE;) {
		led_cur = led_cur % (LED_COUNT+3);
		if(led_cur < LED_COUNT) {
			uint32_t v = led_data[led_cur];
			for(int j=0; j<24; j++) {
				dma_data_bank[i++] = (v & 0x800000) ? WS1 : WS0;
				v <<= 1;
			}
		} else {
			for(int j=0; j<24; j++) {
				dma_data_bank[i++] = 0;
			}
		}
		led_cur++;
	}
}

// Befehl der über den UART reingekommen ist
static void handle_cmd(uint8_t cmd) {
	static enum {
		STATE_WAITING,
		STATE_GOT_ID1,
		STATE_GOT_ID2,
		STATE_GOT_COL1,
		STATE_GOT_COL2,
		STATE_GOT_COL3,
		STATE_GOT_COL4,
		STATE_GOT_COL5,
	} cmd_state = STATE_WAITING;
	static uint16_t id = 0;
	static uint16_t col_r = 0;
	static uint16_t col_g = 0;
	static uint16_t col_b = 0;

	gpio_toggle(GPIOC, GPIO8);	/* LED on/off */

	switch(cmd_state) {
	case STATE_WAITING:
		id = cmd;
		/* send a bunch of 0x00 to re-sync protocol */
		if(id != 0x00)
			cmd_state = STATE_GOT_ID1;
		break;
	case STATE_GOT_ID1:
		id = (id << 8) + cmd;
		cmd_state = STATE_GOT_ID2;
		break;
	case STATE_GOT_ID2:
		col_r = cmd;
		cmd_state = STATE_GOT_COL1;
		break;
	case STATE_GOT_COL1:
		cmd_state = STATE_GOT_COL2;
		break;
	case STATE_GOT_COL2:
		col_g = cmd;
		cmd_state = STATE_GOT_COL3;
		break;
	case STATE_GOT_COL3:
		cmd_state = STATE_GOT_COL4;
		break;
	case STATE_GOT_COL4:
		col_b = cmd;
		cmd_state = STATE_GOT_COL5;
		break;
	case STATE_GOT_COL5:
		id -= ID_OFFSET;
		if(id < LED_COUNT)
			led_data[id] = (col_g << 16) | (col_r << 8) | (col_b);
		cmd_state = STATE_WAITING;
		break;
	}
}

void usart3_isr(void)
{
        /* Check if we were called because of RXNE. */
        //if ((USART_SR(USART2) & USART_SR_RXNE) != 0) {
                handle_cmd(usart_recv(USART2));
        //}
}


static void dma_int_enable(void) {
	/* SPI1 TX on DMA1 Channel 3 */
	nvic_set_priority(NVIC_DMA1_CHANNEL2_3_IRQ, 0);
	nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_IRQ);
}

/* Not used in this example
static void dma_int_disable(void) {
 	nvic_disable_irq(NVIC_DMA1_CHANNEL3_IRQ);
}
*/

static int timer_dma(uint8_t *tx_buf, int tx_len)
{
	dma_int_enable();

	/* Reset DMA channels*/
	dma_channel_reset(DMA1, DMA_CHANNEL3);

	/* Set up tx dma */
	dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t)&TIM_CCR2(TIM3));
	dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t)tx_buf);
	dma_set_number_of_data(DMA1, DMA_CHANNEL3, tx_len);
	dma_set_read_from_memory(DMA1, DMA_CHANNEL3);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL3);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL3, DMA_CCR_PSIZE_32BIT);
	dma_set_memory_size(DMA1, DMA_CHANNEL3, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(DMA1, DMA_CHANNEL3, DMA_CCR_PL_HIGH);

	dma_enable_circular_mode(DMA1, DMA_CHANNEL3);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL3);
	dma_enable_half_transfer_interrupt(DMA1, DMA_CHANNEL3);
	dma_enable_channel(DMA1, DMA_CHANNEL3);

	return 0;
}

/* SPI transmit completed with DMA */
void dma1_channel3_isr(void)
{
	if ((DMA1_ISR & DMA_ISR_TCIF3) != 0) {
		DMA1_IFCR |= DMA_IFCR_CTCIF3;
		populate_dma_data(&dma_data[DMA_BANK_SIZE]);
	}
	if ((DMA1_ISR & DMA_ISR_HTIF3) != 0) {
		DMA1_IFCR |= DMA_IFCR_CHTIF3;
		populate_dma_data(dma_data);
	}
}


int main(void) {

	clock_setup();

	// blaue LED
	gpio_mode_setup (GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);

	// taster
	gpio_mode_setup (GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO0);

	// DEBUG COM
	uart_setup ();

	memset(dma_data, 0, DMA_SIZE);
	memset((void*)led_data, 0, LED_COUNT*sizeof(*led_data));
	//populate_dma_data(dma_data);
	//populate_dma_data(&dma_data[DMA_BANK_SIZE]);


	timer_dma (dma_data, DMA_SIZE);
	pwm_setup ();

	// boot vollständig.
	gpio_set (GPIOC, GPIO8);


	while (1) {

		// kleiner Test: LED an- und abstellbar.
		if (gpio_get(GPIOA, GPIO0) == 1) {
			gpio_toggle (GPIOC, GPIO8);
			while (gpio_get(GPIOA, GPIO0));
			for (volatile int i=0; i<1000000; i++) {
				; // nix.
			}
		}

		//__asm__("wfe");
	}
}
