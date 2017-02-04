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
//#include <libopencm3/usb/cdc.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "defines.h"

// maximum is at about 4000
#define LED_COUNT 1

//int _write(int file, char *ptr, int len);

static void gpioInit (void) {
	// blaue und grüne LED
	gpio_mode_setup (GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 | GPIO9);

	// taster
	gpio_mode_setup (GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO0);

	// Schrittmotortreiber
	gpio_mode_setup (DRIVE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		DRIVE_STEP | DRIVE_DIR | DRIVE_EN | DRIVE_SLP | DRIVE_RST | DRIVE_MS_1 |
		DRIVE_MS_2 | DRIVE_MS_3);
	gpio_set(DRIVE_PORT, DRIVE_EN); // Endstufe aus.

	// WS2812
	gpio_mode_setup(WS2812_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, WS2812_PIN);

	// Reflexkoppler-LED
	gpio_mode_setup(REFLEX_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, REFLEX_LED_PIN);

	// Reflexkoppler-Fototransistor
	gpio_mode_setup(REFLEX_INPUT_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, REFLEX_INPUT_PIN);

}

static void clockInit (void)
{
	rcc_clock_setup_in_hsi_out_48mhz();

	/* Enable clocks for GPIO ports */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Enable TIM3 Periph */
	rcc_periph_clock_enable(RCC_TIM3);

	/* Enable DMA1 clock */
	rcc_periph_clock_enable(RCC_DMA);

	rcc_periph_clock_enable(RCC_USART1);
}

static void uart_setup (void) {

	// setup GPIO: tx on PB10, rx on PB11
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO9); // _TX
	gpio_set_af(GPIOA, GPIO_AF1, GPIO10); // _RX

  // Setup UART parameters.
	usart_disable(USART1);
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, 1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX);

	/* Enable USART3 Receive interrupt. */
	//USART_CR1(USART2) |= USART_CR1_RXNEIE;

	usart_enable(USART1);
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
	// Configure GPIOs: OUT = PA7
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO7);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, GPIO7);

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

void stepperInit (uint8_t microstepping) {
	gpio_clear(DRIVE_PORT, DRIVE_MS_1 | DRIVE_MS_2 | DRIVE_MS_3);

	switch (microstepping) {
		case MICROSTEPPING_NONE:
			microstepping = 1;
			break;
		case MICROSTEPPING_HALF:
			gpio_set(DRIVE_PORT, DRIVE_MS_1);
			microstepping = 2;
			break;
		case MICROSTEPPING_QUARTER:
			gpio_set(DRIVE_PORT, DRIVE_MS_2);
			microstepping = 4;
			break;
		case MICROSTEPPING_EIGHT:
			gpio_set(DRIVE_PORT, DRIVE_MS_1 | DRIVE_MS_2);
			microstepping = 8;
			break;
		case MICROSTEPPING_SIXTEENTH:
			gpio_set(DRIVE_PORT, DRIVE_MS_1 | DRIVE_MS_2 | DRIVE_MS_3);
			microstepping = 16;
			break;
		default:
			microstepping = 1; // nur Vollschritte...
	}
	gpio_set(DRIVE_PORT, DRIVE_RST);
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


static void dma_int_enable(void) {
	// SPI1 TX on DMA1 Channel 3
	nvic_set_priority(NVIC_DMA1_CHANNEL2_3_IRQ, 0);
	nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_IRQ);
}

static int timer_dma(uint8_t *tx_buf, int tx_len)
{
	dma_int_enable();

	// Reset DMA channels
	dma_channel_reset(DMA1, DMA_CHANNEL3);

	// Set up tx dma
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

// SPI transmit completed with DMA
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
	clockInit();
	gpioInit();
	uart_setup ();
	usart_send_blocking(USART1, 'x');

	memset(dma_data, 0, DMA_SIZE);
	memset((void*)led_data, 0, LED_COUNT*sizeof(*led_data));
	populate_dma_data(dma_data);
	populate_dma_data(&dma_data[DMA_BANK_SIZE]);
	usart_send_blocking(USART1, 'A');

	timer_dma (dma_data, DMA_SIZE);
	usart_send_blocking(USART1, 'B');

	pwm_setup ();
	usart_send_blocking(USART1, 'C');

	// boot vollständig.
	gpio_set(GPIOC, GPIO8);
	usart_send_blocking(USART1, 'D');

	// ein paar Variablen
	// Achtung: fahrstrecke ist unabhängig vom aktuellen Mikrosteppinglevel!
	int fahrstrecke = 0; // in Schritten
	uint8_t istReferenziert = 0;
	uint16_t powerVcc = 0;
	uint16_t motorstrom = 0;
	uint16_t drvTemperatur = 0;
	uint32_t ledfarbe = 0x00101010; // Modus-R-G-B (32 Bit)

	while (1) {
		// kleiner Test: LED an- und abstellbar.
		if (gpio_get(GPIOA, GPIO0) == 1) {
			gpio_toggle (GPIOC, GPIO9);
			usart_send_blocking(USART1, 'E');
			led_data[0] = (led_data[0] & 0xFFFF0000) | ((led_data[0] & 0xFF00)+0x1000);
			while (gpio_get(GPIOA, GPIO0));
			for (volatile int i=0; i<1000000; i++) {
				; // nix.
			}
		}
	}
}
