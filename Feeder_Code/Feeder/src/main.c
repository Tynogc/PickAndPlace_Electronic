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
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "defines.h"

#define DMA_BANK_SIZE ((LED_COUNT+5) * 8 * 3)
#define DMA_SIZE (DMA_BANK_SIZE*2)
uint8_t dma_data[DMA_SIZE];
volatile uint32_t led_data[LED_COUNT];
volatile uint32_t led_cur = 0;


#define DMA_STEPPER_BANK_SIZE (500)
#define DMA_STEPPER_SIZE (DMA_STEPPER_BANK_SIZE*2)
uint16_t stepper_data[DMA_STEPPER_SIZE];
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

   // Reflexkoppler-LED
   gpio_mode_setup(REFLEX_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, REFLEX_LED_PIN);

   // Reflexkoppler-Fototransistor
   gpio_mode_setup(REFLEX_INPUT_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, REFLEX_INPUT_PIN);

}

static void clockInit (void) {
	// Takt auf Vollgas
	rcc_clock_setup_in_hsi_out_48mhz();

	// Enable clocks for GPIO ports
   rcc_periph_clock_enable(RCC_GPIOA);
   rcc_periph_clock_enable(RCC_GPIOB);
   rcc_periph_clock_enable(RCC_GPIOC);

   // Enable TIM2/3
   rcc_periph_clock_enable(RCC_TIM2);
   rcc_periph_clock_enable(RCC_TIM3);

   // Enable DMA1
   rcc_periph_clock_enable(RCC_DMA);

   // Enable USART1
   rcc_periph_clock_enable(RCC_USART1);
}

void sys_tick_handler(void) {
	static int i=0;
	if (i==100) {
		gpio_toggle(GPIOC, GPIO8); // blaue LED
		i = 0;
	}
	i++;
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

   usart_enable(USART1);

   systick_counter_disable();
//   systick_set_clocksource(); // takt noch rausfinden...
   systick_clear();
   systick_set_reload(6000000/6/500); // 2kHz
   //systick_interrupt_enable();
   //systick_counter_enable();
}

void uartSendBlkDat(uint32_t timer, const char* data) {
   size_t length = strlen(data);
   for (size_t i = 0; i < length; i++) {
      usart_send_blocking(timer, (uint16_t) data[i]);
   }

}
void uartSendBlkDatLen(uint32_t timer, const char* data, uint16_t length) {
   for (int i = 0; i < length; i++) {
      usart_send_blocking(timer, (uint16_t) data[i]);
   }
}

void stepper_timer_setup(void) {
	// Verwende Timer 2 - Kanal 4 DMA1-Ch4 - Ausgang PB11 - AF2
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11);
	gpio_set_af(GPIOB, GPIO_AF2, GPIO11);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, GPIO11);

	timer_reset(TIM2);

	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM2);
	timer_set_period(TIM2, 0xFFFF);

	timer_disable_oc_output(TIM2, TIM_OC1);
	timer_disable_oc_output(TIM2, TIM_OC2);
	timer_disable_oc_output(TIM2, TIM_OC3);
	timer_disable_oc_output(TIM2, TIM_OC4);

	timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_TOGGLE);
	timer_disable_oc_clear(TIM2, TIM_OC4);
	//timer_enable_oc_preload(TIM2, TIM_OC3);
	timer_set_oc_value(TIM2, TIM_OC4, 0);
	timer_set_oc_polarity_high(TIM2, TIM_OC4);
	timer_enable_oc_output(TIM2, TIM_OC4);

	//timer_enable_preload(TIM2);
	timer_set_counter(TIM2, 0);
	__asm__("DSB");
	timer_enable_counter(TIM2);

	timer_enable_irq(TIM2, TIM_DIER_CC3DE); // Update DMA request
	timer_set_dma_on_compare_event(TIM2);
}

static int stepper_dma(uint16_t *tx_buf, int tx_len) {

   dma_disable_channel(DMA1, DMA_CHANNEL4);

   nvic_set_priority(NVIC_DMA1_CHANNEL4_5_IRQ, 0);
   nvic_enable_irq(NVIC_DMA1_CHANNEL4_5_IRQ);

   // Reset DMA channels
   dma_channel_reset(DMA1, DMA_CHANNEL4);

   // Set up tx dma
   dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t) &TIM2_CCR4);
   dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t) tx_buf);
   dma_set_number_of_data(DMA1, DMA_CHANNEL4, tx_len);
   dma_set_read_from_memory(DMA1, DMA_CHANNEL4);
   dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
   dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_32BIT);
   dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_16BIT);
   dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_HIGH);

   dma_enable_circular_mode(DMA1, DMA_CHANNEL4);
   dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
   dma_enable_half_transfer_interrupt(DMA1, DMA_CHANNEL4);
   dma_enable_transfer_error_interrupt(DMA1, DMA_CHANNEL4);
   dma_enable_channel(DMA1, DMA_CHANNEL4);
   return 0;
}


void pwm_setup(void) {
	// Verwende Timer 3 - Kanal 3 DMA1-Ch3 - Ausgang PB0
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0);
	gpio_set_af(GPIOB, GPIO_AF1, GPIO0);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, GPIO0);

   timer_reset(TIM3);

   timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
   timer_continuous_mode(TIM3);
   timer_set_period(TIM3, WSP);

   timer_disable_oc_output(TIM3, TIM_OC1);
   timer_disable_oc_output(TIM3, TIM_OC2);
   timer_disable_oc_output(TIM3, TIM_OC3);
   timer_disable_oc_output(TIM3, TIM_OC4);

   timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
   timer_disable_oc_clear(TIM3, TIM_OC3);
   timer_enable_oc_preload(TIM3, TIM_OC3);
   timer_set_oc_value(TIM3, TIM_OC3, 0);
   timer_set_oc_polarity_high(TIM3, TIM_OC3);
   timer_enable_oc_output(TIM3, TIM_OC3);

   timer_enable_preload(TIM3);
   timer_set_counter(TIM3, 0);

   timer_enable_counter(TIM3);

   timer_enable_irq(TIM3, TIM_DIER_UDE); // Update DMA request
   timer_set_dma_on_update_event(TIM3);
}

static void populate_dma_data(uint8_t *dma_data_bank) {
   for(int i=0; i<DMA_BANK_SIZE;) {
      led_cur = led_cur % (LED_COUNT+5);
      if(led_cur < LED_COUNT) {
         uint32_t v = led_data[led_cur];
         for(int j=0; j<24; j++) { // schreibe untere 24 Bit jedes 32-Bit Wertes
            dma_data_bank[i++] = (v & 0x800000) ? (uint8_t)WS1 : (uint8_t)WS0; // Timerwert für 1 oder 0 schreiben
            v <<= 1; // eins nach Links schieben
         }
      } else {
         for(int j=0; j<24; j++) { // erzeuge 24 weitere Bit als Whitespace (WS2812-latch)
            dma_data_bank[i++] = 0;
         }
      }
      led_cur++;
   }
}
volatile int32_t aktX = 0;
volatile int32_t sollX = 1000;
volatile float aktV = 0;
volatile int32_t sollV = 30;
volatile int32_t aktA = 0;
volatile uint32_t aMax = 1;

static void generateDmaStepperData(uint16_t* dma_data, uint16_t offset, uint16_t letzterSchritt){
	uint16_t lastValue = 0;
	if (offset >= (DMA_STEPPER_BANK_SIZE-1)){
		__asm__("BKPT #01");
	}
	if (offset != 0){ //Innerhalb des arrays
		lastValue = dma_data[offset-1];
	}else{
		if (dma_data == stepper_data){
			lastValue = dma_data[DMA_STEPPER_SIZE-1];
		}else{
			lastValue = dma_data[-1];
		}
	}
	dma_data[offset] = lastValue + (letzterSchritt/2);
	dma_data[offset+1] = lastValue + letzterSchritt;
}

static void populate_stepper_data(uint16_t *dma_data_bank) {
	if (sollX < aktX){
		__asm__("BKPT #01");
	}
	for (int i = 0; i<DMA_STEPPER_BANK_SIZE/2; i++) {
		uint32_t deltaX = sollX - aktX;
		static uint16_t letzterSchritt = 0;
		float deltaV = sollV - aktV;
		float restschritte = 0.0;
		if (aktV > 0){
			restschritte = (deltaV*deltaV) / (2*aktA) + (sollV * deltaV) / aktA;
		}
		if (deltaX < restschritte + 1) { // bremsen anfangen
			aktV -= aMax * letzterSchritt;

		} else if (aktV < sollV) { // beschleunigen
				aktV += aMax * letzterSchritt;
		}
		// sonst: konstante Fahrtgeschwindigkeit erreicht
		// Schritt erzeugen: v = 1/t -> t = 1/aktV
		letzterSchritt = 1 / (2*aktV);
		aktX++;

		generateDmaStepperData(dma_data_bank, i, letzterSchritt);
	}
}

static int timer_dma(uint8_t *tx_buf, int tx_len) {

   dma_disable_channel(DMA1, DMA_CHANNEL3);

   nvic_set_priority(NVIC_DMA1_CHANNEL2_3_IRQ, 0);
   nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_IRQ);

   // Reset DMA channels
   dma_channel_reset(DMA1, DMA_CHANNEL3);

   // Set up tx dma
   dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t) &TIM3_CCR3);
   dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t) tx_buf);
   dma_set_number_of_data(DMA1, DMA_CHANNEL3, tx_len);
   dma_set_read_from_memory(DMA1, DMA_CHANNEL3);
   dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL3);
   dma_set_peripheral_size(DMA1, DMA_CHANNEL3, DMA_CCR_PSIZE_16BIT);
   dma_set_memory_size(DMA1, DMA_CHANNEL3, DMA_CCR_MSIZE_8BIT);
   dma_set_priority(DMA1, DMA_CHANNEL3, DMA_CCR_PL_HIGH);

   dma_enable_circular_mode(DMA1, DMA_CHANNEL3);
   dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL3);
   dma_enable_half_transfer_interrupt(DMA1, DMA_CHANNEL3);
   dma_enable_transfer_error_interrupt(DMA1, DMA_CHANNEL3);
   dma_enable_channel(DMA1, DMA_CHANNEL3);
   return 0;
}

/*void dma1_channel1_isr(void) {
	;
}*/

void dma1_channel2_3_isr(void) {
   if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL3, DMA_TCIF)) { // dma Puffer leer
	   dma_clear_interrupt_flags(DMA1, DMA_CHANNEL3, DMA_TCIF);
	   populate_dma_data(&dma_data[DMA_BANK_SIZE]);
   }
   if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL3, DMA_HTIF)) { // dma Puffer halbleer
	   dma_clear_interrupt_flags(DMA1, DMA_CHANNEL3, DMA_HTIF);
	   populate_dma_data(dma_data);
   }
   if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL3, DMA_TEIF)) { // dma Übertragungsfehler
	   dma_clear_interrupt_flags(DMA1, DMA_CHANNEL3, DMA_TEIF);
	   uartSendBlkDat(USART1, "Error in DMA ISR Channel 2\r\n");
   }
}

void dma1_channel4_5_isr(void) {
   if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL4, DMA_TCIF)) { // dma Puffer leer
	   dma_clear_interrupt_flags(DMA1, DMA_CHANNEL4, DMA_TCIF);
	   gpio_set(GPIOC, GPIO8);
	   populate_stepper_data(&stepper_data[DMA_STEPPER_BANK_SIZE]);
	   gpio_clear(GPIOC, GPIO8);
	   if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL4, DMA_HTIF)){
		   __asm__("BKPT #01");
	   }
   }
   if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL4, DMA_HTIF)) { // dma Puffer halbleer
	   dma_clear_interrupt_flags(DMA1, DMA_CHANNEL4, DMA_HTIF);
	   gpio_set(GPIOC, GPIO9);
	   populate_stepper_data(stepper_data);
	   gpio_clear(GPIOC, GPIO9);
	   if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL4, DMA_TCIF)){
		   __asm__("BKPT #01");
	   }
   }
   if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL4, DMA_TEIF)) { // dma Übertragungsfehler
	   dma_clear_interrupt_flags(DMA1, DMA_CHANNEL4, DMA_TEIF);
	   uartSendBlkDat(USART1, "Error in DMA ISR Channel 1\r\n");
   }
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
         microstepping = 1;
   }
   gpio_set(DRIVE_PORT, DRIVE_RST);
}




int main(void) {
   clockInit();
   gpioInit();
   uart_setup ();
//   usart_send_blocking(USART1, 'x');

   memset(dma_data, 0, DMA_SIZE);
   memset((void*)led_data, 0, LED_COUNT*sizeof(*led_data));
   populate_dma_data(dma_data);
   populate_dma_data(&dma_data[DMA_BANK_SIZE]);
//   usart_send_blocking(USART1, 'A');

   //pwm_setup ();
//   usart_send_blocking(USART1, 'B');

   //timer_dma (dma_data, DMA_SIZE);
//   usart_send_blocking(USART1, 'C');

   // Motor initalisieren
   memset(stepper_data, 0, DMA_STEPPER_SIZE);
   populate_stepper_data(stepper_data);
   populate_stepper_data(&stepper_data[DMA_STEPPER_BANK_SIZE]);
   stepper_timer_setup();
   stepper_dma(stepper_data, DMA_STEPPER_SIZE);

   // ein paar Variablen
   // Achtung: fahrstrecke ist unabhängig vom aktuellen Mikrosteppinglevel!
   int fahrstrecke = 0; // in Schritten
   uint8_t microstepping = 0;
   uint8_t istReferenziert = 0;
   uint16_t powerVcc = 0;
   uint16_t motorstrom = 0;
   uint16_t drvTemperatur = 0;

   led_data[0] = 0x00550055;
   led_data[1] = 0x00FF0000;
   led_data[2] = 0x0000FF00;
   led_data[3] = 0x000000FF;


   // boot vollständig.
   //gpio_set(GPIOC, GPIO8);
   usart_send_blocking(USART1, 'D');
   while (1) {

      // kleiner Test: LED an- und abstellbar.
      if (gpio_get(GPIOA, GPIO0) == 1) {
         gpio_toggle (GPIOC, GPIO9);
         usart_send_blocking(USART1, 'E');
         led_data[0] = (led_data[0] & 0xFFFF0000) | ((led_data[0] & 0xFF00)+0x1000);
         while (gpio_get(GPIOA, GPIO0));
         for (volatile int i=0; i<1000000; i++) {}
      }
   }
}
