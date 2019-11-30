/*
 * ws2812.h
 *
 *  Created on: 09.03.2017
 *      Author: philipp
 */

#ifndef INC_WS2812_H_
#define INC_WS2812_H_

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#define TICK_NS (1000/48) // 48 ist der CPU-Takt in MHz
#define WS0 (350 / TICK_NS)
#define WS1 (800 / TICK_NS)
#define WSP (1300 / TICK_NS)
#define WSL (20000 / TICK_NS)

typedef struct {
	uint32_t port;
	uint32_t pin;
	uint32_t af_number;
	uint16_t count;
	uint16_t current_led;
	uint32_t timernumber;
	uint32_t timer_rcc;
	uint32_t dmanumber;
	uint8_t timerchannel;
	uint8_t dmachannel;
	uint8_t dmainterrupt;
	uint8_t *dma_buf;
	uint16_t dma_len;
	volatile uint32_t *led_buf;
} ws2812_t;

void ws2812_init(ws2812_t *leds);
void ws2812_dma_init(ws2812_t *data);
void ws2812_generate_data(ws2812_t *data, uint8_t *dma_data_startpointer);

#endif /* INC_WS2812_H_ */
