/*
 * usart.h
 *
 *  Created on: 04.03.2017
 *      Author: philipp
 */

#ifndef INC_USART_H_
#define INC_USART_H_
#include <stdint.h>
#include <string.h>

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "printf.h"

void usart_init(uint32_t usart, uint32_t baud);
void usartSendBlkDat(uint32_t usart, const char* data);
void usartSendBlkDatLen(uint32_t usart, const char* data, uint16_t length);
void usart_send_newline(uint32_t usart);
void usart_send_dec16 (uint32_t usart, uint16_t zahl);


#endif /* INC_USART_H_ */
