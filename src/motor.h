/*
 * motor.h
 *
 *  Created on: 09.03.2017
 *      Author: philipp
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "defines.h"
#include "printf.h"

typedef enum {
	MOTOR_CW = 1, MOTOR_CCW
} rotation_t;

typedef enum {
	FULL = 1, HALF, QUARTER, EIGHT, SIXTEENTH
} microstepping_t;

typedef struct {
	uint32_t step_port;
	uint32_t step;
	uint32_t af_number;
	uint32_t direction_port;
	uint32_t direction;
	uint32_t enable_port;
	uint32_t enable;
	uint32_t sleep_port;
	uint32_t sleep;
	uint32_t reset_port;
	uint32_t reset;
	uint32_t ms1_port;
	uint32_t ms1;
	uint32_t ms2_port;
	uint32_t ms2;
	uint32_t ms3_port;
	uint32_t ms3;
	uint32_t timernumber;
	uint32_t timer_rcc;
	uint32_t dmanumber;
	uint8_t timerchannel;
	uint8_t dmachannel;
	uint8_t dmainterrupt;
} motoranschluss_t;

typedef struct {
	float accel;
	size_t motor_tx_len;
	uint32_t *motor_tx_buf;
	uint8_t microstepping;
	bool reversed;
	float speed_max;
} motoreinstellungen_t;

#define RINGBUFFER_SIZE 8
#define RINGBUFFER_MASK (RINGBUFFER_SIZE-1)

typedef struct {
	uint32_t steps;
	rotation_t rotat;
	float drive_speed;
	float target_speed;
} fahrpfad_t;

typedef struct {
	uint16_t _read;
	uint16_t _write;
	fahrpfad_t _data[RINGBUFFER_SIZE];
} fahrpfad_buf_t;

typedef struct {
	int steps_soll;
	float speed_actual;
	uint16_t last_timerval;
	motoranschluss_t *anschluss;
	motoreinstellungen_t *config;
	fahrpfad_buf_t *fahrpfad_buf;
	const char *name;
	float speed_soll;
	int steps;
	rotation_t direction_actual;
	volatile bool is_running;
} motor_t;



#define STEPPER_CMD_MASK 0xFFFF0000
#define STEPPER_CMD_OFFSET 16
//2er Pot als CMD
#define STEPPER_CMD_STOP (0x1 << STEPPER_CMD_OFFSET)
#define STEPPER_CMD_ROT_CW (0x2 << STEPPER_CMD_OFFSET)
#define STEPPER_CMD_ROT_CCW (0x4 << STEPPER_CMD_OFFSET)

void motor_buffer_init(fahrpfad_buf_t *data);
void motor_buffer_write(fahrpfad_buf_t *data, fahrpfad_t *eingabe);
void motor_buffer_read(fahrpfad_buf_t *data, fahrpfad_t *ausgabe);
void motor_buffer_read_no_change(fahrpfad_buf_t *data, fahrpfad_t *ausgabe);
bool motor_buffer_full(fahrpfad_buf_t *data);
bool motor_buffer_empty (fahrpfad_buf_t *data);

void stepper_gpio_init(motor_t *data);
void stepper_set_direction(motor_t *data, rotation_t richtung);
void stepper_timer_init(motor_t *data);
void stepper_dma_init(motor_t *data);
void stepper_drive(motor_t *data, uint32_t *bufferstart);
void stepper_new_cmd(motor_t* data, uint32_t* stepp_array, uint32_t offset);

#endif /* INC_MOTOR_H_ */
