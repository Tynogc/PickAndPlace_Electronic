/*
 * profiler.h
 *
 *  Created on: 01.10.2015
 *      Author: aschuhmann
 */

#ifndef HANDLER_H_
#define HANDLER_H_

#undef STM32F1
#define STM32F1
#include <stdlib.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/mpu.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/stm32/usart.h>

void hard_fault_handler(void);
void hard_fault_handler_c(uint32_t stack[]);
void printErrorMsg(const char *errMsg);
void printUsageErrorMsg(uint32_t CFSRValue);
void printBusFaultErrorMsg(uint32_t CFSRValue);
void printMemoryManagementErrorMsg(uint32_t CFSRValue);
void stackDump(uint32_t stack[]);
void stackDumpPrimitive(uint32_t stack[]);
void printIntandRet(int data);
void mem_manage_handler(void);

#endif /* PROFILER_H_ */
