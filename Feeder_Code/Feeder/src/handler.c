/*
  Created on: 01.10.2015
 *      Author: aschuhmann


#include "handler.h"

void printErrorMsg(const char * errMsg);
void printUsageErrorMsg(uint32_t CFSRValue);
void printBusFaultErrorMsg(uint32_t CFSRValue);
void printMemoryManagementErrorMsg(uint32_t CFSRValue);

void __attribute__( ( naked ) ) hard_fault_handler(void) {
	__asm__("TST lr, #4");
	__asm__("ITE EQ");
	__asm__("MRSEQ r0, MSP");
	__asm__("MRSNE r0, PSP");
	__asm__("MOV R1, LR");
	__asm__("B hard_fault_handler_c");
}

void __attribute__( ( naked ) ) hard_fault_handler_c(uint32_t *stack) {
	//static char msg[80];W
	printErrorMsg("In Hard Fault Handler\r\n");
	//sprintf(msg, "SCB->HFSR = 0x%08lx\r\n", SCB_HFSR);
	//printErrorMsg(msg);
	if ((SCB_HFSR & (1 << 30)) != 0) {
		printErrorMsg("Forced Hard Fault\r\n");
		//sprintf(msg, "SCB->CFSR = 0x%08lx\r\n", SCB_CFSR );
		//printErrorMsg(msg);
		if ((SCB_CFSR & 0xFFFF0000) != 0) {
			printUsageErrorMsg(SCB_CFSR);
		}
		if ((SCB_CFSR & 0xFF00) != 0) {
			printBusFaultErrorMsg(SCB_CFSR);
		}
		if ((SCB_CFSR & 0xFF) != 0) {
			printMemoryManagementErrorMsg(SCB_CFSR);
		}
	}
	stackDumpPrimitive(stack);
	__asm__ volatile("BKPT #01");
	while (1)
		;
}

void printErrorMsg(const char * errMsg) {
	while (*errMsg != '\0') {
		usart_send_blocking(USART1, *errMsg);
		++errMsg;
	}
}

void printUsageErrorMsg(uint32_t CFSRValue) {
	printErrorMsg("Usage fault: ");
	CFSRValue >>= 16; // right shift to lsb

	if ((CFSRValue & (1 << 9)) != 0) {
		printErrorMsg("Divide by zero\r\n");
	}
	if ((CFSRValue & (1 << 8)) != 0) {
		printErrorMsg("Unaligned Memory Access\r\n");
	}
	if ((CFSRValue & (1 << 3)) != 0) {
		printErrorMsg("Attempt to access a CoProz that doesn't exist\r\n");
	}
	if ((CFSRValue & (1 << 2)) != 0) {
		printErrorMsg("Invalid ProgramCounter Usage Fault\r\n");
	}
	if ((CFSRValue & (1 << 1)) != 0) {
		printErrorMsg("Illegal use of the EPSR Register\r\n");
	}
	if ((CFSRValue & (1 << 0)) != 0) {
		printErrorMsg("Illegal Instuction exectued\r\n");
	}
}

void printBusFaultErrorMsg(uint32_t CFSRValue) {
	printErrorMsg("Bus fault: ");
	CFSRValue = ((CFSRValue & 0x0000FF00) >> 8); // mask and right shift to lsb

	if ((CFSRValue & (1 << 7)) != 0) {
		printErrorMsg("BFAR invalid Address\r\n");
	}
	if ((CFSRValue & (1 << 4)) != 0) {
		printErrorMsg("Stacking Fault\r\n");
	}
	if ((CFSRValue & (1 << 3)) != 0) {
		printErrorMsg("Unstacking Fault\r\n");
	}
	if ((CFSRValue & (1 << 2)) != 0) {
		printErrorMsg("Imprecise data bus error\r\n");
	}
	if ((CFSRValue & (1 << 1)) != 0) {
		printErrorMsg("Precise data bus error\r\n");
	}
	if ((CFSRValue & (1 << 0)) != 0) {
		printErrorMsg("Instruction bus error\r\n");
	}
}

void printMemoryManagementErrorMsg(uint32_t CFSRValue) {
	printErrorMsg("Memory Management fault: ");
	CFSRValue &= 0x000000FF; // mask just mem faults

	if ((CFSRValue & (1 << 7)) != 0) {
		printErrorMsg("MMAR Addr valid\r\n");
	}
	if ((CFSRValue & (1 << 4)) != 0) {
		printErrorMsg("MemManage Fault on Stacking\r\n");
	}
	if ((CFSRValue & (1 << 3)) != 0) {
		printErrorMsg("MemManage Fault on Unstacking\r\n");
	}
	if ((CFSRValue & (1 << 1)) != 0) {
		printErrorMsg("Data access violation error\r\n");
	}
	if ((CFSRValue & (1 << 0)) != 0) {
		printErrorMsg("Instruction access violation error\r\n");
	}
}

enum {
	r0, r1, r2, r3, r12, lr, pc, psr
};

//void stackDump(uint32_t stack[]) {
//	static char msg[80];
//	sprintf(msg, "r0  = 0x%08lx\r\n", stack[r0]);
//	printErrorMsg(msg);
//	sprintf(msg, "r1  = 0x%08lx\r\n", stack[r1]);
//	printErrorMsg(msg);
//	sprintf(msg, "r2  = 0x%08lx\r\n", stack[r2]);
//	printErrorMsg(msg);
//	sprintf(msg, "r3  = 0x%08lx\r\n", stack[r3]);
//	printErrorMsg(msg);
//	sprintf(msg, "r12 = 0x%08lx\r\n", stack[r12]);
//	printErrorMsg(msg);
//	sprintf(msg, "lr  = 0x%08lx\r\n", stack[lr]);
//	printErrorMsg(msg);
//	sprintf(msg, "pc  = 0x%08lx\r\n", stack[pc]);
//	printErrorMsg(msg);
//	sprintf(msg, "psr = 0x%08lx\r\n", stack[psr]);
//	printErrorMsg(msg);
//	sprintf(msg, "BFAR  = 0x%.8X\r\n", *(unsigned int*) 0xE000ED38);
//	printErrorMsg(msg);
//	sprintf(msg, "CFSR  = 0x%.8X\r\n", *(unsigned int*) 0xE000ED28);
//	printErrorMsg(msg);
//	sprintf(msg, "HFSR  = 0x%.8X\r\n", *(unsigned int*) 0xE000ED2C);
//	printErrorMsg(msg);
//	sprintf(msg, "DFSR  = 0x%.8X\r\n", *(unsigned int*) 0xE000ED30);
//	printErrorMsg(msg);
//	sprintf(msg, "AFSR  = 0x%.8X\r\n", *(unsigned int*) 0xE000ED3C);
//	printErrorMsg(msg);
//}
void stackDumpPrimitive(uint32_t stack[]) {
	printIntandRet(stack[r0]);
	printIntandRet(stack[r1]);
	printIntandRet(stack[r2]);
	printIntandRet(stack[r3]);
	printIntandRet(stack[r12]);
	printIntandRet(stack[lr]);
	printIntandRet(stack[pc]);
	printIntandRet(stack[psr]);
}
inline void printIntandRet(int data) {
	usart_send_blocking(USART1, ((uint8_t *) &data)[3]);
	usart_send_blocking(USART1, ((uint8_t *) &data)[2]);
	usart_send_blocking(USART1, ((uint8_t *) &data)[1]);
	usart_send_blocking(USART1, ((uint8_t *) &data)[0]);
	usart_send_blocking(USART1, '\n');
}
*/
