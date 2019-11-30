#include "usart.h"

uint32_t usart_printf = 0;

static void printf_write (void* usart, char dat);

void usart_init(uint32_t usart, uint32_t baud) {

	// Enable Clock
	switch (usart) {
	case USART1:
		rcc_periph_clock_enable(RCC_USART1);
		usart_printf = usart;
		init_printf(&usart_printf,printf_write);
		break;
	case USART2:
		rcc_periph_clock_enable(RCC_USART2);
		break;
	case USART3:
		rcc_periph_clock_enable(RCC_USART3);
		break;
	case USART4:
		rcc_periph_clock_enable(RCC_USART4);
		break;
	default:
		; // gar nix.

	}

	// setup GPIO: tx on PB6, rx on PB7
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
	gpio_set_af(GPIOB, GPIO_AF0, GPIO6); // _TX
	gpio_set_af(GPIOB, GPIO_AF0, GPIO7); // _RX

	// Setup USART parameters.
	usart_disable(usart);
	usart_set_baudrate(usart, baud);
	usart_set_databits(usart, 8);
	usart_set_stopbits(usart, 1);
	usart_set_parity(usart, USART_PARITY_NONE);
	usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);
	usart_set_mode(usart, USART_MODE_TX_RX);

	usart_enable(usart);
}

void usartSendBlkDat(uint32_t usart, const char* data) {
	size_t length = strlen(data);
	for (size_t i = 0; i < length; i++) {
		usart_send_blocking(usart, (uint16_t) data[i]);
	}

}

void usartSendBlkDatLen(uint32_t usart, const char* data, uint16_t length) {
	for (int i = 0; i < length; i++) {
		usart_send_blocking(usart, (uint16_t) data[i]);
	}
}

void usart_send_newline(uint32_t usart) {
	usartSendBlkDat(usart, "\r\n");
}

void usart_send_dec16 (uint32_t usart, uint16_t zahl) {
	usart_send_blocking(usart, 0x30+(zahl/10000));		// 10k   Stelle
	usart_send_blocking(usart, 0x30+(zahl%10000/1000));	//  1k   Stelle
	usart_send_blocking(usart, 0x30+(zahl%1000/100));	// 100er Stelle
	usart_send_blocking(usart, 0x30+(zahl%100/10));		// 10er  Stelle
	usart_send_blocking(usart, 0x30+(zahl%10));			// 1er   Stelle
}


//int _read (int fd, const void *buf, size_t count)
//{
//  size_t CharCnt = 0x00;
//  uint8_t* buf_loc = (uint8_t*) buf;
//  (void)fd;                            /* Parameter is not used, suppress unused argument warning */
//  for (;count > 0x00; --count) {
//    if (usart_get_flag(USART_DEFAULT, USART_ISR_RXNE)) { /* Any data in receiver buffer */
//      if (CharCnt != 0x00) {           /* No, at least one char received? */
//        break;                         /* Yes, return received char(s) */
//      } else {                         /* Wait until a char is received */
//        while (usart_get_flag(USART_DEFAULT, USART_ISR_RXNE)) {};
//      }
//    }
//    CharCnt++;                         /* Increase char counter */
//    /* Save character received by UARTx device into the receive buffer */
//    *buf_loc = (unsigned char)usart_recv(USART_DEFAULT);
//    if (*buf_loc == 0x0D) {     /* New line character (CR) received ? */
//      *buf_loc = '\n';           /* Yes, convert LF to '\n' char. */
//      break;                           /* Stop loop and return received char(s) */
//    }
//    (uint8_t*)buf++;                   /* Increase buffer pointer */
//  }
//  return CharCnt;
//}

void printf_write (void* usart, char dat)
{
	uint32_t usart_int = *(uint32_t*)(usart);
    while (usart_get_flag(usart_int,USART_ISR_TXE) == 0) {};
    if (dat == '\n') {
      /* Send '\r'(0x0D) before each '\n'(0x0A). */
      /* Save a character into the transmit buffer of the UART0 device */
      usart_send_blocking(usart_int,0x0D);
    }
    /* Save a character into the transmit buffer of the UART0 device */
    usart_send(usart_int,dat);
}
