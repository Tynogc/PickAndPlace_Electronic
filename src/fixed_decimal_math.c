#include <libopencm3/stm32/gpio.h>

#include "fixed_decimal_math.h"
#include "usart.h"
#include <math.h>

#define INLINE inline

INLINE bool fixed_dec_add(fixed_decimal_t augend, fixed_decimal_t addend,
		fixed_decimal_t* ergebnis) {
	return __builtin_uadd_overflow(augend, addend, ergebnis);
}

INLINE bool fixed_dec_subtract(fixed_decimal_t minuend,
		fixed_decimal_t subtrahend, fixed_decimal_t* ergebnis) {
	return __builtin_usub_overflow(minuend, subtrahend, ergebnis);
}

INLINE bool fixed_dec_multiply(fixed_decimal_t multiplikator,
		fixed_decimal_t multiplikant, fixed_decimal_t* ergebnis) {
	uint64_t temp = 0;
	temp = (uint64_t)(multiplikator) * (uint64_t)(multiplikant);
	temp += 1 << 15; // addiere ein halbes LSB des Endwertes (Rundung)
	if (temp & 0xFFFF000000000000) {
		return true;
	} else {
		*ergebnis = temp >> 16;
		return false;
	}
}

INLINE bool fixed_dec_divide(fixed_decimal_t divident, fixed_decimal_t divisor,
		fixed_decimal_t* ergebnis) {
	uint_fast64_t temp = (uint_fast64_t)(divident) << 16;
	if (divisor == 0) {
		return true; // Division durch 0 ist nicht gestattet.
	} else {
		temp += divisor >> 1;
		*ergebnis = temp / divisor;
		return false;
	}
}

INLINE bool fixed_dec_radic(fixed_decimal_t radikand, fixed_decimal_t* ergebnis) {
	// Prüfung auf negative Zahlen unter der Wurzel überspringen, weil der Datentyp
	// rein positiv definiert ist.

	uint32_t t, q, b;

	uint64_t r = radikand;
    b = 0x40000000;
    q = 0;

    while( b > 0x40 )
    {
        t = q + b;
        if( r >= t )
        {
            r -= t;
            q = t + b; // equivalent to q += 2*b
        }
        r <<= 1;
        b >>= 1;
    }
    *ergebnis = q>>8;

	return true;
}

void fixed_dec_test(uint32_t usart) {
	bool failed = true;
	fixed_decimal_t ergebnis = 0;

	usartSendBlkDat(usart, "Testcases für fixed-int-math\r\n");
	//usartSendBlkDat(usart, "1. Addieren ohne Überlauf: ");
	failed = fixed_dec_add(0xFFFFFFF0, 1, &ergebnis); // kein Overflow
	if (failed || (ergebnis != 0xFFFFFFF1)) {
		usartSendBlkDat(usart, "Add without overflow failed\r\n");
	}

	//usartSendBlkDat(usart, "2. Addieren mit Überlauf: ");
	failed = fixed_dec_add(0xFFFFFFFF, 1, &ergebnis); // erzeuge Overflow
	if (!failed) {
		usartSendBlkDat(usart, "Add overflow det failed!\r\n");
	}

	//usartSendBlkDat(usart, "3. Subtrahieren ohne Überlauf: ");
	failed = fixed_dec_subtract(0xFFFFFFFF, 1, &ergebnis); // kein Overflow
	if (failed || (ergebnis != 0xFFFFFFFE)) {
		usartSendBlkDat(usart, "Sub without overflow failed!\r\n");
	}

	//usartSendBlkDat(usart, "4. Subtrahieren mit Überlauf: ");
	failed = fixed_dec_subtract(0x000000FF, 0x100, &ergebnis); // erzeuge Overflow
	if (!failed) {
		usartSendBlkDat(usart, "Sub overflow det failed!\r\n");
	}

	//usartSendBlkDat(usart, "5. Multiplizeren ohne Überlauf: ");
//	gpio_set(GPIOC, GPIO7);
	failed = fixed_dec_multiply(0x00FFFFFF, 0x30000, &ergebnis); // kein Overflow
//	gpio_clear(GPIOC, GPIO7);
//	gpio_set(GPIOC, GPIO7);
//	volatile float temp = (float) 0x00FFFFFF * (float) ergebnis;
//	gpio_clear(GPIOC, GPIO7);
	if (failed || (ergebnis != (3*0x00FFFFFF))) {
		usartSendBlkDat(usart, "Mul without overflow failed\r\n");
	}
//	if (temp == 0 || ergebnis == 0) {
//		gpio_clear(GPIOC, GPIO7);
//	}

	//usartSendBlkDat(usart, "6. Multiplizieren mit Überlauf: ");
	failed = fixed_dec_multiply(0xFFFFFFFF, 0x20000, &ergebnis); // erzeuge Overflow
	if (!failed) {
		usartSendBlkDat(usart, "Mul overflow det failed\r\n");
	}

	//usartSendBlkDat(usart, "7. Teilen durch nicht-null: ");
	failed = fixed_dec_divide(3015, 30, &ergebnis);
	if (failed || (ergebnis != 0x00648000)) {
		usartSendBlkDat(usart, "Div without overflow failed!\r\n");
	}

	//usartSendBlkDat(usart, "8. Teilen durch Null: ");
	failed = fixed_dec_divide(0xFF, 0, &ergebnis);
	if (!failed) {
		usartSendBlkDat(usart, "Div throw null error!\r\n");
	}

	failed = fixed_dec_radic(0x90000, &ergebnis);
	if (!failed || ergebnis!=0x30000) {
		usartSendBlkDat(usart, "sqrt failed\r\n");
	}
	failed = fixed_dec_radic(0x118000, &ergebnis);
	if (!failed || ergebnis!=0x42eec) {
		usartSendBlkDat(usart, "sqrt failed\r\n");
	}
//	gpio_set(GPIOC, GPIO7);
	failed = fixed_dec_radic(0xfffe1eb8, &ergebnis); // Zahl knapp am oberen Limit
//	gpio_clear(GPIOC, GPIO7);
//	gpio_set(GPIOC, GPIO7);
//	volatile float temp = sqrtf((float) ergebnis);
//	gpio_clear(GPIOC, GPIO7);
	if (!failed || ergebnis != 0xffff0f) {
		usartSendBlkDat(usart, "sqrt failed\r\n");
	}
}

static inline fixed_decimal_half_t fixed_dec_get_natural(fixed_decimal_t data) {
	return data >> 16;
}

static inline fixed_decimal_half_t fixed_dec_get_frac(fixed_decimal_t data) {
	return data << 16;
}

void fixed_dec_print(fixed_decimal_t data) {
	printf("%u,%u", fixed_dec_get_natural(data), fixed_dec_get_frac(data));
}
