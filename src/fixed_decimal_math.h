/*
 * fixed_decimal_math.h
 *
 *  Created on: 04.03.2017
 *      Author: philipp
 */

#ifndef INC_FIXED_DECIMAL_MATH_H_
#define INC_FIXED_DECIMAL_MATH_H_

#include <stdint.h>
#include <stdbool.h>
#include "usart.h"

typedef uint_fast32_t fixed_decimal_t;
typedef uint_fast16_t fixed_decimal_half_t;

bool fixed_dec_add (fixed_decimal_t augend, fixed_decimal_t addend, fixed_decimal_t* ergebnis);
bool fixed_dec_subtract (fixed_decimal_t minuend, fixed_decimal_t subtrahend, fixed_decimal_t* ergebnis);
bool fixed_dec_multiply (fixed_decimal_t multiplikator, fixed_decimal_t multiplikant, fixed_decimal_t* ergebnis);
bool fixed_dec_divide (fixed_decimal_t divident, fixed_decimal_t divisor, fixed_decimal_t* ergebnis);
bool fixed_dec_radic (fixed_decimal_t radikand, fixed_decimal_t* ergebnis);

void fixed_dec_test(uint32_t usart);

#endif /* INC_FIXED_DECIMAL_MATH_H_ */
