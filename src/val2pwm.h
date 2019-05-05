#ifndef __VAL2PWM_H__
#define __VAL2PWM_H__

#include <stdint.h>

// converts an 0-255 intensity value to an equivalent  0-255 LED PWM value
uint8_t val2pwm(const uint8_t val);

#endif // __VAL2PWM_H__
