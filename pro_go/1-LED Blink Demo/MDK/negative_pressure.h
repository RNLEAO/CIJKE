#ifndef _NEGATIVE_PRESSURE_H
#define _NEGATIVE_PRESSURE_H

#include "headfile.h"

#define NEGATIVE_PRESSURE_DEFAULT_DUTY_PERCENT   5u
#define NEGATIVE_PRESSURE_DEFAULT_PULSE_MS       100u
#define NEGATIVE_PRESSURE_PWM_PERIOD_US          10000UL

extern uint8 negative_pressure_duty_percent;
extern uint16 negative_pressure_pulse_ms;
extern uint8 negative_pressure_last_result;

void negative_pressure_init(void);
void negative_pressure_off(void);
uint8 negative_pressure_fire_once(void);

#endif
