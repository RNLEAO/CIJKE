#include "headfile.h"

#define NEGATIVE_PRESSURE_PWM_PIN      P3_3
#define NEGATIVE_PRESSURE_PWM_OUT      P33

uint8 negative_pressure_duty_percent = NEGATIVE_PRESSURE_DEFAULT_DUTY_PERCENT;
uint16 negative_pressure_pulse_ms = NEGATIVE_PRESSURE_DEFAULT_PULSE_MS;
uint8 negative_pressure_last_result = 0;

static uint8 negative_pressure_clamp_duty(uint8 duty_percent)
{
    if(0u == duty_percent)
    {
        return 0u;
    }
    if(5u > duty_percent)
    {
        return 5u;
    }
    if(10u < duty_percent)
    {
        return 10u;
    }
    return duty_percent;
}

static uint16 negative_pressure_clamp_pulse_ms(uint16 pulse_ms)
{
    if(100u > pulse_ms)
    {
        return 100u;
    }
    if(300u < pulse_ms)
    {
        return 300u;
    }
    return pulse_ms;
}

void negative_pressure_off(void)
{
    NEGATIVE_PRESSURE_PWM_OUT = 0;
}

void negative_pressure_init(void)
{
    negative_pressure_off();
    gpio_mode(NEGATIVE_PRESSURE_PWM_PIN, GPO_PP);
    negative_pressure_off();
}

uint8 negative_pressure_fire_once(void)
{
    uint8 duty_percent;
    uint16 pulse_ms;
    uint16 cycles;
    uint16 index;
    uint32 high_time_us;
    uint32 low_time_us;

    if(pwm_state != 0)
    {
        negative_pressure_off();
        negative_pressure_last_result = 2;
        return 0u;
    }

    duty_percent = negative_pressure_clamp_duty(negative_pressure_duty_percent);
    pulse_ms = negative_pressure_clamp_pulse_ms(negative_pressure_pulse_ms);

    negative_pressure_duty_percent = duty_percent;
    negative_pressure_pulse_ms = pulse_ms;

    if(0u == duty_percent)
    {
        negative_pressure_off();
        negative_pressure_last_result = 0;
        return 0u;
    }

    cycles = (uint16)(((uint32)pulse_ms * 1000u) / NEGATIVE_PRESSURE_PWM_PERIOD_US);
    high_time_us = ((uint32)NEGATIVE_PRESSURE_PWM_PERIOD_US * duty_percent) / 100u;
    low_time_us = NEGATIVE_PRESSURE_PWM_PERIOD_US - high_time_us;

    negative_pressure_last_result = 3;
    for(index = 0u; index < cycles; index++)
    {
        NEGATIVE_PRESSURE_PWM_OUT = 1;
        delay_us(high_time_us);
        NEGATIVE_PRESSURE_PWM_OUT = 0;
        delay_us(low_time_us);
    }

    negative_pressure_off();
    negative_pressure_last_result = 1;
    return 1u;
}
