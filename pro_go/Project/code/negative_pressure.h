#ifndef CIJIANKE_NEGATIVE_PRESSURE_H
#define CIJIANKE_NEGATIVE_PRESSURE_H

#include "common.h"

#define NEGATIVE_PRESSURE_OUTPUT_ENABLE          1U
#define NEGATIVE_PRESSURE_BENCH_TRIGGER_ENABLE   1U
#define NEGATIVE_PRESSURE_PWM_FREQUENCY_HZ   17000UL
#define NEGATIVE_PRESSURE_DUTY_PERCENT           30U
#define NEGATIVE_PRESSURE_PWM_DUTY_VALUE       3000U
#define NEGATIVE_PRESSURE_PREPARE_TICKS         100U
#define NEGATIVE_PRESSURE_HOLD_TICKS            100U
#define NEGATIVE_PRESSURE_RELEASE_TICKS           1U
#define NEGATIVE_PRESSURE_COOLDOWN_TICKS       6000U

typedef enum
{
    NEGATIVE_PRESSURE_STATE_OFF = 0,
    NEGATIVE_PRESSURE_STATE_PREPARE,
    NEGATIVE_PRESSURE_STATE_HOLD,
    NEGATIVE_PRESSURE_STATE_RELEASE,
    NEGATIVE_PRESSURE_STATE_FAULT
} NegativePressureState;

extern bit negative_pressure_enabled;
extern bit negative_pressure_armed;
extern bit negative_pressure_fault_latched;
extern bit negative_pressure_lockout;
extern bit negative_pressure_cooldown_active;

extern uint8 negative_pressure_real_output_percent;
extern uint8 negative_pressure_shot_count;
extern vuint16 negative_pressure_cooldown_ticks_remaining;
extern NegativePressureState negative_pressure_state;

void negative_pressure_init(void);
void negative_pressure_tick(void);
void negative_pressure_set_enabled(bit enabled);
void negative_pressure_set_armed(bit armed);
uint8 negative_pressure_fire(void);
uint8 negative_pressure_reset(void);
const char *negative_pressure_state_text(void);

#endif
