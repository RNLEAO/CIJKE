#ifndef _NEGATIVE_PRESSURE_H
#define _NEGATIVE_PRESSURE_H

#include "headfile.h"

/* Dedicated fan-load profile. P33 requires both independent compile gates. */
#define NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE       1u
#define NEGATIVE_PRESSURE_LOAD_TEST_BUILD_ENABLE         0u
#define NEGATIVE_PRESSURE_LOAD_TEST_PHYSICAL_OUTPUT_ENABLE 0u
#define NEGATIVE_PRESSURE_LOAD_TEST_FIXED_DUTY_PERCENT   5u

#define NEGATIVE_PRESSURE_DEFAULT_DUTY_PERCENT   NEGATIVE_PRESSURE_LOAD_TEST_FIXED_DUTY_PERCENT
#define NEGATIVE_PRESSURE_DEFAULT_PULSE_MS       100u
#define NEGATIVE_PRESSURE_PWM_PERIOD_US          10000UL

/* The state-machine tick is 5 ms: 10 + 10 ticks gives about 100 ms output. */
#define NEGATIVE_PRESSURE_AUTO_PREPARE_TICKS     10u
#define NEGATIVE_PRESSURE_AUTO_HOLD_TICKS        10u
#define NEGATIVE_PRESSURE_AUTO_RELEASE_TICKS     1u
#define NEGATIVE_PRESSURE_AUTO_COOLDOWN_TICKS    6000u
#define NEGATIVE_PRESSURE_AUTO_MAX_SHOTS         1u

typedef enum
{
    NEG_PRESS_AUTO_STATE_OFF = 0,
    NEG_PRESS_AUTO_STATE_PREPARE,
    NEG_PRESS_AUTO_STATE_HOLD,
    NEG_PRESS_AUTO_STATE_RELEASE,
    NEG_PRESS_AUTO_STATE_FAULT
} negative_pressure_auto_state_t;

typedef enum
{
    NEG_PRESS_TRIGGER_NONE = 0,
    NEG_PRESS_TRIGGER_MENU_SIM,
    NEG_PRESS_TRIGGER_CIRCLE,
    NEG_PRESS_TRIGGER_VBAT,
    NEG_PRESS_TRIGGER_ENCODER,
    NEG_PRESS_TRIGGER_FAULT
} negative_pressure_trigger_source_t;

typedef enum
{
    NEG_PRESS_STAGE2_MAP_NONE = 0,
    NEG_PRESS_STAGE2_MAP_P07,
    NEG_PRESS_STAGE2_MAP_FAN_Q2
} negative_pressure_stage2_map_t;

extern uint8 negative_pressure_duty_percent;
extern uint16 negative_pressure_pulse_ms;
extern uint8 negative_pressure_last_result;
extern bit negative_pressure_auto_enabled;
extern bit negative_pressure_auto_request;
extern bit negative_pressure_auto_fault_latched;
extern bit negative_pressure_auto_use_circle_trigger;
extern bit negative_pressure_auto_manual_request;
extern bit negative_pressure_auto_armed;
extern bit negative_pressure_auto_lockout;
extern bit negative_pressure_auto_cooldown_active;
extern bit negative_pressure_stage2_output_enable;
extern uint8 negative_pressure_auto_target_duty_percent;
extern uint8 negative_pressure_auto_real_output_percent;
extern uint8 negative_pressure_auto_shot_count;
extern uint8 negative_pressure_auto_max_shots;
extern uint8 negative_pressure_stage2_output_percent;
extern uint8 negative_pressure_stage2_prepared_percent;
extern uint16 negative_pressure_auto_cooldown_ticks;
extern uint16 negative_pressure_auto_cooldown_ticks_remaining;
extern negative_pressure_auto_state_t negative_pressure_auto_state;
extern negative_pressure_trigger_source_t negative_pressure_auto_trigger_source;
extern negative_pressure_trigger_source_t negative_pressure_auto_request_source;
extern negative_pressure_stage2_map_t negative_pressure_stage2_output_target;

void negative_pressure_init(void);
void negative_pressure_off(void);
uint8 negative_pressure_fire_once(void);
void negative_pressure_auto_reset(void);
void negative_pressure_auto_tick(void);
void negative_pressure_auto_set_enabled(bit enabled);
void negative_pressure_auto_set_armed(bit armed);
void negative_pressure_auto_set_request(bit request_on);
void negative_pressure_auto_set_fault(bit fault_on);
void negative_pressure_auto_set_request_mode(bit use_circle_trigger);
void negative_pressure_auto_update_request(bit request_on, negative_pressure_trigger_source_t source);
void negative_pressure_auto_clear_lockout(void);
void negative_pressure_stage2_set_enable(bit enabled);
void negative_pressure_stage2_set_target(negative_pressure_stage2_map_t target);

#endif
