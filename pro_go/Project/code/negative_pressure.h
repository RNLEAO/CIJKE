#ifndef CIJIANKE_NEGATIVE_PRESSURE_H
#define CIJIANKE_NEGATIVE_PRESSURE_H

#include "common.h"

#define NEGATIVE_PRESSURE_LOAD_TEST_BUILD_ENABLE            0
#define NEGATIVE_PRESSURE_LOAD_TEST_PHYSICAL_OUTPUT_ENABLE  0
#define NEGATIVE_PRESSURE_DEFAULT_DUTY_PERCENT               5U
#define NEGATIVE_PRESSURE_PWM_FREQUENCY_HZ               17000UL

#define NEGATIVE_PRESSURE_AUTO_PREPARE_TICKS              10U
#define NEGATIVE_PRESSURE_AUTO_HOLD_TICKS                 10U
#define NEGATIVE_PRESSURE_AUTO_RELEASE_TICKS               1U
#define NEGATIVE_PRESSURE_AUTO_COOLDOWN_TICKS           6000U
#define NEGATIVE_PRESSURE_AUTO_MAX_SHOTS                   1U

typedef enum
{
    NEG_PRESS_AUTO_STATE_OFF = 0,
    NEG_PRESS_AUTO_STATE_PREPARE,
    NEG_PRESS_AUTO_STATE_HOLD,
    NEG_PRESS_AUTO_STATE_RELEASE,
    NEG_PRESS_AUTO_STATE_FAULT
} NegativePressureAutoState;

typedef enum
{
    NEG_PRESS_TRIGGER_NONE = 0,
    NEG_PRESS_TRIGGER_MENU_SIM,
    NEG_PRESS_TRIGGER_ELEMENT_RING,
    NEG_PRESS_TRIGGER_FAULT
} NegativePressureTrigger;

extern bit negative_pressure_auto_enabled;
extern bit negative_pressure_auto_armed;
extern bit negative_pressure_auto_request;
extern bit negative_pressure_auto_manual_request;
extern bit negative_pressure_auto_use_element_trigger;
extern bit negative_pressure_auto_fault_latched;
extern bit negative_pressure_auto_lockout;
extern bit negative_pressure_auto_cooldown_active;

extern uint8 negative_pressure_duty_percent;
extern uint8 negative_pressure_auto_target_duty_percent;
extern uint8 negative_pressure_auto_real_output_percent;
extern uint8 negative_pressure_auto_shot_count;
extern uint8 negative_pressure_auto_max_shots;
extern uint16 negative_pressure_auto_cooldown_ticks_remaining;
extern NegativePressureAutoState negative_pressure_auto_state;
extern NegativePressureTrigger negative_pressure_auto_trigger_source;

void negative_pressure_init(void);
void negative_pressure_off(void);
void negative_pressure_tick(void);
void negative_pressure_set_enabled(bit enabled);
void negative_pressure_set_armed(bit armed);
void negative_pressure_set_manual_request(bit request_on);
void negative_pressure_set_request_mode(bit use_element_trigger);
void negative_pressure_update_element_request(bit request_on);
void negative_pressure_set_fault(bit fault_on);
void negative_pressure_clear_lockout(void);
void negative_pressure_set_duty_percent(uint8 duty_percent);

#endif
