#include "headfile.h"

#define NEGATIVE_PRESSURE_PWM_CHANNEL PWMB_CH3_P33

bit negative_pressure_auto_enabled = 0;
bit negative_pressure_auto_armed = 0;
bit negative_pressure_auto_request = 0;
bit negative_pressure_auto_manual_request = 0;
bit negative_pressure_auto_use_element_trigger = 0;
bit negative_pressure_auto_fault_latched = 0;
bit negative_pressure_auto_lockout = 0;
bit negative_pressure_auto_cooldown_active = 0;

uint8 negative_pressure_duty_percent = NEGATIVE_PRESSURE_DEFAULT_DUTY_PERCENT;
uint8 negative_pressure_auto_target_duty_percent = 0U;
uint8 negative_pressure_auto_real_output_percent = 0U;
uint8 negative_pressure_auto_shot_count = 0U;
uint8 negative_pressure_auto_max_shots = NEGATIVE_PRESSURE_AUTO_MAX_SHOTS;
uint16 negative_pressure_auto_cooldown_ticks_remaining = 0U;
NegativePressureAutoState negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_OFF;
NegativePressureTrigger negative_pressure_auto_trigger_source = NEG_PRESS_TRIGGER_NONE;

static uint16 negative_pressure_auto_state_ticks = 0U;
static bit negative_pressure_auto_request_last = 0;
static NegativePressureTrigger negative_pressure_auto_request_source = NEG_PRESS_TRIGGER_NONE;

static uint8 negative_pressure_clamp_duty(uint8 duty_percent)
{
    if (duty_percent < 5U) return 5U;
    if (duty_percent > 10U) return 10U;
    return duty_percent;
}

void negative_pressure_off(void)
{
#if NEGATIVE_PRESSURE_LOAD_TEST_BUILD_ENABLE
    pwm_set_duty(NEGATIVE_PRESSURE_PWM_CHANNEL, 0U);
#endif
    negative_pressure_auto_real_output_percent = 0U;
}

static void negative_pressure_reset_state(void)
{
    negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_OFF;
    negative_pressure_auto_state_ticks = 0U;
    negative_pressure_auto_target_duty_percent = 0U;
    negative_pressure_auto_trigger_source = NEG_PRESS_TRIGGER_NONE;
    negative_pressure_off();
}

static void negative_pressure_apply_output(void)
{
#if NEGATIVE_PRESSURE_LOAD_TEST_BUILD_ENABLE && \
    NEGATIVE_PRESSURE_LOAD_TEST_PHYSICAL_OUTPUT_ENABLE
    uint32 duty_value;

    if (negative_pressure_auto_enabled
        && negative_pressure_auto_armed
        && !negative_pressure_auto_fault_latched
        && !negative_pressure_auto_use_element_trigger
        && negative_pressure_auto_request_source == NEG_PRESS_TRIGGER_MENU_SIM
        && pwm_state == 0U
        && (negative_pressure_auto_state == NEG_PRESS_AUTO_STATE_PREPARE
            || negative_pressure_auto_state == NEG_PRESS_AUTO_STATE_HOLD))
    {
        negative_pressure_auto_real_output_percent =
            negative_pressure_clamp_duty(negative_pressure_duty_percent);
        duty_value = ((uint32)PWM_DUTY_MAX
                      * negative_pressure_auto_real_output_percent) / 100U;
        pwm_set_duty(NEGATIVE_PRESSURE_PWM_CHANNEL, duty_value);
        return;
    }
#endif

    negative_pressure_off();
}

void negative_pressure_init(void)
{
#if NEGATIVE_PRESSURE_LOAD_TEST_BUILD_ENABLE
    pwm_init(NEGATIVE_PRESSURE_PWM_CHANNEL,
             NEGATIVE_PRESSURE_PWM_FREQUENCY_HZ,
             0U);
#endif

    negative_pressure_auto_enabled = 0;
    negative_pressure_auto_armed = 0;
    negative_pressure_auto_request = 0;
    negative_pressure_auto_manual_request = 0;
    negative_pressure_auto_use_element_trigger = 0;
    negative_pressure_auto_fault_latched = 0;
    negative_pressure_auto_lockout = 0;
    negative_pressure_auto_cooldown_active = 0;
    negative_pressure_auto_request_last = 0;
    negative_pressure_auto_request_source = NEG_PRESS_TRIGGER_NONE;
    negative_pressure_duty_percent = NEGATIVE_PRESSURE_DEFAULT_DUTY_PERCENT;
    negative_pressure_auto_shot_count = 0U;
    negative_pressure_auto_max_shots = NEGATIVE_PRESSURE_AUTO_MAX_SHOTS;
    negative_pressure_auto_cooldown_ticks_remaining = 0U;
    negative_pressure_reset_state();
}

void negative_pressure_set_enabled(bit enabled)
{
    negative_pressure_auto_enabled = enabled;
    if (!enabled)
    {
        negative_pressure_auto_armed = 0;
        negative_pressure_auto_request = 0;
        negative_pressure_auto_manual_request = 0;
        negative_pressure_auto_request_last = 0;
        negative_pressure_auto_request_source = NEG_PRESS_TRIGGER_NONE;
        negative_pressure_reset_state();
    }
}

void negative_pressure_set_armed(bit armed)
{
    if (armed
        && negative_pressure_auto_enabled
        && !negative_pressure_auto_fault_latched
        && !negative_pressure_auto_lockout
        && !negative_pressure_auto_cooldown_active)
    {
        negative_pressure_auto_armed = 1;
    }
    else
    {
        negative_pressure_auto_armed = 0;
        negative_pressure_reset_state();
    }
}

void negative_pressure_set_manual_request(bit request_on)
{
    negative_pressure_auto_manual_request = request_on;
    if (!negative_pressure_auto_use_element_trigger)
    {
        negative_pressure_auto_request = request_on;
        negative_pressure_auto_request_source = request_on
            ? NEG_PRESS_TRIGGER_MENU_SIM
            : NEG_PRESS_TRIGGER_NONE;
    }
}

void negative_pressure_set_request_mode(bit use_element_trigger)
{
    negative_pressure_auto_use_element_trigger = use_element_trigger;
    negative_pressure_auto_request = 0;
    negative_pressure_auto_manual_request = 0;
    negative_pressure_auto_request_last = 0;
    negative_pressure_auto_request_source = NEG_PRESS_TRIGGER_NONE;
    negative_pressure_reset_state();
}

void negative_pressure_update_element_request(bit request_on)
{
    if (negative_pressure_auto_use_element_trigger)
    {
        negative_pressure_auto_request = request_on;
        negative_pressure_auto_request_source = request_on
            ? NEG_PRESS_TRIGGER_ELEMENT_RING
            : NEG_PRESS_TRIGGER_NONE;
    }
}

void negative_pressure_set_fault(bit fault_on)
{
    negative_pressure_auto_fault_latched = fault_on;
    if (fault_on)
    {
        negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_FAULT;
        negative_pressure_auto_state_ticks = 0U;
        negative_pressure_auto_target_duty_percent = 0U;
        negative_pressure_auto_trigger_source = NEG_PRESS_TRIGGER_FAULT;
        negative_pressure_off();
    }
    else
    {
        negative_pressure_reset_state();
    }
}

void negative_pressure_clear_lockout(void)
{
    if (negative_pressure_auto_fault_latched
        || negative_pressure_auto_cooldown_active)
    {
        return;
    }

    negative_pressure_auto_lockout = 0;
    negative_pressure_auto_shot_count = 0U;
    negative_pressure_auto_armed = 0;
    negative_pressure_reset_state();
}

void negative_pressure_set_duty_percent(uint8 duty_percent)
{
    negative_pressure_duty_percent = negative_pressure_clamp_duty(duty_percent);
}

static void negative_pressure_start_cycle(void)
{
    negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_PREPARE;
    negative_pressure_auto_state_ticks = 0U;
    negative_pressure_auto_target_duty_percent =
        negative_pressure_clamp_duty(negative_pressure_duty_percent);
    negative_pressure_auto_trigger_source = negative_pressure_auto_request_source;
    negative_pressure_auto_shot_count++;
}

static void negative_pressure_finish_cycle(void)
{
    negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_OFF;
    negative_pressure_auto_state_ticks = 0U;
    negative_pressure_auto_target_duty_percent = 0U;
    negative_pressure_auto_cooldown_active = 1;
    negative_pressure_auto_cooldown_ticks_remaining =
        NEGATIVE_PRESSURE_AUTO_COOLDOWN_TICKS;

    if (!negative_pressure_auto_use_element_trigger)
    {
        negative_pressure_auto_request = 0;
        negative_pressure_auto_manual_request = 0;
        negative_pressure_auto_request_source = NEG_PRESS_TRIGGER_NONE;
    }

    if (negative_pressure_auto_shot_count >= negative_pressure_auto_max_shots)
    {
        negative_pressure_auto_lockout = 1;
        negative_pressure_auto_armed = 0;
    }
}

void negative_pressure_tick(void)
{
    bit request_rising_edge;

    if (negative_pressure_auto_cooldown_active)
    {
        if (negative_pressure_auto_cooldown_ticks_remaining > 0U)
        {
            negative_pressure_auto_cooldown_ticks_remaining--;
        }
        if (negative_pressure_auto_cooldown_ticks_remaining == 0U)
        {
            negative_pressure_auto_cooldown_active = 0;
        }
    }

    request_rising_edge = negative_pressure_auto_request
        && !negative_pressure_auto_request_last;
    negative_pressure_auto_request_last = negative_pressure_auto_request;

    if (!negative_pressure_auto_enabled)
    {
        negative_pressure_reset_state();
        return;
    }

    if (negative_pressure_auto_fault_latched)
    {
        negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_FAULT;
        negative_pressure_auto_target_duty_percent = 0U;
        negative_pressure_auto_trigger_source = NEG_PRESS_TRIGGER_FAULT;
        negative_pressure_apply_output();
        return;
    }

    switch (negative_pressure_auto_state)
    {
        case NEG_PRESS_AUTO_STATE_OFF:
            negative_pressure_auto_target_duty_percent = 0U;
            if (request_rising_edge
                && negative_pressure_auto_armed
                && !negative_pressure_auto_lockout
                && !negative_pressure_auto_cooldown_active)
            {
                negative_pressure_start_cycle();
            }
            break;

        case NEG_PRESS_AUTO_STATE_PREPARE:
            negative_pressure_auto_target_duty_percent =
                negative_pressure_clamp_duty(negative_pressure_duty_percent);
            if (!negative_pressure_auto_request)
            {
                negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_RELEASE;
                negative_pressure_auto_state_ticks = 0U;
            }
            else if (negative_pressure_auto_state_ticks
                     >= NEGATIVE_PRESSURE_AUTO_PREPARE_TICKS)
            {
                negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_HOLD;
                negative_pressure_auto_state_ticks = 0U;
            }
            break;

        case NEG_PRESS_AUTO_STATE_HOLD:
            negative_pressure_auto_target_duty_percent =
                negative_pressure_clamp_duty(negative_pressure_duty_percent);
            if (!negative_pressure_auto_request
                || negative_pressure_auto_state_ticks
                   >= NEGATIVE_PRESSURE_AUTO_HOLD_TICKS)
            {
                negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_RELEASE;
                negative_pressure_auto_state_ticks = 0U;
            }
            break;

        case NEG_PRESS_AUTO_STATE_RELEASE:
            negative_pressure_auto_target_duty_percent = 0U;
            if (negative_pressure_auto_state_ticks
                >= NEGATIVE_PRESSURE_AUTO_RELEASE_TICKS)
            {
                negative_pressure_finish_cycle();
            }
            break;

        case NEG_PRESS_AUTO_STATE_FAULT:
        default:
            negative_pressure_auto_target_duty_percent = 0U;
            break;
    }

    negative_pressure_apply_output();
    if (negative_pressure_auto_state != NEG_PRESS_AUTO_STATE_OFF
        && negative_pressure_auto_state != NEG_PRESS_AUTO_STATE_FAULT)
    {
        negative_pressure_auto_state_ticks++;
    }
}
