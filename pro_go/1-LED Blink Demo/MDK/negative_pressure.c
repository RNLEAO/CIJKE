#include "headfile.h"

#define NEGATIVE_PRESSURE_PWM_FREQUENCY_HZ 17000u
#define NEGATIVE_PRESSURE_PWM_CHANNEL  PWMB_CH3_P33
#define NEGATIVE_PRESSURE_STAGE2_P07_PIN P0_7
#define NEGATIVE_PRESSURE_STAGE2_P07_OUT P07

uint8 negative_pressure_duty_percent = NEGATIVE_PRESSURE_DEFAULT_DUTY_PERCENT;
uint16 negative_pressure_pulse_ms = NEGATIVE_PRESSURE_DEFAULT_PULSE_MS;
uint8 negative_pressure_last_result = 0;
bit negative_pressure_auto_enabled = 0;
bit negative_pressure_auto_request = 0;
bit negative_pressure_auto_fault_latched = 0;
bit negative_pressure_auto_use_circle_trigger = 0;
bit negative_pressure_auto_manual_request = 0;
bit negative_pressure_auto_armed = 0;
bit negative_pressure_auto_lockout = 0;
bit negative_pressure_auto_cooldown_active = 0;
bit negative_pressure_stage2_output_enable = 0;
uint8 negative_pressure_auto_target_duty_percent = 0;
uint8 negative_pressure_auto_real_output_percent = 0;
uint8 negative_pressure_auto_shot_count = 0;
uint8 negative_pressure_auto_max_shots = NEGATIVE_PRESSURE_AUTO_MAX_SHOTS;
uint8 negative_pressure_stage2_output_percent = NEGATIVE_PRESSURE_LOAD_TEST_FIXED_DUTY_PERCENT;
uint8 negative_pressure_stage2_prepared_percent = 0;
uint16 negative_pressure_auto_cooldown_ticks = NEGATIVE_PRESSURE_AUTO_COOLDOWN_TICKS;
uint16 negative_pressure_auto_cooldown_ticks_remaining = 0;
negative_pressure_auto_state_t negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_OFF;
negative_pressure_trigger_source_t negative_pressure_auto_trigger_source = NEG_PRESS_TRIGGER_NONE;
negative_pressure_trigger_source_t negative_pressure_auto_request_source = NEG_PRESS_TRIGGER_NONE;
negative_pressure_stage2_map_t negative_pressure_stage2_output_target = NEG_PRESS_STAGE2_MAP_NONE;

static uint16 negative_pressure_auto_state_ticks = 0;
static bit negative_pressure_auto_request_last = 0;

static uint8 negative_pressure_clamp_duty(uint8 duty_percent);
static void negative_pressure_auto_reset_protection(void);

static void negative_pressure_stage2_outputs_off(void)
{
    NEGATIVE_PRESSURE_STAGE2_P07_OUT = 0;
}

static void negative_pressure_stage2_reset_controls(void)
{
    negative_pressure_stage2_output_enable = 0;
    negative_pressure_stage2_prepared_percent = 0;
    negative_pressure_stage2_output_target = NEG_PRESS_STAGE2_MAP_NONE;
    negative_pressure_stage2_outputs_off();
    negative_pressure_off();
}

static void negative_pressure_stage2_apply_output(void)
{
    if(negative_pressure_stage2_output_enable &&
       negative_pressure_stage2_output_target == NEG_PRESS_STAGE2_MAP_P07 &&
       negative_pressure_stage2_prepared_percent > 0u &&
       (negative_pressure_auto_state == NEG_PRESS_AUTO_STATE_PREPARE ||
        negative_pressure_auto_state == NEG_PRESS_AUTO_STATE_HOLD))
    {
        NEGATIVE_PRESSURE_STAGE2_P07_OUT = 1;
    }
    else
    {
        negative_pressure_stage2_outputs_off();
    }
}

#if NEGATIVE_PRESSURE_LOAD_TEST_BUILD_ENABLE && \
    NEGATIVE_PRESSURE_LOAD_TEST_PHYSICAL_OUTPUT_ENABLE
static uint8 negative_pressure_fan_output_allowed(void)
{
    return pwm_state == 0u &&
           negative_pressure_auto_enabled &&
           negative_pressure_auto_armed &&
           !negative_pressure_auto_fault_latched &&
           !negative_pressure_auto_use_circle_trigger &&
           negative_pressure_auto_request &&
           negative_pressure_auto_request_source == NEG_PRESS_TRIGGER_MENU_SIM &&
           negative_pressure_stage2_output_enable &&
           negative_pressure_stage2_output_target == NEG_PRESS_STAGE2_MAP_FAN_Q2 &&
           negative_pressure_stage2_prepared_percent > 0u &&
           (negative_pressure_auto_state == NEG_PRESS_AUTO_STATE_PREPARE ||
            negative_pressure_auto_state == NEG_PRESS_AUTO_STATE_HOLD);
}
#endif

static void negative_pressure_apply_fan_output(void)
{
#if NEGATIVE_PRESSURE_LOAD_TEST_BUILD_ENABLE && \
    NEGATIVE_PRESSURE_LOAD_TEST_PHYSICAL_OUTPUT_ENABLE
    uint8 duty_percent;
    uint32 pwm_duty_value;

    if(!negative_pressure_fan_output_allowed())
    {
        negative_pressure_off();
        return;
    }

    duty_percent = negative_pressure_clamp_duty(negative_pressure_stage2_prepared_percent);
    pwm_duty_value = ((uint32)PWM_DUTY_MAX * duty_percent) / 100u;

    negative_pressure_auto_real_output_percent = duty_percent;
    pwm_duty(NEGATIVE_PRESSURE_PWM_CHANNEL, pwm_duty_value);
#else
    negative_pressure_off();
#endif
}

static uint8 negative_pressure_clamp_duty(uint8 duty_percent)
{
    if(0u == duty_percent)
    {
        return 0u;
    }
#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
    return NEGATIVE_PRESSURE_LOAD_TEST_FIXED_DUTY_PERCENT;
#else
    if(5u > duty_percent)
    {
        return 5u;
    }
    if(10u < duty_percent)
    {
        return 10u;
    }
    return duty_percent;
#endif
}

#if !NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
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
#endif

void negative_pressure_off(void)
{
    pwm_duty(NEGATIVE_PRESSURE_PWM_CHANNEL, 0u);
    negative_pressure_auto_real_output_percent = 0;
}

void negative_pressure_init(void)
{
    gpio_mode(NEGATIVE_PRESSURE_STAGE2_P07_PIN, GPO_PP);
    negative_pressure_stage2_outputs_off();
    pwm_init(NEGATIVE_PRESSURE_PWM_CHANNEL, NEGATIVE_PRESSURE_PWM_FREQUENCY_HZ, 0u);
    negative_pressure_off();

    negative_pressure_duty_percent = NEGATIVE_PRESSURE_LOAD_TEST_FIXED_DUTY_PERCENT;
    negative_pressure_pulse_ms = NEGATIVE_PRESSURE_DEFAULT_PULSE_MS;
    negative_pressure_auto_enabled = 0;
    negative_pressure_auto_request = 0;
    negative_pressure_auto_fault_latched = 0;
    negative_pressure_auto_use_circle_trigger = 0;
    negative_pressure_auto_manual_request = 0;
    negative_pressure_auto_request_source = NEG_PRESS_TRIGGER_NONE;
    negative_pressure_auto_max_shots = NEGATIVE_PRESSURE_AUTO_MAX_SHOTS;
    negative_pressure_stage2_output_percent = NEGATIVE_PRESSURE_LOAD_TEST_FIXED_DUTY_PERCENT;
    negative_pressure_auto_cooldown_ticks = NEGATIVE_PRESSURE_AUTO_COOLDOWN_TICKS;
    negative_pressure_auto_reset_protection();
    negative_pressure_auto_reset();
}

void negative_pressure_auto_reset(void)
{
    negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_OFF;
    negative_pressure_auto_trigger_source = NEG_PRESS_TRIGGER_NONE;
    negative_pressure_auto_target_duty_percent = 0;
    negative_pressure_auto_real_output_percent = 0;
    negative_pressure_stage2_prepared_percent = 0;
    negative_pressure_auto_state_ticks = 0;
    negative_pressure_stage2_outputs_off();
    negative_pressure_off();
}

static void negative_pressure_auto_reset_protection(void)
{
    negative_pressure_auto_armed = 0;
    negative_pressure_auto_lockout = 0;
    negative_pressure_auto_cooldown_active = 0;
    negative_pressure_auto_shot_count = 0;
    negative_pressure_auto_cooldown_ticks_remaining = 0;
    negative_pressure_auto_request_last = 0;
    negative_pressure_stage2_reset_controls();
}

void negative_pressure_auto_set_enabled(bit enabled)
{
    negative_pressure_auto_enabled = enabled;
    if(!enabled)
    {
        negative_pressure_auto_request = 0;
        negative_pressure_auto_manual_request = 0;
        negative_pressure_auto_request_source = NEG_PRESS_TRIGGER_NONE;
        negative_pressure_auto_fault_latched = 0;
#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
        negative_pressure_auto_armed = 0;
        negative_pressure_auto_request_last = 0;
        negative_pressure_stage2_reset_controls();
#else
        negative_pressure_auto_reset_protection();
#endif
        negative_pressure_auto_reset();
    }
}

void negative_pressure_auto_set_armed(bit armed)
{
    negative_pressure_auto_armed = armed;
    if(!armed)
    {
        negative_pressure_auto_reset();
    }
}

void negative_pressure_auto_set_request(bit request_on)
{
    negative_pressure_auto_manual_request = request_on;
    if(!negative_pressure_auto_use_circle_trigger)
    {
        negative_pressure_auto_request = request_on;
        negative_pressure_auto_request_source = request_on ? NEG_PRESS_TRIGGER_MENU_SIM : NEG_PRESS_TRIGGER_NONE;
    }
}

void negative_pressure_auto_set_fault(bit fault_on)
{
    negative_pressure_auto_fault_latched = fault_on;
    if(fault_on)
    {
        negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_FAULT;
        negative_pressure_auto_trigger_source = NEG_PRESS_TRIGGER_FAULT;
        negative_pressure_auto_target_duty_percent = 0;
        negative_pressure_auto_state_ticks = 0;
#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
        negative_pressure_stage2_reset_controls();
#else
        negative_pressure_stage2_outputs_off();
        negative_pressure_off();
#endif
    }
    else if(negative_pressure_auto_state == NEG_PRESS_AUTO_STATE_FAULT)
    {
        negative_pressure_auto_reset();
    }
}

static void negative_pressure_auto_apply_safe_output(void)
{
#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
    negative_pressure_duty_percent = NEGATIVE_PRESSURE_LOAD_TEST_FIXED_DUTY_PERCENT;
    negative_pressure_stage2_output_percent = NEGATIVE_PRESSURE_LOAD_TEST_FIXED_DUTY_PERCENT;
#endif
    if(negative_pressure_stage2_output_enable &&
       negative_pressure_stage2_output_target != NEG_PRESS_STAGE2_MAP_NONE &&
       (negative_pressure_auto_state == NEG_PRESS_AUTO_STATE_PREPARE ||
        negative_pressure_auto_state == NEG_PRESS_AUTO_STATE_HOLD))
    {
        negative_pressure_stage2_prepared_percent =
            negative_pressure_clamp_duty(negative_pressure_stage2_output_percent);
    }
    else
    {
        negative_pressure_stage2_prepared_percent = 0;
    }

    negative_pressure_auto_real_output_percent = 0;
    negative_pressure_off();
    negative_pressure_stage2_apply_output();
    negative_pressure_apply_fan_output();
}

static void negative_pressure_auto_start_cycle(negative_pressure_trigger_source_t active_source)
{
#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
    negative_pressure_duty_percent = NEGATIVE_PRESSURE_LOAD_TEST_FIXED_DUTY_PERCENT;
    negative_pressure_stage2_output_percent = NEGATIVE_PRESSURE_LOAD_TEST_FIXED_DUTY_PERCENT;
#endif
    negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_PREPARE;
    negative_pressure_auto_state_ticks = 0;
    negative_pressure_auto_trigger_source = active_source;
    negative_pressure_auto_target_duty_percent = negative_pressure_clamp_duty(negative_pressure_duty_percent);
    negative_pressure_auto_shot_count++;
}

static void negative_pressure_auto_finish_cycle(void)
{
    negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_OFF;
    negative_pressure_auto_state_ticks = 0;
    negative_pressure_auto_target_duty_percent = 0;
    negative_pressure_auto_cooldown_active = 1;
    negative_pressure_auto_cooldown_ticks_remaining = negative_pressure_auto_cooldown_ticks;
#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
    negative_pressure_auto_request = 0;
    negative_pressure_auto_manual_request = 0;
    negative_pressure_auto_request_source = NEG_PRESS_TRIGGER_NONE;
    negative_pressure_stage2_reset_controls();
#endif
    if(negative_pressure_auto_shot_count >= negative_pressure_auto_max_shots)
    {
        negative_pressure_auto_lockout = 1;
        negative_pressure_auto_armed = 0;
    }
}

void negative_pressure_auto_set_request_mode(bit use_circle_trigger)
{
#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
    negative_pressure_auto_use_circle_trigger = 0;
    if(use_circle_trigger)
    {
        negative_pressure_auto_request = 0;
        negative_pressure_auto_request_source = NEG_PRESS_TRIGGER_NONE;
    }
    else
    {
        negative_pressure_auto_request = negative_pressure_auto_manual_request;
        negative_pressure_auto_request_source = negative_pressure_auto_manual_request ? NEG_PRESS_TRIGGER_MENU_SIM : NEG_PRESS_TRIGGER_NONE;
    }
#else
    negative_pressure_auto_use_circle_trigger = use_circle_trigger;
    if(use_circle_trigger)
    {
        negative_pressure_auto_request = 0;
        negative_pressure_auto_request_source = NEG_PRESS_TRIGGER_NONE;
    }
    else
    {
        negative_pressure_auto_request = negative_pressure_auto_manual_request;
        negative_pressure_auto_request_source = negative_pressure_auto_manual_request ? NEG_PRESS_TRIGGER_MENU_SIM : NEG_PRESS_TRIGGER_NONE;
    }
#endif
}

void negative_pressure_auto_update_request(bit request_on, negative_pressure_trigger_source_t source)
{
#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
    negative_pressure_auto_use_circle_trigger = 0;
    if(request_on || source != NEG_PRESS_TRIGGER_NONE)
    {
        negative_pressure_auto_request = negative_pressure_auto_manual_request;
        negative_pressure_auto_request_source = negative_pressure_auto_manual_request ? NEG_PRESS_TRIGGER_MENU_SIM : NEG_PRESS_TRIGGER_NONE;
    }
#else
    if(negative_pressure_auto_use_circle_trigger)
    {
        negative_pressure_auto_request = request_on;
        negative_pressure_auto_request_source = request_on ? source : NEG_PRESS_TRIGGER_NONE;
    }
#endif
}

void negative_pressure_auto_clear_lockout(void)
{
#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
    if(negative_pressure_auto_cooldown_active ||
       negative_pressure_auto_cooldown_ticks_remaining > 0u)
    {
        negative_pressure_auto_armed = 0;
        negative_pressure_auto_reset();
        return;
    }
#endif
    negative_pressure_auto_lockout = 0;
    negative_pressure_auto_cooldown_active = 0;
    negative_pressure_auto_cooldown_ticks_remaining = 0;
    negative_pressure_auto_shot_count = 0;
    if(negative_pressure_auto_state != NEG_PRESS_AUTO_STATE_FAULT)
    {
        negative_pressure_auto_reset();
    }
}

void negative_pressure_stage2_set_enable(bit enabled)
{
#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
    negative_pressure_stage2_output_percent = NEGATIVE_PRESSURE_LOAD_TEST_FIXED_DUTY_PERCENT;
#endif
    negative_pressure_stage2_output_enable = enabled;
    if(!enabled)
    {
        negative_pressure_stage2_prepared_percent = 0;
        negative_pressure_stage2_outputs_off();
        negative_pressure_off();
    }
}

void negative_pressure_stage2_set_target(negative_pressure_stage2_map_t target)
{
#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
    negative_pressure_stage2_output_percent = NEGATIVE_PRESSURE_LOAD_TEST_FIXED_DUTY_PERCENT;
#endif
    negative_pressure_stage2_outputs_off();
    negative_pressure_off();
    negative_pressure_stage2_output_target = target;
    if(target == NEG_PRESS_STAGE2_MAP_NONE)
    {
        negative_pressure_stage2_output_enable = 0;
        negative_pressure_stage2_prepared_percent = 0;
    }
}

void negative_pressure_auto_tick(void)
{
    negative_pressure_trigger_source_t active_source;
    bit request_rising_edge;

#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
    negative_pressure_auto_set_request_mode(0);
    if(pwm_state != 0u)
    {
        negative_pressure_auto_request = 0;
        negative_pressure_auto_manual_request = 0;
        negative_pressure_auto_request_source = NEG_PRESS_TRIGGER_NONE;
        negative_pressure_auto_request_last = 0;
        negative_pressure_auto_set_fault(1);
        negative_pressure_auto_apply_safe_output();
        return;
    }
#endif

    active_source =
        (negative_pressure_auto_request_source != NEG_PRESS_TRIGGER_NONE) ?
        negative_pressure_auto_request_source :
        negative_pressure_auto_trigger_source;
    request_rising_edge = negative_pressure_auto_request && !negative_pressure_auto_request_last;

    negative_pressure_auto_request_last = negative_pressure_auto_request;

    if(negative_pressure_auto_cooldown_active)
    {
        if(negative_pressure_auto_cooldown_ticks_remaining > 0u)
        {
            negative_pressure_auto_cooldown_ticks_remaining--;
        }
        if(0u == negative_pressure_auto_cooldown_ticks_remaining)
        {
            negative_pressure_auto_cooldown_active = 0;
        }
    }

    if(!negative_pressure_auto_enabled)
    {
        negative_pressure_auto_reset();
        return;
    }

    if(negative_pressure_auto_fault_latched)
    {
        negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_FAULT;
        negative_pressure_auto_trigger_source = NEG_PRESS_TRIGGER_FAULT;
        negative_pressure_auto_target_duty_percent = 0;
        negative_pressure_auto_state_ticks = 0;
#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
        negative_pressure_stage2_reset_controls();
#endif
        negative_pressure_auto_apply_safe_output();
        return;
    }

    switch(negative_pressure_auto_state)
    {
        case NEG_PRESS_AUTO_STATE_OFF:
            negative_pressure_auto_target_duty_percent = 0;
            negative_pressure_auto_trigger_source = negative_pressure_auto_request ? active_source : NEG_PRESS_TRIGGER_NONE;
            if(request_rising_edge &&
               negative_pressure_auto_armed &&
               !negative_pressure_auto_lockout &&
               !negative_pressure_auto_cooldown_active)
            {
                negative_pressure_auto_start_cycle(active_source);
            }
            break;

        case NEG_PRESS_AUTO_STATE_PREPARE:
            negative_pressure_auto_target_duty_percent = negative_pressure_clamp_duty(negative_pressure_duty_percent);
            negative_pressure_auto_trigger_source = active_source;
            if(!negative_pressure_auto_request)
            {
                negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_RELEASE;
                negative_pressure_auto_state_ticks = 0;
            }
            else if(negative_pressure_auto_state_ticks >= NEGATIVE_PRESSURE_AUTO_PREPARE_TICKS)
            {
                negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_HOLD;
                negative_pressure_auto_state_ticks = 0;
            }
            break;

        case NEG_PRESS_AUTO_STATE_HOLD:
            negative_pressure_auto_target_duty_percent = negative_pressure_clamp_duty(negative_pressure_duty_percent);
            negative_pressure_auto_trigger_source = active_source;
            if(!negative_pressure_auto_request || negative_pressure_auto_state_ticks >= NEGATIVE_PRESSURE_AUTO_HOLD_TICKS)
            {
                negative_pressure_auto_state = NEG_PRESS_AUTO_STATE_RELEASE;
                negative_pressure_auto_state_ticks = 0;
            }
            break;

        case NEG_PRESS_AUTO_STATE_RELEASE:
            negative_pressure_auto_target_duty_percent = 0;
            negative_pressure_auto_trigger_source = active_source;
            if(negative_pressure_auto_state_ticks >= NEGATIVE_PRESSURE_AUTO_RELEASE_TICKS)
            {
                negative_pressure_auto_finish_cycle();
            }
            break;

        case NEG_PRESS_AUTO_STATE_FAULT:
        default:
            negative_pressure_auto_target_duty_percent = 0;
            negative_pressure_auto_trigger_source = NEG_PRESS_TRIGGER_FAULT;
            break;
    }

    negative_pressure_auto_apply_safe_output();
    negative_pressure_auto_state_ticks++;
}

uint8 negative_pressure_fire_once(void)
{
#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
    negative_pressure_off();
    negative_pressure_last_result = 4;
    return 0u;
#else
    uint8 duty_percent;
    uint16 pulse_ms;
    uint32 pwm_duty_value;

    if(!NEGATIVE_PRESSURE_LOAD_TEST_BUILD_ENABLE ||
       !NEGATIVE_PRESSURE_LOAD_TEST_PHYSICAL_OUTPUT_ENABLE)
    {
        negative_pressure_off();
        negative_pressure_last_result = 4;
        return 0u;
    }

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

    pwm_duty_value = ((uint32)PWM_DUTY_MAX * duty_percent) / 100u;

    negative_pressure_last_result = 3;
    pwm_duty(NEGATIVE_PRESSURE_PWM_CHANNEL, pwm_duty_value);
    delay_ms(pulse_ms);

    negative_pressure_off();
    negative_pressure_last_result = 1;
    return 1u;
#endif
}
