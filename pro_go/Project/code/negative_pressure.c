#include "headfile.h"

#define NEGATIVE_PRESSURE_PWM_CHANNEL PWMB_CH3_P33

#if PWM_DUTY_MAX != 10000U
#error Negative pressure PWM duty assumes the V3 0-to-10000 scale.
#endif

#if NEGATIVE_PRESSURE_PWM_DUTY_VALUE != 3000U
#error Negative pressure PWM duty must remain at the validated 30 percent value.
#endif

bit negative_pressure_enabled = 0;
bit negative_pressure_armed = 0;
bit negative_pressure_fault_latched = 0;
bit negative_pressure_lockout = 0;
bit negative_pressure_cooldown_active = 0;

uint8 negative_pressure_real_output_percent = 0U;
uint8 negative_pressure_shot_count = 0U;
vuint16 negative_pressure_cooldown_ticks_remaining = 0U;
NegativePressureState negative_pressure_state = NEGATIVE_PRESSURE_STATE_OFF;

static uint16 negative_pressure_state_ticks = 0U;

static void negative_pressure_output_off(void)
{
#if NEGATIVE_PRESSURE_OUTPUT_ENABLE
    pwm_set_duty(NEGATIVE_PRESSURE_PWM_CHANNEL, 0U);
#endif
    negative_pressure_real_output_percent = 0U;
}

static void negative_pressure_apply_output(void)
{
#if NEGATIVE_PRESSURE_OUTPUT_ENABLE
    if (negative_pressure_enabled
        && negative_pressure_armed
        && !negative_pressure_fault_latched
        && pwm_state == 0U
        && (negative_pressure_state == NEGATIVE_PRESSURE_STATE_PREPARE
            || negative_pressure_state == NEGATIVE_PRESSURE_STATE_HOLD))
    {
        pwm_set_duty(
            NEGATIVE_PRESSURE_PWM_CHANNEL,
            NEGATIVE_PRESSURE_PWM_DUTY_VALUE);
        negative_pressure_real_output_percent = NEGATIVE_PRESSURE_DUTY_PERCENT;
        return;
    }
#endif

    negative_pressure_output_off();
}

static void negative_pressure_stop_cycle(void)
{
    negative_pressure_state = NEGATIVE_PRESSURE_STATE_OFF;
    negative_pressure_state_ticks = 0U;
    negative_pressure_output_off();
}

static void negative_pressure_latch_fault(void)
{
    negative_pressure_fault_latched = 1;
    negative_pressure_armed = 0;
    negative_pressure_lockout = 1;
    negative_pressure_state = NEGATIVE_PRESSURE_STATE_FAULT;
    negative_pressure_state_ticks = 0U;
    negative_pressure_output_off();
}

void negative_pressure_init(void)
{
#if NEGATIVE_PRESSURE_OUTPUT_ENABLE
    pwm_init(NEGATIVE_PRESSURE_PWM_CHANNEL,
             NEGATIVE_PRESSURE_PWM_FREQUENCY_HZ,
             0U);
#endif

    negative_pressure_enabled = 0;
    negative_pressure_armed = 0;
    negative_pressure_fault_latched = 0;
    negative_pressure_lockout = 0;
    negative_pressure_cooldown_active = 0;
    negative_pressure_real_output_percent = 0U;
    negative_pressure_shot_count = 0U;
    negative_pressure_cooldown_ticks_remaining = 0U;
    negative_pressure_stop_cycle();
}

void negative_pressure_set_enabled(bit enabled)
{
    negative_pressure_enabled = enabled ? 1 : 0;
    if (!negative_pressure_enabled)
    {
        negative_pressure_armed = 0;
        negative_pressure_stop_cycle();
    }
}

void negative_pressure_set_armed(bit armed)
{
    if (armed
        && negative_pressure_enabled
        && !negative_pressure_fault_latched
        && !negative_pressure_lockout
        && !negative_pressure_cooldown_active
        && pwm_state == 0U)
    {
        negative_pressure_armed = 1;
    }
    else
    {
        negative_pressure_armed = 0;
        negative_pressure_stop_cycle();
    }
}

uint8 negative_pressure_fire(void)
{
#if NEGATIVE_PRESSURE_BENCH_TRIGGER_ENABLE
    if (!negative_pressure_enabled
        || !negative_pressure_armed
        || negative_pressure_fault_latched
        || negative_pressure_lockout
        || negative_pressure_cooldown_active
        || negative_pressure_state != NEGATIVE_PRESSURE_STATE_OFF
        || pwm_state != 0U)
    {
        return 0U;
    }

    negative_pressure_state = NEGATIVE_PRESSURE_STATE_PREPARE;
    negative_pressure_state_ticks = 0U;
    negative_pressure_shot_count++;
    return 1U;
#else
    return 0U;
#endif
}

uint8 negative_pressure_reset(void)
{
    if (pwm_state != 0U || negative_pressure_cooldown_active)
    {
        return 0U;
    }

    negative_pressure_fault_latched = 0;
    negative_pressure_lockout = 0;
    negative_pressure_armed = 0;
    negative_pressure_shot_count = 0U;
    negative_pressure_stop_cycle();
    return 1U;
}

void negative_pressure_tick(void)
{
    if (negative_pressure_cooldown_active)
    {
        if (negative_pressure_cooldown_ticks_remaining > 0U)
        {
            negative_pressure_cooldown_ticks_remaining--;
        }
        if (negative_pressure_cooldown_ticks_remaining == 0U)
        {
            negative_pressure_cooldown_active = 0;
        }
    }

    if (pwm_state != 0U)
    {
        if (negative_pressure_enabled
            || negative_pressure_armed
            || negative_pressure_state != NEGATIVE_PRESSURE_STATE_OFF)
        {
            negative_pressure_latch_fault();
        }
        else
        {
            negative_pressure_output_off();
        }
        return;
    }

    if (negative_pressure_fault_latched)
    {
        negative_pressure_state = NEGATIVE_PRESSURE_STATE_FAULT;
        negative_pressure_output_off();
        return;
    }

    if (!negative_pressure_enabled)
    {
        negative_pressure_stop_cycle();
        return;
    }

    switch (negative_pressure_state)
    {
        case NEGATIVE_PRESSURE_STATE_PREPARE:
            negative_pressure_state_ticks++;
            if (negative_pressure_state_ticks >= NEGATIVE_PRESSURE_PREPARE_TICKS)
            {
                negative_pressure_state = NEGATIVE_PRESSURE_STATE_HOLD;
                negative_pressure_state_ticks = 0U;
            }
            break;

        case NEGATIVE_PRESSURE_STATE_HOLD:
            negative_pressure_state_ticks++;
            if (negative_pressure_state_ticks >= NEGATIVE_PRESSURE_HOLD_TICKS)
            {
                negative_pressure_state = NEGATIVE_PRESSURE_STATE_RELEASE;
                negative_pressure_state_ticks = 0U;
            }
            break;

        case NEGATIVE_PRESSURE_STATE_RELEASE:
            negative_pressure_state_ticks++;
            if (negative_pressure_state_ticks >= NEGATIVE_PRESSURE_RELEASE_TICKS)
            {
                negative_pressure_state = NEGATIVE_PRESSURE_STATE_OFF;
                negative_pressure_state_ticks = 0U;
                negative_pressure_armed = 0;
                negative_pressure_lockout = 1;
                negative_pressure_cooldown_active = 1;
                negative_pressure_cooldown_ticks_remaining =
                    NEGATIVE_PRESSURE_COOLDOWN_TICKS;
            }
            break;

        case NEGATIVE_PRESSURE_STATE_OFF:
            break;

        case NEGATIVE_PRESSURE_STATE_FAULT:
        default:
            negative_pressure_latch_fault();
            return;
    }

    negative_pressure_apply_output();
}

const char *negative_pressure_state_text(void)
{
    switch (negative_pressure_state)
    {
        case NEGATIVE_PRESSURE_STATE_PREPARE: return "PREP ";
        case NEGATIVE_PRESSURE_STATE_HOLD:    return "HOLD ";
        case NEGATIVE_PRESSURE_STATE_RELEASE: return "RELS ";
        case NEGATIVE_PRESSURE_STATE_FAULT:   return "FAULT";
        case NEGATIVE_PRESSURE_STATE_OFF:
        default:                              return "OFF  ";
    }
}
