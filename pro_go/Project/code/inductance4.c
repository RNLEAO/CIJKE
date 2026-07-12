#include "headfile.h"
#include "inductance4.h"

#define INDUCTANCE4_ADC_FULL_SCALE       4095U
#define INDUCTANCE4_MIN_CALIBRATION_SPAN 200U
#define INDUCTANCE4_MAX_MARGIN_NUM       108UL
#define INDUCTANCE4_MAX_MARGIN_DEN       100UL
#define INDUCTANCE4_CONFIG_ADDR          0x0400UL
#define INDUCTANCE4_CONFIG_MAGIC         0x34494E44UL
#define INDUCTANCE4_CONFIG_VERSION       4U

static const uint16 code inductance4_default_min[INDUCTANCE4_CHANNEL_COUNT] =
{
    0U, 0U, 0U, 0U
};

static const uint16 code inductance4_default_max[INDUCTANCE4_CHANNEL_COUNT] =
{
    3999U, 3995U, 3969U, 4007U
};

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 size;
    uint16 min_value[INDUCTANCE4_CHANNEL_COUNT];
    uint16 max_value[INDUCTANCE4_CHANNEL_COUNT];
    float error_a;
    float error_b;
    float error_c;
    float error_p;
    float right_angle_outer_low;
    float right_angle_inner_high;
    float right_angle_inner_low;
    float right_angle_inner_diff;
    float right_angle_min_angle;
    float right_angle_target_angle;
    float right_angle_reverse_speed;
    float right_angle_forward_speed;
    float ring_outer_high;
    float ring_inner_low;
    float ring_inner_active;
    float ring_side_diff;
    float ring_error;
    float ring_exit_error;
    float ring_enter_distance;
    float ring_recover_distance;
    float ring_hold_angle;
    float ring_exit_angle;
    float ring_recover_ticks;
    uint16 checksum;
} Inductance4StoredConfig;

Inductance4Sensor g_inductance4[INDUCTANCE4_CHANNEL_COUNT];

float inductance_error_a = 1.5f;
float inductance_error_b = 2.0f;
float inductance_error_c = 1.0f;
float inductance_error_p = 1.0f;

uint8 inductance4_calibration_active = 0;
uint8 inductance4_calibration_valid = 1;
uint8 inductance4_calibration_failed_channel = 0xFFU;

static uint16 xdata calibration_min[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata calibration_max[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata calibration_adjusted_min[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata calibration_adjusted_max[INDUCTANCE4_CHANNEL_COUNT];
static uint8 calibration_auto_capture = 1U;

static uint16 config_checksum(const uint8 *bytes, uint16 length)
{
    uint16 checksum = 0;

    while (length--)
    {
        checksum = (uint16)(checksum + *bytes++);
    }

    return checksum;
}

static uint16 clamp_u16(uint32 value, uint16 maximum)
{
    if (value > maximum)
    {
        return maximum;
    }

    return (uint16)value;
}

void inductance4_init(void)
{
    uint8 i;

    g_inductance4[INDUCTANCE4_L].adc_channel = ADC_P00;
    g_inductance4[INDUCTANCE4_LM].adc_channel = ADC_P01;
    g_inductance4[INDUCTANCE4_RM].adc_channel = ADC_P05;
    g_inductance4[INDUCTANCE4_R].adc_channel = ADC_P06;

    for (i = 0; i < INDUCTANCE4_CHANNEL_COUNT; i++)
    {
        g_inductance4[i].raw = 0;
        g_inductance4[i].filtered = 0;
        g_inductance4[i].min_value = inductance4_default_min[i];
        g_inductance4[i].max_value = inductance4_default_max[i];
        g_inductance4[i].normalized = 0;
        g_inductance4[i].healthy = 1;
    }
}

uint16 inductance4_trimmed_average(ADCN_enum channel)
{
    uint8 i;
    uint16 value;
    uint16 minimum = INDUCTANCE4_ADC_FULL_SCALE;
    uint16 maximum = 0;
    uint32 sum = 0;

    for (i = 0; i < INDUCTANCE4_SAMPLE_COUNT; i++)
    {
        value = adc_once(channel, ADC_12BIT);
        sum += value;

        if (value < minimum)
        {
            minimum = value;
        }

        if (value > maximum)
        {
            maximum = value;
        }
    }

    sum -= minimum;
    sum -= maximum;

    return (uint16)(sum / (INDUCTANCE4_SAMPLE_COUNT - 2));
}

int16 inductance4_normalize(uint16 value, uint16 max_value, uint16 min_value)
{
    int32 result;

    if (max_value <= min_value)
    {
        return 0;
    }

    result = (((int32)value - (int32)min_value) << 7)
           / ((int32)max_value - (int32)min_value);

    if (result < 0)
    {
        result = 0;
    }
    else if (result > INDUCTANCE4_NORM_MAX)
    {
        result = INDUCTANCE4_NORM_MAX;
    }

    return (int16)result;
}

void inductance4_update(void)
{
    uint8 i;

    for (i = 0; i < INDUCTANCE4_CHANNEL_COUNT; i++)
    {
        g_inductance4[i].raw = inductance4_trimmed_average(g_inductance4[i].adc_channel);
        g_inductance4[i].filtered = g_inductance4[i].raw;

        if (inductance4_calibration_active && calibration_auto_capture)
        {
            if (g_inductance4[i].filtered < calibration_min[i])
            {
                calibration_min[i] = g_inductance4[i].filtered;
            }

            if (g_inductance4[i].filtered > calibration_max[i])
            {
                calibration_max[i] = g_inductance4[i].filtered;
            }
        }

        g_inductance4[i].normalized = inductance4_normalize(
            g_inductance4[i].filtered,
            g_inductance4[i].max_value,
            g_inductance4[i].min_value);
    }

    L_raw = (float)g_inductance4[INDUCTANCE4_L].filtered;
    LM_raw = (float)g_inductance4[INDUCTANCE4_LM].filtered;
    RM_raw = (float)g_inductance4[INDUCTANCE4_RM].filtered;
    R_raw = (float)g_inductance4[INDUCTANCE4_R].filtered;

    L = (float)g_inductance4[INDUCTANCE4_L].normalized;
    LM = (float)g_inductance4[INDUCTANCE4_LM].normalized;
    RM = (float)g_inductance4[INDUCTANCE4_RM].normalized;
    R = (float)g_inductance4[INDUCTANCE4_R].normalized;
    MID = 0.0f;

    inductance4_check_health();
}

float inductance4_calculate_error(void)
{
    float numerator;
    float denominator;

    numerator = inductance_error_a * (L - R)
              + inductance_error_b * (LM - RM);

    denominator = inductance_error_a * (L + R)
                + inductance_error_c * fabs(LM - RM);

    if (denominator < 0.001f)
    {
        return 0.0f;
    }

    return numerator / denominator * inductance_error_p;
}

void inductance4_calibration_start(void)
{
    uint8 i;

    for (i = 0; i < INDUCTANCE4_CHANNEL_COUNT; i++)
    {
        calibration_min[i] = INDUCTANCE4_ADC_FULL_SCALE;
        calibration_max[i] = 0;
    }

    inductance4_calibration_active = 1;
    inductance4_calibration_valid = 0;
    inductance4_calibration_failed_channel = 0xFFU;
    calibration_auto_capture = 1U;
}

void inductance4_guided_calibration_start(void)
{
    inductance4_calibration_start();
    calibration_auto_capture = 0U;
}

void inductance4_calibration_capture_current(void)
{
    uint16 xdata values[INDUCTANCE4_CHANNEL_COUNT];
    uint8 i;

    if (!inductance4_calibration_active)
    {
        return;
    }

    for (i = 0; i < INDUCTANCE4_CHANNEL_COUNT; i++)
    {
        values[i] = g_inductance4[i].filtered;
    }
    inductance4_calibration_capture_values(values);
}

void inductance4_calibration_capture_values(const uint16 *values)
{
    uint8 i;

    if (!inductance4_calibration_active || values == NULL)
    {
        return;
    }

    for (i = 0; i < INDUCTANCE4_CHANNEL_COUNT; i++)
    {
        if (values[i] < calibration_min[i])
        {
            calibration_min[i] = values[i];
        }
        if (values[i] > calibration_max[i])
        {
            calibration_max[i] = values[i];
        }
    }
}

void inductance4_calibration_cancel(void)
{
    inductance4_calibration_active = 0U;
    calibration_auto_capture = 1U;
    inductance4_calibration_failed_channel = 0xFFU;
    inductance4_calibration_valid = inductance4_check_health();
}

void inductance4_get_calibration_extrema(uint16 *min_values, uint16 *max_values)
{
    uint8 i;

    if (min_values == NULL || max_values == NULL)
    {
        return;
    }

    for (i = 0; i < INDUCTANCE4_CHANNEL_COUNT; i++)
    {
        min_values[i] = calibration_min[i];
        max_values[i] = calibration_max[i];
    }
}

uint8 inductance4_calibration_finish(void)
{
    uint8 i;

    inductance4_calibration_active = 0;
    calibration_auto_capture = 1U;
    inductance4_calibration_failed_channel = 0xFFU;

    for (i = 0; i < INDUCTANCE4_CHANNEL_COUNT; i++)
    {
        if (calibration_max[i] <= calibration_min[i]
            || (calibration_max[i] - calibration_min[i]) < INDUCTANCE4_MIN_CALIBRATION_SPAN)
        {
            inductance4_calibration_failed_channel = i;
            inductance4_calibration_valid = 0;
            return 0;
        }

        calibration_adjusted_min[i] = calibration_min[i] / 2U;
        calibration_adjusted_max[i] = clamp_u16(
            ((uint32)calibration_max[i] * INDUCTANCE4_MAX_MARGIN_NUM)
            / INDUCTANCE4_MAX_MARGIN_DEN,
            INDUCTANCE4_ADC_FULL_SCALE);

        if (calibration_adjusted_max[i] <= calibration_adjusted_min[i]
            || (calibration_adjusted_max[i] - calibration_adjusted_min[i])
                < INDUCTANCE4_MIN_CALIBRATION_SPAN)
        {
            inductance4_calibration_failed_channel = i;
            inductance4_calibration_valid = 0;
            return 0;
        }
    }

    for (i = 0; i < INDUCTANCE4_CHANNEL_COUNT; i++)
    {
        g_inductance4[i].min_value = calibration_adjusted_min[i];
        g_inductance4[i].max_value = calibration_adjusted_max[i];
    }

    inductance4_calibration_valid = 1;
    return 1;
}

void inductance4_set_calibration(const uint16 *min_values, const uint16 *max_values)
{
    uint8 i;

    if (min_values == NULL || max_values == NULL)
    {
        return;
    }

    for (i = 0; i < INDUCTANCE4_CHANNEL_COUNT; i++)
    {
        g_inductance4[i].min_value = min_values[i];
        g_inductance4[i].max_value = max_values[i];
    }

    inductance4_calibration_valid = inductance4_check_health();
}

void inductance4_get_calibration(uint16 *min_values, uint16 *max_values)
{
    uint8 i;

    if (min_values == NULL || max_values == NULL)
    {
        return;
    }

    for (i = 0; i < INDUCTANCE4_CHANNEL_COUNT; i++)
    {
        min_values[i] = g_inductance4[i].min_value;
        max_values[i] = g_inductance4[i].max_value;
    }
}

uint8 inductance4_check_health(void)
{
    uint8 i;
    uint8 all_healthy = 1;

    for (i = 0; i < INDUCTANCE4_CHANNEL_COUNT; i++)
    {
        if (g_inductance4[i].max_value <= g_inductance4[i].min_value
            || (g_inductance4[i].max_value - g_inductance4[i].min_value)
                < INDUCTANCE4_MIN_CALIBRATION_SPAN)
        {
            g_inductance4[i].healthy = 0;
            all_healthy = 0;
        }
        else
        {
            g_inductance4[i].healthy = 1;
        }
    }

    return all_healthy;
}

uint8 inductance4_load_config(void)
{
    Inductance4StoredConfig xdata config;
    uint16 xdata expected_checksum;

    iap_read_bytes(INDUCTANCE4_CONFIG_ADDR, (uint8 *)&config, sizeof(config));

    if (config.magic != INDUCTANCE4_CONFIG_MAGIC
        || config.version != INDUCTANCE4_CONFIG_VERSION
        || config.size != sizeof(config))
    {
        return 0;
    }

    expected_checksum = config_checksum(
        (const uint8 *)&config,
        (uint16)((uint8 *)&config.checksum - (uint8 *)&config));

    if (expected_checksum != config.checksum)
    {
        return 0;
    }

    inductance4_set_calibration(config.min_value, config.max_value);
    if (!inductance4_calibration_valid)
    {
        return 0;
    }

    inductance_error_a = config.error_a;
    inductance_error_b = config.error_b;
    inductance_error_c = config.error_c;
    inductance_error_p = config.error_p;
    right_angle_outer_low = config.right_angle_outer_low;
    right_angle_inner_high = config.right_angle_inner_high;
    right_angle_inner_low = config.right_angle_inner_low;
    right_angle_inner_diff = config.right_angle_inner_diff;
    right_angle_min_angle = config.right_angle_min_angle;
    right_angle_target_angle = config.right_angle_target_angle;
    right_angle_reverse_speed = config.right_angle_reverse_speed;
    right_angle_forward_speed = config.right_angle_forward_speed;
    ring_outer_high = config.ring_outer_high;
    ring_inner_low = config.ring_inner_low;
    ring_inner_active = config.ring_inner_active;
    ring_side_diff = config.ring_side_diff;
    ring_error = config.ring_error;
    ring_exit_error = config.ring_exit_error;
    ring_inc_element12 = config.ring_enter_distance;
    ring_inc_element56 = config.ring_recover_distance;
    ring_angle_34 = config.ring_hold_angle;
    ring_angle_45 = config.ring_exit_angle;
    temp_flag_tar = config.ring_recover_ticks;

    return 1;
}

uint8 inductance4_save_config(void)
{
    Inductance4StoredConfig xdata config;

    if (!inductance4_check_health())
    {
        return 0;
    }

    memset(&config, 0, sizeof(config));
    config.magic = INDUCTANCE4_CONFIG_MAGIC;
    config.version = INDUCTANCE4_CONFIG_VERSION;
    config.size = sizeof(config);
    inductance4_get_calibration(config.min_value, config.max_value);
    config.error_a = inductance_error_a;
    config.error_b = inductance_error_b;
    config.error_c = inductance_error_c;
    config.error_p = inductance_error_p;
    config.right_angle_outer_low = right_angle_outer_low;
    config.right_angle_inner_high = right_angle_inner_high;
    config.right_angle_inner_low = right_angle_inner_low;
    config.right_angle_inner_diff = right_angle_inner_diff;
    config.right_angle_min_angle = right_angle_min_angle;
    config.right_angle_target_angle = right_angle_target_angle;
    config.right_angle_reverse_speed = right_angle_reverse_speed;
    config.right_angle_forward_speed = right_angle_forward_speed;
    config.ring_outer_high = ring_outer_high;
    config.ring_inner_low = ring_inner_low;
    config.ring_inner_active = ring_inner_active;
    config.ring_side_diff = ring_side_diff;
    config.ring_error = ring_error;
    config.ring_exit_error = ring_exit_error;
    config.ring_enter_distance = ring_inc_element12;
    config.ring_recover_distance = ring_inc_element56;
    config.ring_hold_angle = ring_angle_34;
    config.ring_exit_angle = ring_angle_45;
    config.ring_recover_ticks = temp_flag_tar;
    config.checksum = config_checksum(
        (const uint8 *)&config,
        (uint16)((uint8 *)&config.checksum - (uint8 *)&config));

    iap_erase_page(INDUCTANCE4_CONFIG_ADDR);
    iap_write_bytes(INDUCTANCE4_CONFIG_ADDR, (uint8 *)&config, sizeof(config));

    return 1;
}
