#include "headfile.h"
#include "inductance4.h"
#include "inductance4_menu.h"

#define INDUCTANCE4_CAL_ITEMS 5U

static char xdata sensor_line[15];
static uint16 xdata format_divisor;
static uint8 xdata format_index;
static uint8 xdata sensor_line_offset;
static uint8 xdata sensor_signal_ok;

static void append_fixed_uint(char *line, uint8 *offset, uint16 value, uint8 digits)
{
    format_divisor = 1U;

    for (format_index = 1U; format_index < digits; format_index++)
    {
        format_divisor *= 10U;
    }

    for (format_index = 0U; format_index < digits; format_index++)
    {
        line[*offset] = (char)('0' + (value / format_divisor) % 10U);
        (*offset)++;
        format_divisor /= 10U;
    }
}

static void show_sensor_row(uint16 y, const char *name, Inductance4Channel channel)
{
    sensor_line_offset = 0U;
    sensor_signal_ok = g_inductance4[channel].filtered > 4U
        && g_inductance4[channel].filtered < 4091U;

    sensor_line[sensor_line_offset++] = name[0];
    sensor_line[sensor_line_offset++] = name[1];
    sensor_line[sensor_line_offset++] = sensor_signal_ok ? ' ' : '!';
    append_fixed_uint(sensor_line, &sensor_line_offset, g_inductance4[channel].filtered, 5U);
    sensor_line[sensor_line_offset++] = ' ';
    append_fixed_uint(sensor_line, &sensor_line_offset, (uint16)g_inductance4[channel].normalized, 3U);
    sensor_line[sensor_line_offset] = '\0';
    lcd_showstr_large(0U, y, sensor_line);
}

void display_inductance4_data(void)
{
    show_sensor_row(2U, "L ", INDUCTANCE4_L);
    show_sensor_row(34U, "LM", INDUCTANCE4_LM);
    show_sensor_row(66U, "RM", INDUCTANCE4_RM);
    show_sensor_row(98U, "R ", INDUCTANCE4_R);
}

static void adjust_selected_parameter(uint8 selected, uint8 key_press)
{
    float *parameter = NULL;

    if (selected == 1U) parameter = &inductance_error_a;
    else if (selected == 2U) parameter = &inductance_error_b;
    else if (selected == 3U) parameter = &inductance_error_c;
    else if (selected == 4U) parameter = &inductance_error_p;

    if (parameter == NULL) return;

    if (key_press == KEY_EVENT_ADJ_INC) *parameter += 0.05f;
    else if (key_press == KEY_EVENT_ADJ_DEC)
    {
        *parameter -= 0.05f;
        if (*parameter < 0.0f) *parameter = 0.0f;
    }
}

void display_inductance4_calibration(uint8 key_press)
{
    static uint8 selected = 0U;
    static uint8 save_result = 0U;

    if (key_press == KEY_EVENT_ITEM_NEXT)
    {
        selected = (uint8)((selected + 1U) % INDUCTANCE4_CAL_ITEMS);
        lcd_clear(WHITE);
    }

    if (selected == 0U && key_press == KEY_EVENT_ADJ_INC)
    {
        pwm_state = 0U;
        if (!inductance4_calibration_active)
        {
            inductance4_calibration_start();
            save_result = 0U;
        }
        else if (inductance4_calibration_finish())
        {
            save_result = inductance4_save_config();
        }
    }
    else
    {
        adjust_selected_parameter(selected, key_press);
    }

    if (key_press == KEY_EVENT_SAVE_ALL)
    {
        save_result = inductance4_save_config();
    }

    lcd_showstr_large(0U, 0U, "I4 CAL");
    ips114_show_string(112U, 4U, inductance4_calibration_active ? "RUN" : "STOP");
    ips114_show_string(160U, 4U, inductance4_calibration_valid ? "OK" : "WAIT");
    ips114_show_string(0U, 38U, selected == 0U ? ">CAL P72" : " CAL P72");
    ips114_show_string(96U, 38U, save_result ? "SAVED" : "     ");

    ips114_show_string(0U, 60U, selected == 1U ? ">A" : " A");
    ips114_show_float(24U, 60U, inductance_error_a, 1U, 2U);
    ips114_show_string(104U, 60U, selected == 2U ? ">B" : " B");
    ips114_show_float(128U, 60U, inductance_error_b, 1U, 2U);

    ips114_show_string(0U, 82U, selected == 3U ? ">C" : " C");
    ips114_show_float(24U, 82U, inductance_error_c, 1U, 2U);
    ips114_show_string(104U, 82U, selected == 4U ? ">P" : " P");
    ips114_show_float(128U, 82U, inductance_error_p, 1U, 2U);

    ips114_show_string(0U, 108U, "P73 SEL  P72 SET");
}
