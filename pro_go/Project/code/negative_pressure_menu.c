#include "headfile.h"

#define NEGATIVE_PRESSURE_AUTO_MENU_ITEMS 6U

static const char *negative_pressure_state_text(void)
{
    switch (negative_pressure_auto_state)
    {
        case NEG_PRESS_AUTO_STATE_PREPARE: return "PREP ";
        case NEG_PRESS_AUTO_STATE_HOLD:    return "HOLD ";
        case NEG_PRESS_AUTO_STATE_RELEASE: return "RELS ";
        case NEG_PRESS_AUTO_STATE_FAULT:   return "FAULT";
        case NEG_PRESS_AUTO_STATE_OFF:
        default:                           return "OFF  ";
    }
}

static const char *negative_pressure_trigger_text(void)
{
    switch (negative_pressure_auto_trigger_source)
    {
        case NEG_PRESS_TRIGGER_MENU_SIM:     return "MENU";
        case NEG_PRESS_TRIGGER_ELEMENT_RING: return "RING";
        case NEG_PRESS_TRIGGER_FAULT:        return "FLT ";
        case NEG_PRESS_TRIGGER_NONE:
        default:                             return "NONE";
    }
}

static void negative_pressure_apply_menu_action(uint8 selected)
{
    switch (selected)
    {
        case 0U:
            negative_pressure_set_enabled(!negative_pressure_auto_enabled);
            break;
        case 1U:
            negative_pressure_set_armed(!negative_pressure_auto_armed);
            break;
        case 2U:
            negative_pressure_set_request_mode(
                !negative_pressure_auto_use_element_trigger);
            break;
        case 3U:
            if (!negative_pressure_auto_use_element_trigger)
            {
                negative_pressure_set_manual_request(
                    !negative_pressure_auto_manual_request);
            }
            break;
        case 4U:
            negative_pressure_clear_lockout();
            break;
        case 5U:
            negative_pressure_set_fault(!negative_pressure_auto_fault_latched);
            break;
        default:
            break;
    }
}

void display_negative_pressure_auto(uint8 key_press)
{
    static uint8 selected = 0U;

    if (key_press == KEY_EVENT_ITEM_NEXT)
    {
        selected = (uint8)((selected + 1U) % NEGATIVE_PRESSURE_AUTO_MENU_ITEMS);
        lcd_clear(WHITE);
    }
    else if (key_press == KEY_EVENT_ADJ_INC
             || key_press == KEY_EVENT_ADJ_DEC)
    {
        negative_pressure_apply_menu_action(selected);
    }

    lcd_showstr(0U, 0U, "NP AUTO SAFE");

    lcd_showstr(0U, 1U, selected == 0U ? ">EN:" : " EN:");
    lcd_showstr(35U, 1U, negative_pressure_auto_enabled ? "ON " : "OFF");
    lcd_showstr(80U, 1U, selected == 1U ? ">ARM:" : " ARM:");
    lcd_showstr(130U, 1U, negative_pressure_auto_armed ? "ON " : "OFF");

    lcd_showstr(0U, 2U, selected == 2U ? ">MODE:" : " MODE:");
    lcd_showstr(50U, 2U,
                negative_pressure_auto_use_element_trigger ? "RING" : "MAN ");
    lcd_showstr(100U, 2U, selected == 3U ? ">REQ:" : " REQ:");
    lcd_showstr(145U, 2U, negative_pressure_auto_request ? "ON " : "OFF");

    lcd_showstr(0U, 3U, selected == 4U ? ">CLEAR" : " CLEAR");
    lcd_showstr(80U, 3U, selected == 5U ? ">FAULT:" : " FAULT:");
    lcd_showstr(140U, 3U,
                negative_pressure_auto_fault_latched ? "ON " : "OFF");

    lcd_showstr(0U, 4U, "STATE:");
    lcd_showstr(55U, 4U, negative_pressure_state_text());
    lcd_showstr(105U, 4U, "SRC:");
    lcd_showstr(145U, 4U, negative_pressure_trigger_text());

    lcd_showstr(0U, 5U, "SHOT:");
    lcd_showuint8(50U, 5U, negative_pressure_auto_shot_count);
    lcd_showstr(90U, 5U, "CD:");
    lcd_showuint16(120U, 5U, negative_pressure_auto_cooldown_ticks_remaining);

    lcd_showstr(0U, 6U, "TARGET:");
    lcd_showuint8(65U, 6U, negative_pressure_auto_target_duty_percent);
    lcd_showstr(110U, 6U, "REAL:");
    lcd_showuint8(155U, 6U, negative_pressure_auto_real_output_percent);

    lcd_showstr(0U, 7U, negative_pressure_auto_lockout
                ? "LOCKOUT ACTIVE "
                : "LOCKOUT CLEAR  ");
}

void display_negative_pressure_output(uint8 key_press)
{
    if (key_press == KEY_EVENT_ADJ_INC)
    {
        negative_pressure_set_duty_percent(
            (uint8)(negative_pressure_duty_percent + 1U));
    }
    else if (key_press == KEY_EVENT_ADJ_DEC)
    {
        negative_pressure_set_duty_percent(
            (uint8)(negative_pressure_duty_percent - 1U));
    }

    lcd_showstr(0U, 0U, "NP OUTPUT P33");
    lcd_showstr(0U, 1U, "FREQ:17000 Hz");
    lcd_showstr(0U, 2U, "DUTY:");
    lcd_showuint8(55U, 2U, negative_pressure_duty_percent);
    lcd_showstr(95U, 2U, "RANGE 5-10");

    lcd_showstr(0U, 3U, "BUILD:");
#if NEGATIVE_PRESSURE_LOAD_TEST_BUILD_ENABLE
    lcd_showstr(60U, 3U, "ON ");
#else
    lcd_showstr(60U, 3U, "OFF");
#endif
    lcd_showstr(105U, 3U, "PHYS:");
#if NEGATIVE_PRESSURE_LOAD_TEST_PHYSICAL_OUTPUT_ENABLE
    lcd_showstr(155U, 3U, "ON ");
#else
    lcd_showstr(155U, 3U, "OFF");
#endif

    lcd_showstr(0U, 4U, "TARGET:");
    lcd_showuint8(65U, 4U, negative_pressure_auto_target_duty_percent);
    lcd_showstr(110U, 4U, "REAL:");
    lcd_showuint8(155U, 4U, negative_pressure_auto_real_output_percent);

    lcd_showstr(0U, 5U, "STATE:");
    lcd_showstr(55U, 5U, negative_pressure_state_text());
    lcd_showstr(105U, 5U, "SRC:");
    lcd_showstr(145U, 5U, negative_pressure_trigger_text());

#if NEGATIVE_PRESSURE_LOAD_TEST_BUILD_ENABLE && \
    NEGATIVE_PRESSURE_LOAD_TEST_PHYSICAL_OUTPUT_ENABLE
    lcd_showstr(0U, 7U, "MANUAL TEST ONLY");
#else
    lcd_showstr(0U, 7U, "OUTPUT LOCKED   ");
#endif
}
