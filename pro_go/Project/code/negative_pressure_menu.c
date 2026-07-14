#include "headfile.h"

#define NEGATIVE_PRESSURE_BENCH_MENU_ITEMS 4U

static void negative_pressure_bench_action(uint8 selected)
{
    switch (selected)
    {
        case 0U:
            negative_pressure_set_enabled(!negative_pressure_enabled);
            break;
        case 1U:
            negative_pressure_set_armed(!negative_pressure_armed);
            break;
        case 2U:
            negative_pressure_fire();
            break;
        case 3U:
            negative_pressure_reset();
            break;
        default:
            break;
    }
}

void display_negative_pressure_bench(uint8 key_press)
{
    static uint8 selected = 0U;

    if (key_press == KEY_EVENT_ITEM_NEXT)
    {
        selected = (uint8)((selected + 1U) % NEGATIVE_PRESSURE_BENCH_MENU_ITEMS);
        lcd_clear(WHITE);
    }
    else if (key_press == KEY_EVENT_ADJ_INC
             || key_press == KEY_EVENT_ADJ_DEC)
    {
        negative_pressure_bench_action(selected);
    }

    lcd_showstr(0U, 0U, "NP BENCH 30% 1S P33");
    lcd_showstr(0U, 1U, selected == 0U ? ">EN:" : " EN:");
    lcd_showstr(35U, 1U, negative_pressure_enabled ? "ON " : "OFF");
    lcd_showstr(85U, 1U, selected == 1U ? ">ARM:" : " ARM:");
    lcd_showstr(135U, 1U, negative_pressure_armed ? "ON " : "OFF");

    lcd_showstr(0U, 2U, selected == 2U ? ">FIRE" : " FIRE");
    lcd_showstr(85U, 2U, selected == 3U ? ">RESET" : " RESET");

    lcd_showstr(0U, 3U, "STATE:");
    lcd_showstr(55U, 3U, negative_pressure_state_text());
    lcd_showstr(115U, 3U, "REAL:");
    lcd_showuint8(160U, 3U, negative_pressure_real_output_percent);

    lcd_showstr(0U, 4U, "STATUS:");
    if (negative_pressure_fault_latched)
    {
        lcd_showstr(60U, 4U, "WAIT ");
    }
    else if (negative_pressure_cooldown_active
             || negative_pressure_lockout
             || negative_pressure_state != NEGATIVE_PRESSURE_STATE_OFF)
    {
        lcd_showstr(60U, 4U, "WAIT ");
    }
    else
    {
        lcd_showstr(60U, 4U, "READY");
    }
}
