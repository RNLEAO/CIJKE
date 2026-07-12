#include "legacy_lcd.h"

#define MENU_FONT_WIDTH       10U
#define MENU_FONT_HEIGHT      20U
#define MENU_LINE_HEIGHT      22U
#define MENU_VISIBLE_DETAILS   4U

static uint8 xdata menu_top_line = 1U;
static uint8 xdata draw_row;
static uint8 xdata draw_column;
static uint8 xdata source_row;
static uint8 xdata source_column;
static uint8 xdata source_bits;
static uint8 xdata string_index;
static char xdata number_buffer[34];

static uint8 menu_physical_line(uint16 logical_line)
{
    if (logical_line == 0U)
    {
        return 0U;
    }
    if (logical_line < menu_top_line
        || logical_line >= (uint16)menu_top_line + MENU_VISIBLE_DETAILS)
    {
        return 0xffU;
    }
    return (uint8)(logical_line - menu_top_line + 1U);
}

static void follow_cursor(uint16 logical_line, const char value[])
{
    uint8 has_cursor = 0U;

    for (string_index = 0U; value[string_index] != '\0'; string_index++)
    {
        if (value[string_index] == '<' || value[string_index] == '>')
        {
            has_cursor = 1U;
            break;
        }
    }

    if (!has_cursor || logical_line == 0U)
    {
        return;
    }
    if (logical_line < menu_top_line)
    {
        menu_top_line = (uint8)logical_line;
    }
    else if (logical_line >= (uint16)menu_top_line + MENU_VISIBLE_DETAILS)
    {
        menu_top_line = (uint8)(logical_line - MENU_VISIBLE_DETAILS + 1U);
    }
}

static void menu_show_char(uint16 x, uint16 y, char value)
{
    if (value < 32 || value > 126)
    {
        value = '?';
    }

    for (draw_row = 0U; draw_row < MENU_FONT_HEIGHT; draw_row++)
    {
        source_row = (uint8)((uint16)draw_row * 16U / MENU_FONT_HEIGHT);
        source_bits = ascii_font_8x16[value - 32][source_row];
        for (draw_column = 0U; draw_column < MENU_FONT_WIDTH; draw_column++)
        {
            source_column = (uint8)((uint16)draw_column * 8U / MENU_FONT_WIDTH);
            ips114_draw_point(
                x + draw_column,
                y + draw_row,
                (source_bits & (uint8)(1U << source_column))
                    ? RGB565_BLACK : RGB565_WHITE);
        }
    }
}

static void menu_show_string_at(uint16 x, uint16 y, const char value[])
{
    for (string_index = 0U;
         value[string_index] != '\0' && x <= 230U;
         string_index++)
    {
        menu_show_char(x, y, value[string_index]);
        x += MENU_FONT_WIDTH;
    }
}

static void menu_show_string(uint16 x, uint16 line, const char value[])
{
    uint8 physical_line;

    follow_cursor(line, value);
    physical_line = menu_physical_line(line);
    if (physical_line == 0xffU)
    {
        return;
    }
    menu_show_string_at(
        (uint16)(x * MENU_FONT_WIDTH / 8U),
        (uint16)physical_line * MENU_LINE_HEIGHT,
        value);
}

void lcd_init(void)
{
    ips114_set_dir(IPS114_CROSSWISE_180);
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
    ips114_init();
}

void lcd_clear(uint16 color)
{
    menu_top_line = 1U;
    ips114_clear(color);
}

void lcd_drawpoint(uint16 x, uint16 y, uint16 color)
{
    ips114_draw_point(x, y, color);
}

void lcd_showchar(uint16 x, uint16 line, const char value)
{
    number_buffer[0] = value;
    number_buffer[1] = '\0';
    menu_show_string(x, line, number_buffer);
}

void lcd_showstr(uint16 x, uint16 line, const char value[])
{
    menu_show_string(x, line, value);
}

void lcd_showint8(uint16 x, uint16 line, int8 value)
{
    zf_sprintf((int8 *)number_buffer, (const int8 *)"%d   ", value);
    menu_show_string(x, line, number_buffer);
}

void lcd_showuint8(uint16 x, uint16 line, uint8 value)
{
    zf_sprintf((int8 *)number_buffer, (const int8 *)"%03u", value);
    menu_show_string(x, line, number_buffer);
}

void lcd_showint16(uint16 x, uint16 line, int16 value)
{
    zf_sprintf((int8 *)number_buffer, (const int8 *)"%d      ", value);
    menu_show_string(x, line, number_buffer);
}

void lcd_showuint16(uint16 x, uint16 line, uint16 value)
{
    zf_sprintf((int8 *)number_buffer, (const int8 *)"%05u", value);
    menu_show_string(x, line, number_buffer);
}

void lcd_showint32(uint16 x, uint16 line, int32 value, uint8 digits)
{
    zf_sprintf((int8 *)number_buffer, (const int8 *)"%d", value);
    number_buffer[digits + 1U] = '\0';
    menu_show_string(x, line, number_buffer);
}

void lcd_showfloat(uint16 x, uint16 line, double value, uint8 digits, uint8 decimals)
{
    uint8 length;
    int8 start;
    int8 end;
    int8 point;

    if (decimals > 6U) decimals = 6U;
    if (digits > 10U) digits = 10U;

    if (value < 0.0)
    {
        length = zf_sprintf((int8 *)number_buffer, (const int8 *)"%f", value);
    }
    else
    {
        number_buffer[0] = ' ';
        length = zf_sprintf((int8 *)&number_buffer[1], (const int8 *)"%f", value);
        length++;
    }

    point = (int8)length - 7;
    start = point - (int8)digits - 1;
    end = point + (int8)decimals + 1;
    while (start < 0)
    {
        number_buffer[end++] = ' ';
        start++;
    }
    number_buffer[start] = value < 0.0 ? '-' : ' ';
    number_buffer[end] = '\0';
    menu_show_string(x, line, &number_buffer[start]);
}

void lcd_showstr_large(uint16 x, uint16 y, const char value[])
{
    menu_show_string_at(x, y, value);
}

void lcd_show_status(uint8 state)
{
    if (state == 0U) ips114_show_string(208U, 119U, "STOP");
    else if (state == 1U) ips114_show_string(216U, 119U, "RUN");
    else ips114_show_string(200U, 119U, "CLEAN");
}

void lcd_show_font(
    uint16 x,
    uint16 y,
    uint8 width,
    uint8 height,
    const uint8 *font_data,
    uint16 font_color,
    uint16 background_color)
{
    uint8 row;
    uint8 column;
    uint8 byte_data;
    uint8 bytes_per_row = (uint8)((width + 7U) / 8U);

    for (row = 0; row < height; row++)
    {
        for (column = 0; column < width; column++)
        {
            byte_data = font_data[(uint16)row * bytes_per_row + column / 8U];
            ips114_draw_point(
                x + column,
                y + row,
                (byte_data & (uint8)(0x80U >> (column & 7U)))
                    ? font_color : background_color);
        }
    }
}
