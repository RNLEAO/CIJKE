#ifndef CIJIANKE_LEGACY_LCD_H
#define CIJIANKE_LEGACY_LCD_H

#include "zf_common_headfile.h"

void lcd_init(void);
void lcd_clear(uint16 color);
void lcd_drawpoint(uint16 x, uint16 y, uint16 color);
void lcd_showchar(uint16 x, uint16 line, const char value);
void lcd_showstr(uint16 x, uint16 line, const char value[]);
void lcd_showint8(uint16 x, uint16 line, int8 value);
void lcd_showuint8(uint16 x, uint16 line, uint8 value);
void lcd_showint16(uint16 x, uint16 line, int16 value);
void lcd_showuint16(uint16 x, uint16 line, uint16 value);
void lcd_showint32(uint16 x, uint16 line, int32 value, uint8 digits);
void lcd_showfloat(uint16 x, uint16 line, double value, uint8 digits, uint8 decimals);
void lcd_showstr_large(uint16 x, uint16 y, const char value[]);
void lcd_show_status(uint8 state);
void lcd_show_font(
    uint16 x,
    uint16 y,
    uint8 width,
    uint8 height,
    const uint8 *font_data,
    uint16 font_color,
    uint16 background_color);

#endif
