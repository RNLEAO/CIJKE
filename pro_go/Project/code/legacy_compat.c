#include "legacy_compat.h"

void board_init(void)
{
    clock_init(SYSTEM_CLOCK_33_1776M);
    debug_init();
}

void delay_init(void)
{
    system_delay_init();
}

void delay_ms(uint16 ms)
{
    system_delay_ms(ms);
}

void delay_us(uint32 us)
{
    while (us > 65535UL)
    {
        system_delay_us(65535U);
        us -= 65535UL;
    }
    system_delay_us((uint16)us);
}

uint16 adc_once(ADCN_enum channel, ADCRES_enum resolution)
{
    adc_init(channel, resolution);
    return adc_convert(channel);
}

void gpio_mode(PIN_enum pin, GPIOMODE_enum mode)
{
    if (mode == GPO_PP)
    {
        gpio_init(pin, GPO, GPIO_LOW, GPO_PUSH_PULL);
    }
    else
    {
        gpio_init(pin, GPI, GPIO_LOW, GPI_IMPEDANCE);
    }
}

void ctimer_count_init(CTIMN_enum timer)
{
    switch (timer)
    {
        case CTIM0_P34:
            gpio_init(IO_P34, GPI, GPIO_LOW, GPI_IMPEDANCE);
            TL0 = 0;
            TH0 = 0;
            TMOD |= 0x04;
            TR0 = 1;
            break;

        case CTIM1_P35:
            gpio_init(IO_P35, GPI, GPIO_LOW, GPI_IMPEDANCE);
            TL1 = 0;
            TH1 = 0;
            TMOD |= 0x40;
            TR1 = 1;
            break;

        case CTIM2_P12:
            gpio_init(IO_P12, GPI, GPIO_LOW, GPI_IMPEDANCE);
            T2L = 0;
            T2H = 0;
            AUXR |= 0x18;
            break;

        case CTIM3_P04:
            gpio_init(IO_P04, GPI, GPIO_LOW, GPI_IMPEDANCE);
            T3L = 0;
            T3H = 0;
            T4T3M |= 0x0C;
            break;

        case CTIM4_P06:
            gpio_init(IO_P06, GPI, GPIO_LOW, GPI_IMPEDANCE);
            T4L = 0;
            T4H = 0;
            T4T3M |= 0xC0;
            break;
    }
}

uint16 ctimer_count_read(CTIMN_enum timer)
{
    uint16 count = 0;

    switch (timer)
    {
        case CTIM0_P34:
            count = ((uint16)TH0 << 8) | (uint8)TL0;
            break;
        case CTIM1_P35:
            count = ((uint16)TH1 << 8) | (uint8)TL1;
            break;
        case CTIM2_P12:
            count = ((uint16)T2H << 8) | (uint8)T2L;
            break;
        case CTIM3_P04:
            count = ((uint16)T3H << 8) | (uint8)T3L;
            break;
        case CTIM4_P06:
            count = ((uint16)T4H << 8) | (uint8)T4L;
            break;
    }

    return count;
}

void ctimer_count_clean(CTIMN_enum timer)
{
    switch (timer)
    {
        case CTIM0_P34:
            TR0 = 0;
            TH0 = 0;
            TL0 = 0;
            TR0 = 1;
            break;
        case CTIM1_P35:
            TR1 = 0;
            TH1 = 0;
            TL1 = 0;
            TR1 = 1;
            break;
        case CTIM2_P12:
            AUXR &= ~(1 << 4);
            T2H = 0;
            T2L = 0;
            AUXR |= 1 << 4;
            break;
        case CTIM3_P04:
            T4T3M &= ~(1 << 3);
            T3H = 0;
            T3L = 0;
            T4T3M |= 1 << 3;
            break;
        case CTIM4_P06:
            T4T3M &= ~(1 << 7);
            T4H = 0;
            T4L = 0;
            T4T3M |= 1 << 7;
            break;
    }
}

void pit_timer_ms(TIMN_enum timer, uint16 period_ms)
{
    switch (timer)
    {
        case TIM_0:
            pit_ms_init(TIM0_PIT, period_ms);
            break;
        case TIM_1:
            pit_ms_init(TIM1_PIT, period_ms);
            break;
        case TIM_2:
            pit_ms_init(TIM2_PIT, period_ms);
            break;
        case TIM_3:
            pit_ms_init(TIM3_PIT, period_ms);
            break;
        case TIM_4:
            pit_ms_init(TIM4_PIT, period_ms);
            break;
        default:
            break;
    }
}

void iap_read_bytes(uint32 address, uint8 *buffer, uint16 length)
{
    iap_read_buff(address, buffer, length);
}

void iap_write_bytes(uint32 address, uint8 *buffer, uint16 length)
{
    iap_write_buff(address, buffer, length);
}

uint32 wireless_uart_send_buff(uint8 *buffer, uint32 length)
{
    return wireless_uart_send_buffer(buffer, length);
}

uint32 wireless_uart_read_buff(uint8 *buffer, uint32 length)
{
    return wireless_uart_read_buffer(buffer, length);
}
