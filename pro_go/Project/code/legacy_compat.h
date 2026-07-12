#ifndef CIJIANKE_LEGACY_COMPAT_H
#define CIJIANKE_LEGACY_COMPAT_H

#include "zf_common_headfile.h"

typedef adc_channel_enum ADCN_enum;
typedef adc_speed_enum ADC_SPEED_enum;
typedef adc_resolution_enum ADCRES_enum;
typedef timer_index_enum TIMN_enum;
typedef gpio_pin_enum PIN_enum;
typedef uint8 GPIOMODE_enum;

typedef enum
{
    CTIM0_P34 = 0,
    CTIM1_P35,
    CTIM2_P12,
    CTIM3_P04,
    CTIM4_P06,
} CTIMN_enum;

#define ADC_P10     ADC_CH0_P10
#define ADC_P11     ADC_CH1_P11
#define ADC_P12     ADC_CH2_P12
#define ADC_P13     ADC_CH3_P13
#define ADC_P14     ADC_CH4_P14
#define ADC_P15     ADC_CH5_P15
#define ADC_P16     ADC_CH6_P16
#define ADC_P17     ADC_CH7_P17
#define ADC_P00     ADC_CH8_P00
#define ADC_P01     ADC_CH9_P01
#define ADC_P02     ADC_CH10_P02
#define ADC_P03     ADC_CH11_P03
#define ADC_P04     ADC_CH12_P04
#define ADC_P05     ADC_CH13_P05
#define ADC_P06     ADC_CH14_P06
#define ADC_POWR    ADC_CH15_POWR

#define P0_0        IO_P00
#define P0_1        IO_P01
#define P0_2        IO_P02
#define P0_3        IO_P03
#define P0_4        IO_P04
#define P0_5        IO_P05
#define P0_6        IO_P06
#define P0_7        IO_P07
#define P2_6        IO_P26
#define P5_2        IO_P52
#define P7_4        IO_P74

#define GPO_PP      ((GPIOMODE_enum)1U)

void board_init(void);
void delay_init(void);
void delay_ms(uint16 ms);
void delay_us(uint32 us);

uint16 adc_once(ADCN_enum channel, ADCRES_enum resolution);

void pwm_duty(pwm_channel_enum channel, uint32 legacy_duty);

void gpio_mode(PIN_enum pin, GPIOMODE_enum mode);

void ctimer_count_init(CTIMN_enum timer);
void ctimer_count_clean(CTIMN_enum timer);
uint16 ctimer_count_read(CTIMN_enum timer);
void pit_timer_ms(TIMN_enum timer, uint16 period_ms);

void iap_read_bytes(uint32 address, uint8 *buffer, uint16 length);
void iap_write_bytes(uint32 address, uint8 *buffer, uint16 length);

uint32 wireless_uart_send_buff(uint8 *buffer, uint32 length);
uint32 wireless_uart_read_buff(uint8 *buffer, uint32 length);

#endif
