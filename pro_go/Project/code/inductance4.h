#ifndef __INDUCTANCE4_H_
#define __INDUCTANCE4_H_

#include "common.h"
#include "zf_adc.h"

#define INDUCTANCE4_CHANNEL_COUNT 4
#define INDUCTANCE4_SAMPLE_COUNT  5
#define INDUCTANCE4_NORM_MAX      128

typedef enum
{
    INDUCTANCE4_L = 0,
    INDUCTANCE4_LM,
    INDUCTANCE4_RM,
    INDUCTANCE4_R
} Inductance4Channel;

typedef struct
{
    ADCN_enum adc_channel;
    uint16 raw;
    uint16 filtered;
    uint16 min_value;
    uint16 max_value;
    int16 normalized;
    uint8 healthy;
} Inductance4Sensor;

extern Inductance4Sensor g_inductance4[INDUCTANCE4_CHANNEL_COUNT];

extern float inductance_error_a;
extern float inductance_error_b;
extern float inductance_error_c;
extern float inductance_error_p;

extern uint8 inductance4_calibration_active;
extern uint8 inductance4_calibration_valid;
extern uint8 inductance4_calibration_failed_channel;

void inductance4_init(void);
void inductance4_update(void);

uint16 inductance4_trimmed_average(ADCN_enum channel);
int16 inductance4_normalize(uint16 value, uint16 max_value, uint16 min_value);
float inductance4_calculate_error(void);

void inductance4_calibration_start(void);
void inductance4_guided_calibration_start(void);
void inductance4_calibration_capture_current(void);
void inductance4_calibration_capture_values(const uint16 *values);
void inductance4_calibration_cancel(void);
uint8 inductance4_calibration_finish(void);
void inductance4_get_calibration_extrema(uint16 *min_values, uint16 *max_values);
void inductance4_set_calibration(const uint16 *min_values, const uint16 *max_values);
void inductance4_get_calibration(uint16 *min_values, uint16 *max_values);
uint8 inductance4_check_health(void);
uint8 inductance4_load_config(void);
uint8 inductance4_save_config(void);

#endif
