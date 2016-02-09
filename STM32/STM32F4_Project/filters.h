#ifndef FILTERS_H
#define FILTERS_H

#include "arm_math.h"
#include "adxrs290.h"

#define ADXRS290_NUM_TAPS       416
#define ADXRS290_DECIMATION     42

extern arm_fir_decimate_instance_f32 adxrs290_lpf[ADXRS290_NUMBER];
extern float adxrs290_lpf_cf[];
extern float ars_state[ADXRS290_NUMBER][ADXRS290_NUM_TAPS + ADXRS290_DECIMATION - 1]; 






#define ACCEL_DECIMATION 16
#define ACCEL_FILTER_SIZE 107   

extern arm_fir_decimate_instance_f32 accel_lpf[3];
extern float accel_lpf_state[3][ACCEL_FILTER_SIZE + ACCEL_DECIMATION - 1];
extern float accel_lpf_coeffs[ACCEL_FILTER_SIZE];
extern float accel_history[3][2*ACCEL_FILTER_SIZE];
extern uint8_t accel_history_index;
extern uint8_t accel_history_filter_index;
extern float filtered_a[3];


#endif