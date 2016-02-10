#ifndef FILTERS_H
#define FILTERS_H

#include "arm_math.h"
#include "adxrs290.h"


#define ACCEL_DECIMATION 16
#define ACCEL_FILTER_SIZE 107   

extern arm_fir_decimate_instance_f32 accel_lpf[3];
extern float accel_lpf_state[3][ACCEL_FILTER_SIZE + ACCEL_DECIMATION - 1];
extern float accel_lpf_coeffs[ACCEL_FILTER_SIZE];
extern float accel_history[3][2*ACCEL_FILTER_SIZE];
extern uint8_t accel_history_index;
extern uint8_t accel_history_filter_index;
extern float filtered_a[3];


#define ADXRS453_DECIMATION 4
#define ADXRS453_FILTER_SIZE 90

extern arm_fir_decimate_instance_f32 adxrs453_lpf;
extern float adxrs453_lpf_state[ADXRS453_FILTER_SIZE + ADXRS453_DECIMATION - 1];
extern float adxrs453_lpf_coeffs[ADXRS453_FILTER_SIZE];
extern float adxrs453_history[2*ADXRS453_FILTER_SIZE];
extern uint8_t adxrs453_history_index;
extern uint8_t adxrs453_history_filter_index;
extern float filtered_arz;


#define ADXRS290_DECIMATION ACCEL_DECIMATION
#define ADXRS290_FILTER_SIZE ACCEL_FILTER_SIZE

extern arm_fir_decimate_instance_f32 adxrs290_lpf[2];
extern float adxrs290_lpf_state[2][ADXRS290_FILTER_SIZE + ADXRS290_DECIMATION - 1];
extern float adxrs290_lpf_coeffs[];
extern float adxrs290_history[2][2*ADXRS290_FILTER_SIZE];
extern uint16_t adxrs290_history_index;
extern uint16_t adxrs290_history_filter_index;
extern float filtered_ar[2];    

extern uint8_t adxrs290_lowpass_ready;
extern uint8_t adxrs290_process_index;
extern uint8_t adxrs290_do_process;


#define QUASISTATIC_HPF_SIZE 101

extern arm_fir_instance_f32 quasistatic_hpf;
extern float quasistatic_hpf_state[QUASISTATIC_HPF_SIZE];
extern float quasistatic_hpf_coeffs[QUASISTATIC_HPF_SIZE];
extern float accel_modulo_history[2*QUASISTATIC_HPF_SIZE];
extern uint8_t accel_modulo_history_index;


#define QUASISTATIC_LPF_SIZE 120

extern arm_fir_instance_f32 quasistatic_lpf;
extern float quasistatic_lpf_state[QUASISTATIC_LPF_SIZE];
extern float quasistatic_lpf_coeffs[QUASISTATIC_LPF_SIZE];
extern float accel_modulo_highpassed_history[2*QUASISTATIC_LPF_SIZE];
extern uint8_t accel_modulo_highpassed_history_index;

#endif
