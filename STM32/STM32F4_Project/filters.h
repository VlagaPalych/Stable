#ifndef FILTERS_H
#define FILTERS_H

#include "arm_math.h"
#include "adxrs290.h"

#define ADXRS290_NUM_TAPS       416
#define ADXRS290_DECIMATION     42

extern arm_fir_decimate_instance_f32 adxrs290_lpf[ADXRS290_NUMBER];
extern float adxrs290_lpf_cf[];
extern float ars_state[ADXRS290_NUMBER][ADXRS290_NUM_TAPS + ADXRS290_DECIMATION - 1]; 


#endif