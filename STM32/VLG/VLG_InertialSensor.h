#ifndef VLG_INERTIAL_SENSOR
#define VLG_INERTIAL_SENSOR

#include "stdint.h"

class VLG_InertialSensor {
public:
    int init();

    int get_gyro_fsr(uint16_t *fsr);
    int set_gyro_fsr(uint16_t fsr);

    int get_accel_fsr(uint8_t *fsr);
    int set_accel_fsr(uint16_t fsr);

    int get_lpf(uint16_t *lpf);
    int set_lpf(uint16_t lpf);

    int get_sample_rate(uint16_t *rate);
    int set_sample_rate(uint16_t rate);

    void parse_raw_data(uint8_t *raw_data);

private:
    void config_exti();
    void config_hardware_exti();

    uint8_t gyro_fsr;
    float gyro_sens;

    uint8_t accel_fsr;
    float accel_sens;

    uint8_t lpf;
    uint16_t sample_rate;

    int16_t accel[3];
    int16_t gyro[3];
};

#endif // VLG_INERTIAL_SENSOR
