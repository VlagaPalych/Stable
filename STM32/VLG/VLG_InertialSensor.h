#ifndef VLG_INERTIAL_SENSOR
#define VLG_INERTIAL_SENSOR

#include "stdint.h"

#define DMP_CODE_SIZE 3062

struct dmp_state_t {
        uint16_t orient;
        uint16_t feature_mask;
        uint16_t fifo_rate;
        uint8_t packet_length;
    };

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

    void parse_quat_accel_gyro(uint8_t *raw_data);

    int dmp_load_firmware();
    int dmp_set_fifo_rate(uint16_t rate);
    int dmp_set_state(bool enable);
    int dmp_enable_feature(uint16_t mask);

    int set_int_enable(bool enable);
    int reset_fifo(void);
    int configure_fifo(uint8_t sensors);
    int get_fifo_config(uint8_t *sensors);
    
    int set_sensors(uint8_t sensors);

    dmp_state_t _dmp_state;
private:
    void config_exti();
    void config_hardware_exti();

    int mem_write(uint16_t mem_addr, uint8_t *data, uint16_t length);
    int mem_read(uint16_t mem_addr, uint8_t *data, uint16_t length);

    int dmp_enable_gyro_cal(uint8_t enable);
    int dmp_enable_lp_quat(uint8_t enable);
    int dmp_enable_6x_lp_quat(uint8_t enable);

    uint8_t gyro_fsr;
    float gyro_sens;

    uint8_t accel_fsr;
    float accel_sens;

    uint8_t lpf;
    uint16_t sample_rate;

    long raw_quat[4];
    int16_t raw_accel[3];
    int16_t raw_gyro[3];

    static const uint8_t _dmp_memory[DMP_CODE_SIZE];
    bool _dmp_loaded;
    bool _dmp_on;
    uint16_t _dmp_sample_rate;
    uint8_t _fifo_enable;
    uint8_t _int_enable;
    uint8_t _sensors;
    uint8_t _clk_src;
};

#endif // VLG_INERTIAL_SENSOR
