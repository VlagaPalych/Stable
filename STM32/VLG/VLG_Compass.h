#ifndef VLG_COMPASS_H
#define VLG_COMPASS_H

#include "stdint.h"

class VLG_Compass {
public:
    int init();

    void reg_write(uint8_t reg, uint8_t data);
    uint8_t reg_read(uint8_t reg);
    void reg_read(uint8_t reg, uint8_t *data, uint8_t size);
    int set_sample_rate(uint8_t rate);
    void timer_enable(uint8_t enable);
    void parse_raw_data(uint8_t *raw_data);

private:
    uint8_t sample_rate;
    float mag_sens_adj[3];
    int16_t compass[3];
};

#endif // VLG_COMPASS_H
