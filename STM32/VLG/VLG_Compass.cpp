#include "VLG_Compass.h"
#include "spi.h"
#include "mpu9250_regs.h"
#include "time.h"
#include "ak8963_regs.h"
#include "stm32f4xx.h"
#include "dma.h"

#define MAG_SENS                0.15f           // uT/LSB
#define AK8963_MILLIGAUSS_SCALE 10.0f 

int VLG_Compass::init() {
    uint8_t data[3];

    // Turn auxiliary I2C on
    uint8_t user_ctrl = mpu_read_byte(USER_CTRL);
    user_ctrl |= BIT_AUX_IF_EN; 
    mpu_write_byte(USER_CTRL, user_ctrl);
    delay_ms(10);

    reg_write(AKM_REG_CNTL, AKM_POWER_DOWN);
    delay_ms(10);
    reg_write(AKM_REG_CNTL, AKM_FUSE_ROM_ACCESS);
    delay_ms(10);

    /* Get sensitivity adjustment data from fuse ROM. */
    reg_read(AKM_REG_ASAX, data, 3);
    for (uint8_t i = 0; i < 3; i++) {
        _mag_sens_adj[i] = (data[i] - 128)*0.5f / 128.0f + 1.0f;
    }

    reg_write(AKM_REG_CNTL, AKM_POWER_DOWN);
    delay_ms(10);
    mpu_write_byte(I2C_SLV0_CTRL, ~(int8_t)BIT_SLAVE_EN);
    
    set_sample_rate(100);
    _calibrator = CompassCalibrator();
    _calibrator.start();
    
    // I2C address for reading
    mpu_write_byte(I2C_SLV1_ADDR, READ_COMMAND | AK8963_I2C_ADDR);
    // reading from HXL register
    mpu_write_byte(I2C_SLV1_REG, AKM_REG_HXL);
    // Read 7 bytes
    mpu_write_byte(I2C_SLV1_CTRL, BIT_SLAVE_EN | 7);
    
    timer_enable(true);
    
    
    
    return 0;
}

void VLG_Compass::reg_write(uint8_t reg, uint8_t data) {
    mpu_write_byte(I2C_SLV0_ADDR, WRITE_COMMAND | AK8963_I2C_ADDR);
    mpu_write_byte(I2C_SLV0_REG, reg);
    mpu_write_byte(I2C_SLV0_DO, data);
    // I2C on, 1 byte
    mpu_write_byte(I2C_SLV0_CTRL, BIT_SLAVE_EN | 1);
}

uint8_t VLG_Compass::reg_read(uint8_t reg) {
    // I2C address for reading
    mpu_write_byte(I2C_SLV0_ADDR, READ_COMMAND | AK8963_I2C_ADDR);
    mpu_write_byte(I2C_SLV0_REG, reg);
    // I2C on, 1 byte
    mpu_write_byte(I2C_SLV0_CTRL, BIT_SLAVE_EN | 1);
    delay_ms(10); 
    return mpu_read_byte(EXT_SENS_DATA_00);
}

void VLG_Compass::reg_read(uint8_t reg, uint8_t *data, uint8_t size) {
    // I2C address for reading
    mpu_write_byte(I2C_SLV0_ADDR, READ_COMMAND | AK8963_I2C_ADDR);
    mpu_write_byte(I2C_SLV0_REG, reg);
    // Read 3 bytes
    mpu_write_byte(I2C_SLV0_CTRL, BIT_SLAVE_EN | size);
    delay_ms(15);
    mpu_read(EXT_SENS_DATA_00, data, 3);
}

/**
 *  @brief      Set compass sampling rate.
 *  The compass on the auxiliary I2C bus is read by the MPU hardware at a
 *  maximum of 100Hz. The actual rate can be set to a fraction of the gyro
 *  sampling rate.
 *
 *  \n WARNING: The new rate may be different than what was requested. Call
 *  mpu_get_compass_sample_rate to check the actual setting.
 *  @param[in]  rate    Desired compass sampling rate (Hz).
 *  @return     0 if successful.
 */
int VLG_Compass::set_sample_rate(uint8_t rate) {
    switch (rate) {
        case 8:
            reg_write(AKM_REG_CNTL, 0x12); // Continuous measurement mode 1 (8 Hz), 16 bit output
            break;
        case 100:
            reg_write(AKM_REG_CNTL, 0x16); // Continuous measurement mode 2 (100 Hz), 16 bit output
            break;
        default:
            return -1;
    }
    _sample_rate = rate;
    delay_ms(10);
    mpu_write_byte(I2C_SLV0_CTRL, ~(int8_t)BIT_SLAVE_EN);
    return 0;
}

void VLG_Compass::timer_enable(uint8_t enable) {
    if (enable) {
        TIM2->PSC = 15999;
        TIM2->ARR = 10;
        TIM2->DIER |= TIM_DIER_UIE;
        NVIC_SetPriority(TIM2_IRQn, 0x02);
        NVIC_EnableIRQ(TIM2_IRQn);
        TIM2->CR1 |= TIM_CR1_CEN;
    } else {
        TIM2->CR1 &= ~TIM_CR1_CEN;
        TIM2->DIER &= ~TIM_DIER_UIE;
        NVIC_DisableIRQ(TIM2_IRQn);
    }
}

void VLG_Compass::parse_raw_data(uint8_t *raw_data) {
    int16_t lsb_val  = 0;
    lsb_val = (int16_t)(raw_data[0] | ((int16_t)raw_data[1] << 8));
    _raw_field.x = lsb_val * _mag_sens_adj[0] * MAG_SENS * AK8963_MILLIGAUSS_SCALE;
    
    lsb_val = (int16_t)(raw_data[2] | ((int16_t)raw_data[3] << 8));
    _raw_field.y = lsb_val * _mag_sens_adj[1] * MAG_SENS * AK8963_MILLIGAUSS_SCALE;
    
    lsb_val = (int16_t)(raw_data[4] | ((int16_t)raw_data[5] << 8));
    _raw_field.z = lsb_val * _mag_sens_adj[2] * MAG_SENS * AK8963_MILLIGAUSS_SCALE;
    
    _fresh_data = true;
    _calibrator.new_sample(_raw_field);
}

extern "C" void TIM2_IRQHandler() {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        
        if (spi2_busy == SPI2_FREE) {
            spi2_busy = SPI2_MAG;
            dma_buf_tx[0] = READ_COMMAND | EXT_SENS_DATA_00;
            mpu_dma_run(dma_buf_tx, dma_buf_rx, 8);
        }
    }
}

bool VLG_Compass::fresh_data() {
    return _fresh_data;
}

void VLG_Compass::update_calibr() {
    bool failure;
    _calibrator.update(failure);
    _fresh_data = false;
}
