#include "VLG_InertialSensor.h"
#include "mpu9250_regs.h"
#include "spi.h"
#include "time.h"
#include "stm32f4xx.h"
#include "dma.h"
#include "main.h"

int VLG_InertialSensor::init() {
    spi2_init();
    
    /* Reset device. */
    mpu_write_byte(PWR_MGMT_1, BIT_RESET);
    delay_ms(100);

    /* Wake up chip. */
    mpu_write_byte(PWR_MGMT_1, 0x01);
    
    if (set_gyro_fsr(2000))
        return -1;
    if (set_accel_fsr(2))
        return -2;
    if (set_lpf(42))
        return -3;
    if (set_sample_rate(50))
        return -4;    
    
    mpu_dma_init();
    config_exti();
    
    return 0;
}


/**
 *  @brief      Get the gyro full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int VLG_InertialSensor::get_gyro_fsr(uint16_t *fsr) {
    switch (gyro_fsr) {
    case INV_FSR_250DPS:
        fsr[0] = 250;
        break;
    case INV_FSR_500DPS:
        fsr[0] = 500;
        break;
    case INV_FSR_1000DPS:
        fsr[0] = 1000;
        break;
    case INV_FSR_2000DPS:
        fsr[0] = 2000;
        break;
    default:
        fsr[0] = 0;
        break;
    }
    return 0;
}

/**
 *  @brief      Set the gyro full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int VLG_InertialSensor::set_gyro_fsr(uint16_t fsr) {
    uint8_t data;

    switch (fsr) {
    case 250:
        data = INV_FSR_250DPS << 3;
        break;
    case 500:
        data = INV_FSR_500DPS << 3;
        break;
    case 1000:
        data = INV_FSR_1000DPS << 3;
        break;
    case 2000:
        data = INV_FSR_2000DPS << 3;
        break;
    default:
        return -2;
    }

    if (gyro_fsr == (data >> 3))
        return 0;
    
    mpu_write_byte(GYRO_CONFIG, data);
    gyro_fsr = data >> 3;
    gyro_sens = (float)0xffff / 2 / fsr;
    return 0;
}

/**
 *  @brief      Get the accel full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int VLG_InertialSensor::get_accel_fsr(uint8_t *fsr) {
    switch (accel_fsr) {
    case INV_FSR_2G:
        fsr[0] = 2;
        break;
    case INV_FSR_4G:
        fsr[0] = 4;
        break;
    case INV_FSR_8G:
        fsr[0] = 8;
        break;
    case INV_FSR_16G:
        fsr[0] = 16;
        break;
    default:
        return -1;
    }
    return 0;
}

/**
 *  @brief      Set the accel full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int VLG_InertialSensor::set_accel_fsr(uint16_t fsr) {
    uint8_t data;

    switch (fsr) {
    case 2:
        data = INV_FSR_2G << 3;
        break;
    case 4:
        data = INV_FSR_4G << 3;
        break;
    case 8:
        data = INV_FSR_8G << 3;
        break;
    case 16:
        data = INV_FSR_16G << 3;
        break;
    default:
        return -2;
    }

    if (accel_fsr == (data >> 3))
        return 0;
    
    mpu_write_byte(ACCEL_CONFIG, data);
    accel_fsr = data >> 3;
    accel_sens = (float)0xffff / 2 / fsr;
    return 0;
}

/**
 *  @brief      Get the current DLPF setting.
 *  @param[out] lpf Current LPF setting.
 *  0 if successful.
 */
int VLG_InertialSensor::get_lpf(uint16_t *lpf) {
    switch (this->lpf) {
    case INV_FILTER_188HZ:
        lpf[0] = 188;
        break;
    case INV_FILTER_98HZ:
        lpf[0] = 98;
        break;
    case INV_FILTER_42HZ:
        lpf[0] = 42;
        break;
    case INV_FILTER_20HZ:
        lpf[0] = 20;
        break;
    case INV_FILTER_10HZ:
        lpf[0] = 10;
        break;
    case INV_FILTER_5HZ:
        lpf[0] = 5;
        break;
    case INV_FILTER_256HZ_NOLPF2:
    case INV_FILTER_2100HZ_NOLPF:
    default:
        lpf[0] = 0;
        break;
    }
    return 0;
}

/**
 *  @brief      Set digital low pass filter.
 *  The following LPF settings are supported: 188, 98, 42, 20, 10, 5.
 *  @param[in]  lpf Desired LPF setting.
 *  @return     0 if successful.
 */
int VLG_InertialSensor::set_lpf(uint16_t lpf) {
    uint8_t data;

    if (lpf >= 188)
        data = INV_FILTER_188HZ;
    else if (lpf >= 98)
        data = INV_FILTER_98HZ;
    else if (lpf >= 42)
        data = INV_FILTER_42HZ;
    else if (lpf >= 20)
        data = INV_FILTER_20HZ;
    else if (lpf >= 10)
        data = INV_FILTER_10HZ;
    else
        data = INV_FILTER_5HZ;

    if (this->lpf == data)
        return 0;
    mpu_write_byte(CONFIG, data);
    
    this->lpf = data;
    return 0;
}

/**
 *  @brief      Get sampling rate.
 *  @param[out] rate    Current sampling rate (Hz).
 *  @return     0 if successful.
 */
int VLG_InertialSensor::get_sample_rate(uint16_t *rate) {
    rate[0] = sample_rate;
    return 0;
}

/**
 *  @brief      Set sampling rate.
 *  Sampling rate must be between 4Hz and 1kHz.
 *  @param[in]  rate    Desired sampling rate (Hz).
 *  @return     0 if successful.
 */
int VLG_InertialSensor::set_sample_rate(uint16_t rate) {
    uint8_t data;

    if (rate < 4)
        rate = 4;
    else if (rate > 1000)
        rate = 1000;

    data = 1000 / rate - 1;
    mpu_write_byte(SMPLRT_DIV, data);
    
    sample_rate = 1000 / (1 + data);

    /* Automatically set LPF to 1/2 sampling rate. */
    set_lpf(sample_rate >> 1);
    return 0;
}

void VLG_InertialSensor::config_exti() {
    mpu_write_byte(INT_PIN_CFG, 0x30);
    mpu_write_byte(INT_ENABLE, 0x01);
    config_hardware_exti();
}

void VLG_InertialSensor::config_hardware_exti() {
    GPIOA->OSPEEDR |= 3 << IMU_INT*2;
    
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;
    
    EXTI->RTSR  |= EXTI_RTSR_TR1;       // rising
    EXTI->IMR   |= EXTI_IMR_MR1;        // non-masking
    NVIC_SetPriority(EXTI1_IRQn, 0x02);
    NVIC_EnableIRQ(EXTI1_IRQn);
}

void VLG_InertialSensor::parse_raw_data(uint8_t *raw_data) {
    for (uint8_t i = 0; i < 3; i++) {
        accel[i] = (int16_t)(((int16_t)raw_data[2*i] << 8) | raw_data[2*i+1]);
        gyro[i] = (int16_t)(((int16_t)raw_data[8+2*i] << 8) | raw_data[8+2*i]);
    }
}

extern "C" void EXTI1_IRQHandler() {
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR = EXTI_PR_PR1; 
        
        if (spi2_busy == SPI2_FREE) {
            spi2_busy = SPI2_IMU;
            dma_buf_tx[0] = READ_COMMAND | ACCEL_XOUT_H;
            mpu_dma_run(dma_buf_tx, dma_buf_rx, 15);
        }
    }
}
