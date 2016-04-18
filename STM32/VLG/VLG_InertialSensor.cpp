#include "VLG_InertialSensor.h"
#include "mpu9250_regs.h"
#include "spi.h"
#include "time.h"
#include "stm32f4xx.h"
#include "dma.h"
#include "main.h"
#include "dmp.h"

int VLG_InertialSensor::init() {
    _sensors = INV_XYZ_ACCEL | INV_XYZ_GYRO | INV_XYZ_COMPASS;
    
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
    if (configure_fifo(_sensors))
        return -5;    
    
    //set_sensors(0);
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

/**
 *  @brief      Enable/disable data ready interrupt.
 *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data ready
 *  interrupt is used.
 *  @param[in]  enable      1 to enable interrupt.
 *  @return     0 if successful.
 */
int VLG_InertialSensor::set_int_enable(bool enable) {
    uint8_t tmp;

    if (_dmp_on) {
        if (enable)
            tmp = BIT_DMP_INT_EN;
        else
            tmp = 0x00;
        mpu_write_byte(INT_ENABLE, tmp);
        _int_enable = tmp;
    } else {
        if (enable && _int_enable)
            return 0;
        if (enable)
            tmp = BIT_DATA_RDY_EN;
        else
            tmp = 0x00;
        mpu_write_byte(INT_ENABLE, tmp);
        _int_enable = tmp;
    }
    return 0;
}

void VLG_InertialSensor::parse_quat_accel_gyro(uint8_t *raw_data) {
    int32_t index = 0;
    if (_dmp_state.feature_mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT)) {
        for (uint8_t i = 0; i < 4; i++) {
            raw_quat[i] = (long)(((long)raw_data[4*i] << 24) | ((long)raw_data[4*i+1] << 16) |
                    ((long)raw_data[4*i+2] << 8) | raw_data[4*i+3]);
        }
        index += 16;
    }
    if (_dmp_state.feature_mask & DMP_FEATURE_SEND_RAW_ACCEL) {
        for (uint8_t i = 0; i < 3; i++) {
            raw_accel[i] = (int16_t)(((int16_t)raw_data[index+2*i] << 8) | raw_data[index+2*i+1]);
        }
        index += 6;
    }
    if (_dmp_state.feature_mask & DMP_FEATURE_SEND_ANY_GYRO) {
        for (uint8_t i = 0; i < 3; i++) {
            raw_gyro[i] = (int16_t)(((int16_t)raw_data[index+2*i] << 8) | raw_data[index+2*i]);
        }
    }
}

extern "C" void EXTI1_IRQHandler() {
//    uint16_t fifo_count = 0;
//    uint8_t tmp[2];
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR = EXTI_PR_PR1; 
        
        if (spi2_busy == SPI2_FREE) {
            spi2_busy = SPI2_IMU;
            
//            mpu_read(FIFO_COUNTH, tmp, 2);
//            fifo_count = (tmp[0] << 8) | tmp[1];
//            
//            if (fifo_count) {
                dma_buf_tx[0] = READ_COMMAND | FIFO_R_W;
                mpu_dma_run(dma_buf_tx, dma_buf_rx, inertial_sensor._dmp_state.packet_length + 1);
            //}
        }
    }
}

/**
 *  @brief  Reset FIFO read/write pointers.
 *  @return 0 if successful.
 */
int VLG_InertialSensor::reset_fifo(void) {
    uint8_t data;

    data = 0;
    mpu_write_byte(INT_ENABLE, 0x00);
    mpu_write_byte(FIFO_EN, 0x00);
    mpu_write_byte(USER_CTRL, 0x00);

    if (_dmp_on) {
        mpu_write_byte(USER_CTRL, BIT_FIFO_RST | BIT_DMP_RST);
        delay_ms(50);
        data = BIT_DMP_EN | BIT_FIFO_EN;
        if (_sensors & INV_XYZ_COMPASS)
            data |= BIT_AUX_IF_EN;
        mpu_write_byte(USER_CTRL, data);
        if (_int_enable)
            data = BIT_DMP_INT_EN;
        else
            data = 0;
        mpu_write_byte(INT_ENABLE, data);
        mpu_write_byte(FIFO_EN, 0x00);
    } else {
        mpu_write_byte(USER_CTRL, BIT_FIFO_RST);
        if (!(_sensors & INV_XYZ_COMPASS))
            data = BIT_FIFO_EN;
        else
            data = BIT_FIFO_EN | BIT_AUX_IF_EN;
        mpu_write_byte(USER_CTRL, data);
        delay_ms(50);
        if (_int_enable)
            data = BIT_DATA_RDY_EN;
        else
            data = 0;
        mpu_write_byte(INT_ENABLE,data);
        mpu_write_byte(FIFO_EN, _fifo_enable);
    }
    return 0;
}

/**
 *  @brief      Get current FIFO configuration.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[out] sensors Mask of sensors in FIFO.
 *  @return     0 if successful.
 */
int VLG_InertialSensor::get_fifo_config(uint8_t *sensors) {
    sensors[0] = _fifo_enable;
    return 0;
}

/**
 *  @brief      Select which sensors are pushed to FIFO.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[in]  sensors Mask of sensors to push to FIFO.
 *  @return     0 if successful.
 */
int VLG_InertialSensor::configure_fifo(uint8_t sensors) {
    uint8_t prev;
    int result = 0;

    if (_dmp_on)
        return 0;
    else {
        prev = _fifo_enable;
        _fifo_enable = sensors & _sensors;
        if (_fifo_enable != sensors)
            /* You're not getting what you asked for. Some sensors are
             * asleep.
             */
            result = -1;
        else
            result = 0;
        if (sensors)
            set_int_enable(1);
        else
            set_int_enable(0);
        if (sensors) {
            if (reset_fifo()) {
                _fifo_enable = prev;
                return -1;
            }
        }
    }
    return result;
}

/**
 *  @brief      Turn specific sensors on/off.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_XYZ_COMPASS
 *  @param[in]  sensors    Mask of sensors to wake.
 *  @return     0 if successful.
 */
int VLG_InertialSensor::set_sensors(uint8_t sensors)  {
    uint8_t data;
    uint8_t user_ctrl;

    if (sensors & INV_XYZ_GYRO)
        data = INV_CLK_PLL;
    else if (sensors)
        data = 0;
    else
        data = BIT_SLEEP;
    mpu_write_byte(PWR_MGMT_1, data);
    
    _clk_src = data & ~BIT_SLEEP;

    data = 0;
    if (!(sensors & INV_X_GYRO))
        data |= BIT_STBY_XG;
    if (!(sensors & INV_Y_GYRO))
        data |= BIT_STBY_YG;
    if (!(sensors & INV_Z_GYRO))
        data |= BIT_STBY_ZG;
    if (!(sensors & INV_XYZ_ACCEL))
        data |= BIT_STBY_XYZA;
    mpu_write_byte(PWR_MGMT_2, data);

    user_ctrl = mpu_read_byte(USER_CTRL);
    /* Handle AKM power management. */
    if (sensors & INV_XYZ_COMPASS) {
        user_ctrl |= BIT_AUX_IF_EN;
    } else {
        user_ctrl &= ~BIT_AUX_IF_EN;
    }
    if (_dmp_on)
        user_ctrl |= BIT_DMP_EN;
    else
        user_ctrl &= ~BIT_DMP_EN;

    /* Enable/disable I2C master mode. */
    mpu_write_byte(USER_CTRL, user_ctrl);
    _sensors = sensors;
    delay_ms(50);
    return 0;
}
