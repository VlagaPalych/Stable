#include "stm32f4xx.h" 
#include "mpu9250.h"
#include "dmp.h"
#include "string.h"
#include "extra_math.h"
#include "spi.h"
#include "dma.h"
#include "stdlib.h"
#include "math.h"


#define min(a,b) ((a<b)?a:b)
void Delay_ms(uint16_t ms);
void Delay_us(uint16_t us);

#define MPU_READ_DATA_SIZE 13
uint8_t MPU_DMA_tx[100] = {READ_COMMAND | FIFO_R_W};
uint8_t MPU_DMA_rx[100];

int16_t accel_data[3];
int16_t gyro_data[3];
int32_t quat_data[4];

float accel[3];
float quat[4];
float temp;
float angleRate[3];
float magField[3];

extern float w1[3];
extern float w2[3];
extern float v1[3];
extern float v2[3];

MPU_Regs regArray[MPU_MAX_DEVICES];
HW hwArray[MPU_MAX_DEVICES];
MPU_Test testArray[MPU_MAX_DEVICES];
MPU_State gyroArray[MPU_MAX_DEVICES];

MPU_Regs *reg;
HW *hw;
MPU_Test *test;
MPU_State *st;
static int deviceIndex = 0;

static int reg_int_cb(MPU_IntParams *int_param) {
    int_param->active_low = 0;
    int_param->pin = 1;
    return 0;
}

#ifdef AK89xx_SECONDARY
static int setup_compass(void);
#define MAX_COMPASS_SAMPLE_RATE (100)
#endif

int MPU_SelectDevice(int device) {
  if ((device < 0) || (device >= MPU_MAX_DEVICES))
    return -1;

  deviceIndex = device;
  reg = regArray + device;
  hw = hwArray + device;
  test = testArray + device;
  st = gyroArray + device;
  return 0;
}

void MPU_InitStructures() {
    reg->who_am_i         = WHO_AM_I;
    reg->rate_div         = SMPLRT_DIV;
    reg->lpf              = CONFIG;
    reg->prod_id          = 0x0C;
    reg->user_ctrl        = USER_CTRL;
    reg->fifo_en          = FIFO_EN;
    reg->gyro_cfg         = GYRO_CONFIG;
    reg->accel_cfg        = ACCEL_CONFIG;
    reg->accel_cfg2       = ACCEL_CONFIG2;
    reg->lp_accel_odr     = LP_ACCEL_ODR;
    reg->motion_thr       = WOM_THR;
    reg->motion_dur       = 0x20;
    reg->fifo_count_h     = FIFO_COUNTH;
    reg->fifo_r_w         = FIFO_R_W;
    reg->raw_gyro         = GYRO_XOUT_H;
    reg->raw_accel        = ACCEL_XOUT_H;
    reg->temp             = TEMP_OUT_H;
    reg->int_enable       = INT_ENABLE;
    reg->dmp_int_status   = 0x39;
    reg->int_status       = INT_STATUS;
    reg->accel_intel      = MOT_DETECT_CTRL;
    reg->pwr_mgmt_1       = PWR_MGMT_1;
    reg->pwr_mgmt_2       = PWR_MGMT_2;
    reg->int_pin_cfg      = INT_PIN_CFG;
    reg->mem_r_w          = MEM_R_W;
    reg->accel_offs       = XA_OFFSET_H;
    reg->i2c_mst          = I2C_MST_CTRL;
    reg->bank_sel         = BANK_SEL;
    reg->mem_start_addr   = 0x6E;
    reg->prgm_start_h     = PRGM_START_H;
#ifdef AK89xx_SECONDARY
    reg->raw_compass      = EXT_SENS_DATA_00;
    reg->yg_offs_tc       = YG_OFFSET_H;
    reg->s0_addr          = I2C_SLV0_ADDR;
    reg->s0_reg           = I2C_SLV0_REG;
    reg->s0_ctrl          = I2C_SLV0_CTRL;
    reg->s1_addr          = I2C_SLV1_ADDR;
    reg->s1_reg           = I2C_SLV1_REG;
    reg->s1_ctrl          = I2C_SLV1_CTRL;
    reg->s4_ctrl          = I2C_SLV4_CTRL;
    reg->s0_do            = I2C_SLV0_DO;
    reg->s1_do            = I2C_SLV1_DO;
    reg->i2c_delay_ctrl   = I2C_MST_DELAY_CTRL;
#endif
    switch (deviceIndex) {
      case 0:
        hw->addr = 0x68;
        break;

      case 1:
        hw->addr = 0x69;
        break;
    }
    hw->max_fifo          = 1024;
    hw->num_reg           = 128;
    hw->temp_sens         = 321;
    hw->temp_offset       = 0;
    hw->bank_size         = 256;
#if defined AK89xx_SECONDARY
    hw->compass_fsr      = AK89xx_FSR;
#endif

    test->gyro_sens      = 32768/250;
    test->accel_sens     = 32768/16;
    test->reg_rate_div   = 0;    /* 1kHz. */
    test->reg_lpf        = 1;    /* 188Hz. */
    test->reg_gyro_fsr   = 0;    /* 250dps. */
    test->reg_accel_fsr  = 0x18; /* 16g. */
    test->wait_ms        = 50;
    test->packet_thresh  = 5;    /* 5% */
    test->min_dps        = 10.f;
    test->max_dps        = 105.f;
    test->max_gyro_var   = 0.14f;
    test->min_g          = 0.3f;
    test->max_g          = 0.95f;
    test->max_accel_var  = 0.14f;

    st->reg = reg;
    st->hw = hw;
    st->test = test;
};

// 0 if check passed
int MPU_WriteByteAndCheck(uint8_t address, uint8_t data) {
    uint8_t test;
    MPU_WriteByte(address, data);
    test = MPU_ReadByte(address);
    return (test != data);
}

int MPU_WriteAndCheck(uint8_t address, uint8_t *data, uint8_t size) {
    uint8_t *test = (uint8_t *)malloc(size);
    int res = 0;
    MPU_Write(address, data, size);
    MPU_Read(address, test, size);
    if (memcmp(data, test, size)) {
        res = -1;
    }
    free(test);
    return res;
}

/**
 *  @brief      Write to the DMP memory.
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data        Bytes to write to memory.
 *  @return     0 if successful.
 */
int MPU_MemWrite(uint16_t mem_addr, uint8_t *data, uint16_t length) {
    uint8_t tmp[2];

    if (!data)
        return -1;
    if (!st->chip_cfg.sensors)
        return -2;

    tmp[0] = (uint8_t)(mem_addr >> 8);
    tmp[1] = (uint8_t)(mem_addr & 0xFF);

    /* Check bank boundaries. */
    if (tmp[1] + length > st->hw->bank_size)
        return -3;

    MPU_Write(st->reg->bank_sel, tmp, 2);
    MPU_Write(st->reg->mem_r_w, data, length);
    return 0;
}

/**
 *  @brief      Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Bytes read from memory.
 *  @return     0 if successful.
 */
int MPU_MemRead(uint16_t mem_addr, uint8_t *data, uint16_t length) {
    uint8_t tmp[2];

    if (!data)
        return -1;
    if (!st->chip_cfg.sensors)
        return -2;

    tmp[0] = (uint8_t)(mem_addr >> 8);
    tmp[1] = (uint8_t)(mem_addr & 0xFF);

    /* Check bank boundaries. */
    if (tmp[1] + length > st->hw->bank_size)
        return -3;

    MPU_Write(st->reg->bank_sel, tmp, 2);
    MPU_Read(st->reg->mem_r_w, data, length);
    return 0;
}

int MPU_Init(MPU_IntParams *int_param) {
    uint8_t data[6], rev;
    int errCode;

    /* Reset device. */
    MPU_WriteByte(st->reg->pwr_mgmt_1, BIT_RESET);
    Delay_ms(100);

    /* Wake up chip. */
    MPU_WriteByte(st->reg->pwr_mgmt_1, 0x01);

#if defined MPU6050
    /* Check product revision. */
    MPU_Read(st->reg->accel_offs, data, 6);
    rev = ((data[5] & 0x01) << 2) | ((data[3] & 0x01) << 1) |
        (data[1] & 0x01);

    if (rev) {
        /* Congrats, these parts are better. */
        if (rev == 1)
            st->chip_cfg.accel_half = 1;
        else if (rev == 2)
            st->chip_cfg.accel_half = 0;
        else {
            return -4;
        }
    } else {
        MPU_ReadByte(st->reg->prod_id, &data[0]);
        rev = data[0] & 0x0F;
        if (!rev) {
            return -6;
        } else if (rev == 4) {
            st->chip_cfg.accel_half = 1;
        } else
            st->chip_cfg.accel_half = 0;
    }
#elif defined MPU6500
#define MPU6500_MEM_REV_ADDR    (0x17)
    MPU_MemRead(MPU6500_MEM_REV_ADDR, &rev, 1);
    if (rev == 0x1)
        st->chip_cfg.accel_half = 0;
    else {
        return -8;
    }

    /* MPU6500 shares 4kB of memory between the DMP and the FIFO. Since the
     * first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
     */
    data[0] = ;
    MPU_WriteByte(st->reg->accel_cfg2, 1, BIT_FIFO_SIZE_1024 | 0x8);
#endif

    /* Set to invalid values to ensure no I2C writes are skipped. */
    st->chip_cfg.sensors = 0xFF;
    st->chip_cfg.gyro_fsr = 0xFF;
    st->chip_cfg.accel_fsr = 0xFF;
    st->chip_cfg.lpf = 0xFF;
    st->chip_cfg.sample_rate = 0xFFFF;
    st->chip_cfg.fifo_enable = 0xFF;
    st->chip_cfg.bypass_mode = 0xFF;
#ifdef AK89xx_SECONDARY
    st->chip_cfg.compass_sample_rate = 0xFFFF;
#endif
    /* mpu_set_sensors always preserves this setting. */
    st->chip_cfg.clk_src = INV_CLK_PLL;
    /* Handled in next call to mpu_set_bypass. */
    st->chip_cfg.active_low_int = 0;
    st->chip_cfg.latched_int = 1;
    st->chip_cfg.int_motion_only = 0;
    st->chip_cfg.lp_accel_mode = 0;
    memset(&st->chip_cfg.cache, 0, sizeof(st->chip_cfg.cache));
    st->chip_cfg.dmp_on = 0;
    st->chip_cfg.dmp_loaded = 0;
    st->chip_cfg.dmp_sample_rate = 0;

    if (MPU_SetGyroFsr(2000))
        return -10;
    if (MPU_SetAccelFsr(2))
        return -11;
    if (MPU_SetLPF(42))
        return -12;
    if (MPU_SetSampleRate(50))
        return -13;
    if (MPU_ConfigureFIFO(0))
        return -14;

    if (int_param)
        reg_int_cb(int_param);

#ifdef AK89xx_SECONDARY
    setup_compass();
    MPU_SetCompassSampleRate(100);
#endif

    MPU_SetSensors(0);
    return 0;
}

/**
 *  @brief      Get the gyro full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int MPU_GetGyroFsr(uint16_t *fsr) {
    switch (st->chip_cfg.gyro_fsr) {
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
int MPU_SetGyroFsr(uint16_t fsr) {
    uint8_t data;

    if (!(st->chip_cfg.sensors))
        return -1;

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

    if (st->chip_cfg.gyro_fsr == (data >> 3))
        return 0;
    st->chip_cfg.gyro_sens = (float)0xffff / 2 / fsr;
    if (MPU_WriteByteAndCheck(st->reg->gyro_cfg, data)) {
        return -3;
    }
    
    st->chip_cfg.gyro_fsr = data >> 3;
    return 0;
}

/**
 *  @brief      Get the accel full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int MPU_GetAccelFsr(uint8_t *fsr) {
    switch (st->chip_cfg.accel_fsr) {
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
    if (st->chip_cfg.accel_half)
        fsr[0] <<= 1;
    return 0;
}

/**
 *  @brief      Set the accel full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int MPU_SetAccelFsr(uint16_t fsr) {
    uint8_t data;

    if (!(st->chip_cfg.sensors))
        return -1;

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

    if (st->chip_cfg.accel_fsr == (data >> 3))
        return 0;
    st->chip_cfg.accel_sens = (float)0xffff / 2 / fsr;
    if (MPU_WriteByteAndCheck(st->reg->accel_cfg, data)) {
        return -3;
    }
    
    st->chip_cfg.accel_fsr = data >> 3;
    return 0;
}

/**
 *  @brief      Get the current DLPF setting.
 *  @param[out] lpf Current LPF setting.
 *  0 if successful.
 */
int MPU_GetLPF(uint16_t *lpf) {
    switch (st->chip_cfg.lpf) {
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
int MPU_SetLPF(uint16_t lpf) {
    uint8_t data;
    if (!(st->chip_cfg.sensors))
        return -1;

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

    if (st->chip_cfg.lpf == data)
        return 0;
    if (MPU_WriteByteAndCheck(st->reg->lpf, data)) {
        return -3;
    }
    
    st->chip_cfg.lpf = data;
    return 0;
}

/**
 *  @brief      Get sampling rate.
 *  @param[out] rate    Current sampling rate (Hz).
 *  @return     0 if successful.
 */
int MPU_GetSampleRate(uint16_t *rate) {
    if (st->chip_cfg.dmp_on)
        return -1;
    else
        rate[0] = st->chip_cfg.sample_rate;
    return 0;
}

/**
 *  @brief      Set sampling rate.
 *  Sampling rate must be between 4Hz and 1kHz.
 *  @param[in]  rate    Desired sampling rate (Hz).
 *  @return     0 if successful.
 */
int MPU_SetSampleRate(uint16_t rate) {
    uint8_t data;

    if (!(st->chip_cfg.sensors))
        return -1;

    if (st->chip_cfg.dmp_on)
        return -1;
    else {
        if (st->chip_cfg.lp_accel_mode) {
            if (rate && (rate <= 40)) {
                /* Just stay in low-power accel mode. */
                MPU_LowPowerAccelMode(rate);
                return 0;
            }
            /* Requested rate exceeds the allowed frequencies in LP accel mode,
             * switch back to full-power mode.
             */
            MPU_LowPowerAccelMode(0);
        }
        if (rate < 4)
            rate = 4;
        else if (rate > 1000)
            rate = 1000;

        data = 1000 / rate - 1;
        if (MPU_WriteByteAndCheck(st->reg->rate_div, data)) {
            return -3;
        }
        
        st->chip_cfg.sample_rate = 1000 / (1 + data);

#ifdef AK89xx_SECONDARY
        MPU_SetCompassSampleRate(min(st->chip_cfg.compass_sample_rate, MAX_COMPASS_SAMPLE_RATE));
#endif

        /* Automatically set LPF to 1/2 sampling rate. */
        MPU_SetLPF(st->chip_cfg.sample_rate >> 1);
        return 0;
    }
}

/**
 *  @brief      Enter low-power accel-only mode.
 *  In low-power accel mode, the chip goes to sleep and only wakes up to sample
 *  the accelerometer at one of the following frequencies:
 *  \n MPU6050: 1.25Hz, 5Hz, 20Hz, 40Hz
 *  \n MPU6500: 1.25Hz, 2.5Hz, 5Hz, 10Hz, 20Hz, 40Hz, 80Hz, 160Hz, 320Hz, 640Hz
 *  \n If the requested rate is not one listed above, the device will be set to
 *  the next highest rate. Requesting a rate above the maximum supported
 *  frequency will result in an error.
 *  \n To select a fractional wake-up frequency, round down the value passed to
 *  @e rate.
 *  @param[in]  rate        Minimum sampling rate, or zero to disable LP
 *                          accel mode.
 *  @return     0 if successful.
 */
int MPU_LowPowerAccelMode(uint8_t rate) {
    uint8_t tmp[2];

    if (rate > 40)
        return -1;

    if (!rate) {
        MPU_SetIntLatched(0);
        tmp[0] = 0;
        tmp[1] = BIT_STBY_XYZG;
        MPU_Write(st->reg->pwr_mgmt_1, tmp, 2);
        st->chip_cfg.lp_accel_mode = 0;
        return 0;
    }
    /* For LP accel, we automatically configure the hardware to produce latched
     * interrupts. In LP accel mode, the hardware cycles into sleep mode before
     * it gets a chance to deassert the interrupt pin; therefore, we shift this
     * responsibility over to the MCU.
     *
     * Any register read will clear the interrupt.
     */
    MPU_SetIntLatched(1);
#if defined MPU6050
    tmp[0] = BIT_LPA_CYCLE;
    if (rate == 1) {
        tmp[1] = INV_LPA_1_25HZ;
        mpu_set_lpf(5);
    } else if (rate <= 5) {
        tmp[1] = INV_LPA_5HZ;
        mpu_set_lpf(5);
    } else if (rate <= 20) {
        tmp[1] = INV_LPA_20HZ;
        mpu_set_lpf(10);
    } else {
        tmp[1] = INV_LPA_40HZ;
        mpu_set_lpf(20);
    }
    tmp[1] = (tmp[1] << 6) | BIT_STBY_XYZG;
    if (i2c_write(st->hw->addr, st->reg->pwr_mgmt_1, 2, tmp))
        return -1;
#elif defined MPU6500
    /* Set wake frequency. */
    if (rate == 1)
        tmp[0] = INV_LPA_1_25HZ;
    else if (rate == 2)
        tmp[0] = INV_LPA_2_5HZ;
    else if (rate <= 5)
        tmp[0] = INV_LPA_5HZ;
    else if (rate <= 10)
        tmp[0] = INV_LPA_10HZ;
    else if (rate <= 20)
        tmp[0] = INV_LPA_20HZ;
    else if (rate <= 40)
        tmp[0] = INV_LPA_40HZ;
    else if (rate <= 80)
        tmp[0] = INV_LPA_80HZ;
    else if (rate <= 160)
        tmp[0] = INV_LPA_160HZ;
    else if (rate <= 320)
        tmp[0] = INV_LPA_320HZ;
    else
        tmp[0] = INV_LPA_640HZ;
    if (i2c_write(st->hw->addr, st->reg->lp_accel_odr, 1, tmp))
        return -1;
    tmp[0] = BIT_LPA_CYCLE;
    if (i2c_write(st->hw->addr, st->reg->pwr_mgmt_1, 1, tmp))
        return -1;
#endif
    st->chip_cfg.sensors = INV_XYZ_ACCEL;
    st->chip_cfg.clk_src = 0;
    st->chip_cfg.lp_accel_mode = 1;
    MPU_ConfigureFIFO(0);

    return 0;
}

/**
 *  @brief      Enable latched interrupts.
 *  Any MPU register will clear the interrupt.
 *  @param[in]  enable  1 to enable, 0 to disable.
 *  @return     0 if successful.
 */
int MPU_SetIntLatched(uint8_t enable) {
    uint8_t tmp;
    if (st->chip_cfg.latched_int == enable)
        return 0;

    if (enable)
        tmp = BIT_LATCH_EN | BIT_ANY_RD_CLR;
    else
        tmp = 0;
    if (st->chip_cfg.bypass_mode)
        tmp |= BIT_BYPASS_EN;
    if (st->chip_cfg.active_low_int)
        tmp |= BIT_ACTL;
    MPU_WriteByte(st->reg->int_pin_cfg, tmp);
    st->chip_cfg.latched_int = enable;
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
int MPU_GetFIFOConfig(uint8_t *sensors) {
    sensors[0] = st->chip_cfg.fifo_enable;
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
int MPU_ConfigureFIFO(uint8_t sensors) {
    uint8_t prev;
    int result = 0;

    /* Compass data isn't going into the FIFO. Stop trying. */
    sensors &= ~INV_XYZ_COMPASS;

    if (st->chip_cfg.dmp_on)
        return 0;
    else {
        if (!(st->chip_cfg.sensors))
            return -1;
        prev = st->chip_cfg.fifo_enable;
        st->chip_cfg.fifo_enable = sensors & st->chip_cfg.sensors;
        if (st->chip_cfg.fifo_enable != sensors)
            /* You're not getting what you asked for. Some sensors are
             * asleep.
             */
            result = -1;
        else
            result = 0;
        if (sensors || st->chip_cfg.lp_accel_mode)
            MPU_SetIntEnable(1);
        else
            MPU_SetIntEnable(0);
        if (sensors) {
            if (MPU_ResetFIFO()) {
                st->chip_cfg.fifo_enable = prev;
                return -1;
            }
        }
    }
    return result;
}

/**
 *  @brief  Reset FIFO read/write pointers.
 *  @return 0 if successful.
 */
int MPU_ResetFIFO(void) {
    uint8_t data;

    if (!(st->chip_cfg.sensors))
        return -1;

    data = 0;
    if (MPU_WriteByteAndCheck(st->reg->int_enable, 0x00)) {
        return -3;
    }
    if (MPU_WriteByteAndCheck(st->reg->fifo_en, 0x00)) {
        return -3;
    }
    if (MPU_WriteByteAndCheck(st->reg->user_ctrl, 0x00)) {
        return -3;
    }

    if (st->chip_cfg.dmp_on) {
        MPU_WriteByte(st->reg->user_ctrl, BIT_FIFO_RST | BIT_DMP_RST);
        Delay_ms(50);
        data = BIT_DMP_EN | BIT_FIFO_EN;
        if (st->chip_cfg.sensors & INV_XYZ_COMPASS)
            data |= BIT_AUX_IF_EN;
        if (MPU_WriteByteAndCheck(st->reg->user_ctrl, data)) {
            return -3;
        }
        if (st->chip_cfg.int_enable)
            data = BIT_DMP_INT_EN;
        else
            data = 0;
        if (MPU_WriteByteAndCheck(st->reg->int_enable, data)) {
            return -3;
        }
        if (MPU_WriteByteAndCheck(st->reg->fifo_en, 0x00)) {
            return -3;
        }
    } else {
        MPU_WriteByte(st->reg->user_ctrl, BIT_FIFO_RST);
        if (st->chip_cfg.bypass_mode || !(st->chip_cfg.sensors & INV_XYZ_COMPASS))
            data = BIT_FIFO_EN;
        else
            data = BIT_FIFO_EN | BIT_AUX_IF_EN;
        if (MPU_WriteByteAndCheck(st->reg->user_ctrl, data)) {
            return -3;
        }
        Delay_ms(50);
        if (st->chip_cfg.int_enable)
            data = BIT_DATA_RDY_EN;
        else
            data = 0;
        if (MPU_WriteByteAndCheck(st->reg->int_enable,data)) {
            return -3;
        }
        if (MPU_WriteByteAndCheck(st->reg->fifo_en, st->chip_cfg.fifo_enable)) {
            return -3;
        }
    }
    return 0;
}

/**
 *  @brief      Enable/disable data ready interrupt.
 *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data ready
 *  interrupt is used.
 *  @param[in]  enable      1 to enable interrupt.
 *  @return     0 if successful.
 */
int MPU_SetIntEnable(uint8_t enable) {
    uint8_t tmp;

    if (st->chip_cfg.dmp_on) {
        if (enable)
            tmp = BIT_DMP_INT_EN;
        else
            tmp = 0x00;
        if (MPU_WriteByteAndCheck(st->reg->int_enable, tmp)) {
            return -3;
        }
        st->chip_cfg.int_enable = tmp;
    } else {
        if (!st->chip_cfg.sensors)
            return -1;
        if (enable && st->chip_cfg.int_enable)
            return 0;
        if (enable)
            tmp = BIT_DATA_RDY_EN;
        else
            tmp = 0x00;
        if (MPU_WriteByteAndCheck(st->reg->int_enable, tmp)) {
            return -3;
        }
        st->chip_cfg.int_enable = tmp;
    }
    return 0;
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
int MPU_SetCompassSampleRate(uint8_t rate) {
#ifdef AK89xx_SECONDARY
    switch (rate) {
        case 8:
            Compass_WriteByte(AKM_REG_CNTL, 0x12); // Continuous measurement mode 1 (8 Hz), 16 bit output
            break;
        case 100:
            Compass_WriteByte(AKM_REG_CNTL, 0x16); // Continuous measurement mode 2 (100 Hz), 16 bit output
            break;
        default:
            return -1;
    }
    st->chip_cfg.compass_sample_rate = rate;
    Delay_ms(10);
    MPU_WriteByte(st->reg->s0_ctrl, ~BIT_SLAVE_EN);
    return 0;
#else
    return -1;
#endif
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
int MPU_SetSensors(uint8_t sensors)  {
    uint8_t data;
#ifdef AK89xx_SECONDARY
    uint8_t user_ctrl;
#endif

    if (sensors & INV_XYZ_GYRO)
        data = INV_CLK_PLL;
    else if (sensors)
        data = 0;
    else
        data = BIT_SLEEP;
    if (MPU_WriteByteAndCheck(st->reg->pwr_mgmt_1, data)) {
        return -3;
    }
    
    st->chip_cfg.clk_src = data & ~BIT_SLEEP;

    data = 0;
    if (!(sensors & INV_X_GYRO))
        data |= BIT_STBY_XG;
    if (!(sensors & INV_Y_GYRO))
        data |= BIT_STBY_YG;
    if (!(sensors & INV_Z_GYRO))
        data |= BIT_STBY_ZG;
    if (!(sensors & INV_XYZ_ACCEL))
        data |= BIT_STBY_XYZA;
    if (MPU_WriteByteAndCheck(st->reg->pwr_mgmt_2, data)) {
        return -3;
    }

    if (sensors && (sensors != INV_XYZ_ACCEL))
        /* Latched interrupts only used in LP accel mode. */
        MPU_SetIntLatched(0);

#ifdef AK89xx_SECONDARY
//#ifdef AK89xx_BYPASS
//    if (sensors & INV_XYZ_COMPASS)
//        mpu_set_bypass(1);
//    else
//        mpu_set_bypass(0);
//#else
    user_ctrl = MPU_ReadByte(st->reg->user_ctrl);
    /* Handle AKM power management. */
    if (sensors & INV_XYZ_COMPASS) {
        MPU_SetCompassSampleRate(st->chip_cfg.compass_sample_rate);
        user_ctrl |= BIT_AUX_IF_EN;
    } else {
        Compass_WriteByte(AKM_REG_CNTL, AKM_POWER_DOWN);
        user_ctrl &= ~BIT_AUX_IF_EN;
    }
    if (st->chip_cfg.dmp_on)
        user_ctrl |= BIT_DMP_EN;
    else
        user_ctrl &= ~BIT_DMP_EN;

    /* Enable/disable I2C master mode. */
    if (MPU_WriteByteAndCheck(st->reg->user_ctrl, user_ctrl)) {
        return -3;
    }
//#endif
#endif
    st->chip_cfg.sensors = sensors;
    st->chip_cfg.lp_accel_mode = 0;
    Delay_ms(50);
    return 0;
}

uint8_t Compass_ReadByte(uint8_t reg) {
    uint8_t tmp;
    // I2C address for reading
    MPU_WriteByte(st->reg->s0_addr, READ_COMMAND | st->chip_cfg.compass_addr);
    // reading from ASAX register
    MPU_WriteByte(st->reg->s0_reg, reg);
    // Read 3 bytes
    MPU_WriteByte(st->reg->s0_ctrl, BIT_SLAVE_EN | 1);
    Delay_ms(10); 
    tmp = MPU_ReadByte(EXT_SENS_DATA_00);
    return tmp;
}

void Compass_WriteByte(uint8_t reg, uint8_t data) {
    // Mag I2C address
    MPU_WriteByte(st->reg->s0_addr, WRITE_COMMAND | st->chip_cfg.compass_addr);
    MPU_WriteByte(st->reg->s0_reg, reg);
    MPU_WriteByte(st->reg->s0_do, data);
    // I2C on, 1 byte
    MPU_WriteByte(st->reg->s0_ctrl, BIT_SLAVE_EN | 1);
}

uint8_t Aux_ReadByte(uint8_t i2c_addr, uint8_t reg) {
    uint8_t tmp;
    // Mag I2C address
    MPU_WriteByte(st->reg->s0_addr, READ_COMMAND | i2c_addr);
    MPU_WriteByte(st->reg->s0_reg, reg);
    // I2C on, 1 byte
    MPU_WriteByte(st->reg->s0_ctrl, BIT_SLAVE_EN | 1);
    Delay_ms(10); // experimental minimum with sample rate 200 Hz
    tmp = MPU_ReadByte(EXT_SENS_DATA_00);
    return tmp;
}

void Compass_Read(uint8_t reg, uint8_t *data, uint8_t size) {
    // I2C address for reading
    MPU_WriteByte(st->reg->s0_addr, READ_COMMAND | st->chip_cfg.compass_addr);
    // reading from ASAX register
    MPU_WriteByte(st->reg->s0_reg, reg);
    // Read 3 bytes
    MPU_WriteByte(st->reg->s0_ctrl, BIT_SLAVE_EN | size);
    Delay_ms(15);
    MPU_Read(EXT_SENS_DATA_00, data, 3);
}

/* This initialization is similar to the one in ak8975.c. */
static int setup_compass(void)
{
#ifdef AK89xx_SECONDARY
    uint8_t data[4], akm_addr, user_ctrl;

    //MPU_SetBypass(1);
    // Turn auxiliary I2C on
    user_ctrl = MPU_ReadByte(st->reg->user_ctrl);
    user_ctrl |= BIT_AUX_IF_EN; 
    if (MPU_WriteByteAndCheck(st->reg->user_ctrl, user_ctrl)) {
        return -1;
    }
    Delay_ms(10);

//    /* Find compass. Possible addresses range from 0x0C to 0x0F. */
//    for (akm_addr = 0x0C; akm_addr <= 0x0F; akm_addr++) {
//        data[0] = Aux_ReadByte(akm_addr, AKM_REG_WHOAMI);
//        if (data[0] == AKM_WHOAMI)
//            break;
//    }

//    if (akm_addr > 0x0F) {
//        /* TODO: Handle this case in all compass-related functions. */
//        return -1;
//    }

    akm_addr = 0x0c;
    st->chip_cfg.compass_addr = akm_addr;

    Compass_WriteByte(AKM_REG_CNTL, AKM_POWER_DOWN);
    Delay_ms(10);
    Compass_WriteByte(AKM_REG_CNTL, AKM_FUSE_ROM_ACCESS);
    Delay_ms(10);

    /* Get sensitivity adjustment data from fuse ROM. */
    Compass_Read(AKM_REG_ASAX, data, 3);
    st->chip_cfg.mag_sens_adj[0] = (data[0] - 128)*0.5f / 128.0f + 1.0f;
    st->chip_cfg.mag_sens_adj[1] = (data[1] - 128)*0.5f / 128.0f + 1.0f;
    st->chip_cfg.mag_sens_adj[2] = (data[2] - 128)*0.5f / 128.0f + 1.0f;

    Compass_WriteByte(AKM_REG_CNTL, AKM_POWER_DOWN);
    Delay_ms(10);
    MPU_WriteByte(st->reg->s0_ctrl, ~BIT_SLAVE_EN);

    //MPU_SetBypass(0);
    
    // I2C address for reading
    MPU_WriteByte(I2C_SLV1_ADDR, READ_COMMAND | AK8963_I2C_ADDRESS);
    // reading from HXL register
    MPU_WriteByte(I2C_SLV1_REG, AK8963_HXL);
    // Read 7 bytes
    MPU_WriteByte(I2C_SLV1_CTRL, BIT_SLAVE_EN | 7);

#ifdef MPU9150
    /* For the MPU9150, the auxiliary I2C bus needs to be set to VDD. */
    MPU_WriteByte(st->reg->yg_offs_tc, BIT_I2C_MST_VDDIO);
#endif

    return 0;
#else
    return -16;
#endif
}

/**
 *  @brief      Set device to bypass mode.
 *  @param[in]  bypass_on   1 to enable bypass mode.
 *  @return     0 if successful.
 */
int MPU_SetBypass(uint8_t bypass_on) {
    uint8_t tmp;

    if (st->chip_cfg.bypass_mode == bypass_on)
        return 0;

    if (bypass_on) {
        MPU_WriteByte(st->reg->user_ctrl, tmp);
        tmp &= ~BIT_AUX_IF_EN;
        MPU_WriteByte(st->reg->user_ctrl, tmp);
        Delay_ms(3);
        tmp = BIT_BYPASS_EN;
        if (st->chip_cfg.active_low_int)
            tmp |= BIT_ACTL;
        if (st->chip_cfg.latched_int)
            tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
        MPU_WriteByte(st->reg->int_pin_cfg, tmp);
    } else {
        /* Enable I2C master mode if compass is being used. */
        MPU_WriteByte(st->reg->user_ctrl, tmp);
 
        if (st->chip_cfg.sensors & INV_XYZ_COMPASS)
            tmp |= BIT_AUX_IF_EN;
        else
            tmp &= ~BIT_AUX_IF_EN;
        MPU_WriteByte(st->reg->user_ctrl, tmp);
        Delay_ms(3);
        if (st->chip_cfg.active_low_int)
            tmp = BIT_ACTL;
        else
            tmp = 0;
        if (st->chip_cfg.latched_int)
            tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
        MPU_WriteByte(st->reg->int_pin_cfg, tmp);
    }
    st->chip_cfg.bypass_mode = bypass_on;
    return 0;
}

void MPU_EXTI_Init() {
    GPIOA->OSPEEDR |= 3 << IMU_INT*2;
    
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;
    
    EXTI->RTSR  |= EXTI_RTSR_TR1;       // rising
    EXTI->IMR   |= EXTI_IMR_MR1;        // non-masking
    NVIC_SetPriority(EXTI1_IRQn, 0x02);
    NVIC_EnableIRQ(EXTI1_IRQn);
}

extern DMP *dmp;

void EXTI1_IRQHandler() {
    uint16_t fifo_count = 0;
    uint8_t tmp[2];
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR = EXTI_PR_PR1;
        
        MPU_Read(FIFO_COUNTH, tmp, 2);
        fifo_count = (tmp[0] << 8) | tmp[1];
        if (fifo_count) {
            MPU_DMA_tx[0] = READ_COMMAND | FIFO_R_W;
            MPU_DMA_Run(MPU_DMA_tx, MPU_DMA_rx, dmp->packet_length + 1);
        }
    }
}

void swap(float *a, float *b) {
    float tmp = *a;
    *a = *b;
    *b = tmp;
}

#include "main.h"
#include "telemetry.h"
#include "extra_math.h"
#include "data_builder.h"
uint8_t compass_data[8];

int get_tick_count(unsigned long *count);
unsigned long timestamp;

short raw_accel[3];
short raw_gyro[3];
short raw_compass[3];

long raw_accel_long[3];
long raw_compass_long[3];

static void parse_quat_accel_gyro(uint8_t *raw_data, long *quat, short *accel, short *gyro) {
    uint8_t i;
    int32_t index = 0;
    if (dmp->feature_mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT)) {
        for (i = 0; i < 4; i++) {
            quat[i] = (long)(((long)raw_data[4*i] << 24) | ((long)raw_data[4*i+1] << 16) |
                    ((long)raw_data[4*i+2] << 8) | raw_data[4*i+3]);
        }
        index += 16;
    }
    if (dmp->feature_mask & DMP_FEATURE_SEND_RAW_ACCEL) {
        for (i = 0; i < 3; i++) {
            accel[i] = (int16_t)(((int16_t)raw_data[index+2*i] << 8) | raw_data[index+2*i+1]);
        }
        index += 6;
    }
    if (dmp->feature_mask & DMP_FEATURE_SEND_ANY_GYRO) {
        for (i = 0; i < 3; i++) {
            gyro[i] = (int16_t)(((int16_t)raw_data[index+2*i] << 8) | raw_data[index+2*i]);
        }
    }
}

static void parse_compass(uint8_t *raw_compass, short *compass) {
    uint8_t i = 0;
    int16_t tmp = 0;
    for (i = 0; i < 3; i++) {
        compass[i] = (raw_compass[2*i] | ((short)raw_compass[2*i+1] << 8));
    }
}

static void short2long(short *s, long *l, uint16_t size) {
    uint16_t i;
    for (i = 0; i < size; i++) {
        l[i] = (long)s[i];
    }
}

void DMA1_Stream3_IRQHandler() {
    uint8_t i = 0;
    if (DMA1->LISR & DMA_LISR_TCIF3) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3;
        DMA1_Stream3->CR &= ~DMA_SxCR_EN;
        MPU_NSS_High();
        
        get_tick_count(&timestamp);
        
        if (DMA1_Stream3->M0AR == (uint32_t)MPU_DMA_rx) {
            parse_quat_accel_gyro(MPU_DMA_rx + 1, dmp_quat_data, raw_accel, raw_gyro);
            
            inv_build_quat(dmp_quat_data, 1, timestamp);
            short2long(raw_accel, raw_accel_long, 3);
            for (i = 0; i < 3; i++) {
                raw_accel_long[i] = (long)((float)(raw_accel_long[i] << 16) / st->chip_cfg.accel_sens);
            }
            inv_build_accel(raw_accel_long, INV_CALIBRATED, timestamp);
            inv_build_gyro(raw_gyro, timestamp);
            
            new_data = BIT_MPL | BIT_DMP;
            
            if (st->chip_cfg.sensors & INV_XYZ_COMPASS) {
                MPU_DMA_tx[0] = READ_COMMAND | EXT_SENS_DATA_00;
                MPU_DMA_Run(MPU_DMA_tx, compass_data, 8);
            }
        } else if (DMA1_Stream3->M0AR == (uint32_t)compass_data) {
            parse_compass(compass_data + 1, raw_compass);
            short2long(raw_compass, raw_compass_long, 3);
            
            
            for (i = 0; i < 3; i++) {
                mine_accel[i]   = raw_accel[i]      / st->chip_cfg.accel_sens;
                mine_gyro[i]    = raw_gyro[i]       / st->chip_cfg.gyro_sens;
                mine_compass[i] = raw_compass[i]    * MAG_SENS * st->chip_cfg.mag_sens_adj[i];
                raw_compass_long[i] = (long)(mine_compass[i] * (1 << 16));
            }
            inv_build_compass(raw_compass_long, INV_CALIBRATED, timestamp);
            mine_compass[2] = -mine_compass[2];
            swap(&mine_compass[0], &mine_compass[1]);
            mine_gyro[0] = -mine_gyro[0];
            mine_gyro[1] = -mine_gyro[1];
            mine_gyro[2] = -mine_gyro[2];
            
            new_data = BIT_MPL | BIT_MINE;
        }
    }
}


/**
 *  @brief      Load and verify DMP image.
 *  @param[in]  length      Length of DMP image.
 *  @param[in]  firmware    DMP code.
 *  @param[in]  start_addr  Starting address of DMP code memory.
 *  @param[in]  sample_rate Fixed sampling rate used when DMP is enabled.
 *  @return     0 if successful.
 */
int MPU_LoadFirmware(uint16_t length, const uint8_t *firmware,
    uint16_t start_addr, uint16_t sample_rate) 
{        
    uint16_t ii;
    uint16_t this_write;

    /* Must divide evenly into st->hw->bank_size to avoid bank crossings. */
#define LOAD_CHUNK  (16)
    uint8_t cur[LOAD_CHUNK], tmp[2];

    if (st->chip_cfg.dmp_loaded)
        /* DMP should only be loaded once. */
        return -1;

    if (!firmware)
        return -2;
        
    for (ii = 0; ii < length; ii += this_write) {
        this_write = min(LOAD_CHUNK, length - ii);
        if (MPU_MemWrite(ii, (uint8_t *)&firmware[ii], this_write)) {
            return -3;
        }
        if (MPU_MemRead(ii, cur, this_write)) {
            return -3;
        }
        if (memcmp(&firmware[ii], cur, this_write)) {
            return -5;
        }
    }

    /* Set program start address. */
    tmp[0] = start_addr >> 8;
    tmp[1] = start_addr & 0xFF;
    if (MPU_WriteAndCheck(st->reg->prgm_start_h, tmp, 2))
        return -6;

    st->chip_cfg.dmp_loaded = 1;
    st->chip_cfg.dmp_sample_rate = sample_rate;
    return 0;
}

/**
 *  @brief      Enable/disable DMP support.
 *  @param[in]  enable  1 to turn on the DMP.
 *  @return     0 if successful.
 */
int MPU_SetDMPState(uint8_t enable) {
    uint8_t tmp;
    if (st->chip_cfg.dmp_on == enable)
        return 0;

    if (enable) {
        if (!st->chip_cfg.dmp_loaded)
            return -1;
        /* Disable data ready interrupt. */
        MPU_SetIntEnable(0);
        /* Disable bypass mode. */
        MPU_SetBypass(0);
        /* Keep constant sample rate, FIFO rate controlled by DMP. */
        MPU_SetSampleRate(st->chip_cfg.dmp_sample_rate);
        /* Remove FIFO elements. */
        MPU_WriteByte(st->reg->fifo_en, 0x00);
        st->chip_cfg.dmp_on = 1;
        /* Enable DMP interrupt. */
        MPU_SetIntEnable(1);
        MPU_ResetFIFO();
    } else {
        /* Disable DMP interrupt. */
        MPU_SetIntEnable(0);
        /* Restore FIFO settings. */
        tmp = st->chip_cfg.fifo_enable;
        MPU_WriteByte(st->reg->fifo_en, st->chip_cfg.fifo_enable);
        st->chip_cfg.dmp_on = 0;
        MPU_ResetFIFO();
    }
    return 0;
}

#define MAX_PACKET_LENGTH (12)

static const uint16_t mpu_st_tb[256] = {
	2620,2646,2672,2699,2726,2753,2781,2808, //7
	2837,2865,2894,2923,2952,2981,3011,3041, //15
	3072,3102,3133,3165,3196,3228,3261,3293, //23
	3326,3359,3393,3427,3461,3496,3531,3566, //31
	3602,3638,3674,3711,3748,3786,3823,3862, //39
	3900,3939,3979,4019,4059,4099,4140,4182, //47
	4224,4266,4308,4352,4395,4439,4483,4528, //55
	4574,4619,4665,4712,4759,4807,4855,4903, //63
	4953,5002,5052,5103,5154,5205,5257,5310, //71
	5363,5417,5471,5525,5581,5636,5693,5750, //79
	5807,5865,5924,5983,6043,6104,6165,6226, //87
	6289,6351,6415,6479,6544,6609,6675,6742, //95
	6810,6878,6946,7016,7086,7157,7229,7301, //103
	7374,7448,7522,7597,7673,7750,7828,7906, //111
	7985,8065,8145,8227,8309,8392,8476,8561, //119
	8647,8733,8820,8909,8998,9088,9178,9270,
	9363,9457,9551,9647,9743,9841,9939,10038,
	10139,10240,10343,10446,10550,10656,10763,10870,
	10979,11089,11200,11312,11425,11539,11654,11771,
	11889,12008,12128,12249,12371,12495,12620,12746,
	12874,13002,13132,13264,13396,13530,13666,13802,
	13940,14080,14221,14363,14506,14652,14798,14946,
	15096,15247,15399,15553,15709,15866,16024,16184,
	16346,16510,16675,16842,17010,17180,17352,17526,
	17701,17878,18057,18237,18420,18604,18790,18978,
	19167,19359,19553,19748,19946,20145,20347,20550,
	20756,20963,21173,21385,21598,21814,22033,22253,
	22475,22700,22927,23156,23388,23622,23858,24097,
	24338,24581,24827,25075,25326,25579,25835,26093,
	26354,26618,26884,27153,27424,27699,27976,28255,
	28538,28823,29112,29403,29697,29994,30294,30597,
	30903,31212,31524,31839,32157,32479,32804,33132
};

#ifdef AK89xx_SECONDARY
static int compass_self_test(void) {
    uint8_t tmp[6];
    uint8_t tries = 10;
    int result = 0x07;
    uint16_t data;

    Compass_WriteByte(AKM_REG_CNTL, AKM_POWER_DOWN);
    Compass_WriteByte(AKM_REG_ASTC, AKM_BIT_SELF_TEST);
    Compass_WriteByte(AKM_REG_CNTL, AKM_MODE_SELF_TEST);

    do {
        Delay_ms(10);
        tmp[0] = Compass_ReadByte(AKM_REG_ST1);
        if (tmp[0] & AKM_DATA_READY)
            break;
    } while (tries--);
    if (!(tmp[0] & AKM_DATA_READY))
        goto AKM_restore;

    Compass_Read(AKM_REG_HXL, tmp, 6);

    result = 0;
#if defined MPU9150
    data = (short)(tmp[1] << 8) | tmp[0];
    if ((data > 100) || (data < -100))
        result |= 0x01;
    data = (short)(tmp[3] << 8) | tmp[2];
    if ((data > 100) || (data < -100))
        result |= 0x02;
    data = (short)(tmp[5] << 8) | tmp[4];
    if ((data > -300) || (data < -1000))
        result |= 0x04;
#elif defined MPU9250
    data = (short)(tmp[1] << 8) | tmp[0];
    if ((data > 200) || (data < -200))  
        result |= 0x01;
    data = (short)(tmp[3] << 8) | tmp[2];
    if ((data > 200) || (data < -200))  
        result |= 0x02;
    data = (short)(tmp[5] << 8) | tmp[4];
    if ((data > -800) || (data < -3200))  
        result |= 0x04;
#endif
AKM_restore:
    Compass_WriteByte(AKM_REG_ASTC, SUPPORTS_AK89xx_HIGH_SENS);
    Compass_WriteByte(AKM_REG_CNTL, SUPPORTS_AK89xx_HIGH_SENS);

    return result;
}
#endif

static int accel_self_test(long *bias_regular, long *bias_st) {
    int i, result = 0, otp_value_zero = 0;
    float accel_st_al_min, accel_st_al_max;
    float st_shift_cust[3], st_shift_ratio[3], ct_shift_prod[3], accel_offset_max;
    uint8_t regs[3];
    
    MPU_Read(SELF_TEST_X_ACCEL, regs, 3);

	for (i = 0; i < 3; i++) {
		if (regs[i] != 0) {
			ct_shift_prod[i] = mpu_st_tb[regs[i] - 1];
			ct_shift_prod[i] *= 65536.f;
			ct_shift_prod[i] /= test->accel_sens;
		}
		else {
			ct_shift_prod[i] = 0;
			otp_value_zero = 1;
		}
	}
	if (otp_value_zero == 0) {
		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = bias_st[i] - bias_regular[i];
			st_shift_ratio[i] = st_shift_cust[i] / ct_shift_prod[i];
			if (fabs(st_shift_ratio[i]) > test->max_accel_var) {
				result |= 1 << i;	//Error condition
			}
		}
	}
	else {
		/* Self Test Pass/Fail Criteria B */
		accel_st_al_min = test->min_g * 65536.f;
		accel_st_al_max = test->max_g * 65536.f;

		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = bias_st[i] - bias_regular[i];
			if(st_shift_cust[i] < accel_st_al_min || st_shift_cust[i] > accel_st_al_max) {
				result |= 1 << i;	//Error condition
			}
		}
	}

	if (result == 0) {
	/* Self Test Pass/Fail Criteria C */
		accel_offset_max = .5f * 65536.f;
		for (i = 0; i < 3; i++) {
			if(fabs(bias_regular[i]) > accel_offset_max) {
				result |= 1 << i;	//Error condition
			}
		}
	}

    return result;
}

static int gyro_self_test(long *bias_regular, long *bias_st) {
    int i, result = 0, otp_value_zero = 0;
    float gyro_st_al_max;
    float st_shift_cust[3], st_shift_ratio[3], ct_shift_prod[3], gyro_offset_max;
    unsigned char regs[3];

    MPU_Read(SELF_TEST_X_GYRO, regs, 3);

	for (i = 0; i < 3; i++) {
		if (regs[i] != 0) {
			ct_shift_prod[i] = mpu_st_tb[regs[i] - 1];
			ct_shift_prod[i] *= 65536.f;
			ct_shift_prod[i] /= test->gyro_sens;
		}
		else {
			ct_shift_prod[i] = 0;
			otp_value_zero = 1;
		}
	}

	if(otp_value_zero == 0) {
		/* Self Test Pass/Fail Criteria A */
		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = bias_st[i] - bias_regular[i];

			st_shift_ratio[i] = st_shift_cust[i] / ct_shift_prod[i];

			if (fabsf(st_shift_ratio[i]) < test->max_gyro_var) {
				result |= 1 << i;	//Error condition
			}
		}
	}
	else {
		/* Self Test Pass/Fail Criteria B */
		gyro_st_al_max = test->max_dps * 65536.f;

		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = bias_st[i] - bias_regular[i];

			if(st_shift_cust[i] < gyro_st_al_max) {
				result |= 1 << i;	//Error condition
			}
		}
	}

	if(result == 0) {
	/* Self Test Pass/Fail Criteria C */
		gyro_offset_max = test->min_dps * 65536.f;
		for (i = 0; i < 3; i++) {
			if(fabsf(bias_regular[i]) > gyro_offset_max) {
				result |= 1 << i;	//Error condition
			}
		}
	}
    return result;
}
#define HWST_MAX_PACKET_LENGTH (512)

static void get_st_biases(long *gyro_bias, long *accel_bias, uint8_t hw_test) {
    uint8_t data[HWST_MAX_PACKET_LENGTH];
    uint8_t packet_count, ii;
    uint16_t fifo_count;
    int s = 0, read_size = 0, ind;

    data[0] = 0x01;
    data[1] = 0;
    MPU_Write(st->reg->pwr_mgmt_1, data, 2);
    Delay_ms(200);
    
    MPU_WriteByte(st->reg->int_enable,  0x00);
    MPU_WriteByte(st->reg->fifo_en,     0x00);
    MPU_WriteByte(st->reg->pwr_mgmt_1,  0x00);
    MPU_WriteByte(st->reg->i2c_mst,     0x00);
    MPU_WriteByte(st->reg->user_ctrl,   0x00);
    
    MPU_WriteByte(st->reg->user_ctrl, BIT_FIFO_RST | BIT_DMP_RST);
    Delay_ms(15);
    
    MPU_WriteByte(st->reg->lpf, st->test->reg_lpf);

    MPU_WriteByte(st->reg->rate_div, st->test->reg_rate_div);

    if (hw_test) {
        MPU_WriteByte(st->reg->gyro_cfg, st->test->reg_gyro_fsr | 0xE0);
        MPU_WriteByte(st->reg->accel_cfg, st->test->reg_accel_fsr | 0xE0);
    } else {
        MPU_WriteByte(st->reg->gyro_cfg, st->test->reg_gyro_fsr);
        MPU_WriteByte(st->reg->accel_cfg, st->test->reg_accel_fsr);
    }
    Delay_ms(test->wait_ms);  //wait 200ms for sensors to stabilize

    /* Enable FIFO */
    MPU_WriteByte(st->reg->user_ctrl, BIT_FIFO_EN);
    MPU_WriteByte(st->reg->fifo_en, INV_XYZ_GYRO | INV_XYZ_ACCEL);

    //initialize the bias return values
    gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0;
    accel_bias[0] = accel_bias[1] = accel_bias[2] = 0;

    //start reading samples
    while (s < test->packet_thresh) {
    	Delay_ms(10); //wait 10ms to fill FIFO
		MPU_Read(st->reg->fifo_count_h, data, 2);
		fifo_count = (data[0] << 8) | data[1];
		packet_count = fifo_count / MAX_PACKET_LENGTH;
		if ((test->packet_thresh - s) < packet_count) {
            packet_count = test->packet_thresh - s;
        }
		read_size = packet_count * MAX_PACKET_LENGTH;

		//burst read from FIFO
		MPU_Read(st->reg->fifo_r_w, data, read_size);

		ind = 0; 
		for (ii = 0; ii < packet_count; ii++) {
			short accel_cur[3], gyro_cur[3];
			accel_cur[0] = ((short)data[ind + 0] << 8) | data[ind + 1];
			accel_cur[1] = ((short)data[ind + 2] << 8) | data[ind + 3];
			accel_cur[2] = ((short)data[ind + 4] << 8) | data[ind + 5];
			accel_bias[0] += (long)accel_cur[0];
			accel_bias[1] += (long)accel_cur[1];
			accel_bias[2] += (long)accel_cur[2];
			gyro_cur[0] = (((short)data[ind + 6] << 8) | data[ind + 7]);
			gyro_cur[1] = (((short)data[ind + 8] << 8) | data[ind + 9]);
			gyro_cur[2] = (((short)data[ind + 10] << 8) | data[ind + 11]);
			gyro_bias[0] += (long)gyro_cur[0];
			gyro_bias[1] += (long)gyro_cur[1];
			gyro_bias[2] += (long)gyro_cur[2];
			ind += MAX_PACKET_LENGTH;
		}
		s += packet_count;
    }

    //stop FIFO
    MPU_WriteByte(st->reg->fifo_en, 0x00);

    gyro_bias[0] = (long)(((long long)gyro_bias[0]<<16) / test->gyro_sens / s);
    gyro_bias[1] = (long)(((long long)gyro_bias[1]<<16) / test->gyro_sens / s);
    gyro_bias[2] = (long)(((long long)gyro_bias[2]<<16) / test->gyro_sens / s);
    accel_bias[0] = (long)(((long long)accel_bias[0]<<16) / test->accel_sens / s);
    accel_bias[1] = (long)(((long long)accel_bias[1]<<16) / test->accel_sens / s);
    accel_bias[2] = (long)(((long long)accel_bias[2]<<16) / test->accel_sens / s);
    /* remove gravity from bias calculation */
    if (accel_bias[2] > 0L)
        accel_bias[2] -= 65536L;
    else
        accel_bias[2] += 65536L;
}

/**
 *  @brief      Trigger gyro/accel/compass self-test for MPU6500/MPU9250
 *  On success/error, the self-test returns a mask representing the sensor(s)
 *  that failed. For each bit, a one (1) represents a "pass" case; conversely,
 *  a zero (0) indicates a failure.
 *
 *  \n The mask is defined as follows:
 *  \n Bit 0:   Gyro.
 *  \n Bit 1:   Accel.
 *  \n Bit 2:   Compass.
 *
 *  @param[out] gyro        Gyro biases in q16 format.
 *  @param[out] accel       Accel biases (if applicable) in q16 format.
 *  @param[in]  debug       Debug flag used to print out more detailed logs. Must first set up logging in Motion Driver.
 *  @return     Result mask (see above).
 */
int MPU_RunSelfTest(long *gyro_bias, long *accel_bias) {
    long gyro_st[3], accel_st[3];
    uint8_t accel_result, gyro_result;
#ifdef AK89xx_SECONDARY
    uint8_t compass_result;
#endif
    int ii;

    int result;
    uint8_t accel_fsr, fifo_sensors, sensors_on;
    uint16_t gyro_fsr, sample_rate, lpf;
    uint8_t dmp_was_on;

    if (st->chip_cfg.dmp_on) {
        MPU_SetDMPState(0);
        dmp_was_on = 1;
    } else
        dmp_was_on = 0;

    /* Get initial settings. */
    MPU_GetGyroFsr(&gyro_fsr);
    MPU_GetAccelFsr(&accel_fsr);
    MPU_GetLPF(&lpf);
    MPU_GetSampleRate(&sample_rate);
    sensors_on = st->chip_cfg.sensors;
    MPU_GetFIFOConfig(&fifo_sensors);

    get_st_biases(gyro_bias, accel_bias, 0);
    get_st_biases(gyro_st, accel_st, 1);

    accel_result = accel_self_test(accel_bias, accel_st);

    gyro_result = gyro_self_test(gyro_bias, gyro_st);

    result = 0;
    if (!gyro_result)
        result |= 0x01;
    if (!accel_result)
        result |= 0x02;

#ifdef AK89xx_SECONDARY
    compass_result = compass_self_test();
    if (!compass_result)
        result |= 0x04;
#else
    result |= 0x04;
#endif
	/* Set to invalid values to ensure no I2C writes are skipped. */
	st->chip_cfg.gyro_fsr = 0xFF;
	st->chip_cfg.accel_fsr = 0xFF;
	st->chip_cfg.lpf = 0xFF;
	st->chip_cfg.sample_rate = 0xFFFF;
	st->chip_cfg.sensors = 0xFF;
	st->chip_cfg.fifo_enable = 0xFF;
	st->chip_cfg.clk_src = INV_CLK_PLL;
	MPU_SetGyroFsr(gyro_fsr);
	MPU_SetAccelFsr(accel_fsr);
	MPU_SetAccelFsr(lpf);
	MPU_SetSampleRate(sample_rate);
	MPU_SetSensors(sensors_on);
	MPU_ConfigureFIFO(fifo_sensors);

	if (dmp_was_on)
		MPU_SetDMPState(1);

	return result;
}

/**
 *  @brief      Read biases to the accel bias 6500 registers.
 *  This function reads from the MPU6500 accel offset cancellations registers.
 *  The format are G in +-8G format. The register is initialized with OTP 
 *  factory trim values.
 *  @param[in]  accel_bias  returned structure with the accel bias
 *  @return     0 if successful.
 */
int MPU_ReadAccelBias(long *accel_bias) {
	uint8_t data[6];
	MPU_Read(XA_OFFSET_H, &data[0], 2);
    MPU_Read(YA_OFFSET_H, &data[2], 2);
    MPU_Read(ZA_OFFSET_H, &data[4], 2);
	accel_bias[0] = ((long)data[0]<<8) | data[1];
	accel_bias[1] = ((long)data[2]<<8) | data[3];
	accel_bias[2] = ((long)data[4]<<8) | data[5];
	return 0;
}

/**
 *  @brief      Push biases to the accel bias 6500 registers.
 *  This function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values. Bias inputs are LSB
 *  in +-16G format.
 *  @param[in]  accel_bias  New biases.
 *  @return     0 if successful.
 */
int MPU_SetAccelBias(const long *accel_bias) {
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    long accel_reg_bias[3] = {0, 0, 0};

    MPU_ReadAccelBias(accel_reg_bias);

    // Preserve bit 0 of factory value (for temperature compensation)
    accel_reg_bias[0] -= (accel_bias[0] & ~1);
    accel_reg_bias[1] -= (accel_bias[1] & ~1);
    accel_reg_bias[2] -= (accel_bias[2] & ~1);

    data[0] = (accel_reg_bias[0] >> 8) & 0xff;
    data[1] = (accel_reg_bias[0]) & 0xff;
    data[2] = (accel_reg_bias[1] >> 8) & 0xff;
    data[3] = (accel_reg_bias[1]) & 0xff;
    data[4] = (accel_reg_bias[2] >> 8) & 0xff;
    data[5] = (accel_reg_bias[2]) & 0xff;

    if (MPU_WriteAndCheck(XA_OFFSET_H, &data[0], 2)) {
        return -1;
    }
    if (MPU_WriteAndCheck(YA_OFFSET_H, &data[2], 2)) {
        return -1;
    }
    if (MPU_WriteAndCheck(ZA_OFFSET_H, &data[4], 2)) {
        return -1;
    }
    return 0;
}

int MPU_ReadGyroBias(long *gyro_bias) {
	uint8_t data[6];
	MPU_Read(XG_OFFSET_H, &data[0], 2);
    MPU_Read(YG_OFFSET_H, &data[2], 2);
    MPU_Read(ZG_OFFSET_H, &data[4], 2);
	gyro_bias[0] = ((long)data[0]<<8) | data[1];
	gyro_bias[1] = ((long)data[2]<<8) | data[3];
	gyro_bias[2] = ((long)data[4]<<8) | data[5];
	return 0;
}

/**
 *  @brief      Push biases to the gyro bias 6500/6050 registers.
 *  This function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values. Bias inputs are LSB
 *  in +-1000dps format.
 *  @param[in]  gyro_bias  New biases.
 *  @return     0 if successful.
 */
int MPU_SetGyroBias(long *gyro_bias)
{
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    uint8_t i=0;
    for(i = 0; i < 3; i++) {
    	gyro_bias[i]= (-gyro_bias[i]);
    }
    data[0] = (gyro_bias[0] >> 8) & 0xff;
    data[1] = (gyro_bias[0]) & 0xff;
    data[2] = (gyro_bias[1] >> 8) & 0xff;
    data[3] = (gyro_bias[1]) & 0xff;
    data[4] = (gyro_bias[2] >> 8) & 0xff;
    data[5] = (gyro_bias[2]) & 0xff;
    if (MPU_WriteAndCheck(XG_OFFSET_H, &data[0], 2)) {
        return -1;
    }
    if (MPU_WriteAndCheck(YG_OFFSET_H, &data[2], 2)) {
        return -1;
    }
    if (MPU_WriteAndCheck(ZG_OFFSET_H, &data[4], 2)) {
        return -1;
    }
    return 0;
}

