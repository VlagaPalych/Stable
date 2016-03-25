#include "stm32f4xx.h" 
#include "mpu9250.h"
#include "dmp.h"
#include "string.h"
#include "extra_math.h"
#include "spi.h"
#include "dma.h"
#include "stdlib.h"


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

float mag_sens_adj[3];

uint8_t dmp_on = 0;

uint8_t meas1 = 1;
uint8_t process = 0;
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

//    if (!data)
//        return -1;
//    if (!st->chip_cfg.sensors)
//        return -2;

    tmp[0] = (uint8_t)(mem_addr >> 8);
    tmp[1] = (uint8_t)(mem_addr & 0xFF);

    /* Check bank boundaries. */
//    if (tmp[1] + length > st->hw->bank_size)
//        return -3;

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

//    if (!data)
//        return -1;
//    if (!st->chip_cfg.sensors)
//        return -2;

    tmp[0] = (uint8_t)(mem_addr >> 8);
    tmp[1] = (uint8_t)(mem_addr & 0xFF);

    /* Check bank boundaries. */
//    if (tmp[1] + length > st->hw->bank_size)
//        return -3;

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
    if (MPU_WriteByteAndCheck(st->reg->gyro_cfg, data)) {
        return -3;
    }
    
    st->chip_cfg.gyro_fsr = data >> 3;
    return 0;
}

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
    if (MPU_WriteByteAndCheck(st->reg->accel_cfg, data)) {
        return -3;
    }
    
    st->chip_cfg.accel_fsr = data >> 3;
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

uint8_t compass_data[8];

void DMA1_Stream3_IRQHandler() {
    uint8_t i = 0;
    int16_t tmp = 0;
    if (DMA1->LISR & DMA_LISR_TCIF3) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3;
        DMA1_Stream3->CR &= ~DMA_SxCR_EN;
        MPU_NSS_High();
        
        if (DMA1_Stream3->M0AR == (uint32_t)MPU_DMA_rx) {
            DMP_ParseFIFOData(MPU_DMA_rx + 1); 
            
            if (st->chip_cfg.sensors & INV_XYZ_COMPASS) {
                MPU_DMA_tx[0] = READ_COMMAND | EXT_SENS_DATA_00;
                MPU_DMA_Run(MPU_DMA_tx, compass_data, 8);
            }
        } else if (DMA1_Stream3->M0AR == (uint32_t)compass_data) {
            magField[0] = (compass_data[1] | (compass_data[2] >> 8)) * MAG_SENS * st->chip_cfg.mag_sens_adj[0];
            magField[1] = (compass_data[3] | (compass_data[6] >> 8)) * MAG_SENS * st->chip_cfg.mag_sens_adj[1];
            magField[2] = (compass_data[5] | (compass_data[7] >> 8)) * MAG_SENS * st->chip_cfg.mag_sens_adj[2];
        }
        
        
        
        
//        for (i = 0; i < 3; i++) {
//            tmp = (MPU_DMA_rx[2*i+1] << 8) | MPU_DMA_rx[2*i+2];
//            accel[i] = tmp / ACCEL_SENSITIVITY;
//        }
        
//        tmp = (MPU_DMA_rx[7] << 8) | MPU_DMA_rx[8];
//        temp = tmp / TEMP_SENSITIBITY + TEMP_OFFSET;
        
//        for (i = 0; i < 3; i++) {
//            tmp = (MPU_DMA_rx[2*i+7] << 8) | MPU_DMA_rx[2*i+8];
//            angleRate[i] = tmp / GYRO_SENSITIVITY; // for QUEST algorithm
//        }
        
        
        
//        for (i = 0; i < 3; i++) {
//            tmp = MPU_DMA_rx[2*i+15] | (MPU_DMA_rx[2*i+16] << 8);
//            magField[i] = tmp * MAG_SENSITIVITY * mag_sens_adj[i];
//        }
//        magField[2] = -magField[2];
//        swap(&magField[0], &magField[1]);  
        
//        if (meas1) {
//            meas1 = 0;
//            memcpy(w1, accel, VECT_SIZE*sizeof(float));
//            memcpy(w2, magField, VECT_SIZE*sizeof(float));
//            
//            Vect_Norm(w1);
//            Vect_Norm(w2);
//        } else {
//            process = 1;
//            memcpy(v1, accel, VECT_SIZE*sizeof(float));
//            memcpy(v2, magField, VECT_SIZE*sizeof(float));
//            
//            Vect_Norm(v1);
//            Vect_Norm(v2);
//        }
    }
}

void Mag_Init() {
    uint8_t tmp[3], i = 0;
    
    // Enable I2C master
    MPU_WriteByte(USER_CTRL, 0x20);
    
    Compass_WriteByte(AK8963_CNTL, 0x00); // Power down magnetometer  
    Delay_ms(10);   
    Compass_WriteByte(AK8963_CNTL, 0x0f); // Enter Fuse ROM access mode
    Delay_ms(10);
    
    // I2C address for reading
    MPU_WriteByte(I2C_SLV0_ADDR, READ_COMMAND | AK8963_I2C_ADDRESS);
    // reading from ASAX register
    MPU_WriteByte(I2C_SLV0_REG, AK8963_ASAX);
    // Read 3 bytes
    MPU_WriteByte(I2C_SLV0_CTRL, 0x83);
    Delay_ms(15);
    MPU_Read(EXT_SENS_DATA_00, tmp, 3);
    
    for (i = 0; i < 3; i++) {
        mag_sens_adj[i] = (tmp[i] - 128)*0.5 / 128.0f + 1.0f;
    }
    
    Compass_WriteByte(AK8963_CNTL, 0x00); // Power down magnetometer  
    Delay_ms(10);
//    

////    Mag_WriteByte(AK8963_CNTL, 0x16); // Continuous measurement mode 2 (100 Hz), 16 bit output
////    Delay_ms(10);
////    
////    // Data ready interrupt waits for external sensor data
////    //MPU_WriteByte(I2C_MST_CTRL, 0x40);
////    // I2C address for reading
////    MPU_WriteByte(I2C_SLV0_ADDR, READ_COMMAND | AK8963_I2C_ADDRESS);
////    // reading from HXL register
////    MPU_WriteByte(I2C_SLV0_REG, AK8963_HXL);
////    // Read 7 bytes
////    MPU_WriteByte(I2C_SLV0_CTRL, 0x87);
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

