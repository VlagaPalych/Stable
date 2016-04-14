#include "dma.h"
#include "spi.h"
#include "stm32f4xx.h"
#include "main.h"
#include "leds.h"

uint8_t dma_buf_tx[100];
uint8_t dma_buf_rx[100];

extern "C" void mpu_dma_init() {
    SPI2->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
    
    NVIC_SetPriority(DMA1_Stream3_IRQn, 0x02);    
    NVIC_SetPriority(DMA1_Stream4_IRQn, 0x02);    
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    
    DMA1_Stream3->PAR   = (uint32_t)&(SPI2->DR);
    DMA1_Stream3->CR    = DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_PL; 
    
    DMA1_Stream4->PAR   = (uint32_t)&(SPI2->DR);
    DMA1_Stream4->CR    |= DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_DIR_0 | DMA_SxCR_PL; 
}

extern "C" void mpu_dma_run(uint8_t *tx, uint8_t *rx, uint16_t size) {
    mpu_nss_low();
    
    DMA1_Stream3->M0AR  = (uint32_t)rx;
    DMA1_Stream3->NDTR  = size;
    DMA1_Stream3->CR    |= DMA_SxCR_EN; 

    DMA1_Stream4->M0AR  = (uint32_t)tx;     
    DMA1_Stream4->NDTR  = size;
    DMA1_Stream4->CR    |= DMA_SxCR_EN;     
}

extern "C" void DMA1_Stream4_IRQHandler() {
    if (DMA1->HISR & DMA_HISR_TCIF4) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4;
        DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    } 
}

extern "C" void DMA1_Stream3_IRQHandler() {
    if (DMA1->LISR & DMA_LISR_TCIF3) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3;
        DMA1_Stream3->CR &= ~DMA_SxCR_EN;
        mpu_nss_high();

        led_toggle(0);
        switch (spi2_busy) {
            case SPI2_IMU:
                spi2_busy = SPI2_FREE;
                inertial_sensor.parse_raw_data(dma_buf_rx + 1);
                break;
            case SPI2_MAG:
                spi2_busy = SPI2_FREE;
                compass.parse_raw_data(dma_buf_rx + 1);
                break;
            default:
                spi2_busy = SPI2_FREE;
                break;
        }
        
        
//        if (DMA1_Stream3->M0AR == (uint32_t)MPU_DMA_rx) {
//            parse_quat_accel_gyro(MPU_DMA_rx + 1, dmp_quat_data, raw_accel, raw_gyro);
//            
//            inv_build_quat(dmp_quat_data, 1, timestamp);
//            short2long(raw_accel, raw_accel_long, 3);
//            for (i = 0; i < 3; i++) {
//                raw_accel_long[i] = (long)((float)(raw_accel_long[i] << 16) / st->chip_cfg.accel_sens);
//            }
//            inv_build_accel(raw_accel_long, INV_CALIBRATED, timestamp);
//            inv_build_gyro(raw_gyro, timestamp);
//            
//            new_data = BIT_MPL | BIT_DMP;
//            
//            if (st->chip_cfg.sensors & INV_XYZ_COMPASS) {
//                MPU_DMA_tx[0] = READ_COMMAND | EXT_SENS_DATA_00;
//                MPU_DMA_Run(MPU_DMA_tx, compass_data, 8);
//            }
//        } else if (DMA1_Stream3->M0AR == (uint32_t)compass_data) {
//            parse_compass(compass_data + 1, raw_compass);
//            short2long(raw_compass, raw_compass_long, 3);
//            
//            
//            for (i = 0; i < 3; i++) {
//                mine_accel[i]   = raw_accel[i]      / st->chip_cfg.accel_sens;
//                mine_gyro[i]    = raw_gyro[i]       / st->chip_cfg.gyro_sens;
//                mine_compass[i] = raw_compass[i]    * MAG_SENS * st->chip_cfg.mag_sens_adj[i];
//                raw_compass_long[i] = (long)(mine_compass[i] * (1 << 16));
//            }
//            inv_build_compass(raw_compass_long, INV_CALIBRATED, timestamp);
//            
//            arm_sub_f32(mine_accel, acc_bias, mag_tmp, VECT_SIZE);
//            arm_mat_mult_f32(&acc_calibr_mat, &mag_tmp_mat, &mine_accel_mat);
//            
//            arm_sub_f32(mine_compass, mag_bias, mag_tmp, VECT_SIZE);
//            arm_mat_mult_f32(&mag_calibr_mat, &mag_tmp_mat, &mine_compass_mat);
//            
//            mine_compass[2] = -mine_compass[2];
//            swap(&mine_compass[0], &mine_compass[1]);
//            
//            
//            mine_gyro[0] = -mine_gyro[0];
//            mine_gyro[1] = -mine_gyro[1];
//            mine_gyro[2] = -mine_gyro[2];
//            
//            new_data = BIT_MPL | BIT_MINE;
//        }
    }
}
