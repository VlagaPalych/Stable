#ifndef DMA_H
#define DMA_H

#include "stdint.h"

extern "C" void mpu_dma_init(void);
extern "C" void mpu_dma_run(uint8_t *tx, uint8_t *rx, uint16_t size);

extern uint8_t dma_buf_tx[100];
extern uint8_t dma_buf_rx[100];

#endif // DMA_H
