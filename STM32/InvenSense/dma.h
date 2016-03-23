#ifndef DMA_H
#define DMA_H

#include "stdint.h"

void MPU_DMA_Init(void);
void MPU_DMA_Run(uint8_t *tx, uint8_t *rx, uint8_t size);

#endif // DMA_H
