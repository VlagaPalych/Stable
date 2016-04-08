#ifndef MAIN_H
#define MAIN_H

#include "stdint.h"
#include "extra_math.h"

#define BIT_MPL   0x01
#define BIT_DMP   0x02
#define BIT_MINE  0x04

extern uint8_t algorithm;
extern uint8_t new_data;
extern uint8_t meas1;

extern float mpl_euler[3];
extern float dmp_euler[3];
extern float mine_euler[3];

extern long dmp_quat_data[4];
extern Quat dmp_orient;

extern float mine_accel[3];
extern float mine_gyro[3];
extern float mine_compass[3];
extern float compass_bias[3];
extern float compass_scale[3];
extern Quat mine_orient;

void Delay_ms(uint16_t ms);

void flash_unlock();
void flash_lock();
uint8_t flash_ready(void);
int flash_erase_sector(uint32_t address);
void flash_erase_all_pages(void);
void flash_write(uint32_t address, uint32_t data);
uint32_t flash_read(uint32_t address);

void mag_calibr_save_to_flash();
void mag_calibr_restore_from_flash();

#endif // MAIN_H