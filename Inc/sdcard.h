#ifndef __SDCARD_H
#define __SDCARD_H

#define SD_SECTOR_SIZE 512
#include "stm32f1xx_hal.h"

uint8_t SD_init(void);
uint8_t SD_ReadSector(uint32_t BlockNumb,uint8_t *buff);

#endif
