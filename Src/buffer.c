#include "buffer.h"

uint8_t buf_buf[SD_SECTOR_SIZE * 2];
uint32_t loaded_sector = UINT32_MAX;
uint32_t result;

uint32_t get_item(uint32_t start_sector, uint32_t addr, uint8_t itemsize){
	uint32_t need_sector = start_sector + (addr * itemsize / SD_SECTOR_SIZE);
	uint16_t need_offset = (addr * itemsize) % SD_SECTOR_SIZE;
  uint8_t overlap = (SD_SECTOR_SIZE-need_offset)<itemsize;
	
	if (need_sector!=loaded_sector){
		SD_ReadSector(need_sector,buf_buf);
		loaded_sector = need_sector;
	}
	if (overlap)
		SD_ReadSector(need_sector+1,buf_buf+SD_SECTOR_SIZE);
	
	result = *(uint32_t *)(buf_buf+need_offset);
	
	return result;
}
