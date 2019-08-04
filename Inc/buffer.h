/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __buffer_H
#define __buffer_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "sdcard.h"
	 
uint32_t get_item(uint32_t start_sector, uint32_t addr, uint8_t itemsize);	 
	 
#ifdef __cplusplus
}
#endif
#endif /*__buffer_H */
