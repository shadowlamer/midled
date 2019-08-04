/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __midi_H
#define __midi_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "usart.h"

#define MIDI_BUF_SIZE 16
	 
void start_midi(uint8_t channel);	 
uint8_t midi_get_note(void);	 
uint8_t midi_get_velocity(void);	 
void midi_rx_callback(UART_HandleTypeDef *huart);
void midi_tx_callback(UART_HandleTypeDef *huart);
	 
#ifdef __cplusplus
}
#endif
#endif /*__buffer_H */
