#include "midi.h"

uint8_t midi_note  =0xff;
uint8_t midi_velocity  =0x00;
uint8_t midi_buf[MIDI_BUF_SIZE];
char *midi_pat = "+IPD,";
uint8_t midi_pat_p;
uint8_t midi_msg_len;
uint8_t midi_msg_read;
uint8_t midi_channel;

void start_midi(uint8_t channel) {
		midi_channel = channel;
		midi_msg_len = 0;
	  midi_pat_p = 0;
		midi_msg_read = 0;
		HAL_UART_Receive_IT(&huart1, midi_buf, 1);
}

uint8_t midi_get_note() {
	return midi_note;
}	

uint8_t midi_get_velocity() {
	return midi_velocity;
}	

void midi_parse_msg() {
	uint8_t p = 0;
	while (p<midi_msg_len) {
		while (((midi_buf[p] & 0xf0)!=0x90) && ((midi_buf[p] & 0xf0)!=0x80) && (p<midi_msg_len)) p++;
		if (p<midi_msg_len) {
			if (((midi_buf[p] & 0xf0) == 0x90) && ((midi_buf[p] & 0x0f) == midi_channel)) { //note on
				midi_note = midi_buf[p+1] & 0x7f;
				midi_velocity = midi_buf[p+2] & 0x7f;
			}
			if (((midi_buf[p] & 0xf0) == 0x80) && ((midi_buf[p] & 0x0f) == midi_channel) && (midi_note == (midi_buf[p+1] & 0x7f))) { //note off
				midi_note = 0xff;
			}
			p+=3;
		}
	}
}

void midi_rx_callback(UART_HandleTypeDef *huart){
	if (midi_msg_read) { //read midi message
		midi_parse_msg();
		start_midi(midi_channel); //once again
	} else {
		if (midi_pat_p<=4) { //finding "+IPD,"
			if (midi_buf[0]==midi_pat[midi_pat_p]) { //matching separate character of the pattern
				midi_pat_p++; //success, waiting for next character
				HAL_UART_Receive_IT(&huart1, midi_buf, 1);		
			} else {
				start_midi(midi_channel); //fail, try again
			}
		} else {  //pattern found, reading length
			if (midi_buf[0]==':') { //all digits of length are read
				midi_msg_read = 1;
				HAL_UART_Receive_IT(&huart1, midi_buf, (midi_msg_len>MIDI_BUF_SIZE)?MIDI_BUF_SIZE:midi_msg_len); //waiting next character
			} else { //reading next digit
				if (midi_buf[0]>'0' && midi_buf[0]<='9') { //next symbol is digit, ok
					midi_msg_len = (midi_msg_len * 10) + (midi_buf[0] - '0'); //update length
					HAL_UART_Receive_IT(&huart1, midi_buf, 1); //waiting next character
				}	else { //fail
					start_midi(midi_channel); //once again
				}			
			}
		}
	}
}

void midi_tx_callback(UART_HandleTypeDef *huart){
//	midi_note = 1;
}
