#ifndef FRAME_SYNC_H
#define FRAME_SYNC_H

#include "main.h"

#define MAX_LENGTH_DATA 255

typedef enum{
	CRC_8	= 1,
	CRC_16	= 2,
	CRC_32	= 4,
} CRC_TYPE_t;

typedef struct{
	CRC_TYPE_t crc_type;
	uint32_t tx_checksum;
	uint32_t rx_checksum;
	uint8_t is_using_stuff_byte;
	uint8_t num_dle_byte;
	uint8_t rx_state;
	uint8_t rx_buf[MAX_LENGTH_DATA];
	uint8_t rx_pointer;
	uint8_t rx_length;
	uint8_t rx_num_crc_byte;
	uint32_t rx_crc;
	uint32_t receiving_timer;
} FRAME_SYNC_DATA_t;

void FRAME_SYNC_Init();
uint8_t FRAME_SYNC_CRC_Type();
void FRAME_SYNC_Change_Setting(FRAME_SYNC_DATA_t *p_new_data);
void FRAME_SYNC_Transmit(uint8_t *tx_frame, uint8_t size);
void FRAME_SYNC_Receive(uint8_t rx_data);
void FRAME_SYNC_RxCpltCallback(uint8_t *p_rx_data, uint8_t data_size);
void FRAME_SYNC_RxFailCallback(uint8_t *p_rx_data, uint8_t data_size);
void FRAME_SYNC_Handle();

#endif
