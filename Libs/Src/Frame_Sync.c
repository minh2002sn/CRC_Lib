#include "Frame_Sync.h"
#include "checksum.h"

#define STX				0x02
#define ETX				0x03
#define DLE				0x10

enum{
	SEARCHING_STX,
	RECEIVING_NUM_DATA,
	RECEIVING_DATA,
	RECEIVING_ETX,
	RECEIVING_CRC,
};

#define huart	huart2

extern UART_HandleTypeDef huart;

FRAME_SYNC_DATA_t FS_Data = {CRC_8};

static void Rx_Reset(){
	FRAME_SYNC_DATA_t temp_data = {FS_Data.crc_type};
	FS_Data = temp_data;
}

static void CRC_Update(uint32_t *crc, uint8_t data){
	switch(FS_Data.crc_type){
		case CRC_8:
			*crc = update_crc_8((uint8_t)*crc, data);
			break;
		case CRC_16:
			*crc = update_crc_16((uint16_t)*crc, data);
			break;
		case CRC_32:
			*crc = update_crc_32(*crc, data);
			break;
		default:
			break;
	}
}

static void Stuffed_Transmit(uint8_t tx_data){
	if(FS_Data.is_using_tx_stuff_byte && tx_data == DLE){
		CRC_Update(&FS_Data.tx_checksum, DLE);
		HAL_UART_Transmit(&huart, &tx_data, 1, 100);
	}
	CRC_Update(&FS_Data.tx_checksum, tx_data);
	HAL_UART_Transmit(&huart, &tx_data, 1, 100);
}

static void Check_Frame(uint8_t *p_frame, uint32_t size){
	for(int i = 0; i < size; i++){
		if(p_frame[i] < 0x20 || p_frame[i] > 0x7E){
			FS_Data.is_using_tx_stuff_byte = 1;
			return;
		}
	}
}

static void CRC_Transmit(uint32_t *crc){
	switch(FS_Data.crc_type){
		case CRC_8:
			HAL_UART_Transmit(&huart2, (uint8_t *)crc + 0, 1, 100);
			break;
		case CRC_16:
			HAL_UART_Transmit(&huart2, (uint8_t *)crc + 0, 1, 100);
			HAL_UART_Transmit(&huart2, (uint8_t *)crc + 1, 1, 100);
			break;
		case CRC_32:
			*crc = ~(*crc);
			HAL_UART_Transmit(&huart2, (uint8_t *)crc + 0, 1, 100);
			HAL_UART_Transmit(&huart2, (uint8_t *)crc + 1, 1, 100);
			HAL_UART_Transmit(&huart2, (uint8_t *)crc + 2, 1, 100);
			HAL_UART_Transmit(&huart2, (uint8_t *)crc + 3, 1, 100);
			break;
		default:
			break;
	}
}

void FRAME_SYNC_Init(){

}

uint8_t FRAME_SYNC_CRC_Type(){
	return FS_Data.crc_type;
}

void FRAME_SYNC_Change_Setting(FRAME_SYNC_DATA_t *p_new_data){
	FS_Data = *p_new_data;
}

void FRAME_SYNC_Transmit(uint8_t *tx_frame, uint8_t size){
	uint8_t temp_data;
	if(FS_Data.crc_type == CRC_32){
		FS_Data.tx_checksum = 0xFFFFFFFF;
	} else{
		FS_Data.tx_checksum = 0;
	}
	Check_Frame(tx_frame, size);

	// Transmit STX
	if(FS_Data.is_using_tx_stuff_byte){
		temp_data = DLE;
		CRC_Update(&FS_Data.tx_checksum, DLE);
		HAL_UART_Transmit(&huart2, &temp_data, 1, 100);
	}
	CRC_Update(&FS_Data.tx_checksum, STX);
	temp_data = STX;
	HAL_UART_Transmit(&huart2, &temp_data, 1, 100);

	// Transmit data length
	Stuffed_Transmit(size);

	// Transmit data
	for(int i = 0; i < size; i++){
		Stuffed_Transmit(tx_frame[i]);
	}

	// Transmit ETX
	if(FS_Data.is_using_tx_stuff_byte){
		temp_data = DLE;
		CRC_Update(&FS_Data.tx_checksum, DLE);
		HAL_UART_Transmit(&huart2, &temp_data, 1, 100);
	}
	CRC_Update(&FS_Data.tx_checksum, ETX);
	temp_data = ETX;
	HAL_UART_Transmit(&huart2, &temp_data, 1, 100);

	// Transmit crc
	CRC_Transmit(&FS_Data.tx_checksum);

	FS_Data.is_using_tx_stuff_byte = 0;
}

void FRAME_SYNC_Receive(uint8_t rx_data){
	// Stuff byte handle
	if(rx_data == DLE){
		if(FS_Data.rx_state == SEARCHING_STX){
			FS_Data.is_using_rx_stuff_byte = 1;
			if(FS_Data.crc_type == CRC_32){
				FS_Data.rx_checksum = 0xFFFFFFFF;
			} else{
				FS_Data.rx_checksum = 0;
			}
			CRC_Update(&FS_Data.rx_checksum, DLE);
			return;
		} else{
			if(FS_Data.is_using_rx_stuff_byte && FS_Data.num_dle_byte == 0){
				CRC_Update(&FS_Data.rx_checksum, DLE);
				FS_Data.num_dle_byte = 1;
				return;
			}
		}
	}
	FS_Data.num_dle_byte = 0;

	// Data handle
	switch(FS_Data.rx_state){
		case SEARCHING_STX:
			if(rx_data == STX){
				CRC_Update(&FS_Data.rx_checksum, rx_data);
				FS_Data.rx_state = RECEIVING_NUM_DATA;
			} else{
				Rx_Reset();
				FRAME_SYNC_RxFailCallback(FS_Data.rx_buf, FS_Data.rx_length);
			}
			break;
		case RECEIVING_NUM_DATA:
			CRC_Update(&FS_Data.rx_checksum, rx_data);
			FS_Data.rx_length = rx_data;
			FS_Data.rx_state = RECEIVING_DATA;
			break;
		case RECEIVING_DATA:
			CRC_Update(&FS_Data.rx_checksum, rx_data);
			FS_Data.rx_buf[FS_Data.rx_pointer++] = rx_data;
			if(FS_Data.rx_pointer == FS_Data.rx_length){
				FS_Data.rx_state = RECEIVING_ETX;
			}
			break;
		case RECEIVING_ETX:
			if(rx_data == ETX){
				CRC_Update(&FS_Data.rx_checksum, rx_data);
				FS_Data.rx_state = RECEIVING_CRC;
			} else{
				Rx_Reset();
				FRAME_SYNC_RxFailCallback(FS_Data.rx_buf, FS_Data.rx_length);
			}
			break;
		case RECEIVING_CRC:
			FS_Data.rx_crc |= (uint32_t)rx_data << ((FS_Data.rx_num_crc_byte) * 8);
			FS_Data.rx_num_crc_byte++;
			if(FS_Data.rx_num_crc_byte == FS_Data.crc_type){
				if(FS_Data.crc_type == CRC_32) FS_Data.rx_checksum = ~FS_Data.rx_checksum;
				if(FS_Data.rx_crc == FS_Data.rx_checksum){
					FRAME_SYNC_RxCpltCallback(FS_Data.rx_buf, FS_Data.rx_length);
				} else{
					FRAME_SYNC_RxFailCallback(FS_Data.rx_buf, FS_Data.rx_length);
				}
				Rx_Reset();
			}
			break;
		default:
			break;
	}

	FS_Data.receiving_timer = HAL_GetTick();

}

void FRAME_SYNC_Handle(){
	if(FS_Data.rx_state != SEARCHING_STX && (HAL_GetTick() - FS_Data.receiving_timer > 500)){
		Rx_Reset();
		FRAME_SYNC_RxFailCallback(FS_Data.rx_buf, FS_Data.rx_length);
	}
}
