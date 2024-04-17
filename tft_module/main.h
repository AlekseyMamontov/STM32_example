/*
 USER CODE BEGIN Header

  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.

 USER CODE END Header */


#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"



struct Ram_to_Flash_block{

	// counter_cycles_write_flash

	uint32_t counter_wr_flash;

	// thermostat matrix
	uint32_t matrix_on_temp;
	uint32_t matrix_off_temp;
	CAN_TxMailBox_TypeDef msg_matrix_on;
	CAN_TxMailBox_TypeDef msg_matrix_off;

	// thermostat punch
	uint32_t punch_on_temp;
	uint32_t punch_off_temp;
	CAN_TxMailBox_TypeDef msg_punch_on;
	CAN_TxMailBox_TypeDef msg_punch_off;

	// rele_delay;
	uint32_t time_ms;
	CAN_TxMailBox_TypeDef msg_delay_on;
	CAN_TxMailBox_TypeDef msg_delay_off;

	uint16_t stat_matrix;
	uint16_t correct_temp_matrix;
	uint16_t stat_punch;
	uint16_t correct_temp_punch;
    uint16_t stat_delay;
    uint16_t reserv;// ==

    // id_msg_object
    uint32_t id_matrix;
    uint32_t matrix_mask[2];

    uint32_t id_punch;
    uint32_t punch_mask[2];

    uint32_t id_counter;
    uint32_t counter_mask[2];

    uint32_t id_cylindr;
    uint32_t cylindr_mask[2];

    uint32_t id_button;
    uint32_t button_mask[2];

    uint8_t  bit_offset[8];
    uint8_t  status_object[8];

};


#define THERMO_ON_OFF     0x01
#define THERMO_HOT_COOL 0x8000
#define THERMO_DISABLED 0x4000 // 1-disabled

struct Block_Thermostat{

	uint32_t *current_temp;
	uint16_t *correct_temp;
	uint32_t *on_temp;
	uint32_t *off_temp;
	CAN_TxMailBox_TypeDef* msg_on;
	CAN_TxMailBox_TypeDef* msg_off;
	uint16_t *stat;
	uint8_t* text_on;
	uint8_t* text_off;
	const
	uint8_t  n_symvol;
};

// Rele_Delay 100ms
#define RELE_ON  0x01
#define RELE_SEND_OFF 0x02
#define RELE_SEND_ON  0x04
#define RELE_DELAY 0x08

struct Rele_Delay{

	uint32_t *time_100ms;
	CAN_TxMailBox_TypeDef* msg_on;
	CAN_TxMailBox_TypeDef* msg_off;
	uint16_t* stat;

};

struct Data_can_msg{

	uint32_t* data_msg;
	uint32_t* id_msg;
	//uint32_t* dlc;
	uint32_t* data_frame;
	uint32_t* mask_frame;

	uint8_t*  bit_offset;
	uint8_t*  status;

};



void init_controller_STM32F072(void);
uint8_t CAN_transmit (CAN_TxMailBox_TypeDef *tx);
void Processing_SDO_Object(CAN_FIFOMailBox_TypeDef*);
void Thermostat_processing(struct Block_Thermostat* temp);
void thermostat_init(struct Block_Thermostat* temp);
void Rele_delay_processing(struct Rele_Delay* rele);
void Rele_delay_init(struct Rele_Delay* rele);
void default_load_rаm(uint8_t* ram, uint8_t* flash,uint32_t size);

/*----------------- CAN_Buffer ------------------*/

struct CAN_frame{

	uint32_t id;
	uint32_t  dlc;
	uint32_t msg[2];

};
#define MAX_BUFFER_CAN 128

struct  CAN_buffer{

	struct CAN_frame* rdata;
	struct CAN_frame* wdata;
	struct CAN_frame* begin_frame;
	struct CAN_frame* end_frame;

};

void init_buffer(struct CAN_buffer* buf,struct CAN_frame *frame);
uint8_t read_can_buffer(struct CAN_buffer* buf,struct CAN_frame *frame);
uint8_t write_can_buffer(struct CAN_buffer* buf,struct CAN_frame *frame);
struct CAN_frame* read_can_buffer2(struct CAN_buffer* buf);

/*---------------Keys -------------------*/

#define MAX_BUFFER_KEY 20

struct KEY_buffer{

	uint8_t* r_keys;
	uint8_t* w_keys;
	uint8_t* begin_buf;
	uint8_t* end_buf;

};
void Keyboard_init(struct KEY_buffer*,uint8_t *);
uint8_t read_keys_buffer(struct KEY_buffer*);
void write_keys_buffer(struct KEY_buffer* key,uint8_t data);
// screens key_processing



/* -----------------------------------------------------------*/


void Error_Handler(void);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
