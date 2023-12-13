/*
 * tft_panel.h
 *
 *  Created on: Dec 10, 2023
 *      Author: oleksii
 *
 rgb888 to rgb565
 (uint8_t r, uint8_t g, uint8_t b) {
 return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
 }
#include "font/20pt.h"
#include "font/console8pt.c"
#include "font/console18pt.c"
 *
 *
 *
 *
 */

#ifndef INC_TFT_PANEL_8BIT_H_
#define INC_TFT_PANEL_8BIT_H_

#define pinRD 3
#define pinWR 4
#define pinRS 5
#define pinCS 6
#define pinRST 7


#define tft_RD   0x01 << pinRD
#define tft_WR   0x01 << pinWR
#define tft_RS   0x01 << pinRS
#define tft_CS   0x01 << pinCS
#define tft_RST  0x01 << pinRST
#define tft_data 0b11111111 //a0-a7
#define tft_pause //HAL_Delay(1)

#define pin_key_up 2
#define pin_key_down 0
#define pin_key_ok 1

#define tft_key_up   0x01 << pin_key_up
#define tft_key_down 0x01 << pin_key_down
#define tft_key_ok	 0x01 << pin_key_ok


#define color_BLACK		0x0000
#define color_WHITE		0xFFFF
#define color_RED		0xF800
#define color_GREEN		0x0400
#define color_BLUE		0x001F
#define color_SILVER	0xC618
#define color_GRAY		0x8410
#define color_MAROON	0x8000
#define color_YELLOW	0xFFE0
#define color_OLIVE		0x8400
#define color_LIME		0x07E0
#define color_AQUA		0x07FF
#define color_TEAL		0x0410
#define color_NAVY		0x0010
#define color_FUCHSIA	0xF81F
#define color_PURPLE	0x8010



struct tft_window {
	uint16_t image_x0;
	uint16_t image_y0;
	uint16_t image_x1;
	uint16_t image_y1;
	uint16_t cursor_x;
	uint16_t cursor_y;
	uint16_t color_font;
	uint16_t color_background;
	uint8_t *font;
};






/**************** TFT module ***************/

void tft_command(uint8_t command){

	GPIOA->BSRR = command;
	GPIOA->BRR = ~command;
	GPIOB->BRR = tft_RS|tft_WR;
	tft_pause;
	GPIOB->BSRR = tft_RS|tft_WR;
	tft_pause;
};
void tft_data8 (uint8_t data){

	GPIOA->BSRR = data;
    GPIOA->BRR = ~data;
	GPIOB->BRR =  tft_WR;
	tft_pause;
	GPIOB->BSRR = tft_WR;
	tft_pause;
};
void tft_data16 (uint16_t data){
 /*
	uint16_t data_clr = ~data;
	uint16_t data_MSB =  ((data_clr&0xFF00)<<8) |((data&0xFF00)>>8);
	uint16_t data_LSB =  ((data_clr&0xFF)<<16) | (data&0xFF);
*/
   uint8_t data_MSB = data >> 8;
   uint8_t data_LSB = data&0xFF;

	GPIOA->BSRR = data_MSB;
    GPIOA->BRR = ~data_MSB;
	GPIOB->BRR =  tft_WR;
	tft_pause;
	GPIOB->BSRR = tft_WR;
	tft_pause;
	GPIOA->BSRR = data_LSB;
	GPIOA->BRR = ~data_LSB;
	GPIOB->BRR =  tft_WR;
	tft_pause;
	GPIOB->BSRR = tft_WR;
	tft_pause;
};
uint8_t* tft_data8_block (uint8_t* block, uint32_t size){

	uint8_t data;
	for(uint32_t i=0; i< size; i++){

		data = *block++;
		GPIOA->BSRR = data;
		GPIOA->BRR = ~data;
		GPIOB->BRR =  tft_WR;
		tft_pause;
		GPIOB->BSRR = tft_WR;
		tft_pause;
	};
return block;
};
void tft_data16_block (uint16_t* block, uint32_t size){

	uint16_t data, data_clr, data_MSB, data_LSB;

	for(uint32_t i=0; i< size; i++){

		data = *(block + i);
		data_clr = ~data;
		data_MSB =  ((data_clr&0xFF00)<<8)| ((data&0xFF00)>>8);
		data_LSB =  ((data_clr&0xFF)<<16) | (data&0xFF);

		GPIOA->BSRR = data_MSB;
		GPIOB->BRR =  tft_WR;
		tft_pause;
		GPIOB->BSRR = tft_WR;
		tft_pause;
		GPIOA->BSRR = data_LSB;
		GPIOB->BRR =  tft_WR;
		tft_pause;
		GPIOB->BSRR = tft_WR;
		tft_pause;

	};

};

void tft_fast_fill_window(uint16_t color, uint32_t size){

	uint16_t data_clr = ~color;
	uint32_t data_MSB =  ((data_clr&0xFF00)<<8)| ((color&0xFF00)>>8);
	uint32_t data_LSB =  ((data_clr&0x00FF)<<16) | (color&0xFF);

	for(uint32_t i=0; i< size; i++){

			GPIOA->BSRR = data_MSB;
			GPIOB->BRR =  tft_WR;
			tft_pause;
			GPIOB->BSRR = tft_WR;
			tft_pause;
			GPIOA->BSRR = data_LSB;
			GPIOB->BRR =  tft_WR;
			tft_pause;
			GPIOB->BSRR = tft_WR;
			tft_pause;
	};
};



void tft_set_window(struct tft_window* win){

	tft_command(0x2A);
	tft_data16(win->image_x0);
	tft_data16(win->image_x1);
	tft_command(0x2B);
	tft_data16(win->image_y0);
	tft_data16(win->image_y1);
	tft_command(0x2C);

}

void tft_fast_clear(struct tft_window* win){

	uint16_t nX = win->image_x1 > win->image_x0? win->image_x1 - win->image_x0 :
												 win->image_x0 - win->image_x1 ;
	uint16_t nY = win->image_y1 > win->image_y0? win->image_y1 - win->image_y0 :
												 win->image_y0 - win->image_y1 ;
	uint32_t size = (nX+1)*(nY+1);
	tft_set_window(win);
	tft_fast_fill_window(win->color_background,size);

};


#define STOP_INIT  0xFF
#define PAUSE_INIT 0xFE


void init_tft_display(uint8_t* buf){

	uint8_t data = 0;
	GPIOB->BSRR = tft_RD|tft_WR|tft_RS|tft_RST;
	GPIOB->BRR  = tft_CS;

	while(*buf != STOP_INIT){

		data = *buf++;
		if(data == PAUSE_INIT){HAL_Delay(*buf++);continue;}
		tft_command(data);
		data = *buf++;
		if(data == 0) continue;
		buf = tft_data8_block(buf,data);

	};
};




uint8_t init_ili9486[]={
//Soft reset
0x01,0,
PAUSE_INIT,150,
//Interface Mode Control
0xb0,1,0x00,
//sleep out
0x11,0,
//RGB565
0x3a,1,0x55,
//Power Control 3
0xC2,1,0x44,
// VCOM Control 1
0xC5,4,0x00,0x00,0x00,0x00,

//PGAMCTRL Positive Gamma Control
0xE0,15,0x0f,0x1f,0x1c,0x0c,0x0f,0x08,0x48,0x98,0x37,0x0a,0x13,0x04,0x11,0x0d,0x00,

//NGAMCTRL Negative Gamma Control
0xE1,15,0x0f,0x32,0x2e,0x0b,0x0d,0x05,0x47,0x75,0x37,0x06,0x10,0x03,0x24,0x20,0x00,

//Digital Gamma Control 1
0xE2,15,0x0f,0x32,0x2e,0x0b,0x0d,0x05,0x47,0x75,0x37,0x06,0x10,0x03,0x24,0x20,0x00,

//Sleep OUT
0x11,0,

//Display Inversion OFF
0x20,0,

//Memory Access Control
0x36,1,0x28,

PAUSE_INIT,150,

//Display ON
0x29,0,

STOP_INIT,
};

const uint8_t Initili9486[] = {
                            //  Initialization commands for ILI9486 screens
								//  19 commands in list:
                            //
  0x11, 0,                  //    1: Out of sleep mode, no args

  0xF2, 8,                  //    2: ???, 8 args
  0x1C,0xA3,0x32,0x02,      //
  0xB2,0x12,0xFF,0x12,      //

  0xF1, 2,                  //    3: ???, 2 args
  0x36,0xA4,                //

  0xF8, 2,                  //    4: ???, 2 args
  0x21,0x04,                //

  0xF9, 2,                  //    5: ???, 2 args
  0x00,0x08,                //

  0xC0, 2,                  //    6: Power control 1, 2 arg
  0x0D,0x0D,                //

  0xC1, 2,                  //    7: Power control 1, 2 arg
  0x43,0x00,                //

  0xC2, 1,                  //    8: Power control 3, 1 arg
  0x00,                     //

  0xC5, 2,                  //    9: VCOM control, 2 arg
  0x00,0x48,                //

  0xB6, 3,                  //   10: Display Function Control, 3 arg
  0x00,0x22,0x3B,           //

  0xE0,15,                  //   11: Positive Gamma Control, 15 arg
  0x0f,0x24,0x1c,0x0a,0x0f, //
  0x08,0x43,0x88,0x32,0x0f, //
  0x10,0x06,0x0f,0x07,0x00, //

  0xE1,15,                  //   12: Negative Gamma Contro, 15 arg
  0x0F,0x38,0x30,0x09,0x0f, //
  0x0f,0x4e,0x77,0x3c,0x07, //
  0x10,0x05,0x23,0x1b,0x00, //

  0x20, 1,                  //   13: Display Inversion OFF, 1 arg
  0x00,                     //

  0x3A, 1,                  //   14: Interface Pixel Format, 1 arg
  0x55,                     //

  0x2A, 4,                  //   15: Column Addess Set, 4 arg
  0x00,0x00,0x01,0xDF,      //

  0x2B, 4,                  //   16: Page Addess Set, 4 arg
  0x00,0x00,0x01,0x3F,      //

  0x36, 1,                  //   17:
  0xF8,                     //      0 x 1111 1000 0xE8
                            //          |||| ||||
                            //          |||| |||*-- D0 x
                            //          |||| ||*--- D1 x
                            //          |||| |*---- D2 MH   Display Data Latch Data Order
                            //                              0 = LCD Refresh Left to Right
                            //                              1 = LCD Refresh Right to Left
                            //          |||| *----- D3 BGR  RGB/BGR Order
                            //                              0 = RGB
                            //                              1 = BGR
                            //          |||*------- D4 ML   Line Address Order
                            //                              0 = LCD Refresh Top to Bottom
                            //                              1 = LCD Refresh Bottom to Top
                            //          ||*-------- D5 MV   Page/Column Order
                            //                              0 = Normal Mode
                            //                              1 = Reverse Mode
                            //          |*--------- D6 MX   Column Address Order
                            //                              0 = Left to Right
                            //                              1 = Right to Left
                            //          *---------- D7 MY   Page Address Order
                            //                              0 = Top to Bottom
                            //                              1 = Bottom to Top

  0x29, 0,                  //   18: Display ON, no args

  0x2C, 0,                  //   19: Memory Write, no args

};



uint8_t init_r61581[]={



// Command, size block, data0...data 255,
// Pause , 0...255 ms.


0x01,0,

PAUSE_INIT,150,

0x28,0,

0x3a,1,0x55,

0x38,0,

0xb0,1,0,

0xB3,4,0x02,0x00,0x00,0x10,

0xB4,1,0x00,

0xD0,3,0x07,0x42,0x18,

0xD1,3,0x00,0x07,0x10,

0xD2,5,0x01,0x02,0xD3,0x01,0x02,

0xD4,2,0x01,0x02,

0xC0,5,0x10,0x3B,0x00,0x02,0x11,

0xC1,3,0x10,0x10,0x88,

0xC5,1,0x03,

0xC6,1,0x02,

0xC8,12,0x00,0x32,0x36,0x45,0x06,0x16,0x37,0x75,0x77,0x54,0x0c,0x00,

0xCC,1,0x00,

0x11,0,

0x36,1,0x0A,

0x20,0,

0x13,0,

PAUSE_INIT,150,

0x29,0,

0x2A,4,0x00,0x00,0x01,0xdf,

0x2B,4,0x00,0x00,0x01,0x3f,

0x2C,0,

STOP_INIT,

};

#define SET_background 0x04
#define SET_font 0x03
#define SET_cursorX 0x01
#define SET_cursorY 0x02
#define SET_font8pt 0x06
#define SET_font18pt 0x05
#define SET_fontNumber 0x07




/*

#define color_BLACK		0x0000
#define color_WHITE		0xFFFF
#define color_RED		0xF800
#define color_GREEN		0x0400
#define color_BLUE		0x001F
#define color_SILVER	0xC618
#define color_GRAY		0x8410
#define color_MAROON	0x8000
#define color_YELLOW	0xFFE0
#define color_OLIVE		0x8400
#define color_LIME		0x07E0
#define color_AQUA		0x07FF
#define color_TEAL		0x0410
#define color_NAVY		0x0010
#define color_FUCHSIA	0xF81F
#define color_PURPLE	0x8010



#define SET_background 0x04
#define SET_font 0x03
#define SET_cursorX 0x01
#define SET_cursorY 0x02
#define SET_font8pt 0x06
#define SET_font18pt 0x05
#define SET_fontNumber 0x07


// 0b0-0-0-RD-RST-RS-CS-WR

#define comm_on 0b00011000
#define comm_of 0b00011001
#define data_on 0b00011100
#define data_of	0b00011101
#define reset_on   0b00000000
#define reset_of   0b00011001

#define STOP_INIT  0xFF
#define PAUSE_INIT 0xFE

#define INIT_ILI9486 init_ili9486
#include "init_ili9486.h"
#define INIT_R61581  init_r61581
#include "init_r61581.h"




typedef struct {

char 		*i2_bus;
uint16_t 	address;
int      	chip_open;
int       	bus_open;

uint8_t 	registr[0x16];
uint8_t 	buf[8192];

pthread_mutex_t lock_registr;

struct i2c_msg msg [MAX_msg];
struct i2c_rdwr_ioctl_data rdwr;

}MCP23017_TFT;


struct TFT_i2c {
		MCP23017_TFT chip;
 uint16_t image_max_x;
 uint16_t image_max_y;
uint8_t *init_display;

};

struct TFT_windows {
	uint16_t image_x0;
	uint16_t image_y0;
	uint16_t image_x1;
	uint16_t image_y1;
	uint16_t cursor_x;
	uint16_t cursor_y;
	uint16_t color_font;
	uint16_t color_background;
	uint8_t *font;
};


#define MAX_info_block 20 // максимальное количество информационных блоков



struct TFT_screen_panel {

	struct TFT_windows *window;
	struct TFT_i2c *tft_model;

	struct TFT_info_block *info_block[MAX_info_block];
	uint8_t N_info_block;


	pthread_mutex_t lock_command;
	uint8_t buffer_command[256];
	void 	*buffer_data[256];
	uint8_t position_buffer;
	uint8_t num_commands;

	uint8_t command;
	void   *data;
	//void   *atrrib_command;
	//uint8_t status_command;

	struct itimerspec time;

	pthread_t tft_thread;

	uint8_t buf[8192];
};

void init_tft_R61581(struct TFT_screen_panel *panel);



//--------------------------------------------------------------------//
// Блок отвечает за отображение статической и динамической информации //
//-------------------------------------------------------------------//
#define MAX_change_block 7  // максимальное количество изменяемых
							// значений в блоке
struct TFT_info_block{
	uint8_t lock_block;
	uint8_t *init_block;    // массив статического отображения информации

	uint8_t *visible_block[MAX_change_block]; // изменения в блоке, n > 0 зайти в блок и изменить информацию на экране.
											  // n>2..255 зизменить с задержкой n-циклов

	uint8_t *change_block[MAX_change_block];  // параметры (курсор. шрифт) где  находится на экране изменияемый текст
	uint8_t *change_text[MAX_change_block];   // указатель на блок где находится отображаемый текст
	uint8_t *new_change_txt[MAX_change_block];// новый текст - текст, показание температуры и др датчиков
	uint8_t lock_change;                      // тtf_fast  выставляет номер блока который выводит на экран на данный момент.
											  // 0 - его здесь нет ))

	uint8_t N_change_block; 		// количество изменяемых параметров.
	uint8_t N_fast_change_block	; 	// количество быстро изменяемых значений в этом блоке
								    // берется с начала списка n- штук на вывод
	uint8_t N_change_char_text[MAX_change_block]; // количество символ в строке


	uint8_t pin[MAX_change_block]; // пин для  индикатора, счетчика и т.п. за котором он следит
	uint8_t pin_previous_values[MAX_change_block]; // предыдущие значение этого пина

	uint16_t indikator_on_color[MAX_change_block];
	uint16_t indikator_off_color[MAX_change_block];

   void (*Change_InfoBlock[MAX_change_block])(struct TFT_screen_panel *panel, int, int ); // обработка данных в этом блоке
   uint8_t (*get_pin)(uint8_t);
};



//////////////////////////////////   open ////////////////////////////////////////////////////////////
int tft_open_mcp_i2c(MCP23017_TFT *chip){

	chip->bus_open = open( chip->i2_bus , O_RDWR);
	if(chip->bus_open < 0) perror("mcp23017_TFT error open() I2C;");

return chip->bus_open; };
//////////////////////////////////   close ////////////////////////////////////////////////////////////
int tft_close_mcp_i2c (MCP23017_TFT *chip){

    int chip_close = close(chip->bus_open);
    if (chip_close <0) perror("mcp23017_TFT error close() I2C");

return chip_close;};
//////////////////////////////////   ioctl ////////////////////////////////////////////////////////////
int tft_ioctl(MCP23017_TFT *chip){

	if(tft_open_mcp_i2c(chip) < 0) return -1;

	if (ioctl(chip->bus_open, I2C_RDWR, &(chip->rdwr)) <0) perror("I2C erorr rdwr");

	return tft_close_mcp_i2c(chip);
	};
//////////////////////////////////   write byte to mcp23017 ////////////////////////////////////////////////////////////

int tft_write_byte_to_mcp (MCP23017_TFT *chip , uint8_t registr, uint8_t value){

//pthread_mutex_lock (&chip->lock_registr);

	uint8_t buf[2] = {registr,value};
	struct i2c_msg msg [1] = {{chip->address, 0, 2, buf }};  //выставить адрес и записать
	struct i2c_rdwr_ioctl_data rdwr = {msg, 1 };
	chip->registr[registr] = value;
	int  error = ioctl(chip->bus_open, I2C_RDWR,&rdwr);
	if ( error < 0 ) perror("I2C mcp23017_TFT tft_write_byte erorr rdwr");

//pthread_mutex_unlock (&chip->lock_registr);

	return  error;
};

/////////////////////////////////////// TFT command ///////////////////////////////////////////////////

void write_tft_command (struct TFT_screen_panel *panel,uint8_t value){

	MCP23017_TFT *chip = &(panel->tft_model-> chip);									 // buf[0]  - номер  регистра

	panel->buf[0] = GPIOA;
	panel->buf[1] = value;
	panel->buf[2] = comm_on;
	panel->buf[3] = value;
	panel->buf[4] = comm_of;																	// buf[1]  - что писать в этот регистр

		chip->msg[0].addr = chip->address;
		chip->msg[0].flags = 0;
		chip->msg[0].len = 5;
		chip->msg[0].buf = panel->buf;
		chip->rdwr.msgs = chip->msg;
		chip->rdwr.nmsgs = 1;


        int chip_write  = ioctl(chip->bus_open, I2C_RDWR, &(chip->rdwr) );
				if ( chip_write < 0 ) perror("I2C erorr rdwr");

 }

///////////////////////////////////////  TFT data ///////////////////////////////////////////////////

void write_tft_data (struct TFT_screen_panel *panel ,uint8_t value){



   MCP23017_TFT *chip = &(panel->tft_model-> chip);

	panel->buf[0] = GPIOA;				// buf[0]  - номер  регистра
	panel->buf[1] = value;				// buf[1]  - что писать в этот регистр
	panel->buf[2] = data_on;
	panel->buf[3] = value;
	panel->buf[4] = data_of;


        chip->msg[0].addr = chip->address;
		chip->msg[0].flags = 0;
		chip->msg[0].len = 5;
		chip->msg[0].buf = panel->buf;
		chip->rdwr.msgs = chip->msg;
		chip->rdwr.nmsgs = 1;


        int chip_write  = ioctl(chip->bus_open, I2C_RDWR, &(chip->rdwr) );
				if ( chip_write < 0 ) perror("I2C erorr rdwr");

 };


//////////////////////////  dynamic time window for font, sprite  //////////////////////////////////

int tft_dynamicWindow (struct TFT_screen_panel *panel,
									uint16_t x0,
									uint16_t y0,
									uint16_t x1,
									uint16_t y1){

	MCP23017_TFT *chip = &(panel->tft_model-> chip);

 uint8_t  x0_msb = x0 >> 8;
 uint8_t  x0_lsb = x0 & 0xFF;
 uint8_t  x1_msb = x1 >> 8;
 uint8_t  x1_lsb = x1 & 0xFF;
 uint8_t  y0_msb = y0 >> 8;
 uint8_t  y0_lsb = y0 & 0xFF;
 uint8_t  y1_msb = y1 >> 8;
 uint8_t  y1_lsb = y1 & 0xFF;


    uint8_t buf[] = {GPIOA,				// buf[0]  - номер  регистра
										// buf[1]  - что писать в этот регистр
	 0x2A,comm_on,0x2A,comm_of,			// X
	 x0_msb,data_on,x0_msb,data_of,		// X0 msb
	 x0_lsb,data_on,x0_lsb,data_of,		// X0 lsb
	 x1_msb,data_on,x1_msb,data_of,		// X1 msb
	 x1_lsb,data_on,x1_lsb,data_of, 	// X1 lsb

	 0x2B,comm_on,0x2B,comm_of, 		// Y
	 y0_msb,data_on,y0_msb,data_of,		// Y0 msb
	 y0_lsb,data_on,y0_lsb,data_of,		// Y0 lsb
	 y1_msb,data_on,y1_msb,data_of, 	// Y1 msb
	 y1_lsb,data_on,y1_lsb,data_of,  	// Y1 lsb

	 0x2C,comm_on,0x2C,comm_of, 		// start to fill
};

		chip->msg[0].addr = chip->address;
		chip->msg[0].flags = 0;
		chip->msg[0].len = 45;
		chip->msg[0].buf = buf;
		chip->rdwr.msgs = chip->msg;
		chip->rdwr.nmsgs = 1;

		int chip_write  = ioctl(chip->bus_open, I2C_RDWR, &(chip->rdwr));
		if ( chip_write < 0 ) perror("I2C erorr rdwr");


return chip_write;
};
/////////////////////////////////// TFT setWindow /////////////////////////////////////////////////

int tft_setWindow(struct TFT_screen_panel *panel,
									uint16_t x0,
									uint16_t y0,
									uint16_t x1,
									uint16_t y1){
		panel->window->image_x0	= x0;
		panel->window->image_x1	= x1;
		panel->window->image_y0	= y0;
		panel->window->image_y1	= y1;

return tft_dynamicWindow(panel,x0,y0,x1,y1);
};

///////////////////////  Actual active visible Window /////////////////////////////////

int tft_activeWindow(struct TFT_screen_panel *panel){

return tft_dynamicWindow (panel,
				   panel->window->image_x0,
				   panel->window->image_y0,
				   panel->window->image_x1,
				   panel->window->image_y1);
};


////////////////////////// TFT clear window  ///////////////////////////////////////////

int tft_clear_window (struct TFT_screen_panel *panel){

	struct TFT_windows *window = panel->window;
	MCP23017_TFT *chip = &(panel->tft_model-> chip);
	int chip_write;

	tft_activeWindow (panel);

	uint32_t size_byte = (window->image_x1 - window->image_x0)*(window->image_y1 - window->image_y0);

	uint8_t  color_msb = window->color_background >> 8;
	uint8_t  color_lsb = window->color_background & 0xFF;

	  panel->buf[0] = GPIOA;				// buf[0]  - номер  регистра
	  int len_byte = 1;						// -buf[1]  - что писать в этот регистр
	  uint16_t size_col = 1;
	  uint16_t ostatok = 0;

	if (size_byte > 960){ size_col = size_byte/960;
				   		  ostatok = size_byte%960;
						  size_byte =960;
						  };

	for(int i=0; i< size_byte; i++){
														// X*Y
		panel->buf[len_byte++]= color_msb;
		panel->buf[len_byte++]= data_on;
		panel->buf[len_byte++]= color_msb;
		panel->buf[len_byte++]= data_of;

		panel->buf[len_byte++]= color_lsb;
		panel->buf[len_byte++]= data_on;
		panel->buf[len_byte++]= color_lsb;
		panel->buf[len_byte++]= data_of;
	};
			chip->msg[0].addr = chip->address;
			chip->msg[0].flags = 0;
			chip->msg[0].len = len_byte;
			chip->msg[0].buf = panel->buf;
			chip->rdwr.msgs = chip->msg;
			chip->rdwr.nmsgs = 1;


				for(int i=0; i< size_col; i++){
					   chip_write  = ioctl(chip->bus_open, I2C_RDWR, &(chip->rdwr) );
						if ( chip_write < 0 ) perror("I2C erorr rdwr");
				};

				if(ostatok > 0){
								chip->msg[0].len = ostatok*8;
					            chip_write  = ioctl(chip->bus_open, I2C_RDWR, &(chip->rdwr) );
									if ( chip_write < 0 ) perror("I2C erorr rdwr");
								};
return chip_write;
};

////////////////////////// TFT full clear 480x320 -> fast ///////////////////////////////
// Цвет может быть задан только 8 битами

int write_tft_clear_fast (struct TFT_screen_panel *panel,uint8_t color){


     tft_setWindow (panel,0,0,panel->tft_model->image_max_x,panel->tft_model->image_max_y);


	MCP23017_TFT *chip = &(panel->tft_model-> chip);
	int chip_write;
	int len_byte;

	panel->buf[0] = GPIOA;				// задаем цвет заполнения
	panel->buf[1] = color;
	panel->buf[2] = data_on;
	panel->buf[3] = color;
	panel->buf[4] = data_of;

		chip->msg[0].addr = chip->address;
		chip->msg[0].flags = 0;
		chip->msg[0].len = 5;
		chip->msg[0].buf = panel->buf;
		chip->rdwr.msgs = chip->msg;
		chip->rdwr.nmsgs = 1;

		chip_write  = ioctl(chip->bus_open, I2C_RDWR, &(chip->rdwr));
		if ( chip_write < 0 ) perror("I2C erorr rdwr");


			panel->buf[0] = IOCON;					// переключится в режим 8 байт
			panel->buf[1] = 0b10100000;				// и пишем по кругу в регистр для строба WR
			chip->msg[0].len = 2;

			chip_write  = ioctl(chip->bus_open, I2C_RDWR, &(chip->rdwr));
			if ( chip_write < 0 ) perror("I2C erorr rdwr");

					panel->buf[0] = GPIOB_8bit;
					len_byte = 1 ;
																	// формируем буфер стробов \_/ 1-0-1
					for(int i=0; i< 3840; i++){             		// 480x2 3840*2 4 строки  заполнения экрана
						panel->buf[len_byte++]= data_on;			// строб на WR \_/
						panel->buf[len_byte++]= data_of;
					};

					chip->msg[0].len = len_byte;
																	//320  4х80 = 320
					for(int i=0; i< 80; i++){
						chip_write  = ioctl(chip->bus_open, I2C_RDWR, &(chip->rdwr));
							if ( chip_write < 0 ) perror("I2C erorr rdwr");
					};

			panel->buf[0] = IOCON_8bit;		// возращаемся в режим 16 bit
			panel->buf[1] = 0b00100000;
			chip->msg[0].len = 2;

			chip_write  = ioctl(chip->bus_open, I2C_RDWR, &(chip->rdwr));
			if ( chip_write < 0 ) perror("I2C erorr rdwr");

return chip_write;
};


const uint8_t InitCommandsList[] = {
                            //  Initialization commands for ILI9486 screens
  19,                       //  19 commands in list:
                            //
  0x11, 0,                  //    1: Out of sleep mode, no args

  0xF2, 8,                  //    2: ???, 8 args
  0x1C,0xA3,0x32,0x02,      //
  0xB2,0x12,0xFF,0x12,      //

  0xF1, 2,                  //    3: ???, 2 args
  0x36,0xA4,                //

  0xF8, 2,                  //    4: ???, 2 args
  0x21,0x04,                //

  0xF9, 2,                  //    5: ???, 2 args
  0x00,0x08,                //

  0xC0, 2,                  //    6: Power control 1, 2 arg
  0x0D,0x0D,                //

  0xC1, 2,                  //    7: Power control 1, 2 arg
  0x43,0x00,                //

  0xC2, 1,                  //    8: Power control 3, 1 arg
  0x00,                     //

  0xC5, 2,                  //    9: VCOM control, 2 arg
  0x00,0x48,                //

  0xB6, 3,                  //   10: Display Function Control, 3 arg
  0x00,0x22,0x3B,           //

  0xE0,15,                  //   11: Positive Gamma Control, 15 arg
  0x0f,0x24,0x1c,0x0a,0x0f, //
  0x08,0x43,0x88,0x32,0x0f, //
  0x10,0x06,0x0f,0x07,0x00, //

  0xE1,15,                  //   12: Negative Gamma Contro, 15 arg
  0x0F,0x38,0x30,0x09,0x0f, //
  0x0f,0x4e,0x77,0x3c,0x07, //
  0x10,0x05,0x23,0x1b,0x00, //

  0x20, 1,                  //   13: Display Inversion OFF, 1 arg
  0x00,                     //

  0x3A, 1,                  //   14: Interface Pixel Format, 1 arg
  0x55,                     //

  0x2A, 4,                  //   15: Column Addess Set, 4 arg
  0x00,0x00,0x01,0xDF,      //

  0x2B, 4,                  //   16: Page Addess Set, 4 arg
  0x00,0x00,0x01,0x3F,      //

  0x36, 1,                  //   17:
  0xF8,                     //      0 x 1111 1000 0xE8
                            //          |||| ||||
                            //          |||| |||*-- D0 x
                            //          |||| ||*--- D1 x
                            //          |||| |*---- D2 MH   Display Data Latch Data Order
                            //                              0 = LCD Refresh Left to Right
                            //                              1 = LCD Refresh Right to Left
                            //          |||| *----- D3 BGR  RGB/BGR Order
                            //                              0 = RGB
                            //                              1 = BGR
                            //          |||*------- D4 ML   Line Address Order
                            //                              0 = LCD Refresh Top to Bottom
                            //                              1 = LCD Refresh Bottom to Top
                            //          ||*-------- D5 MV   Page/Column Order
                            //                              0 = Normal Mode
                            //                              1 = Reverse Mode
                            //          |*--------- D6 MX   Column Address Order
                            //                              0 = Left to Right
                            //                              1 = Right to Left
                            //          *---------- D7 MY   Page Address Order
                            //                              0 = Top to Bottom
                            //                              1 = Bottom to Top

  0x29, 0,                  //   18: Display ON, no args

  0x2C, 0,                  //   19: Memory Write, no args

};

void TDeviceConsole::Init(void)
{
  uint8_t *cmds;
  uint8_t numCommands, numArgs;

  {
    APP_CFG_LCD_POWER_ON;
    APP_CFG_LCD_CS_OFF;
    APP_CFG_LCD_RS_OFF;
    APP_CFG_LCD_WR_OFF;
    APP_CFG_LCD_RST_OFF;
  }

  {
    APP_CFG_LCD_RST_ON;
    msDelay(50);
    APP_CFG_LCD_RST_OFF;
    msDelay(50);
  }

  APP_CFG_LCD_CS_ON;
  {
    cmds = (uint8_t*)InitCommandsList;
    numCommands = *cmds++;            // Number of commands to follow
    while (numCommands--) {           // For each command...
      APP_CFG_LCD_RS_ON;
      APP_CFG_LCD_WR_BUS(*cmds++);    //   Read and issue command
      numArgs = *cmds++;              //   Number of args to follow
      while(numArgs--) {              //   For each argument...
        APP_CFG_LCD_RS_OFF;
        APP_CFG_LCD_WR_BUS(*cmds++);  //     Read and issue argument
      }
    }
  }
  APP_CFG_LCD_CS_OFF;
}
uint8_t init_r61581[]={



// Command, size block, data0...data 255,
// Pause , 0...255 ms.


0x01,0,

PAUSE_INIT,150,

0x28,0,

0x3a,1,0x55,

0x38,0,

0xb0,1,0,

0xB3,4,0x02,0x00,0x00,0x10,

0xB4,1,0x00,

0xD0,3,0x07,0x42,0x18,

0xD1,3,0x00,0x07,0x10,

0xD2,5,0x01,0x02,0xD3,0x01,0x02,

0xD4,2,0x01,0x02,

0xC0,5,0x10,0x3B,0x00,0x02,0x11,

0xC1,3,0x10,0x10,0x88,

0xC5,1,0x03,

0xC6,1,0x02,

0xC8,12,0x00,0x32,0x36,0x45,0x06,0x16,0x37,0x75,0x77,0x54,0x0c,0x00,

0xCC,1,0x00,

0x11,0,

0x36,1,0x28,

0x20,0,

0x13,0,

PAUSE_INIT,150,

0x29,0,

0x2A,4,0x00,0x00,0x01,0xdf,

0x2B,4,0x00,0x00,0x01,0x3f,

0x2C,0,

STOP_INIT,

};/*

LCD_Write_COM(0xB0);
		LCD_Write_DATA(0x1E);

		LCD_Write_COM(0xB0);
		LCD_Write_DATA(0x00);

		LCD_Write_COM(0xB3);
		LCD_Write_DATA(0x02);
		LCD_Write_DATA(0x00);
		LCD_Write_DATA(0x00);
		LCD_Write_DATA(0x10);

		LCD_Write_COM(0xB4);
		LCD_Write_DATA(0x00);//0X10

// 		LCD_Write_COM(0xB9); //PWM Settings for Brightness Control
// 		LCD_Write_DATA(0x01);// Disabled by default.
// 		LCD_Write_DATA(0xFF); //0xFF = Max brightness
// 		LCD_Write_DATA(0xFF);
// 		LCD_Write_DATA(0x18);

		LCD_Write_COM(0xC0);
		LCD_Write_DATA(0x03);
		LCD_Write_DATA(0x3B);//
		LCD_Write_DATA(0x00);
		LCD_Write_DATA(0x00);
		LCD_Write_DATA(0x00);
		LCD_Write_DATA(0x01);
		LCD_Write_DATA(0x00);//NW
		LCD_Write_DATA(0x43);

		LCD_Write_COM(0xC1);
		LCD_Write_DATA(0x08);
		LCD_Write_DATA(0x15);//CLOCK
		LCD_Write_DATA(0x08);
		LCD_Write_DATA(0x08);

		LCD_Write_COM(0xC4);
		LCD_Write_DATA(0x15);
		LCD_Write_DATA(0x03);
		LCD_Write_DATA(0x03);
		LCD_Write_DATA(0x01);

		LCD_Write_COM(0xC6);
		LCD_Write_DATA(0x02);

		LCD_Write_COM(0xC8);
		LCD_Write_DATA(0x0c);
		LCD_Write_DATA(0x05);
		LCD_Write_DATA(0x0A);//0X12
		LCD_Write_DATA(0x6B);//0x7D
		LCD_Write_DATA(0x04);
		LCD_Write_DATA(0x06);//0x08
		LCD_Write_DATA(0x15);//0x0A
		LCD_Write_DATA(0x10);
		LCD_Write_DATA(0x00);
		LCD_Write_DATA(0x60);//0x23

		LCD_Write_COM(0x36);
		LCD_Write_DATA(0x0A);

		LCD_Write_COM(0x0C);
		LCD_Write_DATA(0x55);

		LCD_Write_COM(0x3A);
		LCD_Write_DATA(0x55);

		LCD_Write_COM(0x38);

		LCD_Write_COM(0xD0);
		LCD_Write_DATA(0x07);
		LCD_Write_DATA(0x07);//VCI1
		LCD_Write_DATA(0x14);//VRH 0x1D
		LCD_Write_DATA(0xA2);//BT 0x06

		LCD_Write_COM(0xD1);
		LCD_Write_DATA(0x03);
		LCD_Write_DATA(0x5A);//VCM  0x5A
		LCD_Write_DATA(0x10);//VDV

		LCD_Write_COM(0xD2);
		LCD_Write_DATA(0x03);
		LCD_Write_DATA(0x04);//0x24
		LCD_Write_DATA(0x04);

		LCD_Write_COM(0x11);
		delay(150);

		LCD_Write_COM(0x2A);
		LCD_Write_DATA(0x00);
		LCD_Write_DATA(0x00);
		LCD_Write_DATA(0x01);
		LCD_Write_DATA(0xDF);//320

		LCD_Write_COM(0x2B);
		LCD_Write_DATA(0x00);
		LCD_Write_DATA(0x00);
		LCD_Write_DATA(0x01);
		LCD_Write_DATA(0x3F);//480


		delay(100);

		LCD_Write_COM(0x29);
		delay(30);

		LCD_Write_COM(0x2C);
		delay(30);
		break;

	case R61581:
	LCD_Write_COM(0x2a);
	LCD_Write_DATA(x1>>8);
	LCD_Write_DATA(x1);
	LCD_Write_DATA(x2>>8);
	LCD_Write_DATA(x2);
	LCD_Write_COM(0x2b);
	LCD_Write_DATA(y1>>8);
	LCD_Write_DATA(y1);
	LCD_Write_DATA(y2>>8);
	LCD_Write_DATA(y2);
	LCD_Write_COM(0x2c);
	break;

case ILI9486:
	LCD_Write_COM(0x11);		// Sleep OUT
	delay(50);

	LCD_Write_COM(0xF2);		// ?????
	LCD_Write_DATA(0x1C);
	LCD_Write_DATA(0xA3);
	LCD_Write_DATA(0x32);
	LCD_Write_DATA(0x02);
	LCD_Write_DATA(0xb2);
	LCD_Write_DATA(0x12);
	LCD_Write_DATA(0xFF);
	LCD_Write_DATA(0x12);
	LCD_Write_DATA(0x00);

	LCD_Write_COM(0xF1);		// ?????
	LCD_Write_DATA(0x36);
	LCD_Write_DATA(0xA4);

	LCD_Write_COM(0xF8);		// ?????
	LCD_Write_DATA(0x21);
	LCD_Write_DATA(0x04);

	LCD_Write_COM(0xF9);		// ?????
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x08);

	LCD_Write_COM(0xC0);		// Power Control 1
	LCD_Write_DATA(0x0d);
	LCD_Write_DATA(0x0d);

	LCD_Write_COM(0xC1);		// Power Control 2
	LCD_Write_DATA(0x43);
	LCD_Write_DATA(0x00);

	LCD_Write_COM(0xC2);		// Power Control 3
	LCD_Write_DATA(0x00);

	LCD_Write_COM(0xC5);		// VCOM Control
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x48);

	LCD_Write_COM(0xB6);		// Display Function Control
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x22);		// 0x42 = Rotate display 180 deg.
	LCD_Write_DATA(0x3B);

	LCD_Write_COM(0xE0);		// PGAMCTRL (Positive Gamma Control)
	LCD_Write_DATA(0x0f);
	LCD_Write_DATA(0x24);
	LCD_Write_DATA(0x1c);
	LCD_Write_DATA(0x0a);
	LCD_Write_DATA(0x0f);
	LCD_Write_DATA(0x08);
	LCD_Write_DATA(0x43);
	LCD_Write_DATA(0x88);
	LCD_Write_DATA(0x32);
	LCD_Write_DATA(0x0f);
	LCD_Write_DATA(0x10);
	LCD_Write_DATA(0x06);
	LCD_Write_DATA(0x0f);
	LCD_Write_DATA(0x07);
	LCD_Write_DATA(0x00);

	LCD_Write_COM(0xE1);		// NGAMCTRL (Negative Gamma Control)
	LCD_Write_DATA(0x0F);
	LCD_Write_DATA(0x38);
	LCD_Write_DATA(0x30);
	LCD_Write_DATA(0x09);
	LCD_Write_DATA(0x0f);
	LCD_Write_DATA(0x0f);
	LCD_Write_DATA(0x4e);
	LCD_Write_DATA(0x77);
	LCD_Write_DATA(0x3c);
	LCD_Write_DATA(0x07);
	LCD_Write_DATA(0x10);
	LCD_Write_DATA(0x05);
	LCD_Write_DATA(0x23);
	LCD_Write_DATA(0x1b);
	LCD_Write_DATA(0x00);

	LCD_Write_COM(0x20);		// Display Inversion OFF
	LCD_Write_DATA(0x00);//C8

	LCD_Write_COM(0x36);		// Memory Access Control
	LCD_Write_DATA(0x0A);

	LCD_Write_COM(0x3A);		// Interface Pixel Format
	LCD_Write_DATA(0x55);

	LCD_Write_COM(0x2A);		// Column Addess Set
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x01);
	LCD_Write_DATA(0xDF);

	LCD_Write_COM(0x002B);		// Page Address Set
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x01);
	LCD_Write_DATA(0x3f);
	delay(50);
	LCD_Write_COM(0x0029);		// Display ON
	LCD_Write_COM(0x002C);		// Memory Write
	break;
	case ILI9486:
	LCD_Write_COM(0x2a);
	LCD_Write_DATA(x1>>8);
	LCD_Write_DATA(x1);
	LCD_Write_DATA(x2>>8);
	LCD_Write_DATA(x2);
	LCD_Write_COM(0x2b);
	LCD_Write_DATA(y1>>8);
	LCD_Write_DATA(y1);
	LCD_Write_DATA(y2>>8);
	LCD_Write_DATA(y2);
	LCD_Write_COM(0x2c);
	break;


 */

#endif /* INC_TFT_PANEL_8BIT_H_ */
