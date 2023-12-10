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






 */

#endif /* INC_TFT_PANEL_8BIT_H_ */
