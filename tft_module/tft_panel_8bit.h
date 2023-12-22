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
#include "fonts.h"
 *
 *
 *
 *
 */
#include "fonts.h"
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
	const uint8_t *font;
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
	GPIOB->BRR =  tft_WR; // \_
	GPIOB->BSRR = tft_WR; // _/
	GPIOA->BSRR = data_LSB;
	GPIOA->BRR = ~data_LSB;
	GPIOB->BRR =  tft_WR;
	GPIOB->BSRR = tft_WR;

};
const uint8_t* tft_data8_block (const uint8_t* block, uint32_t size){

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
			GPIOB->BSRR = tft_WR;
			GPIOA->BSRR = data_LSB;
			GPIOB->BRR =  tft_WR;
			GPIOB->BSRR = tft_WR;

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




///////////////// Init tft display //////////////////////

#define STOP_INIT  0xFF
#define PAUSE_INIT 0xFE

void init_tft_display(const uint8_t* buf){

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



///////////////////////// Write char to window Screen Panel //////////////////////////////

void write_char_tft (struct tft_window *window,uint8_t symvol){

	uint32_t  color_msb = (~window->color_font)&0xFF00;
			  color_msb <<= 8;
	 	 	  color_msb |= (window->color_font&0xFF00)>>8;

	uint32_t  color_lsb = (~window->color_font) & 0xFF;
			  color_lsb <<=16;
			  color_lsb |= window->color_font&0xFF;

	uint32_t  background_msb = (~window->color_background)&0xFF00;
			  background_msb <<= 8;
			  background_msb |= (window->color_background&0xFF00)>>8;

	uint32_t  background_lsb = (~window->color_background)&0xFF;
			  background_lsb <<= 16;
			  background_lsb |= window->color_background&0xFF00;


	uint16_t  sizeXfont = *(window->font);
	uint16_t  colXbayt = sizeXfont/8;
	uint16_t  sizeYfont = window->font[1];

	const uint8_t*  mem_znak = window->font + 2;
			  mem_znak +=(symvol*sizeYfont*colXbayt);
    uint8_t   data, mask_data;





		for( uint8_t n=0; n < sizeYfont; n++){

			for (uint8_t l=0; l < colXbayt; l++){

				data = *mem_znak;
				mask_data = 0x80;

				for (uint8_t i=0; i<8 ; i++){

					if (data & mask_data){

						GPIOA->BSRR = color_msb;
						GPIOB->BRR =  tft_WR;
						GPIOB->BSRR = tft_WR;
						GPIOA->BSRR = color_lsb;
						GPIOB->BRR =  tft_WR;
						GPIOB->BSRR = tft_WR;

					} else {

						GPIOA->BSRR = background_msb;
						GPIOB->BRR =  tft_WR;
						GPIOB->BSRR = tft_WR;
						GPIOA->BSRR = background_lsb;
						GPIOB->BRR =  tft_WR;
						GPIOB->BSRR = tft_WR;

					};

					mask_data >>= 1;

				};

			mem_znak ++;

			};
	  };
};
////////////////////////////// print char ////////////////////////////////////////////////

void print_char_tft (struct tft_window *window,uint8_t symvol){

uint16_t x0,x1,y0,y1;

	 x0 = (window->font[0])*(window->cursor_x)+(window->image_x0);
	 x1 = x0 + ((window->font[0])-1);

		if(x1 > window->image_x1 ){	x0 = window->image_x0;
									x1 = x0 + ((window->font[0])-1);
									window->cursor_x = 0;
									window->cursor_y = window->cursor_y + 1;
								};

	y0 = (window->font[1])*(window->cursor_y)+(window->image_y0);
	y1 = y0 + ((window->font[1])-1);

		if(y1 > window->image_y1 ){	y0 = window->image_y0;
									y1 = y0 + ((window->font[1])-1);
									window->cursor_y = 0;
								};
	if(symvol >= 0x20) {

	symvol = symvol - 0x20;

  		tft_command(0x2A);
    	tft_data16(x0);
    	tft_data16(x1);
    	tft_command(0x2B);
    	tft_data16(y0);
    	tft_data16(y1);
    	tft_command(0x2C);


	write_char_tft(window, symvol);

	};

	window->cursor_x = (window->cursor_x) + 1;

};

void tft_print_utf8 (struct tft_window *window,uint8_t* buf){

uint16_t x0,x1,y0,y1,symvol;

	 x0 = (window->font[0])*(window->cursor_x)+(window->image_x0);
	 x1 = x0 + ((window->font[0])-1);

		if(x1 > window->image_x1 ){	x0 = window->image_x0;
									x1 = x0 + ((window->font[0])-1);
									window->cursor_x = 0;
									window->cursor_y = window->cursor_y + 1;
								};

	y0 = (window->font[1])*(window->cursor_y)+(window->image_y0);
	y1 = y0 + ((window->font[1])-1);

		if(y1 > window->image_y1 ){	y0 = window->image_y0;
									y1 = y0 + ((window->font[1])-1);
									window->cursor_y = 0;
								};
	if(symvol >= 0x20) {

	//symvol = symvol - 0x20;

  		tft_command(0x2A);
    	tft_data16(x0);
    	tft_data16(x1);
    	tft_command(0x2B);
    	tft_data16(y0);
    	tft_data16(y1);
    	tft_command(0x2C);


	write_char_tft(window, symvol);

	};

	window->cursor_x = (window->cursor_x) + 1;

};

#define SET_background 0x04
#define SET_font 0x03
#define SET_cursorX 0x01
#define SET_cursorY 0x02
#define SET_font8pt 0x06
#define SET_font18pt 0x05
#define SET_fontNumber 0x07


void tft_printf(struct tft_window *window, uint8_t *text){

	while (*text){

	 if (*text >= 0x20){


		print_char_tft(window,*text);

		}else { switch (*text){

					case SET_cursorX: //set cursor X
						text++;
						window->cursor_x = *text;
						break;
					case SET_cursorY:
						text++;
						window->cursor_y = *text;
						break;

					case SET_font:
						text++;
						window->color_font = *text;
						window->color_font = window->color_font << 8;
						text++;
						window->color_font = window->color_font | (*text);
						break;
					case SET_background:
						text++;
						window->color_background = *text;
						window->color_background = window->color_background << 8;
						text++;
						window->color_background = window->color_background | (*text);
					break;

					case SET_font18pt:
						window->font = console18pt;
					break;

					case SET_font8pt:
						window->font = console8pt;
					break;

					case SET_fontNumber:
				        window->font = number20pt;
					//printf("Массив %d \n", i);
					break;

					case 0x0D :
						window->cursor_x = 0;
						window->cursor_y = window->cursor_y +1 ;
					break;
				};
			};
		text++;
	};


};




//////////////////////////// decode UTF-8 ///////////////////////////

uint8_t* decode_utf8(uint8_t* buf,uint32_t* code_utf8){

    if (buf == NULL || code_utf8 == NULL || *buf == 0) return buf;
    uint32_t codepoint = 0;
    uint8_t   byte = *buf++,
    		  n_bytes = 0;
    *code_utf8 = 0;

    // n bytes ?

    if (!(byte&0x80)){codepoint = byte;
    }else if((byte>>5) == 0x06){codepoint = byte&0x1F;n_bytes = 1;
    }else if((byte>>4) == 0x0E){codepoint = byte&0x0F;n_bytes = 2;
    }else if((byte>>3) == 0x1E){codepoint = byte&0x07;n_bytes = 3;}

    // decode bytes
    for (uint8_t i = 0; i < n_bytes; i++){
        if ((*buf >> 6) != 0x02){codepoint = 0; break;};
        codepoint = (codepoint << 6) | ((*buf++) & 0x3F);
    };

    *code_utf8 = codepoint;
    return buf;
}

//////////////////////////// encode  UTF-8  ////////////////////////////

uint8_t* encode_utf8(uint8_t* buf,uint32_t* code_utf8) {

	if (buf == NULL || code_utf8 == NULL || *buf == 0) return buf;
	uint32_t codepoint = *code_utf8;
    if (codepoint <= 0x7F) {
        *buf++ = codepoint;
    } else if (codepoint <= 0x7FF) {
    	*buf++ = 0xC0 | (codepoint >> 6);
    	*buf++ = 0x80 | (codepoint & 0x3F);
    } else if (codepoint <= 0xFFFF) {
    	*buf++ = 0xE0 | (codepoint >> 12);
    	*buf++ = 0x80 | ((codepoint >> 6) & 0x3F);
    	*buf++ = 0x80 | (codepoint & 0x3F);

    } else if (codepoint <= 0x10FFFF) {
    	*buf++ = 0xF0 | (codepoint >> 18);
    	*buf++ = 0x80 | ((codepoint >> 12) & 0x3F);
    	*buf++ = 0x80 | ((codepoint >> 6) & 0x3F);
    	*buf++ = 0x80 | (codepoint & 0x3F);
    } else {
        return buf;
    }
};



const uint8_t init_ili9486[]={
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


const uint8_t init_r61581[]={
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
0xC0,5,0x12,0x3B,0x00,0x02,0x11,
0xC1,3,0x10,0x10,0x88,
0xC5,1,0x03,
0xC6,1,0x02,
0xC8,12,0x00,0x32,0x36,0x45,0x06,0x16,0x37,0x75,0x77,0x54,0x0c,0x00,
0xCC,1,0x00,
0x11,0,
0x36,1,0x40,
0x20,0,
0x13,0,
PAUSE_INIT,150,
0x29,0,
0x2A,4,0x00,0x00,0x01,0xdf,
0x2B,4,0x00,0x00,0x01,0x3f,
0x2C,0,
STOP_INIT,
};


 const uint8_t r61581_v2[]={
0xB0,01,0x1E,
0xB0,01,0x00,
0xB3,04,0x02,0x00,0x00,0x10,
0xB4,01,0x00,//0X10


// 		LCD_Write_COM(0xB9); //PWM Settings for Brightness Control
// 		LCD_Write_DATA(0x01);// Disabled by default.
// 		LCD_Write_DATA(0xFF); //0xFF = Max brightness
// 		LCD_Write_DATA(0xFF);
// 		LCD_Write_DATA(0x18);

0xC0,8,0x03,0x3B,00,00,00,0x01,0x00,0x43,
0xC1,4,0x08,0x15,0x08,0x08,//CLOCK
0xC4,4,0x15,0x03,0x03,0x01,
0xC6,1,0x02,
0xC8,10,0x0c,0x05,0x0A,0x6B,0x04,0x06,0x15,0x10,0x00,0x60,
0x36,1,0x40,
0x0C,1,0x55,
0x3A,1,0x55,
0x38,0,
0xD0,4,0x07,0x07,0x14,0xA2,
0xD1,3,0x03,0x5A,0x10,
0xD2,3,0x03,0x04,0x04,
0x11,0,
PAUSE_INIT,150,
0x2A,4,0x00,0x00,0x01,0xDF,
0x2B,4,0x00,0x00,0x01,0x3F,
PAUSE_INIT,100,
0x29,00,
PAUSE_INIT,30,
0x2C,00,
PAUSE_INIT,30,
STOP_INIT,00
};









#endif /* INC_TFT_PANEL_8BIT_H_ */
