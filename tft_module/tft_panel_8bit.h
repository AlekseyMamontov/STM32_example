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


#ifndef INC_TFT_PANEL_8BIT_H_
#define INC_TFT_PANEL_8BIT_H_

#include <stdio.h>
#include "tft_init_display.h"
#include "fonts.h"




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


#define MAX_KEYS 3
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

// --------------- Struct tft_display -----------//

struct tft_window {

	uint16_t image_x0;
	uint16_t image_y0;
	uint16_t image_x1;
	uint16_t image_y1;
	uint16_t cursor_x;
	uint16_t cursor_y;
	uint16_t color_font;
	uint16_t color_background;
	uint16_t color_a_background;
	const uint8_t *font;

};

// static - widgets

struct tft_widget {

	uint16_t 			status;
	struct tft_window*  window;
	uint8_t**			text_block;
	const uint8_t*   	code_block;
	void *				data;
	void				(*func)(void* data);

};



struct tft_screen{

	// Widgets

    struct
    tft_widget**   widgets;
    uint8_t    	   n_widgets;
    uint8_t*       dinamic_widgets;
    uint8_t    	   n_dinamic_widgets;

    struct tft_screen *next;
    struct tft_screen *prev;

};

struct TFT_panel {

	// Init

	uint8_t* init_tft;

	// Screens

	struct
	tft_screen* screens;

	// Keys

	uint16_t pin_input;
	uint16_t pin_reg_shift[MAX_KEYS];
	uint16_t wait_keys_action_ms;
	uint8_t  buffer_keys;
	uint8_t  n_buffer;



};




struct tft_stm32raw{

	uint32_t color_pixel;
	uint32_t background_pixel;
	uint32_t alfa_pixel;
	uint32_t border_pixel;

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


//------------------- Init tft display ----------------------//

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




//----------------  Write char to window Screen Panel ---------------//

void tft_write_char (struct tft_window *window,uint8_t symvol){

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
			  background_lsb |= window->color_background&0xFF;


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
//----------------  Write char to window Screen Panel ---------------//

void tft_write_char_alpha (struct tft_window *window,uint8_t symvol,uint8_t asymvol){


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
				  background_lsb |= window->color_background&0xFF;

		uint32_t  a_background_msb = (~window->color_a_background)&0xFF00;
							  background_msb <<= 8;
							  background_msb |= (window->color_a_background&0xFF00)>>8;

		uint32_t  a_background_lsb = (~window->color_a_background)&0xFF;
							  background_lsb <<= 16;
							  background_lsb |= window->color_a_background&0xFF;

		uint16_t  sizeXfont = *(window->font);
		uint16_t  colXbayt = sizeXfont/8;
		uint16_t  sizeYfont = window->font[1];

		const uint8_t*  mem_znak = window->font + 2;
		const uint8_t*  mem_aznak = mem_znak;

				  mem_znak +=(symvol*sizeYfont*colXbayt);
				  mem_aznak +=(asymvol*sizeYfont*colXbayt);

	    uint8_t   data, mask_data,adata;

			for( uint8_t n=0; n < sizeYfont; n++){

				for (uint8_t l=0; l < colXbayt; l++){

					data = *mem_znak;
					adata = *mem_aznak;
					mask_data = 0x80;

					for (uint8_t i=0; i<8 ; i++){

						if(adata&mask_data){

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
						}else{

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
				mem_aznak++;
				};
		  };
};

//---------------------- print char --------------------------//
// printf terminal kursor window

void tft_terminal_print(struct tft_window *window,uint8_t symvol){

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
	if(symvol >= 0x20){

		//временно
		if(window->font == number32pt){
			symvol = (symvol>0x2F && symvol<0x3A)? symvol-0x0f :  0x20 ;
		}

	symvol = symvol - 0x20;

  		tft_command(0x2A);
    	tft_data16(x0);
    	tft_data16(x1);
    	tft_command(0x2B);
    	tft_data16(y0);
    	tft_data16(y1);
    	tft_command(0x2C);

    	tft_write_char(window, symvol);

	};

	window->cursor_x = (window->cursor_x) + 1;

};

//------------------


void tft_print_utf8 (struct tft_window *window,uint8_t* buf){

uint16_t x0,x1,y0,y1;
uint8_t symvol = *buf;

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
	if(symvol >= 0x20){

	//symvol = symvol - 0x20;

  		tft_command(0x2A);
    	tft_data16(x0);
    	tft_data16(x1);
    	tft_command(0x2B);
    	tft_data16(y0);
    	tft_data16(y1);
    	tft_command(0x2C);


	tft_write_char(window, symvol);

	};

	window->cursor_x = (window->cursor_x) + 1;

};

///////////////////////////alpha//////////////////////////

void tft_terminal_aprint(struct tft_window *window,uint8_t symvol,uint8_t asymvol){

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
	if(symvol >= 0x20){

	symvol = symvol - 0x20;

  		tft_command(0x2A);
    	tft_data16(x0);
    	tft_data16(x1);
    	tft_command(0x2B);
    	tft_data16(y0);
    	tft_data16(y1);
    	tft_command(0x2C);

    	 tft_write_char_alpha(window, symvol,asymvol);

	};

	window->cursor_x = (window->cursor_x) + 1;

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

uint8_t* encode_utf8(uint8_t* buf,uint32_t* code_utf8){

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
    };

    	return buf;
}

//---------------------- 32 bit  to char  --------------------//

const uint32_t tft_data_to_char[11] ={
	0,
	1,
	10,
	100,
	1000,
	10000,
	100000,
	1000000,
	10000000,
	100000000,
	1000000000,
};

void tft_convert_data_to_char(uint8_t* buf, uint32_t data_to_char, uint8_t len){

 uint8_t i,numeral,symbol = 0x20;
 uint8_t correct = 11 - len;

 if(correct)
	 for(i=0; i<correct;i++) *(buf+i) = symbol;

 uint32_t del = tft_data_to_char[len];  ///del /=10;

	for(i=0;i<len;i++){

	   if(data_to_char >= del){
		   numeral = data_to_char/del;
		   data_to_char %=del;
		   symbol = 0x30;
	  }else numeral = 0;

   	  if ((del /= 10) < 1) symbol = 0x30;
   	  buf[i+correct] = symbol + numeral;

	};
};

void tft_convert_data_to_char2(char* buf,uint32_t data,uint8_t len){
    if(buf == NULL) return;
    snprintf(buf,len,"%lu",data);
};

//---------------------------- Widgets ----------------------//

#define FUNC 0x1b
#define FUNCs(b) FUNC,b
#define SET_background 0x01
#define SET_font_color 0x02

#define SET_cursorX 0x03
#define SET_cursorXY 0x04
#define SET_cursorY 0x05

#define SAVE_background 0x10
#define LOAD_background 0x11
#define SAVE_color_font 0x12
#define LOAD_color_font 0x13
#define CALL_Widget_block 0x14
#define SAVE_cursorX 0x15
#define LOAD_cursorX 0x16
#define SAVE_cursorY 0x17
#define LOAD_cursorY 0x18
#define SAVE_cursorXY 0x19
#define LOAD_cursorXY 0x1A
#define TXT_DATA 0x1B

#define fSET_background(a,b) FUNC,SET_background,a,b
#define fSET_font_color(a,b) FUNC,SET_font_color,a,b
#define fSET_cursorX(b)     FUNC,SET_cursorX,b
#define fSET_cursorXY(a,b) 	 FUNC,SET_cursorXY,a,b
#define fSAVE_background FUNC,SAVE_background
#define fLOAD_background FUNC,LOAD_background
#define fSAVE_color_font FUNC,SAVE_color_font
#define fLOAD_color_font FUNC,LOAD_color_font
#define fCALL_Widget_block(a) FUNC,CALL_Widget_block,a
#define fSAVE_cursorX FUNC,SAVE_cursorX
#define fLOAD_cursorX FUNC,LOAD_cursorX
#define fSAVE_cursorY FUNC,SAVE_cursorY
#define fLOAD_cursorY FUNC,LOAD_cursorY
#define fSAVE_cursorXY FUNC,SAVE_cursorXY
#define fLOAD_cursorXY FUNC,LOAD_cursorXY

#define SET_font8pt 0x06
#define SET_font18pt 0x07
#define SET_fontNumber 0x08
#define SET_fontNumber32 0x09
#define fSET_font8pt FUNC,SET_font8pt
#define fSET_font18pt FUNC,SET_font18pt
#define fSET_fontNumber FUNC,SET_fontNumber
#define fSET_fontNumber32 FUNC,SET_fontNumber32


#define SET_ON_ALPHA 0x0A
#define SET_OFF_ALPHA 0x0B
#define SET_SYMVOL 0x0C
#define SET_ENTER 0x0D

#define fSET_ON_ALPHA FUNC,SET_ON_ALPHA
#define fSET_OFF_ALPHA FUNC,SET_OFF_ALPHA
#define fSET_SYMVOL(a,b,c) FUNC,SET_SYMVOL,a,b,c
#define fSET_ENTER FUNC,SET_ENTER

#define fSET_TXT_DATA(а) FUNC,TXT_DATA,a


void tft_print_widget(struct TFT_panel* panel, uint8_t num){

	uint8_t alpha = 0;
	uint8_t symvol = 0;
	uint8_t cursorX = 0;
	uint8_t cursorY = 0;
	uint8_t *txt,*text,
			max_widgets = panel->screens->n_widgets;

	if(num > max_widgets && !max_widgets) return;

	struct tft_widget*  widget = *panel->screens->widgets + (num-1);
	struct tft_window*  window =  widget->window;
	text = widget->code_block;
	uint16_t background = window->color_background;
	uint16_t fontcolor =  window->color_font;

	while (*text){

	 if (*text >= 0x20){

		tft_terminal_print(window,*text);

		}else if(*text == FUNC){
			text++;
			switch (*text){

					case SET_cursorX: text++;window->cursor_x = *text;break;
					case SET_cursorXY: text++;window->cursor_x = *text;
					case SET_cursorY: text++; window->cursor_y = *text; break;

					case SET_font_color:text++; window->color_font = *text++;
										window->color_font = window->color_font << 8;
										window->color_font = window->color_font | (*text);
										break;

					case SET_background: text++; window->color_background = *text++;
										 window->color_background = window->color_background << 8;
										 window->color_background = window->color_background | (*text);
										 break;

					case SET_font18pt:     window->font = console18pt;break;
					case SET_font8pt:      window->font = console8pt;break;
					case SET_fontNumber:   window->font = number20pt; break;
					case SET_fontNumber32: window->font = number32pt; break;


					case SET_ON_ALPHA:  alpha = 1;break;
					case SET_OFF_ALPHA: alpha = 0;break;
					case SET_SYMVOL: fontcolor = window->color_font;
									 text++;
									 window->color_font = *text++;
									 window->color_font = window->color_font << 8;
									 window->color_font = window->color_font | (*text++);
									 if(*text)tft_terminal_print(window,*text);
									 window->color_font = fontcolor;
									 break;
					case SET_ENTER: window->cursor_x = 0;
									window->cursor_y = window->cursor_y +1 ;
									break;
					case SAVE_background: background = window->color_background;break;
					case LOAD_background: window->color_background = background ;break;
					case SAVE_color_font: fontcolor = window->color_font ;break;
					case LOAD_color_font: window->color_font = fontcolor ;break;
					case CALL_Widget_block: text++; tft_print_widget(panel,*text);break;
					case SAVE_cursorX: cursorX = window->cursor_x; break;
					case LOAD_cursorX: window->cursor_x = cursorX; break;
					case SAVE_cursorY: cursorY = window->cursor_y; break;
					case LOAD_cursorY: window->cursor_y = cursorY; break;
					case SAVE_cursorXY:cursorX = window->cursor_x; cursorY = window->cursor_y;break;
					case LOAD_cursorXY:window->cursor_x = cursorX; window->cursor_y = cursorY; break;
					case TXT_DATA: text++;
								   txt = *widget->text_block + *text;
								   while(*txt){tft_terminal_print(window,*txt++);}
								   break;


					case 0:break;



				};
			};
		text++;
	};
};

void tft_init_widgets(struct TFT_panel* panel){

	for(uint8_t i=0; i < panel->screens->n_widgets; i++){

		tft_print_widget(panel,i);

	};
};































//-----------------------------------------------------------------------//
/*
 *
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
 *
 *
 *
 * */
uint8_t mask_alfa[]={

		//28 @288 '(' (9 pixels wide)
			0b00000001, 0b10000000, //        ##
			0b00000111, 0b10000000, //      ####
			0b00011110, 0b00000000, //    ####
			0b00011000, 0b00000000, //    ##
			0b00110000, 0b00000000, //   ##
			0b01100000, 0b00000000, //  ##
			0b01100000, 0b00000000, //  ##
			0b11000000, 0b00000000, // ##
			0b11000000, 0b00000000, // ##
			0b11000000, 0b00000000, // ##
			0b11000000, 0b00000000, // ##
			0b11000000, 0b00000000, // ##
			0b11000000, 0b00000000, // ##
			0b11000000, 0b00000000, // ##
			0b11000000, 0b00000000, // ##
			0b01100000, 0b00000000, //  ##
			0b01100000, 0b00000000, //  ##
			0b00110000, 0b00000000, //   ##
			0b00011000, 0b00000000, //    ##
			0b00011110, 0b00000000, //    ####
			0b00000111, 0b10000000, //      ####
			0b00000001, 0b10000000, //        ##
			0b00000000, 0b00000000, //
			0b00000000, 0b00000000, //

			//29 @336 ')' (9 pixels wide)
			0b11000000, 0b00000000, // ##
			0b11110000, 0b00000000, // ####
			0b00111100, 0b00000000, //   ####
			0b00001100, 0b00000000, //     ##
			0b00000110, 0b00000000, //      ##
			0b00000011, 0b00000000, //       ##
			0b00000011, 0b00000000, //       ##
			0b00000001, 0b10000000, //        ##
			0b00000001, 0b10000000, //        ##
			0b00000001, 0b10000000, //        ##
			0b00000001, 0b10000000, //        ##
			0b00000001, 0b10000000, //        ##
			0b00000001, 0b10000000, //        ##
			0b00000001, 0b10000000, //        ##
			0b00000001, 0b10000000, //        ##
			0b00000011, 0b00000000, //       ##
			0b00000011, 0b00000000, //       ##
			0b00000110, 0b00000000, //      ##
			0b00001100, 0b00000000, //     ##
			0b00111100, 0b00000000, //   ####
			0b11110000, 0b00000000, // ####
			0b11000000, 0b00000000, // ##
			0b00000000, 0b00000000, //
			0b00000000, 0b00000000, //

};


void window_oval(struct tft_window* win,uint8_t bits){

	uint16_t nX = win->image_x1 > win->image_x0? win->image_x1 - win->image_x0 :
												 win->image_x0 - win->image_x1 ;
	uint16_t nY = win->image_y1 > win->image_y0? win->image_y1 - win->image_y0 :
												 win->image_y0 - win->image_y1 ;
	uint32_t size = (nX+1)*(nY+1);
	tft_set_window(win);







};


































//------------------------



#endif /* INC_TFT_PANEL_8BIT_H_ */
