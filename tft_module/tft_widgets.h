/*
 * tft_widgets.h
 *
 *  Created on: Jan 11, 2024
 *      Author: Oleksii
 *
 *
    struct tft_widget {

	uint16_t 			status;
	struct tft_window*  window;
	uint8_t**			text_block;
	uint8_t*   			code_block;
	void *				data;
	void				(*func)(void* data);

};
 *
 *
 */

#ifndef INC_TFT_WIDGETS_H_
#define INC_TFT_WIDGETS_H_

struct tft_window Panel_win={

	.image_x0 = 0,
	.image_y0 = 0,
	.image_x1 = 0x13F,
	.image_y1 = 0x1dF,
	.cursor_x = 0,
	.cursor_y = 0,
	.color_font = color_WHITE,
	.color_background = color_BLACK,
	.font = console18pt,

};

uint8_t text[]={0};
struct TFT_panel TFT_CAN_module;
const struct tft_screen Screen1;
const struct tft_screen Screen2;
const struct tft_screen Screen3;

//////////////////////////////////// SCREEN 1 ////////////////////////////////////

struct tft_window Screen1_win={

	.image_x0 = 0,
	.image_y0 = 0,
	.image_x1 = 0x13F,
	.image_y1 = 0x1dF,
	.cursor_x = 0,
	.cursor_y = 0,
	.color_font = color_WHITE,
	.color_background = color_BLACK,
	.font = console18pt,

};

/* Widget Matrix */
const static
uint8_t Matrix_code_block[]={
		fSET_font18pt,
		//0
		fSET_cursorXY(8,0),fSET_SYMVOL(0xF8,0x00,0xA3),
		fSAVE_background,fSET_background(0xF8,0x00),0x20,0x20,0xCC,0xE0,0xF2,0xF0,0xE8,0xF6,0xE0,0x20,fLOAD_background,
		fSET_SYMVOL(0xF8,0x00,0xA4),
		// 1-3
		fSET_cursorXY(8,1),fSET_SYMVOL(0xF8,0x00,0xA6),fSET_cursorX(19),fSET_SYMVOL(0xF8,0x00,0xA5),
		fSET_cursorXY(8,2),fSET_SYMVOL(0xF8,0x00,0xA6),fSET_cursorX(19),fSET_SYMVOL(0xF8,0x00,0xA5),
		fSET_cursorXY(8,3),fSET_SYMVOL(0xF8,0x00,0xA6),fSET_cursorX(19),fSET_SYMVOL(0xF8,0x00,0xA5),
		// 4
		fSET_cursorXY(8,4),fSET_SYMVOL(0xF8,0x00,0xA6),' ','o','n',':',fCALL_Widget_block(1),fSET_cursorX(17),0xad,0xd1,fSET_SYMVOL(0xF8,0x00,0xA5),
        // 5
		fSET_cursorXY(8,5),fSET_SYMVOL(0xF8,0x00,0xA6),'o','f','f',':',fCALL_Widget_block(2),fSET_cursorX(17),0xad,0xd1,fSET_SYMVOL(0xF8,0x00,0xA5),
		// 6
		fSET_cursorXY(8,6),fSAVE_color_font,fSET_font_color(0xF8,0x00),0x9E,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9d,fLOAD_color_font,
		0,0,0
};
#define MATRIX_BUILD 0
struct tft_widget
w_matrix={
	.status = 0x01,
	.window = &Screen1_win,
	.text_block = NULL,
	.code_block = Matrix_code_block,
	.data = NULL,
	.func = NULL,
};
#define MATRIX_ON 1
uint8_t matrix_on[]={fSET_cursorXY(14,4),0x32,0x30,0x30,0,0};
struct tft_widget
w_matrix_on={
	.status = 0x01,
	.window = &Screen1_win,
	.text_block = NULL,
	.code_block = matrix_on,
	.data = NULL,
	.func = NULL,
};
#define MATRIX_OFF 2
uint8_t matrix_off[]={fSET_cursorXY(14,5),0x32,0x30,0x35,0,0};
struct tft_widget
w_matrix_off={
	.status = 0x01,
	.window = &Screen1_win,
	.text_block = NULL,
	.code_block = matrix_off,
	.data = NULL,
	.func = NULL,
};
struct tft_window
win_matrix_temp={
	.image_x0 = 180,
	.image_y0 = 35,
	.image_x1 = 0x13F,
	.image_y1 = 0x1dF,
	.cursor_x = 0,
	.cursor_y = 0,
	.color_font = color_WHITE,
	.color_background = color_BLACK,
	.font = number32pt,
};
#define MATRIX_TEMPERATURE 3
uint8_t matrix_temp[]={fSET_cursorXY(0,0),0x20,0x31,0x30,0,0};
uint8_t matrix_temp_old[]={0x20,0x31,0x30,0,0};
struct
w_data_to_char matrix_data={
	.data = 0,
	.len = 3,
	.new_char = (matrix_temp+4),
	.old_char = matrix_temp_old,
	.panel = &TFT_CAN_module,
	.num_widget = MATRIX_TEMPERATURE,
};
struct tft_widget
w_matrix_temp={
	.status = 0x01,
	.window = &win_matrix_temp,
	.text_block = NULL,
	.code_block = matrix_temp,
	.data = (void*)&matrix_data,
	.func = widget_print_data,
};



/* Widget Punch */

const static uint8_t
Punch_code_block[]={
		//0
		fSET_cursorXY(8,7),fSET_SYMVOL(0x04,0x00,0xA3),
		fSAVE_background,fSET_background(0x04,0x00),0x20,0x20,0xCF,0xF3,0xE0,0xED,0xF1,0xEE,0xED,0x20,fLOAD_background,
		fSET_SYMVOL(0x04,0x00,0xA4),
		// 1-3
		fSET_cursorXY(8,8),fSET_SYMVOL(0x04,0x00,0xA6),fSET_cursorX(19),fSET_SYMVOL(0x04,0x00,0xA5),
		fSET_cursorXY(8,9),fSET_SYMVOL(0x04,0x00,0xA6),fSET_cursorX(19),fSET_SYMVOL(0x04,0x00,0xA5),
		fSET_cursorXY(8,10),fSET_SYMVOL(0x04,0x00,0xA6),fSET_cursorX(19),fSET_SYMVOL(0x04,0x00,0xA5),
		// 4
		fSET_cursorXY(8,11),fSET_SYMVOL(0x04,0x00,0xA6),' ','o','n',':',fCALL_Widget_block(5),fSET_cursorX(17),0xad,0xd1,fSET_SYMVOL(0x04,0x00,0xA5),
        // 5
		fSET_cursorXY(8,12),fSET_SYMVOL(0x04,0x00,0xA6),'o','f','f',':',fCALL_Widget_block(6),fSET_cursorX(17),0xad,0xd1,fSET_SYMVOL(0x04,0x00,0xA5),
		// 6
		fSET_cursorXY(8,13),fSAVE_color_font,fSET_font_color(0x04,0x00),0x9E,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9d,fLOAD_color_font,
		0,0,0
};
#define PUNCH_BUILD 4
struct tft_widget
w_punch={
	.status = 0x01,
	.window = &Screen1_win,
	.text_block = NULL,
	.code_block = Punch_code_block,
	.data = NULL,
	.func = NULL,
};
#define PUNCH_ON 5
uint8_t punch_on[]={fSET_cursorXY(14,11),0x32,0x30,0x33,0,0};
struct tft_widget
w_punch_on={
	.status = 0x01,
	.window = &Screen1_win,
	.text_block = NULL,
	.code_block = punch_on,
	.data = NULL,
	.func = NULL,
};
#define PUNCH_OFF 6
uint8_t punch_off[]={fSET_cursorXY(14,12),0x32,0x30,0x38,0,0};
struct tft_widget
w_punch_off={
	.status = 0x01,
	.window = &Screen1_win,
	.text_block = NULL,
	.code_block = punch_off,
	.data = NULL,
	.func = NULL,
};
struct tft_window
Win_punch_temp={
	.image_x0 = 180,
	.image_y0 = 205,
	.image_x1 = 0x13F,
	.image_y1 = 0x1dF,
	.cursor_x = 0,
	.cursor_y = 0,
	.color_font = color_WHITE,
	.color_background = color_BLACK,
	.font = number32pt,
};
#define PUNCH_TEMPERATURE 7
uint8_t punch_temp[]={fSET_cursorXY(0,0),0x20,0x31,0x30,0,0};
uint8_t punch_temp_old[]={0x20,0x31,0x30,0,0};
struct w_data_to_char
punch_data={
	.data = 0,
	.len = 3,
	.new_char = (punch_temp+4),
	.old_char = punch_temp_old,
	.panel = &TFT_CAN_module,
	.num_widget = PUNCH_TEMPERATURE,
};
struct tft_widget
w_punch_temp={
	.status = 0x01,
	.window = &Win_punch_temp,
	.text_block = NULL,
	.code_block = punch_temp,
	.data = (void*)&punch_data,
	.func = widget_print_data,
};


/*--  Widget COUNTER --*/
const static uint8_t
Pr_Counter_code_block[]={
		//1
		fSET_cursorXY(0,14),fSET_SYMVOL(0x00,0x1F,0xA3),
		fSAVE_background,fSET_background(0x00,0x1F),0x20,0x20,0x20,0x20,0x20,0xC8,0xe7,0xe4,0xe5,0xeb,0xe8,0xe9,0x20,0x20,0x20,0x20,0x20,0x20,fLOAD_background,
		fSET_SYMVOL(0x00,0x1f,0xA4),
		//2
		fSET_cursorXY(0,15),fSET_SYMVOL(0x00,0x1f,0xA6),fSET_cursorX(19),fSET_SYMVOL(0x00,0x1f,0xA5),
		//2
		fSET_cursorXY(0,16),fSET_SYMVOL(0x00,0x1f,0xA6),fSET_cursorX(19),fSET_SYMVOL(0x00,0x1f,0xA5),
		fSET_cursorXY(0,17),fSAVE_color_font,fSET_font_color(0x00,0x1f),
		0x9E,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9d,fLOAD_color_font,
		0,0,0
};
#define COUNTER_BUILD 8
struct tft_widget
w_product_counter={
	.status = 0x01,
	.window = &Screen1_win,
	.text_block = NULL,
	.code_block = Pr_Counter_code_block,
	.data = NULL,
	.func = NULL,
};
struct tft_window
Win_product_counter={
	.image_x0 = 8,
	.image_y0 = 366,
	.image_x1 = 0x13F,
	.image_y1 = 0x1dF,
	.cursor_x = 0,
	.cursor_y = 0,
	.color_font = color_WHITE,
	.color_background = color_BLACK,
	.font = number32pt,
};
#define COUNTER_DATA 9
uint8_t product_counter[]={fSET_cursorXY(0,0),0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x30,0,0};
uint8_t product_counter_old[]={0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x30,0,0};
struct w_data_to_char
counter_data={
	.data = 0,
	.len = 9,
	.new_char = (product_counter+4),
	.old_char = product_counter_old,
	.panel = &TFT_CAN_module,
	.num_widget = COUNTER_DATA,
};
struct tft_widget
w_counter_data ={
	.status = 0x01,
	.window = &Win_product_counter,
	.text_block = NULL,
	.code_block = product_counter,
	.data = (void*)&counter_data,
	.func = widget_print_data,
};


/*--------------- Widget Info---------------------*/


const static uint8_t info_code_block[]={

		//0
		fSET_cursorXY(1,0),fSET_SYMVOL(0xEB,0xC0,0xA3),
		fSAVE_background,fSET_background(0xEB,0xC0),'i','n','f','o',fLOAD_background,
		fSET_SYMVOL(0xEB,0xC0,0xA4),
		// 1-3
		fSET_cursorXY(1,1),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(6),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(1,2),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(6),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(1,3),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(6),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(1,4),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(6),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(1,5),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(6),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(1,6),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(6),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(1,7),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(6),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(1,8),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(6),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(1,9),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(6),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(1,10),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(6),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(1,11),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(6),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(1,12),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(6),fSET_SYMVOL(0xEB,0xC0,0xA5),
		// 4
		fSET_cursorXY(1,13),fSAVE_color_font,fSET_font_color(0xEB,0xC0),0x9E,0x9C,0x9C,0x9C,0x9C,0x9d,fLOAD_color_font,
		0,0,0

};
#define INFO_BUILD 10
struct tft_widget
w_info_block={
	.status = 0x01,
	.window = &Screen1_win,
	.text_block = NULL,
	.code_block = info_code_block,
	.data = NULL,
	.func = NULL,
};


/*------------------------------------ Widget MENU ----------------------------------------------*/


const static uint8_t
screen1_menu_code[]={
		//1
		fSET_cursorXY(0,19),fSAVE_background,fSET_background(0x01,0x28),
		0x20,0xA9,'m','e','n','u',0x20,'o','k',0x20,0x98,'u','p',0x20,0x20,'d','o','w','n',0x20,fLOAD_background,
		0,0,0
};
#define MENU_BUILD 11
struct tft_widget
w_widget_menu0={
	.status = 0x01,
	.window = &Screen1_win,
	.text_block = NULL,
	.code_block = screen1_menu_code,
	.data = NULL,
	.func = NULL,
};


const static
struct tft_widget* Screen_1_widgets[]={

		&w_matrix,			// MATRIX_BUILD 0
		&w_matrix_on,   	// MATRIX_ON 1
		&w_matrix_off,  	// MATRIX_OFF 2
		&w_matrix_temp, 	// MATRIX_TEMPERATURE 3

		&w_punch,       	// PUNCH_BUILD 4
		&w_punch_on,		// PUNCH_ON 5
		&w_punch_off,		// PUNCH_OFF 6
		&w_punch_temp,		// PUNCH_TEMPERATURE 7

		&w_product_counter, // COUNTER_BUILD 8
		&w_counter_data,    // COUNTER_DATA 9

		&w_info_block,      // INFO_BUILD 10
		&w_widget_menu0,	// MENU_BUILD 11
};
uint8_t screen1_build[]={0,3,4,7,8,9,10,11};
uint8_t screen1_dynamic[]={1,2,3,5,6,7,9};

const struct tft_screen
Screen1={
	    .widgets = Screen_1_widgets,
		.n_widgets = sizeof(Screen_1_widgets),
	    .build_widgets = screen1_build,			 // number
	    .n_build_widgets = sizeof(screen1_build),
	    .dynamic_widgets = screen1_dynamic, 		 // number widgets
	    .n_dynamic_widgets =sizeof(screen1_dynamic),

	    .next = &Screen2,
	    .prev = NULL,
};

















////////////////////////////////////////// SCREEN 2 /////////////////////////////////////////////






////////////////////////////////////////// SCREEN 3 /////////////////////////////////////////////


/* old
 *
 *
 *
uint8_t Widget_info2[]={

		//0
		fSET_cursorXY(0,0),fSET_SYMVOL(0xEB,0xC0,0xA3),
		fSAVE_background,fSET_background(0xEB,0xC0),' ','i','n','f','o',' ',fLOAD_background,
		fSET_SYMVOL(0xEB,0xC0,0xA4),
		// 1-3
		fSET_cursorXY(0,1),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(7),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(0,2),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(7),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(0,3),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(7),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(0,4),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(7),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(0,5),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(7),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(0,6),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(7),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(0,7),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(7),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(0,8),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(7),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(0,9),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(7),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(0,10),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(7),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(0,11),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(7),fSET_SYMVOL(0xEB,0xC0,0xA5),
		fSET_cursorXY(0,12),fSET_SYMVOL(0xEB,0xC0,0xA6),fSET_cursorX(7),fSET_SYMVOL(0xEB,0xC0,0xA5),
		// 4
		fSET_cursorXY(0,13),fSAVE_color_font,fSET_font_color(0xEB,0xC0),0x9E,0x9C,0x9C,0x9C,0x9C,0x9C,0x9C,0x9d,fLOAD_color_font,
		0,0,0

};
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
#define CALL_Data_block 0x14
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
#define fCALL_Data_block(a) FUNC,CALL_Data_block,a
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




void tft_print_widgets(struct tft_window *window, const uint8_t* text){

	uint8_t alpha = 0;
	uint8_t symvol = 0;
	uint8_t cursorX = 0;
	uint8_t cursorY = 0;
	uint8_t *txt;
	uint16_t background = window->color_background;
	uint16_t fontcolor =  window->color_font;

	while (*text){

	 if (*text >= 0x20){


		tft_terminal_print(window,*text);

		}else if(*text == FUNC){
			text++;
			switch (*text){

					case SET_cursorX: //set cursor X
						text++;
						window->cursor_x = *text;
						break;

					case SET_cursorXY:
						text++;
						window->cursor_x = *text;

					case SET_cursorY:
						text++;
						window->cursor_y = *text;
						break;

					case SET_font_color:
						text++;
						window->color_font = *text++;
						window->color_font = window->color_font << 8;
						//text++;
						window->color_font = window->color_font | (*text);
						break;

					case SET_background:
						text++;
						window->color_background = *text++;
						window->color_background = window->color_background << 8;
						//text++;
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
					break;
					case SET_fontNumber32:
						window->font = number32pt;
					break;


					case SET_ON_ALPHA:  alpha = 1;break;
					case SET_OFF_ALPHA: alpha = 0;break;
					case SET_SYMVOL:

						fontcolor = window->color_font;
						text++;
						window->color_font = *text++;
						window->color_font = window->color_font << 8;
					    window->color_font = window->color_font | (*text++);
					    if(*text)tft_terminal_print(window,*text);
					    window->color_font = fontcolor;

					break;
					case SET_ENTER:
						window->cursor_x = 0;
						window->cursor_y = window->cursor_y +1 ;
						break;

					case SAVE_background:
						 background = window->color_background;
				    break;
					case LOAD_background:
						 window->color_background = background;
				    break;
					case SAVE_color_font:
						fontcolor = window->color_font;
					break;
					case LOAD_color_font:
						window->color_font = fontcolor ;
					break;
					case CALL_Widget_block:
						text++;
						if(*text < MAX_Wdata){

							txt = block_Wdata[*text];
							tft_print_widgets(window,txt);

						};
					break;

					case SAVE_cursorX:
						 cursorX = window->cursor_x;
						break;
					case LOAD_cursorX:
						window->cursor_x = cursorX;
						break;
					case SAVE_cursorY:
						cursorY = window->cursor_y;
						break;
					case LOAD_cursorY:
						window->cursor_y = cursorY;
						break;
					case SAVE_cursorXY:
						cursorX = window->cursor_x;
						cursorY = window->cursor_y;
						break;
					case LOAD_cursorXY:
						window->cursor_x = cursorX;
						window->cursor_y = cursorY;
						break;

					case 0:



				};
			};
		text++;
	};
};

*/



#endif /* INC_TFT_WIDGETS_H_ */
