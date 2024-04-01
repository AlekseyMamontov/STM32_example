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

/*------------------------------- Widget Matrix -------------------------------------*/
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
uint8_t matrix_on[]={fSET_cursorXY(13,4),0x20,0x32,0x30,0x30,0,0};
struct tft_widget
w_matrix_on={
	.status = 0x00,
	.window = &Screen1_win,
	.text_block = NULL,
	.code_block = matrix_on,
	.data = NULL,
	.func = NULL,
};
#define MATRIX_OFF 2
uint8_t matrix_off[]={fSET_cursorXY(13,5),0x20,0x32,0x30,0x33,0,0};
struct tft_widget
w_matrix_off={
	.status = 0x00,
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



/*--------------------------------- Widget Punch ------------------------------------------------*/

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
uint8_t punch_on[]={fSET_cursorXY(13,11),0x20,0x32,0x30,30,0,0};
struct tft_widget
w_punch_on={
	.status = 0x001,
	.window = &Screen1_win,
	.text_block = NULL,
	.code_block = punch_on,
	.data = NULL,
	.func = NULL,
};
#define PUNCH_OFF 6
uint8_t punch_off[]={fSET_cursorXY(13,12),0x20,0x32,0x30,0x034,0,0};
struct tft_widget
w_punch_off={
	.status = 0x00,
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
	.status = 0,
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


/*-------------------------------------  Widget COUNTER ------------------------------------------------*/

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


/*--------------------------------- Widget Info ------------------------------------------------*/


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

//---------------------------------- Cylinder_block --------------------------------------//

struct tft_sprite cylinder_off={
	.x = 48,
	.y = 40,
	.width = 32,
	.height= 62,
	.pixeldata = cyl_off,
};
struct tft_sprite cylinder_on={
	.x = 48,
	.y = 40,
	.width = 32,
	.height= 62,
	.pixeldata = cyl_on,
};
const struct tft_sprite* cylindr_image[2]={
	&cylinder_off,
	&cylinder_on,
};

#define CYLINDR_BUILD 12
static uint8_t screen1_cylindr_code[]={fIMAGE_RGB565(0),0,0};

struct animation_image
w_cylindr_animation={
	.status = 0x01,
	.current_sprite = screen1_cylindr_code + 2,
	.new_data = 0,
	.old_data = 1,
	.panel = &TFT_CAN_module,
	.num_widget = CYLINDR_BUILD,
};
struct tft_widget
w_cylindr ={

	.status = 0x01,
	.window = &Screen1_win,
	.image_block = cylindr_image,
	.n_images = sizeof(cylindr_image),
	.text_block = NULL,
	.code_block = screen1_cylindr_code,
	.data = (void*) &w_cylindr_animation,
	.func = widget_sprite_on_off,

};

//------------------------------------ 220V_block -------------------------------//

#define V230_BUILD 13

struct tft_window V_230_win={

	.image_x0 = 48,
	.image_y0 = 120,
	.image_x1 = 0x13F,
	.image_y1 = 0x1dF,
	.cursor_x = 0,
	.cursor_y = 0,
	.color_font = color_GRAY,
	.color_background = color_BLACK,
	.font = number32pt,

};
static uint8_t
screen1_V230v2_code[]={fSET_cursorXY(0,0),0x3A,0,0};
struct tft_widget
W_V230v2={
	.status = 0x01,
	.window = &V_230_win,
	.text_block = NULL,
	.code_block = screen1_V230v2_code,
	.data = NULL,
	.func = NULL,

};

//------------------------------------ Button  -------------------------------//

#define BUTTON 14

struct tft_window Knopka_win={

	.image_x0 = 48,
	.image_y0 = 210,
	.image_x1 = 0x13F,
	.image_y1 = 0x1dF,
	.cursor_x = 0,
	.cursor_y = 0,
	.color_font = color_GRAY,
	.color_background = color_BLACK,
	.font = number32pt,

};
static uint8_t
screen1_Knopka_code[]={fSET_cursorXY(0,0),0x3B,0,0};
struct tft_widget
W_button={
	.status = 0x01,
	.window = &Knopka_win,
	.text_block = NULL,
	.code_block = screen1_Knopka_code,
	.data = NULL,
	.func = NULL,
};

//------------------------------------ Sensors  -------------------------------//

#define SENSOR 15

struct tft_window Sensor_win={

	.image_x0 = 48,
	.image_y0 = 205,
	.image_x1 = 0x13F,
	.image_y1 = 0x1dF,
	.cursor_x = 0,
	.cursor_y = 0,
	.color_font = color_GRAY,
	.color_background = color_BLACK,
	.font = number32pt,

};
static uint8_t
screen1_Sensor_code[]={fSET_cursorXY(0,0),0x3D,0,0};
struct tft_widget
W_Sensor={
	.status = 0x01,
	.window = &Knopka_win,
	.text_block = NULL,
	.code_block = screen1_Knopka_code,
	.data = NULL,
	.func = NULL,
};



//-------------------------- Block_Object ----------------------------//

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
		&w_cylindr,  		// CYLINDR_BUILD 12
		&W_V230v2,			// 13 220V
		&W_button,			// BUTTON 14
		&W_Sensor			// SENSOR 15
};

uint8_t screen1_build[]={0,3,4,7,8,9,10,11,12,13,14,15};
uint8_t screen1_dynamic[]={3,7,9,12,1,2,5,6};

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




/*

const
struct tft_sprite V230off={
	.x = 48,
	.y = 120,
	.width = 32,
	.height= 47,
	.pixeldata = V230_off,
};
struct tft_sprite V230on={
	.x = 48,
	.y = 120,
	.width = 32,
	.height= 47,
	.pixeldata = V230_on,
};
const struct tft_sprite* V230_image[2]={
	&V230off,
	&V230on,
};

#define V230_BUILD 13
static uint8_t
screen1_V230_code[]={fIMAGE_RGB565(0),0,0};

struct animation_image
w_V230_animation={
	.status = 0x01,
	.current_sprite = screen1_V230_code + 2,
	.new_data = 0,
	.old_data = 1,
	.panel = &TFT_CAN_module,
	.num_widget = V230_BUILD,
};
struct tft_widget
w_V230 ={

	.status = 0x01,
	.window = &Screen1_win,
	.image_block = V230_image,
	.n_images = sizeof(V230_image),
	.text_block = NULL,
	.code_block = screen1_V230_code,
	.data = (void*) &w_V230_animation,
	.func = widget_sprite_on_off,

};
 */

#endif /* INC_TFT_WIDGETS_H_ */
