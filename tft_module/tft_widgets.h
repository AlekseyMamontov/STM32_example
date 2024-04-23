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
const struct tft_screen Screen_gm;
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
		fSAVE_background,fSET_background(0xF8,0x00),0x20,0x20,М,а,т,р,и,ц,я,0x20,fLOAD_background,
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
		fSAVE_background,fSET_background(0x04,0x00),0x20,0x20,П,у,а,н,с,о,н,0x20,fLOAD_background,
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
		fSAVE_background,fSET_background(0x00,0x1F),0x20,0x20,0x20,0x20,0x20,В,и,р,о,б,'i',в,0x20,0x20,0x20,0x20,0x20,0x20,fLOAD_background,
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
		fSAVE_background,fSET_background(0xEB,0xC0),'I',н,ф,о,fLOAD_background,
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
		0x20,0xA9,М,е,н,ю,0x20,'o','k',0x20,0x98,'u','p',0x20,0x20,'d','o','w','n',0x20,fLOAD_background,
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
#define STATUS_220V 13

struct tft_window win_V220={

	.image_x0 = 48,
	.image_y0 = 190,
	.image_x1 = 0x13F,
	.image_y1 = 0x1dF,
	.cursor_x = 0,
	.cursor_y = 0,
	.color_font = color_GRAY,
	.color_background = color_BLACK,
	.font = number32pt,
};
static uint8_t
screen1_220v_off[]={fSET_cursorXY(0,0),fSET_font_color(0x84,0x10),0x3A,0,0},
screen1_220V_on[]= {fSET_cursorXY(0,0),fSET_font_color(0xFF,0xE0),0x3A,0,0};
uint8_t* screen1_220V_animation[]={screen1_220v_off,screen1_220V_on};

struct change_txt txt_220V ={
		.panel = &TFT_CAN_module,
		.data = 0,
		.txt_block = screen1_220V_animation,
		.n_txt_block = sizeof(screen1_220V_animation),
		.num_widget = STATUS_220V,
};

struct tft_widget
w_V220={
	.status = 0x01,
	.window = &win_V220,
	.text_block = NULL,
	.code_block = screen1_220v_off,
	.data = (void*) &txt_220V,
	.func = widget_txt_change,
};

//------------------------------------ Button  -------------------------------//

#define BUTTON 14

struct tft_window Button_win={

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
screen1_button_off[]={fSET_cursorXY(0,0),fSET_font_color(0x84,0x10),0x3B,0,0},
screen1_button_on[]= {fSET_cursorXY(0,0),fSET_font_color(0xFF,0xE0),0x3C,0,0};
uint8_t* screen1_button_animation[]={screen1_button_off,screen1_button_on};

struct change_txt txt_button ={
		.panel = &TFT_CAN_module,
		.data = 0,
		.txt_block = screen1_button_animation,
		.n_txt_block = sizeof(screen1_button_animation),
		.num_widget = BUTTON,
};

struct tft_widget
w_button={
	.status = 0x01,
	.window = &Button_win,
	.text_block = NULL,
	.code_block = screen1_button_off,
	.data = (void*) &txt_button,
	.func = widget_txt_change,
};

//------------------------------------ Sensors  -------------------------------//

#define ERROR_TFT 15

struct tft_window Sensor_win={

	.image_x0 = 48,
	.image_y0 = 250,
	.image_x1 = 0x13F,
	.image_y1 = 0x1dF,
	.cursor_x = 0,
	.cursor_y = 0,
	.color_font = color_GRAY,
	.color_background = color_BLACK,
	.font = number32pt,

};
static uint8_t
screen1_ERR_off[]={fSET_cursorXY(0,0),fSET_font_color(0x84,0x10),0x3F,0,0},
screen1_ERR_on[]={fSET_cursorXY(0,0),fSET_font_color(0xFF,0xE0),0x3F,0,0};

struct simple_txt n1_sensor={

		.panel = &TFT_CAN_module,
		.num_widget = ERROR_TFT,

};

struct tft_widget
W_ErrorTFT={
	.status = 0x01,
	.window = &Sensor_win,
	.text_block = NULL,
	.code_block = screen1_ERR_off,
	.data = (void*)&n1_sensor,
	.func = widget_txt_simple,
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
		&w_V220,			// 13 220V
		&w_button,			// BUTTON 14
		&W_ErrorTFT			// SENSOR 15
};

uint8_t screen1_build[]={0,3,4,7,8,9,10,11,12,13,14,15};
uint8_t screen1_dynamic[]={3,7,9,12,1,2,5,6,14};

void Screen1_keys(struct TFT_panel* tft);

const struct tft_screen
Screen1={

	    .widgets = Screen_1_widgets,
		.n_widgets = sizeof(Screen_1_widgets)/sizeof(Screen_1_widgets[0]),
	    .build_widgets = screen1_build,			 // number
	    .n_build_widgets = sizeof(screen1_build),
	    .dynamic_widgets = screen1_dynamic, 		 // number widgets
	    .n_dynamic_widgets =sizeof(screen1_dynamic),
		.func_keys = Screen1_keys,
	    .next = &Screen_gm,
	    .prev = NULL,

};




////////////////////////////////////////// SCREEN 2 /////////////////////////////////////////////

struct tft_window Screen_gwin={

	.image_x0 = 0,
	.image_y0 = 0,
	.image_x1 = 0x13F,
	.image_y1 = 0x1dF,
	.cursor_x = 0,
	.cursor_y = 0,
	.color_font = color_WHITE,
	.color_background = 0x0003,
	.font = console18pt,

};
struct tft_window Screen_gwin_activ={

	.image_x0 = 0,
	.image_y0 = 0,
	.image_x1 = 0x13F,
	.image_y1 = 0x1dF,
	.cursor_x = 0,
	.cursor_y = 0,
	.color_font = color_WHITE,
	.color_background = color_BLUE,
	.font = console18pt,

};
/*-------------------------- Widget General MENU ----------------------------------*/

const static

uint8_t MENU_general[]={

		fSET_font18pt,fSAVE_background,fSET_background(0x04,0x00),
		fSET_cursorXY(0,0) ,' ',П,р,о,г,р,а,м,а,' ',в,е,р,с,т,а,т,а,' ',' ',
		fSET_cursorXY(0,10),' ',Н,а,л,а,ш,т,у,в,а,н,н,я,' ',' ',' ',' ',' ',' ',' ',
		fLOAD_background,
		0,0,0
};

struct Menu_block Menu_gcursor={

		.panel =     &TFT_CAN_module,
		.off_block = &Screen_gwin,
		.on_block =  &Screen_gwin_activ,
		.current_widget = 1,
		.new_widget = 1,

};



#define INFO_MENU 0
struct tft_widget
w_gmenu={
	.status = 0x01,
	.window = &Screen_gwin,
	.text_block = NULL,
	.code_block = MENU_general,
	.data = &Menu_gcursor,
	.func = widget_menu_type1,
};






// Activated program menu

#define MENU_D200 1
const static uint8_t MENU_strD200[]={fSET_cursorXY(0,2),' ',Т,а,р,'i',л,к,а,' ','D','2','0','0',' ',' ',' ',' ',' ',' ',' ',0,0};// Plate D200
struct tft_widget w_strD200={
	.status = 0x01,
	.window = &Screen_gwin,
	.text_block = NULL,
	.code_block = MENU_strD200,
	.data = NULL,
	.func = widget_txt_simple,
};

#define MENU_D275 2
const static uint8_t MENU_strD275[]={fSET_cursorXY(0,3),' ',Т,а,р,'i',л,к,а,' ','D','2','7','5',' ',' ',' ',' ',' ',' ',' ',0,0};// Plate D200
struct tft_widget w_strD275={
	.status = 0x01,
	.window = &Screen_gwin,
	.text_block = NULL,
	.code_block = MENU_strD275,
	.data = NULL,
	.func = widget_txt_simple,
};
#define MENU_D300 3
const static uint8_t MENU_strD300[]={fSET_cursorXY(0,4),' ',Т,а,р,'i',л,к,а,' ','D','3','0','0',' ',' ',' ',' ',' ',' ',' ',0,0};// Plate D200
struct tft_widget w_strD300={
	.status = 0x01,
	.window = &Screen_gwin,
	.text_block = NULL,
	.code_block = MENU_strD300,
	.data = NULL,
	.func = widget_txt_simple,
};

#define MENU_D370 4
const static uint8_t MENU_strD370[]={fSET_cursorXY(0,5),' ',Т,а,р,'i',л,к,а,' ','D','3','7','0',' ',' ',' ',' ',' ',' ',' ',0,0};// Plate D200
struct tft_widget w_strD370={
	.status = 0x01,
	.window = &Screen_gwin,
	.text_block = NULL,
	.code_block = MENU_strD370,
	.data = NULL,
	.func = widget_txt_simple,
};

#define MENU_R220 5
const static uint8_t MENU_strR220[]={fSET_cursorXY(0,6),' ',Т,а,р,'i',л,к,а,' ','R','2','2','0','|','R','1','4','4',' ',' ',0,0};// Plate D200
struct tft_widget w_strR220={
	.status = 0x01,
	.window = &Screen_gwin,
	.text_block = NULL,
	.code_block = MENU_strR220,
	.data = NULL,
	.func = widget_txt_simple,
};
#define MENU_L200x140 6
const static uint8_t MENU_str200X140[]={fSET_cursorXY(0,7),' ',Т,а,р,'i',л,к,а,' ','2','0','0','x','1','4','0',' ',' ',' ',' ',0,0};// 200x140
struct tft_widget w_str200x140={
	.status = 0x01,
	.window = &Screen_gwin,
	.text_block = NULL,
	.code_block = MENU_str200X140,
	.data = NULL,
	.func = widget_txt_simple,
};

#define MENU_MANUAL 7
const static uint8_t MENU_manual[]={fSET_cursorXY(0,8),' ',Н,а,л,а,ш,т,о,в,а,н,и,й,' ',' ',' ',' ',' ',' ',' ',0,0};// Налаштований
struct tft_widget w_strManual={
	.status = 0x01,
	.window = &Screen_gwin,
	.text_block = NULL,
	.code_block = MENU_manual,
	.data = NULL,
	.func = widget_txt_simple,
};


#define MENU_TEMP_Matrix 8
const static uint8_t MENU_temp_matrix[]={fSET_cursorXY(0,12),' ',Т,'-',р,е,г,у,л,я,т,о,р,' ',м,а,т,р,и,ц,я,0,0};//Т-регулятор матриця
struct tft_widget w_temp_matrix={
	.status = 0x01,
	.window = &Screen_gwin,
	.text_block = NULL,
	.code_block = MENU_temp_matrix,
	.data = NULL,
	.func = widget_txt_simple,
};

#define MENU_TEMP_punch 9
const static uint8_t MENU_temp_punch[]={fSET_cursorXY(0,13),' ',Т,'-',р,е,г,у,л,я,т,о,р,' ',п,у,а,н,с,о,н,0,0};//Т-регулятор матриця
struct tft_widget w_temp_punch={
	.status = 0x01,
	.window = &Screen_gwin,
	.text_block = NULL,
	.code_block = MENU_temp_punch,
	.data = NULL,
	.func = widget_txt_simple,
};

#define MENU_RELE_delay 10
const static uint8_t MENU_releD[]={fSET_cursorXY(0,14),' ',Р,е,л,е,' ',з,а,т,р,и,м,к,и,' ',' ',' ',' ',' ',' ',0,0};//Т-регулятор матриця
struct tft_widget w_releD={
	.status = 0x01,
	.window = &Screen_gwin,
	.text_block = NULL,
	.code_block = MENU_releD,
	.data = NULL,
	.func = widget_txt_simple,
};


#define MENU_COUNT 11
const static uint8_t MENU_count[]={fSET_cursorXY(0,15),' ',Л,'i',ч,'i',л,ь,н,и,к,' ',п,р,о,д,у,к,ц,'i',ї,0,0};//Лічильник продукції
struct tft_widget w_count={
	.status = 0x01,
	.window = &Screen_gwin,
	.text_block = NULL,
	.code_block = MENU_count,
	.data = NULL,
	.func = widget_txt_simple,
};
#define MENU_PNEVMO 12
const static uint8_t MENU_pnevmo[]={fSET_cursorXY(0,16),' ',П,н,е,в,м,о,ц,и,л,'i',н,д,р,' ',' ',' ',' ',' ',' ',0,0};//Лічильник продукції
struct tft_widget w_pnevmo={
	.status = 0x01,
	.window = &Screen_gwin,
	.text_block = NULL,
	.code_block = MENU_pnevmo,
	.data = NULL,
	.func = widget_txt_simple,
};
#define MENU_230Volt 13
const static uint8_t MENU_230volt[]={fSET_cursorXY(0,17),' ',Н,а,п,р,у,г,а,' ','2','3','0',В,' ',' ',' ',' ',' ',' ',' ',0,0};//Лічильник продукції
struct tft_widget w_230volt={
	.status = 0x01,
	.window = &Screen_gwin,
	.text_block = NULL,
	.code_block = MENU_230volt,
	.data = NULL,
	.func = widget_txt_simple,
};

#define MENU_SAVE_EXIT 14
const static uint8_t MENU_save_exit[]={fSET_cursorXY(0,18),' ',З,б,е,р,е,г,т,и,' ',т,а,' ',в,и,й,т,и,' ',' ',0,0};//Зберегти та вийти
struct tft_widget w_save_exit={
	.status = 0x01,
	.window = &Screen_gwin,
	.text_block = NULL,
	.code_block = MENU_save_exit,
	.data = NULL,
	.func = widget_txt_simple,
};
#define MENU_EXIT 15
const static uint8_t MENU_exit[]={fSET_cursorXY(0,19),' ',В,и,й,т,и,' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',0,0};//Вийти
struct tft_widget w_exit={
	.status = 0x01,
	.window = &Screen_gwin,
	.text_block = NULL,
	.code_block = MENU_exit,
	.data = NULL,
	.func = widget_txt_simple,
};

const static
struct tft_widget* Screen_gwidgets[]={

		&w_gmenu,
		&w_strD200,
		&w_strD275,
		&w_strD300,
		&w_strD370,
		&w_strR220,
		&w_str200x140,
		&w_strManual,
		&w_temp_matrix,
		&w_temp_punch,
		&w_releD,
		&w_count,
		&w_pnevmo,
		&w_230volt,
		&w_save_exit,
		&w_exit,

};

uint8_t screen_gbuild[]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
uint8_t screen_gdynamic[]={0};
void Screen_gkeys(struct TFT_panel* tft);
const struct tft_screen
Screen_gm={

	    .widgets = Screen_gwidgets,
		.n_widgets = sizeof(Screen_gwidgets)/sizeof(Screen_gwidgets[0]),
	    .build_widgets = screen_gbuild,			 // number
	    .n_build_widgets = sizeof(screen_gbuild),
	    .dynamic_widgets = screen_gdynamic, 		 // number widgets
	    .n_dynamic_widgets =sizeof(screen_gdynamic),
		.func_keys = Screen_gkeys,
	    .next = &Screen1,
	    .prev = NULL,

};




/*------------------------------- Widget MENU_1-------------------------------------*/
struct tft_window Screen2_win={

	.image_x0 = 0,
	.image_y0 = 0,
	.image_x1 = 0x13F,
	.image_y1 = 0x1dF,
	.cursor_x = 0,
	.cursor_y = 0,
	.color_font = color_WHITE,
	.color_background = 0x0003,
	.font = console18pt,

};
const static
uint8_t MENU1_build[]={
		fSET_font18pt,fSAVE_background,fSET_background(0x04,0x00),

		fSET_cursorXY(0,0) ,' ','P','r','o','g','r','a','m','s',' ','P','r','e','s','s',' ',' ',' ',' ',' ',
		fSET_cursorXY(0,6) ,' ','R','e','l','a','y',' ','D','e','l','a','y',' ',' ',' ',' ',' ',' ',' ',' ',
		fSET_cursorXY(0,9),'o','n','_','R','e','l','a','y',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
		fSET_cursorXY(0,12),'o','f','f','_','R','e','l','a','y',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
		fSET_cursorXY(0,15),' ','C','o','u','n','t','e','r',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
		fLOAD_background,
		0,0,0};

#define INFO_MENU 0
struct tft_widget
w_menu_build={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU1_build,
	.data = NULL,
	.func = NULL,
};


// Activated program menu

#define MENU_120x160 1
const static uint8_t MENU_str120x160[]={fSET_cursorXY(0,1),' ','1','2','0','x','1','6','0','|','2','0','0','x','1','4','0',' ',' ',' ',' ',0,0};// Plate 120x160/200x140;
struct tft_widget w_120x160={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_str120x160,
	.data = NULL,
	.func = widget_txt_simple,
};

#define MENU_D275_D300 2
const static uint8_t MENU_strD275_D300[]={fSET_cursorXY(0,2),' ','D','2','7','5','|','D','3','0','0',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',0,0};// Plate D300 / D275
struct tft_widget w_strD275_D300={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_strD275_D300,
	.data = NULL,
	.func = widget_txt_simple,
};

#define MENU_hD370 3
const static uint8_t MENU_hstrD370[]={fSET_cursorXY(0,3),' ','D','3','7','0',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',0,0};// Plate D370
struct tft_widget w_strhD370={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_strD370,
	.data = NULL,
	.func = widget_txt_simple,
};

#define MENU_hR220 4
const static uint8_t MENU_hstrR220[]={fSET_cursorXY(0,4),' ','R','2','2','0','|','R','1','4','4',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',0,0};// Plate R220/R144
struct tft_widget w_R220={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_strR220,
	.data = NULL,
	.func = widget_txt_simple,
};

#define MENU_hMANUAL 5
const static uint8_t MENU_strManual[]={fSET_cursorXY(0,5),' ','M','a','n','u','a','l',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',0,0};// Manual
struct tft_widget w_Manual={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_strManual,
	.data = NULL,
	.func = widget_txt_simple,
};

// end menu program --------------------

// Relay Delay configuration

#define MENU_RELE_time 6
const static uint8_t MENU_Rele_txt_time[]={fSET_cursorXY (0,7),'T','i','m','e',' ','d','e','l','a','y',':',' ',' ',' ',' ',' ',' ','m','s',' ',0,0};
struct tft_widget w_Rele_time={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block =  MENU_Rele_txt_time,
	.data = NULL,
	.func = widget_txt_simple,
};
#define MENU_RELE_id 7
const static uint8_t MENU_Rele_txt_id[]={fSET_cursorXY(0,8),'I','D',':',' ',' ',' ',' ',' ',' ',0,0};
struct tft_widget w_Rele_id={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_Rele_txt_id,
	.data = NULL,
	.func = widget_txt_simple,
};
#define MENU_RELE_mask 8
const static uint8_t MENU_Rele_txt_mask[]={fSET_cursorXY(9,8),'M','A','S','K',':',' ',' ',' ',' ',' ',' ',0,0};
struct tft_widget w_Rele_mask={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_Rele_txt_mask,
	.data = NULL,
	.func = widget_txt_simple,
};

// ON_Rele_msg

#define MENU_RELE_ON_id 9
const static uint8_t MENU_Rele_txt_id_on[]={fSET_cursorXY(0,10),'I','D',':',' ',' ',' ',' ',' ',' ',' ',' ',0,0};
struct tft_widget w_Rele_on_id={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block =  MENU_Rele_txt_id_on,
	.data = NULL,
	.func = widget_txt_simple,
};

#define MENU_RELE_ON_dlc 10
const static uint8_t MENU_Rele_txt_dlc_on[]={fSET_cursorXY(15,10),'D','L','C',':',' ',' ',0,0};
struct tft_widget w_Rele_on_dlc={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_Rele_txt_dlc_on,
	.data = NULL,
	.func = widget_txt_simple,
};

#define MENU_RELE_ON_msg 11
const static uint8_t MENU_Rele_txt_msg_on[]={fSET_cursorXY(0,11),'M','S','G',':','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0',0,0};
struct tft_widget w_Rele_on_msg={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_Rele_txt_msg_on,
	.data = NULL,
	.func = widget_txt_simple,
};

// OFF_Rele_msg

#define MENU_RELE_OFF_id 12
const static uint8_t MENU_Rele_txt_id_off[]={fSET_cursorXY(0,13),'I','D',':',' ',' ',' ',' ',' ',' ',' ',' ',0,0};
struct tft_widget w_Rele_off_id={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block =  MENU_Rele_txt_id_off,
	.data = NULL,
	.func = widget_txt_simple,
};
#define MENU_RELE_OFF_dlc 13
const static uint8_t MENU_Rele_txt_dlc_off[]={fSET_cursorXY(15,13),'D','L','C',':',' ',' ',0,0};
struct tft_widget w_Rele_off_dlc={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_Rele_txt_dlc_off,
	.data = NULL,
	.func = widget_txt_simple,
};
#define MENU_RELE_OFF_msg 14
const static uint8_t MENU_Rele_txt_msg_off[]={fSET_cursorXY(0,14),'M','S','G',':','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0',0,0};
struct tft_widget w_Rele_off_msg={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_Rele_txt_msg_off,
	.data = NULL,
	.func = widget_txt_simple,
};

// COUNTER ___
#define MENU_COUNTER_front 15
const static uint8_t MENU_Counter_txt_front[]={fSET_cursorXY (0,16),'F','r','o','n','t',' ','p','i','n',':',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',0,0};
struct tft_widget w_Counter_front={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block =  MENU_Counter_txt_front,
	.data = NULL,
	.func = widget_txt_simple,
};
#define MENU_COUNTER_id 16
const static uint8_t MENU_Counter_txt_id[]={fSET_cursorXY(0,17),'I','D',':',' ',' ',' ',' ',' ',' ',0,0};
struct tft_widget w_Counter_id={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_Counter_txt_id,
	.data = NULL,
	.func = widget_txt_simple,
};
#define MENU_COUNTER_mask 17
const static uint8_t MENU_Counter_txt_mask[]={fSET_cursorXY(9,17),'M','A','S','K',':',' ',' ',' ',' ',' ',' ',0,0};
struct tft_widget w_Counter_mask={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_Counter_txt_mask,
	.data = NULL,
	.func = widget_txt_simple,
};


const static
struct tft_widget* Screen_2_widgets[]={

		&w_menu_build,
		&w_120x160,
		&w_strD275_D300,
		&w_strD370,
		&w_R220,
		&w_Manual ,

		&w_Rele_time,
		&w_Rele_id,
		&w_Rele_mask,

		&w_Rele_on_id,
		&w_Rele_on_dlc,
		&w_Rele_on_msg,

		&w_Rele_off_id,
		&w_Rele_off_dlc,
		&w_Rele_off_msg,

		&w_Counter_front,
		&w_Counter_id,
		&w_Counter_mask




};
uint8_t screen2_build[]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
uint8_t screen2_dynamic[]={1};
void Screen2_keys(struct TFT_panel* tft);
const struct tft_screen
Screen2={

	    .widgets = Screen_2_widgets,
		.n_widgets = sizeof(Screen_2_widgets)/sizeof(Screen_2_widgets[0]),
	    .build_widgets = screen2_build,			 // number
	    .n_build_widgets = sizeof(screen2_build),
	    .dynamic_widgets = screen2_dynamic, 		 // number widgets
	    .n_dynamic_widgets =sizeof(screen2_dynamic),
		.func_keys = Screen2_keys,
	    .next = &Screen1,
	    .prev = NULL,

};


////////////////////////////////////////// SCREEN 3 /////////////////////////////////////////////


struct tft_window Screen3_win={

	.image_x0 = 0,
	.image_y0 = 0,
	.image_x1 = 0x13F,
	.image_y1 = 0x1dF,
	.cursor_x = 0,
	.cursor_y = 0,
	.color_font = color_WHITE,
	.color_background = 0x0007,
	.font = console18pt,

};
const static
uint8_t MENU2_build[]={
		fSET_font18pt,fSAVE_background,fSET_background(0x04,0x00),
		fSET_cursorXY(0,8) ,' ','T','e','m','p','e','r','a','t','u','r','e',' ','M','a','t','r','i','x',' ',
		fSET_cursorXY(0,11),' ','T','e','m','p','e','r','a','t','u','r','e',' ','P','u','n','c','h',' ',' ', // Manual
		fLOAD_background,
		0,0,0};

#define MENU_STR5 6
const static uint8_t MENU_str5[]={fSET_cursorXY(0,10),'O','N',':',' ',' ',' ',' ','O','F','F',':',' ',' ',' ',' ','C',':',' ',' ',' ',0,0};
struct tft_widget w_str5={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_str5,
	.data = NULL,
	.func = widget_txt_simple,
};

#define MENU_STR6 7
const static uint8_t MENU_str6[]={fSET_cursorXY(0,9),'I','D',':',' ',' ',' ',' ',' ',' ','M','A','S','K',':',' ',' ',' ',' ',' ',' ',0,0};
struct tft_widget w_str6={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_str6,
	.data = NULL,
	.func = widget_txt_simple,
};

#define MENU_STR7 8
const static uint8_t MENU_str7[]={fSET_cursorXY(0,11),'O','N',':',' ',' ',' ',' ','O','F','F',':',' ',' ',' ',' ','C',':',' ',' ',' ',0,0};
struct tft_widget w_str7={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_str7,
	.data = NULL,
	.func = widget_txt_simple,
};


#define MENU_STR8 9
const static uint8_t MENU_str8[]={fSET_cursorXY(0,12),'I','D',':',' ',' ',' ',' ',' ',' ','M','A','S','K',':',' ',' ',' ',' ',' ',' ',0,0};
struct tft_widget w_str8={
	.status = 0x01,
	.window = &Screen2_win,
	.text_block = NULL,
	.code_block = MENU_str8,
	.data = NULL,
	.func = widget_txt_simple,
};

#endif /* INC_TFT_WIDGETS_H_ */
