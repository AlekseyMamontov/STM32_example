/*
 * tft_init_display.h
 *
 *  Created on: Jan 11, 2024
 *      Author: oleksii
 */

#ifndef INC_TFT_INIT_DISPLAY_H_
#define INC_TFT_INIT_DISPLAY_H_

///////////////// Init tft display //////////////////////

#define STOP_INIT  0xFF
#define PAUSE_INIT 0xFE



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



 #define INIT_ILI9486 init_ili9486
 #include "init_ili9486.h"
 #define INIT_R61581  init_r61581
 #include "init_r61581.h"


 #define MAX_info_block 20 // максимальное количество информационных блоков



 struct TFT_screen_panel {

 	struct TFT_window *window;

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

 };

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

 };


 r61581
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

 void tft_printf(struct TFT_screen_panel *panel, uint8_t *massiv){

 	int i=0;

 	while (massiv[i]){

 	 if (massiv[i] >= 0x20){

 		print_char_tft(panel,massiv[i]);

 		}else { switch (massiv[i]){

 					case SET_cursorX: //set cursor X
 						i++;
 						panel->window->cursor_x = massiv[i];
 						break;
 					case SET_cursorY:
 						i++;
 						panel->window->cursor_y = massiv[i];
 						break;

 					case SET_font:
 						i++;
 						panel->window->color_font = massiv[i];
 						panel->window->color_font = panel->window->color_font << 8;
 						i++;
 						panel->window->color_font = panel->window->color_font | massiv[i];
 						break;
 					case SET_background:
 						i++;
 						panel->window->color_background = massiv[i];
 						panel->window->color_background = panel->window->color_background << 8;
 						i++;
 						panel->window->color_background = panel->window->color_background | massiv[i];
 					break;

 					case SET_font18pt:
 						panel->window->font = console18pt;
 					break;

 					case SET_font8pt:
 						panel->window->font = console8pt;
 					break;

 					case SET_fontNumber:
 					panel->window->font = number20pt;
 					//printf("Массив %d \n", i);
 					break;

 					case 0x0D :
 						panel->window->cursor_x = 0;
 						panel->window->cursor_y = panel->window->cursor_y +1 ;
 					break;
 				};
 			};
 		i++;
 	};


 };

  */

#endif /* INC_TFT_INIT_DISPLAY_H_ */
