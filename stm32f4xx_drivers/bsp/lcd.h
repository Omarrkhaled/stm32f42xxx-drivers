/*
 * lcd.h
 *
 *  Created on: Oct 3, 2022
 *      Author: omark
 */

#ifndef LCD_H_
#define LCD_H_

#include "main.h"


/****************************** APPLICATION CONFIGURABLE ITEMS *******************************/

#define LCD_GPIO_PORT					GPIOD
#define LCD_GPIO_RS						GPIO_PIN_11
#define LCD_GPIO_RW						GPIO_PIN_12
#define LCD_GPIO_E						GPIO_PIN_13
#define LCD_GPIO_D0						GPIO_PIN_0
#define LCD_GPIO_D1						GPIO_PIN_1
#define LCD_GPIO_D2						GPIO_PIN_2
#define LCD_GPIO_D3						GPIO_PIN_3
#define LCD_GPIO_D4						GPIO_PIN_4
#define LCD_GPIO_D5						GPIO_PIN_5
#define LCD_GPIO_D6						GPIO_PIN_6
#define LCD_GPIO_D7						GPIO_PIN_7

/*************************************** LCD COMMANDS ****************************************/
#define LCD_CMD_4DL_2N_5X8F				0x28U
#define LCD_CMD_8DL_2N_5X8F				0x38U
#define LCD_CMD_DISP_OFF				0x08U
#define LCD_CMD_DON_CURON				0x0EU
#define LCD_CMD_DON_CUROFF				0x0CU
#define LCD_CMD_DISP_CLEAR				0x01U
#define LCD_CMD_RET_HOME				0x02U
#define LCD_CMD_ENTRY					0x06U

#define LCD_BUSY						1U
#define LCD_NOT_BUSY					0


/************************************** Supported APIs ***************************************/
void lcd_init(void);
void lcd_send_command(uint8_t cmd);
void lcd_send_data(uint8_t ch);
void lcd_disp_clear(void);
void lcd_display_return_home(void);
void lcd_print_string(char* str);
void lcd_set_cursor(uint8_t row, uint8_t col);

#endif /* LCD_H_ */
