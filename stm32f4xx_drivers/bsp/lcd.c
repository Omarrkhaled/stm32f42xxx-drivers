/*
 * lcd.c
 *
 *  Created on: Oct 3, 2022
 *      Author: omark
 */

#include "main.h"

//static void write_8_bits(uint8_t val);
static void udelay(uint32_t cnt);
static void lcd_enable(void);
static void write_4_bits(uint8_t val);
static void mdelay(uint32_t cnt);

/*
 * PORT: D
 * RS => PD11
 * RW => PD12
 * E  => PD13
 * D0 => PD0
 * D1 => PD1
 * D2 => PD2
 * D3 => PD3
 * D4 => PD4
 * D5 => PD5
 * D6 => PD6
 * D7 => PD7
 *
 * */

void lcd_init()
{
	// GPIO PINS USED FOR LCD CONNECTIONS
	GPIO_InitTypeDef lcd_signals;

	lcd_signals.Mode = GPIO_MODE_OUTPUT_PP;
	lcd_signals.Pull = GPIO_NOPULL;
	lcd_signals.Speed = GPIO_SPEED_HIGH;
	// RS signal
	lcd_signals.Pin = LCD_GPIO_RS;
	HAL_GPIO_Init(LCD_GPIO_PORT, &lcd_signals);
	// RnW signal
	lcd_signals.Pin = LCD_GPIO_RW;
	HAL_GPIO_Init(LCD_GPIO_PORT, &lcd_signals);
	// E signal
	lcd_signals.Pin = LCD_GPIO_E;
	HAL_GPIO_Init(LCD_GPIO_PORT, &lcd_signals);
	// D4 => D7
	lcd_signals.Pin = LCD_GPIO_D4;
	HAL_GPIO_Init(LCD_GPIO_PORT, &lcd_signals);
	lcd_signals.Pin = LCD_GPIO_D5;
	HAL_GPIO_Init(LCD_GPIO_PORT, &lcd_signals);
	lcd_signals.Pin = LCD_GPIO_D6;
	HAL_GPIO_Init(LCD_GPIO_PORT, &lcd_signals);
	lcd_signals.Pin = LCD_GPIO_D7;
	HAL_GPIO_Init(LCD_GPIO_PORT, &lcd_signals);


	// LCD CONFIGURATIONS

	// WAIT FOR ~40 MS
	mdelay(50);


	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(0x3);

	mdelay(5);

	write_4_bits(0x3);

	udelay(150);

	write_4_bits(0x3);
	write_4_bits(0x2);

	lcd_send_command(LCD_CMD_4DL_2N_5X8F);

	lcd_send_command(LCD_CMD_DISP_OFF);

	lcd_send_command(LCD_CMD_DISP_CLEAR);

	lcd_send_command(LCD_CMD_ENTRY);

	lcd_send_command(LCD_CMD_DON_CUROFF);

}

void lcd_send_command(uint8_t cmd)
{
	// RS = 0(command), RnW = 0(write)
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(cmd >> 4);

	write_4_bits(cmd & 0x0f);
}

void lcd_send_data(uint8_t ch)
{
	// RS = 1, RnW = 0
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(ch >> 4);

	write_4_bits(ch & 0x0F);

}

void lcd_print_string(char* str)
{
	do
	{
		lcd_send_data(*str++);
	}
	while (*str);
}

void lcd_disp_clear(void)
{
	lcd_send_command(LCD_CMD_DISP_CLEAR);
	mdelay(2);
}

void lcd_display_return_home(void)
{
	lcd_send_command(LCD_CMD_RET_HOME);
	mdelay(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col)
{
	col--;
	switch (row)
	{
	case 1:
		lcd_send_command((col | 0x80));
		break;
	case 2:
		lcd_send_command((col | 0xC0));
		break;
	default:
		break;
	}
}



/************************************ PRIVATE FUNCTIONS ****************************************/
static void write_4_bits(uint8_t val)
{
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_D4, (val >> 0) & 0x01);
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_D5, (val >> 1) & 0x01);
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_D6, (val >> 2) & 0x01);
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_D7, (val >> 3) & 0x01);

	lcd_enable();
}

static void lcd_enable()
{
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_E, GPIO_PIN_SET);
	udelay(400);
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_E, GPIO_PIN_RESET);
	udelay(200);
}

static void mdelay(uint32_t cnt)
{
	for (uint32_t i = 0; i < (cnt*1000); i++);
}

static void udelay(uint32_t cnt)
{
	for (uint32_t i = 0; i < cnt; i++);
}

/*
static void write_8_bits(uint8_t val)
{
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_D0, (val >> 0) & 0x01);
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_D1, (val >> 1) & 0x01);
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_D2, (val >> 2) & 0x01);
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_D3, (val >> 3) & 0x01);
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_D4, (val >> 4) & 0x01);
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_D5, (val >> 5) & 0x01);
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_D6, (val >> 6) & 0x01);
	HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_GPIO_D7, (val >> 7) & 0x01);

	lcd_enable();
}
*/
