/*
 * RTC_Display.c
 *
 *  Created on: Oct 3, 2022
 *      Author: omark
 */

#include "stm32f429xx.h"
#include "ds1307.h"
#include "lcd.h"
#include <stdio.h>
#include <stdlib.h>

#define SYSTICK_REG_CSR						((uint32_t*)0xE000E010)
#define SYSTICK_REG_RVR						((uint32_t*)0xE000E014)
#define SYSTICK_CLOCK_SRC					16000000UL
#define SYSTICK_CTRL_EN_BIT					0U
#define SYSTICK_CTRL_INT_BIT				1U
#define SYSTICK_CTRL_CLKSRC_BIT				2U

char* get_day_of_week(uint8_t d);
char* time_to_string(RTC_Time_t *p_time);
void number_to_string(uint8_t num, char* buf);
char* date_to_string(RTC_Date_t *p_date);
void init_systick_timer(uint32_t tick_hz);

RTC_Time_t curr_time;
RTC_Date_t curr_date;

int main(void)
{
	lcd_init();
	lcd_set_cursor(1, 1);

	ds1307_init();

	init_systick_timer(1);

	curr_date.day = THURSDAY;
	curr_date.date = 6;
	curr_date.month = 10;
	curr_date.year = 22;

	curr_time.format = TIME_FORMAT_12HR_AM;
	curr_time.hours  = 10;
	curr_time.minutes = 35;
	curr_time.seconds = 0;

	ds1307_set_current_date(&curr_date);
	ds1307_set_current_time(&curr_time);

	ds1307_get_current_date(&curr_date);
	ds1307_get_current_time(&curr_time);

	char *am_pm;

	if (curr_time.format != TIME_FORMAT_24HR)
	{
		am_pm = (curr_time.format) ? "PM" : "AM";
		//printf("Current Time = %s %s\n", time_to_string(&curr_time), am_pm);
		lcd_print_string(time_to_string(&curr_time));
		lcd_print_string(am_pm);
	}
	else
	{
		//printf("Current Time = %s\n", time_to_string(&curr_time));
		lcd_print_string(time_to_string(&curr_time));
	}
	//printf("Current Date = %s <%s>\n", date_to_string(&curr_date), get_day_of_week(curr_date.day));
	lcd_set_cursor(2, 1);
	lcd_print_string(date_to_string(&curr_date));

	while (1);

	return 0;
}

char* get_day_of_week(uint8_t d)
{
	static char* days[] = {"Saturday", "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday"};
	return days[d-1];
}

void number_to_string(uint8_t num, char* buf)
{
	if (num < 10)
	{
		buf[0] = 48;
		buf[1] = num + 48;
	}
	else if (num >= 10 && num < 99)
	{
		buf[0] = (num / 10) + 48;
		buf[1] = (num % 10) + 48;
	}
}

//hh:mm:ss
char* time_to_string(RTC_Time_t *p_time)
{
	static char buffer[9];
	buffer[2] = ':';
	buffer[5] = ':';
	number_to_string(p_time->hours, buffer);
	number_to_string(p_time->minutes, &buffer[3]);
	number_to_string(p_time->seconds, &buffer[6]);
	buffer[8] = '\0';
	return buffer;
}

//dd/mm/yy
char* date_to_string(RTC_Date_t *p_date)
{
	static char buff[9];
	buff[2] = '/';
	buff[5] = '/';
	number_to_string(p_date->date, buff);
	number_to_string(p_date->month, &buff[3]);
	number_to_string(p_date->year, &buff[6]);
	buff[8] = '\0';
	return buff;
}

void init_systick_timer(uint32_t tick_hz)
{
	uint32_t count_val = (SYSTICK_CLOCK_SRC / tick_hz) - 1;

	*(SYSTICK_REG_RVR) &= ~(0x00ffffff);

	*(SYSTICK_REG_RVR) |= count_val;

	*(SYSTICK_REG_CSR) |= (1 << SYSTICK_CTRL_INT_BIT);
	*(SYSTICK_REG_CSR) |= (1 << SYSTICK_CTRL_CLKSRC_BIT);
	*(SYSTICK_REG_CSR) |= (1 << SYSTICK_CTRL_EN_BIT);

}

void SysTick_Handler()
{
	ds1307_get_current_time(&curr_time);
	lcd_set_cursor(1, 1);
	char *am_pm;
	if (curr_time.format != TIME_FORMAT_24HR)
	{
		am_pm = (curr_time.format) ? "PM" : "AM";
		//printf("Current Time = %s %s\n", time_to_string(&curr_time), am_pm);
		lcd_print_string(time_to_string(&curr_time));
		lcd_print_string(am_pm);
	}
	else
	{
		//printf("Current Time = %s\n", time_to_string(&curr_time));
		lcd_print_string(time_to_string(&curr_time));
	}

	ds1307_get_current_date(&curr_date);
	//printf("Current Date = %s <%s>\n", date_to_string(&curr_date), get_day_of_week(curr_date.day));
	lcd_set_cursor(2, 1);
	lcd_print_string(date_to_string(&curr_date));
	//lcd_print_string("<");
	//lcd_print_string(get_day_of_week(curr_date.day));
	//lcd_print_string(">");
}
