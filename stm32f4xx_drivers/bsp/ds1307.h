/*
 * ds1307.h
 *
 *  Created on: Oct 3, 2022
 *      Author: omark
 */

#ifndef DS1307_H_
#define DS1307_H_

#include "stm32f429xx.h"
#include <stdint.h>
#include <string.h>

/************************************* REGISTER ADDRESSES **************************************/
#define DS1307_ADDR_SEC						0x00
#define DS1307_ADDR_MIN						0x01
#define DS1307_ADDR_HR						0x02
#define DS1307_ADDR_DAY						0x03
#define DS1307_ADDR_DATE					0x04
#define DS1307_ADDR_MON						0x05
#define DS1307_ADDR_YR						0x06

/**************************************** TIME FORMATS *****************************************/
#define TIME_FORMAT_12HR_AM					0
#define TIME_FORMAT_12HR_PM					1U
#define TIME_FORMAT_24HR					2U

/************************************** SLAVE ADDRESS ******************************************/
#define DS1307_I2C_ADDR						0x68

/******************************************* DAYS **********************************************/
#define SATURDAY							1U
#define SUNDAY								2U
#define MONDAY								3U
#define TUESDAY								4U
#define WEDNESDAY							5U
#define THURSDAY							6U
#define FRIDAY								7U

/******************************* APPLICATION CONFIGURABLE ITEMS *******************************/
#define DS1307_I2C							I2C2
#define DS1307_I2C_GPIO_PORT				GPIOF
#define DS1307_I2C_SCL_PIN					GPIO_PIN_1
#define DS1307_I2C_SDA_PIN					GPIO_PIN_0
#define DS1307_I2C_SPEED					I2C_SCL_SPEED_STD
#define DS1307_I2C_PUPD						GPIO_PIN_PUP

/**********************************************************************************************/

/************************************* Registers Structure ************************************/

typedef struct
{
	uint8_t day;
	uint8_t month;
	uint8_t year;
	uint8_t date;

} RTC_Date_t;


typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t format;

} RTC_Time_t;


/************************************** Supported APIs ***************************************/

uint8_t ds1307_init(void);
void ds1307_set_current_time(RTC_Time_t *rtc_time);
void ds1307_get_current_time(RTC_Time_t *rtc_time);
void ds1307_set_current_date(RTC_Date_t *rtc_date);
void ds1307_get_current_date(RTC_Date_t *rtc_date);

#endif /* DS1307_H_ */
