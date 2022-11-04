/*
 * ds1307.c
 *
 *  Created on: Oct 3, 2022
 *      Author: omark
 */

#include "ds1307.h"

static void ds1307_i2c_pins_config(void);
static void ds1307_i2c_config(void);
static void ds1307_write(uint8_t val, uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_addr);
static uint8_t binary_to_bcd(uint8_t val);
static uint8_t bcd_to_binary(uint8_t val);

I2Cx_Handle_t ds1307_i2c_handle;

/********************************************************************
 * @fn				- ds1307_init
 *
 * @brief			- configures the I2C signals pins and the I2C peripheral in standard mode,
 * 					- enables ACKing
 *
 * @param[in]		- none
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
uint8_t ds1307_init(void)
{
	uint8_t clock_state;

	// INITIALIZE I2C PINS
	ds1307_i2c_pins_config();

	// INITIALIZE I2C PERIPHERAL
	ds1307_i2c_config();

	// ENABLE I2C PERIPHERAL
	I2C_PControl(DS1307_I2C, ENABLE);

	// ENABLE ACKING
	I2C_ManageACKing(DS1307_I2C, ENABLE);

	// PULL CLOCK HALT TO GND
	ds1307_write(0x00, DS1307_ADDR_SEC);

	clock_state  = ds1307_read(DS1307_ADDR_SEC);

	return ((clock_state >> 7) & (0x1));
}

/********************************************************************
 * @fn				- ds1307_set_current_time
 *
 * @brief			- sets the current time in hrs, mins, secs
 *
 * @param[in]		- time user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void ds1307_set_current_time(RTC_Time_t *rtc_time)
{
	uint8_t seconds, hrs;
	seconds = binary_to_bcd(rtc_time->seconds);
	seconds &= ~(1 << 7);
	ds1307_write(seconds, DS1307_ADDR_SEC);

	ds1307_write(binary_to_bcd(rtc_time->minutes), DS1307_ADDR_MIN);

	hrs = binary_to_bcd(rtc_time->hours);
	if (rtc_time->format == TIME_FORMAT_24HR)
	{
		hrs &= ~(1 << 6);
	}
	else
	{
		hrs |= (1 << 6);
		hrs = (rtc_time->format == TIME_FORMAT_12HR_PM) ? hrs | (1 << 5) : hrs & ~(1 << 5);
	}
	ds1307_write(hrs, DS1307_ADDR_HR);
}

/********************************************************************
 * @fn				- ds1307_get_current_time
 *
 * @brief			- assign the time structure members with the current time values
 *
 * @param[in]		- time user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void ds1307_get_current_time(RTC_Time_t *rtc_time)
{
	uint8_t seconds, hrs;
	seconds = ds1307_read(DS1307_ADDR_SEC);
	seconds &= ~(1 << 7);
	rtc_time->seconds = bcd_to_binary(seconds);

	rtc_time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));

	hrs = ds1307_read(DS1307_ADDR_HR);
	if (hrs & (1 << 6))
	{
		// 12 Hour format
		rtc_time->format = hrs & (1 << 5);
		hrs &= ~(0x3 << 5);
	}
	else
	{
		// 24 Hour format
		rtc_time->format = TIME_FORMAT_24HR;
	}
	rtc_time->hours = bcd_to_binary(hrs);
}

/********************************************************************
 * @fn				- ds1307_set_current_date
 *
 * @brief			- sets the current time in hrs, mins, secs
 *
 * @param[in]		- date user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void ds1307_set_current_date(RTC_Date_t *rtc_date)
{
	ds1307_write(binary_to_bcd(rtc_date->date), DS1307_ADDR_DATE);

	ds1307_write(binary_to_bcd(rtc_date->day), DS1307_ADDR_DAY);

	ds1307_write(binary_to_bcd(rtc_date->month), DS1307_ADDR_MON);

	ds1307_write(binary_to_bcd(rtc_date->year), DS1307_ADDR_YR);
}

/********************************************************************
 * @fn				- ds1307_get_current_date
 *
 * @brief			- assign the time structure members with the current time values
 *
 * @param[in]		- date user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void ds1307_get_current_date(RTC_Date_t *rtc_date)
{
	rtc_date->date = bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));
	rtc_date->day = bcd_to_binary(ds1307_read(DS1307_ADDR_DAY));
	rtc_date->month = bcd_to_binary(ds1307_read(DS1307_ADDR_MON));
	rtc_date->year = bcd_to_binary(ds1307_read(DS1307_ADDR_YR));
}

/******************************* PRIVATE HELPER FUNCTIONS **********************************/

// I2Cx SDA, SCL signals configuration
static void ds1307_i2c_pins_config(void)
{
	GPIOx_Handle_t ds1307_i2c_scl;
	GPIOx_Handle_t ds1307_i2c_sda;

	memset(&ds1307_i2c_scl, 0, sizeof(ds1307_i2c_scl));
	memset(&ds1307_i2c_sda, 0, sizeof(ds1307_i2c_sda));

	/*
	 * I2C2
	 * SCL : PF1
	 * SDA : PF0
	 * */

	ds1307_i2c_scl.pGPIO = DS1307_I2C_GPIO_PORT;
	ds1307_i2c_scl.GPIO_pinConfig.pinMode = GPIO_MODE_ALT;
	ds1307_i2c_scl.GPIO_pinConfig.pinAltFuncMode = GPIO_AF4;
	ds1307_i2c_scl.GPIO_pinConfig.pinNumber = DS1307_I2C_SCL_PIN;
	ds1307_i2c_scl.GPIO_pinConfig.pinOpType = GPIO_OUT_TYPE_OD;
	ds1307_i2c_scl.GPIO_pinConfig.pinPuPdCtrl = DS1307_I2C_PUPD;
	ds1307_i2c_scl.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_HIGH;

	GPIO_Init(&ds1307_i2c_scl);

	ds1307_i2c_sda.pGPIO = DS1307_I2C_GPIO_PORT;
	ds1307_i2c_sda.GPIO_pinConfig.pinMode = GPIO_MODE_ALT;
	ds1307_i2c_sda.GPIO_pinConfig.pinAltFuncMode = GPIO_AF4;
	ds1307_i2c_sda.GPIO_pinConfig.pinNumber = DS1307_I2C_SDA_PIN;
	ds1307_i2c_sda.GPIO_pinConfig.pinOpType = GPIO_OUT_TYPE_OD;
	ds1307_i2c_sda.GPIO_pinConfig.pinPuPdCtrl = DS1307_I2C_PUPD;
	ds1307_i2c_sda.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_HIGH;

	GPIO_Init(&ds1307_i2c_sda);
}

// configuring the I2Cx peripheral speed, ACKing
static void ds1307_i2c_config(void)
{
	ds1307_i2c_handle.pI2Cx = DS1307_I2C;
	ds1307_i2c_handle.I2Cx_Config.I2C_AckCtrl = I2C_ACK_EN;
	ds1307_i2c_handle.I2Cx_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;

	I2C_Init(&ds1307_i2c_handle);
}

// writes a given value in the given register address
static void ds1307_write(uint8_t val, uint8_t reg_addr)
{
	uint8_t tx[2];
	tx[0] = reg_addr;
	tx[1] = val;
	I2C_MasterTX(&ds1307_i2c_handle, tx, 2, DS1307_I2C_ADDR, 0);
}

// returns byte value stored in the given register address
static uint8_t ds1307_read(uint8_t reg_addr)
{
	uint8_t temp;
	I2C_MasterTX(&ds1307_i2c_handle, &reg_addr, 1, DS1307_I2C_ADDR, I2C_SR_ENABLE);
	I2C_MasterRX(&ds1307_i2c_handle, &temp, 1, DS1307_I2C_ADDR, I2C_SR_DISABLE);
	return temp;
}

static uint8_t binary_to_bcd(uint8_t val)
{
	uint8_t m, n;
	if (val >= 10)
	{
		m = val / 10;
		n = val % 10;
		val = (m << 4) | n;
	}

	return val;
}

static uint8_t bcd_to_binary(uint8_t val)
{
	uint8_t m, n;
	m = (uint8_t)(val >> 4) * 10;
	n = val & (uint8_t)(0x0f);
	return (m+n);
}
