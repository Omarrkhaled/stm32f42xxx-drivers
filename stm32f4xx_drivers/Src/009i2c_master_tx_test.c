/*
 * 009i2c_master_tx_test.c
 *
 *  Created on: Sep 28, 2022
 *      Author: omark
 */


#include "stm32f429xx.h"
#include <string.h>

#define SLAVE_ADDR				0x68

I2Cx_Handle_t I2C2_handle;

void Btn_Cfg(void);
void I2C2_GPIOF_Cfg(void);
void I2C2_Cfg(void);
void delay(void);


/*
 * Peripheral Used : I2C2
 * SDA => PF0
 * SCL => PF1
 *
 * */


int main(void)
{
	char user_data[] = "Hello world\n";

	// Configure USER button
	Btn_Cfg();

	// Configure GPIOF I2C2 pin signals
	I2C2_GPIOF_Cfg();

	// Configure I2C2 peripheral
	I2C2_Cfg();

	// enable I2C2
	I2C_PControl(I2C2, ENABLE);

	while (1)
	{
		// hang till button is pressed
		while (! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13));

		delay();

		I2C_MasterTX(&I2C2_handle, (uint8_t*)user_data, strlen(user_data), SLAVE_ADDR);

	}

	return 0;
}

void Btn_Cfg()
{
	GPIOx_Handle_t GPIOC_handle;

	GPIOC_handle.pGPIO = GPIOC;
	GPIOC_handle.GPIO_pinConfig.pinNumber = GPIO_PIN_13;
	GPIOC_handle.GPIO_pinConfig.pinMode = GPIO_MODE_IN;
	GPIOC_handle.GPIO_pinConfig.pinOpType = GPIO_OUT_TYPE_PP;
	GPIOC_handle.GPIO_pinConfig.pinPuPdCtrl = GPIO_NO_PUPD;
	GPIOC_handle.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_LOW;

	GPIO_Init(&GPIOC_handle);
}

void I2C2_GPIOF_Cfg()
{
	GPIOx_Handle_t GPIOF_handle;

	GPIOF_handle.pGPIO = GPIOF;
	GPIOF_handle.GPIO_pinConfig.pinMode = GPIO_MODE_ALT;
	GPIOF_handle.GPIO_pinConfig.pinAltFuncMode = GPIO_AF4;
	GPIOF_handle.GPIO_pinConfig.pinOpType = GPIO_OUT_TYPE_OD;
	GPIOF_handle.GPIO_pinConfig.pinPuPdCtrl = GPIO_PIN_PUP;
	GPIOF_handle.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_LOW;

	// SDA
	GPIOF_handle.GPIO_pinConfig.pinNumber = GPIO_PIN_0;
	GPIO_Init(&GPIOF_handle);

	// SCL
	GPIOF_handle.GPIO_pinConfig.pinNumber = GPIO_PIN_1;
	GPIO_Init(&GPIOF_handle);
}

void I2C2_Cfg()
{

	I2C2_handle.pI2Cx = I2C2;
	I2C2_handle.I2Cx_Config.I2C_AckCtrl = I2C_ACK_EN;
	I2C2_handle.I2Cx_Config.I2C_SCLSpeed = I2C_SCL_SPEED_STD;

	I2C_Init(&I2C2_handle);
}

void delay()
{
	for (uint32_t i = 0; i < 500000/2; i++);
}
