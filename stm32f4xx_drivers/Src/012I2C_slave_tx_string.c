/*
 * 012I2C_slave_tx_string.c
 *
 *  Created on: Oct 1, 2022
 *      Author: omark
 */


#include "stm32f429xx.h"
#include <stdio.h>
#include <string.h>

#define BUFFER_MAX				32
#define SLAVE_ADDR				0x69
#define MY_ADDR					SLAVE_ADDR


I2Cx_Handle_t I2C2_handle;
uint32_t command_code;

void delay(void);
void I2C2_GPIOF_Cfg(void);
void I2C2_Cfg(void);

uint8_t tx_buffer[BUFFER_MAX] = "STM32 slave mode testing..";

int main(void)
{
	// I2C2_GPIOF pin configs
	I2C2_GPIOF_Cfg();

	// I2C2 configs
	I2C2_Cfg();

	// I2C2 event & error IRQ line enable
	I2C_IRQInterruptConfig(IRQ_NO_I2C2_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C2_ER, ENABLE);

	// enable interrupt control bits
	I2C_SlaveEnDiCallbackEvents(I2C2, ENABLE);

	// enable peripehral I2C2
	I2C_PControl(I2C2, ENABLE);

	// enable ACKing
	I2C_ManageACKing(I2C2, ENABLE);

	while (1);

	return 0;
}

void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}

void I2C2_GPIOF_Cfg(void)
{
	GPIOx_Handle_t GPIOF_handle;

	GPIOF_handle.pGPIO = GPIOF;
	GPIOF_handle.GPIO_pinConfig.pinMode = GPIO_MODE_ALT;
	GPIOF_handle.GPIO_pinConfig.pinAltFuncMode = GPIO_AF4;
	GPIOF_handle.GPIO_pinConfig.pinOpType = GPIO_OUT_TYPE_OD;
	GPIOF_handle.GPIO_pinConfig.pinPuPdCtrl = GPIO_PIN_PUP;
	GPIOF_handle.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_HIGH;

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
	I2C2_handle.I2Cx_Config.I2C_DevAddrr = MY_ADDR;
	I2C2_handle.I2Cx_Config.I2C_SCLSpeed = I2C_SCL_SPEED_STD;

	I2C_Init(&I2C2_handle);
}

void I2C2_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C2_handle);
}

void I2C2_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C2_handle);
}

void I2C_ApplicationCallback(I2Cx_Handle_t *pHandle, uint8_t appEvent)
{
	static uint8_t cnt = 0;

	if (appEvent == I2C_EV_DATA_REQ)
	{
		// master sent a command, decode the command and transmit the requested data
		if (command_code == 0x51)
		{
			// send the data length
			I2C_SlaveTX(pHandle->pI2Cx, strlen((char*)tx_buffer));

		}
		else if (command_code == 0x52)
		{
			// send the data
			I2C_SlaveTX(pHandle->pI2Cx, tx_buffer[cnt++]);
		}
	}

	else if (appEvent == I2C_EV_DATA_RCV)
	{
		// read the command sent by master
		command_code = I2C_SlaveRX(pHandle->pI2Cx);
	}

	else if (appEvent == I2C_ERROR_AF)
	{
		// happens only if the I2C peripheral is in transmission state, and decoded to
		// send no more
		command_code == 0xff;
		cnt = 0;
	}

	else if (appEvent == I2C_EV_STOP)
	{
		// this happens only during slave reception .
		// master has ended the I2C communication with the slave.
	}
}
