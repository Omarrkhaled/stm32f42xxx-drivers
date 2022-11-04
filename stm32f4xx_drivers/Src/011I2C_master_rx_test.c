/*
 * 011I2C_master_rx_test.c
 *
 *  Created on: Sep 30, 2022
 *      Author: omark
 */

#include "stm32f429xx.h"
#include <stdio.h>

#define CMD_SEND_LEN			0x51
#define CMD_SEND_DATA			0X52
#define BUFFER_MAX				32
#define SLAVE_ADDR				0x68


I2Cx_Handle_t I2C2_handle;

uint8_t rxComplt = RESET;

void delay(void);
void I2C2_GPIOF_Cfg(void);
void I2C2_Cfg(void);
void Btn_Cfg(void);


int main(void)
{

	printf("Application is running\n");

	uint8_t command_code;
	uint8_t len_read;
	uint8_t rcv_buffer[BUFFER_MAX];

	// USER button configuration
	Btn_Cfg();

	// I2C2_GPIOF pin configs
	I2C2_GPIOF_Cfg();

	// I2C2 configs
	I2C2_Cfg();

	// I2C2 event & error IRQ line enable
	I2C_IRQInterruptConfig(IRQ_NO_I2C2_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C2_ER, ENABLE);

	// enable peripehral I2C2
	I2C_PControl(I2C2, ENABLE);

	// enable ACKing
	I2C_ManageACKing(I2C2, ENABLE);

	while (1)
	{
		while (! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13));

		delay();

		// command the slave to send length of data expected
		command_code = CMD_SEND_LEN;
		while (I2C_MasterTXIT(&I2C2_handle, &command_code, 1, SLAVE_ADDR, I2C_SR_ENABLE) != I2C_STATE_RDY);

		// read the length information sent from slave TX
		while (I2C_MasterRXIT(&I2C2_handle, &len_read, 1, SLAVE_ADDR, I2C_SR_ENABLE) != I2C_STATE_RDY);

		// command the slave to send the data
		command_code = CMD_SEND_DATA;
		while (I2C_MasterTXIT(&I2C2_handle, &command_code, 1, SLAVE_ADDR, I2C_SR_ENABLE) != I2C_STATE_RDY);

		// receive the data
		while (I2C_MasterRXIT(&I2C2_handle, rcv_buffer, len_read, SLAVE_ADDR, I2C_SR_DISABLE) != I2C_STATE_RDY);

		rxComplt = RESET;

		while (rxComplt != SET)
		{

		}

		rcv_buffer[len_read+1] = '\0';
		printf("Rcvd: %s", rcv_buffer);
	}

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
	GPIOF_handle.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_MED;

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

void Btn_Cfg(void)
{
	GPIOx_Handle_t GPIOC_handle;

	GPIOC_handle.pGPIO = GPIOC;
	GPIOC_handle.GPIO_pinConfig.pinNumber = GPIO_PIN_13;
	GPIOC_handle.GPIO_pinConfig.pinMode = GPIO_MODE_IN;
	GPIOC_handle.GPIO_pinConfig.pinOpType = GPIO_OUT_TYPE_PP;
	GPIOC_handle.GPIO_pinConfig.pinPuPdCtrl = GPIO_NO_PUPD;
	GPIOC_handle.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_HIGH;

	GPIO_Init(&GPIOC_handle);
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
	if (appEvent == I2C_EV_TX_CMPLT)
	{
		printf("Transmission finished\n");
	}
	else if (appEvent == I2C_EV_RX_CMPLT)
	{
		printf("Reception finished\n");
		rxComplt = SET;
	}
	else if (appEvent == I2C_EV_STOP)
	{
		printf("Stop condition generated\n");
	}
	else if (appEvent == I2C_ERROR_AF)
	{
		printf("Error: ACKing failure\n");
		I2C_CloseTransmission(pHandle);
		I2C_GenerateStopCondition(pHandle->pI2Cx);
		while (1);
	}
}
