/*
 * 014UART_tx_test.c
 *
 *  Created on: Oct 2, 2022
 *      Author: omark
 */


#include "stm32f429xx.h"
#include <string.h>
#include <stdio.h>


USARTx_Handle_t USART2_Handle;

void USART2_GPIOD_Cfg(void);
void USART2_Cfg(void);
void Btn_Cfg(void);
void delay(void);

char user_data[1024] = "UART Testing..\n";

int main(void)
{
	Btn_Cfg();

	USART2_GPIOD_Cfg();

	USART2_Cfg();

	USART_PControl(USART2, ENABLE);

	while (1)
	{
		while (! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13));

		delay();

		USART_SendData(&USART2_Handle, (uint8_t*)user_data, strlen(user_data));

	}

	return 0;

}

void USART2_Cfg(void)
{
	USART2_Handle.pUSARTx = USART2;
	USART2_Handle.USARTx_Config.USART_wordLength = USART_WORD_8BITS;
	USART2_Handle.USARTx_Config.USART_no_of_stopBits = USART_STOP_BITS_1;
	USART2_Handle.USARTx_Config.USART_baudRate = USART_STD_BAUDRATE_115200;
	USART2_Handle.USARTx_Config.USART_mode = USART_MODE_TX_ONLY;
	USART2_Handle.USARTx_Config.USART_parityControl = USART_PARITY_DISABLE;
	USART2_Handle.USARTx_Config.USART_hardwareFlowControl = USART_HW_FLOW_CTRL_NONE;

	USART_Init(&USART2_Handle);
}

void USART2_GPIOD_Cfg(void)
{
	GPIOx_Handle_t GPIOD_Handle;

	GPIOD_Handle.pGPIO = GPIOD;
	GPIOD_Handle.GPIO_pinConfig.pinMode = GPIO_MODE_ALT;
	GPIOD_Handle.GPIO_pinConfig.pinAltFuncMode = GPIO_AF7;
	GPIOD_Handle.GPIO_pinConfig.pinOpType = GPIO_OUT_TYPE_PP;
	GPIOD_Handle.GPIO_pinConfig.pinPuPdCtrl = GPIO_NO_PUPD;
	GPIOD_Handle.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_HIGH;

	// TX pin
	GPIOD_Handle.GPIO_pinConfig.pinNumber = GPIO_PIN_5;
	GPIO_Init(&GPIOD_Handle);

	// RX pin
	GPIOD_Handle.GPIO_pinConfig.pinNumber = GPIO_PIN_6;
	GPIO_Init(&GPIOD_Handle);
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

void delay(void)
{
	for (uint32_t i = 0;i < 500000/2; i++);
}
