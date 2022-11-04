/*
 * 015UART_rx_test.c
 *
 *  Created on: Oct 3, 2022
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

uint8_t rx_cmplt = RESET;

char *msg[3] = {"capitalize this", " that one too", "aaaaaaand this one"};

char rcv_buffer[1024];

int main(void)
{

	uint32_t cnt = 0;

	// configure the USER button
	Btn_Cfg();

	// configure USART2 signal pins
	USART2_GPIOD_Cfg();

	// configure USART2
	USART2_Cfg();

	// configure USART2 IRQ line
	USART_IRQConfig(USART2_IRQ_NO, ENABLE);

	// enable the peripehral (UE = 1)
	USART_PControl(USART2, ENABLE);

	printf("Application is running\n");

	while (1)
	{
		while (! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13));

		delay();

		// next message index
		cnt %= 3; // making sure index doesn't cross 2

		// enable reception interrupt mode (RXNEIE = 1)
		while (USART_ReceiveDataIT(&USART2_Handle, (uint8_t*)rcv_buffer, strlen(msg[cnt])) != USART_STATE_READY);

		// send the message indexed by cnt (polling)
		USART_SendData(&USART2_Handle, (uint8_t*)msg[cnt], strlen(msg[cnt]));

		printf("Transmitted: %s\n", msg[cnt]);

		// wait for all bytes to be received from the arduino, when all bytes are received,
		// rx_cmplt flag will be set in the application call back handler
		while (rx_cmplt != SET);

		rcv_buffer[strlen(msg[cnt]) + 1] = '\0';

		printf("Received: %s\n", rcv_buffer);

		rx_cmplt = RESET;

		cnt++;
	}

	return 0;

}

void USART2_Cfg(void)
{
	USART2_Handle.pUSARTx = USART2;
	USART2_Handle.USARTx_Config.USART_wordLength = USART_WORD_8BITS;
	USART2_Handle.USARTx_Config.USART_no_of_stopBits = USART_STOP_BITS_1;
	USART2_Handle.USARTx_Config.USART_baudRate = USART_STD_BAUDRATE_115200;
	USART2_Handle.USARTx_Config.USART_mode = USART_MODE_TXRX;
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
	GPIOD_Handle.GPIO_pinConfig.pinPuPdCtrl = GPIO_PIN_PUP;
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

void USART2_IRQHandler(void)
{
	USART_InterruptHandling(&USART2_Handle);
}

void USART_ApplicationCallback(USARTx_Handle_t *pHandle, uint8_t appEvent)
{

	if (appEvent == USART_RX_CMPLT)
	{
		rx_cmplt = SET;
	}

	else if (appEvent == USART_TX_CMPLT)
	{
		;
	}
}
