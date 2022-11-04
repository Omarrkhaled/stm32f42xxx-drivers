/*
 * 006SPI_st_master_arduino_slave.c
 *
 *  Created on: Sep 22, 2022
 *      Author: omark
 */


#include "stm32f429xx.h"
#include <string.h>

/*
 * SPI Peripheral: SPI1
 * SCLK : PA5
 * MOSI : PA7
 * MISO : PA6
 * NSS  : PA4
 * */
void delay(void);
void SPI_GPIO_Config(void);
void SPI1_Config(void);
void configure_button(void);

int main(void)
{

	char user_data[] = "I wish i can finish my courses ASAP";

	// configure user button
	configure_button();

	// configure SPI_gpio pins
	SPI_GPIO_Config();

	// configure SPI1 peripheral control register 1
	SPI1_Config();

	// enable SSOE pin so when peripheral SPI1 is enabled, NSS is pulled to low
	// and when SPI1 is disabled, it's pulled to high (automatically managed by HW)
	SPI_SSOEConfig(SPI1, ENABLE);

	while (1)
	{
		// check if button is pressed
		while (!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13));

		// delay for avoiding de-bouncing
		delay();

		// enable SPI1 peripheral
		SPI_PControl(SPI1, ENABLE);

		// send data length information
		uint8_t data_length = strlen(user_data);
		SPI_SendData(SPI1, &data_length, 1);

		// send data
		SPI_SendData(SPI1, (uint8_t*)user_data , strlen(user_data));

		// check whether SPI1 is still busy or not
		while (SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG));

		// disable SPI1 after communication is done
		SPI_PControl(SPI1, DISABLE);
	}
}

void SPI_GPIO_Config(void)
{
	GPIOx_Handle_t GPIOA_config;

	GPIOA_config.pGPIO = GPIOA;
	GPIOA_config.GPIO_pinConfig.pinMode = GPIO_MODE_ALT;
	GPIOA_config.GPIO_pinConfig.pinAltFuncMode = GPIO_AF5;
	GPIOA_config.GPIO_pinConfig.pinOpType = GPIO_OUT_TYPE_PP;
	GPIOA_config.GPIO_pinConfig.pinPuPdCtrl = GPIO_NO_PUPD;
	GPIOA_config.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_VHIGH;

	// Configure SCLK signal pin
	GPIOA_config.GPIO_pinConfig.pinNumber = GPIO_PIN_5;
	GPIO_Init(&GPIOA_config);

	// Configure MOSI
	GPIOA_config.GPIO_pinConfig.pinNumber = GPIO_PIN_7;
	GPIO_Init(&GPIOA_config);

	// Configure MISO
	//GPIOA_config.GPIO_pinConfig.pinNumber = GPIO_PIN_6;
	//GPIO_Init(&GPIOA_config);

	// Configure NSS
	GPIOA_config.GPIO_pinConfig.pinNumber = GPIO_PIN_4;
	GPIO_Init(&GPIOA_config);

}

void SPI1_Config(void)
{
	SPI_Handle_t SPI1_master_config;

	SPI1_master_config.pSPIx = SPI1;
	SPI1_master_config.SPI_config.device_mode = SPI_DEV_MODE_MASTER;
	SPI1_master_config.SPI_config.DFF = SPI_DFF_8BITS;
	SPI1_master_config.SPI_config.speed = SPI_SCLK_SPEED_DIV8;
	SPI1_master_config.SPI_config.bus_config = SPI_BUS_CONFIG_FD;
	SPI1_master_config.SPI_config.SSM = SPI_SSM_HW;
	SPI1_master_config.SPI_config.CPOL = SPI_CPOL_LOW;
	SPI1_master_config.SPI_config.CPHA = SPI_CPHA_LOW;

	SPI_Init(&SPI1_master_config);
}

void configure_button(void)
{
	GPIOx_Handle_t gpio_btn;

	gpio_btn.pGPIO = GPIOC;
	gpio_btn.GPIO_pinConfig.pinMode = GPIO_MODE_IN;
	gpio_btn.GPIO_pinConfig.pinNumber = GPIO_PIN_13;
	gpio_btn.GPIO_pinConfig.pinOpType = GPIO_OUT_TYPE_PP;
	gpio_btn.GPIO_pinConfig.pinPuPdCtrl = GPIO_NO_PUPD;
	gpio_btn.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_VHIGH;

	// enable port C, initialize it
	GPIO_Init(&gpio_btn);
}

void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}
