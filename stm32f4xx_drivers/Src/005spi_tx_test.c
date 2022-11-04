/*
 * 005spi_tx_test.c
 *
 *  Created on: Sep 21, 2022
 *      Author: omark
 */


#include "stm32f429xx.h"
#include <string.h>

/*
 * SPI2_NSS --> PB12
 * SPI2_SCLK  --> PB13
 * SPI2_MISO --> PB14
 * SPI2_MOSI --> PB15
 * Alternate Function Mode: 5
 * */

void SPI_GPIO_Init(void);
void SPI2_Config(void);

int main(void)
{

	char user_data[] = "Hello world";

	// configure GPIO pins to behave as SPI signals
	SPI_GPIO_Init();

	// configure SPI1 peripheral registers
	SPI2_Config();

	// set SSI bit to 1 to tie NSS of the master device/peripheral to +VCC
	SPI_SSIConfig(SPI2, ENABLE);

	// enable SPI2 peripheral
	SPI_PControl(SPI2, ENABLE);

	// send data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	//lets confirm SPI is not busy
	while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

	//Disable the SPI2 peripheral
	SPI_PControl(SPI2,DISABLE);

	while(1);

	return 0;
}


void SPI_GPIO_Init(void)
{

	GPIOx_Handle_t SPI_pins;
	SPI_pins.pGPIO = GPIOB;
	SPI_pins.GPIO_pinConfig.pinMode = GPIO_MODE_ALT;
	SPI_pins.GPIO_pinConfig.pinAltFuncMode = GPIO_AF5;
	SPI_pins.GPIO_pinConfig.pinOpType = GPIO_OUT_TYPE_PP;
	SPI_pins.GPIO_pinConfig.pinPuPdCtrl = GPIO_NO_PUPD;
	SPI_pins.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_HIGH;

	// configure SCLK
	SPI_pins.GPIO_pinConfig.pinNumber = GPIO_PIN_13;
	GPIO_Init(&SPI_pins);

	// configure NSS
	//SPI_pins.GPIO_pinConfig.pinNumber = GPIO_PIN_12;
	//GPIO_Init(&SPI_pins);

	// configure MOSI
	SPI_pins.GPIO_pinConfig.pinNumber = GPIO_PIN_15;
	GPIO_Init(&SPI_pins);

	// configure MISO
	//SPI_pins.GPIO_pinConfig.pinNumber = GPIO_PIN_13;
	//GPIO_Init(&SPI_pins);
}


void SPI2_Config(void)
{
	SPI_Handle_t spi_dev;

	// configure the SPI2 peripheral
	spi_dev.pSPIx = SPI2;
	spi_dev.SPI_config.device_mode = SPI_DEV_MODE_MASTER;
	spi_dev.SPI_config.speed = SPI_SCLK_SPEED_DIV2;
	spi_dev.SPI_config.bus_config = SPI_BUS_CONFIG_FD;
	spi_dev.SPI_config.DFF = SPI_DFF_8BITS;
	spi_dev.SPI_config.CPOL = SPI_CPOL_HIGH;
	spi_dev.SPI_config.CPHA = SPI_CPHA_LOW;
	spi_dev.SPI_config.SSM = SPI_SSM_SW; // as there are no slaves in this application

	// initialize SPI2 registers
	SPI_Init(&spi_dev);

}
