#include "stm32f429xx.h"
#include <stdio.h>

#define CMD_LED_CTRL			0X50
#define CMD_SENSOR_READ			0X51
#define CMD_LED_READ			0X52
#define CMD_PRINT				0X53
#define CMD_ID_READ				0X54

#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4
#define ANALOG_PIN5				5

#define LED_ON					1
#define LED_OFF					0

#define LED_PIN					9

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
uint8_t SPI_VerifyResponse(uint8_t ack_byte);

extern void initialise_monitor_handles();

int main(void)
{

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	initialise_monitor_handles();

	printf("Application is running\n");

	// configure user button
	configure_button();

	// configure SPI_gpio pins
	SPI_GPIO_Config();

	// configure SPI1 peripheral control register 1
	SPI1_Config();

	printf("SPI1 initialized\n");

	// enable SSOE pin so when peripheral SPI1 is enabled, NSS is pulled to low
	// and when SPI1 is disabled, it's pulled to high (automatically managed by HW)
	SPI_SSOEConfig(SPI1, ENABLE);

	while (1)
	{

		uint8_t command_code = CMD_LED_CTRL;
		uint8_t ack_byte;
		uint8_t args[2];

		// check if button is pressed
		while (!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13));

		// delay for avoiding de-bouncing
		delay();

		// enable SPI1 peripheral
		SPI_PControl(SPI1, ENABLE);

		// send the first command
		// 1. CMD_LED_CTRL
		SPI_SendData(SPI1, &command_code, 1);

		// do a dummy read to clear off RXNE
		SPI_ReceiveData(SPI1, &dummy_read, 1);

		// get the ack_byte out of slave shift register by sending a dummy byte to it
		SPI_SendData(SPI1, &dummy_write, 1);

		// receive the data which arrive at the master RX buffer
		SPI_ReceiveData(SPI1, &ack_byte, 1);

		// check if ACK or NACK
		if (SPI_VerifyResponse(ack_byte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;

			// send command arguments
			SPI_SendData(SPI1, args, 2);

			printf("CMD_LED_CTRL executed\n");
		}

		// send the second command
		// CMD_SENSOR_READ <ANALOG_PIN_NUMBER>

		// hang until the button is pressed
		while (! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13));

		// delay
		delay();

		command_code = CMD_SENSOR_READ;

		// send the command
		SPI_SendData(SPI1, &command_code, 1);

		// get rid of the received garbage byte
		SPI_ReceiveData(SPI1, &dummy_read, 1);

		// get ACK/NACK byte out of slave SR
		SPI_SendData(SPI1, &dummy_write, 1);

		// read the returned byte from slave in ack_byte
		SPI_ReceiveData(SPI1, &ack_byte, 1);

		// check if it's NACK/ACK
		if(SPI_VerifyResponse(ack_byte))
		{
			// read analog pin no.0
			args[0] = ANALOG_PIN0;

			// send the command argument
			SPI_SendData(SPI1, args, 1);

			// clear off RXNE flag / RX buffer
			SPI_ReceiveData(SPI1, &dummy_read, 1);

			// insert some delay so that slave can be ready to send the analog data as
			// it does ADC on it
			delay();

			// transmit a dummy byte to receive the response
			SPI_SendData(SPI1, &dummy_write, 1);

			uint8_t analog_read;

			// receive the data
			SPI_ReceiveData(SPI1, &analog_read, 1);

			printf("CMD_SENSOR_READ %d\n", analog_read);

		}

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
	GPIOA_config.GPIO_pinConfig.pinNumber = GPIO_PIN_6;
	GPIO_Init(&GPIOA_config);

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

uint8_t SPI_VerifyResponse(uint8_t ack_byte)
{
	if (ack_byte == (uint8_t)0xF5)
		return 1;
	else
		return 0;
}
