#include "stm32f429xx.h"
#include <string.h>
#include <stdio.h>

#define BUFFER_MAX			500

volatile char read_byte;

char rcv_buffer[BUFFER_MAX];

// flag for checking whether the whole string is received or still / or buffer length is full
uint8_t data_available = 0;

// flag for each received byte
volatile uint8_t rcv_stop = 0;

SPI_Handle_t SPI1_handle;

/*
 * PA4 --> SPI1_NSS
 * PA5 --> SPI1_SCLK
 * PA6 -> SPI1_MISO
 * PA7 --> SPI1_MOSI
 * ALT function mode : 5
 */

void delay(void);
void SPI1_GPIOA_Config(void);
void SPI1_Config(void);
void GPIO_IT_PinConfig(void);

int main(void)
{

	uint8_t dummy_write = 0xff;

	// initialize the configured GPIOG interrupt pin
	GPIO_IT_PinConfig();

	// initialize SPI1 GPIOA pins
	SPI1_GPIOA_Config();

	// initialize SPI1 peripheral parameters
	SPI1_Config();

	// enable SSOE control bit .. which makes NSS pulled to low when SPE = 1 (peripheral enabled)
	// and pulled to high when SPE = 0 (peripheral disabled)
	SPI_SSOEConfig(SPI1, ENABLE);

	// enable IRQ line of SPI1
	SPI_InterruptConfig(NVIC_IRQ_NO_SPI1, ENABLE);


	while(1)
	{
		// reset the rcv flag for new data (bytes)
		rcv_stop = 0;

		// hang until there is data to receive
		while(!data_available);

		// disable the IRQ line of GPIO interrupt pin
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI2, DISABLE);

		// enable SPI1 peripheral
		SPI_PControl(SPI1, ENABLE);

		// now , fetch the data from SPI1 byte by byte
		while (!rcv_stop)
		{
			while(SPI_SendDataIT(&SPI1_handle, &dummy_write, 1) == SPI_TX_BSY_STATE);
			while(SPI_ReceiveDataIT(&SPI1_handle, (uint8_t*)&read_byte, 1) == SPI_RX_BSY_STATE);
		}

		// hang if SPI1 peripheral is busy, before disabling it
		while (SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG));

		// disable SPI1
		SPI_PControl(SPI1, DISABLE);

		// print the received message
		printf("Rcvd data: %s\n", rcv_buffer);

		// reset this flag to hang in the above while loop again
		data_available = 0;

		// enable slave interrupt again
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI2, ENABLE);
	}

	return 0;
}

void delay(void)
{
	for (uint32_t i =0; i < 500000; i++);
}

void SPI1_GPIOA_Config(void)
{
	GPIOx_Handle_t GPIOA_handle;

	GPIOA_handle.pGPIO = GPIOA;
	GPIOA_handle.GPIO_pinConfig.pinMode = GPIO_MODE_ALT;
	GPIOA_handle.GPIO_pinConfig.pinAltFuncMode = GPIO_AF5;
	GPIOA_handle.GPIO_pinConfig.pinOpType = GPIO_OUT_TYPE_PP;
	GPIOA_handle.GPIO_pinConfig.pinPuPdCtrl = GPIO_NO_PUPD;
	GPIOA_handle.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_HIGH;

	// NSS
	GPIOA_handle.GPIO_pinConfig.pinNumber = GPIO_PIN_4;
	GPIO_Init(&GPIOA_handle);

	// SCLK
	GPIOA_handle.GPIO_pinConfig.pinNumber = GPIO_PIN_5;
	GPIO_Init(&GPIOA_handle);

	// MISO
	GPIOA_handle.GPIO_pinConfig.pinNumber = GPIO_PIN_6;
	GPIO_Init(&GPIOA_handle);

	// MOSI
	GPIOA_handle.GPIO_pinConfig.pinNumber = GPIO_PIN_7;
	GPIO_Init(&GPIOA_handle);
}

void SPI1_Config(void)
{
	SPI1_handle.pSPIx = SPI1;
	SPI1_handle.SPI_config.device_mode = SPI_DEV_MODE_MASTER;
	SPI1_handle.SPI_config.bus_config = SPI_BUS_CONFIG_FD;
	SPI1_handle.SPI_config.DFF = SPI_DFF_8BITS;
	SPI1_handle.SPI_config.SSM = SPI_SSM_HW;
	SPI1_handle.SPI_config.speed = SPI_SCLK_SPEED_DIV32;
	SPI1_handle.SPI_config.CPOL = SPI_CPOL_LOW;
	SPI1_handle.SPI_config.CPHA = SPI_CPHA_LOW;

	SPI_Init(&SPI1_handle);
}

void GPIO_IT_PinConfig(void)
{
	GPIOx_Handle_t GPIOG_handle;

	GPIOG_handle.pGPIO = GPIOG;
	GPIOG_handle.GPIO_pinConfig.pinMode = GPIO_MODE_IT_FT;
	GPIOG_handle.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_LOW;
	GPIOG_handle.GPIO_pinConfig.pinNumber = GPIO_PIN_2;
	GPIOG_handle.GPIO_pinConfig.pinOpType = GPIO_OUT_TYPE_PP;
	GPIOG_handle.GPIO_pinConfig.pinPuPdCtrl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOG_handle);

	// slave interrupt configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI2, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI2, ENABLE);
}

void EXTI2_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_2);

	// if control reached here, that means new data has arrived,
	// so, set the data_available flag to break the while loop
	data_available = 1;
}

void SPI1_IRQHandler(void)
{
	SPI_InterruptHanling(&SPI1_handle);
}

void SPI_ApplicationCallback(SPI_Handle_t *pSPI_Handle, uint8_t app_event)
{
	static uint32_t i = 0;

	if (app_event == SPI_EVENT_RX_CMPLT)
	{
		rcv_buffer[i++] = read_byte;

		if (read_byte == '\0' || i == BUFFER_MAX)
		{
			rcv_stop = 1;
			rcv_buffer[i-1] = '\0';
			i = 0;
		}
	}
}
