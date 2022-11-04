/*
 * 002led_button.c
 *
 *  Created on: Sep 18, 2022
 *      Author: omark
 */


#include "stm32f429xx_gpio_driver.h"

void delay(void);

int main(void)
{

	GPIOx_Handle_t gpio_led, gpio_button;

	gpio_led.pGPIO = GPIOB;
	gpio_led.GPIO_pinConfig.pinNumber = GPIO_PIN_0;
	gpio_led.GPIO_pinConfig.pinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_pinConfig.pinOpType = GPIO_OUT_TYPE_PP;
	gpio_led.GPIO_pinConfig.pinPuPdCtrl = GPIO_NO_PUPD;
	gpio_led.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_HIGH;

	gpio_button.pGPIO = GPIOC;
	gpio_button.GPIO_pinConfig.pinNumber = GPIO_PIN_13;
	gpio_button.GPIO_pinConfig.pinMode = GPIO_MODE_IN;
	gpio_button.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_VHIGH;
	gpio_button.GPIO_pinConfig.pinPuPdCtrl = GPIO_NO_PUPD;

	GPIO_PClockControl(GPIOB, ENABLE);
	GPIO_PClockControl(GPIOC, ENABLE);

	GPIO_Init(&gpio_button);
	GPIO_Init(&gpio_led);

	while (1)
	{
		if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13))
		{
			delay();
			GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_0);
		}
	}


	return 0;
}

void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}
