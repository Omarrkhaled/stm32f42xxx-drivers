/*
 * 001LEDtoggling.c
 *
 *  Created on: Sep 16, 2022
 *      Author: omark
 */


#include "stm32f429xx_gpio_driver.h"


int main(void)
{

	GPIOx_Handle_t gpio_led;
	gpio_led.pGPIO = GPIOB;
	gpio_led.GPIO_pinConfig.pinNumber = GPIO_PIN_0;
	gpio_led.GPIO_pinConfig.pinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_HIGH;
	gpio_led.GPIO_pinConfig.pinOpType = GPIO_OUT_TYPE_PP;
	gpio_led.GPIO_pinConfig.pinPuPdCtrl = GPIO_NO_PUPD;

	GPIO_PClockControl(GPIOB, ENABLE);

	GPIO_Init(&gpio_led);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_0);
		for(uint32_t i = 0; i < 1000000; i++);
	}

	return 0;
}
