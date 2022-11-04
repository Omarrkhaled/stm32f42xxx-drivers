/*
 * 003led_ext_button.c
 *
 *  Created on: Sep 18, 2022
 *      Author: omark
 */


#include "stm32f429xx_gpio_driver.h"

#define HIGH			1
#define LOW				0
#define BTN_PRESSED 	LOW

void delay(void);

int main(void)
{

	GPIOx_Handle_t gpio_btn, gpio_led;

	// button configuration / input pin.
	gpio_btn.pGPIO = GPIOG;
	gpio_btn.GPIO_pinConfig.pinNumber = GPIO_PIN_2;
	gpio_btn.GPIO_pinConfig.pinMode = GPIO_MODE_IN;
	gpio_btn.GPIO_pinConfig.pinPuPdCtrl = GPIO_PIN_PUP;
	gpio_btn.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_HIGH;

	// LED configuration / output pin
	gpio_led.pGPIO = GPIOG;
	gpio_led.GPIO_pinConfig.pinNumber = GPIO_PIN_3;
	gpio_led.GPIO_pinConfig.pinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_pinConfig.pinOpType = GPIO_OUT_TYPE_PP;
	gpio_led.GPIO_pinConfig.pinPuPdCtrl = GPIO_NO_PUPD;
	gpio_led.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_HIGH;

	GPIO_PClockControl(GPIOG, ENABLE);

	GPIO_Init(&gpio_btn);
	GPIO_Init(&gpio_led);

	while (1)
	{
		if (GPIO_ReadFromInputPin(GPIOG, GPIO_PIN_2) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_3);
		}
	}


return 0;
}

void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}
