/*
 * 004led_button_interrupt.c
 *
 *  Created on: Sep 20, 2022
 *      Author: omark
 */


#include "stm32f429xx_gpio_driver.h"
#include <string.h>

void delay(void);

int main(void)
{

	GPIOx_Handle_t gpio_led, gpio_btn;

	// initialize to 0 to avoid garbage values and bugs
	memset(&gpio_btn, 0, sizeof(gpio_btn));
	memset(&gpio_led, 0, sizeof(gpio_led));

	// button configuration
	gpio_btn.pGPIO = GPIOG;
	gpio_btn.GPIO_pinConfig.pinNumber = GPIO_PIN_2;
	gpio_btn.GPIO_pinConfig.pinMode = GPIO_MODE_IT_FT;
	gpio_btn.GPIO_pinConfig.pinPuPdCtrl = GPIO_PIN_PUP;

	// led configuration
	gpio_led.pGPIO = GPIOG;
	gpio_led.GPIO_pinConfig.pinNumber = GPIO_PIN_3;
	gpio_led.GPIO_pinConfig.pinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_pinConfig.pinOpType = GPIO_OUT_TYPE_PP;
	gpio_led.GPIO_pinConfig.pinSpeed = GPIO_OUT_SPEED_VHIGH;

	GPIO_PClockControl(GPIOG, ENABLE);

	GPIO_Init(&gpio_btn);
	GPIO_Init(&gpio_led);

	// IRQ configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI2, NVIC_IRQ_PRI5);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI2, ENABLE);

	while(1);

}

void EXTI2_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_2);
	GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_3);
}

void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}
