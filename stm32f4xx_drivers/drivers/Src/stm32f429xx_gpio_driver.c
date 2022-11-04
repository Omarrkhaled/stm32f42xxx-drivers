/*
 * stm32f429xx_gpio_driver.c
 *
 *  Created on: Sep 15, 2022
 *      Author: omark
 */


#include "stm32f429xx_gpio_driver.h" // driver header file

/********************************************************************
 * @fn				- GPIO_PClockControl
 *
 * @brief			- Enables/ Disables the peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the given GPIO peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void GPIO_PClockControl(GPIOx_t *pGPIOx, uint8_t en_di)
{
	if (en_di == ENABLE)
	{
		if (pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if (pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if (pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if (pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if (pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if (pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
		else if (pGPIOx == GPIOG)
			GPIOG_PCLK_EN();
		else if (pGPIOx == GPIOH)
			GPIOH_PCLK_EN();
		else if (pGPIOx == GPIOI)
			GPIOI_PCLK_EN();
		else if (pGPIOx == GPIOK)
			GPIOK_PCLK_EN();
	}
	else
	{
		if (pGPIOx == GPIOA)
			GPIOA_PCLK_DI();
		else if (pGPIOx == GPIOB)
			GPIOB_PCLK_DI();
		else if (pGPIOx == GPIOC)
			GPIOC_PCLK_DI();
		else if (pGPIOx == GPIOD)
			GPIOD_PCLK_DI();
		else if (pGPIOx == GPIOE)
			GPIOE_PCLK_DI();
		else if (pGPIOx == GPIOF)
			GPIOF_PCLK_DI();
		else if (pGPIOx == GPIOG)
			GPIOG_PCLK_DI();
		else if (pGPIOx == GPIOH)
			GPIOH_PCLK_DI();
		else if (pGPIOx == GPIOI)
			GPIOI_PCLK_DI();
		else if (pGPIOx == GPIOK)
			GPIOK_PCLK_DI();
	}
}

/********************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- Initialize the given GPIO peripheral registers after enabling it's
 * 					- clock
 *
 * @param[in]		- GPIO handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void GPIO_Init(GPIOx_Handle_t *pGPIOxHandle)
{
	// enable the GPIO peripheral clock
	GPIO_PClockControl(pGPIOxHandle->pGPIO, ENABLE);

	uint32_t temp = 0;

	//1. configure the mode of the GPIO pin
	if (pGPIOxHandle->GPIO_pinConfig.pinMode <= GPIO_MODE_ANALOG)
	{
		// (non-interrupt mode)
	temp = pGPIOxHandle->GPIO_pinConfig.pinMode << (2*pGPIOxHandle->GPIO_pinConfig.pinNumber);
	pGPIOxHandle->pGPIO->MODER &= ~(3 << 2*pGPIOxHandle->GPIO_pinConfig.pinNumber);
	pGPIOxHandle->pGPIO->MODER |= temp;
	}
	else
	{
		// (interrupt mode)
		if (pGPIOxHandle->GPIO_pinConfig.pinMode == GPIO_MODE_IT_FT)
		{
			// configure the FTSR
			EXTI->EXTI_FTSR |=  (1 << pGPIOxHandle->GPIO_pinConfig.pinNumber);
			EXTI->EXTI_RTSR &= ~(1 << pGPIOxHandle->GPIO_pinConfig.pinNumber);
		}
		else if (pGPIOxHandle->GPIO_pinConfig.pinMode == GPIO_MODE_IT_RT)
		{
			// configure the RTSR
			EXTI->EXTI_RTSR |=  (1 << pGPIOxHandle->GPIO_pinConfig.pinNumber);
			EXTI->EXTI_FTSR &= ~(1 << pGPIOxHandle->GPIO_pinConfig.pinNumber);
		}
		else if (pGPIOxHandle->GPIO_pinConfig.pinMode == GPIO_MODE_IT_RFT)
		{
			// configure both FTSR and RSTR registers
			EXTI->EXTI_RTSR |=  (1 << pGPIOxHandle->GPIO_pinConfig.pinNumber);
			EXTI->EXTI_FTSR |= ~(1 << pGPIOxHandle->GPIO_pinConfig.pinNumber);
		}

		// configure the GPIO port selection
		uint8_t temp1 = pGPIOxHandle->GPIO_pinConfig.pinNumber / 4;
		uint8_t temp2 = pGPIOxHandle->GPIO_pinConfig.pinNumber % 4;
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= (GPIO_BASEADDR_TO_CODE(pGPIOxHandle->pGPIO) << temp2 * 4);

		// enable EXTI interrupt delivery through IMR (interrupt mask register)
		EXTI->EXTI_IMR |= (1 << pGPIOxHandle->GPIO_pinConfig.pinNumber);

	}

	//2. configure the speed
	temp = pGPIOxHandle->GPIO_pinConfig.pinSpeed << (2*pGPIOxHandle->GPIO_pinConfig.pinNumber);
	pGPIOxHandle->pGPIO->OSPEEDR &= ~(3 << 2*pGPIOxHandle->GPIO_pinConfig.pinNumber);
	pGPIOxHandle->pGPIO->OSPEEDR |= temp;

	//3. configure the PUPD settings
	temp = (pGPIOxHandle->GPIO_pinConfig.pinPuPdCtrl << ( 2 * pGPIOxHandle->GPIO_pinConfig.pinNumber) );
	pGPIOxHandle->pGPIO->PUPDR &= ~( 0x3 << ( 2 * pGPIOxHandle->GPIO_pinConfig.pinNumber)); //clearing
	pGPIOxHandle->pGPIO->PUPDR |= temp;


	//4. configure the OPtype
		temp = pGPIOxHandle->GPIO_pinConfig.pinOpType << (pGPIOxHandle->GPIO_pinConfig.pinNumber);
		pGPIOxHandle->pGPIO->OTYPER &= ~(1 << pGPIOxHandle->GPIO_pinConfig.pinNumber);
		pGPIOxHandle->pGPIO->OTYPER |= temp;

	//5. configure the alternative functionality
	if (pGPIOxHandle->GPIO_pinConfig.pinMode == GPIO_MODE_ALT)
	{
		uint8_t temp1 = pGPIOxHandle->GPIO_pinConfig.pinNumber / 8;
		uint8_t temp2 = pGPIOxHandle->GPIO_pinConfig.pinNumber % 8;
		pGPIOxHandle->pGPIO->AFR[temp1] &= ~(0xF << (4*temp2));
		pGPIOxHandle->pGPIO->AFR[temp1] |= (pGPIOxHandle->GPIO_pinConfig.pinAltFuncMode << (4*temp2));

	}

}

/********************************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			- resets all GPIO peripheral registers to it's default/reset value
 *
 * @param[in]		- GPIO base address
 *
 * @return			- none
 *
 * @note			- none
 *
 * */

void GPIO_DeInit(GPIOx_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
		GPIOA_REG_RESET();
	else if (pGPIOx == GPIOB)
		GPIOB_REG_RESET();
	else if (pGPIOx == GPIOC)
		GPIOC_REG_RESET();
	else if (pGPIOx == GPIOD)
		GPIOD_REG_RESET();
	else if (pGPIOx == GPIOE)
		GPIOE_REG_RESET();
	else if (pGPIOx == GPIOF)
		GPIOF_REG_RESET();
	else if (pGPIOx == GPIOG)
		GPIOG_REG_RESET();
	else if (pGPIOx == GPIOH)
		GPIOH_REG_RESET();
	else if (pGPIOx == GPIOI)
		GPIOI_REG_RESET();
	else if (pGPIOx == GPIOK)
		GPIOK_REG_RESET();
}

/********************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- reads the given input pin of the given GPIO port
 *
 * @param[in]		- base address of the given GPIO
 * @param[in]		- input pin number
 *
 * @return			- 0 or 1
 *
 * @note			- none
 *
 * */
uint8_t GPIO_ReadFromInputPin(GPIOx_t *pGPIOx, uint8_t pin_number)
{
	uint8_t val;
	val = ((pGPIOx->IDR >> pin_number) & 0x00000001);
	return val;
}

/********************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- reads the GPIO peripheral port
 *
 * @param[in]		- base address of the GPIO
 * @param[in]		-
 *
 * @return			- data of the given port input data register
 *
 * @note			- none
 *
 * */
uint16_t GPIO_ReadFromInputPort(GPIOx_t *pGPIOx)
{
	uint16_t val;
	val = (uint16_t) pGPIOx->IDR;
	return val;
}

/********************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			- writes the given data to an output pin
 *
 * @param[in]		- base address of the GPIO
 * @param[in]		- number of the output data pin
 * @param[in]		- value to be written to the pin
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void GPIO_WriteToOutputPin(GPIOx_t *pGPIOx, uint8_t pin_number, uint8_t val)
{
	if (val == GPIO_PIN_SET)
	{
		// write 1 to the corresponding bit field in ODR register
		pGPIOx->ODR |= (1 << pin_number);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << pin_number);
		// write 0 to the corresponding bit field in ODR register
	}
}

/********************************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- writes the given data to the GPIO port
 *
 * @param[in]		- base address of the GPIO
 * @param[in]		- value to be written to the port
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void GPIO_WriteToOutputPort(GPIOx_t *pGPIOx, uint16_t val)
{
	pGPIOx->ODR |= val;
}

/********************************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			- toggles the data of the given output data pin
 *
 * @param[in]		- base address of the GPIO
 * @param[in]		- output data pin number
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void GPIO_ToggleOutputPin(GPIOx_t *pGPIOx, uint8_t pin_number)
{
	pGPIOx->ODR ^= (1 << pin_number);
}

/********************************************************************
 * @fn				- GPIO_IRQInterruptConfig
 *
 * @brief			- enable/disable the given IRQ
 *
 * @param[in]		- number of the IRQ
 * @param[in]		- ENABLE/DISABLE macro
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void GPIO_IRQInterruptConfig(uint8_t IRQ_number, uint8_t en_di)
{
	if (en_di == ENABLE)
	{
		if (IRQ_number <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQ_number);
		}

		else if (IRQ_number > 31 && IRQ_number < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQ_number % 32));
		}

		else if (IRQ_number >= 64 && IRQ_number < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQ_number % 64));
		}
	}

	else
	{
		if (IRQ_number <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQ_number);
		}
		else if (IRQ_number > 31 && IRQ_number < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQ_number % 32));
		}

		else if (IRQ_number >= 64 && IRQ_number < 96)
		{
			*NVIC_ICER2 |= (1 << IRQ_number % 64);
		}
	}
}

/********************************************************************
 * @fn				- GPIO_IRQPriorityConfig
 *
 * @brief			- configure interrupt priority
 *
 * @param[in]		- number of the IRQ
 * @param[in]		- priority of the IRQ
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void GPIO_IRQPriorityConfig(uint8_t IRQ_number, uint32_t IRQ_prio)
{
	// which register of the 60 IPR's
	uint8_t iprx = IRQ_number / 4;

	// which section of the selected IPRx register
	uint8_t iprx_section = IRQ_number % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_IMPLEMENTED_PRIO_BITS);

	*(NVIC_IPR_BASEADDR + iprx) |= (IRQ_prio << shift_amount);
}

/********************************************************************
 * @fn				- GPIO_IRQHandling
 *
 * @brief			- clears the pending bit of the pin that triggered the interrupt
 *
 * @param[in]		- pin number
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void GPIO_IRQHandling(uint8_t pin_number)
{
	if (EXTI->EXTI_PR & (1 << pin_number))
	{
		EXTI->EXTI_PR |= (1 << pin_number);
	}
}
