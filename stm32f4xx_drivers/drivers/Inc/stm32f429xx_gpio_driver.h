/*
 * stm32f429xx_gpio_driver.h
 *
 *  Created on: Sep 15, 2022
 *      Author: omark
 */

#ifndef INC_STM32F429XX_GPIO_DRIVER_H_
#define INC_STM32F429XX_GPIO_DRIVER_H_

#include "stm32f429xx.h" // MCU header file

typedef struct
{
	uint8_t pinNumber;			/*!< possible pin numbers at @GPIO_PIN_NUMBERS>*/
	uint8_t pinMode;			/*!< possible values from @@GPIO_Pin_Modes>*/
	uint8_t pinSpeed;			/*!< possible values from @GPIO_OUT_TYPE>*/
	uint8_t pinPuPdCtrl;		/*!< possible values from @GPIO_OUT_SPEED>*/
	uint8_t pinOpType;			/*!< possible values from @GPIO_PUPPD>*/
	uint8_t pinAltFuncMode;		/*!< possible values from @GPIO_AF>*/

} GPIOx_PinConfig_t;

typedef struct
{
	GPIOx_t *pGPIO;
	GPIOx_PinConfig_t GPIO_pinConfig;

} GPIOx_Handle_t;

/*
 * @GPIO_Pin_Modes
 * GPIO pin possible modes
 * */
#define GPIO_MODE_IN				0
#define GPIO_MODE_OUT 				1
#define GPIO_MODE_ALT				2
#define GPIO_MODE_ANALOG			3
#define GPIO_MODE_IT_FT				4
#define GPIO_MODE_IT_RT				5
#define GPIO_MODE_IT_RFT			6

/*
 * @GPIO_OUT_TYPE
 * GPIO output possible modes
 * */
#define GPIO_OUT_TYPE_PP			0
#define GPIO_OUT_TYPE_OD			1

/*
 * @GPIO_OUT_SPEED
 * GPIO output possible speeds
 * */
#define GPIO_OUT_SPEED_LOW			0
#define GPIO_OUT_SPEED_MED			1
#define GPIO_OUT_SPEED_HIGH			2
#define GPIO_OUT_SPEED_VHIGH		3

/*
 * @GPIO_PUPPD
 * GPIO pull up or pull down enable/disable
 * */
#define GPIO_NO_PUPD				0
#define GPIO_PIN_PUP				1
#define GPIO_PIN_PD					2

/*
 * @GPIO_AF
 * GPIO alternate function possible modes
 * */
#define GPIO_AF0					0
#define GPIO_AF1					1
#define GPIO_AF2					2
#define GPIO_AF3					3
#define GPIO_AF4					4
#define GPIO_AF5					5
#define GPIO_AF6					6
#define GPIO_AF7					7
#define GPIO_AF8					8
#define GPIO_AF9					9
#define GPIO_AF10					10
#define GPIO_AF11					11
#define GPIO_AF12					12
#define GPIO_AF13					13
#define GPIO_AF14					14
#define GPIO_AF15					15

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 * */
#define GPIO_PIN_0					0
#define GPIO_PIN_1					1
#define GPIO_PIN_2					2
#define GPIO_PIN_3					3
#define GPIO_PIN_4					4
#define GPIO_PIN_5					5
#define GPIO_PIN_6					6
#define GPIO_PIN_7					7
#define GPIO_PIN_8					8
#define GPIO_PIN_9					9
#define GPIO_PIN_10					10
#define GPIO_PIN_11					11
#define GPIO_PIN_12					12
#define GPIO_PIN_13					13
#define GPIO_PIN_14					14
#define GPIO_PIN_15					15


/****************************************************************************
 * 						APIs supported by this driver
 *
 ****************************************************************************/
/*
 * GPIO Clock Control
 * */
void GPIO_PClockControl(GPIOx_t *pGPIOx, uint8_t en_di);

/*
 * GPIO Init and De-Init
 * */
void GPIO_Init(GPIOx_Handle_t *pGPIOxHandle);
void GPIO_DeInit(GPIOx_t *pGPIOx);

/*
 * GPIO Data Reading
 * */
uint8_t GPIO_ReadFromInputPin(GPIOx_t *pGPIOx, uint8_t pin_number);
uint16_t GPIO_ReadFromInputPort(GPIOx_t *pGPIOx);

/*
 * GPIO Data Writing
 * */
void GPIO_WriteToOutputPin(GPIOx_t *pGPIOx, uint8_t pin_number, uint8_t val);
void GPIO_WriteToOutputPort(GPIOx_t *pGPIOx, uint16_t val);
void GPIO_ToggleOutputPin(GPIOx_t *pGPIOx, uint8_t pin_number);

/*
 * GPIO IRQs Handling
 * */
void GPIO_IRQInterruptConfig(uint8_t IRQ_number, uint8_t en_di);
void GPIO_IRQPriorityConfig(uint8_t IRQ_number, uint32_t IRQ_prio);
void GPIO_IRQHandling(uint8_t pin_number);

#endif /* INC_STM32F429XX_GPIO_DRIVER_H_ */
