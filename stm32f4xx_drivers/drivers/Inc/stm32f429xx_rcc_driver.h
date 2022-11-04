/*
 * stm32f429xx_rcc_driver.h
 *
 *  Created on: Oct 2, 2022
 *      Author: omark
 */

#ifndef INC_STM32F429XX_RCC_DRIVER_H_
#define INC_STM32F429XX_RCC_DRIVER_H_

#include "stm32f429xx.h"


uint32_t RCC_GetfPClk1Value(void);
uint32_t RCC_GetPLLClkValue(void);
uint32_t RCC_GetfPClk2Value(void);


#endif /* INC_STM32F429XX_RCC_DRIVER_H_ */
