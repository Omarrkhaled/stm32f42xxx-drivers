/*
 * stm32f429xx_rcc_driver.c
 *
 *  Created on: Oct 2, 2022
 *      Author: omark
 */
#include "stm32f429xx_rcc_driver.h"

uint16_t AHB1_preScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t  APB_preScaler[4] =  {2, 4, 8, 16};


/********************************************************************
 * @fn				- RCC_GetfPClk1Value
 *
 * @brief			- calculates the APB1 bus clock rate
 *
 * @param[in]		- none
 *
 * @return			- APB1 clock rate
 *
 * @note			- none
 *
 * */
uint32_t RCC_GetfPClk1Value(void)
{
	uint32_t fpClk1, SysClk;
	uint16_t ahb_prescaler, temp;
	uint8_t apb_prescaler;

	// for recognizing the system clock source
	uint8_t SWS = ((RCC->CFGR) >> 2) & 0x3;

	if (SWS == 0)
	{
		SysClk = 16000000;
	}
	else if (SWS == 1)
	{
		SysClk = 8000000;
	}

	else if (SWS == 2)
	{
		RCC_GetPLLClkValue();
	}

	temp = (RCC->CFGR >> 4) & 0xF;

	if (temp < 8)
	{
		ahb_prescaler = 1;
	}

	else if (temp >= 8)
	{
		ahb_prescaler = AHB1_preScaler[temp - 8];
	}

	temp = (RCC->CFGR >> 10) & 0x7;

	if (temp < 4)
	{
		apb_prescaler = 1;
	}

	else if (temp >= 4)
	{
		apb_prescaler = APB_preScaler[temp - 4];
	}

	fpClk1 = (SysClk / ahb_prescaler) / apb_prescaler;

	return fpClk1;
}



/********************************************************************
 * @fn				- RCC_GetfPClk2Value
 *
 * @brief			- calculates the APB2 bus clock rate
 *
 * @param[in]		- none
 *
 * @return			- APB2 clock rate value
 *
 * @note			- none
 *
 * */
uint32_t RCC_GetfPClk2Value(void)
{
	uint32_t fpClk2, SysClk;
	uint16_t ahb_prescaler, temp;
	uint8_t  apb_prescaler;

	// for recognizing the system clock source
	uint8_t SWS = ((RCC->CFGR) >> 2) & 0x3;

	if (SWS == 0)
	{
		SysClk = 16000000;
	}
	else if (SWS == 1)
	{
		SysClk = 8000000;
	}

	else if (SWS == 2)
	{
		RCC_GetPLLClkValue();
	}

	temp = (RCC->CFGR >> 4) & 0xF;

	if (temp < 8)
	{
		ahb_prescaler = 1;
	}

	else if (temp >= 8)
	{
		ahb_prescaler = AHB1_preScaler[temp - 8];
	}

	temp = (RCC->CFGR >> 13) & 0x7;

	if (temp < 4)
	{
		apb_prescaler = 1;
	}

	else if (temp >= 4)
	{
		apb_prescaler = APB_preScaler[temp - 4];
	}

	fpClk2 = (SysClk / ahb_prescaler) / apb_prescaler;

	return fpClk2;
}


uint32_t RCC_GetPLLClkValue(void)
{
	return 0;
}
