/*
 * stm32f429xx_usart_driver.c
 *
 *  Created on: Oct 1, 2022
 *      Author: omark
 */
#include "stm32f429xx_usart_driver.h"

/* Private Helper Functions */
static void USART_TXHandler(USARTx_Handle_t *pHandle);
static void USART_RXHandler(USARTx_Handle_t *pHandle);

/**********************************************************************************************
 * 								APIs supported by this driver
 **********************************************************************************************/

/*
 * Peripheral control and clock control
 * */

/********************************************************************
 * @fn				- USART_PClockControl
 *
 * @brief			- enables / disables the given USART peripheral clock
 *
 * @param[in]		- base address of the given USART peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void USART_PClockControl(USARTx_t *pUSARTx, uint8_t en_di)
{
	if (en_di == ENABLE)
	{
		if (pUSARTx == USART1)
			USART1_PCLK_EN();
		else if (pUSARTx == USART2)
			USART2_PCLK_EN();
		else if (pUSARTx == USART3)
			USART3_PCLK_EN();
		else if (pUSARTx == UART4)
			UART4_PCLK_EN();
		else if (pUSARTx == UART5)
			UART4_PCLK_EN();
		else if (pUSARTx == USART6)
			USART6_PCLK_EN();
		else if (pUSARTx == UART7)
			UART7_PCLK_EN();
		else if (pUSARTx == UART8)
			UART8_PCLK_EN();
	}

	else
	{
		if (pUSARTx == USART1)
			USART1_PCLK_DI();
		else if (pUSARTx == USART2)
			USART2_PCLK_DI();
		else if (pUSARTx == USART3)
			USART3_PCLK_DI();
		else if (pUSARTx == UART4)
			UART4_PCLK_DI();
		else if (pUSARTx == UART5)
			UART4_PCLK_DI();
		else if (pUSARTx == USART6)
			USART6_PCLK_DI();
		else if (pUSARTx == UART7)
			UART7_PCLK_DI();
		else if (pUSARTx == UART8)
			UART8_PCLK_DI();
	}
}

/********************************************************************
 * @fn				- USART_PControl
 *
 * @brief			- enables / disables the given USART peripheral
 *
 * @param[in]		- base address of the given USART peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void USART_PControl(USARTx_t *pUSARTx, uint8_t en_di)
{
	if (en_di == ENABLE)
	{
		pUSARTx->CR[0] |=  (1 << USART_CR1_UE);
	}

	else
	{
		pUSARTx->CR[0] &= ~(1 << USART_CR1_UE);
	}
}

/*
 * Peripheral Initialization and De-initialization
 * */

/********************************************************************
 * @fn				- USART_Init
 *
 * @brief			- initializes USARTx peripheral parameters
 *
 * @param[in]		- USART user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void USART_Init(USARTx_Handle_t *pHandle)
{
	uint32_t temp_reg = 0;

	pHandle->pUSARTx->CR[0] |= (1 << 15);

	// enable the peripheral clock
	USART_PClockControl(pHandle->pUSARTx, ENABLE);

	// CR1 register configuration
	// configure word length
	temp_reg |= (pHandle->USARTx_Config.USART_wordLength << USART_CR1_M);

	// configure USART mode (TX / RX / TX-RX)
	if (pHandle->USARTx_Config.USART_mode == USART_MODE_RX_ONLY)
	{
		temp_reg |= (1 << USART_CR1_RE);
	}
	else if (pHandle->USARTx_Config.USART_mode == USART_MODE_TX_ONLY)
	{
		temp_reg |= (1 << USART_CR1_TE);
	}
	else if (pHandle->USARTx_Config.USART_mode == USART_MODE_TXRX)
	{
		temp_reg |= (( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE));
	}

	// configure parity bit
	if (pHandle->USARTx_Config.USART_parityControl == USART_PARITY_EN_EVEN)
	{
		temp_reg |=  (1 << USART_CR1_PCE);
		temp_reg &= ~(1 << USART_CR1_PS);
	}
	else if (pHandle->USARTx_Config.USART_parityControl == USART_PARITY_EN_ODD)
	{
		temp_reg |=  (1 << USART_CR1_PCE);
		temp_reg |=  (1 << USART_CR1_PS);
	}
	else if (pHandle->USARTx_Config.USART_parityControl == USART_PARITY_DISABLE)
	{
		temp_reg &= ~(1 << USART_CR1_PCE);
	}

	// mask the CR1 register with temp_reg value
	pHandle->pUSARTx->CR[0] |= temp_reg;

	// CR2 register configuration
	// configure the number of stop bits
	temp_reg = pHandle->USARTx_Config.USART_no_of_stopBits << USART_CR2_STOP;
	// mask
	pHandle->pUSARTx->CR[1] |= temp_reg;

	// CR3 register configuration
	temp_reg = 0;
	// configure hardware flow control
	if (pHandle->USARTx_Config.USART_hardwareFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		temp_reg |= (1 << USART_CR3_CTSE);
	}
	else if (pHandle->USARTx_Config.USART_hardwareFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		temp_reg |= (1 << USART_CR3_RTSE);
	}
	else if (pHandle->USARTx_Config.USART_hardwareFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		temp_reg |= (3 << USART_CR3_RTSE);
	}

	// mask
	pHandle->pUSARTx->CR[2] |= temp_reg;

	// BRR register configuration
	// configure the baud rate
	USART_SetBaudrate(pHandle->pUSARTx, pHandle->USARTx_Config.USART_baudRate);
}

/********************************************************************
 * @fn				- USART_DeInit
 *
 * @brief			- resets all of the USARTx peripheral registers
 *
 * @param[in]		- USART peripheral base address
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void USART_DeInit(USARTx_t *pUSARTx)
{
	if (pUSARTx == USART1)
		RCC->APB2RSTR |= (1 << 4);
	else if (pUSARTx == USART6)
		RCC->APB2RSTR |= (1 << 5);
	else if (pUSARTx == USART2)
		RCC->APB1RSTR |= (1 << 17);
	else if (pUSARTx == USART3)
		RCC->APB1RSTR |= (1 << 18);
	else if (pUSARTx == UART4)
		RCC->APB1RSTR |= (1 << 19);
	else if (pUSARTx == UART5)
		RCC->APB1RSTR |= (1 << 20);
	else if (pUSARTx == UART7)
		RCC->APB1RSTR |= (1 << 30);
	else if (pUSARTx == UART8)
		RCC->APB1RSTR |= (1 << 31);
}

/*
 * Data Communication APIs (polling)
 * */

/********************************************************************
 * @fn				- USART_SendData
 *
 * @brief			- transmit data to external world
 *
 * @param[in]		- USART user handle
 * @param[in]		- reference to data transmitting software buffer
 * @param[in]		- data length
 *
 * @return			- none
 *
 * @note			- blocking call
 *
 * */
void USART_SendData(USARTx_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint16_t *pData;

	for (uint32_t i = 0; i < len; i++)
	{
		// block until TXE flag is set
		while (! USART_GetFlagStatus(pHandle->pUSARTx, USART_FLAG_TXE));

		// check data frame format
		if (pHandle->USARTx_Config.USART_wordLength == USART_WORD_9BITS)
		{
			// 9 bits of data will be transmitted
			pData = (uint16_t*)pTxBuffer;
			pHandle->pUSARTx->DR = (*pData & ((uint16_t)0x01ff));

			// check parity selection / control
			if (pHandle->USARTx_Config.USART_parityControl == USART_PARITY_DISABLE)
			{
				// no parity bit will be used, 2 bytes for data
				// increment the TX buffer by 2 bytes
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				// 1 bit will be used to parity, 1 byte for data
				// increment the TX buffer by 1 byte
				// 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			// 8 bits of data will be transmitted
			pHandle->pUSARTx->DR = (*pTxBuffer & ((uint8_t)0xff));
			pTxBuffer++;
		}
	}
	// hang until transmission is completed
	while (! USART_GetFlagStatus(pHandle->pUSARTx, USART_FLAG_TC));
}

/********************************************************************
 * @fn				- USART_ReceiveData
 *
 * @brief			- receive data from the external world
 *
 * @param[in]		- USART user handle
 * @param[in]		- reference to data receiving software buffer
 * @param[in]		- data length
 *
 * @return			- none
 *
 * @note			- blocking call
 *
 * */
void USART_ReceiveData(USARTx_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t len)
{

	for (uint32_t i = 0; i < len; i++)
	{
		// block until RXNE flag is set
		while (! USART_GetFlagStatus(pHandle->pUSARTx, USART_FLAG_RXNE));

		if (pHandle->USARTx_Config.USART_wordLength == USART_WORD_9BITS)
		{
			// 9 bits of data are going to be received
			// check parity
			if (pHandle->USARTx_Config.USART_parityControl == USART_PARITY_DISABLE)
			{
				*((uint16_t*)pRxBuffer) = ((pHandle->pUSARTx->DR) & ((uint16_t)0x01ff));
				// no parity bit is used, increment the buffer by 2 bytes
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				*pRxBuffer = ((pHandle->pUSARTx->DR) & ((uint8_t)0xff));
				// 1 parity bit used, increment the buffer by 1 byte
				pRxBuffer++;
			}
		}
		else
		{
			// 8 bits of data are going to be received
			// check if a bit is used for parity or not
			if (pHandle->USARTx_Config.USART_parityControl == USART_PARITY_DISABLE)
			{
				// 1 byte of data, no parity bit
				*pRxBuffer = (uint8_t) (pHandle->pUSARTx->DR & ((uint8_t)0xff));
			}
			else
			{
				// 7 bits of data, 1 parity bit
				*pRxBuffer = (uint8_t) (pHandle->pUSARTx->DR & ((uint8_t)0x7f));
			}

			pRxBuffer++;
		}
	}
}

/********************************************************************
 * @fn				- USART_SendDataIT
 *
 * @brief			- transmit data to the external world
 *
 * @param[in]		- USART user handle
 * @param[in]		- reference to data receiving software buffer
 * @param[in]		- data length
 *
 * @return			- none
 *
 * @note			- interrupt mode
 *
 * */
uint8_t USART_SendDataIT(USARTx_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pHandle->TxBusyState;

	if (state != USART_STATE_TX_BUSY)
	{
		pHandle->pTxBuffer = pTxBuffer;
		pHandle->TxLen = len;
		pHandle->TxBusyState = USART_STATE_TX_BUSY;

		pHandle->pUSARTx->CR[0] |= (1 << USART_CR1_TXEIE);

		pHandle->pUSARTx->CR[0] |= (1 << USART_CR1_TCIE);
	}

	return state;
}

/********************************************************************
 * @fn				- USART_ReceiveDataIT
 *
 * @brief			- receive data from the external world
 *
 * @param[in]		- USART user handle
 * @param[in]		- reference to data receiving software buffer
 * @param[in]		- data length
 *
 * @return			- none
 *
 * @note			- interrupt mode
 *
 * */
uint8_t USART_ReceiveDataIT(USARTx_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pHandle->RxBusyState;

	if (state != USART_STATE_RX_BUSY)
	{
		pHandle->pRxBuffer = pRxBuffer;
		pHandle->RxLen = len;
		pHandle->RxBusyState = USART_STATE_RX_BUSY;

		(void)pHandle->pUSARTx->DR;

		pHandle->pUSARTx->CR[0] |= (1 << USART_CR1_RXNEIE);

	}

	return state;
}


/*
 * Interrupt configuration and Handling APIs
 * */

/********************************************************************
 * @fn				- USART_IRQPriorityConfig
 *
 * @brief			- configure the interrupt priority
 *
 * @param[in]		- USART IRQ number
 * @param[in]		- priority level
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void USART_IRQPriorityConfig(uint8_t IRQ_number, uint8_t prio)
{
	uint8_t iprx = IRQ_number / 4;
	uint8_t iprx_section = IRQ_number % 4;
	uint8_t samt = (iprx_section * 8) + (8 - NO_IMPLEMENTED_PRIO_BITS);
	*(NVIC_IPR_BASEADDR + iprx) |= (prio << samt);
}

/********************************************************************
 * @fn				- USART_IRQConfig
 *
 * @brief			- enable or disable the IRQ line
 *
 * @param[in]		- USARTx IRQ number
 * @param[in]		- ENABLE / DISABLE
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void USART_IRQConfig(uint8_t IRQ_number, uint8_t en_di)
{
	if (en_di == ENABLE)
	{
		if (IRQ_number <= 31)
		{
			*(NVIC_ISER0) |= (1 << IRQ_number);
		}
		else if ((IRQ_number >= 32) && (IRQ_number < 64))
		{
			*(NVIC_ISER1) |= (1 << IRQ_number % 32);
		}
		else if ((IRQ_number >= 65) && (IRQ_number < 96))
			*(NVIC_ISER2) |= (1 << IRQ_number % 64);
	}

	else
	{
		if (IRQ_number <= 31)
		{
			*(NVIC_ICER0) |= (1 << IRQ_number);
		}
		else if ((IRQ_number > 31) && (IRQ_number < 64))
		{
			*(NVIC_ICER1) |= (1 << IRQ_number % 32);
		}
		else if ((IRQ_number >= 64) && (IRQ_number < 96))
			*(NVIC_ICER2) |= (1 << IRQ_number % 64);
	}
}

/********************************************************************
 * @fn				- USART_InterruptHandling
 *
 * @brief			- decode and handle the triggered interrupt
 *
 * @param[in]		- USARTx user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void USART_InterruptHandling(USARTx_Handle_t *pHandle)
{
	// decode the interrupt (which event has triggered it)
	uint32_t temp1, temp2, temp3;

	// check for completion of transmission interrupt
	temp1 = pHandle->pUSARTx->SR & USART_FLAG_TC;
	temp2 = pHandle->pUSARTx->CR[0] & (1 << USART_CR1_TCIE);
	if (temp1 && temp2)
	{
		// TC = 1, TCIE = 1
		if (pHandle->TxBusyState == USART_STATE_TX_BUSY)
		{
			if (pHandle->TxLen == 0)
			{
				USART_CloseTransmission(pHandle);

				// notify the application
				USART_ApplicationCallback(pHandle, USART_TX_CMPLT);
			}
		}
	}

	// check for transmission interrupt interrupt
	temp1 = pHandle->pUSARTx->SR & USART_FLAG_TXE;
	temp2 = pHandle->pUSARTx->CR[0] & (1 << USART_CR1_TXEIE);
	if (temp1 && temp2)
	{
		// TXE flag is set, TXEIE control bit is enabled
		if (pHandle->TxBusyState == USART_STATE_TX_BUSY)
		{
				if (pHandle->TxLen > 0)
				{
					// send the data (write into TDR register)
					USART_TXHandler(pHandle);
				}
		}
	}

	// check for receiving interrupt
	temp1 = pHandle->pUSARTx->SR & USART_SR_RXNE;
	temp2 = pHandle->pUSARTx->CR[0] & (1 << USART_CR1_RXNEIE);
	if (temp1 && temp2)
	{
		// RXNE = 1, RXNEIE = 1
		if (pHandle->RxBusyState == USART_STATE_RX_BUSY)
		{
			if (pHandle->RxLen > 0)
			{
				// receive the data (read RDR register)
				USART_RXHandler(pHandle);
			}
			if (pHandle->RxLen == 0)
			{
				// end receiving data
				USART_CloseReception(pHandle);

				// notify the application
				USART_ApplicationCallback(pHandle, USART_RX_CMPLT);
			}
		}
	}

	// check for hardware flow control interrupt
	temp1 = pHandle->pUSARTx->SR & USART_SR_CTS;
	temp2 = pHandle->pUSARTx->CR[2] & (1 << USART_CR3_CTSIE);
	temp3 = pHandle->pUSARTx->CR[2] & (1 << USART_CR3_CTSE);
	if (temp1 && temp2 && temp3)
	{
		//CTSE = 1, CTS = 1, CTSIE = 1
		USART_ClearFlag(pHandle->pUSARTx, USART_FLAG_CTS);

		// notify the application
		USART_ApplicationCallback(pHandle, USART_CTS_CMPLT);
	}

	// check for idle state of TX / RX lines
	temp1 = pHandle->pUSARTx->SR & USART_SR_IDLE;
	temp2 = pHandle->pUSARTx->CR[0] & (1 << USART_CR1_IDLEIE);
	if (temp1 && temp2)
	{
		// IDLE = 1, IDLEIE = 1
		pHandle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);

		// notify the application
		USART_ApplicationCallback(pHandle, USART_EV_IDLE);
	}


	/******************************** ERROR INTERRUPTS *********************************/

	if (pHandle->pUSARTx->CR[2] & (1 << USART_CR3_EIE))
	{
		temp1 = pHandle->pUSARTx->SR;
		if (temp1 & USART_FLAG_ORE)
		{
			// over-run error
			USART_ApplicationCallback(pHandle, USART_ER_OVR);
		}

		else if (temp1 & USART_FLAG_NF)
		{
			// noise is detected on a received frame
			USART_ApplicationCallback(pHandle, USART_ER_NE);
		}

		else if (temp1 & USART_FLAG_FE)
		{
			// hardware when a de-synchronization, excessive noise or a break character is detected
			USART_ApplicationCallback(pHandle, USART_ER_FE);
		}

	}
}


/*
 * Other APIs
 * */

/********************************************************************
 * @fn				- USART_CloseTransmission
 *
 * @brief			- end transmission of data
 *
 * @param[in]		- USARTx user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void USART_CloseTransmission(USARTx_Handle_t *pHandle)
{
	pHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);
	pHandle->TxBusyState = USART_STATE_READY;
	pHandle->pTxBuffer = NULL;
	pHandle->TxLen = 0;
}

/********************************************************************
 * @fn				- USART_CloseReception
 *
 * @brief			- end reception of data
 *
 * @param[in]		- USARTx user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void USART_CloseReception(USARTx_Handle_t *pHandle)
{
	// disable interrupt control line
	pHandle->pUSARTx->CR[0] &= ~(1 << USART_CR1_RXNEIE);

	pHandle->RxBusyState = USART_STATE_READY;
}

/********************************************************************
 * @fn				- USART_GetFlagStatus
 *
 * @brief			- check SR specified flag status
 *
 * @param[in]		- USARTx base address
 * @param[in]		- flag name
 *
 * @return			- flag status (SET / RESET)
 *
 * @note			- none
 *
 * */
uint8_t USART_GetFlagStatus(USARTx_t *pUSARTx, uint16_t flag_name)
{
	if (pUSARTx->SR & flag_name)
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}
}

/********************************************************************
 * @fn				- USART_ClearFlag
 *
 * @brief			- clear the sent flag
 *
 * @param[in]		- USARTx base address
 * @param[in]		- flag name
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void USART_ClearFlag(USARTx_t *pUSARTx, uint16_t flag_name)
{
	pUSARTx->SR &= ~(flag_name);
}


/********************************************************************
 * @fn				- USART_SetBaudrate
 *
 * @brief			- configure the desired baud rate into BRR register
 *
 * @param[in]		- USARTx base address
 * @param[in]		- baud rate
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void USART_SetBaudrate(USARTx_t *pUSARTx, uint32_t baudRate)
{

	uint32_t fpclkx;
	uint32_t USART_div, M_part, F_part;
	uint32_t temp_reg = 0;

	// APB2
	if ((pUSARTx == USART1) || (pUSARTx == USART6))
	{
		// here we use fpclk1
		fpclkx = RCC_GetfPClk2Value();
	}
	// APB1
	else
	{
		// here we use fpclk1
		fpclkx = RCC_GetfPClk1Value();
	}

	// check over sampling technique (8 / 16)
	if (pUSARTx->CR[0] & (1 << USART_CR1_OVER8))
	{
		// OVER8 = 1
		USART_div = ((25 * fpclkx) / (2 *baudRate));
	}
	else
	{
		// OVER8 = 0
		USART_div = ((25 * fpclkx) / (4 *baudRate));
	}

	M_part = USART_div / 100;

	// shift the mantissa part to it's position in BRR register
	temp_reg |= (M_part << 4);

	// extract the fraction part
	F_part = USART_div - (M_part * 100);

	if (pUSARTx->CR[0] & (1 << USART_CR1_OVER8))
	{
		// OVER8 = 1
		F_part = (((F_part * 8) + 50) / 100) & ((uint8_t)0x07);
	}
	else
	{
		// OVER8 = 0
		F_part = ((F_part * 16 + 50) / 100) & ((uint8_t)0x0f);
	}

	temp_reg |= (F_part);

	// mask
	pUSARTx->BRR |= temp_reg;
}

__weak void USART_ApplicationCallback(USARTx_Handle_t *pHandle, uint8_t appEvent)
{

}

/*
 * Private Helper Functions
 * */
static void USART_TXHandler(USARTx_Handle_t *pHandle)
{
	uint16_t *pData;

		if (pHandle->pUSARTx->CR[0] & (1 << USART_CR1_M))
		{
			// 9 bit word length
			pData = (uint16_t*)pHandle->pTxBuffer;
			pHandle->pUSARTx->DR = *pData & (uint16_t)0x01ff;

			// check parity
			if (! (pHandle->pUSARTx->CR[0] & (1 << USART_CR1_PCE)))
			{
				// 9 bits of user data ..
				// no parity bit is used, increment the buffer by 2 bytes
				pHandle->pTxBuffer++;
				pHandle->pTxBuffer++;
				pHandle->TxLen -= 2;
			}
			else
			{
				// 8 bits of user data, 1 bit for parity
				// 1 parity bit used, increment the buffer by 1 byte
				pHandle->pTxBuffer++;
				pHandle->TxLen--;
			}
		}
		else
		{
			// 8 bit word length
			pHandle->pUSARTx->DR = *pHandle->pTxBuffer & (uint8_t)0xff;
			pHandle->pTxBuffer++;
			pHandle->TxLen--;
		}

		if (pHandle->TxLen == 0)
		{
			// disable TXEIE interrupt control line
			pHandle->pUSARTx->CR[0] &= ~(1 << USART_CR1_TXEIE);
		}
}

static void USART_RXHandler(USARTx_Handle_t *pHandle)
{
		// check word length
		if (pHandle->pUSARTx->CR[0] & (1 << USART_CR1_M))
		{
			// 9 bit word length
			// check parity control
			if (!(pHandle->pUSARTx->CR[0] & (1 << USART_CR1_PCE)))
			{
				// parity disabled, 9 bits of user data
				*((uint16_t*)pHandle->pRxBuffer) = (pHandle->pUSARTx->DR & (uint16_t)0x01ff);
				pHandle->pRxBuffer++;
				pHandle->pRxBuffer++;
				pHandle->RxLen -= 2;
			}
			else
			{
				// parity enabled, 8 bits of user data, 1 bit for parity
				*pHandle->pRxBuffer = (pHandle->pUSARTx->DR & (uint8_t)0xff);
				pHandle->pRxBuffer++;
				pHandle->RxLen--;
			}
		}

		else
		{
			// 8 bit word length
			// check for parity control
			if (!(pHandle->pUSARTx->CR[0] & (1 << USART_CR1_PCE)))
			{
				*pHandle->pRxBuffer = (uint8_t)pHandle->pUSARTx->DR & (uint8_t)0xff;
			}
			else
			{
				*pHandle->pRxBuffer = (uint8_t)pHandle->pUSARTx->DR & (uint8_t)0x7f;
			}

			pHandle->pRxBuffer++;
			pHandle->RxLen--;
		}
}
