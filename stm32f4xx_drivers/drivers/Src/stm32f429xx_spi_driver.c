/*
 * srm32f429xx_spi_driver.c
 *
 *  Created on: Sep 20, 2022
 *      Author: omark
 */


#include "stm32f42xx_spi_driver.h"

/* Private Helper Functions */
static void spi_tx_interrupt_handling(SPI_Handle_t *pSPI_Handle);
static void spi_rx_interrupt_handling(SPI_Handle_t *pSPI_Handle);
static void spi_ovrr_interrupt_handling(SPI_Handle_t *pSPI_Handle);

/*
 * Peripheral clock setup
 * */

/********************************************************************
 * @fn				- SPI_PClockControl
 *
 * @brief			- Enables/ Disables the peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the given SPI peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void SPI_PClockControl(SPIx_t *pSPIx, uint8_t en_di)
{
	if (en_di == ENABLE)
	{
		if (pSPIx == SPI1)
			SPI1_PCLK_EN();
		else if (pSPIx == SPI2)
			SPI2_PCLK_EN();
		else if (pSPIx == SPI3)
			SPI3_PCLK_EN();
		else if (pSPIx == SPI4)
			SPI4_PCLK_EN();
		else if (pSPIx == SPI5)
			SPI5_PCLK_EN();
		else if (pSPIx == SPI6)
			SPI6_PCLK_EN();
	}

	else
	{
		if (pSPIx == SPI1)
			SPI1_PCLK_DI();
		else if (pSPIx == SPI2)
			SPI2_PCLK_DI();
		else if (pSPIx == SPI3)
			SPI3_PCLK_DI();
		else if (pSPIx == SPI4)
			SPI4_PCLK_DI();
		else if (pSPIx == SPI5)
			SPI5_PCLK_DI();
		else if (pSPIx == SPI6)
			SPI6_PCLK_DI();
	}
}

/*
 * Status Register Flags Check
 * */


uint8_t SPI_GetFlagStatus(SPIx_t *pSPIx, uint8_t flagName)
{
	if (pSPIx->SR & (flagName))
		return FLAG_SET;

	return FLAG_RESET;
}

/*
 * Init and De-Init
 * */

/********************************************************************
 * @fn				- SPI_Init
 *
 * @brief			- initialized the SPI peripheral registers
 *
 * @param[in]		- user SPI handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void SPI_Init(SPI_Handle_t *pSPI_Handle)
{

	// SPI peripheral clock enable
	SPI_PClockControl(pSPI_Handle->pSPIx, ENABLE);

	// configure CR1 register
	uint32_t tempreg = 0;

	// configure device mode
	tempreg |= (pSPI_Handle->SPI_config.device_mode << SPI_CR1_MSTR);

	// configure the bus communication
	if (pSPI_Handle->SPI_config.bus_config == SPI_BUS_CONFIG_FD)
	{
		// full-duplex
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE);
	}
	else if (pSPI_Handle->SPI_config.bus_config == SPI_BUS_CONFIG_HD)
	{
		// half-duplex
		tempreg |= (1 << SPI_CR1_BIDI_MODE);
	}
	else if (pSPI_Handle->SPI_config.bus_config == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// simplex receive-only
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE);
		tempreg |=  (1 << SPI_CR1_RXONLY);
	}

	// configure the data frame format
	tempreg |= (pSPI_Handle->SPI_config.DFF << SPI_CR1_DFF);

	//configure the clock phase
	tempreg |= (pSPI_Handle->SPI_config.CPHA << SPI_CR1_CPHA);

	// configure the clock polarity
	tempreg |= (pSPI_Handle->SPI_config.CPOL << SPI_CR1_CPOL);

	// configure the software/hardware slave management
	tempreg |= (pSPI_Handle->SPI_config.SSM << SPI_CR1_SSM);

	// configure the serial clock speed
	tempreg |= (pSPI_Handle->SPI_config.speed << SPI_CR1_BR);

	// mask tempreg to control register 1
	pSPI_Handle->pSPIx->CR[0] |= tempreg;

}

/********************************************************************
 * @fn				- SPI_DeInit
 *
 * @brief			- resets the SPI peripheral registers
 *
 * @param[in]		- SPI peripheral address
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void SPI_DeInit(SPIx_t *pSPIx)
{
	if (pSPIx == SPI1)
		SPI1_REG_RESET();
	else if (pSPIx == SPI2)
		SPI2_REG_RESET();
	else if (pSPIx == SPI3)
		SPI3_REG_RESET();
	else if (pSPIx == SPI4)
		SPI4_REG_RESET();
	else if (pSPIx == SPI5)
		SPI5_REG_RESET();
	else if (pSPIx == SPI6)
		SPI6_REG_RESET();
}

/*
 * Data Transmission and Receiving
 * */

/********************************************************************
 * @fn				- SPI_SendData
 *
 * @brief			- transmit data to external world
 *
 * @param[in]		- SPI peripheral address
 * @param[in]		- transmitter buffer address
 * @param[in]		- length of the data to be sent
 *
 * @return			- none
 *
 * @note			- pTxBuffer is a software buffer
 * @note			- This is a blocking call, until all bytes are transferred,
 * 					  this function will block
 *
 * */
void SPI_SendData(SPIx_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while (len)
	{
		// wait until TX buffer is empty
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// check the data frame format
		if ((pSPIx->CR[0] & (1 << SPI_CR1_DFF)))
		{
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len --;
			len --;
			(uint16_t*)pTxBuffer++;
		}

		else
		{
			pSPIx->DR = *(pTxBuffer);
			len--;
			pTxBuffer++;
		}
	}
}

/********************************************************************
 * @fn				- SPI_SendDataIT
 *
 * @brief			- transmit data to external world
 *
 * @param[in]		- SPI user handle
 * @param[in]		- transmitter buffer address
 * @param[in]		- length of the data to be sent
 *
 * @return			- none
 *
 * @note			- pTxBuffer is a software buffer
 * @note			- This is a blocking call, until all bytes are transferred,
 * 					  this function will block
 *
 * */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPI_Handle->TxState;

	if (state != SPI_TX_BSY_STATE)
	{
	// save the TX buffer and length of data
	pSPI_Handle->pTxBuffer = pTxBuffer;
	pSPI_Handle->TxLen = len;

	// mark the SPI state as busy in transmission so that no other code can take over the
	// peripheral until transmission is over
	pSPI_Handle->TxState = SPI_TX_BSY_STATE;

	// set TXEIE control bit to get interrupt whenever TXE flag is set
	pSPI_Handle->pSPIx->CR[1] |= (1 << SPI_CR2_TXEIE);

	}

	return state;
}

/********************************************************************
 * @fn				- SPI_ReceiveData
 *
 * @brief			- receive data from external world
 *
 * @param[in]		- SPI peripheral address
 * @param[in]		- receiver buffer address
 * @param[in]		- length of the data to be received
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void SPI_ReceiveData(SPIx_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while (len)
	{
		// hang until RX buffer is full
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// check the data frame format (byte / 2 bytes)

		if ((pSPIx->CR[0] & (1 << SPI_CR1_DFF)))
		{
			*((uint16_t*)pRxBuffer) = (uint16_t) pSPIx->DR;
			len--;
			len--;
			(uint16_t*)pRxBuffer++;
		}

		else
		{
			*pRxBuffer = pSPIx-> DR;
			len--;
			pRxBuffer++;
		}
	}
}


/********************************************************************
 * @fn				- SPI_ReceiveDataIT
 *
 * @brief			- receive data from external world
 *
 * @param[in]		- SPI peripheral address
 * @param[in]		- receiver buffer address
 * @param[in]		- length of the data to be received
 *
 * @return			- none
 *
 * @note			- none
 *
 * */

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPI_Handle->RxState;

	if (state != SPI_RX_BSY_STATE)
	{

	// save RX buffer and length of data
	pSPI_Handle->pRxBuffer = pRxBuffer;
	pSPI_Handle->RxLen = len;

	// mark SPI state busy in reception
	pSPI_Handle->RxState = SPI_RX_BSY_STATE;

	// set RXNEIE control bit to enable SPI interrupt on full RX buffer
	pSPI_Handle->pSPIx->CR[1] |= (1 << SPI_CR2_RXNEIE);

	}

	return state;
}

/*
 * IRQ configuration and ISR handling
 * */

/********************************************************************
 * @fn				- SPI_InterruptPriorityConfig
 *
 * @brief			- configure the corresponding IRQ with the priority level
 *
 * @param[in]		- SPIx IRQ number
 * @param[in]		- priority level
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void SPI_InterruptPriorityConfig(uint8_t IRQ_number, uint8_t priority_level)
{
	uint8_t iprx_reg_no = IRQ_number / 4;
	uint8_t iprx_section = IRQ_number % 4;
	uint8_t iprx_samt = (iprx_section * 8) + (8 - NO_IMPLEMENTED_PRIO_BITS);

	*(NVIC_IPR_BASEADDR + iprx_reg_no) |= (priority_level << iprx_samt);
}

/********************************************************************
 * @fn				- SPI_InterruptConfig
 *
 * @brief			- sets / clears the corresponding IRQ bit in order to enable / disable
 * 					- the received irq_number
 *
 * @param[in]		- IRQ number of the SPI triggered interrupt to NVIC unit
 * @param[in]		- ENABLE / DISABLE
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void SPI_InterruptConfig(uint8_t IRQ_number, uint8_t en_di)
{
	if (en_di == ENABLE)
	{
		if (IRQ_number <= 31)
			*(NVIC_ISER0) |= (1 << IRQ_number);

		else if ((IRQ_number) > (31 && IRQ_number < 64))
			*(NVIC_ISER1) |= (1 << IRQ_number % 32);

		else if ((IRQ_number >= 64) && (IRQ_number < 96))
			*(NVIC_ISER2) |= (1 << IRQ_number % 64);
	}

	else
	{
		if (IRQ_number <= 31)
			*(NVIC_ICER0) |= (1 << IRQ_number);

		else if ((IRQ_number > 31) && (IRQ_number < 64))
			*(NVIC_ICER1) |= (1 << IRQ_number % 32);

		else if ((IRQ_number >= 64) && (IRQ_number < 96))
			*(NVIC_ICER2) |= (1 << IRQ_number % 64);
	}
}

/********************************************************************
 * @fn				- SPI_InterruptHanling
 *
 * @brief			- handles the triggered SPI interrupt
 *
 * @param[in]		- SPI user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void SPI_InterruptHanling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;

	// check for transmission interrupt
	temp1 = (pHandle->pSPIx->SR & (1 << SPI_SR_TXE));
	temp2 = (pHandle->pSPIx->CR[1] & (1 << SPI_CR2_TXEIE));

	if (temp1 && temp2)
	{
		spi_tx_interrupt_handling(pHandle);
	}

	// check for reception interrupt
	temp1 = (pHandle->pSPIx->SR & (1 << SPI_SR_RXNE));
	temp2 = (pHandle->pSPIx->CR[1] & (1 << SPI_CR2_RXNEIE));

	if (temp1 && temp2)
	{
		spi_rx_interrupt_handling(pHandle);
	}

	// check for overrun error interrupt
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR[1] & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		spi_ovrr_interrupt_handling(pHandle);
	}
}

/*
 * Other Peripheral Control APIs
 * */


/********************************************************************
 * @fn				- SPI_PControl
 *
 * @brief			- Enable SPIx peripheral to start communicating/ exchanging data
 *
 * @param[in]		- SPIx peripheral address
 * @param[in]		- ENABLE / DISABLE
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void SPI_PControl(SPIx_t *pSPIx, uint8_t en_di)
{
	if(en_di == ENABLE)
	{
		pSPIx->CR[0] |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR[0] &= ~(1 << SPI_CR1_SPE);
	}
}

/********************************************************************
 * @fn				- SPI_SSIConfig
 *
 * @brief			- Set or Reset SSI bit position
 *
 * @param[in]		- SPIx peripheral address
 * @param[in]		- ENABLE / DISABLE
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void SPI_SSIConfig(SPIx_t *pSPIx, uint8_t en_di)
{
	if (en_di == ENABLE)
		pSPIx->CR[0] |= (1 << SPI_CR1_SSI);
	else
		pSPIx->CR[0] &= (1 << SPI_CR1_SSI);
}

/********************************************************************
 * @fn				- SPI_SSOEConfig
 *
 * @brief			- Set or Reset SSOE bit position
 *
 * @param[in]		- SPIx peripheral address
 * @param[in]		- ENABLE / DISABLE
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void SPI_SSOEConfig(SPIx_t *pSPIx, uint8_t en_di)
{
	if (en_di == ENABLE)
	{
		pSPIx->CR[1] |= (1 << SPI_CR2_SSOE);
	}

	else
	{
		pSPIx->CR[1] &= ~(1 << SPI_CR2_SSOE);
	}
}

/********************************************************************
 * @fn				- SPI_ClearOVRFlag
 *
 * @brief			- Set or Reset SSOE bit position
 *
 * @param[in]		- SPIx user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void SPI_ClearOVRFlag(SPI_Handle_t *pSPI_Handle)
{
	uint8_t temp;
	temp = pSPI_Handle->pSPIx->DR;
	temp = pSPI_Handle->pSPIx->SR;
	(void)temp;
}

/********************************************************************
 * @fn				- SPI_CloseTransmission
 *
 * @brief			- End SPIx transmission explicitly
 *
 * @param[in]		- SPIx user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void SPI_CloseTransmission(SPI_Handle_t *pSPI_Handle)
{
	pSPI_Handle->pSPIx->CR[1] &= ~(1 << SPI_CR2_TXEIE);
	pSPI_Handle->TxState = SPI_READY_STATE;
	pSPI_Handle->pTxBuffer = NULL;
	pSPI_Handle->TxLen = 0;
}

/********************************************************************
 * @fn				- SPI_CloseReception
 *
 * @brief			- End SPIx reception of data explicitly
 *
 * @param[in]		- SPIx user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void SPI_CloseReception(SPI_Handle_t *pSPI_Handle)
{
	pSPI_Handle->pSPIx->CR[1] &= ~(1 << SPI_CR2_RXNEIE);
	pSPI_Handle->pRxBuffer = NULL;
	pSPI_Handle->RxState = SPI_READY_STATE;
	pSPI_Handle->RxLen = 0;
}


/*
 *  Helper Functions Implementation
 * */

static void spi_tx_interrupt_handling(SPI_Handle_t *pSPI_Handle)
{
	if ((pSPI_Handle->pSPIx->SR & (1 << SPI_CR1_DFF)))
	{
		pSPI_Handle->pSPIx->DR = *((uint16_t*)pSPI_Handle->pTxBuffer);
		pSPI_Handle->TxLen--;
		pSPI_Handle->TxLen--;
		(uint16_t*)pSPI_Handle->pTxBuffer++;
	}

	else
	{
		pSPI_Handle->pSPIx->DR = *(pSPI_Handle->pTxBuffer);
		pSPI_Handle->TxLen--;
		pSPI_Handle->pTxBuffer++;
	}

	if (! pSPI_Handle->TxLen)
	{
		// end the communication, informing the application that transmission is over
		SPI_CloseTransmission(pSPI_Handle);
		SPI_ApplicationCallback(pSPI_Handle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rx_interrupt_handling(SPI_Handle_t *pSPI_Handle)
{
	if (pSPI_Handle->pSPIx->CR[0] & (1 << SPI_CR1_DFF))
	{
		*((uint16_t*)(pSPI_Handle->pRxBuffer)) = (uint16_t) pSPI_Handle->pSPIx->DR;
		pSPI_Handle->RxLen--;
		pSPI_Handle->RxLen--;
		(uint16_t*)pSPI_Handle->pRxBuffer++;
	}

	else
	{
		*(pSPI_Handle->pRxBuffer) = pSPI_Handle->pSPIx->DR;
		pSPI_Handle->RxLen--;
		pSPI_Handle->pRxBuffer++;
	}

	if (!pSPI_Handle->RxLen)
	{
		// end the communication, informing the application that no more receiving interrupts
		// are required
		SPI_CloseReception(pSPI_Handle);
		SPI_ApplicationCallback(pSPI_Handle, SPI_EVENT_RX_CMPLT);
	}

}

static void spi_ovrr_interrupt_handling(SPI_Handle_t *pSPI_Handle)
{
	uint8_t temp;

	// clear OVR error, if SPIx peripheral is not transmitting data in the time being
	if (pSPI_Handle->TxState != SPI_TX_BSY_STATE)
	{
		temp = pSPI_Handle->pSPIx->DR;
		temp = pSPI_Handle->pSPIx->SR;
	}
	(void)temp;

	// inform the application, and it has the choice whether to clear OVR bit or not
	SPI_ApplicationCallback(pSPI_Handle, SPI_EVENT_OVR_ERR);
}

__weak void SPI_ApplicationCallback(SPI_Handle_t *pSPI_Handle, uint8_t app_event)
{
	// weak implementation, user should override this function
}
