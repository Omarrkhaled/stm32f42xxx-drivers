/*
 * stm32f429xx_i2c_driver.c
 *
 *  Created on: Sep 27, 2022
 *      Author: omark
 */

#include "stm32f429xx_i2c_driver.h"

static void I2C_GenerateStart(I2Cx_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2Cx_t *pI2Cx, uint8_t slave_address);
static void I2C_ExecuteAddressPhaseRead(I2Cx_t *pI2Cx, uint8_t slave_address);
static void I2C_ClearADDRFlag(I2Cx_Handle_t *pI2Cx);
static void I2C_MasterTXITHandler(I2Cx_Handle_t *pHandle);
static void I2C_MasterRXITHandler(I2Cx_Handle_t *pHandle);

/**********************************************************************************************
 * 								APIs supported by this driver
 **********************************************************************************************/

/********************************************************************
 * @fn				- I2C_PClockControl
 *
 * @brief			- enables / disables the given I2C peripheral clock on it's bus domain
 *
 * @param[in]		- base address of the given I2C peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void I2C_PClockControl(I2Cx_t *pI2Cx, uint8_t en_di)
{
	if (en_di == ENABLE)
	{
		if (pI2Cx == I2C1)
			I2C1_PCLK_EN();

		else if (pI2Cx == I2C2)
			I2C2_PCLK_EN();

		else if (pI2Cx == I2C3)
			I2C3_PCLK_EN();
	}

	else
	{
		if (pI2Cx == I2C1)
			I2C1_PCLK_DI();

		else if (pI2Cx == I2C2)
			I2C2_PCLK_DI();

		else if (pI2Cx == I2C3)
			I2C3_PCLK_DI();
	}
}

/********************************************************************
 * @fn				- I2C_PControl
 *
 * @brief			- enables / disables the given I2C peripheral
 *
 * @param[in]		- base address of the given I2C peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void I2C_PControl(I2Cx_t *pI2Cx, uint8_t en_di)
{
	if (en_di == ENABLE)
	{
		pI2Cx->CR[0] |=  (1 << I2Cx_CR1_PE);
	}

	else
	{
		pI2Cx->CR[0] &= ~(1 << I2Cx_CR1_PE);
	}
}

/********************************************************************
 * @fn				- I2C_Init
 *
 * @brief			- configures the peripheral parameters
 *
 * @param[in]		- I2C user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void I2C_Init(I2Cx_Handle_t *pHandle)
{

	// enable the peripheral clock
	I2C_PClockControl(pHandle->pI2Cx, ENABLE);

	uint32_t temp_reg = 0;

	// enable ACKing
	temp_reg = pHandle->I2Cx_Config.I2C_AckCtrl << I2Cx_CR1_ACK;
	pHandle->pI2Cx->CR[0] |= temp_reg;

	// configure the FRQ field of CR2
	temp_reg = RCC_GetfPClk1Value() / 1000000U;
	pHandle->pI2Cx->CR[1] |= temp_reg;

	//configure the device address
	temp_reg  = pHandle->I2Cx_Config.I2C_DevAddrr << 1;
	temp_reg |= (1 << 14);
	pHandle->pI2Cx->OAR[0] |= temp_reg;

	// configure CCR bit fields according to provided SCL speed
	uint16_t ccr_field;

	if (pHandle->I2Cx_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_STD)
	{

		ccr_field = 0.5 * (RCC_GetfPClk1Value() / pHandle->I2Cx_Config.I2C_SCLSpeed);
		temp_reg = ccr_field;
	}

	else
	{
		// configure to FM mode
		temp_reg = (1 << 15);

		// configure duty cycle
		temp_reg |= (pHandle->I2Cx_Config.I2C_FMDutyCycle << 14);

		if (pHandle->I2Cx_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_field = (1/3) * (RCC_GetfPClk1Value() / pHandle->I2Cx_Config.I2C_SCLSpeed);
		}

		else
		{
			ccr_field = (1/25) * (RCC_GetfPClk1Value() / pHandle->I2Cx_Config.I2C_SCLSpeed);
		}

		temp_reg |= ccr_field & 0xffff;
	}

	pHandle->pI2Cx->CCR |= temp_reg;

	// configure rise time of SDA, SCL signals
	if (pHandle->I2Cx_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_STD)
	{
		// SM
		temp_reg = (RCC_GetfPClk1Value() / 1000000U) + 1;
	}

	else
	{
		// FM
		temp_reg = ((RCC_GetfPClk1Value() * 300) / 1000000000U) + 1;
	}

	pHandle->pI2Cx->TRISE |= (temp_reg & 0x3f);

}

/********************************************************************
 * @fn				- I2C_DeInit
 *
 * @brief			- resets all the I2Cx peripheral register
 *
 * @param[in]		- I2C user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void I2C_DeInit(I2Cx_t *pI2Cx)
{
	if (pI2Cx == I2C1)
		I2C1_REG_RESET();

	else if (pI2Cx == I2C2)
		I2C2_REG_RESET();

	else if (pI2Cx == I2C3)
		I2C3_REG_RESET();
}

/*
 * Master TX / RX, Slave TX / RX
 * */

/********************************************************************
 * @fn				- I2C_MasterTX
 *
 * @brief			- transmit data to external world, as master
 *
 * @param[in]		- I2Cx user handle
 * @param[in]		- data software transmitting buffer
 * @param[in]		- length of data to be transmitted
 * @param[in]		- address of the designated slave
 * @param[in]		- repeated start: ENABLE / DISABLE
 *
 * @return			- none
 *
 * @note			- this is a blocking call implementation (polling)
 *
 * */
void I2C_MasterTX(I2Cx_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slave_addr, uint8_t sr)
{
	// invoke the start condition
	I2C_GenerateStart(pHandle->pI2Cx);

	// block until Start Bit is set (SB flag in SR1 register) which means start condition is
	// successfully generated
	while (! I2C_GetFlagStatus(pHandle->pI2Cx, I2C_SB_FLAG));

	// send the address of the designated slave with R/nW bit reset
	// R/nW => 0
	I2C_ExecuteAddressPhaseWrite(pHandle->pI2Cx,slave_addr);

	// block until addressing phase is complete
	while (! I2C_GetFlagStatus(pHandle->pI2Cx, I2C_ADDR_FLAG));

	// clear ADDR flag
	I2C_ClearADDRFlag(pHandle);

	// now, start sending the data
	while (len)
	{
	// first, hang until DR is empty
	while (! I2C_GetFlagStatus(pHandle->pI2Cx, I2C_TXE_FLAG));
	// write the data in DR register
	pHandle->pI2Cx->DR = *pTxBuffer;
	pTxBuffer++;
	len--;
	}

	// now, all data is transmitted (len = 0), so, close the communication
	// wait for TXE = 1, BTF = 1 before generating the stop condition
	while (! I2C_GetFlagStatus(pHandle->pI2Cx, I2C_TXE_FLAG));
	while (! I2C_GetFlagStatus(pHandle->pI2Cx, I2C_BTF_FLAG));

	// here, generate the stop condition
	if (sr == I2C_SR_DISABLE)
		I2C_GenerateStopCondition(pHandle->pI2Cx);

}

/********************************************************************
 * @fn				- I2C_MasterTXIT
 *
 * @brief			- transmit data to external world, as master
 *
 * @param[in]		- I2Cx user handle
 * @param[in]		- data software transmitting buffer
 * @param[in]		- length of data to be transmitted
 * @param[in]		- address of the designated slave
 * @param[in]		- repeated start: ENABLE / DISABLE
 *
 * @return			- application state
 *
 * @note			- none
 *
 * */
uint8_t I2C_MasterTXIT(I2Cx_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slave_addr, uint8_t sr)
{
	uint8_t state = pHandle->TxRxState;

	if ((state != I2C_STATE_TX_BSY) && (state != I2C_STATE_RX_BSY))
	{
		// store the TX buffer address, data length, etc..
		pHandle->pTxBuffer = pTxBuffer;
		pHandle->TxLen = len;
		pHandle->DevAddr = slave_addr;
		pHandle->TxRxState = I2C_STATE_TX_BSY;
		pHandle->sr = sr;

		// generate the start condition
		I2C_GenerateStart(pHandle->pI2Cx);

		// enable Interrupt control bits
		pHandle->pI2Cx->CR[1] |= (1 << I2Cx_CR2_ITEVTEN);
		pHandle->pI2Cx->CR[1] |= (1 << I2Cx_CR2_ITBUFEN);
		pHandle->pI2Cx->CR[1] |= (1 << I2Cx_CR2_ITERREN);
	}


	return state;
}


/********************************************************************
 * @fn				- I2C_MasterRX
 *
 * @brief			- receive data from external world
 *
 * @param[in]		- I2Cx user handle
 * @param[in]		- data software receiving buffer
 * @param[in]		- length of data to be received
 * @param[in]		- address of the designated slave
 * @param[in]		- repeated start: ENABLE / DISABLE
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void I2C_MasterRX(I2Cx_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slave_addr, uint8_t sr)
{
	//generate the start condition
	I2C_GenerateStart(pHandle->pI2Cx);

	// block until SB is set
	while (! I2C_GetFlagStatus(pHandle->pI2Cx, I2C_SB_FLAG));

	// transmit the address of the designated slave and the R/nW control bit
	I2C_ExecuteAddressPhaseRead(pHandle->pI2Cx, slave_addr);

	// block until ADDR is set
	while (! I2C_GetFlagStatus(pHandle->pI2Cx, I2C_ADDR_FLAG));

	// procedure to read only 1 byte from the slave
	if (len == 1)
	{
		// disable ACKing
		I2C_ManageACKing(pHandle->pI2Cx, DISABLE);

		// clear ADDR flag to release the clock from stretching
		I2C_ClearADDRFlag(pHandle);

		// confirm DR is full (the whole byte is received)
		while (! I2C_GetFlagStatus(pHandle->pI2Cx, I2C_RXNE_FLAG));

		// set STOP bit
		if (sr == I2C_SR_DISABLE)
			I2C_GenerateStopCondition(pHandle->pI2Cx);

		// read the byte
		*pRxBuffer = pHandle->pI2Cx->DR;
	}

	// procedure to read more than 1 byte from the slave
	if (len > 1)
	{
		// move the data from the shift register to the DR register
		I2C_ClearADDRFlag(pHandle);

		while (len)
		{

		// block till DR is full
		while (! I2C_GetFlagStatus(pHandle->pI2Cx, I2C_RXNE_FLAG));

		if (len == 2)
		{
		// now, length of data remaining is 2 bytes, so disable ACKing
		I2C_ManageACKing(pHandle->pI2Cx, DISABLE);

		// set STOP control bit
		if (sr == I2C_SR_DISABLE)
			I2C_GenerateStopCondition(pHandle->pI2Cx);

		}

		// read the data from the DR
		*pRxBuffer = pHandle->pI2Cx->DR;
		pRxBuffer++;
		len--;

		}
	}

	// re-enable ACKing
	if (pHandle->I2Cx_Config.I2C_AckCtrl == I2C_ACK_EN)
		I2C_ManageACKing(pHandle->pI2Cx, ENABLE);
}

/********************************************************************
 * @fn				- I2C_MasterRX
 *
 * @brief			- receive data from external world
 *
 * @param[in]		- I2Cx user handle
 * @param[in]		- data software receiving buffer
 * @param[in]		- length of data to be received
 * @param[in]		- address of the designated slave
 * @param[in]		- repeated start: ENABLE / DISABLE
 *
 * @return			- application state
 *
 * @note			- none
 *
 * */
uint8_t I2C_MasterRXIT(I2Cx_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slave_addr, uint8_t sr)
{

	uint8_t state = pHandle->TxRxState;

	if ((state != I2C_STATE_TX_BSY) && (state != I2C_STATE_RX_BSY))
	{
		// store the TX buffer address, data length, etc..
		pHandle->pRxBuffer = pRxBuffer;
		pHandle->RxSize = len;
		pHandle->RxLen = len;
		pHandle->DevAddr = slave_addr;
		pHandle->TxRxState = I2C_STATE_RX_BSY;
		pHandle->sr = sr;

		// generate the start condition
		I2C_GenerateStart(pHandle->pI2Cx);

		// enable Interrupt control bits
		pHandle->pI2Cx->CR[1] |= (1 << I2Cx_CR2_ITBUFEN);
		pHandle->pI2Cx->CR[1] |= (1 << I2Cx_CR2_ITEVTEN);
		pHandle->pI2Cx->CR[1] |= (1 << I2Cx_CR2_ITERREN);
	}


	return state;

}

/********************************************************************
 * @fn				- I2C_SlaveTX
 *
 * @brief			-
 *
 * @param[in]		- I2Cx peripheral address
 * @param[in]		- requested data
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void I2C_SlaveTX(I2Cx_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}

/********************************************************************
 * @fn				- I2C_SlaveRX
 *
 * @brief			- receives the data
 *
 * @param[in]		- I2Cx peripheral address
 *
 * @return			- the received data / command from the master
 *
 * @note			- none
 *
 * */
uint8_t I2C_SlaveRX(I2Cx_t *pI2Cx)
{
	return ((uint8_t)pI2Cx->DR);
}

/*
 * IRQ Configuration and Interrupt Handling
 * */

/********************************************************************
 * @fn				- I2C_IRQPriorityConfig
 *
 * @brief			- configures I2Cx peripheral IRQ priority
 *
 * @param[in]		- IRQ number
 * @param[in]		- priority level
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void I2C_IRQPriorityConfig(uint8_t IRQ_number, uint8_t pri)
{
	uint8_t iprx = IRQ_number / 4;
	uint8_t iprx_section = IRQ_number % 4;

	uint8_t samt = (iprx_section * 8) + (8 - NO_IMPLEMENTED_PRIO_BITS);

	*(NVIC_IPR_BASEADDR + iprx) |= (pri << samt);
}

/********************************************************************
 * @fn				- I2C_IRQInterruptConfig
 *
 * @brief			- configures I2Cx peripheral ENABLE / DISABLE
 *
 * @param[in]		- IRQ number
 * @param[in]		- ENABLE / DISABLE
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void I2C_IRQInterruptConfig(uint8_t IRQ_number, uint8_t en_di)
{
	if (en_di == ENABLE)
	{
		if (IRQ_number <= 31)
			*(NVIC_ISER0) |= (1 << IRQ_number);

		else if (IRQ_number > 31 && IRQ_number < 64)
			*(NVIC_ISER1) |= (1 << IRQ_number % 32);

		else if (IRQ_number >= 64 && IRQ_number < 96)
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
 * @fn				- I2C_EV_IRQHandling
 *
 * @brief			- decode and handle the triggered event interrupt
 *
 * @param[in]		- I2C user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void I2C_EV_IRQHandling(I2Cx_Handle_t *pHandle)
{
	uint32_t temp1, temp2, temp3;

	temp1 = (pHandle->pI2Cx->CR[1]) & (1 << I2Cx_CR2_ITEVTEN);
	temp2 = (pHandle->pI2Cx->CR[1]) & (1 << I2Cx_CR2_ITBUFEN);

	// decode which event triggered this interrupt

	// SB
	temp3 = pHandle->pI2Cx->SR[0] & (1 << I2Cx_SR1_SB);
	if (temp3 && temp1)
	{
		// handle SB event
		if (pHandle->TxRxState == I2C_STATE_TX_BSY)
			I2C_ExecuteAddressPhaseWrite(pHandle->pI2Cx, pHandle->DevAddr);
		else if (pHandle->TxRxState == I2C_STATE_RX_BSY)
			I2C_ExecuteAddressPhaseRead(pHandle->pI2Cx, pHandle->DevAddr);
	}

	// ADDR
	temp3 = pHandle->pI2Cx->SR[0] & (1 << I2Cx_SR1_ADDR);
	if (temp3 && temp1)
	{
		// handle ADDR event
		I2C_ClearADDRFlag(pHandle);
	}

	// BTF (Byte Transfer Finished)
	temp3 = pHandle->pI2Cx->SR[0] & (1 << I2Cx_SR1_BTF);
	if (temp3 && temp1)
	{
		// handle BTF event
		// use BTF flag in order to close the communication (TX or RX) after the last byte
		// has been transferred successfully
		if (pHandle->TxRxState == I2C_STATE_TX_BSY)
		{
			// confirm TxE flag is set, also
			if (pHandle->pI2Cx->SR[0] & (1 << I2Cx_SR1_TxE))
			{
				// now, BTF and TxE = 1

				if (pHandle->TxLen == 0)
				{
					// generate stop condition
					if (pHandle->sr == I2C_SR_DISABLE)
						I2C_GenerateStopCondition(pHandle->pI2Cx);

					// end transmission
					I2C_CloseTransmission(pHandle);

					// notify the application that transmission is finished / closed
					I2C_ApplicationCallback(pHandle, I2C_EV_TX_CMPLT);
				}
			}
		}

		else if (pHandle->TxRxState == I2C_STATE_RX_BSY)
		{
			;
		}
	}

	// STOPF
	temp3 = pHandle->pI2Cx->SR[0] & (1 << I2Cx_SR1_STOPF);
	if (temp3 && temp1)
	{
		// handle STOPF event
		// clear the STOPF bit (read SR1 (already done), write into CR1)
		pHandle->pI2Cx->CR[0] |= 0x0000;

		// notify the application that stop condition is detected
		I2C_ApplicationCallback(pHandle, I2C_EV_STOP);
	}

	// TxE
	temp3 = pHandle->pI2Cx->SR[0] & (1 << I2Cx_SR1_TxE);
	if (temp1 && temp2 && temp3)
	{
		// handle TX event
		// as TxE flag is set, we have to transmit data now.
		// check for device mode
		if (pHandle->pI2Cx->SR[1] & (1 << I2Cx_SR2_MSL))
		{
			// master mode
			if (pHandle->TxRxState == I2C_STATE_TX_BSY)
			{
				I2C_MasterTXITHandler(pHandle);
			}
		}

		else
		{
			// slave mode
			if (pHandle->pI2Cx->SR[1] & (1 << I2Cx_SR2_TRA))
			{
				I2C_ApplicationCallback(pHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	// RxNE
	temp3 = pHandle->pI2Cx->SR[0] & (1 << I2Cx_SR1_RxNE);
	if (temp1 && temp2 && temp3)
	{
		// handle RX event
		// check device mode
		if (pHandle->pI2Cx->SR[1] & (1 << I2Cx_SR2_MSL))
		{
			if (pHandle->TxRxState == I2C_STATE_RX_BSY)
			{
				I2C_MasterRXITHandler(pHandle);
			}
		}
		else
		{
			if (! ((pHandle->pI2Cx->SR[1]) & (1 << I2Cx_SR2_TRA)))
			{
				I2C_ApplicationCallback(pHandle, I2C_EV_DATA_RCV);
			}
		}
	}

}

/********************************************************************
 * @fn				- I2C_ER_IRQHandling
 *
 * @brief			- decode and handle the triggered error interrupt
 *
 * @param[in]		- I2C user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void I2C_ER_IRQHandling(I2Cx_Handle_t *pHandle)
{
	uint32_t temp1, temp2;

	temp1 = pHandle->pI2Cx->CR[1] & (1 << I2Cx_CR2_ITERREN);

	// OVR
	temp2 = pHandle->pI2Cx->SR[0] & (1 << I2Cx_SR1_OVR);
	if (temp1 && temp2)
	{
		// handle OVR error

		// clear
		pHandle->pI2Cx->SR[0] &= ~(1 << I2Cx_SR1_OVR);

		// notify
		I2C_ApplicationCallback(pHandle, I2C_ERROR_OVR);
	}

	// ARLO (arbitration lost)
	temp2 = pHandle->pI2Cx->SR[0] & (1 << I2Cx_SR1_ARLO);
	if (temp1 && temp2)
	{
		// handle ARLO error

		// clear
		pHandle->pI2Cx->SR[0] &= ~(1 << I2Cx_SR1_ARLO);

		// notify
		I2C_ApplicationCallback(pHandle, I2C_ERROR_ARLO);
	}

	// AF (acknowledge failure)
	temp2 = pHandle->pI2Cx->SR[0] & (1 << I2Cx_SR1_AF);
	if (temp1 && temp2)
	{
		// handle AF error

		// clear
		pHandle->pI2Cx->SR[0] &= ~(1 << I2Cx_SR1_AF);

		// notify
		I2C_ApplicationCallback(pHandle, I2C_ERROR_AF);
	}

	// BERR (bus error)
	temp2 = pHandle->pI2Cx->SR[0] & (1 << I2Cx_SR1_BERR);
	if (temp1 && temp2)
	{
		// handle BERR error

		// clear BERR flag
		pHandle->pI2Cx->SR[0] &= ~(1 << I2Cx_SR1_BERR);
		// notify the application about the error
		I2C_ApplicationCallback(pHandle, I2C_ERROR_BERR);

	}

}

/*
 * Application Callback
 * */
__weak void I2C_ApplicationCallback(I2Cx_Handle_t *pHandle, uint8_t appEvent)
{

}


/*
 * Other APIs
 * */


/********************************************************************
 * @fn				- I2C_GetFlagStatus
 *
 * @brief			- returns the clock speed of the bus domain on which I2Cx peripheral hangs on
 *
 * @param[in]		- I2Cx peripheral address
 * @param[in]		- flag to be checked from SR1 register
 *
 * @return			- flag status
 *
 * @note			- none
 *
 * */
uint8_t I2C_GetFlagStatus(I2Cx_t *pI2Cx, uint8_t flag_name)
{
	if (pI2Cx->SR[0] & flag_name)
		return FLAG_SET;
	else
		return FLAG_RESET;
}


/********************************************************************
 * @fn				- ManageACKing
 *
 * @brief			- enable or disable ACKing
 *
 * @param[in]		- I2Cx peripheral address
 * @param[in]		- ENABLE / DISABLE
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void I2C_ManageACKing(I2Cx_t *pI2Cx, uint8_t en_di)
{
	if (en_di == ENABLE)
	{
		pI2Cx->CR[0] |= (1 << I2Cx_CR1_ACK);
	}

	else
	{
		pI2Cx->CR[0] &= ~(1 << I2Cx_CR1_ACK);
	}
}

/********************************************************************
 * @fn				- I2C_CloseTransmission
 *
 * @brief			- end transmitting data
 *
 * @param[in]		- I2Cx user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void I2C_CloseTransmission(I2Cx_Handle_t *pHandle)
{
	pHandle->pI2Cx->CR[1] &= ~(1 << I2Cx_CR2_ITBUFEN);
	pHandle->pI2Cx->CR[1] &= ~(1 << I2Cx_CR2_ITEVTEN);

	pHandle->TxRxState = I2C_STATE_RDY;
	pHandle->TxLen = 0;
	pHandle->pTxBuffer = NULL;
}

/********************************************************************
 * @fn				- I2C_CloseReception
 *
 * @brief			- end receiving data
 *
 * @param[in]		- I2Cx user handle
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void I2C_CloseReception(I2Cx_Handle_t *pHandle)
{
	pHandle->pI2Cx->CR[1] &= ~(1 << I2Cx_CR2_ITBUFEN);
	pHandle->pI2Cx->CR[1] &= ~(1 << I2Cx_CR2_ITEVTEN);

	pHandle->TxRxState = I2C_STATE_RDY;
	pHandle->RxLen = 0;
	pHandle->RxSize = 0;
	pHandle->pRxBuffer = NULL;

	if (pHandle->I2Cx_Config.I2C_AckCtrl == I2C_ACK_EN)
		I2C_ManageACKing(pHandle->pI2Cx, ENABLE);
}

/********************************************************************
 * @fn				- I2C_GenerateStopCondition
 *
 * @brief			- generate a stop condition
 *
 * @param[in]		- I2Cx peripheral address
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void I2C_GenerateStopCondition(I2Cx_t *pIC2x)
{
	pIC2x->CR[0] |= (1 << I2Cx_CR1_STOP);
}

/********************************************************************
 * @fn				- I2C_SlaveEnDiCallbackEvents
 *
 * @brief			- enable / disable interrupt control bits
 *
 * @param[in]		- I2Cx peripheral address
 * @param[in]		- ENABLE / DISABLE
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void I2C_SlaveEnDiCallbackEvents(I2Cx_t *pI2Cx, uint8_t en_di)
{
	if (en_di == ENABLE)
	{
		pI2Cx->CR[1] |= (1 << I2Cx_CR2_ITBUFEN);
		pI2Cx->CR[1] |= (1 << I2Cx_CR2_ITEVTEN);
		pI2Cx->CR[1] |= (1 << I2Cx_CR2_ITERREN);
	}

	else
	{
		pI2Cx->CR[1] &= ~(1 << I2Cx_CR2_ITBUFEN);
		pI2Cx->CR[1] &= ~(1 << I2Cx_CR2_ITEVTEN);
		pI2Cx->CR[1] &= ~(1 << I2Cx_CR2_ITERREN);
	}
}


/*
 * Private Helper Functions
 * */
static void I2C_GenerateStart(I2Cx_t *pI2Cx)
{
	pI2Cx->CR[0] |= (1 << I2Cx_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2Cx_t *pI2Cx, uint8_t slave_address)
{
	slave_address  = slave_address << 1;
	slave_address  &= ~(1);
	pI2Cx->DR = slave_address;
}

static void I2C_ExecuteAddressPhaseRead(I2Cx_t *pI2Cx, uint8_t slave_address)
{
	slave_address  = slave_address << 1;
	slave_address  |= 1;
	pI2Cx->DR = slave_address;
}

static void I2C_ClearADDRFlag(I2Cx_Handle_t *pHandle)
{
	uint32_t dummy_read;

	// check for device mode
	if (pHandle->pI2Cx->SR[1] & (1 << I2Cx_SR2_MSL))
	{
		// device is in master mode
		// check device state
		if (pHandle->TxRxState == I2C_STATE_RX_BSY)
		{
			if (pHandle->RxSize == 1)
			{
				// disable acking
				I2C_ManageACKing(pHandle->pI2Cx, DISABLE);

				// clear the ADDR flag
				dummy_read = pHandle->pI2Cx->SR[0];
				dummy_read = pHandle->pI2Cx->SR[1];
				(void)dummy_read;
			}

			else
			{
				dummy_read = pHandle->pI2Cx->SR[0];
				dummy_read = pHandle->pI2Cx->SR[1];
				(void)dummy_read;
			}
		}
	}

	else
	{
		// device is in slave mode
		dummy_read = pHandle->pI2Cx->SR[0];
		dummy_read = pHandle->pI2Cx->SR[1];
		(void)dummy_read;
	}
}


static void I2C_MasterTXITHandler(I2Cx_Handle_t *pHandle)
{
		// check if there is remaining data
		if (pHandle->TxLen > 0)
		{
			pHandle->pI2Cx->DR = *(pHandle->pTxBuffer);
			pHandle->TxLen--;
			pHandle->pTxBuffer++;
		}
}


static void I2C_MasterRXITHandler(I2Cx_Handle_t *pHandle)
{
		// case 1: receiving only 1 byte of data
		if (pHandle->RxSize == 1)
		{
			*(pHandle->pRxBuffer) = pHandle->pI2Cx->DR;
			 pHandle->RxLen--;
		}

		// case 2: receiving more than 1 byte of data
		if (pHandle->RxSize > 1)
		{
			if (pHandle->RxLen == 2)
				I2C_ManageACKing(pHandle->pI2Cx, DISABLE);

			// read the data
			*(pHandle->pRxBuffer) = pHandle->pI2Cx->DR;
			pHandle->RxLen--;
			pHandle->pRxBuffer++;
		}

		if (pHandle->RxLen == 0)
		{
			// generate a stop condition
			if (pHandle->sr == I2C_SR_DISABLE)
				I2C_GenerateStopCondition(pHandle->pI2Cx);

			// end reception
			I2C_CloseReception(pHandle);

			// notify the application
			I2C_ApplicationCallback(pHandle, I2C_EV_RX_CMPLT);
		}
}
