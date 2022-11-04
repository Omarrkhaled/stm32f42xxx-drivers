/*
 * stm32f429xx_i2c_driver.h
 *
 *  Created on: Sep 27, 2022
 *      Author: omark
 */

#ifndef INC_STM32F429XX_I2C_DRIVER_H_
#define INC_STM32F429XX_I2C_DRIVER_H_

#include "stm32f429xx.h"

/*
 * Configuration MACROS
 * */

/*
 * @I2C_SCL_SPEED
 * */
#define I2C_SCL_SPEED_STD				100000U
#define I2C_SCL_SPEED_FM4K				400000U
#define I2C_SCL_SPEED_FM2K				200000U


/*
 * @I2C_ACKControl
 * */
#define I2C_ACK_EN						1U
#define I2C_ACK_DI						0U

/*
 * @I2C_FMDutyCycle
 * */
#define I2C_FM_DUTY_2					0U
#define I2C_FM_DUTY_16_9				1U

/*****************************************************************************************/


/*
 * SR1 Flags
 * */
#define I2C_SB_FLAG						(1 << I2Cx_SR1_SB)
#define I2C_ADDR_FLAG					(1 << I2Cx_SR1_ADDR)
#define I2C_BTF_FLAG					(1 << I2Cx_SR1_BTF)
#define I2C_STOPF_FLAG					(1 << I2Cx_SR1_STOPF)
#define I2C_RXNE_FLAG					(1 << I2Cx_SR1_RxNE)
#define I2C_TXE_FLAG					(1 << I2Cx_SR1_TxE)
#define I2C_BERR_FLAG					(1 << I2Cx_SR1_BERR)
#define I2C_ARLO_FLAG					(1 << I2Cx_SR1_ARLO)
#define I2C_AF_FLAG						(1 << I2Cx_SR1_AF)
#define I2C_OVR_FLAG					(1 << I2Cx_SR1_OVR)
#define I2C_PECERR_FLAG					(1 << I2Cx_SR1_PECERR)
#define I2C_TIMEOUT_FLAG				(1 << I2Cx_SR1_TIMEOUT)
#define I2C_SMBALERT_FLAG				(1 << I2Cx_SR1_SMBALERT)

/*
 * SR2 Flags
 * */
#define I2C_MSL_FLAG					(1 << I2Cx_SR2_MSL)
#define I2C_BUSY_FLAG					(1 << I2Cx_SR2_BUSY)
#define I2C_TRA_FLAG					(1 << I2Cx_SR2_TRA)
#define I2C_GENCALL_FLAG				(1 << I2Cx_SR2_GENCALL)
#define I2C_SMBDEFAULT_FLAG				(1 << I2Cx_SR2_SMBDEFAULT)
#define I2C_SMBHOST_FLAG				(1 << I2Cx_SR2_SMBHOST)
#define I2C_DUALF_FLAG					(1 << I2Cx_SR2_DUALF)

// repeated start enable / disable MACROS
#define I2C_SR_ENABLE					1U
#define I2C_SR_DISABLE					0U

// Application states
#define I2C_STATE_RDY					0U
#define I2C_STATE_RX_BSY				1U
#define I2C_STATE_TX_BSY				2U

// Application Callback events
#define I2C_EV_STOP						0U
#define I2C_EV_TX_CMPLT					1U
#define I2C_EV_RX_CMPLT					2U

// I2C error flags
#define I2C_ERROR_BERR					3U
#define I2C_ERROR_ARLO					4U
#define I2C_ERROR_AF					5U
#define I2C_ERROR_OVR					6U
#define I2C_ERROR_TIMEOUT				7U
#define I2C_EV_DATA_REQ					8U
#define I2C_EV_DATA_RCV					9U

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DevAddrr;
	uint8_t  I2C_AckCtrl;
	uint16_t I2C_FMDutyCycle;

} I2Cx_Config_t;

typedef struct
{
	I2Cx_t *pI2Cx;
	I2Cx_Config_t I2Cx_Config;
	uint8_t *pTxBuffer;						/*! < to store application TX buffer address >*/
	uint8_t *pRxBuffer;						/*! < to store application RX buffer address >*/
	uint8_t TxRxState;						/*! < to store communication state >*/
	uint32_t TxLen;							/*! < to store TX buffer length >*/
	uint32_t RxLen;							/*! < to store RX buffer length >*/
	uint8_t sr;								/*! < to store repeated start value >*/
	uint8_t DevAddr;						/*! < to store device/slave address >*/
	uint32_t RxSize;						/*! < to store RX size >*/

} I2Cx_Handle_t;


/**********************************************************************************************
 * 								APIs supported by this driver
 **********************************************************************************************/
/*
 * Peripheral control and clock control
 * */
void I2C_PClockControl(I2Cx_t *pI2Cx, uint8_t en_di);
void I2C_PControl(I2Cx_t *pI2Cx, uint8_t en_di);

/*
 * Initialization and De-Initialization
 * */
void I2C_Init(I2Cx_Handle_t *pHandle);
void I2C_DeInit(I2Cx_t *pI2Cx);

/*
 * Master TX / RX, Slave TX / RX (polling)
 * */
void I2C_MasterTX(I2Cx_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slave_addr, uint8_t sr);
void I2C_MasterRX(I2Cx_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slave_addr, uint8_t sr);

/*
 * Master TX / RX, Slave TX / RX (interrupt)
 * */
uint8_t I2C_MasterTXIT(I2Cx_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slave_addr, uint8_t sr);
uint8_t I2C_MasterRXIT(I2Cx_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slave_addr, uint8_t sr);

void I2C_SlaveTX(I2Cx_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveRX(I2Cx_t *pI2Cx);


/*
 * IRQ Configuration and Interrupt Handling
 * */

void I2C_IRQPriorityConfig(uint8_t IRQ_number, uint8_t pri);
void I2C_IRQInterruptConfig(uint8_t IRQ_number, uint8_t en_di);
void I2C_EV_IRQHandling(I2Cx_Handle_t *pHandle);
void I2C_ER_IRQHandling(I2Cx_Handle_t *pHandle);

/*
 * Application Callback
 * */

__weak void I2C_ApplicationCallback(I2Cx_Handle_t *pHandle, uint8_t appEvent);

/*
 * Other APIs
 * */
uint8_t I2C_GetFlagStatus(I2Cx_t *pI2Cx, uint8_t flag_name);
void I2C_ManageACKing(I2Cx_t *pI2Cx, uint8_t en_di);
void I2C_CloseTransmission(I2Cx_Handle_t *pHandle);
void I2C_CloseReception(I2Cx_Handle_t *pHandle);
void I2C_GenerateStopCondition(I2Cx_t *pIC2x);
void I2C_SlaveEnDiCallbackEvents(I2Cx_t *pI2Cx, uint8_t en_di);


#endif /* INC_STM32F429XX_I2C_DRIVER_H_ */
