/*
 * stm32f429xx_usart_driver.h
 *
 *  Created on: Oct 1, 2022
 *      Author: omark
 */

#ifndef INC_STM32F429XX_USART_DRIVER_H_
#define INC_STM32F429XX_USART_DRIVER_H_

#include "stm32f429xx.h"

/*********************************** CONFIGURATION MACROS ***************************************/

/*
 * @USART_Modes
 * */
#define USART_MODE_TX_ONLY					0U
#define USART_MODE_RX_ONLY					1U
#define USART_MODE_TXRX						2U

/*
 * @USART_WordLength
 * */
#define USART_WORD_8BITS					0U
#define USART_WORD_9BITS					1U

/*
 * @USART_Parity
 * */
#define USART_PARITY_DISABLE				0U
#define USART_PARITY_EN_ODD					2U
#define USART_PARITY_EN_EVEN				1U

#define USART_OVER8							1U
#define USART_OVER16						0U

/*
 * @USART_StopBits
 * */
#define USART_STOP_BITS_1					0U
#define USART_STOP_BITS_0_5	 				1U
#define USART_STOP_BITS_2					2U
#define USART_STOP_BITS_1_5					3U

#define USART_CPOL_LOW						0U
#define USART_CPOL_HIGH						1U

#define USART_CPHA_LOW						0U
#define USART_CPHA_HIGH						1U

/*
 * @USART_HardwareFlowControl
 * */
#define USART_HW_FLOW_CTRL_NONE				0U
#define USART_HW_FLOW_CTRL_CTS				1U
#define USART_HW_FLOW_CTRL_RTS				2U
#define USART_HW_FLOW_CTRL_CTS_RTS			3U

#define USART_HD_ENABLE						1U
#define USART_HD_DISABLE					0U

/*
 * @USART_Baudrates
 * */
#define USART_STD_BAUDRATE_1200				1200U
#define USART_STD_BAUDRATE_2400				2400U
#define USART_STD_BAUDRATE_9600				9600U
#define USART_STD_BAUDRATE_19200			19200U
#define USART_STD_BAUDRATE_38400			38400U
#define USART_STD_BAUDRATE_57600			57600U
#define USART_STD_BAUDRATE_115200			115200U
#define USART_STD_BAUDRATE_230400			230400U
#define USART_STD_BAUDRATE_460800			460800U
#define USART_STD_BAUDRATE_921600			921600U
#define USART_STD_BAUDRATE_2M				2000000U
#define USART_STD_BAUDRATE_3M				3000000U

/************************************************************************************************/

/************************************* USARTx SR Flags ******************************************/

#define USART_FLAG_PE						(1 << USART_SR_PE)
#define USART_FLAG_FE						(1 << USART_SR_FE)
#define USART_FLAG_NF						(1 << USART_SR_NF)
#define USART_FLAG_ORE						(1 << USART_SR_ORE)
#define USART_FLAG_IDLE						(1 << USART_SR_IDLE)
#define USART_FLAG_RXNE						(1 << USART_SR_RXNE)
#define USART_FLAG_TC						(1 << USART_SR_TC)
#define USART_FLAG_TXE						(1 << USART_SR_TXE)
#define USART_FLAG_LBD						(1 << USART_SR_LBD)
#define USART_FLAG_CTS						(1 << USART_SR_CTS)

/***********************************************************************************************/

/************************************** USARTx States ******************************************/

#define USART_STATE_READY					0U
#define USART_STATE_TX_BUSY					1U
#define USART_STATE_RX_BUSY					2U


/************************************ Application States ***************************************/

#define USART_TX_CMPLT						3U
#define USART_RX_CMPLT						4U
#define USART_ER_OVR						5U
#define USART_EV_IDLE						6U
#define USART_CTS_CMPLT						7U
#define USART_ER_PARITY						8U
#define USART_ER_NE							9U
#define USART_ER_FE							10U

/***********************************************************************************************/

typedef struct
{
	uint8_t USART_mode;								/*!< possible options at @USART_Modes >*/
	uint32_t USART_baudRate;						/*!< possible options at @USART_Baudrates >*/
	uint8_t USART_wordLength;						/*!< possible options at @USART_WordLength >*/
	uint8_t USART_no_of_stopBits;					/*!< possible options at @USART_StopBits >*/
	uint8_t USART_parityControl;					/*!< possible options at @USART_Parity >*/
	uint8_t USART_hardwareFlowControl;				/*!< possible options at @USART_HardwareFlowControl >*/

} USARTx_Config_t;

typedef struct
{
	USARTx_t *pUSARTx;
	USARTx_Config_t USARTx_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;

} USARTx_Handle_t;

/**********************************************************************************************
 * 								APIs supported by this driver
 **********************************************************************************************/

/*
 * Peripheral control and clock control
 * */
void USART_PClockControl(USARTx_t *pUSARTx, uint8_t en_di);
void USART_PControl(USARTx_t *pUSARTx, uint8_t en_di);

/*
 * Peripheral Initialization and De-initialization
 * */
void USART_Init(USARTx_Handle_t *pHandle);
void USART_DeInit(USARTx_t *pUSARTx);

/*
 * Data Communication APIs (polling)
 * */

void USART_SendData(USARTx_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t len);
void USART_ReceiveData(USARTx_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t len);
uint8_t USART_SendDataIT(USARTx_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t USART_ReceiveDataIT(USARTx_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t len);

/*
 * Interrupt configuration and Handling APIs
 * */

void USART_IRQPriorityConfig(uint8_t IRQ_number, uint8_t prio);
void USART_IRQConfig(uint8_t IRQ_number, uint8_t en_di);
void USART_InterruptHandling(USARTx_Handle_t *pHandle);

/*
 * Other APIs
 * */

void USART_CloseTransmission(USARTx_Handle_t *pHandle);
void USART_CloseReception(USARTx_Handle_t *pHandle);
uint8_t USART_GetFlagStatus(USARTx_t *pUSARTx, uint16_t flag_name);
void USART_ClearFlag(USARTx_t *pUSARTx, uint16_t flag_name);
void USART_SetBaudrate(USARTx_t *pUSARTx, uint32_t baudRate);

__weak void USART_ApplicationCallback(USARTx_Handle_t *pHandle, uint8_t appEvent);
/*
 * Private Helper Functions
 * */


/**********************************************************************************************/


#endif /* INC_STM32F429XX_USART_DRIVER_H_ */
