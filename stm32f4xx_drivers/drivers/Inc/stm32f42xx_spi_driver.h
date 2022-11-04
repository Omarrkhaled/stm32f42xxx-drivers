/*
 * stm32f42xx_spi_driver.h
 *
 *  Created on: Sep 20, 2022
 *      Author: omark
 */

#ifndef INC_STM32F42XX_SPI_DRIVER_H_
#define INC_STM32F42XX_SPI_DRIVER_H_

#include "stm32f429xx.h"

/*
 *********************************** Configuration MACROS ***********************************
 **/

/*
 * @SPI_DeviceMode
 * */
#define SPI_DEV_MODE_MASTER					1U
#define SPI_DEV_MODE_SLAVE					0U

/*
 * @SPI_BusConfig
 * */
#define SPI_BUS_CONFIG_FD					1U
#define SPI_BUS_CONFIG_HD					2U
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3U

/*
 * @SPI_DFFs
 * */
#define SPI_DFF_8BITS						0U
#define SPI_DFF_16BITS						1U

/*
 * @SPI_CPHA
 * */
#define SPI_CPHA_LOW						0U
#define SPI_CPHA_HIGH						1U

/*
 * @SPI_CPOL
 * */
#define SPI_CPOL_LOW						0U
#define SPI_CPOL_HIGH						1U

/*
 * @SPI_SSM
 * */
#define SPI_SSM_HW							0U
#define SPI_SSM_SW							1U

/*
 * @SPI_SCLK_SPEED
 * */
#define SPI_SCLK_SPEED_DIV2					0U
#define SPI_SCLK_SPEED_DIV4					1U
#define SPI_SCLK_SPEED_DIV8					2U
#define SPI_SCLK_SPEED_DIV16				3U
#define SPI_SCLK_SPEED_DIV32				4U
#define SPI_SCLK_SPEED_DIV64				5U
#define SPI_SCLK_SPEED_DIV128				6U
#define SPI_SCLK_SPEED_DIV256				7U
/********************************************************************************************/

/* SPI Status Register Flags */
#define SPI_TXE_FLAG				(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG				(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG				(1 << SPI_SR_BSY)

/* SPI Possible States */
#define SPI_READY_STATE				0
#define SPI_RX_BSY_STATE			1
#define SPI_TX_BSY_STATE			2

/* Callback MACROS */
#define SPI_EVENT_TX_CMPLT			1
#define SPI_EVENT_RX_CMPLT			2
#define SPI_EVENT_OVR_ERR			3
#define SPI_EVENT_CRC_ERR			4

/*
 ********Configuration structure and peripheral Handle for the sample application************
 * */
typedef struct
{
	uint8_t device_mode;			/*!< possible device modes at @SPI_DeviceMode >*/
	uint8_t bus_config;				/*!< possible bus configurations at @SPI_BusConfig >*/
	uint8_t DFF;					/*!< possible data frame formats at @SPI_DFFs >*/
	uint8_t CPHA;					/*!< possible clock phases at @SPI_CPHA >*/
	uint8_t CPOL;					/*!< possible clock polarities at @SPI_CPOL >*/
	uint8_t SSM;					/*!< possible SSM values at @SPI_SSM >*/
	uint8_t speed;					/*!< possible speeds @SPI_SCLK_SPEED >*/

} SPI_Config_t;

typedef struct
{
	SPIx_t *pSPIx;
	SPI_Config_t SPI_config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;

} SPI_Handle_t;

/********************************************************************************************/

/**
 *********************************** APIs Supported ****************************************
 **/

/*
 * Peripheral clock setup
 * */
void SPI_PClockControl(SPIx_t *pSPIx, uint8_t en_di);

/*
 * Init and De-Init
 * */
void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPIx_t *pSPIx);

/*
 * Data Transmission and Receiving
 * */

// polling / blocking call
void SPI_SendData(SPIx_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPIx_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

// interrupt based
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ configuration and ISR handling
 * */
void SPI_InterruptPriorityConfig(uint8_t IRQ_number, uint8_t priority_level);
void SPI_InterruptConfig(uint8_t IRQ_number, uint8_t en_di);
void SPI_InterruptHanling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 * */
void SPI_PControl(SPIx_t *pSPIx, uint8_t en_di);
void SPI_SSIConfig(SPIx_t *pSPIx, uint8_t en_di);
void SPI_SSOEConfig(SPIx_t *pSPIx, uint8_t en_di);
uint8_t SPI_GetFlagStatus(SPIx_t *pSPIx, uint8_t flagName);
void SPI_ClearOVRFlag(SPI_Handle_t *pSPI_Handle);
void SPI_CloseTransmission(SPI_Handle_t *pSPI_Handle);
void SPI_CloseReception(SPI_Handle_t *pSPI_Handle);

/*
 * Application Callback
 * */
__weak void SPI_ApplicationCallback(SPI_Handle_t *pSPI_Handle, uint8_t app_event);


/********************************************************************************************/
#endif /* INC_STM32F42XX_SPI_DRIVER_H_ */
