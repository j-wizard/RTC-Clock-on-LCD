/*
 * stm32f446re_i2c_drivers.h
 *
 *  Created on: Feb 11, 2023
 *      Author: jwizard
 */

#ifndef INC_STM32F446RE_I2C_DRIVERS_H_
#define INC_STM32F446RE_I2C_DRIVERS_H_

#include "stm32f446re.h"

typedef struct{
	uint32_t SCLSpeed;
	uint32_t DeviceAddress;
	uint32_t ACKControl;
	uint32_t FMDutyCycle;


}I2C_Config_t;


typedef struct{
	I2C_REG_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;				//Storing app. Tx buffer address.
	uint8_t *pRxBuffer;				//Storing app. Rx buffer address.
	uint32_t TxLen;					//Storing Tx Length
	uint32_t RxLen;					//Storing Tx Length
	uint8_t TxRxState;				//Storing communication state
	uint8_t DevAddr;				//Storing slave/device address
	uint32_t RxSize;				//Storing Rx size
	uint8_t SR;						//Storing repeated start value
}I2C_Handle_t;

//SCL SPEEDS
#define I2C_SCL_SPEED_SM			100000
#define I2C_SCL_SPEED_FM			400000
#define I2C_SCL_SPEED_FM2K			200000


//ACK CONTROL
#define I2C_ACK_ENABLE				1
#define I2C_ACK_DISABLE				0


//DUTY CYCLE
#define I2C_FM_DUTY_2				0
#define I2C_FM_DUTY_16_9			1

#define I2C_SB_FLAG					(1 << 0)
#define I2C_ADDR_FLAG				(1 << 1)
#define I2C_BTF_FLAG				(1 << 2)
#define I2C_STOPF_FLAG				(1 << 4)
#define I2C_RXNE_FLAG				(1 << 6)
#define I2C_TXE_FLAG				(1 << 7)
#define I2C_BERR_FLAG				(1 << 8)
#define I2C_ARLO_FLAG				(1 << 9)
#define I2C_AF_FLAG					(1 << 10)
#define I2C_OVR_FLAG				(1 << 11)
#define I2C_TIMEOUT_FLAG			(1 << 14)


//I2C application events macros
#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9


//I2C Application States
#define I2C_READY					0
#define I2C_BUSY_IN_RX				1
#define I2C_BUSY_IN_TX				2

//I2C Application Events Macros
#define I2C_EV_TX_CMPLT				0
#define I2C_EV_RX_CMPLT				1
#define I2C_EV_STOP					2

//Peripheral Clock Control
void I2C_PeriCLKCTRL(I2C_REG_t* pI2Cx, uint8_t ENorDIS);
//////////////////////////////////////////

//I2C ENABLE/DISABLE PERIPHERAL CONTROL
void I2C_PeripheralControl(I2C_REG_t *pI2Cx,uint8_t ENorDIS);


//PIN Initialization and De-Initialization
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_REG_t *pI2Cx);
///////////////////////////////////////////

void I2CMasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t SR);
void I2CMasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t SR);

void I2C_SlaveSendData(I2C_REG_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_REG_t *pI2C);

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);

uint8_t I2CMasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t SR);
uint8_t I2CMasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t SR);

uint8_t I2C_GetFlagStatus(I2C_REG_t *pI2Cx, uint32_t FlagName);
void I2C_ManageACK(I2C_REG_t *pI2Cx, uint8_t EnorDis);


//GPIO IRQ Handling and COnfiguration
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDIS);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

#endif /* INC_STM32F446RE_I2C_DRIVERS_H_ */
