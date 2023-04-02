/*
 * stm32f446re_i2c_drivers.c
 *
 *  Created on: Feb 11, 2023
 *      Author: jwizard
 */

#include "stm32f446re_i2c_drivers.h"
#include <stdio.h>

uint16_t AHB_Prescaler[8] = {2,4,8,16,32,64,256,512};
uint8_t APB_Prescaler[4] = {2,4,8,16};

static void I2C_GenerateStartCondition(I2C_REG_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_REG_t *pI2Cx);

void I2C_GenerateStartCondition(I2C_REG_t *pI2Cx){
	pI2Cx->CR1 |= (1 << 8);
}

static void I2C_GenerateStopCondition(I2C_REG_t *pI2Cx){
	pI2Cx->CR1 |= (1 << 9);
}

static void I2CExecuteAddressPhase(I2C_REG_t *pI2Cx, uint8_t SlaveAddress, uint8_t RorW){
	SlaveAddress = SlaveAddress << 1;
	if(RorW == 0){
		SlaveAddress &= ~(1); //Write bit
	}
	else if(RorW == 1){
		SlaveAddress |= (1); //Read bit
	}
	pI2Cx->DR = SlaveAddress;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){
	uint32_t dummy_read;

	//Check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << 0)){
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			if(pI2CHandle->RxSize == 1){
				//Disable the ACK
				I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);

				//Clear ADDR flag, read SR2
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;

			}
		}
		else{
			//Clear ADDR flag, read SR2
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}
	else{
		//Device is in slave mode
		//Clear ADDR flag, read SR2
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}



uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1, SystemClk;


	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = (RCC->RCC_CFGR >> 2) & 0x3;

	if(clksrc == 0){
		SystemClk = 16000000;
	}
	else if(clksrc == 1){
		SystemClk = 8000000;
	}

	//FOR AHB
	temp = (RCC->RCC_CFGR >> 4) & 0xF;

	if(temp < 8){
		ahbp = 1;
	}
	else{
		ahbp = AHB_Prescaler[temp-8];
	}

	//FOR APB1
	temp = (RCC->RCC_CFGR >> 10) & 0x7;

		if(temp < 4){
			apb1p = 1;
		}
		else{
			ahbp = APB_Prescaler[temp-4];
		}

		pclk1 = (SystemClk / ahbp) / apb1p;

		return pclk1;
}

//Peripheral Clock Control
void I2C_PeriCLKCTRL(I2C_REG_t* pI2Cx, uint8_t ENorDIS){
	if(ENorDIS == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_CLK_EN();
		}
		else if(pI2Cx == I2C2){
			I2C2_CLK_EN();
		}
		else if(pI2Cx == I2C3){
			I2C3_CLK_EN();
		}
	}
	else if(ENorDIS == DISABLE){
		if(pI2Cx == I2C1){
			I2C1_CLK_DIS();
		}
		else if(pI2Cx == I2C2){
			I2C2_CLK_DIS();
		}
		else if(pI2Cx == I2C3){
			I2C3_CLK_DIS();
		}
	}
}
//////////////////////////////////////////

//I2C ENABLE/DISABLE PERIPHERAL CONTROL
void I2C_PeripheralControl(I2C_REG_t *pI2Cx,uint8_t ENorDIS){
	if(ENorDIS == ENABLE){
			pI2Cx->CR1 |= (1 << 0); //Enable the ACK
		}
		else{
			pI2Cx->CR1 &= ~(1 << 0); //Disable the ACK
		}
}


//PIN Initialization and De-Initialization
void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t temp = 0;

	//Enable the clock for I2C
	I2C_PeriCLKCTRL(pI2CHandle->pI2Cx, ENABLE);

	temp |= pI2CHandle->I2C_Config.ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = temp;

	//Configure the FREQ
	temp = 0;
	temp |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = temp & 0x3F;

	temp |= pI2CHandle->I2C_Config.DeviceAddress << 1;
	temp |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = temp;

	//Configure CCR value
	uint16_t ccr_value = 0;
	temp = 0;
	if(pI2CHandle->I2C_Config.SCLSpeed <= I2C_SCL_SPEED_SM){
		//Standard Mode
		ccr_value = RCC_GetPCLK1Value() / ( 2*pI2CHandle->I2C_Config.SCLSpeed);
		temp |= (ccr_value & 0xFFF);
	}
	else{
		//Fast Mode
		temp |= (1 << 15);
		temp |= (pI2CHandle->I2C_Config.FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = RCC_GetPCLK1Value() / ( 3*pI2CHandle->I2C_Config.SCLSpeed);
		}
		else{
			ccr_value = RCC_GetPCLK1Value() / ( 25*pI2CHandle->I2C_Config.SCLSpeed);
		}
		temp |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = temp;

	//TRise Config
	if(pI2CHandle->I2C_Config.SCLSpeed <= I2C_SCL_SPEED_SM){
			//Standard Mode
		temp = (RCC_GetPCLK1Value() / 1000000U) + 1;
		}
		else{
			temp = (RCC_GetPCLK1Value() * 300 / 1000000U) + 1;
		}

	pI2CHandle->pI2Cx->TRISE = temp & 0x3F;

}


void I2C_DeInit(I2C_REG_t *pI2Cx){
	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}
///////////////////////////////////////////

uint8_t I2C_GetFlagStatus(I2C_REG_t *pI2Cx, uint32_t FlagName){
	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_ManageACK(I2C_REG_t *pI2Cx, uint8_t EnorDis){
	if(EnorDis == ENABLE){
		pI2Cx->CR1 |= (1 << 10);
	}
	else if(EnorDis == DISABLE){
		pI2Cx->CR1 &= ~(1 << 10);
	}
}

void I2CMasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t SR){
	//Generate Start Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Wait for Start bit (SB to be set to 1, indicating start generation)
	//Note: Unitl SB is cleared, SCL with be stretched (pulled to low)
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_SB_FLAG)));

	//Send Address of the slave with r/rw bit set to w(0) (Total: 8bits)
	I2CExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddress, RESET);

	//Confirm that address phase is completed by checking the ADDR flag in the SR1
	//Note: Until ADDR is cleared, SCL with be stretched (pulled to low)
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ADDR_FLAG)));

	//Clear the ADDR flag according to its software sequence
	I2C_ClearADDRFlag(pI2CHandle);

	//Send Data until Len becomes zero
	while(Len > 0){
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG)); // wait until TXE is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//Wait until TXE = 1 and BTF = 1 before generating STOP condition
	//Note: When both TXE = 1 and BTF = 1, this means that both SR and DR are empty and next transmission should begin
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));// Wait until TXE = 1

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG));// Wait until BTF = 1

	if(SR == 0){ //Determine if repeated start
		//Generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}

}

void I2CMasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t SR){
	//1. Generate START Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2.Confirm that the start generation is completed by checking the SB flag in the SR1
	//Note: Until SB is cleared, SCL will be stretched (pulled to low)
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_SB_FLAG)));

	//3.Send the address of the slave with r/rw bit set to R(1) (total: 8 bits)
	I2CExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddress, SET);

	//4. Wait until address phase is completed by checking the ADDR flag in the SR1
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ADDR_FLAG)));

	//Procedure to read only 1 byte from slave
	if(Len == 1){
		//Disable Acking
		I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);

		//Clear the addr flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Wait until the RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));

		if(SR == 0){
			//Generate STOP condition
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//Read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}
	else if(Len > 1){
		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Read the data until len becomes zero
		for(uint32_t i=Len; i>0; i--){
			//Wait until RXNE becomes 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));

			if(i==2){ //if last two bytes are remaining
				//Clear the ack bit
				I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);

				if(SR == 0){
					//Generate STOP condition
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//Read the data from the data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//Increment the buffer address
			pRxBuffer++;
		}
	}

	//Re-enable Acking
	I2C_ManageACK(pI2CHandle->pI2Cx, ENABLE);
}

uint8_t I2CMasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t SR){
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pTxBuffer = pTxBuffer;
			pI2CHandle->TxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
			pI2CHandle->DevAddr = SlaveAddress;
			pI2CHandle->SR = SR;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << 10);

			//Implement the code to enable ITEVTEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << 9);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << 8);


		}

		return busystate;
}
uint8_t I2CMasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t SR){
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pTxBuffer = pRxBuffer;
			pI2CHandle->TxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
			pI2CHandle->DevAddr = SlaveAddress;
			pI2CHandle->SR = SR;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << 10);

			//Implement the code to enable ITEVTEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << 9);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << 8);


		}

		return busystate;
}

//I2CIRQ Handling and Configuration
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDIS){
	if(ENorDIS == ENABLE) //Program ISER(Interrupt Set Enable Register
		{
			if(IRQNumber <= 31)
			{
				//program ISER0 register
				*NVIC_ISER0 |= ( 1 << IRQNumber );

			}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
			{
				//program ISER1 register
				*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96 )
			{
				//program ISER2 register //64 to 95
				*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
			}
		}else //Program ICER(Interrupt Clear Enable Register
		{
			if(IRQNumber <= 31)
			{
				//program ICER0 register
				*NVIC_ICER0 |= ( 1 << IRQNumber );
			}else if(IRQNumber > 31 && IRQNumber < 64 )
			{
				//program ICER1 register
				*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 6 && IRQNumber < 96 )
			{
				//program ICER2 register
				*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
			}
		}
}
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	//1. first lets find out the ipr register
		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section  = IRQNumber %4 ;

		uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

		*(NVIC_PR_BASE + iprx) |=  (IRQPriority << shift_amount);
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->TxLen > 0){
		//Load the data into DR
		pI2CHandle->pI2Cx->DR = *pI2CHandle->pTxBuffer;

		//Decrement TXLen
		pI2CHandle->TxLen--;

		//Increment buffer address
		pI2CHandle->pTxBuffer++;
		}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->RxSize == 1){
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxSize > 1){
		if(pI2CHandle->RxSize == 2){
			//Clear the ACK bit
			I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);
		}
		//Read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0){
		//Close the I2C Data reception and notify the application

		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		I2C_CloseReceiveData(pI2CHandle);

		I2C_ApplicationEventCallback(pI2CHandle->pI2Cx, I2C_EV_RX_CMPLT);
	}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << 10);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << 9);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageACK(pI2CHandle->pI2Cx,ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << 10); //DIsable ITBUFEN

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << 9); //Disable ITEVEN


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_SlaveSendData(I2C_REG_t *pI2C, uint8_t data){
	pI2C->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_REG_t *pI2C){
	return (uint8_t) pI2C->DR;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	//Interrupt Handling for both Master and Slave
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << 9); //Read ITEVEN
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << 10);//Read ITBUFEN
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 0); //Read SB

	//Handle Interrupt for SB
	if(temp1 && temp3){
		//SB Flag is set
		//Execute Address Phase
		//Note: Block will not be executed in SLave mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			I2CExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, 0);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			I2CExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, 1);
		}
	}

	//Handle for ADDR Event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 1); //Read ADDR
	if(temp1 && temp3){
		//ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//Handle for BTF Event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 2); //Read BTF
	if(temp1 && temp3){
		//BTF flag is set
		//Confirm length is zero before closing communication
		if(pI2CHandle->TxLen == 0){
			//if TX buffer is busy...
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
				//Confirm SR is 0 befor generating stop condition
				if(pI2CHandle->pI2Cx->SR1 & (I2C_TXE_FLAG)){
					if(pI2CHandle->SR == 0){
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

						//Reset all member elements of the handle structure
						I2C_CloseSendData(pI2CHandle);

						//Notify application about transmission completion
						I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);
					}
				}
			}
			else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
				;
			}
		}
	}

	//Handle for STOPF Event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 4); //Read STOPF
	if(temp1 && temp3){
		//STOPF flag is set
		//Only applies to the slave, detects STOP after ACK
		//Must read SR1 register (see line above) followed by writing to the CR1 register
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}

	//Handle for TXE Event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 7); //Read TXE
	if(temp1 && temp2 && temp3){
		//TXE flag is set
		//Confirm this is the master by checking the MSL Bbit
		if(pI2CHandle->pI2Cx->SR2 & (1 << 0)){
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else{
			//slave
			if(pI2CHandle->pI2Cx->SR2 & (1 << 2)){
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
			}
		}
	}

	//Handle for RXNE Event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 6); //Read RXNE
	if(temp1 && temp2 && temp3){
		//RXNE flag is set
		//Check device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << 0)){
			//Device is in Master mode
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else{
			if(!(pI2CHandle->pI2Cx->SR2 & (1 << 2))){
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}

	}

}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){

	uint32_t temp1,temp2;

	//Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << 8);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< 8);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << 8);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,8);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << 9);
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << 9);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << 10);
	if(temp1  && temp2)
	{
		//This is ACK failure error

		//Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << 10);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << 11);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

		//Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << 11);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << 14);
	if(temp1  && temp2)
	{
		//This is Time out error

		//Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << 14);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}


}
