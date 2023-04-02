/*
 * stm32f446re_gpio_drivers.c
 *
 *  Created on: Jan 16, 2023
 *      Author: jwizard
 */


#include "stm32f446re_gpio_drivers.h"
#include <stdbool.h>

//PIN Initialization and De-Initialization
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0;

	GPIO_PeriCLKCTRL(pGPIOHandle->pGPIOx, ENABLE);

	//Configure the pin mode
	if(pGPIOHandle->GPIO_PinConfig.PinMode <= GPIO_MODE_ANALOG){
		temp = pGPIOHandle->GPIO_PinConfig.PinMode << (2 * pGPIOHandle->GPIO_PinConfig.PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2 *pGPIOHandle->GPIO_PinConfig.PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else{
		if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_FT){
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
			EXTI->RTSR |= ~(1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_RT){
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
			EXTI->FTSR |= ~(1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_RFT){
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
		}

		//Configure port selection in SYSCFG EXTICR
		if(pGPIOHandle->GPIO_PinConfig.PinNumber <=3){
			temp = pGPIOHandle->GPIO_PinConfig.PinNumber % 4;
			uint8_t portcode = GPIO_BASE_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_CLK_EN();
			SYSCFG->EXTICR1 = portcode << (temp * 4);
		}
		else if(pGPIOHandle->GPIO_PinConfig.PinNumber >=4 && pGPIOHandle->GPIO_PinConfig.PinNumber <=7){
			temp = pGPIOHandle->GPIO_PinConfig.PinNumber % 4;
			uint8_t portcode = GPIO_BASE_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_CLK_EN();
			SYSCFG->EXTICR2 = portcode << (temp * 4);
		}
		else if(pGPIOHandle->GPIO_PinConfig.PinNumber >=8 && pGPIOHandle->GPIO_PinConfig.PinNumber <=11){
			temp = pGPIOHandle->GPIO_PinConfig.PinNumber % 4;
			uint8_t portcode = GPIO_BASE_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_CLK_EN();
			SYSCFG->EXTICR3 = portcode << (temp * 4);
		}
		else if(pGPIOHandle->GPIO_PinConfig.PinNumber >=12 && pGPIOHandle->GPIO_PinConfig.PinNumber <=15){
			temp = pGPIOHandle->GPIO_PinConfig.PinNumber % 4;
			uint8_t portcode = GPIO_BASE_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_CLK_EN();
			SYSCFG->EXTICR3 = portcode << (temp * 4);
		}

	}

	temp = 0;
	//Configure Pin Speed
	temp = pGPIOHandle->GPIO_PinConfig.PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << 2 *pGPIOHandle->GPIO_PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//Configure PUPD settings
	temp = pGPIOHandle->GPIO_PinConfig.PinPUPDCtrl << (2 * pGPIOHandle->GPIO_PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << 2 * pGPIOHandle->GPIO_PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//Configure Output Type
	temp = pGPIOHandle->GPIO_PinConfig.PinOPType << (pGPIOHandle->GPIO_PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//Configure Alternate Function
	if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_ALTFUN){
		temp = pGPIOHandle->GPIO_PinConfig.PinALTFunc << ((pGPIOHandle->GPIO_PinConfig.PinNumber % 8) * 4);
		if(pGPIOHandle->GPIO_PinConfig.PinNumber <=7){
			pGPIOHandle->pGPIOx->AFRL &= ~(0xF <<  (pGPIOHandle->GPIO_PinConfig.PinNumber % 8) * 4);
			pGPIOHandle->pGPIOx->AFRL |= temp;
		}
		else{
			pGPIOHandle->pGPIOx->AFRH &= ~(0xF << (pGPIOHandle->GPIO_PinConfig.PinNumber % 8) * 4);
			pGPIOHandle->pGPIOx->AFRH |= temp;
		}
	}
}


void GPIO_DeInit(GPIO_REG_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
}
///////////////////////////////////////////

//Peripheral Clock Control
void GPIO_PeriCLKCTRL(GPIO_REG_t* pGPIOx, uint8_t ENorDIS){
	if(ENorDIS == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_CLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_CLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_CLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_CLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_CLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_CLK_EN();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_CLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_CLK_EN();
		}
	}
	else if(ENorDIS == DISABLE){
		if(pGPIOx == GPIOA){
			GPIOA_CLK_DIS();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_CLK_DIS();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_CLK_DIS();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_CLK_DIS();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_CLK_DIS();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_CLK_DIS();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_CLK_DIS();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_CLK_DIS();
		}
	}
}
//////////////////////////////////////////


//Read GPIOx input pin
bool GPIO_ReadInPin(GPIO_REG_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)(pGPIOx->IDR >> PinNumber) & 0x00000001;
	return value;
}

//Read entire GPIOx Port
uint16_t GPIO_ReadInPort(GPIO_REG_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

//Write to GPIOx pin
void GPIO_WriteOutPin(GPIO_REG_t *pGPIOx, uint8_t PinNumber, uint8_t value){
	if(value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

//Write to entire GPIOx port
void GPIO_WriteOutPort(GPIO_REG_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}

//Toggle GPIOx pin on/off
void GPIO_TogglePin(GPIO_REG_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}
//////////////////////////////////////////

//GPIO IRQ Handling and COnfiguration
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDIS){
	if(ENorDIS == ENABLE){ //Program ISER( Interrupr Set Enable Register
		if(IRQNumber <= 31){
			//Program ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64){
			//Program ISER1
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
			//Program ISER2
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}
	else{ //Program ICER(Interrrupt Clear Enable Register
		if(IRQNumber <= 31){
			//Program ICER0
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64){
			//Program ICER1
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
			//Program ICER2
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE + (iprx * 4)) |= (IRQPriority << shift_amount);
}


void GPIO_IRQHandling(uint8_t PinNumber){
	//clear exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber)){
		EXTI->PR |= (1 << PinNumber);
	}
}
///////////////////////////////////////////
