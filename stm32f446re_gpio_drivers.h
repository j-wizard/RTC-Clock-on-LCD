/*
 * stm32f446re_gpio_drivers.h
 *
 *  Created on: Jan 16, 2023
 *      Author: jwizard
 */

#ifndef INC_STM32F446RE_GPIO_DRIVERS_H_
#define INC_STM32F446RE_GPIO_DRIVERS_H_

#include "stm32f446re.h"
#include <stdbool.h>

//struct for pin configurations
typedef struct{
	uint8_t PinNumber;
	uint8_t PinMode;
	uint8_t PinSpeed;
	uint8_t PinPUPDCtrl;
	uint8_t PinOPType;
	uint8_t PinALTFunc;
}GPIO_PinConfig_t;
////////////////////////////////////////////

//Structure for GPIO Handling
typedef struct{
	GPIO_REG_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;
////////////////////////////////////////////

//GPIO PIN NUMBERS
#define GPIO_PIN_NO0		0
#define GPIO_PIN_NO1		1
#define GPIO_PIN_NO2		2
#define GPIO_PIN_NO3		3
#define GPIO_PIN_NO4		4
#define GPIO_PIN_NO5		5
#define GPIO_PIN_NO6		6
#define GPIO_PIN_NO7		7
#define GPIO_PIN_NO8		8
#define GPIO_PIN_NO9		9
#define GPIO_PIN_NO10		10
#define GPIO_PIN_NO11		11
#define GPIO_PIN_NO12		12
#define GPIO_PIN_NO13		13
#define GPIO_PIN_NO14		14
#define GPIO_PIN_NO15		15
////////////////////////////////////////////


//PIN MODES
#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUTPUT 	1
#define GPIO_MODE_ALTFUN	2
#define GPIO_MODE_ANALOG	3

#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6
////////////////////////////////////////////


//PIN OUTPUT TYPES
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD 	1
///////////////////////////////////////////

//PIN SPEEDS
#define GPIO_LO_SPD			0
#define GPIO_MED_SPD		1
#define GPIO_FAST_SPD		2
#define GPIO_HI_SPD			3
//////////////////////////////////////////

//PIN PULLUP/PULLDOWN CONFIGURATION
#define GPIO_NO_PUPD		0
#define GPIO_PU				1
#define GPIO_PD				2
///////////////////////////////////////////

//PIN Initialization and De-Initialization
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_REG_t *pGPIOx);
///////////////////////////////////////////

//Peripheral Clock Control
void GPIO_PeriCLKCTRL(GPIO_REG_t* pGPIOx, uint8_t ENorDIS);
//////////////////////////////////////////


//GPIO READ/WRITE
bool GPIO_ReadInPin(GPIO_REG_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadInPort(GPIO_REG_t *pGPIOx);
void GPIO_WriteOutPin(GPIO_REG_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteOutPort(GPIO_REG_t *pGPIOx, uint16_t value);
void GPIO_TogglePin(GPIO_REG_t *pGPIOx, uint8_t PinNumber);
//////////////////////////////////////////

//GPIO IRQ Handling and COnfiguration
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDIS);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);
///////////////////////////////////////////


#endif /* INC_STM32F446RE_GPIO_DRIVERS_H_ */
