/*
 * stm32f446re.h
 *
 *  Created on: Jan 13, 2023
 *      Author: jwizard
 */

#ifndef INC_STM32F446RE_H_
#define INC_STM32F446RE_H_


#include <stdint.h>

#define _vol volatile

//ARM Cortex Mx Processor NVIC ISERx register addresses
#define NVIC_ISER0				(_vol uint32_t*) 0xE000E100
#define NVIC_ISER1				(_vol uint32_t*) 0xE000E104
#define NVIC_ISER2				(_vol uint32_t*) 0xE000E108
#define NVIC_ISER3				(_vol uint32_t*) 0xE000E10C

//ARM Cortex Mx Processor NVIC ICERx register addresses
#define NVIC_ICER0				(_vol uint32_t*) 0xE000E180
#define NVIC_ICER1				(_vol uint32_t*) 0xE000E184
#define NVIC_ICER2				(_vol uint32_t*) 0xE000E188
#define NVIC_ICER3				(_vol uint32_t*) 0xE000E18C


//Base of Interrupt priority register
#define NVIC_PR_BASE			(_vol uint32_t*) 0xE000E400


#define ENABLE 					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_SET				SET
#define FLAG_RESET				RESET


//Addresses of memories
#define FLASH_BASE 0x08000000U //base address of flash memory
#define SRAM_BASE  0x20000000U //base address of SRAM 1 and 2
#define ROM_BASE   0x1FFF0000U //base address of ROM


//Addresses of Buses
#define PERIPH_BASE 0x40000000U //base address of peripherals
#define APB1_BASE  PERIPH_BASE //base address of APB1 (TIM2)
#define APB2_BASE  0x40010000U //base address of APB2 (TIM1)
#define AHB1_BASE  0x40020000U //base address of AHB1 (GPIOA)
#define AHB2_BASE  0x50000000U //base address of AHB2 (USB OTG FS)


//Addresses of GPIOx
#define GPIOA_BASE (AHB1_BASE + 0x0000) // base address of GPIOA
#define GPIOB_BASE (AHB1_BASE + 0x0400) // base address of GPIOB
#define GPIOC_BASE (AHB1_BASE + 0x0800) // base address of GPIOC
#define GPIOD_BASE (AHB1_BASE + 0x0C00) // base address of GPIOD
#define GPIOE_BASE (AHB1_BASE + 0x1000) // base address of GPIOE
#define GPIOF_BASE (AHB1_BASE + 0x1400) // base address of GPIOF
#define GPIOG_BASE (AHB1_BASE + 0x1800) // base address of GPIOG
#define GPIOH_BASE (AHB1_BASE + 0x0C00) // base address of GPIOH

#define RCC_BASE   (AHB1_BASE + 0x3800) // base address of RCC


//Addresses of APB1 Peripherals

//SPI Peripherals
#define SPI2_BASE  (APB1_BASE + 0x3800)
#define SPI3_BASE  (APB1_BASE + 0x3C00)


//UART/USART Peripherals
#define USART2_BASE (APB1_BASE + 0x4400)
#define USART3_BASE (APB1_BASE + 0x4800)

#define UART4_BASE (APB1_BASE + 0x4C00)
#define UART5_BASE (APB1_BASE + 0x5000)


//I2C Peripherals
#define I2C1_BASE  (APB1_BASE + 0x5400)
#define I2C2_BASE  (APB1_BASE + 0x5800)
#define I2C3_BASE  (APB1_BASE + 0x5C00)


//Addresses of APB2 Peripherals

//USART Peripherals
#define USART1_BASE (APB2_BASE + 0x1000)
#define USART6_BASE (APB2_BASE + 0x1400)


//SPI Peripherals
#define SPI1_BASE   (APB2_BASE + 0x3000)
#define SPI4_BASE   (APB2_BASE + 0x3400)


#define SYSCFG_BASE (APB2_BASE + 0x3800)

#define EXTI_BASE   (APB2_BASE + 0x3C00)


///////////////////////////////////////////////////Peripheral Register Definition Structures//////////////////////////////////////////////////

typedef struct{
	_vol uint32_t MODER; 							//GPIO MODE REGISTER															0x00
	_vol uint32_t OTYPER; 							//GPIO Output Type Register 													0x04
	_vol uint32_t OSPEEDR; 							//GPIO Output Speed Register													0x08
	_vol uint32_t PUPDR; 							//GPIO Pullup/Pulldown Register													0x0C
	_vol uint32_t IDR; 								//GPIO Input Data Register														0x10
	_vol uint32_t ODR; 								//GPIO Output Data Register														0x14
	_vol uint32_t BSRR; 							// GPIO Bit Set/Bit Reset 														0x18
	_vol uint32_t LCKR; 							//GPIO Lock Register															0x1C
	_vol uint32_t AFRL; 							//GPIO Alternate Function Low Register											0x20
	_vol uint32_t AFRH; 							//GPIO Alternate Function High Register											0x24

}GPIO_REG_t;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct{
	_vol uint32_t CR;								//SPI Control Register 1														0x00
	_vol uint32_t CR2;								//SPI Control Register 2														0x04
	_vol uint32_t SR;								//SPI Status Register															0x08
	_vol uint32_t DR;								//SPI Data Register																0x0c
	_vol uint32_t CRCPR;							//SPI CRC Polynomial Register													0x10
	_vol uint32_t RXCRCR;							//SPI RX CRC Register 															0x14
	_vol uint32_t TXCRCR;							//SPI TX CRC Register															0x18
	_vol uint32_t I2SCFGR;							//SPI I2S Config Register														0x1C
	_vol uint32_t I2SPR;							//SPI I2S Prescaler Register													0x20

}SPI_REG_t;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct{
	_vol uint32_t CR1;								//I2C Control Register 1														0x00
	_vol uint32_t CR2;								//I2C Control Register 2														0x04
	_vol uint32_t OAR1;								//I2C OWN Address Register 1													0x08
	_vol uint32_t OAR2;								//I2C OWN Address Register 2													0x0C
	_vol uint32_t DR;								//I2C Data Register																0x10
	_vol uint32_t SR1;								//I2C Status Register 1															0x14
	_vol uint32_t SR2;								//I2C Status Register 2															0x18
	_vol uint32_t CCR;								//I2C Clock Control Register													0x1C
	_vol uint32_t TRISE;							//I2C TRISE Register															0x20
	_vol uint32_t FLTR;								//I2C FLTR Register																0x24

}I2C_REG_t;



//Define structures  for each GPIO
#define GPIOA					((GPIO_REG_t*) GPIOA_BASE)
#define GPIOB					((GPIO_REG_t*) GPIOB_BASE)
#define GPIOC					((GPIO_REG_t*) GPIOC_BASE)
#define GPIOD					((GPIO_REG_t*) GPIOD_BASE)
#define GPIOE					((GPIO_REG_t*) GPIOE_BASE)
#define GPIOF					((GPIO_REG_t*) GPIOF_BASE)
#define GPIOG					((GPIO_REG_t*) GPIOG_BASE)
#define GPIOH					((GPIO_REG_t*) GPIOH_BASE)


//Define structures for each SPI
#define SPI1					((SPI_REG_t*) SPI1_BASE)
#define SPI2					((SPI_REG_t*) SPI2_BASE)
#define SPI3					((SPI_REG_t*) SPI3_BASE)
#define SPI4					((SPI_REG_t*) SPI4_BASE)


//Define structures for each I2C
#define I2C1					((I2C_REG_t*) I2C1_BASE)
#define I2C2					((I2C_REG_t*) I2C2_BASE)
#define I2C3					((I2C_REG_t*) I2C2_BASE)

#define GPIO_BASE_TO_CODE(x)  (	(x == GPIOA) ? 0 :\
								(x == GPIOB) ? 1 :\
								(x == GPIOC) ? 2 :\
								(x == GPIOD) ? 3 :\
								(x == GPIOE) ? 4 :\
								(x == GPIOF) ? 5 :\
								(x == GPIOG) ? 6 :\
								(x == GPIOH) ? 7 :0	)


#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI5_10			40
#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34
#define IRQ_NO_I2C3_EV			72
#define IRQ_NO_I2C3_ER			73


#define NO_PR_BITS_IMPLEMENTED	4

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Struct for RCC datatype
typedef struct{
	_vol uint32_t RCC_CR;							//RCC Clock Control Register													0x00
	_vol uint32_t RCC_PLLCFGR;						//RCC PLL Configuration Register												0x04
	_vol uint32_t RCC_CFGR;							//RCC Clock Config Register														0x08
	_vol uint32_t RCC_CIR;							//RCC Clock Interrupt Register													0x0C
	_vol uint32_t RCC_AHB1RSTR;						//RCC AHB1 Peripheral Reset Register											0x10
	_vol uint32_t RCC_AHB2RSTR;						//RCC AHB2 Peripheral Reset Register											0x14
	_vol uint32_t RCC_AHB3RSTR;						//RCC AHB3 Peripheral Reset Register											0x18
	_vol uint32_t RESERVED1;						//																				0x1C
	_vol uint32_t RCC_APB1RSTR;						//RCC APB1 Peripheral Reset Register											0X20
	_vol uint32_t RCC_APB2RSTR;						//RCC APB2 Peripheral Reset Register											0X24
	_vol uint32_t RESERVED2;						//																				0X28
	_vol uint32_t RESERVED3;						//																				0X2C
	_vol uint32_t RCC_AHB1ENR;						//RCC AHB1 Peripheral Clock Enable Register										0x30
	_vol uint32_t RCC_AHB2ENR;						//RCC AHB2 Peripheral Clock Enable Register										0x34
	_vol uint32_t RCC_AHB3ENR;						//RCC AHB3 Peripheral Clock Enable Register										0x38
	_vol uint32_t RESERVED4;						//																				0x3C
	_vol uint32_t RCC_APB1ENR;						//RCC APB1 Peripheral Clock Enable Register										0x40
	_vol uint32_t RCC_APB2ENR;						//RCC APB2 Peripheral Clock Enable Register										0x44
	_vol uint32_t RESERVED5;						//																				0x48
	_vol uint32_t RESERVED6;						//																				0x4C
	_vol uint32_t RCC_AHB1LPENR;					//RCC AHB1 Peripheral Low Power Enable Register									0x50
	_vol uint32_t RCC_AHB2LPENR;					//RCC AHB2 Peripheral Low Power Enable Register									0x54
	_vol uint32_t RCC_AHB3LPENR;					//RCC AHB3 Peripheral Low Power ENABLE Register									0x58
	_vol uint32_t RESERVED7;						//																				0x5C
	_vol uint32_t RCC_APB1LPENR;					//RCC APB1 Peripheral Low Power Enable Register									0x60
	_vol uint32_t RCC_APB2LPENR;					//RCC APB2 Peripheral Low Power Enable Register									0x64
	_vol uint32_t RESERVED8;						//																				0x68
	_vol uint32_t RESERVED9;						//																				0x6C
	_vol uint32_t RCC_BDCR;							//RCC Backup Domain Control Register											0x70
	_vol uint32_t RCC_CSR;							//RCC Clock Control & Status Register											0x74
	_vol uint32_t RESERVED10;						//																				0x78
	_vol uint32_t RESERVED11;						//																				0x7C
	_vol uint32_t RCC_SSCGR;						//RCC Spread Spectrum Clock Generation Register									0x80
	_vol uint32_t RCC_PLLI2SCFGR;					//RCC PLLI2S Configuration Register												0x84
	_vol uint32_t RCC_PLLSAICFGR;					//RCC PLL Configuration Register												0x88
	_vol uint32_t RCC_DCKCFGR;						//RCC Dedicated Clock COnfiguration Register									0x8C
	_vol uint32_t RCC_CKGATENR;						//RCC Clocks Gated ENable Register												0x90
	_vol uint32_t RCC_DCKCFGR2;						//RCC Dedicated Clocks Configuration Register 2									0x94
}RCC_REG_t;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct{
	_vol uint32_t IMR;
	_vol uint32_t EMR;
	_vol uint32_t RTSR;
	_vol uint32_t FTSR;
	_vol uint32_t SWIER;
	_vol uint32_t PR;

}EXTI_REG_t;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct{
	_vol uint32_t MEMRMP;
	_vol uint32_t PMC;
	_vol uint32_t EXTICR1;
	_vol uint32_t EXTICR2;
	_vol uint32_t EXTICR3;
	_vol uint32_t EXTICR4;
	_vol uint32_t RESERVED[2];
	_vol uint32_t CMPCR;
	_vol uint32_t RESERVED2[2];
	_vol uint32_t CFGR;


}SYSCFG_REG_t;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define RCC								((RCC_REG_t*)RCC_BASE)
#define EXTI							((EXTI_REG_t*)EXTI_BASE)
#define SYSCFG							((SYSCFG_REG_t*)SYSCFG_BASE)

//Clock enable macros for GPIOx peripherals
#define GPIOA_CLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 4))
#define GPIOF_CLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 5))
#define GPIOG_CLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 6))
#define GPIOH_CLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 7))


//Clock enable macros for I2C peripherals
#define I2C1_CLK_EN()					(RCC->RCC_APB1ENR |= (1 << 21))
#define I2C2_CLK_EN()					(RCC->RCC_APB1ENR |= (1 << 22))
#define I2C3_CLK_EN()					(RCC->RCC_APB1ENR |= (1 << 23))

//Clock enable macros for SPI peripherals
#define SPI1_CLK_EN()					(RCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_CLK_EN()					(RCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_CLK_EN()					(RCC->RCC_APB1ENR |= (1 << 15))
#define SPI4_CLK_EN()					(RCC->RCC_APB2ENR |= (1 << 13))

//Clock enable macros for USART peripherals
#define USART1_CLK_EN()					(RCC->RCC_APB2ENR |= (1 << 4))
#define USART2_CLK_EN()					(RCC->RCC_APB1ENR |= (1 << 17))
#define USART3_CLK_EN()					(RCC->RCC_APB1ENR |= (1 << 18))
#define USART4_CLK_EN()					(RCC->RCC_APB1ENR |= (1 << 19))
#define USART5_CLK_EN()					(RCC->RCC_APB1ENR |= (1 << 20))
#define USART6_CLK_EN()					(RCC->RCC_APB1ENR |= (1 << 5))


//Clock enable macros for SYSCFG peripherals
#define SYSCFG_CLK_EN()					(RCC->RCC_APB2ENR |= (1 << 14))


//Clock disable macros for GPIO peripherals
#define GPIOA_CLK_DIS()					(RCC->RCC_AHB1ENR &= ~(1 << 0))
#define GPIOB_CLK_DIS()					(RCC->RCC_AHB1ENR &= ~(1 << 1))
#define GPIOC_CLK_DIS()					(RCC->RCC_AHB1ENR &= ~(1 << 2))
#define GPIOD_CLK_DIS()					(RCC->RCC_AHB1ENR &= ~(1 << 3))
#define GPIOE_CLK_DIS()					(RCC->RCC_AHB1ENR &= ~(1 << 4))
#define GPIOF_CLK_DIS()					(RCC->RCC_AHB1ENR &= ~(1 << 5))
#define GPIOG_CLK_DIS()					(RCC->RCC_AHB1ENR &= ~(1 << 6))
#define GPIOH_CLK_DIS()					(RCC->RCC_AHB1ENR &= ~(1 << 7))


//Clock disable macros for I2C peripherals
#define I2C1_CLK_DIS()					(RCC->RCC_APB1ENR &= ~(1 << 21))
#define I2C2_CLK_DIS()					(RCC->RCC_APB1ENR &= ~(1 << 22))
#define I2C3_CLK_DIS()					(RCC->RCC_APB1ENR &= ~(1 << 23))


//Clock disable macros for SPI peripherals
#define SPI1_CLK_DIS()					(RCC->RCC_APB2ENR &= ~(1 << 12))
#define SPI2_CLK_DIS()					(RCC->RCC_APB1ENR &= ~(1 << 14))
#define SPI3_CLK_DIS()					(RCC->RCC_APB1ENR &= ~(1 << 15))
#define SPI4_CLK_DIS()					(RCC->RCC_APB2ENR &= ~(1 << 13))


//Clock disable macros for USART peripherals
#define USART1_CLK_DIS()					(RCC->RCC_APB2ENR &= ~(1 << 4))
#define USART2_CLK_DIS()					(RCC->RCC_APB1ENR &= ~(1 << 17))
#define USART3_CLK_DIS()					(RCC->RCC_APB1ENR &= ~(1 << 18))
#define USART4_CLK_DIS()					(RCC->RCC_APB1ENR &= ~(1 << 19))
#define USART5_CLK_DIS()					(RCC->RCC_APB1ENR &= ~(1 << 20))
#define USART6_CLK_DIS()					(RCC->RCC_APB1ENR &= ~(1 << 5))

//RCC Reset for GPIO Ports
#define GPIOA_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 0)); (RCC->RCC_AHB1RSTR |= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 1)); (RCC->RCC_AHB1RSTR |= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 2)); (RCC->RCC_AHB1RSTR |= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 3)); (RCC->RCC_AHB1RSTR |= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 4)); (RCC->RCC_AHB1RSTR |= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 5)); (RCC->RCC_AHB1RSTR |= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 6)); (RCC->RCC_AHB1RSTR |= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 7)); (RCC->RCC_AHB1RSTR |= ~(1 << 7));} while(0)

//RCC Reset for SPI Registers
#define SPI1_REG_RESET()				do{(RCC->RCC_AHB2RSTR |= (1 << 12)); (RCC->RCC_AHB2RSTR |= ~(1 << 12));} while(0)
#define SPI2_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 14)); (RCC->RCC_AHB1RSTR |= ~(1 << 14));} while(0)
#define SPI3_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 15)); (RCC->RCC_AHB1RSTR |= ~(1 << 15));} while(0)
#define SPI4_REG_RESET()				do{(RCC->RCC_AHB2RSTR |= (1 << 13)); (RCC->RCC_AHB2RSTR |= ~(1 << 13));} while(0)

//RCC Reset for SPI Registers
#define I2C1_REG_RESET()				do{(RCC->RCC_APB1RSTR |= (1 << 21)); (RCC->RCC_APB1RSTR |= ~(1 << 21));} while(0)
#define I2C2_REG_RESET()				do{(RCC->RCC_APB1RSTR |= (1 << 22)); (RCC->RCC_APB1RSTR |= ~(1 << 22));} while(0)
#define I2C3_REG_RESET()				do{(RCC->RCC_APB1RSTR |= (1 << 23)); (RCC->RCC_APB1RSTR |= ~(1 << 23));} while(0)



//Clock disable macros for SYSCFG peripherals
#define SYSCFG_CLK_DIS()					(RCC->RCC_APB2ENR &= ~(1 << 14))



#endif /* INC_STM32F446RE_H_ */
