/*
 * lcd.h
 *
 *  Created on: Mar 5, 2023
 *      Author: jwizard
 */

#ifndef LCD_H_
#define LCD_H_

#include "stm32f446re.h"
#include "stm32f446re_gpio_drivers.h"

void lcd_send_command(uint8_t cmd);
void lcd_display_clear();
void lcd_print_string();
void lcd_print_char(uint8_t data);
void lcd_display_return_home();
void lcd_set_cursor(uint8_t row, uint8_t column);

void lcd_init(void);

void mdelay(uint32_t count);
void udelay(uint32_t count);


#define LCD_GPIO_PORT	GPIOC
#define LCD_GPIO_RS		GPIO_PIN_NO0
#define LCD_GPIO_RW		GPIO_PIN_NO1
#define LCD_GPIO_EN		GPIO_PIN_NO2
#define LCD_GPIO_D4		GPIO_PIN_NO3
#define LCD_GPIO_D5		GPIO_PIN_NO4
#define LCD_GPIO_D6		GPIO_PIN_NO5
#define LCD_GPIO_D7		GPIO_PIN_NO6


//LCD commands
#define LCD_CMD_4DL_2N_5X8F  		0x28 //4 bit data line. 2 rows, and 5x8 font
#define LCD_CMD_DON_CURON    		0x0E //Display on and Cursor on
#define LCD_CMD_INCADD       		0x06 //Increment RAM Address
#define LCD_CMD_DIS_CLEAR    		0X01 //Display Clear
#define LCD_CMD_DIS_RETURN_HOME  	0x02 //Display return home


#endif /* LCD_H_ */
