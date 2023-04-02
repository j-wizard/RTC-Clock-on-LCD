/*
 * lcd.c
 *
 *  Created on: Mar 5, 2023
 *      Author: jwizard
 */

#include "lcd.h"

static void Write_4_Bits(uint8_t value);
static void lcd_enable(void);


void lcd_send_command(uint8_t cmd){
	//RS = 0 for LCD Command
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	//RW = 0 for write
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	Write_4_Bits(cmd >> 4); //Higher nibble of command
	Write_4_Bits(cmd & 0x0F); // Lower nibble of command


}


void lcd_print_string(char *message)
{

      do
      {
          lcd_print_char((uint8_t)*message++);
      }
      while (*message != '\0');

}

void lcd_print_char(uint8_t data){
	//RS = 1 for LCD data
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

	//RW = 0 for write
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	Write_4_Bits(data >> 4); //Higher nibble of command
	Write_4_Bits(data & 0x0F); // Lower nibble of command
}

void lcd_init(void){
	//Configure GPIO pins used for LCD connections
	GPIO_Handle_t lcd_signal;
	lcd_signal.pGPIOx = LCD_GPIO_PORT;
	lcd_signal.GPIO_PinConfig.PinMode = GPIO_MODE_OUTPUT;
	lcd_signal.GPIO_PinConfig.PinOPType = GPIO_OP_TYPE_PP;
	lcd_signal.GPIO_PinConfig.PinPUPDCtrl = GPIO_NO_PUPD;
	lcd_signal.GPIO_PinConfig.PinSpeed = GPIO_FAST_SPD;

	lcd_signal.GPIO_PinConfig.PinNumber = LCD_GPIO_RS;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.PinNumber = LCD_GPIO_RW;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.PinNumber = LCD_GPIO_EN;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.PinNumber = LCD_GPIO_D4;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.PinNumber = LCD_GPIO_D5;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.PinNumber = LCD_GPIO_D6;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.PinNumber = LCD_GPIO_D7;
	GPIO_Init(&lcd_signal);

	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);


	//Do LCD Initialization
	mdelay(40); //Wait 40 milliseconds after LCD powers on

	//RS = 0 for LCD Command
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	//RW = 0 for write
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	//Write initial command when LCD powers on
	Write_4_Bits(0x3);

	//Delay at least 4.1 milliseconds
	mdelay(4);

	Write_4_Bits(0x3);

	//Wait 100 microseconds
	udelay(100);

	Write_4_Bits(0x3);

	Write_4_Bits(0x2);

	//function set command
	lcd_send_command(LCD_CMD_4DL_2N_5X8F);

	//disply ON and cursor ON
	lcd_send_command(LCD_CMD_DON_CURON);

	lcd_display_clear();

	//entry mode set
	lcd_send_command(LCD_CMD_INCADD);

}

static void Write_4_Bits(uint8_t value){
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_D4, (value >> 0) & 0x1);
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_D5, (value >> 1) & 0x1);
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_D6, (value >> 2) & 0x1);
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_D7, (value >> 3) & 0x1);

	lcd_enable();
}

void lcd_display_clear(void)
{
	//Display clear
	lcd_send_command(LCD_CMD_DIS_CLEAR);


	mdelay(2);
}

/*Cursor returns to home position */
void lcd_display_return_home(void)
{

	lcd_send_command(LCD_CMD_DIS_RETURN_HOME);
	/*
	 * check page number 24 of datasheet.
	 * return home command execution wait time is around 2ms
	 */
	mdelay(2);
}


/**
  *   Set Lcd to a specified location given by row and column information
  *   Row Number (1 to 2)
  *   Column Number (1 to 16) Assuming a 2 X 16 characters display
  */
void lcd_set_cursor(uint8_t row, uint8_t column)
{
  column--;
  switch (row)
  {
    case 1:
      /* Set cursor to 1st row address and add index*/
      lcd_send_command((column |= 0x80));
      break;
    case 2:
      /* Set cursor to 2nd row address and add index*/
        lcd_send_command((column |= 0xC0));
      break;
    default:
      break;
  }
}

static void lcd_enable(void){
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	udelay(10);
	GPIO_WriteOutPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	udelay(100);
}

void mdelay(uint32_t count){
	for(int i=0;i < (count *1000);i++);
}

void udelay(uint32_t count){
	for(int i=0;i < (count *1);i++);
}

