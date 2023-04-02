/*
 * ds1307.c
 *
 *  Created on: Mar 5, 2023
 *      Author: jwizard
 */

#include <stdint.h>
#include <string.h>
#include "ds1307.h"



static void ds1307_i2c_pin_config();
static void ds1307_i2c_config(void);

static void ds1307_write(uint8_t value,uint8_t address);
static uint8_t ds1307_read(uint8_t address);

static uint8_t binary_to_bcd(uint8_t value);
static uint8_t bcd_to_binary(uint8_t value);

I2C_Handle_t g_ds1307I2CHandle;

uint8_t ds1307_init(){
	//Initialize I2C pins
	ds1307_i2c_pin_config();

	//Initialize I2C peripheral
	ds1307_i2c_config();

	//Enable the I2C Peripheral
	I2C_PeripheralControl(DS1307_I2C, ENABLE);

	//Make clock halt (CH) = 0
	ds1307_write(0x00,DS1307_ADDR_SEC); //CH must be zero to enable clock

	uint8_t clock_state = ds1307_read(DS1307_ADDR_SEC);

	return (clock_state >> 7) & 0x1;
}

void ds1307_set_current_time(RTC_time_t* time){
	uint8_t seconds,hours;
	seconds = binary_to_bcd(time->seconds);
	seconds &= ~(1 << 7);
	ds1307_write(seconds, DS1307_ADDR_SEC);
	ds1307_write(binary_to_bcd(time->minutes), DS1307_ADDR_MIN);

	hours = binary_to_bcd(time->hours);

	if(time->time_format == TIME_FORMAT_24HRS){
		hours &= ~(1 << 6);
	}else{
		hours |= (1 << 6);
		hours = (time->time_format == TIME_FORMAT_12HRS_PM) ? hours |= (1 << 5) : hours & ~(1 << 5);
	}

	ds1307_write(hours, DS1307_ADDR_HRS);


}
void ds1307_get_current_time(RTC_time_t* time){
	uint8_t seconds, hours;

	seconds = ds1307_read(DS1307_ADDR_SEC);
	seconds &= ~(1 << 7);
	time->seconds = bcd_to_binary(seconds);

	time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));

	hours = ds1307_read(DS1307_ADDR_HRS);
	if(hours & (1 << 6)){
		//12 hour format
		if(hours & (1 << 5)){
			//PM
			time->time_format = TIME_FORMAT_12HRS_PM;
			hours &= ~(0x3 << 5);//Clear 6 and 5
		}
		else{
			//AM
			time->time_format = TIME_FORMAT_12HRS_AM;
			hours &= ~(0x3 << 5);//Clear 6 and 5
		}
	}
	else{
		//24 hour format
		time->time_format = TIME_FORMAT_24HRS;
	}
	time->hours = bcd_to_binary(hours);


}

void ds1307_set_current_date(RTC_date_t* date){
	ds1307_write(binary_to_bcd(date->date), DS1307_ADDR_DATE);

	ds1307_write(binary_to_bcd(date->month), DS1307_ADDR_MONTH);

	ds1307_write(binary_to_bcd(date->year), DS1307_ADDR_YEAR);

	ds1307_write(binary_to_bcd(date->day), DS1307_ADDR_DAY);
}
void ds1307_get_current_date(RTC_date_t* date){
	date->date = bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));

	date->month = bcd_to_binary(ds1307_read(DS1307_ADDR_MONTH));

	date->year = bcd_to_binary(ds1307_read(DS1307_ADDR_YEAR));

	date->day = bcd_to_binary(ds1307_read(DS1307_ADDR_DAY));
}

static void ds1307_i2c_pin_config(void){
	GPIO_Handle_t i2c_sda, i2c_scl;

	memset(&i2c_sda, 0, sizeof(i2c_sda));
	memset(&i2c_sda, 0, sizeof(i2c_scl));

	i2c_sda.pGPIOx = DS1307_I2C_PORT;
	i2c_sda.GPIO_PinConfig.PinALTFunc = 4;
	i2c_sda.GPIO_PinConfig.PinMode = GPIO_MODE_ALTFUN;
	i2c_sda.GPIO_PinConfig.PinNumber = DS1307_I2C_SDA_PIN;
	i2c_sda.GPIO_PinConfig.PinOPType = GPIO_OP_TYPE_OD;
	i2c_sda.GPIO_PinConfig.PinPUPDCtrl = DS1307_I2C_PUPD;
	i2c_sda.GPIO_PinConfig.PinSpeed = GPIO_FAST_SPD;

	GPIO_Init(&i2c_sda);

	i2c_scl.pGPIOx = DS1307_I2C_PORT;
	i2c_scl.GPIO_PinConfig.PinALTFunc = 4;
	i2c_scl.GPIO_PinConfig.PinMode = GPIO_MODE_ALTFUN;
	i2c_scl.GPIO_PinConfig.PinNumber = DS1307_I2C_SCL_PIN;
	i2c_scl.GPIO_PinConfig.PinOPType = GPIO_OP_TYPE_OD;
	i2c_scl.GPIO_PinConfig.PinPUPDCtrl = DS1307_I2C_PUPD;
	i2c_scl.GPIO_PinConfig.PinSpeed = GPIO_FAST_SPD;

	GPIO_Init(&i2c_scl);
}

static void ds1307_i2c_config(void){
	g_ds1307I2CHandle.pI2Cx = DS1307_I2C;
	g_ds1307I2CHandle.I2C_Config.ACKControl = I2C_ACK_ENABLE;
	g_ds1307I2CHandle.I2C_Config.SCLSpeed = DS1307_I2C_SPEED;
	I2C_Init(&g_ds1307I2CHandle);
}

static void ds1307_write(uint8_t value,uint8_t address){
	uint8_t tx[2];
	tx[0] = address;
	tx[1] = value;
	I2CMasterSendData(&g_ds1307I2CHandle, tx, 2, DS1307_I2C_ADDRESS,0);

}

static uint8_t ds1307_read(uint8_t address){
	uint8_t data;
	I2CMasterSendData(&g_ds1307I2CHandle, &address, 1, DS1307_I2C_ADDRESS, 0);
	I2CMasterReceiveData(&g_ds1307I2CHandle, &data, 1, DS1307_I2C_ADDRESS, 0);

	return data;
}

static uint8_t binary_to_bcd(uint8_t value){
	uint8_t m,n;
	uint8_t bcd = value;

	if(value >= 10){
		m = value / 10; //get left digit
		n = value % 10; //get right digit
		bcd = (m << 4) | n;
	}

	return bcd;
}

static uint8_t bcd_to_binary(uint8_t value){
	uint8_t m,n;
	uint8_t bin = value;

	m = (uint8_t)(value >> 4) * 10;
	n = value & (uint8_t)0x0F;

	return (m+n);
}




