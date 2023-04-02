/*
 * rtc_lcd.c
 *
 *  Created on: Mar 5, 2023
 *      Author: jwizard
 */

#include <stdio.h>
#include "ds1307.h"
#include "lcd.h"

#define SYSTICK_TIM_CLK		16000000UL

void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}

char *get_day_of_week(uint8_t code){
	char *days[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
	return days[code];
}

//Convert numbers into a string format (ASCII)
void number_to_string(uint8_t num, char* buf){
	if(num < 10){
		buf[0] = '0';
		buf[1] = num+48;// covert to ASCII format
	}
	else if(num >= 10 && num <= 99){
		buf[0] = (num/10) + 48; // covert to ASCII format
		buf[1] = (num % 10) + 48; // covert to ASCII format
	}
}

//Time Format: hh:mm:ss
char *time_to_string(RTC_time_t *time){
	static char buf[9];

	buf[2] = ':';
	buf[5] = ':';

	number_to_string(time->hours,&buf[0]); 		//hh
	number_to_string(time->minutes,&buf[3]); 	//mm
	number_to_string(time->seconds,&buf[6]); 	//ss

	buf[8] = '\0';

	return buf;
}

//Date Format: mm/dd/yy
char *date_to_string(RTC_date_t *date){
	static char buf[9];

	buf[2] = '/';
	buf[5] = '/';

	number_to_string(date->month,buf); 		//mm
	number_to_string(date->date,&buf[3]); 	//dd
	number_to_string(date->year,&buf[6]); 	//yy

	buf[8] = '\0';

	return buf;
}

int main(void){
	RTC_time_t current_time;
	RTC_date_t current_date;

	printf("RTC Test\n");

	lcd_init();

	lcd_print_string("RTC Test");

	mdelay(2000);

	lcd_display_clear();
	lcd_display_return_home();

	if(ds1307_init()){
		printf("Initialization has failed\n");
		while(1);
	}

	init_systick_timer(1);

	current_date.day = WEDNESDAY;
	current_date.date = 29;
	current_date.month = 3;
	current_date.year = 23;

	current_time.hours = 22;
	current_time.minutes = 1;
	current_time.seconds = 0;
	current_time.time_format = TIME_FORMAT_24HRS;

	ds1307_set_current_date(&current_date);
	ds1307_set_current_time(&current_time);

	ds1307_get_current_date(&current_date);
	ds1307_get_current_time(&current_time);

	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS){
		am_pm = (current_time.time_format) ? "PM" : "AM";
		//printf("Current time: %s %s\n", time_to_string(&current_time), am_pm);
		lcd_print_string(time_to_string(&current_time));
		lcd_print_string(am_pm);
	}else{
		//printf("Current time: %s\n", time_to_string(&current_time));
		lcd_print_string(time_to_string(&current_time));
	}

	//printf("Current date: %s <%s>\n",date_to_string(&current_date), get_day_of_week(current_date.day));
	lcd_set_cursor(2, 1);
	lcd_print_string(date_to_string(&current_date));
	while(1);

	return 0;
}

void SysTick_Handler(void){
	RTC_time_t current_time;
	RTC_date_t current_date;
	ds1307_get_current_time(&current_time);
	ds1307_get_current_date(&current_date);
	lcd_set_cursor(1, 1);

	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS){
		am_pm = (current_time.time_format) ? "PM" : "AM";
		//printf("Current time: %s %s\n", time_to_string(&current_time), am_pm);
		lcd_print_string(time_to_string(&current_time));
		lcd_print_string(am_pm);
	}else{
		//printf("Current time: %s\n", time_to_string(&current_time));
		lcd_print_string(time_to_string(&current_time));
	}
	lcd_set_cursor(2, 1);
	//printf("Current date: %s <%s>\n",date_to_string(&current_date), get_day_of_week(current_date.day));
	lcd_print_string(date_to_string(&current_date));
	lcd_print_char(" ");
	lcd_print_string(get_day_of_week(current_date.day));
}


