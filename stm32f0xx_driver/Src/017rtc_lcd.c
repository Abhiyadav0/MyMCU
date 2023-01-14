/*
 * test.c
 *
 *  Created on: 10-Dec-2022
 *      Author: Abhi
 */

#include<stdio.h>
#include<string.h>
#include "stm32f030xx.h"
#include "Lcd.h"

char time[200];
char date[200];

static void mdelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000); i++);
}


void get_time(void){

	 RTC_Time X;
	 RTC_Date Y;
	 RTC_Get_Time(&X);
	 RTC_Get_Date(&Y);

	sprintf((char*)time,"%02d:%02d:%02d",X.hour,X.min,X.seconds);
	sprintf((char*)date,"%02d-%02d-%02d",Y.day,Y.month, Y.year );

}


int main()
{



	        RTC_Time X;
		X.am_pm = 1;
		X.hour = 6;
		X.min = 53;
		X.seconds = 48;

		RTC_Date Y;
		Y.day = 05;
		Y.month = 01;
		Y.year = 2022;
		Y.week_day = Monday;

		RTC_Init();
		RTC_Set_Date(&Y);
		RTC_Set_Time(&X, Time_Format_12Hour);
		RTC_Start();
		lcd_init();




    				 while(1)
    				    {
    					 get_time();
    					 lcd_set_cursor(1, 1);
    					 lcd_print_string(time);
                                         lcd_set_cursor(2, 1);
    					 lcd_print_string(date);
                                         mdelay(500);

    				    }
    }



