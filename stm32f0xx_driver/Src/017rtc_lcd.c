/*
 * Test_rtc.c
 *
 *  Created on: 21-Oct-2022
 *      Author: Abhi
 */
#include<stdio.h>
#include "stm32f030xx.h"
#include "Lcd.h"

#define SYSTICK_TIM_CLK   8000000UL



static void mdelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000); i++);
}

void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}

void number_to_string(uint8_t num , char* buf)
{

	if(num < 10){
		buf[0] = '0';
		buf[1] = num+48;
	}else if(num >= 10 && num < 99)
	{
		buf[0] = (num/10) + 48;
		buf[1]= (num % 10) + 48;
	}
}



//hh:mm:ss
char* time_to_string(RTC_Time *rtc_time)
{
	static char buf[9];

	buf[2]= ':';
	buf[5]= ':';

	number_to_string(rtc_time->hour,buf);
	number_to_string(rtc_time->min,&buf[3]);
	number_to_string(rtc_time->seconds,&buf[6]);

	buf[8] = '\0';

	return buf;

}

//dd/mm/yy
char* date_to_string(RTC_Date *rtc_date)
{
	static char buf[9];

	buf[2]= '/';
	buf[5]= '/';

	number_to_string(rtc_date->day,buf);
	number_to_string(rtc_date->month,&buf[3]);
	number_to_string(rtc_date->year,&buf[6]);

	buf[8]= '\0';

	return buf;

}


int main()
{

       // init_systick_timer(1);
        RTC_Time X;
    	X.am_pm = 1;
    	X.hour = 6;
    	X.min = 00;
    	X.seconds = 0;

    	RTC_Date Y;
    	Y.day = 1;
    	Y.month = 10;
    	Y.year = 2021;
    	Y.week_day = Monday;

    	RTC_Init();
    	RTC_Set_Date(&Y);
    	RTC_Set_Time(&X, Time_Format_12Hour);
    	RTC_Start();
    	lcd_init();

    		        RTC_Get_Date(&Y);
    				RTC_Get_Time(&X);
    				lcd_set_cursor(1, 1);
    				lcd_print_string(time_to_string(&X));

    			    lcd_set_cursor(2, 1);
    				lcd_print_string(date_to_string(&Y));



    				while(1);
    }



void SysTick_Handler(void)
{
	    RTC_Time X;
	    RTC_Date Y;

	    RTC_Init();
		RTC_Set_Date(&Y);
	    RTC_Set_Time(&X, Time_Format_12Hour);
		RTC_Get_Date(&Y);
 		RTC_Get_Time(&X);
		lcd_set_cursor(1, 1);
		lcd_print_string(time_to_string(&X));
        lcd_set_cursor(2, 1);
    	lcd_print_string(date_to_string(&Y));



}


