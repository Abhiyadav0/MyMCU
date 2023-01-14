 /*
 * 017rtc_lcd.c
 *
 *  Created on: 22-May-2022
 *      Author: Abhi
 */

#include<stdio.h>

#include "lcd.h"

#define SYSTICK_TIM_CLK   8000000UL

/* Enable this macro if you want to test RTC on LCD */
#define PRINT_LCD


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


char* get_day_of_week(uint8_t i)
{
	char* days[] = { "Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};

	return days[i-1];
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

	number_to_string(rtc_time->hours,buf);
	number_to_string(rtc_time->minutes,&buf[3]);
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

	number_to_string(rtc_date->date,buf);
	number_to_string(rtc_date->month,&buf[3]);
	number_to_string(rtc_date->year,&buf[6]);

	buf[8]= '\0';

	return buf;

}

static void mdelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000); i++);
}

int main(void)
{

	RTC_Time current_time;
	RTC_Date current_date;

#ifndef PRINT_LCD
	printf("RTC test\n");
#else
	lcd_init();

	lcd_print_string("RTC Test...");


	mdelay(2000);

	lcd_display_clear();
	lcd_display_return_home();
#endif

	if(RTC_Init()){
		printf("RTC init has failed\n");
		while(1);
	}

	init_systick_timer(1);


	current_date.date = 15;
	current_date.month = 1;
	current_date.year = 21;
	current_date.week_day = Friday;

	current_time.hours = 11;
	current_time.minutes = 59;
	current_time.seconds = 30;
	current_time.time_format = Time_Format_12Hour;

	RTC_Set_Date(&current_date);
	RTC_Set_Time(&current_time,Time_Format_12Hour);

	RTC_Get_Time(&current_time);
	RTC_Get_Date(&current_date);

	char *am_pm;
	if(current_time.time_format != Time_Format_24Hour){
		am_pm = (current_time.time_format) ? "PM" : "AM";
#ifndef PRINT_LCD
		printf("Current time = %s %s\n",time_to_string(&current_time),am_pm); // 04:25:41 PM
#else
		lcd_print_string(time_to_string(&current_time));
		lcd_print_string(am_pm);
#endif
	}else{
#ifndef PRINT_LCD
		printf("Current time = %s\n",time_to_string(&current_time)); // 04:25:41
#else
		lcd_print_string(time_to_string(&current_time));
#endif
	}

#ifndef PRINT_LCD
	printf("Current date = %s <%s>\n",date_to_string(&current_date), get_day_of_week(current_date.day));
#else
	lcd_set_cursor(2, 1);
	lcd_print_string(date_to_string(&current_date));
#endif


	while(1);

	return 0;
}

void SysTick_Handler(void)
{
	RTC_Time current_time;
	RTC_Date current_date;

	RTC_Set_Time(&current_time,Time_Format_12Hour);

	char *am_pm;
	if(current_time.time_format != Time_Format_24Hour){
		am_pm = (current_time.time_format) ? "PM" : "AM";
#ifndef PRINT_LCD
		printf("Current time = %s %s\n",time_to_string(&current_time),am_pm); // 04:25:41 PM
#else
		lcd_set_cursor(1, 1);
		lcd_print_string(time_to_string(&current_time));
		lcd_print_string(am_pm);
#endif

	}else{
#ifndef PRINT_LCD
		printf("Current time = %s\n",time_to_string(&current_time)); // 04:25:41
#else
		lcd_set_cursor(1, 1);
		lcd_print_string(time_to_string(&current_time));
#endif
	}

	RTC_Get_Date(&current_date);

#ifndef PRINT_LCD
	printf("Current date = %s <%s>\n",date_to_string(&current_date), get_day_of_week(current_date.day));
#else
	lcd_set_cursor(2, 1);
	lcd_print_string(date_to_string(&current_date));
	lcd_print_char('<');
	lcd_print_string(get_day_of_week(current_date.week_day));
	lcd_print_char('>');
#endif


}
