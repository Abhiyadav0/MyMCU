//****NOTE******

/**
OpenOCD Debugger and Semi-hosting
======================================
Set the linker arguments 
-specs=rdimon.specs -lc -lrdimon

Add semi-hosting run command
monitor arm semihosting enable 

Add the below function call to main.c 
extern void initialise_monitor_handles(void);
initialise_monitor_handles();
*
**/

#include<stdio.h>
#include "stm32f030xx.h"
#include "Lcd.h"

extern void initialise_monitor_handles(void);

static void mdelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000); i++);
}


int main()
{

        initialise_monitor_handles();
	RTC_Time X;
	X.am_pm = 1;
	X.hour = 6;
	X.min = 53;
	X.seconds = 48;

	RTC_Date Y;
	Y.day = 05;
	Y.month = 03;
	Y.year = 2019;
	Y.week_day = Monday;

	RTC_Init();
	RTC_Set_Date(&Y);
	RTC_Set_Time(&X, Time_Format_12Hour);
	RTC_Start();
	while(1){
	                RTC_Get_Date(&Y);
                        RTC_Get_Time(&X);
			printf("Date: %d/%d/%d \r\n",Y.day,Y.month,Y.year);
			printf("Time: %d:%d:%d \r\n", X.hour, X.min, X.seconds);
			mdelay(1);
	}
	return 0;
}
