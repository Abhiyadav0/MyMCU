/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Abhi
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************/


#include<stdio.h>
#include "stm32f030xx.h"
#include "Lcd.h"

#define SYSTICK_TIM_CLK   8000000UL
extern void initialise_monitor_handles(void);

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

static void mdelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000); i++);
}


int main()
{


	init_systick_timer(1);
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


void SysTick_Handler(void)
{
	    RTC_Time X;
	    RTC_Date Y;

	        RTC_Init();
	    	RTC_Set_Date(&Y);
	    	RTC_Set_Time(&X, Time_Format_12Hour);
	    	RTC_Start();

	    	        RTC_Get_Date(&Y);
	    			RTC_Get_Time(&X);
	    			printf("Date: %d/%d/%d \r\n",Y.day,Y.month,Y.year);
	    			printf("Time: %d:%d:%d \r\n", X.hour, X.min, X.seconds);
	    			mdelay(1);



}
