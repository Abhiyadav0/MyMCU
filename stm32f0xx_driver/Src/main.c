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
#include<string.h>
#include "stm32f030xx.h"
#include "Lcd.h"

char time[200];
char date[200];

char msg1[1024] = " Time...\n\r";
char msg2[1024] = " Date...\n\r";

USART_Handle_t usart2_handle;

void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}

void 	USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode =1;

	//USART2 TX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpios);

	//USART2 RX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpios);


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
	USART2_GPIOInit();
    USART2_Init();
    USART_PeripheralControl(USART2,ENABLE);

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

		    get_time();
            USART_SendData(&usart2_handle,(uint8_t*)time,strlen(time));
            USART_SendData(&usart2_handle,(uint8_t*)msg1,strlen(msg1));
            USART_SendData(&usart2_handle,(uint8_t*)date,strlen(date));
            USART_SendData(&usart2_handle,(uint8_t*)msg2,strlen(msg2));
			mdelay(500);
	}
	return 0;
}


