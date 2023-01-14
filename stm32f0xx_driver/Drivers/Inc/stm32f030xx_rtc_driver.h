/*
 * stm32f030xx_rtc_driver.h
 *
 *  Created on: 30-Dec-2022
 *      Author: Abhi
 */

#ifndef INC_STM32F030XX_RTC_DRIVER_H_
#define INC_STM32F030XX_RTC_DRIVER_H_

#include "stm32f030xx.h"

#define Time_Format_24Hour	0
#define Time_Format_12Hour	1

#define Monday 		1
#define Tuesday 	2
#define Wednesday 	3
#define Thurday 	4
#define Friday 		5
#define Saturday	6
#define Sunday 		7

#define	January		1
#define	February	2
#define	March		3
#define	April		4
#define	May			5
#define	June		6
#define	July		7
#define	August		8
#define	September	9
#define	October		10
#define	November	11
#define	December	12

typedef struct RTC_Date{
	int day;
	int month;
	int year;
	int week_day;
}RTC_Date;

typedef struct RTC_Time{
	int am_pm;
	int hour;
	int min;
	int seconds;
	int milliseconds;
}RTC_Time;



void RTC_Init(void);
void RTC_Set_Time(RTC_Time *T, int time_format);
void RTC_Set_Date(RTC_Date *D);
void RTC_Start(void);

void RTC_Get_Time(RTC_Time *T);
void RTC_Get_Date(RTC_Date *D);


#endif /* INC_STM32F030XX_RTC_DRIVER_H_ */
