/*
 * stm32f030xx_rtc_driver.c
 *
 *  Created on: 30-Dec-2022
 *      Author: Abhi
 */

#include "stm32f030xx_rtc_driver.h"

static uint8_t bcd_to_binary(uint8_t value);
static uint8_t binary_to_bcd(uint8_t value);



void RTC_Init(void)
{
	          PWR_PCLK_EN();                           //peripheral clock enable for power control register(PWR)
			  PWR -> CR |= PWR_CR_DBP;                             //DBP Set for RTC registers write access
			 /* clock selection in RCC. enable to operate RTC */
			  RCC->CSR |= 1<<0;                                    //LSI Clock enable
			 /*RTC clock source selection in RCC_CSR Register*/
			  RCC->BDCR |= 2<<8;                                   //LSI Clock selection for RTC
			 /*RTC clock enable*/
			  RCC->BDCR |= 1<<15;
			 /*Write access for RTC registers */
			  RTC->WPR = 0xCA;
			  RTC->WPR = 0x53;
			 /*Enable init phase */
			  RTC->ISR |= RTC_ISR_INIT;
			 /*Wait until it is allow to modify RTC register values */
			  while ((RTC->ISR & RTC_ISR_INITF) != RTC_ISR_INITF)
			 	  {
			 	  /* add time out here for a robust application */
			 	  }
			 /*set prescaler, 40kHz/128 => 312 Hz, 312Hz/312 => 1Hz */
			 RTC->PRER = 0x007F0137;
}

void RTC_Set_Time(RTC_Time *T, int time_format)
{
	int hrs,mnts,ses;
	uint32_t t;
	hrs = binary_to_bcd(T->hour);
	mnts = binary_to_bcd(T->min);
	ses = binary_to_bcd(T->seconds);
	t = hrs<< 16 |mnts << 8| ses << 0;
	RTC -> TR = t ;// time
}
void RTC_Set_Date(RTC_Date *D)
{
	int yrs,wd,mnts,dte;
	uint32_t d;
	D->year = D -> year - 2000;
	yrs = binary_to_bcd(D->year);
    wd = D->week_day;
    mnts = binary_to_bcd(D->month);
    dte = binary_to_bcd(D->day);
    d =  yrs << 16 | wd << 13  | mnts << 8  | dte << 0;
	RTC -> DR = d ;// time

}
void RTC_Start(void)
{
	RTC -> CR |= RTC_CR_FMT | RTC_CR_TSE; //Configure the time format
	RTC -> ISR &= ~RTC_ISR_INIT; //Exit initialization mode.
	RTC -> WPR = 0xFF;
	PWR -> CR &= ~PWR_CR_DBP;
}

void RTC_Get_Time(RTC_Time *T)
{
	uint32_t t;
	int seconds, mins, hours;
	t = RTC -> TR;
	hours=bcd_to_binary(T->hour);
    mins=bcd_to_binary(T->min);
    seconds= bcd_to_binary(T->seconds);
	}



void RTC_Get_Date(RTC_Date *D)
{
	uint32_t d;
	int date, month, year;
	int wd;
    d = RTC -> DR;
    year = 2000 + bcd_to_binary(D -> year);;
    month = bcd_to_binary(D -> month);
    date = bcd_to_binary(D -> day);
}


static uint8_t binary_to_bcd(uint8_t value)
{
	uint8_t m , n;
	uint8_t bcd;

	bcd = value;
	if(value >= 10)
	{
		m = value /10;
		n = value % 10;
		bcd = (m << 4) | n ;
	}

	return bcd;
}

static uint8_t bcd_to_binary(uint8_t value)
{
	uint8_t m , n;
	m = (uint8_t) ((value >> 4 ) * 10);
	n =  value & (uint8_t)0x0F;
	return (m+n);
}
