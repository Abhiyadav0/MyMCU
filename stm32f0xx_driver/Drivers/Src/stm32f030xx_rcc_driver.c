/*
 * stm32f030xx_rcc_driver.c
 *
 *  Created on: 17-May-2022
 *      Author: Abhi
 */


#include "stm32f030xx_rcc_driver.h"


uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB_PreScaler[4] = { 2, 4 , 8, 16};



uint32_t RCC_GetPCLKValue(void)
{
	uint32_t pclk1,SystemClk;

	uint8_t clksrc,temp,ahbp,apbp;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0 )
	{
		SystemClk = 8000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if (clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	//for ahb
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}



	//apb
	temp = ((RCC->CFGR >> 8 ) & 0x7);

	if(temp < 4)
	{
		apbp = 1;
	}else
	{
		apbp = APB_PreScaler[temp-4];
	}

	pclk1 =  (SystemClk / ahbp) /apbp;

	return pclk1;
}





uint32_t  RCC_GetPLLOutputClock()
{

	return 0;
}
