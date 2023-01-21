/*
 * 006spi_tx_testing.c
 *
 *  Created on: Feb 10, 2019
 *      Author: admin
 */


#include<string.h>
#include "stm32f030xx.h"


#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW

void delay(void)
{
	// this will introduce ~200ms delay when system clock is 16MHz
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

int main(void)
{

	GPIO_Handle_t GpioLed, GPIOBtn;

	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GPIOBtn,0,sizeof(GpioLed));

	//this is led gpio configuration
		GpioLed.pGPIOx = GPIOC;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
		GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_PeriClockControl(GPIOC,ENABLE);

		GPIO_Init(&GpioLed);


	//this is btn gpio configuration
		GPIOBtn.pGPIOx = GPIOA;
		GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
		GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
		GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

		GPIO_PeriClockControl(GPIOA,ENABLE);

		GPIO_Init(&GPIOBtn);

	GPIO_WriteToOutputPin(GPIOA,GPIO_PIN_NO_15,GPIO_PIN_RESET);
	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI4_15,NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI4_15,ENABLE);

    while(1);

}


void EXTI4_15_IRQHandler(void)
{
        delay(); //200ms . wait till button de-bouncing gets over
	GPIO_IRQHandling(GPIO_PIN_NO_15); //clear the pending event from exti line
	GPIO_ToggleOutputPin(GPIOC,GPIO_PIN_NO_1);
}
