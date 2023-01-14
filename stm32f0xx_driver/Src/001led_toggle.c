
/*
 * 002led_button.c
 *
 *  Created on: Feb 1, 2019
 *      Author: admin
 */


#include "stm32f030xx.h"



void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}


int main(void)
{

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD	;
	GPIO_Init(&GpioLed);


	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&GpioLed);



	GPIO_PeriClockControl(GPIOC, ENABLE);
	lcd_init();

	lcd_print_string("RTC Test...");
	lcd_display_clear();
	lcd_print_string("UPLOADING DATA...");
	lcd_display_clear();

	lcd_print_string("VIRUS DETECTED...");

	while(1)
	{
		/*GPIO_ToggleOutputPin(GPIOC,GPIO_PIN_NO_1);
		delay();
		GPIO_ToggleOutputPin(GPIOC,GPIO_PIN_NO_2);
		delay();
*/
	}
	return 0;
}
