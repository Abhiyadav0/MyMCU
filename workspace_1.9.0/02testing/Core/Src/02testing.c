/*
 * 02testing.c
 *
 *  Created on: 03-May-2022
 *      Author: Abhi
 */

#include "stm32f030x8.h"
void delayMs(int n);

int main(void) {
RCC->AHBENR |= 1<<19;/* enable GPIOA clock */
RCC->AHBENR |= 1<<17;
GPIOC->MODER &= ~0x00000030; /* clear pin mode */
GPIOC->MODER |= 0x00000010; /* set pin to output mode */
GPIOA->MODER &= ~0x0C0000000;
while(1) {
if (GPIOA->IDR & 0x8000) /* if PC13 is high */
GPIOC->ODR = 0x00000004; /* turn off green LED */
else
GPIOC->ODR = 0x00000200; /* turn on green LED */
}} /* 16 MHz SYSCLK */
void delayMs(int n) {
int i;
for (; n > 0; n--)
for (i = 0; i < 3195; i++) ;
}




