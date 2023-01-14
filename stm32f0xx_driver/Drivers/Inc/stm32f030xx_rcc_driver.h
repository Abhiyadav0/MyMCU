/*
 * stm32f030xx_rcc_driver.h
 *
 *  Created on: 17-May-2022
 *      Author: Abhi
 */

#ifndef INC_STM32F030XX_RCC_DRIVER_H_
#define INC_STM32F030XX_RCC_DRIVER_H_
#include "stm32f030xx.h"

//This returns the APB clock value
uint32_t RCC_GetPCLKValue(void);
uint32_t  RCC_GetPLLOutputClock(void);


#endif /* INC_STM32F030XX_RCC_DRIVER_H_ */
