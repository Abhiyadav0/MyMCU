/*
 * stm32f030xx.h
 *
 *  Created on: Apr 30, 2022
 *      Author: Abhi
 */

#ifndef INC_STM32F030XX_H_
#define INC_STM32F030XX_H_
#include<stddef.h>
#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))



/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER          ( (__vo uint32_t*)0xE000E100 )



/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER 			((__vo uint32_t*)0xE000E180)



/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR						0x08000000U   		/*!<explain this macro briefly here  */
#define SRAM_BASEADDR						0x20000000U  		/*!<explain this macro briefly here  */
#define ROM_BASEADDR						0x1FFFEC00U         /*!<explain this macro briefly here  */
#define SRAM 								SRAM_BASEADDR       /*!<explain this macro briefly here  */


/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR               0x40000000UL
#define APBPERIPH_BASEADDR            PERIPH_BASEADDR
#define AHBPERIPH_BASEADDR           (PERIPH_BASEADDR + 0x00020000UL)
#define AHB2PERIPH_BASEADDR          (PERIPH_BASEADDR + 0x08000000UL)
/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 * TODO : Complete for all other peripherals
 */

#define GPIOA_BASEADDR            (AHB2PERIPH_BASEADDR + 0x00000000UL)
#define GPIOB_BASEADDR            (AHB2PERIPH_BASEADDR + 0x00000400UL)
#define GPIOC_BASEADDR            (AHB2PERIPH_BASEADDR + 0x00000800UL)
#define GPIOD_BASEADDR            (AHB2PERIPH_BASEADDR + 0x00000C00UL)
#define GPIOF_BASEADDR            (AHB2PERIPH_BASEADDR + 0x00001400UL)


/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO : Complete for all other peripherals
 */

#define RCC_BASEADDR              (AHBPERIPH_BASEADDR + 0x00001000UL)







/*
 * Base addresses of peripherals which are hanging on APB bus
 * TODO : Complete for all other peripherals
 */
#define TIM3_BASEADDR             (APBPERIPH_BASEADDR + 0x00000400UL)
#define TIM6_BASEADDR             (APBPERIPH_BASEADDR + 0x00001000UL)
#define TIM14_BASEADDR            (APBPERIPH_BASEADDR + 0x00002000UL)
#define RTC_BASEADDR              (APBPERIPH_BASEADDR + 0x00002800UL)

#define IWDG_BASEADDR             (APBPERIPH_BASEADDR + 0x00003000UL)
#define SPI2_BASEADDR             (APBPERIPH_BASEADDR + 0x00003800UL)
#define USART2_BASEADDR           (APBPERIPH_BASEADDR + 0x00004400UL)
#define I2C1_BASEADDR             (APBPERIPH_BASEADDR + 0x00005400UL)
#define I2C2_BASEADDR             (APBPERIPH_BASEADDR + 0x00005800UL)
#define PWR_BASEADDR              (APBPERIPH_BASEADDR + 0x00007000UL)
#define SYSCFG_BASEADDR           (APBPERIPH_BASEADDR + 0x00010000UL)
#define EXTI_BASEADDR             (APBPERIPH_BASEADDR + 0x00010400UL)
#define ADC1_BASEADDR             (APBPERIPH_BASEADDR + 0x00012400UL)
#define ADC_BASEADDR              (APBPERIPH_BASEADDR + 0x00012708UL)
#define TIM1_BASEADDR             (APBPERIPH_BASEADDR + 0x00012C00UL)
#define SPI1_BASEADDR             (APBPERIPH_BASEADDR + 0x00013000UL)
#define USART1_BASEADDR           (APBPERIPH_BASEADDR + 0x00013800UL)
#define TIM15_BASEADDR            (APBPERIPH_BASEADDR + 0x00014000UL)
#define TIM16_BASEADDR            (APBPERIPH_BASEADDR + 0x00014400UL)
#define TIM17_BASEADDR            (APBPERIPH_BASEADDR + 0x00014800UL)
#define DBGMCU_BASEADDR           (APBPERIPH_BASEADDR + 0x00015800UL)
#define USART3_BASEADDR           (APBPERIPH_BASEADDR + 0x00004800UL)
#define USART4_BASEADDR           (APBPERIPH_BASEADDR + 0x00004C00UL)
#define USART5_BASEADDR           (APBPERIPH_BASEADDR + 0x00005000UL)
#define USART6_BASEADDR           (APBPERIPH_BASEADDR + 0x00011400UL)


/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

typedef struct
{
	__vo uint32_t MODER;                        /*!< GPIO port mode register,                    	                                       Address offset: 0x00      */
	__vo uint32_t OTYPER;                       /*!< GPIO port output type register     			                                       Address offset: 0x04      */
	__vo uint32_t OSPEEDR;                      /*!< GPIO port output speed register     			                                       Address offset: 0x08      */
	__vo uint32_t PUPDR;                        /*!<GPIO port pull-up/pull-down register     	                                 		   Address offset: 0x0C      */
	__vo uint32_t IDR;                          /*!< GPIO port input data register     			                                           Address offset: 0x10      */
	__vo uint32_t ODR;                          /*!<GPIO port output data register     		                                               Address offset: 0x14      */
	__vo uint32_t BSRR;                         /*!< GPIO port bit set/reset register     		                                           Address offset: 0x1A      */
	__vo uint32_t LCKR;                         /*!< GPIO port configuration lock register    		                                       Address offset: 0x1C      */
	__vo uint32_t AFR[2];					 /*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
	__vo uint32_t BRR;                          /*!<GPIO port bit reset register     			                                             Address offset: 0x28     */
}GPIO_RegDef_t;



/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
  __vo  uint32_t CR;            /*!< RCC clock control register,                                   Address offset: 0x00 */
  __vo uint32_t CFGR;       /*!< RCC clock configuration register,                            Address offset: 0x04 */
  __vo uint32_t CIR;        /*!< RCC clock interrupt register,                                Address offset: 0x08 */
  __vo uint32_t APB2RSTR;   /*!< RCC APB2 peripheral reset register,                          Address offset: 0x0C */
  __vo uint32_t APB1RSTR;   /*!< RCC APB1 peripheral reset register,                          Address offset: 0x10 */
  __vo uint32_t AHBENR;     /*!< RCC AHB peripheral clock register,                           Address offset: 0x14 */
  __vo uint32_t APB2ENR;    /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x18 */
  __vo uint32_t APB1ENR;    /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x1C */
  __vo uint32_t BDCR;       /*!< RCC Backup domain control register,                          Address offset: 0x20 */
  __vo uint32_t CSR;        /*!< RCC clock control & status register,                         Address offset: 0x24 */
  __vo uint32_t AHBRSTR;    /*!< RCC AHB peripheral reset register,                           Address offset: 0x28 */
  __vo uint32_t CFGR2;      /*!< RCC clock configuration register 2,                          Address offset: 0x2C */
  __vo uint32_t CFGR3;      /*!< RCC clock configuration register 3,                          Address offset: 0x30 */
  __vo uint32_t CR2;        /*!< RCC clock control register 2,                                Address offset: 0x34 */

} RCC_RegDef_t;



/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo  uint32_t IMR;          /*!<EXTI Interrupt mask register,                 Address offset: 0x00 */
	__vo uint32_t EMR;          /*!<EXTI Event mask register,                     Address offset: 0x04 */
	__vo uint32_t RTSR;         /*!<EXTI Rising trigger selection register ,      Address offset: 0x08 */
	__vo uint32_t FTSR;         /*!<EXTI Falling trigger selection register,      Address offset: 0x0C */
	__vo uint32_t SWIER;        /*!<EXTI Software interrupt event register,       Address offset: 0x10 */
	__vo uint32_t PR;           /*!<EXTI Pending register,                        Address offset: 0x14 */

}EXTI_RegDef_t;


/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;        /*!< SPI Control register 1 (not used in I2S mode),      Address offset: 0x00 */
	__vo uint32_t CR2;        /*!< SPI Control register 2,                             Address offset: 0x04 */
	__vo uint32_t SR;         /*!< SPI Status register,                                Address offset: 0x08 */
	__vo uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
	__vo uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
	__vo uint32_t RXCRCR;     /*!< SPI Rx CRC register (not used in I2S mode),         Address offset: 0x14 */
	__vo uint32_t TXCRCR;     /*!< SPI Tx CRC register (not used in I2S mode),         Address offset: 0x18 */
	__vo uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
} SPI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo  uint32_t CFGR1;       /*!< SYSCFG configuration register 1,                           Address offset: 0x00 */
	__vo   uint32_t RESERVED;    /*!< Reserved,                                                                  0x04 */
	__vo uint32_t EXTICR[4];   /*!< SYSCFG external interrupt configuration register,     Address offset: 0x14-0x08 */
	__vo uint32_t CFGR2;       /*!< SYSCFG configuration register 2,                           Address offset: 0x18 */
} SYSCFG_RegDef_t;


/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
  __vo  uint32_t CR1;     /*!< I2C Control register 1,            Address offset: 0x00 */
  __vo uint32_t CR2;      /*!< I2C Control register 2,            Address offset: 0x04 */
  __vo uint32_t OAR1;     /*!< I2C Own address 1 register,        Address offset: 0x08 */
  __vo uint32_t OAR2;     /*!< I2C Own address 2 register,        Address offset: 0x0C */
  __vo uint32_t TIMINGR;  /*!< I2C Timing register,               Address offset: 0x10 */
  __vo uint32_t TIMEOUTR; /*!< I2C Timeout register,              Address offset: 0x14 */
  __vo uint32_t ISR;      /*!< I2C Interrupt and status register, Address offset: 0x18 */
  __vo uint32_t ICR;      /*!< I2C Interrupt clear register,      Address offset: 0x1C */
  __vo uint32_t PECR;     /*!< I2C PEC register,                  Address offset: 0x20 */
  __vo uint32_t RXDR;     /*!< I2C Receive data register,         Address offset: 0x24 */
  __vo uint32_t TXDR;     /*!< I2C Transmit data register,        Address offset: 0x28 */
}I2C_RegDef_t;

/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t CR1;    /*!< USART Control register 1,                 Address offset: 0x00 */
	__vo uint32_t CR2;    /*!< USART Control register 2,                 Address offset: 0x04 */
	__vo uint32_t CR3;    /*!< USART Control register 3,                 Address offset: 0x08 */
	__vo uint32_t BRR;    /*!< USART Baud rate register,                 Address offset: 0x0C */
	__vo uint32_t GTPR;   /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
	__vo uint32_t RTOR;   /*!< USART Receiver Time Out register,         Address offset: 0x14 */
	__vo uint32_t RQR;    /*!< USART Request register,                   Address offset: 0x18 */
	__vo uint32_t ISR;    /*!< USART Interrupt and status register,      Address offset: 0x1C */
	__vo uint32_t ICR;    /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
	__vo uint16_t RDR;    /*!< USART Receive Data register,              Address offset: 0x24 */
	     uint16_t RESERVED1;  /*!< Reserved, 0x26                                                 */
	__vo uint16_t TDR;    /*!< USART Transmit Data register,             Address offset: 0x28 */
	     uint16_t RESERVED2;  /*!< Reserved, 0x2A                                                 */
} USART_RegDef_t;


/*
 * peripheral register definition structure for PWR
 */
typedef struct
{
	__vo uint32_t CR;    /*!< Power control register,                         Address offset: 0x00 */
	__vo uint32_t CSR;    /*!< Power control/status register,                 Address offset: 0x04 */

} PWR_RegDef_t;



typedef struct
{
	   __vo uint32_t TR;         /*!< RTC time register,                                         Address offset: 0x00 */
       __vo uint32_t DR;         /*!< RTC date register,                                         Address offset: 0x04 */
       __vo uint32_t CR;         /*!< RTC control register,                                      Address offset: 0x08 */
	   __vo uint32_t ISR;        /*!< RTC initialization and status register,                    Address offset: 0x0C */
	   __vo uint32_t PRER;       /*!< RTC prescaler register,                                    Address offset: 0x10 */
          uint32_t RESERVED1;    /*!< Reserved,                                                  Address offset: 0x14 */
          uint32_t RESERVED2;    /*!< Reserved,                                                  Address offset: 0x18 */
       __vo uint32_t ALRMAR;     /*!< RTC alarm A register,                                      Address offset: 0x1C */
          uint32_t RESERVED3;    /*!< Reserved,                                                  Address offset: 0x20 */
       __vo uint32_t WPR;        /*!< RTC write protection register,                             Address offset: 0x24 */
       __vo uint32_t SSR;        /*!< RTC sub second register,                                   Address offset: 0x28 */
       __vo uint32_t SHIFTR;     /*!< RTC shift control register,                                Address offset: 0x2C */
       __vo uint32_t TSTR;       /*!< RTC time stamp time register,                              Address offset: 0x30 */
       __vo uint32_t TSDR;       /*!< RTC time stamp date register,                              Address offset: 0x34 */
       __vo uint32_t TSSSR;      /*!< RTC time-stamp sub second register,                        Address offset: 0x38 */
       __vo uint32_t CALR;       /*!< RTC calibration register,                                  Address offset: 0x3C */
       __vo uint32_t TAFCR;      /*!< RTC tamper and alternate function configuration register,  Address offset: 0x40 */
       __vo uint32_t ALRMASSR;   /*!< RTC alarm A sub second register,                           Address offset: 0x44 */
} RTC_RegDef_t;




/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)


#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1  				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  				((SPI_RegDef_t*)SPI2_BASEADDR)


#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)


#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
#define USART4  		    ((USART_RegDef_t*)USART4_BASEADDR)
#define USART5  			((USART_RegDef_t*)USART5_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)

#define PWR  			    ((PWR_RegDef_t*)PWR_BASEADDR)


#define RTC  			    ((RTC_RegDef_t*)RTC_BASEADDR)


/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()    	(RCC->AHBENR |= (1 << 17))
#define GPIOB_PCLK_EN()		(RCC->AHBENR |= (1 << 18))
#define GPIOC_PCLK_EN()		(RCC->AHBENR |= (1 << 19))
#define GPIOD_PCLK_EN()		(RCC->AHBENR |= (1 << 20))
#define GPIOF_PCLK_EN()		(RCC->AHBENR |= (1 << 22))



/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))



/*
 * Clock Enable Macros for SPIx peripheralsbu
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))



/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 14))
#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() (RCC->APB1ENR |= (1 << 18))
#define USART4_PCCK_EN() (RCC->APB1ENR |= (1 << 19))
#define USART5_PCCK_EN() (RCC->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN() (RCC->APB2ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 0))



/*
 * Clock Disable Macros for SPIx peripherals
 */
#define PWR_PCLK_EN() (RCC->APB1ENR |= (1 << 28))



/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()     (RCC->AHBENR &= ~(1 << 17))
#define GPIOB_PCLK_DI()		(RCC->AHBENR &= ~(1 << 18))
#define GPIOC_PCLK_DI()		(RCC->AHBENR &= ~(1 << 19))
#define GPIOD_PCLK_DI()		(RCC->AHBENR &= ~(1 << 20))
#define GPIOF_PCLK_DI()		(RCC->AHBENR &= ~(1 << 22))

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCCK_DI() (RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCCK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCCK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define USART4_PCCK_DI() (RCC->APB1ENR &= ~(1 << 19))
#define USART5_PCCK_DI() (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCCK_DI() (RCC->APB2ENR &= ~(1 << 5))



/*
 * Clock Disable Macros for PWR peripherals
 */
#define PWR_PCLK_DI() (RCC->APB1ENR &= ~(1 << 28))



/*
 * Clock Disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 0))

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 17)); (RCC->AHBRSTR &= ~(1 << 17)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 18)); (RCC->AHBRSTR &= ~(1 << 18)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 19)); (RCC->AHBRSTR &= ~(1 << 19)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 20)); (RCC->AHBRSTR &= ~(1 << 20)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 22)); (RCC->AHBRSTR &= ~(1 << 22)); }while(0)

/*
 *  Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->AHBRSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->AHBRSTR &= ~(1 << 14)); }while(0)


/*
 *  Macros to reset I2Cx peripherals
 */
#define I2C1_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->AHBRSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->AHBRSTR &= ~(1 << 22)); }while(0)

/*
 *  Macros to reset USARTx peripherals
 */
#define USART1_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 14)); (RCC->APB2RSTR &= ~(1 << 14)); }while(0)
#define USART2_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); }while(0)
#define USART3_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18)); }while(0)
#define USART4_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19)); }while(0)
#define USART5_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20)); }while(0)
#define USART6_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 5));  (RCC->APB2RSTR &= ~(1 << 5));  }while(0)


/*
 *  Macros to reset PWR
 *   peripherals
 */
#define PWR_REG_RESET()                  do{ (RCC->APB1RSTR |= (1 << 28)); (RCC->AHBRSTR &= ~(1 << 28)); }while(0)



/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 5
 * ) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
										(x == GPIOF)?5:0)



/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 *
 */

#define IRQ_NO_EXTI0_1 		    5
#define IRQ_NO_EXTI2_3		    6
#define IRQ_NO_EXTI4_15         7
#define IRQ_NO_EXTI15_10 	    40

#define IRQ_NO_SPI1		     	25
#define IRQ_NO_SPI2             26

#define IRQ_NO_I2C1             30
#define IRQ_NO_I2C2             31
#define IRQ_NO_USART1	        27
#define IRQ_NO_USART2	        28
#define IRQ_NO_USART3_4_5_6	    29


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15


//some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET          RESET
#define FLAG_SET 			SET


/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_NSSP			 	    3
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
//#define I2C_CR1_START 					8
//#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10
#define I2C_CR2_START 					13
#define I2C_CR2_STOP  				 	14
#define I2C_CR2_ACK 				 	15
/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */


#define USART_CR1_UE					0
#define USART_CR1_RE 					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M0					12
#define USART_CR1_MME 					13
#define USART_CR1_CMIE                  14
#define USART_CR1_OVER8  				15

/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				4
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_CLKEN                 11
#define USART_CR2_STOP   				12
#define USART_CR2_SWAP                  15
#define USART_CR2_RXINV                 16
#define USART_CR2_TXINV					17
#define USART_CR2_DATAINV				18
#define USART_CR2_MSBFIRST				19



/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_HDSEL   				3
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11
#define USART_CR3_OVRDI					12
#define USART_CR3_DDRE					13
#define USART_CR3_DEM					14
#define USART_CR3_DEP					15

/*
 * Bit position definitions USART_ISR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_CTSIF					9
#define USART_SR_CTS        			10
#define USART_SR_RTOF					11
#define USART_SR_ABRE					14
#define USART_SR_ABRF					15
#define USART_SR_BUSY					16
#define USART_SR_CMF					17
#define USART_SR_SBKF					18
#define USART_SR_RWU					19


/*****************************************************************************/
/*                                                                           */
/*                          Power Control (PWR)                              */
/*                                                                           */
/*****************************************************************************/

/* Note: No specific macro feature on this device */


/********************  Bit definition for PWR_CR register  *******************/
#define PWR_CR_LPDS_Pos            (0U)
#define PWR_CR_LPDS_Msk            (0x1UL << PWR_CR_LPDS_Pos)                   /*!< 0x00000001 */
#define PWR_CR_LPDS                PWR_CR_LPDS_Msk                             /*!< Low-power Deepsleep */
#define PWR_CR_PDDS_Pos            (1U)
#define PWR_CR_PDDS_Msk            (0x1UL << PWR_CR_PDDS_Pos)                   /*!< 0x00000002 */
#define PWR_CR_PDDS                PWR_CR_PDDS_Msk                             /*!< Power Down Deepsleep */
#define PWR_CR_CWUF_Pos            (2U)
#define PWR_CR_CWUF_Msk            (0x1UL << PWR_CR_CWUF_Pos)                   /*!< 0x00000004 */
#define PWR_CR_CWUF                PWR_CR_CWUF_Msk                             /*!< Clear Wakeup Flag */
#define PWR_CR_CSBF_Pos            (3U)
#define PWR_CR_CSBF_Msk            (0x1UL << PWR_CR_CSBF_Pos)                   /*!< 0x00000008 */
#define PWR_CR_CSBF                PWR_CR_CSBF_Msk                             /*!< Clear Standby Flag */
#define PWR_CR_DBP_Pos             (8U)
#define PWR_CR_DBP_Msk             (0x1UL << PWR_CR_DBP_Pos)                    /*!< 0x00000100 */
#define PWR_CR_DBP                 PWR_CR_DBP_Msk                              /*!< Disable Backup Domain write protection */

/*******************  Bit definition for PWR_CSR register  *******************/
#define PWR_CSR_WUF_Pos            (0U)
#define PWR_CSR_WUF_Msk            (0x1UL << PWR_CSR_WUF_Pos)                   /*!< 0x00000001 */
#define PWR_CSR_WUF                PWR_CSR_WUF_Msk                             /*!< Wakeup Flag */
#define PWR_CSR_SBF_Pos            (1U)
#define PWR_CSR_SBF_Msk            (0x1UL << PWR_CSR_SBF_Pos)                   /*!< 0x00000002 */
#define PWR_CSR_SBF                PWR_CSR_SBF_Msk                             /*!< Standby Flag */

#define PWR_CSR_EWUP1_Pos          (8U)
#define PWR_CSR_EWUP1_Msk          (0x1UL << PWR_CSR_EWUP1_Pos)                 /*!< 0x00000100 */
#define PWR_CSR_EWUP1              PWR_CSR_EWUP1_Msk                           /*!< Enable WKUP pin 1 */
#define PWR_CSR_EWUP2_Pos          (9U)
#define PWR_CSR_EWUP2_Msk          (0x1UL << PWR_CSR_EWUP2_Pos)                 /*!< 0x00000200 */
#define PWR_CSR_EWUP2              PWR_CSR_EWUP2_Msk                           /*!< Enable WKUP pin 2 */




/*****************************************************************************/
/*                                                                           */
/*                           Real-Time Clock (RTC)                           */
/*                                                                           */
/*****************************************************************************/




/********************  Bits definition for RTC_CR register  ******************/
#define RTC_CR_COE_Pos               (23U)
#define RTC_CR_COE_Msk               (0x1UL << RTC_CR_COE_Pos)                  /*!< 0x00800000 */
#define RTC_CR_COE                   RTC_CR_COE_Msk
#define RTC_CR_OSEL_Pos              (21U)
#define RTC_CR_OSEL_Msk              (0x3UL << RTC_CR_OSEL_Pos)                 /*!< 0x00600000 */
#define RTC_CR_OSEL                  RTC_CR_OSEL_Msk
#define RTC_CR_OSEL_0                (0x1UL << RTC_CR_OSEL_Pos)                 /*!< 0x00200000 */
#define RTC_CR_OSEL_1                (0x2UL << RTC_CR_OSEL_Pos)                 /*!< 0x00400000 */
#define RTC_CR_POL_Pos               (20U)
#define RTC_CR_POL_Msk               (0x1UL << RTC_CR_POL_Pos)                  /*!< 0x00100000 */
#define RTC_CR_POL                   RTC_CR_POL_Msk
#define RTC_CR_COSEL_Pos             (19U)
#define RTC_CR_COSEL_Msk             (0x1UL << RTC_CR_COSEL_Pos)                /*!< 0x00080000 */
#define RTC_CR_COSEL                 RTC_CR_COSEL_Msk
#define RTC_CR_BKP_Pos               (18U)
#define RTC_CR_BKP_Msk               (0x1UL << RTC_CR_BKP_Pos)                  /*!< 0x00040000 */
#define RTC_CR_BKP                   RTC_CR_BKP_Msk
#define RTC_CR_SUB1H_Pos             (17U)
#define RTC_CR_SUB1H_Msk             (0x1UL << RTC_CR_SUB1H_Pos)                /*!< 0x00020000 */
#define RTC_CR_SUB1H                 RTC_CR_SUB1H_Msk
#define RTC_CR_ADD1H_Pos             (16U)
#define RTC_CR_ADD1H_Msk             (0x1UL << RTC_CR_ADD1H_Pos)                /*!< 0x00010000 */
#define RTC_CR_ADD1H                 RTC_CR_ADD1H_Msk
#define RTC_CR_TSIE_Pos              (15U)
#define RTC_CR_TSIE_Msk              (0x1UL << RTC_CR_TSIE_Pos)                 /*!< 0x00008000 */
#define RTC_CR_TSIE                  RTC_CR_TSIE_Msk
#define RTC_CR_ALRAIE_Pos            (12U)
#define RTC_CR_ALRAIE_Msk            (0x1UL << RTC_CR_ALRAIE_Pos)               /*!< 0x00001000 */
#define RTC_CR_ALRAIE                RTC_CR_ALRAIE_Msk
#define RTC_CR_TSE_Pos               (11U)
#define RTC_CR_TSE_Msk               (0x1UL << RTC_CR_TSE_Pos)                  /*!< 0x00000800 */
#define RTC_CR_TSE                   RTC_CR_TSE_Msk
#define RTC_CR_ALRAE_Pos             (8U)
#define RTC_CR_ALRAE_Msk             (0x1UL << RTC_CR_ALRAE_Pos)                /*!< 0x00000100 */
#define RTC_CR_ALRAE                 RTC_CR_ALRAE_Msk
#define RTC_CR_FMT_Pos               (6U)
#define RTC_CR_FMT_Msk               (0x1UL << RTC_CR_FMT_Pos)                  /*!< 0x00000040 */
#define RTC_CR_FMT                   RTC_CR_FMT_Msk
#define RTC_CR_BYPSHAD_Pos           (5U)
#define RTC_CR_BYPSHAD_Msk           (0x1UL << RTC_CR_BYPSHAD_Pos)              /*!< 0x00000020 */
#define RTC_CR_BYPSHAD               RTC_CR_BYPSHAD_Msk
#define RTC_CR_REFCKON_Pos           (4U)
#define RTC_CR_REFCKON_Msk           (0x1UL << RTC_CR_REFCKON_Pos)              /*!< 0x00000010 */
#define RTC_CR_REFCKON               RTC_CR_REFCKON_Msk
#define RTC_CR_TSEDGE_Pos            (3U)
#define RTC_CR_TSEDGE_Msk            (0x1UL << RTC_CR_TSEDGE_Pos)               /*!< 0x00000008 */
#define RTC_CR_TSEDGE                RTC_CR_TSEDGE_Msk

/* Legacy defines */
#define RTC_CR_BCK_Pos               RTC_CR_BKP_Pos
#define RTC_CR_BCK_Msk               RTC_CR_BKP_Msk
#define RTC_CR_BCK                   RTC_CR_BKP

/********************  Bits definition for RTC_ISR register  *****************/
#define RTC_ISR_RECALPF_Pos          (16U)
#define RTC_ISR_RECALPF_Msk          (0x1UL << RTC_ISR_RECALPF_Pos)             /*!< 0x00010000 */
#define RTC_ISR_RECALPF              RTC_ISR_RECALPF_Msk
#define RTC_ISR_TAMP2F_Pos           (14U)
#define RTC_ISR_TAMP2F_Msk           (0x1UL << RTC_ISR_TAMP2F_Pos)              /*!< 0x00004000 */
#define RTC_ISR_TAMP2F               RTC_ISR_TAMP2F_Msk
#define RTC_ISR_TAMP1F_Pos           (13U)
#define RTC_ISR_TAMP1F_Msk           (0x1UL << RTC_ISR_TAMP1F_Pos)              /*!< 0x00002000 */
#define RTC_ISR_TAMP1F               RTC_ISR_TAMP1F_Msk
#define RTC_ISR_TSOVF_Pos            (12U)
#define RTC_ISR_TSOVF_Msk            (0x1UL << RTC_ISR_TSOVF_Pos)               /*!< 0x00001000 */
#define RTC_ISR_TSOVF                RTC_ISR_TSOVF_Msk
#define RTC_ISR_TSF_Pos              (11U)
#define RTC_ISR_TSF_Msk              (0x1UL << RTC_ISR_TSF_Pos)                 /*!< 0x00000800 */
#define RTC_ISR_TSF                  RTC_ISR_TSF_Msk
#define RTC_ISR_ALRAF_Pos            (8U)
#define RTC_ISR_ALRAF_Msk            (0x1UL << RTC_ISR_ALRAF_Pos)               /*!< 0x00000100 */
#define RTC_ISR_ALRAF                RTC_ISR_ALRAF_Msk
#define RTC_ISR_INIT_Pos             (7U)
#define RTC_ISR_INIT_Msk             (0x1UL << RTC_ISR_INIT_Pos)                /*!< 0x00000080 */
#define RTC_ISR_INIT                 RTC_ISR_INIT_Msk
#define RTC_ISR_INITF_Pos            (6U)
#define RTC_ISR_INITF_Msk            (0x1UL << RTC_ISR_INITF_Pos)               /*!< 0x00000040 */
#define RTC_ISR_INITF                RTC_ISR_INITF_Msk
#define RTC_ISR_RSF_Pos              (5U)
#define RTC_ISR_RSF_Msk              (0x1UL << RTC_ISR_RSF_Pos)                 /*!< 0x00000020 */
#define RTC_ISR_RSF                  RTC_ISR_RSF_Msk
#define RTC_ISR_INITS_Pos            (4U)
#define RTC_ISR_INITS_Msk            (0x1UL << RTC_ISR_INITS_Pos)               /*!< 0x00000010 */
#define RTC_ISR_INITS                RTC_ISR_INITS_Msk
#define RTC_ISR_SHPF_Pos             (3U)
#define RTC_ISR_SHPF_Msk             (0x1UL << RTC_ISR_SHPF_Pos)                /*!< 0x00000008 */
#define RTC_ISR_SHPF                 RTC_ISR_SHPF_Msk
#define RTC_ISR_ALRAWF_Pos           (0U)
#define RTC_ISR_ALRAWF_Msk           (0x1UL << RTC_ISR_ALRAWF_Pos)              /*!< 0x00000001 */
#define RTC_ISR_ALRAWF               RTC_ISR_ALRAWF_Msk

/********************  Bits definition for RTC_PRER register  ****************/
#define RTC_PRER_PREDIV_A_Pos        (16U)
#define RTC_PRER_PREDIV_A_Msk        (0x7FUL << RTC_PRER_PREDIV_A_Pos)          /*!< 0x007F0000 */
#define RTC_PRER_PREDIV_A            RTC_PRER_PREDIV_A_Msk
#define RTC_PRER_PREDIV_S_Pos        (0U)
#define RTC_PRER_PREDIV_S_Msk        (0x7FFFUL << RTC_PRER_PREDIV_S_Pos)        /*!< 0x00007FFF */
#define RTC_PRER_PREDIV_S            RTC_PRER_PREDIV_S_Msk

/********************  Bits definition for RTC_ALRMAR register  **************/
#define RTC_ALRMAR_MSK4_Pos          (31U)
#define RTC_ALRMAR_MSK4_Msk          (0x1UL << RTC_ALRMAR_MSK4_Pos)             /*!< 0x80000000 */
#define RTC_ALRMAR_MSK4              RTC_ALRMAR_MSK4_Msk
#define RTC_ALRMAR_WDSEL_Pos         (30U)
#define RTC_ALRMAR_WDSEL_Msk         (0x1UL << RTC_ALRMAR_WDSEL_Pos)            /*!< 0x40000000 */
#define RTC_ALRMAR_WDSEL             RTC_ALRMAR_WDSEL_Msk
#define RTC_ALRMAR_DT_Pos            (28U)
#define RTC_ALRMAR_DT_Msk            (0x3UL << RTC_ALRMAR_DT_Pos)               /*!< 0x30000000 */
#define RTC_ALRMAR_DT                RTC_ALRMAR_DT_Msk
#define RTC_ALRMAR_DT_0              (0x1UL << RTC_ALRMAR_DT_Pos)               /*!< 0x10000000 */
#define RTC_ALRMAR_DT_1              (0x2UL << RTC_ALRMAR_DT_Pos)               /*!< 0x20000000 */
#define RTC_ALRMAR_DU_Pos            (24U)
#define RTC_ALRMAR_DU_Msk            (0xFUL << RTC_ALRMAR_DU_Pos)               /*!< 0x0F000000 */
#define RTC_ALRMAR_DU                RTC_ALRMAR_DU_Msk
#define RTC_ALRMAR_DU_0              (0x1UL << RTC_ALRMAR_DU_Pos)               /*!< 0x01000000 */
#define RTC_ALRMAR_DU_1              (0x2UL << RTC_ALRMAR_DU_Pos)               /*!< 0x02000000 */
#define RTC_ALRMAR_DU_2              (0x4UL << RTC_ALRMAR_DU_Pos)               /*!< 0x04000000 */
#define RTC_ALRMAR_DU_3              (0x8UL << RTC_ALRMAR_DU_Pos)               /*!< 0x08000000 */
#define RTC_ALRMAR_MSK3_Pos          (23U)
#define RTC_ALRMAR_MSK3_Msk          (0x1UL << RTC_ALRMAR_MSK3_Pos)             /*!< 0x00800000 */
#define RTC_ALRMAR_MSK3              RTC_ALRMAR_MSK3_Msk
#define RTC_ALRMAR_PM_Pos            (22U)
#define RTC_ALRMAR_PM_Msk            (0x1UL << RTC_ALRMAR_PM_Pos)               /*!< 0x00400000 */
#define RTC_ALRMAR_PM                RTC_ALRMAR_PM_Msk
#define RTC_ALRMAR_HT_Pos            (20U)
#define RTC_ALRMAR_HT_Msk            (0x3UL << RTC_ALRMAR_HT_Pos)               /*!< 0x00300000 */
#define RTC_ALRMAR_HT                RTC_ALRMAR_HT_Msk
#define RTC_ALRMAR_HT_0              (0x1UL << RTC_ALRMAR_HT_Pos)               /*!< 0x00100000 */
#define RTC_ALRMAR_HT_1              (0x2UL << RTC_ALRMAR_HT_Pos)               /*!< 0x00200000 */
#define RTC_ALRMAR_HU_Pos            (16U)
#define RTC_ALRMAR_HU_Msk            (0xFUL << RTC_ALRMAR_HU_Pos)               /*!< 0x000F0000 */
#define RTC_ALRMAR_HU                RTC_ALRMAR_HU_Msk
#define RTC_ALRMAR_HU_0              (0x1UL << RTC_ALRMAR_HU_Pos)               /*!< 0x00010000 */
#define RTC_ALRMAR_HU_1              (0x2UL << RTC_ALRMAR_HU_Pos)               /*!< 0x00020000 */
#define RTC_ALRMAR_HU_2              (0x4UL << RTC_ALRMAR_HU_Pos)               /*!< 0x00040000 */
#define RTC_ALRMAR_HU_3              (0x8UL << RTC_ALRMAR_HU_Pos)               /*!< 0x00080000 */
#define RTC_ALRMAR_MSK2_Pos          (15U)
#define RTC_ALRMAR_MSK2_Msk          (0x1UL << RTC_ALRMAR_MSK2_Pos)             /*!< 0x00008000 */
#define RTC_ALRMAR_MSK2              RTC_ALRMAR_MSK2_Msk
#define RTC_ALRMAR_MNT_Pos           (12U)
#define RTC_ALRMAR_MNT_Msk           (0x7UL << RTC_ALRMAR_MNT_Pos)              /*!< 0x00007000 */
#define RTC_ALRMAR_MNT               RTC_ALRMAR_MNT_Msk
#define RTC_ALRMAR_MNT_0             (0x1UL << RTC_ALRMAR_MNT_Pos)              /*!< 0x00001000 */
#define RTC_ALRMAR_MNT_1             (0x2UL << RTC_ALRMAR_MNT_Pos)              /*!< 0x00002000 */
#define RTC_ALRMAR_MNT_2             (0x4UL << RTC_ALRMAR_MNT_Pos)              /*!< 0x00004000 */
#define RTC_ALRMAR_MNU_Pos           (8U)
#define RTC_ALRMAR_MNU_Msk           (0xFUL << RTC_ALRMAR_MNU_Pos)              /*!< 0x00000F00 */
#define RTC_ALRMAR_MNU               RTC_ALRMAR_MNU_Msk
#define RTC_ALRMAR_MNU_0             (0x1UL << RTC_ALRMAR_MNU_Pos)              /*!< 0x00000100 */
#define RTC_ALRMAR_MNU_1             (0x2UL << RTC_ALRMAR_MNU_Pos)              /*!< 0x00000200 */
#define RTC_ALRMAR_MNU_2             (0x4UL << RTC_ALRMAR_MNU_Pos)              /*!< 0x00000400 */
#define RTC_ALRMAR_MNU_3             (0x8UL << RTC_ALRMAR_MNU_Pos)              /*!< 0x00000800 */
#define RTC_ALRMAR_MSK1_Pos          (7U)
#define RTC_ALRMAR_MSK1_Msk          (0x1UL << RTC_ALRMAR_MSK1_Pos)             /*!< 0x00000080 */
#define RTC_ALRMAR_MSK1              RTC_ALRMAR_MSK1_Msk
#define RTC_ALRMAR_ST_Pos            (4U)
#define RTC_ALRMAR_ST_Msk            (0x7UL << RTC_ALRMAR_ST_Pos)               /*!< 0x00000070 */
#define RTC_ALRMAR_ST                RTC_ALRMAR_ST_Msk
#define RTC_ALRMAR_ST_0              (0x1UL << RTC_ALRMAR_ST_Pos)               /*!< 0x00000010 */
#define RTC_ALRMAR_ST_1              (0x2UL << RTC_ALRMAR_ST_Pos)               /*!< 0x00000020 */
#define RTC_ALRMAR_ST_2              (0x4UL << RTC_ALRMAR_ST_Pos)               /*!< 0x00000040 */
#define RTC_ALRMAR_SU_Pos            (0U)
#define RTC_ALRMAR_SU_Msk            (0xFUL << RTC_ALRMAR_SU_Pos)               /*!< 0x0000000F */
#define RTC_ALRMAR_SU                RTC_ALRMAR_SU_Msk
#define RTC_ALRMAR_SU_0              (0x1UL << RTC_ALRMAR_SU_Pos)               /*!< 0x00000001 */
#define RTC_ALRMAR_SU_1              (0x2UL << RTC_ALRMAR_SU_Pos)               /*!< 0x00000002 */
#define RTC_ALRMAR_SU_2              (0x4UL << RTC_ALRMAR_SU_Pos)               /*!< 0x00000004 */
#define RTC_ALRMAR_SU_3              (0x8UL << RTC_ALRMAR_SU_Pos)               /*!< 0x00000008 */

/********************  Bits definition for RTC_WPR register  *****************/
#define RTC_WPR_KEY_Pos              (0U)
#define RTC_WPR_KEY_Msk              (0xFFUL << RTC_WPR_KEY_Pos)                /*!< 0x000000FF */
#define RTC_WPR_KEY                  RTC_WPR_KEY_Msk

/********************  Bits definition for RTC_SSR register  *****************/
#define RTC_SSR_SS_Pos               (0U)
#define RTC_SSR_SS_Msk               (0xFFFFUL << RTC_SSR_SS_Pos)               /*!< 0x0000FFFF */
#define RTC_SSR_SS                   RTC_SSR_SS_Msk

/********************  Bits definition for RTC_SHIFTR register  **************/
#define RTC_SHIFTR_SUBFS_Pos         (0U)
#define RTC_SHIFTR_SUBFS_Msk         (0x7FFFUL << RTC_SHIFTR_SUBFS_Pos)         /*!< 0x00007FFF */
#define RTC_SHIFTR_SUBFS             RTC_SHIFTR_SUBFS_Msk
#define RTC_SHIFTR_ADD1S_Pos         (31U)
#define RTC_SHIFTR_ADD1S_Msk         (0x1UL << RTC_SHIFTR_ADD1S_Pos)            /*!< 0x80000000 */
#define RTC_SHIFTR_ADD1S             RTC_SHIFTR_ADD1S_Msk

/********************  Bits definition for RTC_TSTR register  ****************/
#define RTC_TSTR_PM_Pos              (22U)
#define RTC_TSTR_PM_Msk              (0x1UL << RTC_TSTR_PM_Pos)                 /*!< 0x00400000 */
#define RTC_TSTR_PM                  RTC_TSTR_PM_Msk
#define RTC_TSTR_HT_Pos              (20U)
#define RTC_TSTR_HT_Msk              (0x3UL << RTC_TSTR_HT_Pos)                 /*!< 0x00300000 */
#define RTC_TSTR_HT                  RTC_TSTR_HT_Msk
#define RTC_TSTR_HT_0                (0x1UL << RTC_TSTR_HT_Pos)                 /*!< 0x00100000 */
#define RTC_TSTR_HT_1                (0x2UL << RTC_TSTR_HT_Pos)                 /*!< 0x00200000 */
#define RTC_TSTR_HU_Pos              (16U)
#define RTC_TSTR_HU_Msk              (0xFUL << RTC_TSTR_HU_Pos)                 /*!< 0x000F0000 */
#define RTC_TSTR_HU                  RTC_TSTR_HU_Msk
#define RTC_TSTR_HU_0                (0x1UL << RTC_TSTR_HU_Pos)                 /*!< 0x00010000 */
#define RTC_TSTR_HU_1                (0x2UL << RTC_TSTR_HU_Pos)                 /*!< 0x00020000 */
#define RTC_TSTR_HU_2                (0x4UL << RTC_TSTR_HU_Pos)                 /*!< 0x00040000 */
#define RTC_TSTR_HU_3                (0x8UL << RTC_TSTR_HU_Pos)                 /*!< 0x00080000 */
#define RTC_TSTR_MNT_Pos             (12U)
#define RTC_TSTR_MNT_Msk             (0x7UL << RTC_TSTR_MNT_Pos)                /*!< 0x00007000 */
#define RTC_TSTR_MNT                 RTC_TSTR_MNT_Msk
#define RTC_TSTR_MNT_0               (0x1UL << RTC_TSTR_MNT_Pos)                /*!< 0x00001000 */
#define RTC_TSTR_MNT_1               (0x2UL << RTC_TSTR_MNT_Pos)                /*!< 0x00002000 */
#define RTC_TSTR_MNT_2               (0x4UL << RTC_TSTR_MNT_Pos)                /*!< 0x00004000 */
#define RTC_TSTR_MNU_Pos             (8U)
#define RTC_TSTR_MNU_Msk             (0xFUL << RTC_TSTR_MNU_Pos)                /*!< 0x00000F00 */
#define RTC_TSTR_MNU                 RTC_TSTR_MNU_Msk
#define RTC_TSTR_MNU_0               (0x1UL << RTC_TSTR_MNU_Pos)                /*!< 0x00000100 */
#define RTC_TSTR_MNU_1               (0x2UL << RTC_TSTR_MNU_Pos)                /*!< 0x00000200 */
#define RTC_TSTR_MNU_2               (0x4UL << RTC_TSTR_MNU_Pos)                /*!< 0x00000400 */
#define RTC_TSTR_MNU_3               (0x8UL << RTC_TSTR_MNU_Pos)                /*!< 0x00000800 */
#define RTC_TSTR_ST_Pos              (4U)
#define RTC_TSTR_ST_Msk              (0x7UL << RTC_TSTR_ST_Pos)                 /*!< 0x00000070 */
#define RTC_TSTR_ST                  RTC_TSTR_ST_Msk
#define RTC_TSTR_ST_0                (0x1UL << RTC_TSTR_ST_Pos)                 /*!< 0x00000010 */
#define RTC_TSTR_ST_1                (0x2UL << RTC_TSTR_ST_Pos)                 /*!< 0x00000020 */
#define RTC_TSTR_ST_2                (0x4UL << RTC_TSTR_ST_Pos)                 /*!< 0x00000040 */
#define RTC_TSTR_SU_Pos              (0U)
#define RTC_TSTR_SU_Msk              (0xFUL << RTC_TSTR_SU_Pos)                 /*!< 0x0000000F */
#define RTC_TSTR_SU                  RTC_TSTR_SU_Msk
#define RTC_TSTR_SU_0                (0x1UL << RTC_TSTR_SU_Pos)                 /*!< 0x00000001 */
#define RTC_TSTR_SU_1                (0x2UL << RTC_TSTR_SU_Pos)                 /*!< 0x00000002 */
#define RTC_TSTR_SU_2                (0x4UL << RTC_TSTR_SU_Pos)                 /*!< 0x00000004 */
#define RTC_TSTR_SU_3                (0x8UL << RTC_TSTR_SU_Pos)                 /*!< 0x00000008 */

/********************  Bits definition for RTC_TSDR register  ****************/
#define RTC_TSDR_WDU_Pos             (13U)
#define RTC_TSDR_WDU_Msk             (0x7UL << RTC_TSDR_WDU_Pos)                /*!< 0x0000E000 */
#define RTC_TSDR_WDU                 RTC_TSDR_WDU_Msk
#define RTC_TSDR_WDU_0               (0x1UL << RTC_TSDR_WDU_Pos)                /*!< 0x00002000 */
#define RTC_TSDR_WDU_1               (0x2UL << RTC_TSDR_WDU_Pos)                /*!< 0x00004000 */
#define RTC_TSDR_WDU_2               (0x4UL << RTC_TSDR_WDU_Pos)                /*!< 0x00008000 */
#define RTC_TSDR_MT_Pos              (12U)
#define RTC_TSDR_MT_Msk              (0x1UL << RTC_TSDR_MT_Pos)                 /*!< 0x00001000 */
#define RTC_TSDR_MT                  RTC_TSDR_MT_Msk
#define RTC_TSDR_MU_Pos              (8U)
#define RTC_TSDR_MU_Msk              (0xFUL << RTC_TSDR_MU_Pos)                 /*!< 0x00000F00 */
#define RTC_TSDR_MU                  RTC_TSDR_MU_Msk
#define RTC_TSDR_MU_0                (0x1UL << RTC_TSDR_MU_Pos)                 /*!< 0x00000100 */
#define RTC_TSDR_MU_1                (0x2UL << RTC_TSDR_MU_Pos)                 /*!< 0x00000200 */
#define RTC_TSDR_MU_2                (0x4UL << RTC_TSDR_MU_Pos)                 /*!< 0x00000400 */
#define RTC_TSDR_MU_3                (0x8UL << RTC_TSDR_MU_Pos)                 /*!< 0x00000800 */
#define RTC_TSDR_DT_Pos              (4U)
#define RTC_TSDR_DT_Msk              (0x3UL << RTC_TSDR_DT_Pos)                 /*!< 0x00000030 */
#define RTC_TSDR_DT                  RTC_TSDR_DT_Msk
#define RTC_TSDR_DT_0                (0x1UL << RTC_TSDR_DT_Pos)                 /*!< 0x00000010 */
#define RTC_TSDR_DT_1                (0x2UL << RTC_TSDR_DT_Pos)                 /*!< 0x00000020 */
#define RTC_TSDR_DU_Pos              (0U)
#define RTC_TSDR_DU_Msk              (0xFUL << RTC_TSDR_DU_Pos)                 /*!< 0x0000000F */
#define RTC_TSDR_DU                  RTC_TSDR_DU_Msk
#define RTC_TSDR_DU_0                (0x1UL << RTC_TSDR_DU_Pos)                 /*!< 0x00000001 */
#define RTC_TSDR_DU_1                (0x2UL << RTC_TSDR_DU_Pos)                 /*!< 0x00000002 */
#define RTC_TSDR_DU_2                (0x4UL << RTC_TSDR_DU_Pos)                 /*!< 0x00000004 */
#define RTC_TSDR_DU_3                (0x8UL << RTC_TSDR_DU_Pos)                 /*!< 0x00000008 */

/********************  Bits definition for RTC_TSSSR register  ***************/
#define RTC_TSSSR_SS_Pos             (0U)
#define RTC_TSSSR_SS_Msk             (0xFFFFUL << RTC_TSSSR_SS_Pos)             /*!< 0x0000FFFF */
#define RTC_TSSSR_SS                 RTC_TSSSR_SS_Msk

/********************  Bits definition for RTC_CALR register  ****************/
#define RTC_CALR_CALP_Pos            (15U)
#define RTC_CALR_CALP_Msk            (0x1UL << RTC_CALR_CALP_Pos)               /*!< 0x00008000 */
#define RTC_CALR_CALP                RTC_CALR_CALP_Msk
#define RTC_CALR_CALW8_Pos           (14U)
#define RTC_CALR_CALW8_Msk           (0x1UL << RTC_CALR_CALW8_Pos)              /*!< 0x00004000 */
#define RTC_CALR_CALW8               RTC_CALR_CALW8_Msk
#define RTC_CALR_CALW16_Pos          (13U)
#define RTC_CALR_CALW16_Msk          (0x1UL << RTC_CALR_CALW16_Pos)             /*!< 0x00002000 */
#define RTC_CALR_CALW16              RTC_CALR_CALW16_Msk
#define RTC_CALR_CALM_Pos            (0U)
#define RTC_CALR_CALM_Msk            (0x1FFUL << RTC_CALR_CALM_Pos)             /*!< 0x000001FF */
#define RTC_CALR_CALM                RTC_CALR_CALM_Msk
#define RTC_CALR_CALM_0              (0x001UL << RTC_CALR_CALM_Pos)             /*!< 0x00000001 */
#define RTC_CALR_CALM_1              (0x002UL << RTC_CALR_CALM_Pos)             /*!< 0x00000002 */
#define RTC_CALR_CALM_2              (0x004UL << RTC_CALR_CALM_Pos)             /*!< 0x00000004 */
#define RTC_CALR_CALM_3              (0x008UL << RTC_CALR_CALM_Pos)             /*!< 0x00000008 */
#define RTC_CALR_CALM_4              (0x010UL << RTC_CALR_CALM_Pos)             /*!< 0x00000010 */
#define RTC_CALR_CALM_5              (0x020UL << RTC_CALR_CALM_Pos)             /*!< 0x00000020 */
#define RTC_CALR_CALM_6              (0x040UL << RTC_CALR_CALM_Pos)             /*!< 0x00000040 */
#define RTC_CALR_CALM_7              (0x080UL << RTC_CALR_CALM_Pos)             /*!< 0x00000080 */
#define RTC_CALR_CALM_8              (0x100UL << RTC_CALR_CALM_Pos)             /*!< 0x00000100 */

/********************  Bits definition for RTC_TAFCR register  ***************/
#define RTC_TAFCR_PC15MODE_Pos       (23U)
#define RTC_TAFCR_PC15MODE_Msk       (0x1UL << RTC_TAFCR_PC15MODE_Pos)          /*!< 0x00800000 */
#define RTC_TAFCR_PC15MODE           RTC_TAFCR_PC15MODE_Msk
#define RTC_TAFCR_PC15VALUE_Pos      (22U)
#define RTC_TAFCR_PC15VALUE_Msk      (0x1UL << RTC_TAFCR_PC15VALUE_Pos)         /*!< 0x00400000 */
#define RTC_TAFCR_PC15VALUE          RTC_TAFCR_PC15VALUE_Msk
#define RTC_TAFCR_PC14MODE_Pos       (21U)
#define RTC_TAFCR_PC14MODE_Msk       (0x1UL << RTC_TAFCR_PC14MODE_Pos)          /*!< 0x00200000 */
#define RTC_TAFCR_PC14MODE           RTC_TAFCR_PC14MODE_Msk
#define RTC_TAFCR_PC14VALUE_Pos      (20U)
#define RTC_TAFCR_PC14VALUE_Msk      (0x1UL << RTC_TAFCR_PC14VALUE_Pos)         /*!< 0x00100000 */
#define RTC_TAFCR_PC14VALUE          RTC_TAFCR_PC14VALUE_Msk
#define RTC_TAFCR_PC13MODE_Pos       (19U)
#define RTC_TAFCR_PC13MODE_Msk       (0x1UL << RTC_TAFCR_PC13MODE_Pos)          /*!< 0x00080000 */
#define RTC_TAFCR_PC13MODE           RTC_TAFCR_PC13MODE_Msk
#define RTC_TAFCR_PC13VALUE_Pos      (18U)
#define RTC_TAFCR_PC13VALUE_Msk      (0x1UL << RTC_TAFCR_PC13VALUE_Pos)         /*!< 0x00040000 */
#define RTC_TAFCR_PC13VALUE          RTC_TAFCR_PC13VALUE_Msk
#define RTC_TAFCR_TAMPPUDIS_Pos      (15U)
#define RTC_TAFCR_TAMPPUDIS_Msk      (0x1UL << RTC_TAFCR_TAMPPUDIS_Pos)         /*!< 0x00008000 */
#define RTC_TAFCR_TAMPPUDIS          RTC_TAFCR_TAMPPUDIS_Msk
#define RTC_TAFCR_TAMPPRCH_Pos       (13U)
#define RTC_TAFCR_TAMPPRCH_Msk       (0x3UL << RTC_TAFCR_TAMPPRCH_Pos)          /*!< 0x00006000 */
#define RTC_TAFCR_TAMPPRCH           RTC_TAFCR_TAMPPRCH_Msk
#define RTC_TAFCR_TAMPPRCH_0         (0x1UL << RTC_TAFCR_TAMPPRCH_Pos)          /*!< 0x00002000 */
#define RTC_TAFCR_TAMPPRCH_1         (0x2UL << RTC_TAFCR_TAMPPRCH_Pos)          /*!< 0x00004000 */
#define RTC_TAFCR_TAMPFLT_Pos        (11U)
#define RTC_TAFCR_TAMPFLT_Msk        (0x3UL << RTC_TAFCR_TAMPFLT_Pos)           /*!< 0x00001800 */
#define RTC_TAFCR_TAMPFLT            RTC_TAFCR_TAMPFLT_Msk
#define RTC_TAFCR_TAMPFLT_0          (0x1UL << RTC_TAFCR_TAMPFLT_Pos)           /*!< 0x00000800 */
#define RTC_TAFCR_TAMPFLT_1          (0x2UL << RTC_TAFCR_TAMPFLT_Pos)           /*!< 0x00001000 */
#define RTC_TAFCR_TAMPFREQ_Pos       (8U)
#define RTC_TAFCR_TAMPFREQ_Msk       (0x7UL << RTC_TAFCR_TAMPFREQ_Pos)          /*!< 0x00000700 */
#define RTC_TAFCR_TAMPFREQ           RTC_TAFCR_TAMPFREQ_Msk
#define RTC_TAFCR_TAMPFREQ_0         (0x1UL << RTC_TAFCR_TAMPFREQ_Pos)          /*!< 0x00000100 */
#define RTC_TAFCR_TAMPFREQ_1         (0x2UL << RTC_TAFCR_TAMPFREQ_Pos)          /*!< 0x00000200 */
#define RTC_TAFCR_TAMPFREQ_2         (0x4UL << RTC_TAFCR_TAMPFREQ_Pos)          /*!< 0x00000400 */
#define RTC_TAFCR_TAMPTS_Pos         (7U)
#define RTC_TAFCR_TAMPTS_Msk         (0x1UL << RTC_TAFCR_TAMPTS_Pos)            /*!< 0x00000080 */
#define RTC_TAFCR_TAMPTS             RTC_TAFCR_TAMPTS_Msk
#define RTC_TAFCR_TAMP2TRG_Pos       (4U)
#define RTC_TAFCR_TAMP2TRG_Msk       (0x1UL << RTC_TAFCR_TAMP2TRG_Pos)          /*!< 0x00000010 */
#define RTC_TAFCR_TAMP2TRG           RTC_TAFCR_TAMP2TRG_Msk
#define RTC_TAFCR_TAMP2E_Pos         (3U)
#define RTC_TAFCR_TAMP2E_Msk         (0x1UL << RTC_TAFCR_TAMP2E_Pos)            /*!< 0x00000008 */
#define RTC_TAFCR_TAMP2E             RTC_TAFCR_TAMP2E_Msk
#define RTC_TAFCR_TAMPIE_Pos         (2U)
#define RTC_TAFCR_TAMPIE_Msk         (0x1UL << RTC_TAFCR_TAMPIE_Pos)            /*!< 0x00000004 */
#define RTC_TAFCR_TAMPIE             RTC_TAFCR_TAMPIE_Msk
#define RTC_TAFCR_TAMP1TRG_Pos       (1U)
#define RTC_TAFCR_TAMP1TRG_Msk       (0x1UL << RTC_TAFCR_TAMP1TRG_Pos)          /*!< 0x00000002 */
#define RTC_TAFCR_TAMP1TRG           RTC_TAFCR_TAMP1TRG_Msk
#define RTC_TAFCR_TAMP1E_Pos         (0U)
#define RTC_TAFCR_TAMP1E_Msk         (0x1UL << RTC_TAFCR_TAMP1E_Pos)            /*!< 0x00000001 */
#define RTC_TAFCR_TAMP1E             RTC_TAFCR_TAMP1E_Msk

/* Reference defines */
#define RTC_TAFCR_ALARMOUTTYPE               RTC_TAFCR_PC13VALUE

/********************  Bits definition for RTC_ALRMASSR register  ************/
#define RTC_ALRMASSR_MASKSS_Pos      (24U)
#define RTC_ALRMASSR_MASKSS_Msk      (0xFUL << RTC_ALRMASSR_MASKSS_Pos)         /*!< 0x0F000000 */
#define RTC_ALRMASSR_MASKSS          RTC_ALRMASSR_MASKSS_Msk
#define RTC_ALRMASSR_MASKSS_0        (0x1UL << RTC_ALRMASSR_MASKSS_Pos)         /*!< 0x01000000 */
#define RTC_ALRMASSR_MASKSS_1        (0x2UL << RTC_ALRMASSR_MASKSS_Pos)         /*!< 0x02000000 */
#define RTC_ALRMASSR_MASKSS_2        (0x4UL << RTC_ALRMASSR_MASKSS_Pos)         /*!< 0x04000000 */
#define RTC_ALRMASSR_MASKSS_3        (0x8UL << RTC_ALRMASSR_MASKSS_Pos)         /*!< 0x08000000 */
#define RTC_ALRMASSR_SS_Pos          (0U)
#define RTC_ALRMASSR_SS_Msk          (0x7FFFUL << RTC_ALRMASSR_SS_Pos)          /*!< 0x00007FFF */
#define RTC_ALRMASSR_SS              RTC_ALRMASSR_SS_Msk



#include "stm32f030xx_gpio_driver.h"
#include "stm32f030xx_spi_driver.h"
#include "stm32f030xx_i2c_driver.h"
#include "stm32f030xx_usart_driver.h"
#include "stm32f030xx_rcc_driver.h"
#include "stm32f030xx_rtc_driver.h"


#endif /* INC_STM32F030XX_H_ */
