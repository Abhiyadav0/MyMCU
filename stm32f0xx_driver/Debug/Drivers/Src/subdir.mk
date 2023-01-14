################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f030xx_USART_driver.c \
../Drivers/Src/stm32f030xx_gpio_driver.c \
../Drivers/Src/stm32f030xx_i2c_driver.c \
../Drivers/Src/stm32f030xx_rcc_driver.c \
../Drivers/Src/stm32f030xx_rtc_driver.c \
../Drivers/Src/stm32f030xx_spi_driver.c 

OBJS += \
./Drivers/Src/stm32f030xx_USART_driver.o \
./Drivers/Src/stm32f030xx_gpio_driver.o \
./Drivers/Src/stm32f030xx_i2c_driver.o \
./Drivers/Src/stm32f030xx_rcc_driver.o \
./Drivers/Src/stm32f030xx_rtc_driver.o \
./Drivers/Src/stm32f030xx_spi_driver.o 

C_DEPS += \
./Drivers/Src/stm32f030xx_USART_driver.d \
./Drivers/Src/stm32f030xx_gpio_driver.d \
./Drivers/Src/stm32f030xx_i2c_driver.d \
./Drivers/Src/stm32f030xx_rcc_driver.d \
./Drivers/Src/stm32f030xx_rtc_driver.d \
./Drivers/Src/stm32f030xx_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32F030R8Tx -DSTM32 -DSTM32F0 -c -I../Inc -I"C:/Users/Abhi/Pictures/Downloads/Embedded C/My_workspace/target/stm32f0xx_driver/Drivers/Inc" -I"C:/Users/Abhi/Pictures/Downloads/Embedded C/My_workspace/target/stm32f0xx_driver/Bsp" -I"C:/Users/Abhi/Pictures/Downloads/Embedded C/My_workspace/target/stm32f0xx_driver/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/stm32f030xx_USART_driver.d ./Drivers/Src/stm32f030xx_USART_driver.o ./Drivers/Src/stm32f030xx_USART_driver.su ./Drivers/Src/stm32f030xx_gpio_driver.d ./Drivers/Src/stm32f030xx_gpio_driver.o ./Drivers/Src/stm32f030xx_gpio_driver.su ./Drivers/Src/stm32f030xx_i2c_driver.d ./Drivers/Src/stm32f030xx_i2c_driver.o ./Drivers/Src/stm32f030xx_i2c_driver.su ./Drivers/Src/stm32f030xx_rcc_driver.d ./Drivers/Src/stm32f030xx_rcc_driver.o ./Drivers/Src/stm32f030xx_rcc_driver.su ./Drivers/Src/stm32f030xx_rtc_driver.d ./Drivers/Src/stm32f030xx_rtc_driver.o ./Drivers/Src/stm32f030xx_rtc_driver.su ./Drivers/Src/stm32f030xx_spi_driver.d ./Drivers/Src/stm32f030xx_spi_driver.o ./Drivers/Src/stm32f030xx_spi_driver.su

.PHONY: clean-Drivers-2f-Src

