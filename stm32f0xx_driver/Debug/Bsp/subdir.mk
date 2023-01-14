################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Bsp/LCD.c 

OBJS += \
./Bsp/LCD.o 

C_DEPS += \
./Bsp/LCD.d 


# Each subdirectory must supply rules for building sources it contributes
Bsp/%.o Bsp/%.su: ../Bsp/%.c Bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32F030R8Tx -DSTM32 -DSTM32F0 -c -I../Inc -I"C:/Users/Abhi/Pictures/Downloads/Embedded C/My_workspace/target/stm32f0xx_driver/Drivers/Inc" -I"C:/Users/Abhi/Pictures/Downloads/Embedded C/My_workspace/target/stm32f0xx_driver/Bsp" -I"C:/Users/Abhi/Pictures/Downloads/Embedded C/My_workspace/target/stm32f0xx_driver/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Bsp

clean-Bsp:
	-$(RM) ./Bsp/LCD.d ./Bsp/LCD.o ./Bsp/LCD.su

.PHONY: clean-Bsp

