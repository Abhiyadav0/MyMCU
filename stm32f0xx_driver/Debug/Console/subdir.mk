################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Console/console.c 

OBJS += \
./Console/console.o 

C_DEPS += \
./Console/console.d 


# Each subdirectory must supply rules for building sources it contributes
Console/%.o Console/%.su: ../Console/%.c Console/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32F030R8Tx -DSTM32 -DSTM32F0 -c -I../Inc -I"C:/Users/Abhi/Pictures/Downloads/Embedded C/My_workspace/target/stm32f0xx_driver/Drivers/Inc" -I"C:/Users/Abhi/Pictures/Downloads/Embedded C/My_workspace/target/stm32f0xx_driver/Bsp" -I"C:/Users/Abhi/Pictures/Downloads/Embedded C/My_workspace/target/stm32f0xx_driver/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Console

clean-Console:
	-$(RM) ./Console/console.d ./Console/console.o ./Console/console.su

.PHONY: clean-Console

