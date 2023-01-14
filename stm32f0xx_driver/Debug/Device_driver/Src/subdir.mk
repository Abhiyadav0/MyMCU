################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Device_driver/Src/LCD.c 

OBJS += \
./Device_driver/Src/LCD.o 

C_DEPS += \
./Device_driver/Src/LCD.d 


# Each subdirectory must supply rules for building sources it contributes
Device_driver/Src/%.o Device_driver/Src/%.su: ../Device_driver/Src/%.c Device_driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32F030R8Tx -DSTM32 -DSTM32F0 -c -I../Inc -I"C:/Users/Abhi/Pictures/Downloads/Embedded C/My_workspace/target/stm32f0xx_driver/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Device_driver-2f-Src

clean-Device_driver-2f-Src:
	-$(RM) ./Device_driver/Src/LCD.d ./Device_driver/Src/LCD.o ./Device_driver/Src/LCD.su

.PHONY: clean-Device_driver-2f-Src

