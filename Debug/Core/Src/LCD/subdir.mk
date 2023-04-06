################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/LCD/i2c-lcd.c 

OBJS += \
./Core/Src/LCD/i2c-lcd.o 

C_DEPS += \
./Core/Src/LCD/i2c-lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/LCD/%.o Core/Src/LCD/%.su: ../Core/Src/LCD/%.c Core/Src/LCD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-LCD

clean-Core-2f-Src-2f-LCD:
	-$(RM) ./Core/Src/LCD/i2c-lcd.d ./Core/Src/LCD/i2c-lcd.o ./Core/Src/LCD/i2c-lcd.su

.PHONY: clean-Core-2f-Src-2f-LCD

