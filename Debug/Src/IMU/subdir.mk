################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/IMU/MPU9250.c 

OBJS += \
./Src/IMU/MPU9250.o 

C_DEPS += \
./Src/IMU/MPU9250.d 


# Each subdirectory must supply rules for building sources it contributes
Src/IMU/MPU9250.o: ../Src/IMU/MPU9250.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Src/IMU/MPU9250.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

