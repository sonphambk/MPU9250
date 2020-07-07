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
Src/IMU/%.o: ../Src/IMU/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -I"C:/Users/PC/Desktop/MPU9250/Inc" -I"C:/Users/PC/Desktop/MPU9250/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/PC/Desktop/MPU9250/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/PC/Desktop/MPU9250/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/PC/Desktop/MPU9250/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


