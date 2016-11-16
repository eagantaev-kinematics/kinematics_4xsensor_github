################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/workspace/ac6_mars_workspace/l051workspace/fromcube/kinematics_4xsensor_github/l051_kinematics4x_1/Drivers/CMSIS/Device/ST/STM32L0xx/Source/Templates/system_stm32l0xx.c 

OBJS += \
./Drivers/CMSIS/system_stm32l0xx.o 

C_DEPS += \
./Drivers/CMSIS/system_stm32l0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/system_stm32l0xx.o: C:/workspace/ac6_mars_workspace/l051workspace/fromcube/kinematics_4xsensor_github/l051_kinematics4x_1/Drivers/CMSIS/Device/ST/STM32L0xx/Source/Templates/system_stm32l0xx.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32L051xx -I../../Inc -I"C:/workspace/ac6_mars_workspace/l051workspace/fromcube/kinematics_4xsensor_github/l051_kinematics4x_1/Inc" -I"C:/workspace/ac6_mars_workspace/l051workspace/fromcube/kinematics_4xsensor_github/l051_kinematics4x_1/Drivers/STM32L0xx_HAL_Driver/Inc" -I"C:/workspace/ac6_mars_workspace/l051workspace/fromcube/kinematics_4xsensor_github/l051_kinematics4x_1/Drivers/STM32L0xx_HAL_Driver/Inc/Legacy" -I"C:/workspace/ac6_mars_workspace/l051workspace/fromcube/kinematics_4xsensor_github/l051_kinematics4x_1/Drivers/CMSIS/Include" -I"C:/workspace/ac6_mars_workspace/l051workspace/fromcube/kinematics_4xsensor_github/l051_kinematics4x_1/Drivers/CMSIS/Device/ST/STM32L0xx/Include"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


