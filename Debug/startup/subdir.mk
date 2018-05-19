################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32f042.s 

OBJS += \
./startup/startup_stm32f042.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -I"C:/Users/calan/Desktop/st-usart-driver-master/inc" -I"C:/Users/calan/Desktop/st-usart-driver-master/CMSIS/core" -I"C:/Users/calan/Desktop/st-usart-driver-master/CMSIS/device" -I"C:/Users/calan/Desktop/st-usart-driver-master/StdPeriph_Driver/inc" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


