################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/NextionLibrary/Nextion.c 

C_DEPS += \
./Drivers/NextionLibrary/Nextion.d 

OBJS += \
./Drivers/NextionLibrary/Nextion.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/NextionLibrary/%.o Drivers/NextionLibrary/%.su Drivers/NextionLibrary/%.cyclo: ../Drivers/NextionLibrary/%.c Drivers/NextionLibrary/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G031xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-NextionLibrary

clean-Drivers-2f-NextionLibrary:
	-$(RM) ./Drivers/NextionLibrary/Nextion.cyclo ./Drivers/NextionLibrary/Nextion.d ./Drivers/NextionLibrary/Nextion.o ./Drivers/NextionLibrary/Nextion.su

.PHONY: clean-Drivers-2f-NextionLibrary

