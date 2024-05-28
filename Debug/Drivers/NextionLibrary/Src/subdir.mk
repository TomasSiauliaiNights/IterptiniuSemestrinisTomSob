################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/NextionLibrary/Src/Nextion.c 

C_DEPS += \
./Drivers/NextionLibrary/Src/Nextion.d 

OBJS += \
./Drivers/NextionLibrary/Src/Nextion.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/NextionLibrary/Src/%.o Drivers/NextionLibrary/Src/%.su Drivers/NextionLibrary/Src/%.cyclo: ../Drivers/NextionLibrary/Src/%.c Drivers/NextionLibrary/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G031xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/gagag/STM32CubeIDE/workspace_1.15.1/SemestrinisProjektas/Drivers/NextionLibrary/Inc" -I"C:/Users/gagag/STM32CubeIDE/workspace_1.15.1/SemestrinisProjektas/Drivers/NextionLibrary/Src" -I"C:/Users/gagag/STM32CubeIDE/workspace_1.15.1/SemestrinisProjektas/Drivers/HX711-STM32-Library-main/Lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-NextionLibrary-2f-Src

clean-Drivers-2f-NextionLibrary-2f-Src:
	-$(RM) ./Drivers/NextionLibrary/Src/Nextion.cyclo ./Drivers/NextionLibrary/Src/Nextion.d ./Drivers/NextionLibrary/Src/Nextion.o ./Drivers/NextionLibrary/Src/Nextion.su

.PHONY: clean-Drivers-2f-NextionLibrary-2f-Src

