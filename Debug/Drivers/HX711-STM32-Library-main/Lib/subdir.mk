################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/HX711-STM32-Library-main/Lib/HX711.c 

C_DEPS += \
./Drivers/HX711-STM32-Library-main/Lib/HX711.d 

OBJS += \
./Drivers/HX711-STM32-Library-main/Lib/HX711.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/HX711-STM32-Library-main/Lib/%.o Drivers/HX711-STM32-Library-main/Lib/%.su Drivers/HX711-STM32-Library-main/Lib/%.cyclo: ../Drivers/HX711-STM32-Library-main/Lib/%.c Drivers/HX711-STM32-Library-main/Lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G031xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/gagag/STM32CubeIDE/workspace_1.15.1/SemestrinisProjektas/Drivers/NextionLibrary/Inc" -I"C:/Users/gagag/STM32CubeIDE/workspace_1.15.1/SemestrinisProjektas/Drivers/NextionLibrary/Src" -I"C:/Users/gagag/STM32CubeIDE/workspace_1.15.1/SemestrinisProjektas/Drivers/HX711-STM32-Library-main/Lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-HX711-2d-STM32-2d-Library-2d-main-2f-Lib

clean-Drivers-2f-HX711-2d-STM32-2d-Library-2d-main-2f-Lib:
	-$(RM) ./Drivers/HX711-STM32-Library-main/Lib/HX711.cyclo ./Drivers/HX711-STM32-Library-main/Lib/HX711.d ./Drivers/HX711-STM32-Library-main/Lib/HX711.o ./Drivers/HX711-STM32-Library-main/Lib/HX711.su

.PHONY: clean-Drivers-2f-HX711-2d-STM32-2d-Library-2d-main-2f-Lib

