################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/DShot300/DShot.c 

OBJS += \
./Drivers/DShot300/DShot.o 

C_DEPS += \
./Drivers/DShot300/DShot.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/DShot300/%.o Drivers/DShot300/%.su Drivers/DShot300/%.cyclo: ../Drivers/DShot300/%.c Drivers/DShot300/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BMI088 -I../Drivers/BARO -I../Drivers/FLASH -I../Drivers/DShot300 -I../Drivers/flight_control -I../Drivers/light_flow -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-DShot300

clean-Drivers-2f-DShot300:
	-$(RM) ./Drivers/DShot300/DShot.cyclo ./Drivers/DShot300/DShot.d ./Drivers/DShot300/DShot.o ./Drivers/DShot300/DShot.su

.PHONY: clean-Drivers-2f-DShot300

