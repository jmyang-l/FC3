################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/0/BMI088.c \
../Drivers/0/low_pass_filter.c 

OBJS += \
./Drivers/0/BMI088.o \
./Drivers/0/low_pass_filter.o 

C_DEPS += \
./Drivers/0/BMI088.d \
./Drivers/0/low_pass_filter.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/0/%.o Drivers/0/%.su Drivers/0/%.cyclo: ../Drivers/0/%.c Drivers/0/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BMI088 -I../Drivers/BARO -I../Drivers/FLASH -I../Drivers/DShot300 -I../Drivers/flight_control -I../Drivers/light_flow -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-0

clean-Drivers-2f-0:
	-$(RM) ./Drivers/0/BMI088.cyclo ./Drivers/0/BMI088.d ./Drivers/0/BMI088.o ./Drivers/0/BMI088.su ./Drivers/0/low_pass_filter.cyclo ./Drivers/0/low_pass_filter.d ./Drivers/0/low_pass_filter.o ./Drivers/0/low_pass_filter.su

.PHONY: clean-Drivers-2f-0

