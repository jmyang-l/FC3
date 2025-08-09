################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/flight_control/attitude_control.c \
../Drivers/flight_control/attitude_ekf.c \
../Drivers/flight_control/attitude_estimator.c \
../Drivers/flight_control/mixer_out.c \
../Drivers/flight_control/position_control.c \
../Drivers/flight_control/position_estimator.c \
../Drivers/flight_control/process.c 

OBJS += \
./Drivers/flight_control/attitude_control.o \
./Drivers/flight_control/attitude_ekf.o \
./Drivers/flight_control/attitude_estimator.o \
./Drivers/flight_control/mixer_out.o \
./Drivers/flight_control/position_control.o \
./Drivers/flight_control/position_estimator.o \
./Drivers/flight_control/process.o 

C_DEPS += \
./Drivers/flight_control/attitude_control.d \
./Drivers/flight_control/attitude_ekf.d \
./Drivers/flight_control/attitude_estimator.d \
./Drivers/flight_control/mixer_out.d \
./Drivers/flight_control/position_control.d \
./Drivers/flight_control/position_estimator.d \
./Drivers/flight_control/process.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/flight_control/%.o Drivers/flight_control/%.su Drivers/flight_control/%.cyclo: ../Drivers/flight_control/%.c Drivers/flight_control/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BMI088 -I../Drivers/BARO -I../Drivers/FLASH -I../Drivers/DShot300 -I../Drivers/flight_control -I../Drivers/light_flow -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-flight_control

clean-Drivers-2f-flight_control:
	-$(RM) ./Drivers/flight_control/attitude_control.cyclo ./Drivers/flight_control/attitude_control.d ./Drivers/flight_control/attitude_control.o ./Drivers/flight_control/attitude_control.su ./Drivers/flight_control/attitude_ekf.cyclo ./Drivers/flight_control/attitude_ekf.d ./Drivers/flight_control/attitude_ekf.o ./Drivers/flight_control/attitude_ekf.su ./Drivers/flight_control/attitude_estimator.cyclo ./Drivers/flight_control/attitude_estimator.d ./Drivers/flight_control/attitude_estimator.o ./Drivers/flight_control/attitude_estimator.su ./Drivers/flight_control/mixer_out.cyclo ./Drivers/flight_control/mixer_out.d ./Drivers/flight_control/mixer_out.o ./Drivers/flight_control/mixer_out.su ./Drivers/flight_control/position_control.cyclo ./Drivers/flight_control/position_control.d ./Drivers/flight_control/position_control.o ./Drivers/flight_control/position_control.su ./Drivers/flight_control/position_estimator.cyclo ./Drivers/flight_control/position_estimator.d ./Drivers/flight_control/position_estimator.o ./Drivers/flight_control/position_estimator.su ./Drivers/flight_control/process.cyclo ./Drivers/flight_control/process.d ./Drivers/flight_control/process.o ./Drivers/flight_control/process.su

.PHONY: clean-Drivers-2f-flight_control

