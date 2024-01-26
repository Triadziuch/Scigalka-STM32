################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MEMS/Target/com.c \
../MEMS/Target/demo_serial.c \
../MEMS/Target/iks01a3_mems_control.c \
../MEMS/Target/iks01a3_mems_control_ex.c \
../MEMS/Target/motion_fx_manager.c \
../MEMS/Target/serial_protocol.c 

OBJS += \
./MEMS/Target/com.o \
./MEMS/Target/demo_serial.o \
./MEMS/Target/iks01a3_mems_control.o \
./MEMS/Target/iks01a3_mems_control_ex.o \
./MEMS/Target/motion_fx_manager.o \
./MEMS/Target/serial_protocol.o 

C_DEPS += \
./MEMS/Target/com.d \
./MEMS/Target/demo_serial.d \
./MEMS/Target/iks01a3_mems_control.d \
./MEMS/Target/iks01a3_mems_control_ex.d \
./MEMS/Target/motion_fx_manager.d \
./MEMS/Target/serial_protocol.d 


# Each subdirectory must supply rules for building sources it contributes
MEMS/Target/%.o MEMS/Target/%.su MEMS/Target/%.cyclo: ../MEMS/Target/%.c MEMS/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../MEMS/App -I../MEMS/Target -I../Drivers/BSP/custom -I../Drivers/BSP/Components/lsm6dso -I../Drivers/BSP/Components/lis2dw12 -I../Drivers/BSP/Components/lis2mdl -I../Drivers/BSP/Components/hts221 -I../Drivers/BSP/Components/lps22hh -I../Drivers/BSP/Components/stts751 -I../Drivers/BSP/IKS01A3 -I../Drivers/BSP/Components/Common -I../Middlewares/ST/STM32_MotionFX_Library/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MEMS-2f-Target

clean-MEMS-2f-Target:
	-$(RM) ./MEMS/Target/com.cyclo ./MEMS/Target/com.d ./MEMS/Target/com.o ./MEMS/Target/com.su ./MEMS/Target/demo_serial.cyclo ./MEMS/Target/demo_serial.d ./MEMS/Target/demo_serial.o ./MEMS/Target/demo_serial.su ./MEMS/Target/iks01a3_mems_control.cyclo ./MEMS/Target/iks01a3_mems_control.d ./MEMS/Target/iks01a3_mems_control.o ./MEMS/Target/iks01a3_mems_control.su ./MEMS/Target/iks01a3_mems_control_ex.cyclo ./MEMS/Target/iks01a3_mems_control_ex.d ./MEMS/Target/iks01a3_mems_control_ex.o ./MEMS/Target/iks01a3_mems_control_ex.su ./MEMS/Target/motion_fx_manager.cyclo ./MEMS/Target/motion_fx_manager.d ./MEMS/Target/motion_fx_manager.o ./MEMS/Target/motion_fx_manager.su ./MEMS/Target/serial_protocol.cyclo ./MEMS/Target/serial_protocol.d ./MEMS/Target/serial_protocol.o ./MEMS/Target/serial_protocol.su

.PHONY: clean-MEMS-2f-Target

