################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/spi_arduino.c \
../Src/stm32f401_gpio.c \
../Src/stm32f401_spi.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/spi_arduino.o \
./Src/stm32f401_gpio.o \
./Src/stm32f401_spi.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/spi_arduino.d \
./Src/stm32f401_gpio.d \
./Src/stm32f401_spi.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F401RETx -DSTM32F4 -c -I../Inc -I"Y:/MCU1-COURSE/MCU1/spi_f/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/spi_arduino.cyclo ./Src/spi_arduino.d ./Src/spi_arduino.o ./Src/spi_arduino.su ./Src/stm32f401_gpio.cyclo ./Src/stm32f401_gpio.d ./Src/stm32f401_gpio.o ./Src/stm32f401_gpio.su ./Src/stm32f401_spi.cyclo ./Src/stm32f401_spi.d ./Src/stm32f401_spi.o ./Src/stm32f401_spi.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

