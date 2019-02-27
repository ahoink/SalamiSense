################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/USBPD_CUST_NVM_API.c \
../Src/USB_PD_core.c \
../Src/bmp3.c \
../Src/lsm303agr_reg.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/USBPD_CUST_NVM_API.o \
./Src/USB_PD_core.o \
./Src/bmp3.o \
./Src/lsm303agr_reg.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/USBPD_CUST_NVM_API.d \
./Src/USB_PD_core.d \
./Src/bmp3.d \
./Src/lsm303agr_reg.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F411xE -I"M:/OneDrive/Documents/Drexel/Senior Design/Rock960Cape/STM32/firmware/Inc" -I"M:/OneDrive/Documents/Drexel/Senior Design/Rock960Cape/STM32/firmware/Drivers/STM32F4xx_HAL_Driver/Inc" -I"M:/OneDrive/Documents/Drexel/Senior Design/Rock960Cape/STM32/firmware/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"M:/OneDrive/Documents/Drexel/Senior Design/Rock960Cape/STM32/firmware/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"M:/OneDrive/Documents/Drexel/Senior Design/Rock960Cape/STM32/firmware/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


