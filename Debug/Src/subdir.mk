################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/009command_spi_it.c \
../Src/main.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/009command_spi_it.o \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/009command_spi_it.d \
./Src/main.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/009command_spi_it.o: ../Src/009command_spi_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DARM_MATH_CM4 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/vladi/Desktop/Microcontrollers/MCU1/master/stm32f4xx_drivers/Device/ST/STM32F4xx/Include" -I"C:/Users/vladi/Desktop/Microcontrollers/MCU1/master/stm32f4xx_drivers/Include" -I"C:/Users/vladi/Desktop/Microcontrollers/MCU1/master/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/009command_spi_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/main.o: ../Src/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DARM_MATH_CM4 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/vladi/Desktop/Microcontrollers/MCU1/master/stm32f4xx_drivers/Device/ST/STM32F4xx/Include" -I"C:/Users/vladi/Desktop/Microcontrollers/MCU1/master/stm32f4xx_drivers/Include" -I"C:/Users/vladi/Desktop/Microcontrollers/MCU1/master/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DARM_MATH_CM4 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/vladi/Desktop/Microcontrollers/MCU1/master/stm32f4xx_drivers/Device/ST/STM32F4xx/Include" -I"C:/Users/vladi/Desktop/Microcontrollers/MCU1/master/stm32f4xx_drivers/Include" -I"C:/Users/vladi/Desktop/Microcontrollers/MCU1/master/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DARM_MATH_CM4 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/vladi/Desktop/Microcontrollers/MCU1/master/stm32f4xx_drivers/Device/ST/STM32F4xx/Include" -I"C:/Users/vladi/Desktop/Microcontrollers/MCU1/master/stm32f4xx_drivers/Include" -I"C:/Users/vladi/Desktop/Microcontrollers/MCU1/master/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

