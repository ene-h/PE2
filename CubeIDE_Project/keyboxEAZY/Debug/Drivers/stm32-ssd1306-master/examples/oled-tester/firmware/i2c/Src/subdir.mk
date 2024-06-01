################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/main.c \
../Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/stm32f4xx_hal_msp.c \
../Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/stm32f4xx_it.c \
../Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/system_stm32f4xx.c 

OBJS += \
./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/main.o \
./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/stm32f4xx_hal_msp.o \
./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/stm32f4xx_it.o \
./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/system_stm32f4xx.o 

C_DEPS += \
./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/main.d \
./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/stm32f4xx_hal_msp.d \
./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/stm32f4xx_it.d \
./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/%.o Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/%.su: ../Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/%.c Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/HID/Inc -I"C:/Users/eneas/STM32CubeIDE/workspace_1.10.1/keyboxEAZY/Drivers/ssd1306" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-stm32-2d-ssd1306-2d-master-2f-examples-2f-oled-2d-tester-2f-firmware-2f-i2c-2f-Src

clean-Drivers-2f-stm32-2d-ssd1306-2d-master-2f-examples-2f-oled-2d-tester-2f-firmware-2f-i2c-2f-Src:
	-$(RM) ./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/main.d ./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/main.o ./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/main.su ./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/stm32f4xx_hal_msp.d ./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/stm32f4xx_hal_msp.o ./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/stm32f4xx_hal_msp.su ./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/stm32f4xx_it.d ./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/stm32f4xx_it.o ./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/stm32f4xx_it.su ./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/system_stm32f4xx.d ./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/system_stm32f4xx.o ./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/i2c/Src/system_stm32f4xx.su

.PHONY: clean-Drivers-2f-stm32-2d-ssd1306-2d-master-2f-examples-2f-oled-2d-tester-2f-firmware-2f-i2c-2f-Src

