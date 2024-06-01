################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/stm32-ssd1306-master/ssd1306/ssd1306.c \
../Drivers/stm32-ssd1306-master/ssd1306/ssd1306_fonts.c \
../Drivers/stm32-ssd1306-master/ssd1306/ssd1306_tests.c 

OBJS += \
./Drivers/stm32-ssd1306-master/ssd1306/ssd1306.o \
./Drivers/stm32-ssd1306-master/ssd1306/ssd1306_fonts.o \
./Drivers/stm32-ssd1306-master/ssd1306/ssd1306_tests.o 

C_DEPS += \
./Drivers/stm32-ssd1306-master/ssd1306/ssd1306.d \
./Drivers/stm32-ssd1306-master/ssd1306/ssd1306_fonts.d \
./Drivers/stm32-ssd1306-master/ssd1306/ssd1306_tests.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/stm32-ssd1306-master/ssd1306/%.o Drivers/stm32-ssd1306-master/ssd1306/%.su: ../Drivers/stm32-ssd1306-master/ssd1306/%.c Drivers/stm32-ssd1306-master/ssd1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/HID/Inc -I"C:/Users/eneas/STM32CubeIDE/workspace_1.10.1/keyboxEAZY/Drivers/ssd1306" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-stm32-2d-ssd1306-2d-master-2f-ssd1306

clean-Drivers-2f-stm32-2d-ssd1306-2d-master-2f-ssd1306:
	-$(RM) ./Drivers/stm32-ssd1306-master/ssd1306/ssd1306.d ./Drivers/stm32-ssd1306-master/ssd1306/ssd1306.o ./Drivers/stm32-ssd1306-master/ssd1306/ssd1306.su ./Drivers/stm32-ssd1306-master/ssd1306/ssd1306_fonts.d ./Drivers/stm32-ssd1306-master/ssd1306/ssd1306_fonts.o ./Drivers/stm32-ssd1306-master/ssd1306/ssd1306_fonts.su ./Drivers/stm32-ssd1306-master/ssd1306/ssd1306_tests.d ./Drivers/stm32-ssd1306-master/ssd1306/ssd1306_tests.o ./Drivers/stm32-ssd1306-master/ssd1306/ssd1306_tests.su

.PHONY: clean-Drivers-2f-stm32-2d-ssd1306-2d-master-2f-ssd1306

