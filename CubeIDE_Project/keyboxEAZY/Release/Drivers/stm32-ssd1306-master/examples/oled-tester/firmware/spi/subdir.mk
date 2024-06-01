################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/spi/startup_stm32f411xe.s 

OBJS += \
./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/spi/startup_stm32f411xe.o 

S_DEPS += \
./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/spi/startup_stm32f411xe.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/spi/%.o: ../Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/spi/%.s Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/spi/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -c -I"C:/Users/eneas/STM32CubeIDE/workspace_1.10.1/keyboxEAZY/Drivers/ssd1306" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Drivers-2f-stm32-2d-ssd1306-2d-master-2f-examples-2f-oled-2d-tester-2f-firmware-2f-spi

clean-Drivers-2f-stm32-2d-ssd1306-2d-master-2f-examples-2f-oled-2d-tester-2f-firmware-2f-spi:
	-$(RM) ./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/spi/startup_stm32f411xe.d ./Drivers/stm32-ssd1306-master/examples/oled-tester/firmware/spi/startup_stm32f411xe.o

.PHONY: clean-Drivers-2f-stm32-2d-ssd1306-2d-master-2f-examples-2f-oled-2d-tester-2f-firmware-2f-spi

