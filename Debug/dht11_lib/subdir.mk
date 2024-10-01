################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../dht11_lib/DHT11.c 

OBJS += \
./dht11_lib/DHT11.o 

C_DEPS += \
./dht11_lib/DHT11.d 


# Each subdirectory must supply rules for building sources it contributes
dht11_lib/%.o dht11_lib/%.su dht11_lib/%.cyclo: ../dht11_lib/%.c dht11_lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F410Rx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/alisi/HAL_LIB_STM32/UART/DHT11_Temp_Hum_sens/dht11_lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-dht11_lib

clean-dht11_lib:
	-$(RM) ./dht11_lib/DHT11.cyclo ./dht11_lib/DHT11.d ./dht11_lib/DHT11.o ./dht11_lib/DHT11.su

.PHONY: clean-dht11_lib

