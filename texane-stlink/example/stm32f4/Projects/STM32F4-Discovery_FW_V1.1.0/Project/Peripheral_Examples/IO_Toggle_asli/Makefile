EXECUTABLE=blinku.elf
BIN_IMAGE=blinku.bin

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS=-g -O2 -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m4	
CFLAGS+=-ffreestanding -nostdlib

# to run from FLASH
CFLAGS+=-Wl,-T,stm32_flash.ld

# stm32f4_discovery lib
CFLAGS+=-I../../STM32F4xx_StdPeriph_Driver/inc
CFLAGS+=-I../../STM32F4xx_StdPeriph_Driver/inc/device_support
CFLAGS+=-I../../STM32F4xx_StdPeriph_Driver/inc/core_support

#mulai di modif
#ARMS=../../../Libraries/CMSIS/ST/STM32F4xx/Source/Templates/arm/
ARMS=../../../Libraries/CMSIS/ST/STM32F4xx/Source/Templates/gcc_ride7/


#CFLAGS+=-I../../../Projects/STM32F4-Discovery_FW_V1.1.0/Utilities/STM32F4-Discovery
CFLAGS+=-I../../../Libraries/CMSIS/ST/STM32F4xx/Include
CFLAGS+=-I../../../Libraries/CMSIS/Include
CFLAGS+=-I../../../Utilities/STM32F4-Discovery

all: $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

$(EXECUTABLE): main.c system_stm32f4xx.c $(ARMS)startup_stm32f4xx.s stm32f4xx_it.c 
	$(CC) $(CFLAGS) $^ -o $@  -L../../STM32F4xx_StdPeriph_Driver/build -lSTM32F4xx_StdPeriph_Driver

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)

.PHONY: all clean
