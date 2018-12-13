EXECUTABLE=blinku.elf
BIN_IMAGE=blinku.bin

CC=arm-none-eabi-gcc
DD=arm-none-eabi-objdump
OBJCOPY=arm-none-eabi-objcopy

CFLAGS=-g -O2 -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m4	
#CFLAGS+=-ffreestanding -nostdlib

#CFLAGS+=-ffunction-sections -fdata-sections -mno-sched-prolog -fno-hosted
# membuang code yang tidak perlu
#CFLAGS  += -Wl,--gc-sections

#membuang yang tidak perlu
#LDFLAGS  = --gc-sections



# stm32f4_discovery lib
CFLAGS+=-I../../STM32F4xx_StdPeriph_Driver/inc
CFLAGS+=-I../../STM32F4xx_StdPeriph_Driver/inc/device_support
CFLAGS+=-I../../STM32F4xx_StdPeriph_Driver/inc/core_support

#source assembly
ARMS=../../../Libraries/CMSIS/ST/STM32F4xx/Source/Templates/gcc_ride7/


#CFLAGS+=-I../../../Projects/STM32F4-Discovery_FW_V1.1.0/Utilities/STM32F4-Discovery
CFLAGS+=-I../../../Libraries/CMSIS/ST/STM32F4xx/Include
CFLAGS+=-I../../../Libraries/CMSIS/Include
CFLAGS+=-I../../../Utilities/STM32F4-Discovery
CFLAGS+=-I../../../Libraries/STM32F4xx_StdPeriph_Driver/inc

# supaya pakai vfp dan calling yang benar, jika tidak, akan pakai stdlib yang emulasi		
#CFLAGS  += -mfpu=vfp -mfloat-abi=hard
#CFLAGS  += -mfpu=vfp -mfloat-abi=softfp -Os
CFLAGS  += -mfpu=vfp -mfloat-abi=softfp

#ASFLAGS  =	-mfpu=vfp -mfloat-abi=hard
#ASFLAGS  =	-mfpu=vfp -mfloat-abi=softfp

# to run from FLASH
CFLAGS+=-Wl,-T,stm32_flash.ld

all: $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@
	$(DD) -D $^ > disam

$(EXECUTABLE): main.c system_stm32f4xx.c $(ARMS)startup_stm32f4xx.s stm32f4xx_it.c
	$(CC) $(CFLAGS)  $^ -o $@ -L../../../Libraries/STM32F4xx_StdPeriph_Driver/buildku -lSTM32F4xx_StdPeriph_Driver -lm


clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)
	rm ../../../Libraries/STM32F4xx_StdPeriph_Driver/src/*.o

.PHONY: all clean
