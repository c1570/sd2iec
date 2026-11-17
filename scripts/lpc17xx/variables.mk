# architecture-dependent variables

#---------------- Source code ----------------
ASMSRC = lpc17xx/pseudoboot.S lpc17xx/startup.S lpc17xx/crc.S

SRC += lpc17xx/iec-bus.c
SRC += lpc17xx/llfl-common.c
SRC += llfl/llfl-jiffydos.c
SRC += llfl/llfl-turbodisk.c
SRC += llfl/llfl-fc3exos.c
SRC += llfl/llfl-ar6.c
SRC += llfl/llfl-dreamload.c
SRC += llfl/llfl-ulm3.c
SRC += llfl/llfl-epyxcart.c
SRC += llfl/llfl-geos.c
SRC += lpc17xx/llfl-parallel.c
SRC += llfl/llfl-n0sdos.c

ifeq ($(CONFIG_UART_DEBUG),y)
  SRC += lpc17xx/printf.c
endif

# Various RTC implementations
ifeq ($(CONFIG_RTC_LPC17XX),y)
  SRC += rtc.c lpc17xx/rtc_lpc17xx.c
endif

# I2C is always needed for the config EEPROM
NEED_I2C := y
SRC += lpc17xx/i2c_lpc17xx.c lpc17xx/arch-eeprom.c

#---------------- Toolchain ----------------
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size
NM = arm-none-eabi-nm


#---------------- Bootloader ----------------
BINARY_LENGTH = 0x80000
CRCGEN        = scripts/lpc17xx/crcgen-lpc.pl


#---------------- Architecture variables ----------------
ARCH_CFLAGS  = -mthumb -mcpu=cortex-m3 -nostartfiles -Isrc/llfl
ARCH_ASFLAGS = -mthumb -mcpu=cortex-m3
ARCH_LDFLAGS = -Tscripts/lpc17xx/$(CONFIG_MCU).ld

#---------------- Config ----------------
# currently no stack tracking supported
