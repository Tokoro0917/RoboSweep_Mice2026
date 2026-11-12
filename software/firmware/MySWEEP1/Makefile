# STM32CubeIDE がなくてもビルドできるようにするための Makefile
#   make        … build/MySWEEP1.elf と .bin を作る
#   make clean  … build/ を消す
# 必要なもの: arm-none-eabi-gcc（Ubuntu: apt install gcc-arm-none-eabi libnewlib-arm-none-eabi）
# 設定は STM32CubeIDE の Debug 構成（.cproject）に合わせています。

TARGET  = MySWEEP1
BUILD   = build
PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size

MCU     = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS    = -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG
INCS    = -ICore/Inc \
          -IDrivers/STM32F4xx_HAL_Driver/Inc \
          -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
          -IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
          -IDrivers/CMSIS/Include
CFLAGS  = $(MCU) $(DEFS) $(INCS) -std=gnu11 -g3 -O0 -Wall \
          -ffunction-sections -fdata-sections -MMD -MP
LDFLAGS = $(MCU) -TSTM32F446RETX_FLASH.ld --specs=nosys.specs --specs=nano.specs \
          -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map -u _printf_float -lm

CSRC    = $(wildcard Core/Src/*.c) $(wildcard Drivers/STM32F4xx_HAL_Driver/Src/*.c)
ASRC    = Core/Startup/startup_stm32f446retx.s
OBJS    = $(addprefix $(BUILD)/,$(CSRC:.c=.o) $(ASRC:.s=.o))

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(MCU) -x assembler-with-cpp -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(BUILD)

-include $(OBJS:.o=.d)
.PHONY: all clean
