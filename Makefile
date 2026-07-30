PROJECT := twitch_gimbal
BUILD_PRESET ?= Debug
BUILD_DIR := build/$(BUILD_PRESET)

ELF := $(BUILD_DIR)/$(PROJECT).elf
HEX := $(BUILD_DIR)/$(PROJECT).hex
BIN := $(BUILD_DIR)/$(PROJECT).bin
ASM := $(BUILD_DIR)/$(PROJECT).asm

OPENOCD_CFG ?= openocd.cfg

.PHONY: all configure build clean size hex bin asm artifacts flash erase reset release flash-release

all: artifacts

configure:
	cmake --preset $(BUILD_PRESET)

build: configure
	cmake --build --preset $(BUILD_PRESET)

size: build
	arm-none-eabi-size $(ELF)

hex: build
	arm-none-eabi-objcopy -O ihex $(ELF) $(HEX)

bin: build
	arm-none-eabi-objcopy -O binary -S $(ELF) $(BIN)

asm: build
	arm-none-eabi-objdump $(ELF) -dSC > $(ASM)

artifacts: build hex bin size

flash: artifacts
	openocd -f $(OPENOCD_CFG) \
		-c "init; reset halt; flash write_image erase $(ELF); verify_image $(ELF); reset run; exit"

erase:
	openocd -f $(OPENOCD_CFG) \
		-c "init; reset halt; stm32f4x mass_erase 0; reset run; exit"

reset:
	openocd -f $(OPENOCD_CFG) \
		-c "init; reset run; exit"

clean:
	rm -rf build

release:
	$(MAKE) artifacts BUILD_PRESET=Release

flash-release:
	$(MAKE) flash BUILD_PRESET=Release
