# Compila firmwares para probar el modelo: el del TPI (sin modificar) y los de prueba.
# Uso: make TOOL=~/opt/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin
TOOL ?= $(HOME)/opt/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin
CC = $(TOOL)/arm-none-eabi-gcc
OBJCOPY = $(TOOL)/arm-none-eabi-objcopy
TPI = ../../../../Firmware
OUT = build
CFLAGS = -mcpu=cortex-m3 -mthumb -std=c99 -O2 -g -Wall -Wextra -ffunction-sections -fdata-sections -Icmsis -I$(TPI)
LDFLAGS = -mcpu=cortex-m3 -mthumb -nostartfiles -Tlpc1769.ld -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs
COMMON = startup_lpc17xx.c cmsis/system_LPC17xx.c

PRUEBAS = $(basename $(notdir $(wildcard pruebas/*.c)))

all: $(OUT)/tpi.axf $(OUT)/tpi.hex $(addprefix $(OUT)/,$(addsuffix .axf,$(PRUEBAS))) $(addprefix $(OUT)/,$(addsuffix .hex,$(PRUEBAS)))

$(OUT)/tpi.axf: $(wildcard $(TPI)/*.c) $(COMMON) | $(OUT)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(wildcard $(TPI)/*.c) $(COMMON)

$(OUT)/%.axf: pruebas/%.c $(COMMON) | $(OUT)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $< $(COMMON)

$(OUT)/%.hex: $(OUT)/%.axf
	$(OBJCOPY) -O ihex $< $@

$(OUT):
	mkdir -p $(OUT)

clean:
	rm -rf $(OUT)
