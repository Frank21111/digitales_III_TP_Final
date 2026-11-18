#!/usr/bin/env bash
# Valida el modelo dentro del simulador de Proteus (PROSPICE.EXE), sin la interfaz gráfica.
# Uso (Git Bash en Windows):  ./correr.sh [x86|x64]
# Variables: TOOL = carpeta bin del arm-none-eabi-gcc; PROSPICE = ruta a PROSPICE.EXE.
set -euo pipefail
cd "$(dirname "$0")"
ARQ=${1:-x86}
TOOL=${TOOL:-/c/nxp/MCUXpressoIDE_11.6.0_8187/ide/plugins/com.nxp.mcuxpresso.tools.win32_11.6.0.202112161359/tools/bin}
FW=../firmware
OUT=../../build/proteus
mkdir -p "$OUT"
cp ../../bin/$ARQ/LPC1769.DLL "$OUT/"
cp ../../../Pruebas/*.axf "$OUT/"
for e in 2 3 6 9; do
    "$TOOL/arm-none-eabi-gcc" -mcpu=cortex-m3 -mthumb -std=c99 -O2 -ffunction-sections -fdata-sections \
        -I$FW/cmsis -DESC=$e -nostartfiles -T$FW/lpc1769.ld -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs \
        -o "$OUT/probador$e.axf" probador.c $FW/startup_lpc17xx.c $FW/cmsis/system_LPC17xx.c
done
HERE=$(pwd)
cd "$OUT"
python "$HERE/validar.py" "$HERE" tpi.axf
