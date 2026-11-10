#!/usr/bin/env bash
# Compila la DLL del modelo para Proteus (Windows, 64 y 32 bits) y las pruebas de Linux.
# Requiere llvm-mingw y el xPack de arm-none-eabi-gcc (ver README).
set -euo pipefail
cd "$(dirname "$0")"
MINGW=${MINGW:-$HOME/opt/llvm-mingw-20261104-ucrt-ubuntu-22.04-x86_64/bin}
ARM=${ARM:-$HOME/opt/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin}
CORE="src/core/cpu.cpp src/core/lpc1769.cpp"
LPC="src/vsm/models.cpp src/vsm/exports_lpc.cpp $CORE"
SONAR="src/vsm/models.cpp src/vsm/exports_sonar.cpp $CORE"
FLAGS="-std=c++17 -O3 -Wall -Wextra -shared -static -s"

mkdir -p bin/x64 bin/x86 build
for a in x64:x86_64 x86:i686; do
    d=${a%%:*}; t=${a##*:}
    "$MINGW/$t-w64-mingw32-clang++" $FLAGS -o bin/$d/LPC1769.DLL $LPC
    "$MINGW/$t-w64-mingw32-clang++" $FLAGS -o bin/$d/HCSR04.DLL $SONAR
done

make -s -C tests/firmware TOOL="$ARM"

g++ -std=c++17 -O2 -Wall -Wextra -o build/soc_test  tests/soc_test.cpp $CORE
g++ -std=c++17 -O2 -Wall -Wextra -o build/mock_host tests/mock_host.cpp $LPC
g++ -std=c++17 -O2 -Wall -Wextra -o build/run_axf tests/run_axf.cpp $CORE
g++ -std=c++17 -O2 -Wall -Wextra -o build/cpu_regress tests/cpu_regress.cpp src/core/cpu.cpp
g++ -std=c++17 -O2 -shared -fPIC -o build/libcpu_api.so tests/cpu_api.cpp src/core/cpu.cpp

if [ "${1:-}" = "test" ]; then
    build/cpu_regress
    build/soc_test tests/firmware/build/tpi.axf
    build/mock_host tests/firmware/build/tpi.axf
    build/mock_host tests/firmware/build/tpi.axf digital
    python3 tests/diff_test.py build/libcpu_api.so "$ARM" 3000 1
fi
