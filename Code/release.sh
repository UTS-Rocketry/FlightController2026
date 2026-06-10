#!/bin/bash
set -e

echo "=== FLIGHT (RELEASE) BUILD ==="
echo "Reconfiguring as Release (DEBUG off)..."
rm -rf build
cmake -B build \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Release

echo "Building..."
cmake --build build

# Safety check: confirm DEBUG is NOT in the compile flags
if grep -q "DDEBUG" build/compile_commands.json; then
  echo ""
  echo "!!! WARNING: -DDEBUG found in compile flags — this is NOT a clean release build !!!"
  echo "!!! Check CMakeLists.txt for an unconditional DEBUG definition. Aborting flash. !!!"
  exit 1
fi
echo "Confirmed: DEBUG is off."

echo "Converting and flashing..."
arm-none-eabi-objcopy -O binary build/Code.elf build/Code.bin
st-flash write build/Code.bin 0x08000000

echo "=== RELEASE FLASHED ==="