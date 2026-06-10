#!/bin/bash
set -e
cmake --build build \
  && arm-none-eabi-objcopy -O binary build/Code.elf build/Code.bin \
  && st-flash write build/Code.bin 0x08000000   