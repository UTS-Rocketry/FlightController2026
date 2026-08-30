#!/bin/bash
set -euo pipefail

echo "=== ODIN ZEPHYR FLIGHT BUILD ==="
west build -b weact_stm32f405_core . -d build/zephyr --pristine
west flash -d build/zephyr
echo "=== FLIGHT FIRMWARE FLASHED ==="
