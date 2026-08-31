#!/bin/bash
set -euo pipefail

echo "=== ODIN ZEPHYR HIL SIMULATION BUILD (PYRO LOCKED OUT) ==="
west build -b weact_stm32f405_core . -d build/zephyr-sim --pristine -- \
    -DEXTRA_CONF_FILE=sim.conf
west flash -d build/zephyr-sim
echo "=== SIMULATION FIRMWARE FLASHED; ARM FROM THE GROUND STATION TO START ==="
