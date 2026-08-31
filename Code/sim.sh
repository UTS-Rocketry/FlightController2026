#!/bin/bash
set -euo pipefail

echo "=== ODIN ZEPHYR HIL SIMULATION BUILD (PYRO LOCKED OUT) ==="
west build -b weact_stm32f405_core . -d build/zephyr-sim --pristine -- \
    -DEXTRA_CONF_FILE=sim.conf

flashed=0
for attempt in {1..30}; do
    echo "ST-Link flash attempt ${attempt}/30"
    if st-flash --hot-plug --reset write \
        build/zephyr-sim/zephyr/zephyr.bin 0x08000000; then
        flashed=1
        break
    fi
done

if [[ ${flashed} -ne 1 ]]; then
    echo "ST-Link could not attach. Connect NRST and retry with --connect-under-reset."
    exit 1
fi

echo "=== 2000 M SIMULATION FLASHED; AUTO-START ENABLED, PYRO LOCKED OUT ==="
