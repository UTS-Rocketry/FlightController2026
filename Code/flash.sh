#!/bin/bash
set -euo pipefail

west build -b weact_stm32f405_core . -d build/zephyr
west flash -d build/zephyr
