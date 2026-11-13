#!/usr/bin/env bash
# Explicit MC02/H723 image, no mass erase. Run only when deployment is intended.
set -euo pipefail
firmware_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$firmware_dir"
cmake --preset Sentry
cmake --build --preset Sentry
exec openocd -f interface/cmsis-dap.cfg \
  -c 'cmsis_dap_backend usb_bulk; transport select swd' \
  -f target/stm32h7x.cfg \
  -c 'adapter speed 100; stm32h7x.cpu0 configure -event reset-init {adapter speed 100}' \
  -c "program build/SentryH7/sentry_dm_mc02.elf verify reset exit"
