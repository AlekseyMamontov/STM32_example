# Copyright (c) 2026 Oleksii Mamontov <mamontov1@gmail.com>
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_SERIES_ESP32S3)
  # Same recipe as esp32s3_box3. Flashing and JTAG go through the Type-C port
  # (native USB-Serial/JTAG on IO19/IO20).
  if(NOT "${OPENOCD}" MATCHES "^${ESPRESSIF_TOOLCHAIN_PATH}/.*")
    set(OPENOCD OPENOCD-NOTFOUND)
  endif()
  find_program(OPENOCD openocd PATHS ${ESPRESSIF_TOOLCHAIN_PATH}/openocd-esp32/bin NO_DEFAULT_PATH)

  include(${ZEPHYR_BASE}/boards/common/esp32.board.cmake)
  include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
else()
  # One ST-LINK per MCU: select it with  west flash --dev-id <serial>
  board_runner_args(openocd "--config=${BOARD_DIR}/support/openocd.cfg")
  include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
endif()
