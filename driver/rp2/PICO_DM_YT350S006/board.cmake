# Copyright (c) 2024 IotaHydrae(writeforever@foxmail.com)
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

# LCD Pins for SPI and PIO interface
set(TFT_SPIX      0)    # which spi controller will be used.
set(TFT_SCL_PIN   10)
set(TFT_SDA_PIN   11)
set(TFT_RES_PIN   15)
set(TFT_DC_PIN    14)
set(TFT_CS_PIN    9)
set(TFT_BLK_PIN   13)
set(TFT_HOR_RES   480)
set(TFT_VER_RES   320)
set(TFT_BUS_CLK_KHZ 50000)

message(STATUS "TFT_BUS_CLK_KHZ=${TFT_BUS_CLK_KHZ}")
set(DISP_OVER_PIO 1)  # 1: PIO, 0: SPI

set(TFT_MODEL_YT350S006 1)
set(TFT_MODEL_HP35006_D 0)

list(APPEND MICROPY_SOURCE_EXTMOD
    ${CMAKE_CURRENT_LIST_DIR}/../pio/pio_spi_tx.c
)

target_compile_definitions(${MICROPY_TARGET} PRIVATE
# When using MACRAW mode (with lwIP), maximum buffer space must be used for the raw socket
    TFT_SCL_PIN=${TFT_SCL_PIN}
    TFT_SDA_PIN=${TFT_SDA_PIN}
    TFT_RES_PIN=${TFT_RES_PIN}
    TFT_DC_PIN=${TFT_DC_PIN}
    TFT_CS_PIN=${TFT_CS_PIN}
    TFT_BLK_PIN=${TFT_BLK_PIN}
    TFT_HOR_RES=${TFT_HOR_RES}
    TFT_VER_RES=${TFT_VER_RES}
    TFT_BUS_CLK_KHZ=${TFT_BUS_CLK_KHZ}
    DISP_OVER_PIO=${DISP_OVER_PIO}
    TFT_MODEL_YT350S006=${TFT_MODEL_YT350S006}
    TFT_MODEL_HP35006_D=${TFT_MODEL_HP35006_D}
)
