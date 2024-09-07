# Copyright (c) 2024 IotaHydrae(writeforever@foxmail.com)
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

set(LCD_X_RES 480)
set(LCD_Y_RES 320)

set(LCD_PIN_DB_BASE 0)
set(LCD_PIN_DB_COUNT 8)
set(LCD_PIN_WR  19)
set(LCD_PIN_RS  20)
set(LCD_PIN_RST 18)
set(LCD_PIN_BL  28)
set(LCD_PIN_CS  29)
set(PIO_USE_DMA 1)
set(DISP_OVER_PIO 1)
set(I80_BUS_CLK_KHZ 50000)

target_compile_definitions(${MICROPY_TARGET} PRIVATE
# When using MACRAW mode (with lwIP), maximum buffer space must be used for the raw socket
    LCD_X_RES=${LCD_X_RES}
    LCD_Y_RES=${LCD_Y_RES}
    LCD_PIN_DB_BASE=${LCD_PIN_DB_BASE}
    LCD_PIN_DB_COUNT=${LCD_PIN_DB_COUNT}
    LCD_PIN_WR=${LCD_PIN_WR}
    LCD_PIN_RS=${LCD_PIN_RS}
    LCD_PIN_RST=${LCD_PIN_RST}
    LCD_PIN_BL=${LCD_PIN_BL}
    LCD_PIN_CS=${LCD_PIN_CS}
    PIO_USE_DMA=${PIO_USE_DMA}
    DISP_OVER_PIO=${DISP_OVER_PIO}
    I80_BUS_CLK_KHZ=${I80_BUS_CLK_KHZ}
)
