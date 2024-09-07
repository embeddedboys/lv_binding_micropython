// The MIT License (MIT)
//
// Copyright (c) 2024 embeddedboys developers
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#define pr_fmt(fmt) "tsc2007: " fmt

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"

#include "../../lib/lv_bindings/lvgl/lvgl.h"
#include "../../lib/lv_bindings/driver/include/common.h"

#define DRV_NAME "tsc2007"
#define DEBUG 0
#if DEBUG
    #define pr_debug(...) mp_printf(&mp_plat_print, __VA_ARGS__)
#else
    #define pr_debug(...)
#endif

#define TSC2007_X_RES 415
#define TSC2007_Y_RES 285
#define TSC2007_X_OFFS (5)
#define TSC2007_Y_OFFS (-20)

#define TSC2007_ADDR       0x48
#define TSC2007_DEF_SPEED  400000
#define TSC2007_PIN_SCL    27
#define TSC2007_PIN_SDA    26
#define TSC2007_PIN_IRQ    21
#define TSC2007_PIN_RST    29

#define TSC2007_CMD_READ_X 0xC0
#define TSC2007_CMD_READ_Y 0xD0

enum {
    TSC2007_RESOLUTION_8BIT = 8,
    TSC2007_RESOLUTION_12BIT = 12,
};

enum {
    TSC2007_POWER_MODE_NORMAL,
    TSC2007_POWER_MODE_LOW_POWER,
};

typedef enum {
    TSC2007_DIR_NOP       = 0x00,
    TSC2007_DIR_INVERT_X  = 0x01,
    TSC2007_DIR_INVERT_Y  = 0x02,
    TSC2007_DIR_SWITCH_XY = 0x04,
} tsc2007_direction_t;

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

struct tsc2007_data {
    struct {
        uint8_t addr;
        i2c_inst_t *master;
        uint32_t speed;

        uint8_t scl_pin;
        uint8_t sda_pin;
    } i2c;

    uint8_t pin_irq;
    uint8_t pin_rst;

    uint16_t x_res;
    uint16_t y_res;
    u16            tp_x_res;
    u16            tp_y_res;
    int            x_offs;
    int            y_offs;

    /* for res-touch like */
    u8             resolution;
    float               sc_x;     /* scaling constant for x_res */
    float               sc_y;     /* scaling constant for y_res */

    tsc2007_direction_t dir;   /* direction set */
    bool invert_x;
    bool invert_y;
    bool switch_xy;
    uint16_t (*read_x)(struct tsc2007_data *priv);
    uint16_t (*read_y)(struct tsc2007_data *priv);
} g_tsc2007_data;

extern int i2c_bus_scan(i2c_inst_t *i2c);

static int tsc2007_write_reg(struct tsc2007_data *priv, uint8_t reg, uint8_t val)
{
    uint16_t buf = val << 8 | reg;
    i2c_write_blocking(priv->i2c.master, priv->i2c.addr, (uint8_t *)&buf, sizeof(buf), false);
    return 0;
}
#define write_reg tsc2007_write_reg

static uint8_t tsc2007_read_reg(struct tsc2007_data *priv, uint8_t reg)
{
    uint8_t val;
    i2c_write_blocking(priv->i2c.master, priv->i2c.addr, &reg, 1, true);
    i2c_read_blocking(priv->i2c.master, priv->i2c.addr, &val, 1, false);
    return val;
}
#define read_reg tsc2007_read_reg

static void __tsc2007_reset(struct tsc2007_data *priv)
{
    gpio_put(priv->pin_rst, 1);
    sleep_ms(10);
    gpio_put(priv->pin_rst, 0);
    sleep_ms(10);
    gpio_put(priv->pin_rst, 1);
    sleep_ms(10);
}

static uint16_t __tsc2007_read_x(struct tsc2007_data *priv)
{
    uint8_t val = read_reg(priv, TSC2007_CMD_READ_X);
    u16 this_x = 0;

    if (priv->invert_x)
        this_x = (priv->x_res - (val * priv->x_res) / (1 << priv->resolution));
    else
        this_x = (val * priv->x_res) / (1 << priv->resolution);

    pr_debug("x : %d, sc_x : %f\n", this_x, priv->sc_x);
    this_x += priv->x_offs;
    this_x *= priv->sc_x;
    pr_debug("x : %d, sc_x : %f\n", this_x, priv->sc_x);

    return this_x;
}

uint16_t tsc2007_read_x(void)
{
    return g_tsc2007_data.read_x(&g_tsc2007_data);
}

static uint16_t __tsc2007_read_y(struct tsc2007_data *priv)
{
    uint8_t val = read_reg(priv, TSC2007_CMD_READ_Y);
    u16 this_y = 0;

    if (priv->invert_y)
        this_y = (priv->y_res - (val * priv->y_res) / (1 << priv->resolution));
    else
        this_y = (val * priv->y_res) / (1 << priv->resolution);

    pr_debug("y : %d, sc_y : %f\n", this_y, priv->sc_y);
    this_y += priv->y_offs;
    this_y *= priv->sc_y;
    pr_debug("y : %d, sc_y : %f\n", this_y, priv->sc_y);

    return this_y;
}

uint16_t tsc2007_read_y(void)
{
    return g_tsc2007_data.read_y(&g_tsc2007_data);
}

static bool __tsc2007_is_pressed(struct tsc2007_data *priv)
{
    return !gpio_get(priv->pin_irq);
}

bool tsc2007_is_pressed(void)
{
    return __tsc2007_is_pressed(&g_tsc2007_data);
}

static void swap_float(float *a, float *b)
{
    float temp = *a;
    *a = *b;
    *b = temp;
}

static void __tsc2007_set_dir(struct tsc2007_data *priv, tsc2007_direction_t dir)
{
    priv->dir = dir;

    if (dir & TSC2007_DIR_INVERT_X)
        priv->invert_x = true;
    else
        priv->invert_x = false;

    if (dir & TSC2007_DIR_INVERT_Y)
        priv->invert_y = true;
    else
        priv->invert_y = false;

    if (dir & TSC2007_DIR_SWITCH_XY) {
        priv->switch_xy = true;

        priv->read_x = __tsc2007_read_y;
        priv->read_y = __tsc2007_read_x;

        bool invert_tmp = priv->invert_x;
        priv->invert_x = priv->invert_y;
        priv->invert_y = invert_tmp;

        u16 offs_tmp = priv->x_offs;
        priv->x_offs = priv->y_offs;
        priv->y_offs = offs_tmp;

        swap_float(&priv->sc_x, &priv->sc_y);

        u16 res_tmp = priv->x_res;
        priv->x_res = priv->y_res;
        priv->y_res = res_tmp;
    } else {
        priv->switch_xy = false;

        priv->read_x = __tsc2007_read_x;
        priv->read_y = __tsc2007_read_y;
    }
}

void tsc2007_set_dir(tsc2007_direction_t dir)
{
    __tsc2007_set_dir(&g_tsc2007_data, dir);
}

static void tsc2007_hw_init(struct tsc2007_data *priv)
{
    i2c_init(priv->i2c.master, TSC2007_DEF_SPEED);
    gpio_set_function(priv->i2c.scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(priv->i2c.sda_pin, GPIO_FUNC_I2C);

    gpio_pull_up(priv->i2c.scl_pin);
    gpio_pull_up(priv->i2c.sda_pin);

    gpio_init(priv->pin_rst);
    gpio_set_dir(priv->pin_rst, GPIO_OUT);
    gpio_pull_up(priv->pin_rst);

    gpio_init(priv->pin_irq);
    gpio_pull_up(priv->pin_irq);
    gpio_set_dir(priv->pin_irq, GPIO_IN);


    __tsc2007_reset(priv);
    /* initialize touch direction */
    __tsc2007_set_dir(priv, TSC2007_DIR_SWITCH_XY | TSC2007_DIR_INVERT_Y);
}

static int tsc2007_probe(struct tsc2007_data *priv)
{
    priv->i2c.master  = i2c1;
    priv->i2c.addr    = TSC2007_ADDR;
    priv->i2c.scl_pin = TSC2007_PIN_SCL;
    priv->i2c.sda_pin = TSC2007_PIN_SDA;

    priv->pin_rst     = TSC2007_PIN_RST;
    priv->pin_irq     = TSC2007_PIN_IRQ;

    priv->x_res = LCD_X_RES;
    priv->y_res = LCD_Y_RES;
    priv->resolution = TSC2007_RESOLUTION_8BIT;

    float tft_x = LCD_X_RES;
    float tft_y = LCD_Y_RES;

    /* Real Resolution Values of Touch Panel */
    float touch_x = TSC2007_X_RES;
    float touch_y = TSC2007_Y_RES;
    priv->x_offs = TSC2007_X_OFFS;
    priv->y_offs = TSC2007_Y_OFFS;

    /* Calc and save scaling values */
    priv->sc_x = (float)(tft_x / touch_x);
    priv->sc_y = (float)(tft_y / touch_y);

    pr_debug("sc_x : %f, sc_y : %f", priv->sc_x, priv->sc_y);

    tsc2007_hw_init(priv);

    return 0;
}

int tsc2007_driver_init(void)
{
    pr_debug("tsc2007_driver_init\n");
    tsc2007_probe(&g_tsc2007_data);
    return 0;
}

STATIC mp_obj_t tsc2007_init_func(void)
{
    tsc2007_driver_init();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(tsc2007_init_obj, tsc2007_init_func);

STATIC bool mp_tsc2007_ts_read(struct _lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    static int32_t last_x = 0;
    static int32_t last_y = 0;

    if (tsc2007_is_pressed()) {
        data->point.x = last_x = tsc2007_read_x();
        data->point.y = last_y = tsc2007_read_y();
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->point.x = last_x;
        data->point.y = last_y;
        data->state = LV_INDEV_STATE_REL;
    }

    return false;
}
DEFINE_PTR_OBJ(mp_tsc2007_ts_read);

STATIC const mp_rom_map_elem_t tsc2007_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_tsc2007) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&tsc2007_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_ts_read), MP_ROM_PTR(&PTR_OBJ(mp_tsc2007_ts_read)) },
};
STATIC MP_DEFINE_CONST_DICT(tsc2007_module_globals, tsc2007_module_globals_table);

const mp_obj_module_t tsc2007_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&tsc2007_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_tsc2007, tsc2007_module);
