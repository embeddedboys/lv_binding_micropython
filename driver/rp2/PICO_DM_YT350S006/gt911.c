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

#define pr_fmt(fmt) "gt911: " fmt

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"

#include "../../lib/lv_bindings/lvgl/lvgl.h"
#include "../../lib/lv_bindings/driver/include/common.h"

#define DRV_NAME "gt911"
#define DEBUG 0
#if DEBUG
    #define pr_debug(...) mp_printf(&mp_plat_print, __VA_ARGS__)
#else
    #define pr_debug(...)
#endif

#define GT911_PIN_SCL   7
#define GT911_PIN_SDA   6
#define GT911_PIN_RST   22
#define GT911_PIN_IRQ   28

#define GT911_CMD_WR    0x28
#define GT911_CMD_RD    0x29

#define GT911_REG_CTRL  0x8040  // GT911 Control Register
#define GT911_REG_CFGS  0x8047  // GT911 Configuration Start Address Regsiter
#define GT911_REG_CHCK  0x80FF  // GT911 Checksum Register
#define GT911_REG_PID   0x8140  // GT911 Product ID Register

#define GT911_REG_GSTID   0x814E  // GT911 Touch state
#define GT911_REG_TP1_X   0x8150  // Touch Point 1 X Data start Address
#define GT911_REG_TP1_Y   0x8152  // Touch Point 1 Y Data start Address
// #define GT911_REG_TP2   0x8158  // Touch Point 2 Data Address
// #define GT911_REG_TP3   0x8160  // Touch Point 3 Data Address
// #define GT911_REG_TP4   0x8168  // Touch Point 4 Data Address
// #define GT911_REG_TP5   0x8170  // Touch Point 5 Data Address

#define GT911_X_RES     TFT_HOR_RES
#define GT911_Y_RES     TFT_VER_RES

#define GT911_DEF_I2C   i2c1
#define GT911_ADDR      0x14
// #define GT911_ADDR      0x5D
#define GT911_DEF_SPEED 400000

#define dm_gpio_set_value(p,v) gpio_put(p, v)
#define mdelay(v) sleep_ms(v)
#define udelay(v) sleep_us(v)

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

typedef enum {
    GT911_DIR_NOP       = 0x00,
    GT911_DIR_REVERT_X  = 0x01,
    GT911_DIR_REVERT_Y  = 0x02,
    GT911_DIR_SWITCH_XY = 0x04,
} gt911_direction_t;

struct gt911_data {
    struct {
        u8 addr;
        i2c_inst_t *master;
        u32 speed;

        u8 pin_scl;
        u8 pin_sda;
    } i2c;

    u8 pin_irq;
    u8 pin_rst;

    u16 x_res;
    u16 y_res;

    gt911_direction_t dir;   /* direction set */
    bool revert_x;
    bool revert_y;
    u16 (*read_x)(struct gt911_data *priv);
    u16 (*read_y)(struct gt911_data *priv);
} g_gt911_data;

static void gt911_write_addr16(struct gt911_data *priv, u16 reg, u8 *txbuf, u8 len)
{
    u8 buf[32] = {0};

    buf[0] = reg >> 8;
    buf[1] = reg & 0xFF;

    memcpy(buf + 2, txbuf, len);
    len+=2;

    // for (int i = 0; i < len; i++)
    //     printf("buf[%d] : 0x%x\n", i, buf[i]);

    i2c_write_blocking(priv->i2c.master, priv->i2c.addr, buf, len, false);
}
#define write_addr16 gt911_write_addr16

static void gt911_read_addr16(struct gt911_data *priv, u16 reg, u8 *rxbuf, u8 len)
{
    u8 buf[2];

    buf[0] = reg >> 8;
    buf[1] = reg & 0xFF;

    i2c_write_blocking(priv->i2c.master, priv->i2c.addr, buf, 2, false);
    i2c_read_blocking(priv->i2c.master, priv->i2c.addr, rxbuf, len, false);
}
#define read_addr16 gt911_read_addr16

static void __gt911_reset(struct gt911_data *priv)
{
    gpio_put(priv->pin_rst, 1);
    sleep_ms(10);
    gpio_put(priv->pin_rst, 0);
    sleep_ms(10);
    gpio_put(priv->pin_rst, 1);
    sleep_ms(10);
}

static u16 __gt911_read_x(struct gt911_data *priv)
{
    u16 this_x = 0;

    read_addr16(priv, GT911_REG_TP1_X, (u8  *)&this_x, sizeof(this_x));
    if (priv->revert_x)
        this_x = priv->x_res - this_x;
    pr_debug("this_x : %d\n", this_x);

    return this_x;
}

u16 gt911_read_x(void)
{
    return g_gt911_data.read_x(&g_gt911_data);
}

static u16 __gt911_read_y(struct gt911_data *priv)
{
    u16 this_y = 0;

    read_addr16(priv, GT911_REG_TP1_Y, (u8  *)&this_y, sizeof(this_y));
    if (priv->revert_y)
        this_y = priv->y_res - this_y;
    pr_debug("this_y : %d\n", this_y);

    return this_y;
}

u16 gt911_read_y(void)
{
    return g_gt911_data.read_y(&g_gt911_data);
}

static bool __gt911_is_pressed(struct gt911_data *priv)
{
    u8 state = 0;
    read_addr16(priv, GT911_REG_GSTID, &state, 1);
    // printf("state : %d\n", state);
    write_addr16(priv, GT911_REG_GSTID, (u8 []){0x00}, 1);
    if (state & 0x80)
        return true;

    return false;
}

bool gt911_is_pressed(void)
{
    return __gt911_is_pressed(&g_gt911_data);
}

static void __gt911_set_dir(struct gt911_data *priv, gt911_direction_t dir)
{
    priv->dir = dir;

    if (dir & GT911_DIR_REVERT_X)
        priv->revert_x = true;
    else
        priv->revert_x = false;

    if (dir & GT911_DIR_REVERT_Y)
        priv->revert_y = true;
    else
        priv->revert_y = false;

    if (dir & GT911_DIR_SWITCH_XY) {
        priv->read_x = __gt911_read_y;
        priv->read_y = __gt911_read_x;

        priv->revert_x = !priv->revert_x;
        priv->revert_y = !priv->revert_y;

        priv->x_res = GT911_Y_RES;
        priv->y_res = GT911_X_RES;
    } else {
        priv->read_x = __gt911_read_x;
        priv->read_y = __gt911_read_y;
    }
}

void gt911_set_dir(gt911_direction_t dir)
{
    __gt911_set_dir(&g_gt911_data, dir);
}

static void gt911_hw_init(struct gt911_data *priv)
{
    pr_debug("%s\n", __func__);

    pr_debug("initialzing i2c controller\n");
    i2c_init(priv->i2c.master, GT911_DEF_SPEED);

    pr_debug("set gpio i2c function\n");
    gpio_init(priv->i2c.pin_scl);
    gpio_init(priv->i2c.pin_sda);
    gpio_set_function(priv->i2c.pin_scl, GPIO_FUNC_I2C);
    gpio_set_function(priv->i2c.pin_sda, GPIO_FUNC_I2C);

    pr_debug("pull up i2c gpio\n");
    gpio_pull_up(priv->i2c.pin_scl);
    gpio_pull_up(priv->i2c.pin_sda);

    pr_debug("initialzing reset pin\n");
    gpio_init(priv->pin_rst);
    gpio_set_dir(priv->pin_rst, GPIO_OUT);
    // gpio_pull_up(priv->spec->pin_rst);
    gpio_put(priv->pin_rst, 0);

    pr_debug("set irq pin as output\n");
    gpio_init(priv->pin_irq);
    gpio_set_dir(priv->pin_irq, GPIO_OUT);
    gpio_pull_up(priv->pin_irq);
    gpio_put(priv->pin_irq, 1);
    udelay(150);

    pr_debug("chip reset\n");
    // priv->ops->reset(priv);
    __gt911_reset(priv);

    pr_debug("set irq pin as input\n");
    gpio_init(priv->pin_irq);
    gpio_set_dir(priv->pin_irq, GPIO_IN);
    gpio_pull_down(priv->pin_irq);

    u8 temp[5];
    read_addr16(priv, GT911_REG_PID, temp, 4);
    pr_debug("Product ID : %s\n", (char *)temp);
    if (0 == strcmp((char *)temp, "911"))  {
        pr_debug("GT911 found\n");
        temp[0] = 0x02;
        write_addr16(priv, GT911_REG_CTRL, temp, 1);
        read_addr16(priv, GT911_REG_CFGS, temp, 1);
        pr_debug("Config reg : 0x%02x\n", temp[0]);
        if (temp[0] < 0x60)
            pr_debug("Default Ver : %d\n", temp[0]);

        mdelay(10);
        temp[0] = 0x00;
        write_addr16(priv, GT911_REG_CTRL, temp, 1);
    }

    __gt911_set_dir(priv, GT911_DIR_SWITCH_XY | GT911_DIR_REVERT_Y);
}

static int gt911_probe(struct gt911_data *priv)
{
    priv->i2c.master  = i2c1;
    priv->i2c.addr    = GT911_ADDR;
    priv->i2c.pin_scl = GT911_PIN_SCL;
    priv->i2c.pin_sda = GT911_PIN_SDA;

    priv->pin_rst     = GT911_PIN_RST;
    priv->pin_irq     = GT911_PIN_IRQ;

    priv->x_res = GT911_X_RES;
    priv->y_res = GT911_Y_RES;

    priv->revert_x = false;
    priv->revert_y = false;

    priv->dir = GT911_DIR_SWITCH_XY | GT911_DIR_REVERT_Y;

    gt911_hw_init(priv);

    return 0;
}

int gt911_driver_init(void)
{
    pr_debug("gt911_driver_init\n");
    gt911_probe(&g_gt911_data);
    return 0;
}

STATIC mp_obj_t gt911_init_func(void)
{
    gt911_driver_init();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(gt911_init_obj, gt911_init_func);

STATIC bool mp_gt911_ts_read(struct _lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    static int32_t last_x = 0;
    static int32_t last_y = 0;

    if (gt911_is_pressed()) {
        data->point.x = last_x = gt911_read_x();
        data->point.y = last_y = gt911_read_y();
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->point.x = last_x;
        data->point.y = last_y;
        data->state = LV_INDEV_STATE_REL;
    }

    return false;
}
DEFINE_PTR_OBJ(mp_gt911_ts_read);

STATIC const mp_rom_map_elem_t gt911_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_gt911) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&gt911_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_ts_read), MP_ROM_PTR(&PTR_OBJ(mp_gt911_ts_read)) },
};
STATIC MP_DEFINE_CONST_DICT(gt911_module_globals, gt911_module_globals_table);

const mp_obj_module_t gt911_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&gt911_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_gt911, gt911_module);
