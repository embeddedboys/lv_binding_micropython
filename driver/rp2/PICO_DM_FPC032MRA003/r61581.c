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

#define pr_fmt(fmt) "r61581: " fmt

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>

#include "pico/time.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/platform.h"

#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"

#include "../../lib/lv_bindings/lvgl/lvgl.h"
#include "../../lib/lv_bindings/driver/include/common.h"

#define DRV_NAME "r61581"
#define DEBUG 0
#if DEBUG
    #define pr_debug(...) mp_printf(&mp_plat_print, __VA_ARGS__)
#else
    #define pr_debug(...)
#endif

#define R61581_X_RES LCD_X_RES
#define R61581_Y_RES LCD_Y_RES

#define R61581_HSPL         0       /*HSYNC signal polarity*/
#define R61581_HSL          10      /*HSYNC length (Not Implemented)*/
#define R61581_HFP          10      /*Horitontal Front poarch (Not Implemented)*/
#define R61581_HBP          10      /*Horitontal Back poarch (Not Implemented */
#define R61581_VSPL         0       /*VSYNC signal polarity*/
#define R61581_VSL          10      /*VSYNC length (Not Implemented)*/
#define R61581_VFP          8       /*Vertical Front poarch*/
#define R61581_VBP          8       /*Vertical Back poarch */
#define R61581_DPL          0       /*DCLK signal polarity*/
#define R61581_EPL          1       /*ENABLE signal polarity*/
#define R61581_ORI          0       /*0, 180*/
#define R61581_LV_COLOR_DEPTH 16    /*Fix 16 bit*/

#define ARRAY_SIZE(arr) (sizeof(arr)/sizeof(arr[0]))
#define dm_gpio_set_value(p,v) gpio_put(p, v)
#define mdelay(v) sleep_ms(v)

extern int i80_pio_init(uint8_t db_base, uint8_t db_count, uint8_t pin_wr);
extern int i80_write_buf_rs(void *buf, size_t len, bool rs);

struct r61581_priv;

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

struct r61581_operations {
    int (*init_display)(struct r61581_priv *priv);
    int (*reset)(struct r61581_priv *priv);
    int (*clear)(struct r61581_priv *priv, u16 clear);
    int (*blank)(struct r61581_priv *priv, bool on);
    int (*sleep)(struct r61581_priv *priv, bool on);
    int (*set_var)(struct r61581_priv *priv);
    int (*set_addr_win)(struct r61581_priv *priv, int xs, int ys, int xe, int ye);
    int (*set_cursor)(struct r61581_priv *priv, int x, int y);
};

struct r61581_display {
    u32                     xres;
    u32                     yres;
    u32                     bpp;
    u32                     rotate;
};

struct r61581_priv {
    u8                      *buf;

    struct {
        int reset;
        int cs;   /* chip select */
        int rs;   /* register/data select */
        int wr;   /* write signal */
        int rd;   /* read signal */
        int bl;   /* backlight */
        int db[LCD_PIN_DB_COUNT];
    } gpio;
    
    /* device specific */
    const struct r61581_operations  *tftops;
    struct r61581_display           *display;
} g_priv;

static void fbtft_write_gpio8_wr(struct r61581_priv *priv, void *buf, size_t len)
{
    u8 data;
    int i;
#ifndef DO_NOT_OPTIMIZE_FBTFT_WRITE_GPIO
    static u8 prev_data;
#endif

    /* Start writing by pulling down /WR */
    dm_gpio_set_value(priv->gpio.wr, 1);

    while (len) {
        data = *(u8 *)buf;
        
        /* Start writing by pulling down /WR */
        dm_gpio_set_value(priv->gpio.wr, 0);

        // pr_debug("data : 0x%x\n", data);
        
        /* Set data */
#ifndef DO_NOT_OPTIMIZE_FBTFT_WRITE_GPIO
        if (data == prev_data) {
            dm_gpio_set_value(priv->gpio.wr, 1); /* used as delay */
        } else {
            for (i = 0; i < 8; i++) {
                if ((data & 1) != (prev_data & 1))
                    dm_gpio_set_value(priv->gpio.db[i],
                                      data & 1);
                data >>= 1;
                prev_data >>= 1;
            }
        }
#else
        for (i = 0; i < 8; i++) {
            dm_gpio_set_value(&priv->gpio.db[i], data & 1);
            data >>= 1;
        }
#endif
        
        /* Pullup /WR */
        dm_gpio_set_value(priv->gpio.wr, 1);
        
#ifndef DO_NOT_OPTIMIZE_FBTFT_WRITE_GPIO
        prev_data = *(u8 *)buf;
#endif
        buf ++;
        len --;
    }
}

static void fbtft_write_gpio16_wr_rs(struct r61581_priv *priv, void *buf, size_t len, bool rs)
{
    dm_gpio_set_value(priv->gpio.rs, rs);
    fbtft_write_gpio8_wr(priv, buf, len);
}

/* rs=0 means writing register, rs=1 means writing data */
#if DISP_OVER_PIO
    #define write_buf_rs(p, b, l, r) i80_write_buf_rs(b, l, r)
#else
    #define write_buf_rs(p, b, l, r) fbtft_write_gpio8_wr_rs(p, b, l, r)
#endif

static int r61581_write_reg(struct r61581_priv *priv, int len, ...)
{
    u8 *buf = (u8 *)priv->buf;
    va_list args;
    int i;
    
    va_start(args, len);
    *buf = (u8)va_arg(args, unsigned int);
    write_buf_rs(priv, buf, sizeof(u8), 0);
    len--;
    
    /* if there no privams */
    if (len == 0)
        return 0;
    
    for (i = 0; i < len; i++) {
        *buf = (u8)va_arg(args, unsigned int);
        buf++;
    }

    write_buf_rs(priv, priv->buf, len, 1);
    va_end(args);
    
    return 0;
}
#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__}) / sizeof(int))
#define write_reg(priv, ...) \
    r61581_write_reg(priv, NUMARGS(__VA_ARGS__), __VA_ARGS__)

static int r61581_reset(struct r61581_priv *priv)
{
    dm_gpio_set_value(priv->gpio.reset, 1);
    mdelay(10);
    dm_gpio_set_value(priv->gpio.reset, 0);
    mdelay(10);
    dm_gpio_set_value(priv->gpio.reset, 1);
    mdelay(10);
    return 0;
}

static int r61581_set_var(struct r61581_priv *priv)
{
    pr_debug("%s\n", __func__);
    return 0;
}

static int r61581_init_display(struct r61581_priv *priv)
{
    pr_debug("%s, writing initial sequence...\n", __func__);
    priv->tftops->reset(priv);
    // dm_gpio_set_value(priv->gpio.rd, 1);
    // mdelay(150);

    write_reg(priv, 0xB0, 0x00);
    write_reg(priv, 0xB3, 0x02, 0x00, 0x00, 0x00);

    /* Backlight control */

    write_reg(priv, 0xC0, 0x13, 0x3B, 0x00, 0x02, 0x00, 0x01, 0x00, 0x43);
    write_reg(priv, 0xC1, 0x08, 0x16, 0x08, 0x08);
    write_reg(priv, 0xC4, 0x11, 0x07, 0x03, 0x03);
    write_reg(priv, 0xC6, 0x00);
    write_reg(priv, 0xC8, 0x03, 0x03, 0x13, 0x5C, 0x03, 0x07, 0x14, 0x08, 0x00, 0x21, 0x08, 0x14, 0x07, 0x53, 0x0C, 0x13, 0x03, 0x03, 0x21, 0x00);
    write_reg(priv, 0x0C, 0x55);
    write_reg(priv, 0x36, (1 << 6) | (1 << 5));
    write_reg(priv, 0x38);
    write_reg(priv, 0x3A, 0x55);
    write_reg(priv, 0xD0, 0x07, 0x07, 0x1D, 0x03);
    write_reg(priv, 0xD1, 0x03, 0x30, 0x10);
    write_reg(priv, 0xD2, 0x03, 0x14, 0x04);

    write_reg(priv, 0x11);
    mdelay(10);
    write_reg(priv, 0x29);

    return 0;
}

static int r61581_set_addr_win(struct r61581_priv *priv, int xs, int ys, int xe,
                                int ye)
{
    /* set column adddress */
    write_reg(priv, 0x2A, xs >> 8, xs & 0xFF, xe >> 8, xe & 0xFF);
    
    /* set row address */
    write_reg(priv, 0x2B, ys >> 8, ys & 0xFF, ye >> 8, ye & 0xFF);
    
    /* write start */
    write_reg(priv, 0x2C);
    return 0;
}

static int r61581_clear(struct r61581_priv *priv, u16 clear)
{
    u32 width =  priv->display->xres;
    u32 height = priv->display->yres;

    clear = (clear << 8) | (clear >> 8);

    pr_debug("clearing screen (%d x %d) with color 0x%x\n",
        width, height, clear);

    priv->tftops->set_addr_win(priv, 0, 0, width, height);
    
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            write_buf_rs(priv, &clear, sizeof(u16), 1);
        }
    }

    return 0;
}

static int r61581_blank(struct r61581_priv *priv, bool on)
{
    pr_debug("%s\n", __func__);
    return 0;
}

static int r61581_sleep(struct r61581_priv *priv, bool on)
{
    pr_debug("%s\n", __func__);
    return 0;
}

static const struct r61581_operations default_r61581_ops = {
    .init_display    = r61581_init_display,
    .reset           = r61581_reset,
    .clear           = r61581_clear,
    .blank           = r61581_blank,
    .sleep           = r61581_sleep,
    .set_var         = r61581_set_var,
    .set_addr_win    = r61581_set_addr_win,
};

static int r61581_gpio_init(struct r61581_priv *priv)
{
    pr_debug("initializing gpios...\n");

#if DISP_OVER_PIO
    gpio_init(priv->gpio.reset);
    gpio_init(priv->gpio.bl);
    // gpio_init(priv->gpio.cs);
    gpio_init(priv->gpio.rs);
    // gpio_init(priv->gpio.rd);

    gpio_set_dir(priv->gpio.reset, GPIO_OUT);
    gpio_set_dir(priv->gpio.bl, GPIO_OUT);
    // gpio_set_dir(priv->gpio.cs, GPIO_OUT);
    gpio_set_dir(priv->gpio.rs, GPIO_OUT);
    // gpio_set_dir(priv->gpio.rd, GPIO_OUT);
#else
    int *pp = (int *)&priv->gpio;

    int len = sizeof(priv->gpio)/sizeof(priv->gpio.reset);

    while(len--) {
        gpio_init(*pp);
        gpio_set_dir(*pp, GPIO_OUT);
        pp++;
    }
#endif
    return 0;
}

static int r61581_hw_init(struct r61581_priv *priv)
{
    pr_debug("initializing hardware...\n");

#if DISP_OVER_PIO
    i80_pio_init(priv->gpio.db[0], ARRAY_SIZE(priv->gpio.db), priv->gpio.wr);
#endif
    r61581_gpio_init(priv);

    priv->tftops->init_display(priv);

    /* clear screen to black */
    priv->tftops->clear(priv, 0x0);

    pr_debug("enable backlight\n");
    dm_gpio_set_value(priv->gpio.bl, 1);

    return 0;
}

static struct r61581_display default_r61581_display = {
    .xres   = R61581_X_RES,
    .yres   = R61581_Y_RES,
    .bpp    = 16,
    .rotate = 0,
};

static void r61581_video_sync(struct r61581_priv *priv, int xs, int ys, int xe, int ye, void *vmem, size_t len)
{
    // pr_debug("video sync: xs=%d, ys=%d, xe=%d, ye=%d, len=%d\n", xs, ys, xe, ye, len);
    // printf("video sync: xs=%d, ys=%d, xe=%d, ye=%d, len=%d\n", xs, ys, xe, ye, len);
    priv->tftops->set_addr_win(priv, xs, ys, xe, ye);

    /* 
     * 8080 8-bit Data Bus for 16-bit/pixel (RGB 5-6-5 Bits Input)
     *      DB 7     R4  G2
     *      DB 6     R3  G1
     *      DB 5     R2  G0
     *      DB 4     R1  B4
     *      DB 3     R0  B3
     *      DB 2     G5  B2
     *      DB 1     G4  B1
     *      DB 0     G3  B0
     * But a 16-bit Data Bus RGB565 order like this:
     * B0 - B4, G0 - G5, R0 - R4 from DB0 to DB16
     * So simply swap 2 bytes here from pixel buffer.
     */
    u16 *p = (u16 *)vmem;
    for (size_t i = 0; i < len; i++)
        p[i] = (p[i] << 8) | (p[i] >> 8);

    write_buf_rs(priv, vmem, len * 2, 1);
}

// static uint8_t _internal_buf[64];
static int r61581_probe(struct r61581_priv *priv)
{
    pr_debug("%s\n", __func__);
    
    priv->buf = (uint8_t *)m_malloc(64);
    
    priv->display = &default_r61581_display;
    priv->tftops = &default_r61581_ops;

    pr_debug("%s, set gpios\n", __func__);
    priv->gpio.bl    = LCD_PIN_BL;
    priv->gpio.reset = LCD_PIN_RST;
    // priv->gpio.rd    = 21;
    priv->gpio.rs    = LCD_PIN_RS;
    priv->gpio.wr    = LCD_PIN_WR;
    // priv->gpio.cs    = LCD_PIN_CS;

    /* pin0 - pin15 for I8080 data bus */
    for (int i = LCD_PIN_DB_BASE; i < ARRAY_SIZE(priv->gpio.db); i++)
        priv->gpio.db[i] = i;

    r61581_hw_init(priv);
    
    return 0;
}

int r61581_driver_init(void)
{
    r61581_probe(&g_priv);
    return 0;
}

/* ----------------------- Micropython ------------------------------ */

lv_color_t *fb[2] = {NULL, NULL};           // framebuffer pointers

STATIC mp_obj_t r61581_framebuffer(mp_obj_t n_obj)
{
	int n = mp_obj_get_int(n_obj) -1;

	if (n<0 || n>1){
		return mp_const_none;
	}

	if(fb[n]==NULL){
		fb[n] = m_malloc(sizeof(lv_color_t) * R61581_X_RES * 10);
	}
	return mp_obj_new_bytearray_by_ref(sizeof(lv_color_t) * R61581_X_RES * 10 , (void *)fb[n]);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(r61581_framebuffer_obj, r61581_framebuffer);

STATIC mp_obj_t r61581_init_func(void)
{
    if (fb[0] == NULL) {
        mp_obj_new_exception_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Failed allocating frame buffer"));
    }

    r61581_driver_init();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(r61581_init_obj, r61581_init_func);

STATIC mp_obj_t r61581_deinit_func(void)
{
    if (g_priv.buf!=NULL) {
        m_free(g_priv.buf);
        g_priv.buf = NULL;
    }

    if(fb[0]!=NULL){
    	m_free(fb[0]);
    	fb[0]=NULL;
    }

    if(fb[1]!=NULL){
    	m_free(fb[1]);
    	fb[1]=NULL;
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(r61581_deinit_obj, r61581_deinit_func);

STATIC void r61581_flush(lv_disp_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    // pr_debug("%s: xs=%d, ys=%d, xe=%d, ye=%d, len=%d\n", __func__, area->x1, area->y1, area->x2, area->y2, lv_area_get_size(area));
    r61581_video_sync(&g_priv, area->x1, area->y1, area->x2, area->y2, (void *)color_p, lv_area_get_size(area));
    lv_disp_flush_ready(disp_drv);
}
DEFINE_PTR_OBJ(r61581_flush);

STATIC const mp_rom_map_elem_t r61581_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),  MP_OBJ_NEW_QSTR(MP_QSTR_r61581) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&r61581_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&r61581_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_flush), MP_ROM_PTR(&PTR_OBJ(r61581_flush)) },
    { MP_ROM_QSTR(MP_QSTR_framebuffer), MP_ROM_PTR(&PTR_OBJ(r61581_framebuffer)) },
};
STATIC MP_DEFINE_CONST_DICT(r61581_module_globals, r61581_module_globals_table);

const mp_obj_module_t r61581_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&r61581_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_r61581, r61581_module);
