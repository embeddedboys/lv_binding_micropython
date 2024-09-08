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

#define pr_fmt(fmt) "st7796u: " fmt

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>

#include "pico/time.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/platform.h"

#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"

#include "../../lib/lv_bindings/lvgl/lvgl.h"
#include "../../lib/lv_bindings/driver/include/common.h"

#define DRV_NAME "st7796u"
#define DEBUG 0
#if DEBUG
    #define pr_debug(...) mp_printf(&mp_plat_print, __VA_ARGS__)
#else
    #define pr_debug(...)
#endif

#define st7796u_X_RES LCD_X_RES
#define st7796u_Y_RES LCD_Y_RES

#define ARRAY_SIZE(arr) (sizeof(arr)/sizeof(arr[0]))
#define dm_gpio_set_value(p,v) gpio_put(p, v)
#define mdelay(v) sleep_ms(v)

struct st7796u_priv;

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

struct st7796u_operations {
    void (*write_reg)(struct st7796u_priv *priv, int len, ...);
    void (*write_vmem)(struct st7796u_priv *priv, void *vmem, size_t len);

    int (*init_display)(struct st7796u_priv *priv);
    int (*reset)(struct st7796u_priv *priv);
    int (*clear)(struct st7796u_priv *priv, u16 clear);
    int (*sleep)(struct st7796u_priv *priv, bool on);
    int (*set_backlight)(struct st7796u_priv *priv, uint level);
    void (*set_addr_win)(struct st7796u_priv *priv, int xs, int ys, int xe, int ye);
    void (*video_sync)(struct st7796u_priv *priv, int xs, int ys, int xe, int ye, void *vmem, size_t len);
};

struct st7796u_display {
    u32                     xres;
    u32                     yres;
    u32                     bpp;
    u32                     rotate;
    u32                     backlight;

    u32                     xoffs;
    u32                     yoffs;

    bool                    need_tx_buf;

    struct st7796u_operations          tftops;
};

struct st7796u_priv {
    u8                      *buf;

    struct {
        void                *buf;
        size_t              len;
    } txbuf;

    struct {
        int scl;
        int sda;
        int reset;
        int dc;
        int cs;
        int blk;
    } gpio;

    /* device specific */
    struct st7796u_operations  *tftops;
    struct st7796u_display           *display;
} g_priv;

#if TFT_SPIX == 0
    #define spi_ifce spi0
#elif TFT_SPIX == 1
    #define spi_ifce spi1
#else
    #define spi_ifce spi_default
#endif

#if DISP_OVER_PIO
extern int pio_spi_tx_init(uint data_pin, uint clk_pin);
extern void pio_spi_tx_write_buf_dc(void *buf, size_t len, bool dc);
#define write_buf_dc(p, b, l, r) pio_spi_tx_write_buf_dc(b, l, r)
#else
#define write_buf_dc(p, b, l, r) tft_spi_write_buf_dc(p, b, l, r)
#endif

void tft_spi_write_buf_dc(struct st7796u_priv *priv, void *buf, size_t len, bool dc)
{
    gpio_put(TFT_DC_PIN, dc);

    dm_gpio_set_value(TFT_CS_PIN, 0);
    spi_write_blocking(spi_ifce, buf, len);
    dm_gpio_set_value(TFT_CS_PIN, 1);
}

#define define_tft_write_reg(func, reg_type) \
void func(struct st7796u_priv *priv, int len, ...)  \
{   \
    reg_type *buf = (reg_type *)priv->buf; \
    va_list args;   \
    int i;  \
    \
    va_start(args, len);    \
    *buf = (reg_type)va_arg(args, unsigned int); \
    write_buf_dc(priv, buf, sizeof(reg_type), 0); \
    len--;  \
    \
    /* if there no privams */  \
    if (len == 0)  \
        goto exit_no_param; \
    \
    for (i = 0; i < len; i++) { \
        *buf++ = (reg_type)va_arg(args, unsigned int); \
    }   \
    \
    len *= sizeof(reg_type);    \
    write_buf_dc(priv, priv->buf, len, 1);  \
exit_no_param:  \
    va_end(args);   \
}

define_tft_write_reg(tft_write_reg8, u8)
define_tft_write_reg(tft_write_reg16, u16)
#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__}) / sizeof(int))
#define write_reg(priv, ...) \
    priv->tftops->write_reg(priv, NUMARGS(__VA_ARGS__), __VA_ARGS__)

static int st7796u_reset(struct st7796u_priv *priv)
{
    dm_gpio_set_value(priv->gpio.reset, 1);
    mdelay(10);
    dm_gpio_set_value(priv->gpio.reset, 0);
    mdelay(10);
    dm_gpio_set_value(priv->gpio.reset, 1);
    mdelay(10);
    return 0;
}

static int st7796u_set_var(struct st7796u_priv *priv)
{
    pr_debug("%s\n", __func__);
    return 0;
}

static int st7796u_init_display(struct st7796u_priv *priv)
{
    pr_debug("%s, writing initial sequence...\n", __func__);
    st7796u_reset(priv);

#if TFT_MODEL_YT350S006
    write_reg(priv, 0x11);
    mdelay(120);

    write_reg(priv, 0xF0, 0xC3);
    write_reg(priv, 0xF0, 0x96);

    write_reg(priv, 0x36, (1 << 5) | (1 << 3));
    write_reg(priv, 0xB4, 0x01);

    write_reg(priv, 0x3A, 0x05);
    write_reg(priv, 0xE8, 0x40, 0x82, 0x07, 0x18, 0x27, 0x0A, 0xB6, 0x33);
    write_reg(priv, 0xC5, 0x23);
    write_reg(priv, 0xC2, 0xA7);
    write_reg(priv, 0xE0, 0xF0, 0x01, 0x06, 0x0F, 0x12, 0x1D, 0x36, 0x54, 0x44, 0x0C, 0x18, 0x16, 0x13, 0x15);
    write_reg(priv, 0xE1, 0xF0, 0x01, 0x05, 0x0A, 0x0B, 0x07, 0x32, 0x44, 0x44, 0x0C, 0x18, 0x17, 0x13, 0x16);

    write_reg(priv, 0xF0, 0x3C);
    write_reg(priv, 0xF0, 0x69);
    mdelay(120);

    write_reg(priv, 0x29);
#endif

#if TFT_MODEL_HP35006_D
    write_reg(priv, 0xF0, 0xC3);
    write_reg(priv, 0xF0, 0x96);

    write_reg(priv, 0x36, (1 << 5) | (1 << 3));
    write_reg(priv, 0x3A, 0x55);
    write_reg(priv, 0xB4, 0x01);
    write_reg(priv, 0xB1, 0x80, 0x01);
    write_reg(priv, 0xB5, 0x1F, 0x50, 0x00, 0x20);
    write_reg(priv, 0xB6, 0x8A, 0x07, 0x3B);

    write_reg(priv, 0xC0, 0x80, 0x64);
    write_reg(priv, 0xC1, 0x13);
    write_reg(priv, 0xC2, 0xA7);
    write_reg(priv, 0xC5, 0x09);

    write_reg(priv, 0xE8, 0x40, 0x8a, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33);
    write_reg(priv, 0xE0, 0xF0, 0x06, 0x0B, 0x07, 0x06, 0x05, 0x2E, 0x33, 0x47, 0x3A, 0x17, 0x16, 0x2E, 0x31);
    write_reg(priv, 0xE1, 0xF0, 0x09, 0x0D, 0x09, 0x08, 0x23, 0x2E, 0x33, 0x46, 0x38, 0x13, 0x13, 0x2C, 0x32);

    write_reg(priv, 0xF0, 0x3C);
    write_reg(priv, 0xF0, 0x69);

    write_reg(priv, 0x35, 0x00);
    write_reg(priv, 0x11);
    mdelay(120);

    write_reg(priv, 0x29);
    mdelay(50);
    write_reg(priv, 0x21);
#endif

    return 0;
}

static void st7796u_set_addr_win(struct st7796u_priv *priv, int xs, int ys, int xe,
                                int ye)
{
    /* set column adddress */
    write_reg(priv, 0x2A, xs >> 8, xs & 0xFF, xe >> 8, xe & 0xFF);

    /* set row address */
    write_reg(priv, 0x2B, ys >> 8, ys & 0xFF, ye >> 8, ye & 0xFF);

    /* write start */
    write_reg(priv, 0x2C);
}

static int st7796u_clear(struct st7796u_priv *priv, u16 clear)
{
    u32 width = priv->display->xres;
    u32 height = priv->display->yres;
    int x, y;

    // pr_debug("clearing screen (%d x %d) with color 0x%x\n", width, height, clear);

    priv->tftops->set_addr_win(priv, 0, 0,
                         priv->display->xres - 1,
                         priv->display->yres - 1);
    
    for (x = 0; x < width; x++) {
        for (y = 0; y < height; y++) {
            write_buf_dc(priv, &clear, sizeof(u16), 1);
        }
    }

    return 0;
}

static int st7796u_blank(struct st7796u_priv *priv, bool on)
{
    pr_debug("%s\n", __func__);
    return 0;
}

static int st7796u_sleep(struct st7796u_priv *priv, bool on)
{
    pr_debug("%s\n", __func__);
    return 0;
}

static void st7796u_video_sync(struct st7796u_priv *priv, int xs, int ys, int xe, int ye, void *vmem, size_t len)
{
    static u16 *p;

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
    p = (u16 *)vmem;
    for (size_t i = 0; i < len; i++)
        p[i] = (p[i] << 8) | (p[i] >> 8);

    write_buf_dc(priv, vmem, len * 2, 1);
}

static struct st7796u_operations default_st7796u_ops = {
    .init_display    = st7796u_init_display,
    .reset           = st7796u_reset,
    .clear           = st7796u_clear,
    .sleep           = st7796u_sleep,
    .set_addr_win    = st7796u_set_addr_win,
};

static int st7796u_gpio_init(struct st7796u_priv *priv)
{
    pr_debug("initializing gpios...\n");

    gpio_init(priv->gpio.cs);
    gpio_set_dir(priv->gpio.cs, GPIO_OUT);
    gpio_put(priv->gpio.cs, 1);
    // bi_decl(bi_1pin_with_name(priv->gpio.cs, "SPI CS"));

    gpio_init(priv->gpio.reset);
    gpio_set_dir(priv->gpio.reset, GPIO_OUT);
    // bi_decl(bi_1pin_with_name(priv->gpio.reset, "TFT RES"));

    gpio_init(priv->gpio.dc);
    gpio_set_dir(priv->gpio.dc, GPIO_OUT);
    // bi_decl(bi_1pin_with_name(priv->gpio.dc, "TFT DC"));

    gpio_init(priv->gpio.blk);
    gpio_set_dir(priv->gpio.blk, GPIO_OUT);

    return 0;
}

static int st7796u_hw_init(struct st7796u_priv *priv)
{
    pr_debug("%s\n", __func__);

    pr_debug("TFT interface type: %s\n", DISP_OVER_PIO ? "PIO" : "SPI");

#if DISP_OVER_PIO
    pio_spi_tx_init(TFT_SDA_PIN, TFT_SCL_PIN);
    bi_decl(bi_2pins_with_func(TFT_SCL_PIN, TFT_SDA_PIN, GPIO_FUNC_PIO0));
#else
    spi_init(spi_ifce, TFT_BUS_CLK_KHZ * 1000);
    gpio_set_function(TFT_SCL_PIN, GPIO_FUNC_SPI);
    gpio_set_function(TFT_SDA_PIN, GPIO_FUNC_SPI);
    bi_decl(bi_2pins_with_func(TFT_SCL_PIN, TFT_SDA_PIN, GPIO_FUNC_SPI));
    pr_debug("spi%d initialized at %d kHz\n", spi_get_index(spi_ifce), spi_get_baudrate(spi_ifce) / 1000 );
#endif

    st7796u_gpio_init(priv);

    if (!priv->tftops->init_display) {
        // pr_error("init_display must be provided\n");
        return -1;
    }

    pr_debug("initializing display...\n");
    priv->tftops->init_display(priv);

    /* clear screen to black */
    pr_debug("clearing screen...\n");
    priv->tftops->clear(priv, 0x0);

    pr_debug("enbaling backlight...\n");
    dm_gpio_set_value(priv->gpio.blk, 1);

    return 0;
}

static struct st7796u_display default_st7796u_display = {
    .xres   = TFT_HOR_RES,
    .yres   = TFT_VER_RES,
    .bpp    = 16,
    .rotate = 0,
};

// static uint8_t _internal_buf[64];
static int st7796u_probe(struct st7796u_priv *priv)
{
    pr_debug("%s\n", __func__);
    
    priv->buf = (uint8_t *)m_malloc(64);
    
    priv->display = &default_st7796u_display;
    priv->tftops = &default_st7796u_ops;

    pr_debug("%s, set gpios\n", __func__);

    priv->gpio.scl   = TFT_SCL_PIN;
    priv->gpio.sda   = TFT_SDA_PIN;
    priv->gpio.reset = TFT_RES_PIN;
    priv->gpio.dc    = TFT_DC_PIN;
    priv->gpio.cs    = TFT_CS_PIN;
    priv->gpio.blk   = TFT_BLK_PIN;

    priv->tftops->write_reg = tft_write_reg8;

    // if (priv->display->need_tx_buf) {
    //     priv->txbuf.buf = (u8 *)malloc(TFT_TX_BUF_SIZE);
    //     if (!priv->txbuf.buf) {
    //         pr_debug("failed to allocate tx buffer\n");
    //         goto exit_free_tftops;
    //     }
    //     priv->txbuf.len = TFT_TX_BUF_SIZE;
    // }

    st7796u_hw_init(priv);
    
    return 0;
}

int st7796u_driver_init(void)
{
    st7796u_probe(&g_priv);
    return 0;
}

/* ----------------------- Micropython ------------------------------ */

lv_color_t *fb[2] = {NULL, NULL};           // framebuffer pointers

STATIC mp_obj_t st7796u_framebuffer(mp_obj_t n_obj)
{
	int n = mp_obj_get_int(n_obj) -1;

	if (n<0 || n>1){
		return mp_const_none;
	}

	if(fb[n]==NULL){
		fb[n] = m_malloc(sizeof(lv_color_t) * TFT_HOR_RES * 10);
	}
	return mp_obj_new_bytearray_by_ref(sizeof(lv_color_t) * TFT_HOR_RES * 10 , (void *)fb[n]);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(st7796u_framebuffer_obj, st7796u_framebuffer);

STATIC mp_obj_t st7796u_init_func(void)
{
    if (fb[0] == NULL) {
        mp_obj_new_exception_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Failed allocating frame buffer"));
    }

    st7796u_driver_init();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(st7796u_init_obj, st7796u_init_func);

STATIC mp_obj_t st7796u_deinit_func(void)
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
STATIC MP_DEFINE_CONST_FUN_OBJ_0(st7796u_deinit_obj, st7796u_deinit_func);

STATIC void st7796u_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    // pr_debug("%s: xs=%d, ys=%d, xe=%d, ye=%d, len=%d\n", __func__, area->x1, area->y1, area->x2, area->y2, lv_area_get_size(area));
    st7796u_video_sync(&g_priv, area->x1, area->y1, area->x2, area->y2, (void *)color_p, lv_area_get_size(area));
    lv_disp_flush_ready(disp_drv);
}
DEFINE_PTR_OBJ(st7796u_flush);

STATIC const mp_rom_map_elem_t st7796u_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),  MP_OBJ_NEW_QSTR(MP_QSTR_st7796u) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&st7796u_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&st7796u_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_flush), MP_ROM_PTR(&PTR_OBJ(st7796u_flush)) },
    { MP_ROM_QSTR(MP_QSTR_framebuffer), MP_ROM_PTR(&PTR_OBJ(st7796u_framebuffer)) },
};
STATIC MP_DEFINE_CONST_DICT(st7796u_module_globals, st7796u_module_globals_table);

const mp_obj_module_t st7796u_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&st7796u_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_st7796u, st7796u_module);
