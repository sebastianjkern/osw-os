#include "drvs.hpp"

static Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
static Arduino_GC9A01 *gfx = new Arduino_GC9A01(bus, TFT_RESET, 0, true);

void lv_flush_spi(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1);
    lv_disp_flush_ready(disp);
}

void init_disp_drvs()
{
    // Init SPI Graphics Driver
    gfx->begin();

    // Init input device drivers
    lv_init();

    // Init the display buffer for lvgl
    static lv_disp_buf_t disp_buf;
    static lv_color_t buf[WIDTH * 10];
    lv_disp_drv_t disp_drv;

    lv_disp_buf_init(&disp_buf, buf, NULL, WIDTH * 10);

    // Init the LVGL Display driver
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = WIDTH;
    disp_drv.ver_res = HEIGHT;
    disp_drv.flush_cb = lv_flush_spi;
    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    // Turn the display on after init
    gfx->displayOn();
}
