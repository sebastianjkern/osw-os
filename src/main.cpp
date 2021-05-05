#include <Arduino.h>

#include "Arduino_Display.h"
#include "Arduino_ESP32SPI.h"
#include "Arduino_GFX.h"

#include "../lvgl/src/lvgl.h"

#define TFT_CS 5
#define TFT_DC 12
#define TFT_RESET 33
#define TFT_SCK 18
#define TFT_MOSI 23
#define TFT_MISO -1
#define TFT_LED 9

#define WIDTH 240
#define HEIGHT 240

#define CENTER_W 120
#define CENTER_H 120

Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
Arduino_GC9A01 *gfx = new Arduino_GC9A01(bus, TFT_RESET, 0, true);

void disp_driver_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    int32_t x, y;
    for(y = area->y1; y <= area->y2; y++) {
        for(x = area->x1; x <= area->x2; x++) {
            gfx->drawPixel(x, y, *(uint16_t *) color_p);
            color_p++;
        }
    }

    lv_disp_flush_ready(disp);
}

void setup() {
  Serial.begin(115200);

  // SPI Graphics Driver Initialization
  gfx->begin();
  gfx->displayOn();

  // LVGL Driver Initialization
  lv_init();

  static lv_disp_buf_t disp_buf;
  static lv_color_t buf[240 * 10];
  lv_disp_drv_t disp_drv;

  lv_disp_buf_init(&disp_buf, buf, NULL, 240 * 10);

  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 240;
  disp_drv.ver_res = 240;
  disp_drv.flush_cb = disp_driver_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);

  // GUI Stuff
  lv_obj_t *screenMain;
  lv_obj_t *label;

  screenMain = lv_obj_create(NULL, NULL);

  label = lv_label_create(screenMain, NULL);
  lv_label_set_long_mode(label, LV_LABEL_LONG_BREAK);
  lv_label_set_text(label, "Hello, OSW!");

  lv_label_set_align(label, LV_LABEL_ALIGN_CENTER);
  lv_obj_set_size(label, 100, 40);
  lv_obj_set_pos(label, CENTER_W - 50, CENTER_H - 20);

  lv_scr_load(screenMain);
  lv_task_handler();
}

void loop() {
  // lv_task_handler();
  delay(1);
}
