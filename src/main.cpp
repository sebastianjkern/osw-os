#include <Arduino.h>

#include "Arduino_Display.h"
#include "Arduino_ESP32SPI.h"
#include "Arduino_GFX.h"
#include "freertos/FreeRTOS.h"

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
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *) color_p, area->x2 - area->x1 +1, area->y2 - area->y1 +1);
    lv_disp_flush_ready(disp);
}

void setup_gui() {

    // GUI Stuff
    lv_obj_t *screenMain;
    screenMain = lv_obj_create(NULL, NULL);

    /*Create a chart*/
    lv_obj_t * chart;
    chart = lv_chart_create(screenMain, NULL);
    lv_obj_set_size(chart, 200, 150);
    lv_obj_align(chart, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);   /*Show lines and points too*/

    /*Add two data series*/
    lv_chart_series_t *ser1 = lv_chart_add_series(chart, LV_COLOR_RED);
    lv_chart_series_t *ser2 = lv_chart_add_series(chart, LV_COLOR_GREEN);

    /*Set the next points on 'ser1'*/
    lv_chart_set_next(chart, ser1, 10);
    lv_chart_set_next(chart, ser1, 10);
    lv_chart_set_next(chart, ser1, 10);
    lv_chart_set_next(chart, ser1, 10);
    lv_chart_set_next(chart, ser1, 10);
    lv_chart_set_next(chart, ser1, 10);
    lv_chart_set_next(chart, ser1, 10);
    lv_chart_set_next(chart, ser1, 30);
    lv_chart_set_next(chart, ser1, 70);
    lv_chart_set_next(chart, ser1, 90);

    /*Directly set points on 'ser2'*/
    ser2->points[0] = 90;
    ser2->points[1] = 70;
    ser2->points[2] = 65;
    ser2->points[3] = 65;
    ser2->points[4] = 65;
    ser2->points[5] = 65;
    ser2->points[6] = 65;
    ser2->points[7] = 65;
    ser2->points[8] = 65;
    ser2->points[9] = 65;

    lv_chart_refresh(chart); /*Required after direct set*/
    lv_scr_load(screenMain);
    lv_task_handler();
}

void gui(void *parameter) {
    int counter = 0;
    setup_gui();

    while (true)
    {
        lv_task_handler();

        // Limit to 60 fps
        delay(1000/60);
        if (counter > 120) {
            Serial.println("Hello from the GUI Task");
            counter = 0;
        } else {
            counter++;
        }
        
    }
    
    vTaskDelete(NULL);
}

void handle_input(void *parameter) {
    while (true)
    {
        delay(1000);
        Serial.println("Handling input");
    }
    
    vTaskDelete(NULL);
}

void read_sensors(void *parameter) {
    while (true)
    {
        delay(1000);
        Serial.println("Handling sensors");
    }
    
    vTaskDelete(NULL);
}

void init_drivers() {
    // SPI Graphics Driver Initialization
    gfx->begin();
    
    // LVGL Driver Initialization
    lv_init();

    static lv_disp_buf_t disp_buf;
    static lv_color_t buf[WIDTH * 10];
    lv_disp_drv_t disp_drv;

    lv_disp_buf_init(&disp_buf, buf, NULL, WIDTH * 10);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = WIDTH;
    disp_drv.ver_res = HEIGHT;
    disp_drv.flush_cb = disp_driver_flush;
    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    gfx->displayOn();
}


void setup() {
    Serial.begin(115200);

    init_drivers();

    xTaskCreatePinnedToCore(gui, "gui", 10000, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(handle_input, "input", 10000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(read_sensors, "sensors", 10000, NULL, 1, NULL, 1);
}

void loop() {
  delay(1000/30);
}
