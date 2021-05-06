#include <Arduino.h>

#include "Arduino_Display.h"
#include "Arduino_ESP32SPI.h"
#include "Arduino_GFX.h"

// Include FreeRTOS TaskDelay
#define INCLUDE_vTaskDelay 1
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

#define FRAMERATE 30
#define FRAMETIME 1000/FRAMERATE

Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
Arduino_GC9A01 *gfx = new Arduino_GC9A01(bus, TFT_RESET, 0, true);

void disp_driver_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *) color_p, area->x2 - area->x1 +1, area->y2 - area->y1 +1);
    lv_disp_flush_ready(disp);
}

static void arc_loader(lv_task_t * t) {
    static int16_t a = 270;
    a+=5;

    lv_arc_set_end_angle((lv_obj_t *) t->user_data, a);

    if(a >= 270 + 360) {
        a = 270;
    }
}

void gui(void *parameter) {
    lv_obj_t * arc = lv_arc_create(lv_scr_act(), NULL);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_angles(arc, 270, 270);
    lv_obj_align(arc, NULL, LV_ALIGN_CENTER, 0, 0);

    lv_task_create(arc_loader, 20, LV_TASK_PRIO_LOWEST, arc);

    uint32_t time;
    
    for ( ;; ) {
        time = lv_task_handler();
        vTaskDelay(time/ portTICK_PERIOD_MS);
    }
    
    vTaskDelete(NULL);
}

void handle_input(void *parameter) {
    for ( ;; ) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        Serial.println("Handling input");
    }
    
    vTaskDelete(NULL);
}

void read_sensors(void *parameter) {
    for ( ;; )
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        Serial.println("Handling sensors");
    }
    
    vTaskDelete(NULL);
}

void init_drivers() {
    // Init SPI Graphics Driver
    gfx->begin();
    
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
    disp_drv.flush_cb = disp_driver_flush;
    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    // Turn the display on after init
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
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
