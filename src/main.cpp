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
    int32_t x, y;
    for(y = area->y1; y <= area->y2; y++) {
        for(x = area->x1; x <= area->x2; x++) {
            gfx->drawPixel(x, y, *(uint16_t *) color_p);
            color_p++;
        }
    }

    lv_disp_flush_ready(disp);
}

void gui(void *parameter) {
    int counter = 0;
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

void setup_gui() {
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
    screenMain = lv_obj_create(NULL, NULL);

    lv_obj_t  *calendar = lv_calendar_create(screenMain, NULL);
    lv_obj_set_size(calendar, 210, 210);
    lv_obj_align(calendar, NULL, LV_ALIGN_CENTER, 0, 0);

    /*Make the date number smaller to be sure they fit into their area*/
    lv_obj_set_style_local_text_font(calendar, LV_CALENDAR_PART_DATE, LV_STATE_DEFAULT, lv_theme_get_font_small());

    /*Set today's date*/
    lv_calendar_date_t today;
    today.year = 2018;
    today.month = 10;
    today.day = 23;

    lv_calendar_set_today_date(calendar, &today);
    lv_calendar_set_showed_date(calendar, &today);

    /*Highlight a few days*/
    static lv_calendar_date_t highlighted_days[3];       /*Only its pointer will be saved so should be static*/
    highlighted_days[0].year = 2018;
    highlighted_days[0].month = 10;
    highlighted_days[0].day = 6;

    highlighted_days[1].year = 2018;
    highlighted_days[1].month = 10;
    highlighted_days[1].day = 11;

    highlighted_days[2].year = 2018;
    highlighted_days[2].month = 11;
    highlighted_days[2].day = 22;

    lv_calendar_set_highlighted_dates(calendar, highlighted_days, 3);

    lv_scr_load(screenMain);
    lv_task_handler();
}

void setup() {
    Serial.begin(115200);

    setup_gui();

    xTaskCreatePinnedToCore(gui, "gui", 10000, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(handle_input, "input", 10000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(read_sensors, "sensors", 10000, NULL, 1, NULL, 1);
}

void loop() {
  delay(1000/30);
}
