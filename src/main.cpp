#include <Arduino.h>

#include "Arduino_Display.h"
#include "Arduino_ESP32SPI.h"
#include "Arduino_GFX.h"

// Include FreeRTOS TaskDelay
#define INCLUDE_vTaskDelay 1
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

#include "../lvgl/src/lvgl.h"

#include "pins.hpp"
#include "drvs.hpp"

static void arc_loader(lv_task_t *t)
{
    static int16_t a = 270;
    a += 5;

    lv_arc_set_end_angle((lv_obj_t *)t->user_data, a);

    if (a >= 270 + 360)
    {
        a = 270;
    }
}

void gui(void *parameter)
{
    lv_obj_t *arc = lv_arc_create(lv_scr_act(), NULL);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_angles(arc, 270, 270);
    lv_obj_align(arc, NULL, LV_ALIGN_CENTER, 0, 0);

    lv_task_create(arc_loader, 20, LV_TASK_PRIO_LOWEST, arc);

    uint32_t time;

    for (;;)
    {
        time = lv_task_handler();
        vTaskDelay(time / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void handle_input(void *parameter)
{
    button_event_t ev;
    QueueHandle_t button_events = button_init(PIN_BIT(BTN_1) | PIN_BIT(BTN_2) | PIN_BIT(BTN_3));

    for (;;)
    {
        if (xQueueReceive(button_events, &ev, 1000 / portTICK_PERIOD_MS))
        {
            if ((ev.pin == BTN_1) && (ev.event == BUTTON_DOWN))
            {
                Serial.println("Btn1 pressed");
            }
            if ((ev.pin == BTN_2) && (ev.event == BUTTON_DOWN))
            {
                Serial.println("Btn2 pressed");
            }
            if ((ev.pin == BTN_3) && (ev.event == BUTTON_DOWN))
            {
                Serial.println("Btn3 pressed");
            }
        }
    }

    vTaskDelete(NULL);
}

void read_sensors(void *parameter)
{
    for (;;)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void setup()
{
    Serial.begin(115200);

    init_disp_drvs();

    xTaskCreatePinnedToCore(gui, "gui", 10000, NULL, 1, NULL, 0);

    xTaskCreatePinnedToCore(handle_input, "input", 10000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(read_sensors, "sensors", 10000, NULL, 1, NULL, 1);
}

void loop()
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
