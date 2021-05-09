#include <Arduino.h>

#include "Arduino_Display.h"
#include "Arduino_ESP32SPI.h"
#include "Arduino_GFX.h"

#define INCLUDE_vTaskDelay 1
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"

#include "../lvgl/src/lvgl.h"

#include "pins.hpp"
#include "drvs.hpp"
#include "gui.hpp"

static screens_t screens;
static SemaphoreHandle_t mutex_handle;
static StaticSemaphore_t mutex_buffer;

void screen_loader()
{
    // Make sure that the screen is loaded in the first execution
    static uint8_t loaded_screen = -1;

    // Take Mutex to ensure no memory faults
    if (xSemaphoreTake(mutex_handle, 100) == pdTRUE)
    {
        if (screens.screen_id == loaded_screen)
        {
            xSemaphoreGive(mutex_handle);
            return;
        }

        Serial.printf("Switching to screen: %i\n", screens.screen_id);

        lv_obj_t *handle = lv_scr_act();
        lv_scr_load(screens.functions[screens.screen_id]());
        loaded_screen = screens.screen_id;
        lv_obj_del(handle);
        xSemaphoreGive(mutex_handle);
    }
}

void gui(void *parameter)
{
    uint32_t time;

    for (;;)
    {
        screen_loader();
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
            if ((ev.pin == BTN_1) && (ev.event == BUTTON_UP))
            {
                Serial.println("Btn1 pressed");
                if (xSemaphoreTake(mutex_handle, 100) == pdTRUE)
                {
                    if (screens.screen_id - 1 < 0)
                    {
                        screens.screen_id = APP_NUMBER - 1;
                    }
                    else
                    {
                        screens.screen_id--;
                    }

                    xSemaphoreGive(mutex_handle);
                }
            }
            if ((ev.pin == BTN_2) && (ev.event == BUTTON_UP))
            {
                Serial.println("Btn2 pressed");
                if (xSemaphoreTake(mutex_handle, 100) == pdTRUE)
                {
                    screens.screen_id = (screens.screen_id + 1) % APP_NUMBER;
                    xSemaphoreGive(mutex_handle);
                }
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

    mutex_handle = xSemaphoreCreateMutexStatic(&mutex_buffer);
    if (mutex_handle != NULL)
    {
        xTaskCreatePinnedToCore(gui, "gui", 10000, NULL, 1, NULL, 0);

        xTaskCreatePinnedToCore(handle_input, "input", 10000, NULL, 1, NULL, 1);
        xTaskCreatePinnedToCore(read_sensors, "sensors", 10000, NULL, 1, NULL, 1);
    }
    else
    {
        Serial.println("Failed to allocate mutex");
    }
}

void loop()
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
