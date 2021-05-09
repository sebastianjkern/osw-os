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

// Display drivers
#define WIDTH 240
#define HEIGHT 240

void init_disp_drvs();

// Input drivers

// Button debouncer
#define PIN_BIT(x) (1ULL << x)

#define BUTTON_DOWN (1)
#define BUTTON_UP (2)
#define BUTTON_HELD (3)

typedef struct
{
    uint8_t pin;
    uint8_t event;
} button_event_t;

QueueHandle_t button_init(unsigned long long pin_select);
QueueHandle_t pulled_button_init(unsigned long long pin_select, gpio_pull_mode_t pull_mode);
