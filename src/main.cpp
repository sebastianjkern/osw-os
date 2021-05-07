#include <Arduino.h>

#include "Arduino_Display.h"
#include "Arduino_ESP32SPI.h"
#include "Arduino_GFX.h"

// Include FreeRTOS TaskDelay
#define INCLUDE_vTaskDelay 1
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

#include "../lvgl/src/lvgl.h"

#define TFT_CS 5
#define TFT_DC 12
#define TFT_RESET 33
#define TFT_SCK 18
#define TFT_MOSI 23
#define TFT_MISO -1
#define TFT_LED 9

#define BTN_1 0
#define BTN_2 10
#define BTN_3 13

#define WIDTH 240
#define HEIGHT 240
#define CENTER_W 120
#define CENTER_H 120

static Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
static Arduino_GC9A01 *gfx = new Arduino_GC9A01(bus, TFT_RESET, 0, true);

// DEBOUNCER: https://github.com/craftmetrics/esp32-button
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "BUTTON"
#define PIN_BIT(x) (1ULL<<x)

#define BUTTON_DOWN (1)
#define BUTTON_UP (2)
#define BUTTON_HELD (3)

QueueHandle_t button_init(unsigned long long pin_select);
QueueHandle_t pulled_button_init(unsigned long long pin_select, gpio_pull_mode_t pull_mode);

typedef struct {
  uint8_t pin;
    uint8_t event;
} button_event_t;

typedef struct {
  uint8_t pin;
  bool inverted;
  uint16_t history;
  uint32_t down_time;
  uint32_t next_long_time;
} debounce_t;

int pin_count = -1;
debounce_t * debounce;
QueueHandle_t queue;

static void update_button(debounce_t *d) {
    d->history = (d->history << 1) | gpio_get_level((gpio_num_t) d->pin);
}

#define MASK   0b1111000000111111
static bool button_rose(debounce_t *d) {
    if ((d->history & MASK) == 0b0000000000111111) {
        d->history = 0xffff;
        return 1;
    }
    return 0;
}
static bool button_fell(debounce_t *d) {
    if ((d->history & MASK) == 0b1111000000000000) {
        d->history = 0x0000;
        return 1;
    }
    return 0;
}
static bool button_down(debounce_t *d) {
    if (d->inverted) return button_fell(d);
    return button_rose(d);
}
static bool button_up(debounce_t *d) {
    if (d->inverted) return button_rose(d);
    return button_fell(d);
}

#define LONG_PRESS_DURATION (2000)
#define LONG_PRESS_REPEAT (50)

static void send_event(debounce_t db, int ev) {
    button_event_t event = {
        .pin = db.pin,
        .event = ev,
    };
    xQueueSend(queue, &event, portMAX_DELAY);
}

static void button_task(void *pvParameter)
{
    for (;;) {
        for (int idx=0; idx<pin_count; idx++) {
            update_button(&debounce[idx]);
            if (debounce[idx].down_time && millis() >= debounce[idx].next_long_time) {
                ESP_LOGI(TAG, "%d LONG", debounce[idx].pin);
                debounce[idx].next_long_time = debounce[idx].next_long_time + LONG_PRESS_REPEAT;
                send_event(debounce[idx], BUTTON_HELD);
            } else if (button_down(&debounce[idx]) && debounce[idx].down_time == 0) {
                debounce[idx].down_time = millis();
                ESP_LOGI(TAG, "%d DOWN", debounce[idx].pin);
                debounce[idx].next_long_time = debounce[idx].down_time + LONG_PRESS_DURATION;
                send_event(debounce[idx], BUTTON_DOWN);
            } else if (button_up(&debounce[idx])) {
                debounce[idx].down_time = 0;
                ESP_LOGI(TAG, "%d UP", debounce[idx].pin);
                send_event(debounce[idx], BUTTON_UP);
            }
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

QueueHandle_t button_init(unsigned long long pin_select) {
    return pulled_button_init(pin_select, GPIO_FLOATING);
}


QueueHandle_t pulled_button_init(unsigned long long pin_select, gpio_pull_mode_t pull_mode)
{
    if (pin_count != -1) {
        ESP_LOGI(TAG, "Already initialized");
        return NULL;
    }

    // Configure the pins
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = (gpio_pullup_t) (pull_mode == GPIO_PULLUP_ONLY || pull_mode == GPIO_PULLUP_PULLDOWN);
    io_conf.pull_down_en = (gpio_pulldown_t) (pull_mode == GPIO_PULLDOWN_ONLY || pull_mode == GPIO_PULLUP_PULLDOWN);;
    io_conf.pin_bit_mask = pin_select;
    gpio_config(&io_conf);

    // Scan the pin map to determine number of pins
    pin_count = 0;
    for (int pin=0; pin<=39; pin++) {
        if ((1ULL<<pin) & pin_select) {
            pin_count++;
        }
    }

    // Initialize global state and queue
    debounce = (debounce_t *) calloc(pin_count, sizeof(debounce_t));
    queue = xQueueCreate(4, sizeof(button_event_t));

    // Scan the pin map to determine each pin number, populate the state
    uint32_t idx = 0;
    for (int pin=0; pin<=39; pin++) {
        if ((1ULL<<pin) & pin_select) {
            ESP_LOGI(TAG, "Registering button input: %d", pin);
            debounce[idx].pin = pin;
            debounce[idx].down_time = 0;
            debounce[idx].inverted = true;
            if (debounce[idx].inverted) debounce[idx].history = 0xffff;
            idx++;
        }
    }

    // Spawn a task to monitor the pins
    xTaskCreate(&button_task, "button_task", 4096, NULL, 10, NULL);

    return queue;
}
// END DEBOUNCER


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
    lv_obj_t *arc = lv_arc_create(lv_scr_act(), NULL);
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
    button_event_t ev;
    QueueHandle_t button_events = button_init(PIN_BIT(BTN_1) | PIN_BIT(BTN_2) | PIN_BIT(BTN_3));
    
    for ( ;; ) {
        if (xQueueReceive(button_events, &ev, 1000/portTICK_PERIOD_MS)) {
            if ((ev.pin == BTN_1) && (ev.event == BUTTON_DOWN)) {
                Serial.println("Btn1 pressed");
            }
            if ((ev.pin == BTN_2) && (ev.event == BUTTON_DOWN)) {
                Serial.println("Btn2 pressed");
            }
            if ((ev.pin == BTN_3) && (ev.event == BUTTON_DOWN)) {
                Serial.println("Btn3 pressed");
            }
        }
    }
    
    vTaskDelete(NULL);
}

void read_sensors(void *parameter) {
    for ( ;; )
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    vTaskDelete(NULL);
}

void init_drivers() {
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
