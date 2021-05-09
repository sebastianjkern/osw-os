#include "../lvgl/src/lvgl.h"

#define APP_NUMBER 3

lv_obj_t *app1();
lv_obj_t *app2();
lv_obj_t *app3();

static void arc_loader(lv_task_t *t);

typedef struct
{
    uint8_t screen_id = 0;

    lv_obj_t *(*functions[APP_NUMBER])(void) = {
        app1,
        app2,
        app3};
} screens_t;
