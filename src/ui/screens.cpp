#include "gui.hpp"

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

lv_obj_t *app1()
{
    static lv_task_t *task = NULL;

    lv_obj_t *scr = lv_obj_create(NULL, NULL);

    lv_obj_t *arc = lv_arc_create(scr, NULL);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_angles(arc, 270, 270);
    lv_obj_align(arc, NULL, LV_ALIGN_CENTER, 0, 0);

    if (task == NULL)
    {
        /*Prevent task from being created double*/
        task = lv_task_create(arc_loader, 20, LV_TASK_PRIO_LOWEST, arc);
    }

    return scr;
}

lv_obj_t *app2()
{
    lv_obj_t *scr = lv_obj_create(NULL, NULL);
    return scr;
}

lv_obj_t *app3()
{
    lv_obj_t *scr = lv_obj_create(NULL, NULL);
    return scr;
}
