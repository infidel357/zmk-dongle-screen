/*
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/lvgl.h>
#include "custom_idle_image.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if CONFIG_DONGLE_SCREEN_IDLE_IMAGE

LV_IMG_DECLARE(idle_face);

static lv_obj_t *idle_image_obj = NULL;

void custom_idle_image_init(lv_obj_t *parent)
{
    idle_image_obj = lv_img_create(parent);
    lv_img_set_src(idle_image_obj, &idle_face);
    lv_obj_align(idle_image_obj, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(idle_image_obj, LV_OBJ_FLAG_HIDDEN);
}

void custom_idle_image_set_visible(bool visible)
{
    if (idle_image_obj == NULL) {
        return;
    }

    if (!lvgl_lock(-1)) {
        LOG_WRN("Could not lock LVGL for idle image update");
        return;
    }

    if (visible) {
        lv_obj_clear_flag(idle_image_obj, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(idle_image_obj);
    } else {
        lv_obj_add_flag(idle_image_obj, LV_OBJ_FLAG_HIDDEN);
    }

    lvgl_unlock();
}

#else

void custom_idle_image_init(lv_obj_t *parent)
{
    ARG_UNUSED(parent);
}

void custom_idle_image_set_visible(bool visible)
{
    ARG_UNUSED(visible);
}

#endif