/*
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>

#include <zmk/display.h>

#include "custom_idle_image.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

LV_IMG_DECLARE(idle_face);

/*
 * Accessed only during screen initialization and from the display work queue.
 */
static lv_obj_t *idle_image_obj;
static bool applied_visible;

/*
 * Desired visibility may be updated from another thread.
 * The display work handler reads the latest requested value.
 */
static atomic_t requested_visible = ATOMIC_INIT(0);
static atomic_t image_ready = ATOMIC_INIT(0);

static void idle_image_update_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!atomic_get(&image_ready)) {
        return;
    }

    bool visible = atomic_get(&requested_visible) != 0;

    if (visible == applied_visible) {
        return;
    }

    if (visible) {
        lv_obj_move_foreground(idle_image_obj);
        lv_obj_clear_flag(idle_image_obj, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(idle_image_obj, LV_OBJ_FLAG_HIDDEN);
    }

    applied_visible = visible;
}

K_WORK_DEFINE(idle_image_update_work, idle_image_update_handler);

static void submit_idle_image_update(void)
{
    int ret = k_work_submit_to_queue(
        zmk_display_work_q(),
        &idle_image_update_work
    );

    if (ret < 0) {
        LOG_ERR("Failed to submit idle image update: %d", ret);
    }
}

void custom_idle_image_init(lv_obj_t *parent)
{
    /*
     * Called while the status screen is being constructed.
     * Do not call this function from the idle thread.
     */
    if (atomic_get(&image_ready)) {
        LOG_WRN("Idle image is already initialized");
        return;
    }

    idle_image_obj = lv_img_create(parent);

    if (idle_image_obj == NULL) {
        LOG_ERR("Failed to create idle image");
        return;
    }

    lv_img_set_src(idle_image_obj, &idle_face);
    
    /* Сдвигаем картинку на 2 пикселя вверх, чтобы убрать артефакт снизу */
    lv_obj_align(idle_image_obj, LV_ALIGN_CENTER, 0, 10);
    
    /* Убираем возможность прокрутки, чтобы не было полос */
    lv_obj_clear_flag(idle_image_obj, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_add_flag(idle_image_obj, LV_OBJ_FLAG_HIDDEN);

    applied_visible = false;
    atomic_set(&image_ready, 1);

    /*
     * Apply a request that may have arrived before screen initialization.
     */
    if (atomic_get(&requested_visible)) {
        submit_idle_image_update();
    }
}

void custom_idle_image_set_visible(bool visible)
{
    atomic_val_t desired = visible ? 1 : 0;
    atomic_val_t previous = atomic_set(&requested_visible, desired);

    if (!atomic_get(&image_ready)) {
        return;
    }

    /*
     * No LVGL calls or work submission when the requested state
     * has not changed.
     */
    if (previous == desired) {
        return;
    }

    submit_idle_image_update();
}
