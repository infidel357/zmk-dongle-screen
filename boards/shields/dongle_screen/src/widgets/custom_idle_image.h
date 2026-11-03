/*
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <lvgl.h>

#if defined(CONFIG_DONGLE_SCREEN_IDLE_IMAGE) && CONFIG_DONGLE_SCREEN_IDLE_IMAGE

void custom_idle_image_init(lv_obj_t *parent);
void custom_idle_image_set_visible(bool visible);

#else

static inline void custom_idle_image_init(lv_obj_t *parent)
{
    (void)parent;
}

static inline void custom_idle_image_set_visible(bool visible)
{
    (void)visible;
}

#endif