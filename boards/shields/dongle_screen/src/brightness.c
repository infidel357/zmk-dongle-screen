/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/layer_state_changed.h>

#include "widgets/custom_idle_image.h"

#include <stdlib.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if CONFIG_DONGLE_SCREEN_MIN_BRIGHTNESS > CONFIG_DONGLE_SCREEN_MAX_BRIGHTNESS
#error "DONGLE_SCREEN_MIN_BRIGHTNESS must be less than or equal to DONGLE_SCREEN_MAX_BRIGHTNESS!"
#endif

#if CONFIG_DONGLE_SCREEN_DEFAULT_BRIGHTNESS + CONFIG_DONGLE_SCREEN_BRIGHTNESS_MODIFIER < CONFIG_DONGLE_SCREEN_MIN_BRIGHTNESS
#error "DONGLE_SCREEN_DEFAULT_BRIGHTNESS + MODIFIER can't be smaller than MIN brightness value!"
#endif

#if CONFIG_DONGLE_SCREEN_DEFAULT_BRIGHTNESS + CONFIG_DONGLE_SCREEN_BRIGHTNESS_MODIFIER > CONFIG_DONGLE_SCREEN_MAX_BRIGHTNESS
#error "DONGLE_SCREEN_DEFAULT_BRIGHTNESS + MODIFIER can't be greater than MAX brightness value!"
#endif

#if CONFIG_DONGLE_SCREEN_BRIGHTNESS_MODIFIER + CONFIG_DONGLE_SCREEN_MIN_BRIGHTNESS > CONFIG_DONGLE_SCREEN_MAX_BRIGHTNESS
#error "DONGLE_SCREEN_BRIGHTNESS_MODIFIER + DONGLE_SCREEN_MIN_BRIGHTNESS can't be greater than MAX brightness!"
#endif

#if CONFIG_DONGLE_SCREEN_BRIGHTNESS_MODIFIER + CONFIG_DONGLE_SCREEN_MAX_BRIGHTNESS < CONFIG_DONGLE_SCREEN_MIN_BRIGHTNESS
#error "DONGLE_SCREEN_BRIGHTNESS_MODIFIER + DONGLE_SCREEN_MAX_BRIGHTNESS can't be smaller than MIN brightness!"
#endif

#if CONFIG_DONGLE_SCREEN_AMBIENT_LIGHT && \
    (CONFIG_DONGLE_SCREEN_AMBIENT_LIGHT_MIN_RAW_VALUE >= \
     CONFIG_DONGLE_SCREEN_AMBIENT_LIGHT_MAX_RAW_VALUE)
#error "Ambient light MIN_RAW_VALUE must be less than MAX_RAW_VALUE!"
#endif

#define SCREEN_IDLE_TIMEOUT_MS \
    (CONFIG_DONGLE_SCREEN_IDLE_TIMEOUT_S * 1000LL)

#define BRIGHTNESS_CHANGE_THRESHOLD 5
#define FADE_QUEUE_SIZE 4

#if CONFIG_DONGLE_SCREEN_IDLE_IMAGE
#define SCREEN_IDLE_IMAGE_BLANK_DELAY_MS \
    (CONFIG_DONGLE_SCREEN_IDLE_IMAGE_BLANK_DELAY_S * 1000LL)

#define SCREEN_IDLE_TOTAL_TIMEOUT_MS \
    (SCREEN_IDLE_TIMEOUT_MS + SCREEN_IDLE_IMAGE_BLANK_DELAY_MS)
#endif

static const struct device *pwm_leds_dev = DEVICE_DT_GET_ONE(pwm_leds);

#define DISP_BL DT_NODE_CHILD_IDX(DT_NODELABEL(disp_bl))

/*
 * Protects last_activity, screen_on, off_through_modifier,
 * current_brightness and brightness_modifier.
 *
 * Internal helpers ending in "_locked" require this mutex.
 * No LVGL calls are made directly from this file.
 */
K_MUTEX_DEFINE(brightness_state_mutex);

#if CONFIG_DONGLE_SCREEN_IDLE_TIMEOUT_S > 0
/*
 * A pending activity notification is retained even when the idle
 * thread has not entered its wait yet.
 */
K_SEM_DEFINE(screen_idle_signal, 0, 1);
#endif

static int64_t last_activity;

static const uint8_t max_brightness =
    CONFIG_DONGLE_SCREEN_MAX_BRIGHTNESS;

static const uint8_t min_brightness =
    CONFIG_DONGLE_SCREEN_MIN_BRIGHTNESS;

static int8_t current_brightness =
    CONFIG_DONGLE_SCREEN_DEFAULT_BRIGHTNESS;

static int8_t brightness_modifier =
    CONFIG_DONGLE_SCREEN_BRIGHTNESS_MODIFIER;

static bool screen_on = true;
static bool off_through_modifier;

struct brightness_result {
    uint8_t adjusted_brightness;
    int8_t adjusted_modifier;
    uint8_t effective_brightness;
    bool was_clamped;
    bool hit_min_limit;
    bool hit_max_limit;
};

struct fade_request_t {
    uint8_t from;
    uint8_t to;
};

K_MSGQ_DEFINE(
    fade_msgq,
    sizeof(struct fade_request_t),
    FADE_QUEUE_SIZE,
    4
);

int random0to100(void)
{
    return rand() % 101;
}

/*
 * int16_t avoids narrowing sums such as brightness + modifier
 * to int8_t before checking their bounds.
 */
static uint8_t clamp_brightness(int16_t value)
{
    if (value > max_brightness) {
        return max_brightness;
    }

    if (value < min_brightness) {
        return min_brightness;
    }

    return (uint8_t)value;
}

static void apply_brightness(uint8_t value)
{
    int ret = led_set_brightness(pwm_leds_dev, DISP_BL, value);

    if (ret < 0) {
        LOG_ERR("Failed to set screen brightness: %d", ret);
        return;
    }

    LOG_DBG("Screen brightness set to %u", value);
}

static int8_t calculate_safe_modifier_change(
    uint8_t base_brightness,
    int8_t current_modifier,
    int8_t desired_change)
{
    int16_t current_effective =
        base_brightness + current_modifier;

    int16_t desired_effective =
        current_effective + desired_change;

    int16_t boundary =
        desired_change > 0 ? max_brightness : min_brightness;

    if ((desired_change > 0 && desired_effective <= boundary) ||
        (desired_change < 0 && desired_effective >= boundary)) {
        return desired_change;
    }

    int16_t safe_change = boundary - current_effective;

    if ((desired_change > 0 && safe_change > 0) ||
        (desired_change < 0 && safe_change < 0)) {
        return (int8_t)safe_change;
    }

    return 0;
}

static struct brightness_result calculate_brightness_with_bounds(
    uint8_t base_brightness,
    int8_t modifier,
    bool enforce_ambient_constraints)
{
    struct brightness_result result = {0};

    result.adjusted_brightness = clamp_brightness(base_brightness);
    result.adjusted_modifier = modifier;

#if CONFIG_DONGLE_SCREEN_AMBIENT_LIGHT
    if (enforce_ambient_constraints) {
        int16_t effective =
            result.adjusted_brightness + result.adjusted_modifier;

        if (effective <= min_brightness) {
            int16_t needed_increase =
                min_brightness - effective + 1;

            result.adjusted_brightness = clamp_brightness(
                result.adjusted_brightness + needed_increase
            );

            result.was_clamped = true;
            result.hit_min_limit = true;
        } else if (effective > max_brightness) {
            int16_t needed_decrease =
                effective - max_brightness;

            if (result.adjusted_brightness >= needed_decrease) {
                result.adjusted_brightness -= needed_decrease;
            } else {
                result.adjusted_brightness = min_brightness;
            }

            result.was_clamped = true;
            result.hit_max_limit = true;
        }
    }
#else
    ARG_UNUSED(enforce_ambient_constraints);
#endif

    result.effective_brightness = clamp_brightness(
        result.adjusted_brightness + result.adjusted_modifier
    );

    if (result.effective_brightness <= min_brightness) {
        result.hit_min_limit = true;
    }

    if (result.effective_brightness >= max_brightness) {
        result.hit_max_limit = true;
    }

    return result;
}

static bool should_screen_turn_off(
    uint8_t base_brightness,
    int8_t modifier)
{
    return (base_brightness + modifier) < min_brightness;
}

static bool should_screen_turn_on(
    uint8_t base_brightness,
    int8_t modifier)
{
    return (base_brightness + modifier) > min_brightness;
}

static float ease_in_out(float t)
{
    if (t < 0.5f) {
        return 4.0f * t * t * t;
    }

    float f = -2.0f * t + 2.0f;

    return 1.0f - (f * f * f) / 2.0f;
}

void fade_thread(void)
{
    struct fade_request_t req;

    while (1) {
        if (k_msgq_get(&fade_msgq, &req, K_FOREVER) != 0) {
            continue;
        }

        if (req.from == req.to || abs(req.to - req.from) <= 1) {
            apply_brightness(req.to);
            continue;
        }

        int diff = abs(req.to - req.from);
        int steps = CLAMP(diff * 2, 6, 32);
        int total_duration_ms = CLAMP(diff * 20, 500, 1000);
        int delay_us = (total_duration_ms * 1000) / steps;

        uint8_t last_applied = 255;

        for (int i = 0; i <= steps; i++) {
            float t = (float)i / steps;
            float eased = ease_in_out(t);

            float interpolated =
                req.from + (req.to - req.from) * eased;

            uint8_t brightness =
                (uint8_t)(interpolated + 0.5f);

            if (brightness != last_applied) {
                apply_brightness(brightness);
                last_applied = brightness;
            }

            k_usleep(delay_us);
        }

        if (last_applied != req.to) {
            apply_brightness(req.to);
        }
    }
}

K_THREAD_DEFINE(
    fade_tid,
    768,
    fade_thread,
    NULL,
    NULL,
    NULL,
    6,
    0,
    0
);

static void fade_to_brightness(uint8_t from, uint8_t to)
{
    struct fade_request_t req = {
        .from = from,
        .to = to,
    };

    k_msgq_purge(&fade_msgq);

    int ret = k_msgq_put(&fade_msgq, &req, K_NO_WAIT);

    if (ret != 0) {
        LOG_WRN("Failed to enqueue brightness fade: %d", ret);
    }
}

static void set_screen_brightness_locked(uint8_t value, bool ambient)
{
    struct brightness_result result =
        calculate_brightness_with_bounds(
            value,
            brightness_modifier,
            ambient
        );

    uint8_t current_effective = clamp_brightness(
        current_brightness + brightness_modifier
    );

    current_brightness = result.adjusted_brightness;

    /*
     * Updating the desired brightness must not illuminate a
     * manually disabled or idle-blanked screen.
     */
    if (screen_on) {
        fade_to_brightness(
            current_effective,
            result.effective_brightness
        );
    }
}

void set_screen_brightness(uint8_t value, bool ambient)
{
    k_mutex_lock(&brightness_state_mutex, K_FOREVER);
    set_screen_brightness_locked(value, ambient);
    k_mutex_unlock(&brightness_state_mutex);
}

static void screen_set_on_locked(bool on)
{
    if (on && !screen_on) {
        if (should_screen_turn_off(
                current_brightness,
                brightness_modifier)) {
            struct brightness_result result =
                calculate_brightness_with_bounds(
                    current_brightness,
                    brightness_modifier,
                    false
                );

            current_brightness = result.adjusted_brightness;
        }

        fade_to_brightness(
            0,
            clamp_brightness(
                current_brightness + brightness_modifier
            )
        );

        screen_on = true;
        off_through_modifier = false;

        LOG_INF("Screen on (smooth)");
    } else if (!on && screen_on) {
        fade_to_brightness(
            clamp_brightness(
                current_brightness + brightness_modifier
            ),
            0
        );

        screen_on = false;

        LOG_INF("Screen off (smooth)");
    }
}

static void notify_idle_thread(void)
{
#if CONFIG_DONGLE_SCREEN_IDLE_TIMEOUT_S > 0
    k_sem_give(&screen_idle_signal);
#endif
}

/*
 * Called with brightness_state_mutex held.
 *
 * wake_automatically respects a manually disabled screen.
 * Reconnect has a separate explicit wake policy below.
 */
static void record_activity_locked(bool wake_automatically)
{
    last_activity = k_uptime_get();

    custom_idle_image_set_visible(false);

    if (wake_automatically &&
        !screen_on &&
        !off_through_modifier) {
        screen_set_on_locked(true);
    }
}

#if CONFIG_DONGLE_SCREEN_IDLE_TIMEOUT_S > 0

void screen_idle_thread(void)
{
    /*
     * Start idle evaluation only after initialization has supplied
     * the initial timestamp and signaled readiness.
     */
    k_sem_take(&screen_idle_signal, K_FOREVER);

    while (1) {
        k_timeout_t wait = K_FOREVER;

        k_mutex_lock(&brightness_state_mutex, K_FOREVER);

        int64_t elapsed = k_uptime_get() - last_activity;

        /*
         * An automatically blanked screen waits for activity.
         * A manually disabled screen still reaches its idle
         * deadline, preserving the existing YADS wake policy.
         */
        if (screen_on || off_through_modifier) {
#if CONFIG_DONGLE_SCREEN_IDLE_IMAGE
            if (elapsed >= SCREEN_IDLE_TOTAL_TIMEOUT_MS) {
                /*
                 * Keep the idle image visible while the backlight
                 * fades out. Activity will request hiding it.
                 */
                screen_set_on_locked(false);
                off_through_modifier = false;
            } else if (elapsed >= SCREEN_IDLE_TIMEOUT_MS) {
                custom_idle_image_set_visible(screen_on);

                wait = K_MSEC(
                    SCREEN_IDLE_TOTAL_TIMEOUT_MS - elapsed
                );
            } else {
                custom_idle_image_set_visible(false);

                wait = K_MSEC(
                    SCREEN_IDLE_TIMEOUT_MS - elapsed
                );
            }
#else
            if (elapsed >= SCREEN_IDLE_TIMEOUT_MS) {
                screen_set_on_locked(false);
                off_through_modifier = false;
            } else {
                wait = K_MSEC(
                    SCREEN_IDLE_TIMEOUT_MS - elapsed
                );
            }
#endif
        }

        /*
         * Do not hide the image merely because screen_on is false:
         * the backlight may still be fading out.
         */
        k_mutex_unlock(&brightness_state_mutex);

        /*
         * Activity is retained by the semaphore even if it
         * arrives between unlocking and entering this wait.
         */
        k_sem_take(&screen_idle_signal, wait);
    }
}

K_THREAD_DEFINE(
    screen_idle_tid,
    1024,
    screen_idle_thread,
    NULL,
    NULL,
    NULL,
    7,
    0,
    0
);

void brightness_wake_screen_on_reconnect(void)
{
    k_mutex_lock(&brightness_state_mutex, K_FOREVER);

    /*
     * Preserve the original explicit reconnect wake behavior,
     * including waking a manually disabled screen.
     */
    record_activity_locked(false);
    screen_set_on_locked(true);

    k_mutex_unlock(&brightness_state_mutex);

    notify_idle_thread();

    LOG_DBG("Peripheral reconnect: screen activity reset");
}

#endif

#if CONFIG_DONGLE_SCREEN_BRIGHTNESS_KEYBOARD_CONTROL

static void increase_brightness_locked(void)
{
    int8_t safe_increase = calculate_safe_modifier_change(
        current_brightness,
        brightness_modifier,
        CONFIG_DONGLE_SCREEN_BRIGHTNESS_STEP
    );

    if (safe_increase <= 0) {
        LOG_DBG("Brightness is already at maximum");
        return;
    }

    brightness_modifier += safe_increase;

    set_screen_brightness_locked(current_brightness, false);

    if (should_screen_turn_on(
            current_brightness,
            brightness_modifier) &&
        off_through_modifier) {
        screen_set_on_locked(true);
    }
}

static void decrease_brightness_locked(void)
{
    int8_t safe_decrease = calculate_safe_modifier_change(
        current_brightness,
        brightness_modifier,
        -CONFIG_DONGLE_SCREEN_BRIGHTNESS_STEP
    );

    if (safe_decrease >= 0) {
        LOG_DBG("Brightness is already at minimum");
        return;
    }

    brightness_modifier += safe_decrease;

    set_screen_brightness_locked(current_brightness, false);

    if (should_screen_turn_off(
            current_brightness,
            brightness_modifier)) {
        off_through_modifier = true;
        screen_set_on_locked(false);
    }
}

#endif

#if CONFIG_DONGLE_SCREEN_IDLE_TIMEOUT_S > 0 || \
    CONFIG_DONGLE_SCREEN_BRIGHTNESS_KEYBOARD_CONTROL

static int key_listener(const zmk_event_t *eh)
{
    const struct zmk_keycode_state_changed *ev =
        as_zmk_keycode_state_changed(eh);

    k_mutex_lock(&brightness_state_mutex, K_FOREVER);

    /*
     * As in the original listener, key releases and layer events
     * also reset inactivity. Keyboard controls are processed
     * only on key down.
     */
    record_activity_locked(false);

    bool control_handled = false;

#if CONFIG_DONGLE_SCREEN_BRIGHTNESS_KEYBOARD_CONTROL
    if (ev && ev->state) {
        if (ev->keycode ==
            CONFIG_DONGLE_SCREEN_BRIGHTNESS_UP_KEYCODE) {
            increase_brightness_locked();
            control_handled = true;
        } else if (ev->keycode ==
                   CONFIG_DONGLE_SCREEN_BRIGHTNESS_DOWN_KEYCODE) {
            decrease_brightness_locked();
            control_handled = true;
        } else if (ev->keycode ==
                   CONFIG_DONGLE_SCREEN_TOGGLE_KEYCODE) {
            if (screen_on) {
                off_through_modifier = true;
                screen_set_on_locked(false);
            } else {
                screen_set_on_locked(true);
            }

            control_handled = true;
        }
    }
#else
    ARG_UNUSED(ev);
#endif

    if (!control_handled) {
#if CONFIG_DONGLE_SCREEN_IDLE_TIMEOUT_S > 0
        if (!screen_on && !off_through_modifier) {
            screen_set_on_locked(true);
        }
#else
        /*
         * Preserve the original behavior with idle disabled.
         */
        if (!screen_on) {
            screen_set_on_locked(true);
        }
#endif
    }

    k_mutex_unlock(&brightness_state_mutex);

    notify_idle_thread();

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(screen_idle, key_listener);
ZMK_SUBSCRIPTION(screen_idle, zmk_keycode_state_changed);
ZMK_SUBSCRIPTION(screen_idle, zmk_layer_state_changed);

#endif

#if CONFIG_DONGLE_SCREEN_AMBIENT_LIGHT

#define AMBIENT_LIGHT_SENSOR_NODE DT_INST(0, avago_apds9960)

static const struct device *ambient_sensor =
    DEVICE_DT_GET(AMBIENT_LIGHT_SENSOR_NODE);

const int32_t min_sensor =
    CONFIG_DONGLE_SCREEN_AMBIENT_LIGHT_MIN_RAW_VALUE;

const int32_t max_sensor =
    CONFIG_DONGLE_SCREEN_AMBIENT_LIGHT_MAX_RAW_VALUE;

static uint8_t ambient_to_brightness(int32_t sensor_value)
{
    sensor_value = CLAMP(
        sensor_value,
        min_sensor,
        max_sensor
    );

    uint8_t brightness =
        min_brightness +
        ((sensor_value - min_sensor) *
         (max_brightness - min_brightness)) /
        (max_sensor - min_sensor);

    return clamp_brightness(brightness);
}

static void ambient_light_thread(void)
{
    struct sensor_value val;
    uint8_t last_brightness = 0xFF;

    while (1) {
#ifndef CONFIG_DONGLE_SCREEN_AMBIENT_LIGHT_TEST
        if (!device_is_ready(ambient_sensor)) {
            LOG_ERR("Ambient light sensor not ready");
            k_sleep(K_SECONDS(5));
            continue;
        }

        int rc = sensor_sample_fetch(ambient_sensor);

        if (rc < 0) {
            k_sleep(K_MSEC(
                CONFIG_DONGLE_SCREEN_AMBIENT_LIGHT_EVALUATION_INTERVAL_MS
            ));
            continue;
        }

        rc = sensor_channel_get(
            ambient_sensor,
            SENSOR_CHAN_LIGHT,
            &val
        );

        if (rc < 0) {
            k_sleep(K_MSEC(
                CONFIG_DONGLE_SCREEN_AMBIENT_LIGHT_EVALUATION_INTERVAL_MS
            ));
            continue;
        }
#else
        k_sleep(K_SECONDS(10));
        val.val1 = random0to100();
#endif

        uint8_t new_brightness =
            ambient_to_brightness(val.val1);

        if (abs((int)new_brightness -
                (int)last_brightness) >
            BRIGHTNESS_CHANGE_THRESHOLD) {
            k_mutex_lock(
                &brightness_state_mutex,
                K_FOREVER
            );

            set_screen_brightness_locked(
                new_brightness,
                true
            );

            k_mutex_unlock(&brightness_state_mutex);

            last_brightness = new_brightness;
        }

        k_sleep(K_MSEC(
            CONFIG_DONGLE_SCREEN_AMBIENT_LIGHT_EVALUATION_INTERVAL_MS
        ));
    }
}

K_THREAD_DEFINE(
    ambient_light_tid,
    1024,
    ambient_light_thread,
    NULL,
    NULL,
    NULL,
    7,
    0,
    0
);

#endif

static int init_fixed_brightness(void)
{
    if (!device_is_ready(pwm_leds_dev)) {
        LOG_ERR("Screen backlight device is not ready");
        return -ENODEV;
    }

    k_mutex_lock(&brightness_state_mutex, K_FOREVER);

    set_screen_brightness_locked(
        current_brightness,
        false
    );

    last_activity = k_uptime_get();

    k_mutex_unlock(&brightness_state_mutex);

    notify_idle_thread();

#if CONFIG_DONGLE_SCREEN_IDLE_TIMEOUT_S == 0
    LOG_INF("Screen idle timeout disabled");
#endif

    return 0;
}

SYS_INIT(
    init_fixed_brightness,
    APPLICATION,
    CONFIG_APPLICATION_INIT_PRIORITY
);
