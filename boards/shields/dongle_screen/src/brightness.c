/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>
#include <zephyr/lvgl.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include "widgets/custom_idle_image.h"
#include <math.h>
#include <stdlib.h>

int random0to100()
{
    return rand() % 101; // 0 to 100
}

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
#error "DONGLE_SCREEN_BRIGHTNESS_MODIFIER + DONGLE_SCREEN_MIN_BRIGHTNESS can't be greater than DONGLE_SCREEN_MAX_BRIGHTNESS!"
#endif

#if CONFIG_DONGLE_SCREEN_BRIGHTNESS_MODIFIER + CONFIG_DONGLE_SCREEN_MAX_BRIGHTNESS < CONFIG_DONGLE_SCREEN_MIN_BRIGHTNESS
#error "DONGLE_SCREEN_BRIGHTNESS_MODIFIER + DONGLE_SCREEN_MAX_BRIGHTNESS can't be smaller than DONGLE_SCREEN_MIN_BRIGHTNESS!"
#endif

#if CONFIG_DONGLE_SCREEN_AMBIENT_LIGHT && (CONFIG_DONGLE_SCREEN_AMBIENT_LIGHT_MIN_RAW_VALUE > CONFIG_DONGLE_SCREEN_AMBIENT_LIGHT_MAX_RAW_VALUE)
#error "DONGLE_SCREEN_AMBIENT_LIGHT_MIN_RAW_VALUE can't be greater than DONGLE_SCREEN_AMBIENT_LIGHT_MAX_RAW_VALUE when DONGLE_SCREEN_AMBIENT_LIGHT is activated!"
#endif

#define BRIGHTNESS_STEP 1
#define BRIGHTNESS_DELAY_MS 2
#define BRIGHTNESS_FADE_DURATION_MS 500
#define SCREEN_IDLE_TIMEOUT_MS (CONFIG_DONGLE_SCREEN_IDLE_TIMEOUT_S * 1000)
#define BRIGHTNESS_CHANGE_THRESHOLD 5

#if CONFIG_DONGLE_SCREEN_IDLE_IMAGE
#define SCREEN_IDLE_IMAGE_BLANK_DELAY_MS \
    (CONFIG_DONGLE_SCREEN_IDLE_IMAGE_BLANK_DELAY_S * 1000)
#define SCREEN_IDLE_TOTAL_TIMEOUT_MS \
    (SCREEN_IDLE_TIMEOUT_MS + SCREEN_IDLE_IMAGE_BLANK_DELAY_MS)
#endif

static const struct device *pwm_leds_dev = DEVICE_DT_GET_ONE(pwm_leds);
#define DISP_BL DT_NODE_CHILD_IDX(DT_NODELABEL(disp_bl))

static int64_t last_activity = 0;
static uint8_t max_brightness = CONFIG_DONGLE_SCREEN_MAX_BRIGHTNESS;
static uint8_t min_brightness = CONFIG_DONGLE_SCREEN_MIN_BRIGHTNESS;
static int8_t current_brightness = CONFIG_DONGLE_SCREEN_DEFAULT_BRIGHTNESS;

static int8_t brightness_modifier = CONFIG_DONGLE_SCREEN_BRIGHTNESS_MODIFIER;

static bool off_through_modifier = false; // Used to track if the screen was turned off through the brightness modifier

/**
 * @brief Structure to hold brightness calculation results
 */
struct brightness_result
{
    uint8_t adjusted_brightness;  // The adjusted base brightness value
    int8_t adjusted_modifier;     // The adjusted modifier value
    uint8_t effective_brightness; // Final brightness (adjusted_brightness + adjusted_modifier)
    bool was_clamped;             // Whether any clamping occurred
    bool hit_min_limit;           // Whether minimum limit was reached
    bool hit_max_limit;           // Whether maximum limit was reached
};

static uint8_t clamp_brightness(int8_t value)
{
    if (value > max_brightness)
    {
        LOG_WRN("CLAMPED: Screen brightness %d would be over %d", value, max_brightness);
        return max_brightness;
    }
    if (value < min_brightness)
    {
        LOG_WRN("CLAMPED: Screen brightness %d would be under %d", value, min_brightness);
        return min_brightness;
    }
    return value;
}

static void apply_brightness(uint8_t value)
{
    led_set_brightness(pwm_leds_dev, DISP_BL, value);
    LOG_INF("Screen brightness set to %d", value);
}

static int8_t calculate_safe_modifier_change(uint8_t base_brightness, int8_t current_modifier, int8_t desired_change)
{
    int16_t current_effective = base_brightness + current_modifier;
    int16_t desired_effective = current_effective + desired_change;

    // Determine the appropriate boundary based on direction of change
    int16_t boundary = (desired_change > 0) ? max_brightness : min_brightness;

    // Check if the desired change is within bounds
    if ((desired_change > 0 && desired_effective <= boundary) ||
        (desired_change < 0 && desired_effective >= boundary))
    {
        return desired_change; // Full change is safe
    }

    // Calculate maximum safe change toward the boundary
    int16_t safe_change = boundary - current_effective;

    // Ensure we don't return a change in the wrong direction or zero when some change is possible
    if ((desired_change > 0 && safe_change > 0) || (desired_change < 0 && safe_change < 0))
    {
        return (int8_t)safe_change;
    }

    return 0; // No safe change possible
}

static struct brightness_result calculate_brightness_with_bounds(uint8_t base_brightness, int8_t modifier, bool enforce_ambient_constraints)
{
    struct brightness_result result = {0};

    // Start with clamped base brightness
    result.adjusted_brightness = clamp_brightness(base_brightness);
    result.adjusted_modifier = modifier;

    // Handle ambient light constraints by adjusting base brightness
    if (enforce_ambient_constraints)
    {
#if CONFIG_DONGLE_SCREEN_AMBIENT_LIGHT
        int16_t effective = result.adjusted_brightness + result.adjusted_modifier;
        if (effective <= min_brightness)
        {
            // Need to increase base brightness to meet minimum
            uint8_t needed_increase = min_brightness - effective + 1; // +1 to get above minimum
            uint8_t old_brightness = result.adjusted_brightness;

            result.adjusted_brightness = clamp_brightness(result.adjusted_brightness + needed_increase);
            result.was_clamped = true;
            result.hit_min_limit = true;

            LOG_DBG("Ambient: brightness (%d) + modifier (%d) = %d below min (%d), adjusted brightness by +%d to %d, resulting in %d",
                    old_brightness, modifier, effective, min_brightness,
                    result.adjusted_brightness - old_brightness, result.adjusted_brightness,
                    result.adjusted_brightness + modifier);
        }
        else if (effective > max_brightness)
        {
            // Need to decrease base brightness to stay within maximum
            uint8_t needed_decrease = effective - max_brightness;
            uint8_t old_brightness = result.adjusted_brightness;

            if (result.adjusted_brightness >= needed_decrease)
            {
                result.adjusted_brightness -= needed_decrease;
            }
            else
            {
                result.adjusted_brightness = min_brightness;
            }

            result.was_clamped = true;
            result.hit_max_limit = true;

            LOG_DBG("Ambient: brightness (%d) + modifier (%d) = %d above max (%d), adjusted brightness by -%d to %d, resulting in %d",
                    old_brightness, modifier, effective, max_brightness,
                    old_brightness - result.adjusted_brightness, result.adjusted_brightness,
                    result.adjusted_brightness + modifier);
        }
#endif
    }

    // Recalculate effective brightness after any ambient adjustments
    result.effective_brightness = clamp_brightness(result.adjusted_brightness + result.adjusted_modifier);

    // Final boundary check
    if (result.effective_brightness <= min_brightness)
    {
        result.hit_min_limit = true;
    }
    if (result.effective_brightness >= max_brightness)
    {
        result.hit_max_limit = true;
    }

    return result;
}

static bool should_screen_turn_off(uint8_t base_brightness, int8_t modifier)
{
    return (base_brightness + modifier) < min_brightness;
}

static bool should_screen_turn_on(uint8_t base_brightness, int8_t modifier)
{
    return (base_brightness + modifier) > min_brightness;
}

// Threaded fade logic
// Contains starting and target brightness levels to be animated
struct fade_request_t
{
    uint8_t from; // Starting brightness level
    uint8_t to;   // Target brightness level
};

#define FADE_QUEUE_SIZE 4

// Message queue used to send fade requests to the fade handler thread.
// It holds up to 4 fade_request_t elements and ensures brightness updates are handled sequentially.
K_MSGQ_DEFINE(fade_msgq, sizeof(struct fade_request_t), FADE_QUEUE_SIZE, 4);

// Cubic ease-in-out function to smooth the interpolation curve.
// Provides a natural "S-curve" animation effect: starts slow, accelerates, then slows again.
// Helps avoid abrupt changes in perceived brightness.
static float ease_in_out(float t)
{
    if (t < 0