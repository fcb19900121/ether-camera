/*
 * External Trigger Module for Camera Capture
 * 
 * Features:
 * - Rising edge: Reset timestamp, start capture
 * - Falling edge: Stop capture
 * - Timestamp embedded in each frame
 */
#ifndef APP_TRIGGER_H
#define APP_TRIGGER_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Trigger GPIO pin (change as needed)
#define TRIGGER_GPIO_PIN    CONFIG_TRIGGER_GPIO_PIN

/**
 * @brief Frame info structure with timestamp
 * 
 * Timing diagram:
 *   Trigger → STREAMON → Exposure → Transfer → Callback
 *                         ↑                      ↑
 *                   capture_time_us        callback_time_us
 */
typedef struct {
    uint32_t frame_number;          // Frame sequence number since trigger
    int64_t timestamp_us;           // Timestamp in us since trigger (legacy, same as callback_time)
    int64_t callback_time_us;       // When callback was invoked (frame transfer complete)
    int64_t capture_time_us;        // Estimated actual capture time (callback - transfer_time)
    int64_t transfer_time_us;       // Frame transfer duration (for reference)
    bool is_triggered;              // Whether capture is active (triggered)
} trigger_frame_info_t;

// Frame transfer time: actual MIPI transfer measured ~0.1ms (V4L2 timestamp used when available)
// This fallback constant is only used when V4L2 frame timestamp is 0
#define FRAME_TRANSFER_TIME_US  500   // ~0.5ms conservative fallback

/**
 * @brief Initialize the external trigger module
 * 
 * Configures GPIO interrupt for rising and falling edge detection.
 * 
 * @return ESP_OK on success
 */
esp_err_t app_trigger_init(void);

/**
 * @brief Deinitialize the trigger module
 */
void app_trigger_deinit(void);

/**
 * @brief Check if capture is triggered (active)
 * 
 * @return true if triggered (between rising and falling edge)
 */
bool app_trigger_is_active(void);

/**
 * @brief Get current timestamp since last trigger
 * 
 * @return Timestamp in microseconds since rising edge
 */
int64_t app_trigger_get_timestamp_us(void);

/**
 * @brief Get the trigger start time (absolute system time)
 * 
 * @return Trigger start time in microseconds (esp_timer_get_time() at trigger)
 */
int64_t app_trigger_get_start_time(void);

/**
 * @brief Get frame info for current frame (does NOT allocate frame number)
 * 
 * Call this at the start of frame processing to check trigger state and get timestamp.
 * Frame number will be 0 - call app_trigger_alloc_frame_number() after successful
 * encoding to get the actual frame number.
 * 
 * @param info Pointer to frame info structure to fill
 * @return ESP_OK if triggered and info filled, ESP_ERR_INVALID_STATE if not triggered
 */
esp_err_t app_trigger_get_frame_info(trigger_frame_info_t *info);

/**
 * @brief Allocate a frame number for a successfully encoded frame
 * 
 * Call this ONLY after a frame has been successfully encoded and is ready to be stored.
 * Frame numbers start from 1 and increment sequentially.
 * 
 * @return Allocated frame number (1, 2, 3, ...), or 0 if allocation failed
 */
uint32_t app_trigger_alloc_frame_number(void);

/**
 * @brief Manually trigger capture start (for testing without GPIO)
 */
void app_trigger_manual_start(void);

/**
 * @brief Manually trigger capture stop (for testing without GPIO)
 */
void app_trigger_manual_stop(void);

/**
 * @brief Reset trigger state (called when camera stream stops)
 * Clears start time so app_trigger_get_frame_info returns invalid state
 */
void app_trigger_reset(void);

/**
 * @brief Trigger state change callback type
 * @param active true when trigger becomes active (rising edge), false when inactive (falling edge)
 */
typedef void (*app_trigger_callback_t)(bool active);

/**
 * @brief Register callback for trigger state changes
 * @param callback Callback function to call on trigger state changes
 */
void app_trigger_register_callback(app_trigger_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif // APP_TRIGGER_H
