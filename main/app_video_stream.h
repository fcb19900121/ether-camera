/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef APP_VIDEO_STREAM_H
#define APP_VIDEO_STREAM_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_TRIGGER_MODE_GPIO = 0,
    APP_TRIGGER_MODE_UDP = 1,
} app_trigger_mode_t;

typedef struct {
    bool active;
    uint32_t frame_count;
    uint32_t dropped_frames_est;
    int64_t first_frame_latency_us;
    int64_t avg_frame_interval_us;
    int64_t last_frame_interval_us;
} app_trigger_stats_t;

/**
 * @brief Initialize and start the video stream HTTP server
 * 
 * @param video_fd File descriptor for the video device
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t app_video_stream_server_start(int video_fd);

/**
 * @brief Stop the video stream HTTP server
 * 
 * @return ESP_OK on success
 */
esp_err_t app_video_stream_server_stop(void);

/**
 * @brief Get the HTTP server handle
 * 
 * @return HTTP server handle
 */
httpd_handle_t app_video_stream_get_server(void);

/**
 * @brief Get the calibrated startup delay (from self-test)
 * 
 * This returns the average delay from STREAMON to first frame arrival,
 * measured during power-on self-test.
 * 
 * @return Calibrated delay in microseconds
 */
int64_t app_video_stream_get_calibrated_delay_us(void);

/**
 * @brief Get the startup delay jitter (standard deviation)
 * 
 * @return Jitter in microseconds
 */
int64_t app_video_stream_get_delay_jitter_us(void);

/**
 * @brief Check if startup calibration has been performed
 * 
 * @return true if calibration is done
 */
bool app_video_stream_is_calibrated(void);

/**
 * @brief Extract timestamp from JPEG data with embedded timestamp
 * 
 * Each JPEG frame contains an embedded APP4 segment with:
 * - Capture time (microseconds relative to trigger)
 * - Frame sequence number
 * 
 * @param jpeg_data JPEG data with embedded timestamp
 * @param jpeg_size JPEG data size
 * @param capture_time_us Output: capture timestamp in microseconds
 * @param frame_num Output: frame sequence number
 * @return true if timestamp was found and extracted, false otherwise
 */
bool jpeg_extract_timestamp(const uint8_t *jpeg_data, size_t jpeg_size,
                            int64_t *capture_time_us, uint32_t *frame_num);

/**
 * @brief Get raw JPEG data size without timestamp header
 * 
 * @param jpeg_with_ts JPEG data with embedded timestamp
 * @param size_with_ts Size of JPEG with timestamp
 * @param raw_jpeg_start Output: pointer to reconstruct raw JPEG
 * @return Size of raw JPEG data (without timestamp header)
 */
size_t jpeg_get_raw_data(const uint8_t *jpeg_with_ts, size_t size_with_ts,
                         const uint8_t **raw_jpeg_start);

/**
 * @brief Set JPEG encoding quality at runtime.
 *
 * Takes effect on the next encoded frame.
 *
 * @param quality  Quality value 1-100 (higher = better quality, larger file).
 *                 Values outside this range are clamped.
 */
void app_video_stream_set_jpeg_quality(int quality);

/**
 * @brief Get the current JPEG encoding quality.
 *
 * @return Current quality value (1-100).
 */
int app_video_stream_get_jpeg_quality(void);

/**
 * @brief Set the number of frames to skip after each trigger before recording.
 *
 * 0 = record from the very first frame (default, lowest latency).
 * Increase if the first N frames are overexposed or noisy due to AE settling.
 * Takes effect on the next trigger event.
 *
 * @param n  Number of frames to skip (0–255 practical range).
 */
void app_video_stream_set_skip_frames(uint32_t n);

/**
 * @brief Get the current skip-frames-on-trigger setting.
 *
 * @return Number of frames skipped after each trigger.
 */
uint32_t app_video_stream_get_skip_frames(void);

/**
 * @brief Set trigger source mode.
 *
 * GPIO mode uses external hardware trigger.
 * UDP mode allows software trigger start/stop commands.
 */
void app_video_stream_set_trigger_mode(app_trigger_mode_t mode);

/**
 * @brief Get current trigger source mode.
 */
app_trigger_mode_t app_video_stream_get_trigger_mode(void);

/**
 * @brief Start UDP software trigger capture.
 *
 * Valid only when trigger mode is APP_TRIGGER_MODE_UDP.
 */
esp_err_t app_video_stream_udp_trigger_start(void);

/**
 * @brief Stop UDP software trigger capture.
 *
 * Valid only when trigger mode is APP_TRIGGER_MODE_UDP.
 */
esp_err_t app_video_stream_udp_trigger_stop(void);

/**
 * @brief Read current trigger statistics.
 */
void app_video_stream_get_trigger_stats(app_trigger_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif // APP_VIDEO_STREAM_H
