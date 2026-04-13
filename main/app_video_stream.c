/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * HTTP Video Stream Server for ESP32-P4
 * Supports both MJPEG and H264 streaming over HTTP
 * - MJPEG: Works directly in browsers with <img> tag
 * - H264: More efficient, requires MSE/WebRTC for browser playback
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/jpeg_encode.h"
#include "linux/videodev2.h"
#include "esp_video_device.h"
#include "app_video_stream.h"
#include "app_video.h"
#include "app_trigger.h"

static const char *TAG = "video_stream";

#define PART_BOUNDARY "123456789000000000000987654321"
#define JPEG_ENC_QUALITY_DEFAULT 60  // Default quality (50-70 streaming, 75-90 high quality)
#define H264_BUFFER_COUNT 2

/* Runtime-configurable JPEG quality (1-100). Changed via app_video_stream_set_jpeg_quality(). */
static volatile int g_jpeg_quality = JPEG_ENC_QUALITY_DEFAULT;

// Manual exposure configuration (set to 0 to use auto exposure)
// Exposure time in microseconds: 5000=5ms (normal indoor), 10000=10ms (bright), 20000=20ms (dim)
// 720p@25fps frame period = 40ms, so any value up to ~38ms is safe without locking fps.
#define MANUAL_EXPOSURE_US  0      // 0 = use IPA auto exposure (sensor hardware AEC/AGC)

// JPEG timestamp embedding marker (APP4 segment)
#define JPEG_TS_MARKER_ID "TSMP"  // Timestamp marker identifier
#define JPEG_TS_APP_SEGMENT 0xE4   // APP4 segment
#define JPEG_TS_HEADER_SIZE 20     // 2(marker) + 2(length) + 4(id) + 8(timestamp) + 4(frame_num)

// Frame decimation: keep 1 out of every N raw frames
// 720p@25fps native (VTS configured in sensor registers for 25fps)
// Set to 1 to keep all frames (no decimation)
#define FRAME_DECIMATION 1   // No decimation: sensor outputs 25fps directly

// Frame ring buffer configuration
#define FRAME_RING_BUFFER_SIZE 10  // Cache up to 10 frames (400ms at 25fps)

// Ring buffer frame structure
typedef struct {
    uint8_t *data;
    size_t size;
    int64_t timestamp_us;
    uint32_t frame_number;
    bool valid;
} cached_frame_t;

// Ring buffer for frame caching
static cached_frame_t frame_ring_buffer[FRAME_RING_BUFFER_SIZE];
static volatile int ring_write_index = 0;
static volatile int ring_read_index = 0;
static volatile int ring_frame_count = 0;
static size_t max_frame_size = 0;

// Current frame trigger info (for overlay/metadata)
static trigger_frame_info_t g_current_frame_info = {0};

// Stream mode selection
typedef enum {
    STREAM_MODE_MJPEG,
    STREAM_MODE_H264
} stream_mode_t;

static stream_mode_t current_stream_mode = STREAM_MODE_MJPEG;

static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static httpd_handle_t stream_httpd = NULL;
static int g_video_fd = -1;
static volatile bool stream_active = false;
static SemaphoreHandle_t stream_mutex = NULL;

// Hardware JPEG encoder (for MJPEG mode)
static jpeg_encoder_handle_t jpeg_encoder = NULL;
static jpeg_encode_cfg_t jpeg_enc_config;
static uint8_t *jpeg_output_buffer = NULL;
static size_t jpeg_output_buffer_size = 0;

// H264 encoder device (for H264 mode)
static int h264_fd = -1;

static size_t encoded_frame_size = 0;
static bool capture_active = false;  // Whether capture/encoding is running (independent of HTTP)

// Sent frame counter (incremented by stream_handler, read by video_frame_callback for FPS log)
static volatile uint32_t sent_frame_count = 0;

// Snapshot buffer (always holds latest frame)
static uint8_t *snapshot_buffer = NULL;
static size_t snapshot_size = 0;
static int64_t snapshot_timestamp_us = 0;
static volatile bool snapshot_ready = false;

// Temporary buffer for timestamp embedding
static uint8_t *jpeg_ts_buffer = NULL;
static size_t jpeg_ts_buffer_size = 0;

// FPS statistics
static int64_t last_frame_time = 0;
static int frame_count = 0;
static volatile bool camera_stream_started = false;  // Track if camera capture is running

// Raw frame counter (for debugging - counts all frames from camera)
static uint32_t raw_frame_counter = 0;

// Last frame timestamp for interval calculation
static int64_t last_capture_time_us = 0;

// Client ready flag: trigger only starts capture when a stream client is connected and ready
static volatile bool client_ready = false;
static volatile bool waiting_for_trigger = false;

// Camera preheat flag (kept for compatibility, not used with manual exposure)
static volatile bool camera_preheating = false;

// AE warmup: skip first N frames after trigger to let AE stabilize
// Number of frames to skip after trigger before recording (runtime configurable).
// 0 = record from the very first frame (lowest latency).
// Increase if the first few frames are overexposed or noisy.
static volatile uint32_t g_skip_frames_on_trigger = 0;  // Set via UDP / runtime API
static volatile uint32_t frames_to_skip = 0;            // Decremented in callback

// Trigger mode and UDP trigger runtime state/statistics
static volatile app_trigger_mode_t g_trigger_mode = APP_TRIGGER_MODE_GPIO;
static volatile bool g_udp_trigger_active = false;
static volatile int64_t g_udp_trigger_start_time = 0;
static volatile uint32_t g_udp_frame_counter = 0;
static volatile uint32_t g_udp_dropped_frames_est = 0;
static volatile int64_t g_udp_first_frame_latency_us = -1;
static volatile int64_t g_udp_last_capture_time_us = 0;
static volatile int64_t g_udp_last_frame_interval_us = 0;
static volatile int64_t g_udp_interval_sum_us = 0;
static volatile uint32_t g_udp_interval_samples = 0;
static volatile bool g_combined_trigger_active = false;

#define EXPECTED_FRAME_INTERVAL_US 40000  // 25fps nominal interval

// Trigger event enum (must be declared before notify_trigger_state_if_changed)
typedef enum {
    TRIGGER_EVENT_START,
    TRIGGER_EVENT_STOP
} trigger_event_t;

// Event queue for deferred trigger handling (to avoid stack overflow in callback)
static QueueHandle_t trigger_event_queue = NULL;
static TaskHandle_t trigger_handler_task = NULL;

// Semaphore to notify stream sender when a new frame is in the ring buffer
static SemaphoreHandle_t frame_ready_sem = NULL;

static bool is_current_trigger_active(void)
{
    return g_udp_trigger_active || app_trigger_is_active();
}

static void notify_trigger_state_if_changed(bool new_active)
{
    if (new_active == g_combined_trigger_active) {
        return;
    }
    g_combined_trigger_active = new_active;

    if (trigger_event_queue == NULL) {
        return;
    }

    trigger_event_t event = new_active ? TRIGGER_EVENT_START : TRIGGER_EVENT_STOP;
    if (xQueueSend(trigger_event_queue, &event, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Trigger event queue full!");
    }
}

// Startup latency calibration results
#define CALIBRATION_SAMPLES 3
static int64_t calibrated_startup_delay_us = 200000;  // Default 200ms, will be calibrated
static int64_t startup_delay_samples[CALIBRATION_SAMPLES] = {0};
static int64_t startup_delay_jitter_us = 0;  // Standard deviation
static bool calibration_done = false;

/**
 * @brief Task to handle trigger events (runs heavy V4L2 operations)
 * This task has sufficient stack space for STREAMON/STREAMOFF operations
 * 
 * Important: Camera only starts when BOTH conditions are met:
 *   1. External trigger signal (GPIO rising edge)
 *   2. Stream client is connected and ready (client_ready = true)
 */
static void trigger_event_handler_task(void *arg)
{
    trigger_event_t event;
    
    while (1) {
        if (xQueueReceive(trigger_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            if (event == TRIGGER_EVENT_START) {
                ESP_LOGI(TAG, ">>> Processing TRIGGER_START event (preheat=%d)", camera_preheating);
                
                // Clear ring buffer and reset counters for fresh recording
                ring_write_index = 0;
                ring_read_index = 0;
                ring_frame_count = 0;
                raw_frame_counter = 0;
                last_capture_time_us = 0;  // Reset frame interval tracking
                frames_to_skip = g_skip_frames_on_trigger;  // Skip first N frames (configurable)

                if (g_trigger_mode == APP_TRIGGER_MODE_UDP) {
                    g_udp_frame_counter = 0;
                    g_udp_dropped_frames_est = 0;
                    g_udp_first_frame_latency_us = -1;
                    g_udp_last_capture_time_us = 0;
                    g_udp_last_frame_interval_us = 0;
                    g_udp_interval_sum_us = 0;
                    g_udp_interval_samples = 0;
                }
                
                ESP_LOGI(TAG, "Will skip first %lu frames after trigger", (unsigned long)g_skip_frames_on_trigger);
                
                // Notify waiting client that trigger has arrived
                waiting_for_trigger = false;
                
                // End preheat phase - start recording frames
                // AE should already be converged from preheat
                if (camera_preheating) {
                    camera_preheating = false;
                    ESP_LOGI(TAG, "Preheat complete, AE stable, now recording!");
                }
                
                // Start camera stream (if not already preheating)
                if (g_video_fd >= 0 && !camera_stream_started) {
                    int64_t start_time = esp_timer_get_time();
                    esp_err_t ret = app_video_stream_on(g_video_fd);
                    int64_t elapsed = esp_timer_get_time() - start_time;
                    if (ret == ESP_OK) {
                        camera_stream_started = true;
                        ESP_LOGI(TAG, "Camera STREAM ON (took %lld us = %.2f ms)", 
                                 elapsed, elapsed / 1000.0f);
                    } else {
                        ESP_LOGE(TAG, "Failed to start camera stream");
                    }
                }
            } else if (event == TRIGGER_EVENT_STOP) {
                //ESP_LOGI(TAG, ">>> Processing TRIGGER_STOP event (cached: %d frames)", ring_frame_count);
                
                // Stop camera stream
                if (g_video_fd >= 0 && camera_stream_started) {
                    camera_stream_started = false;  // Always clear — prevents double-stop race
                    esp_err_t ret = app_video_stream_off(g_video_fd);
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG, "Camera STREAM OFF");
                    } else {
                        ESP_LOGE(TAG, "Failed to stop camera stream");
                    }
                }
                
                // NOTE: Do NOT call app_trigger_reset() here!
                // The ISR already sets up new trigger state on next rising edge.
                // Calling reset here would clear g_trigger_start_time that was
                // already set by a new rising edge ISR (race condition).

                // FIX Bug 2: Signal stream loop to exit so the httpd socket is released.
                // Without this, stream_handler blocks on frame_ready_sem indefinitely
                // after camera stops, holding the socket slot until the client disconnects.
                // Over multiple trigger cycles this exhausts the httpd socket pool.
                stream_active = false;
                if (frame_ready_sem != NULL) {
                    xSemaphoreGive(frame_ready_sem);  // Wake up the sleeping stream loop
                }
                ESP_LOGI(TAG, "Stream loop signalled to exit (socket will be released)");
            }
        }
    }
}

/**
 * @brief Lightweight callback for trigger state changes
 * Only posts events to queue - heavy work done in handler task
 * This runs in trigger_cb task with limited stack (2KB)
 */
static void trigger_state_callback(bool active)
{
    bool combined = g_udp_trigger_active || active;
    notify_trigger_state_if_changed(combined);
}

/**
 * @brief Embed timestamp into JPEG data using APP4 segment
 * 
 * JPEG structure: FFD8 [APP segments...] [image data] FFD9
 * We insert custom APP4 segment after SOI: FF E4 [length] [data]
 * 
 * @param jpeg_in Original JPEG data
 * @param jpeg_in_size Original JPEG size
 * @param jpeg_out Output buffer (must be at least jpeg_in_size + JPEG_TS_HEADER_SIZE)
 * @param capture_time_us Capture timestamp in microseconds (relative to trigger)
 * @param frame_num Frame sequence number
 * @return Output JPEG size with embedded timestamp
 */
static size_t jpeg_embed_timestamp(const uint8_t *jpeg_in, size_t jpeg_in_size,
                                    uint8_t *jpeg_out, int64_t capture_time_us, uint32_t frame_num)
{
    // Check JPEG SOI marker (FFD8)
    if (jpeg_in_size < 2 || jpeg_in[0] != 0xFF || jpeg_in[1] != 0xD8) {
        // Not a valid JPEG, copy as-is
        memcpy(jpeg_out, jpeg_in, jpeg_in_size);
        return jpeg_in_size;
    }
    
    // APP4 segment structure:
    // Marker: FF E4
    // Length: 2 bytes (big-endian, includes length field itself)
    // Identifier: "TSMP" (4 bytes)
    // Timestamp: 8 bytes (big-endian, microseconds)
    // Frame number: 4 bytes (big-endian)
    uint16_t app_length = 2 + 4 + 8 + 4;  // length field + id + timestamp + frame_num
    
    // Copy SOI
    jpeg_out[0] = 0xFF;
    jpeg_out[1] = 0xD8;
    
    // Insert APP4 segment
    jpeg_out[2] = 0xFF;
    jpeg_out[3] = JPEG_TS_APP_SEGMENT;  // APP4
    jpeg_out[4] = (app_length >> 8) & 0xFF;  // Length high byte
    jpeg_out[5] = app_length & 0xFF;         // Length low byte
    
    // Identifier
    memcpy(&jpeg_out[6], JPEG_TS_MARKER_ID, 4);
    
    // Timestamp (big-endian for cross-platform compatibility)
    jpeg_out[10] = (capture_time_us >> 56) & 0xFF;
    jpeg_out[11] = (capture_time_us >> 48) & 0xFF;
    jpeg_out[12] = (capture_time_us >> 40) & 0xFF;
    jpeg_out[13] = (capture_time_us >> 32) & 0xFF;
    jpeg_out[14] = (capture_time_us >> 24) & 0xFF;
    jpeg_out[15] = (capture_time_us >> 16) & 0xFF;
    jpeg_out[16] = (capture_time_us >> 8) & 0xFF;
    jpeg_out[17] = capture_time_us & 0xFF;
    
    // Frame number (big-endian)
    jpeg_out[18] = (frame_num >> 24) & 0xFF;
    jpeg_out[19] = (frame_num >> 16) & 0xFF;
    jpeg_out[20] = (frame_num >> 8) & 0xFF;
    jpeg_out[21] = frame_num & 0xFF;
    
    // Copy remaining JPEG data (skip original SOI)
    memcpy(&jpeg_out[22], &jpeg_in[2], jpeg_in_size - 2);
    
    return jpeg_in_size + JPEG_TS_HEADER_SIZE;
}

/**
 * @brief Extract timestamp from JPEG data with embedded timestamp
 * 
 * @param jpeg_data JPEG data with embedded timestamp
 * @param jpeg_size JPEG data size
 * @param capture_time_us Output: capture timestamp in microseconds
 * @param frame_num Output: frame sequence number
 * @return true if timestamp was found and extracted, false otherwise
 */
bool jpeg_extract_timestamp(const uint8_t *jpeg_data, size_t jpeg_size,
                            int64_t *capture_time_us, uint32_t *frame_num)
{
    if (jpeg_size < 22) return false;
    
    // Check SOI
    if (jpeg_data[0] != 0xFF || jpeg_data[1] != 0xD8) return false;
    
    // Check APP4 marker
    if (jpeg_data[2] != 0xFF || jpeg_data[3] != JPEG_TS_APP_SEGMENT) return false;
    
    // Check identifier
    if (memcmp(&jpeg_data[6], JPEG_TS_MARKER_ID, 4) != 0) return false;
    
    // Extract timestamp (big-endian)
    *capture_time_us = ((int64_t)jpeg_data[10] << 56) |
                       ((int64_t)jpeg_data[11] << 48) |
                       ((int64_t)jpeg_data[12] << 40) |
                       ((int64_t)jpeg_data[13] << 32) |
                       ((int64_t)jpeg_data[14] << 24) |
                       ((int64_t)jpeg_data[15] << 16) |
                       ((int64_t)jpeg_data[16] << 8) |
                       (int64_t)jpeg_data[17];
    
    // Extract frame number (big-endian)
    *frame_num = ((uint32_t)jpeg_data[18] << 24) |
                 ((uint32_t)jpeg_data[19] << 16) |
                 ((uint32_t)jpeg_data[20] << 8) |
                 (uint32_t)jpeg_data[21];
    
    return true;
}

/**
 * @brief Get raw JPEG data without timestamp header
 * 
 * @param jpeg_with_ts JPEG data with embedded timestamp
 * @param size_with_ts Size of JPEG with timestamp
 * @param raw_jpeg_start Output: pointer to raw JPEG start (SOI)
 * @return Size of raw JPEG data
 */
size_t jpeg_get_raw_data(const uint8_t *jpeg_with_ts, size_t size_with_ts,
                         const uint8_t **raw_jpeg_start)
{
    if (size_with_ts < 22) {
        *raw_jpeg_start = jpeg_with_ts;
        return size_with_ts;
    }
    
    // Check if this JPEG has our timestamp header
    if (jpeg_with_ts[2] == 0xFF && jpeg_with_ts[3] == JPEG_TS_APP_SEGMENT &&
        memcmp(&jpeg_with_ts[6], JPEG_TS_MARKER_ID, 4) == 0) {
        // Return pointer to data after our APP4 segment, but include SOI
        // The raw JPEG starts at offset 22 (after our header), but we need to
        // reconstruct SOI + rest of data
        *raw_jpeg_start = jpeg_with_ts;  // Caller should handle reconstruction
        return size_with_ts - JPEG_TS_HEADER_SIZE;
    }
    
    *raw_jpeg_start = jpeg_with_ts;
    return size_with_ts;
}

/**
 * @brief Add an encoded frame to the ring buffer
 */
static void ring_buffer_add_frame(const uint8_t *data, size_t size, int64_t timestamp, uint32_t frame_num)
{
    if (size > max_frame_size || data == NULL) {
        return;
    }
    
    cached_frame_t *frame = &frame_ring_buffer[ring_write_index];
    
    // Copy frame data
    memcpy(frame->data, data, size);
    frame->size = size;
    frame->timestamp_us = timestamp;
    frame->frame_number = frame_num;
    frame->valid = true;
    
    // Update write index
    ring_write_index = (ring_write_index + 1) % FRAME_RING_BUFFER_SIZE;
    
    // Update frame count
    if (ring_frame_count < FRAME_RING_BUFFER_SIZE) {
        ring_frame_count++;
    } else {
        // Buffer is full, advance read index (discard oldest frame)
        ring_read_index = (ring_read_index + 1) % FRAME_RING_BUFFER_SIZE;
        ESP_LOGW(TAG, "Ring buffer full! Dropping oldest frame to make room for Frame %lu", (unsigned long)frame_num);
    }
}

/**
 * @brief Get a frame from the ring buffer (advances read pointer)
 * @return Pointer to cached frame, or NULL if buffer empty
 */
static cached_frame_t* ring_buffer_get_frame(void)
{
    if (ring_frame_count == 0) {
        return NULL;
    }
    
    cached_frame_t *frame = &frame_ring_buffer[ring_read_index];
    if (!frame->valid) {
        return NULL;
    }
    
    ring_read_index = (ring_read_index + 1) % FRAME_RING_BUFFER_SIZE;
    ring_frame_count--;
    frame->valid = false;
    
    return frame;
}

/**
 * @brief Get cached frame count
 */
static int ring_buffer_count(void)
{
    return ring_frame_count;
}

/**
 * @brief Calibration state machine
 */
typedef enum {
    CALIB_STATE_IDLE,
    CALIB_STATE_WAITING_FIRST_FRAME,
} calib_state_t;

static volatile calib_state_t calib_state = CALIB_STATE_IDLE;
static volatile int64_t calib_streamon_time = 0;
static volatile int64_t calib_first_frame_time = 0;      // 回调执行时间（帧完成）
static volatile int64_t calib_frame_start_time = 0;      // V4L2时间戳（帧开始）

/**
 * @brief Simple callback to measure first frame arrival during calibration
 * This temporarily replaces the normal frame callback during self-test
 */
static void calibration_frame_callback(uint8_t *camera_buf, uint8_t camera_buf_index, 
                                        uint32_t camera_buf_hes, uint32_t camera_buf_ves, 
                                        size_t camera_buf_len, void *user_data)
{
    (void)camera_buf;
    (void)camera_buf_index;
    (void)camera_buf_hes;
    (void)camera_buf_ves;
    (void)camera_buf_len;
    (void)user_data;
    
    // Always log to confirm callback is being called
    ESP_LOGI(TAG, "  [CALIB] Callback! state=%d", calib_state);
    
    if (calib_state == CALIB_STATE_WAITING_FIRST_FRAME) {
        calib_first_frame_time = esp_timer_get_time();         // 帧接收完成时间
        calib_frame_start_time = app_video_get_frame_timestamp_us();  // 帧开始接收时间 (V4L2)
        calib_state = CALIB_STATE_IDLE;
        ESP_LOGI(TAG, "  [CALIB] First frame - V4L2_ts=%lld, esp_ts=%lld", 
                 calib_frame_start_time, calib_first_frame_time);
    }
}

/**
 * @brief Perform startup latency self-test
 * Measures the delay from STREAMON to first frame arrival
 * 
 * @param video_fd File descriptor for the video device
 * @return ESP_OK on success
 */
static esp_err_t perform_startup_calibration(int video_fd)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Starting camera startup latency self-test...");
    ESP_LOGI(TAG, "  Samples: %d", CALIBRATION_SAMPLES);
    ESP_LOGI(TAG, "========================================");
    
    int64_t total_delay = 0;
    int valid_samples = 0;
    
    // Temporarily register calibration callback
    app_video_register_frame_operation_cb(calibration_frame_callback);
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        ESP_LOGI(TAG, "[%d/%d] Measuring startup delay...", i + 1, CALIBRATION_SAMPLES);
        
        // Reset calibration state
        calib_state = CALIB_STATE_WAITING_FIRST_FRAME;
        calib_first_frame_time = 0;
        
        // Record STREAMON time
        calib_streamon_time = esp_timer_get_time();
        
        // Start camera stream
        esp_err_t ret = app_video_stream_on(video_fd);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "  Failed to start stream");
            startup_delay_samples[i] = 0;
            continue;
        }
        
        ESP_LOGI(TAG, "  STREAMON done, waiting for first frame...");
        
        // Wait for first frame (max 1000ms timeout - MIPI init takes ~200ms)
        int timeout_ms = 1000;
        while (calib_state == CALIB_STATE_WAITING_FIRST_FRAME && timeout_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            timeout_ms -= 10;
        }
        
        // Calculate delay - check if we got a frame (calib_state changed to IDLE)
        if (calib_first_frame_time > 0) {
            int64_t delay_to_complete = calib_first_frame_time - calib_streamon_time;  // STREAMON到帧完成
            
            // V4L2 timestamp may not be available on all drivers
            if (calib_frame_start_time > 0) {
                int64_t delay_to_start = calib_frame_start_time - calib_streamon_time;     // STREAMON到帧开始
                int64_t transfer_time = calib_first_frame_time - calib_frame_start_time;   // 帧传输时间
                
                startup_delay_samples[i] = delay_to_start;  // 使用帧开始时间
                total_delay += delay_to_start;
                valid_samples++;
                
                ESP_LOGI(TAG, "  Sample %d:", i + 1);
                ESP_LOGI(TAG, "    STREAMON → Frame Start:    %lld us (%.2f ms)", 
                         delay_to_start, delay_to_start / 1000.0f);
                ESP_LOGI(TAG, "    STREAMON → Frame Complete: %lld us (%.2f ms)", 
                         delay_to_complete, delay_to_complete / 1000.0f);
                ESP_LOGI(TAG, "    Frame Transfer Time:       %lld us (%.2f ms)", 
                         transfer_time, transfer_time / 1000.0f);
            } else {
                // V4L2 timestamp not available, use callback time only
                startup_delay_samples[i] = delay_to_complete;
                total_delay += delay_to_complete;
                valid_samples++;
                
                ESP_LOGI(TAG, "  Sample %d: %lld us (%.2f ms) [V4L2 timestamp N/A]", 
                         i + 1, delay_to_complete, delay_to_complete / 1000.0f);
            }
        } else {
            ESP_LOGW(TAG, "  Sample %d: TIMEOUT (state=%d, frame_time=%lld)", 
                     i + 1, calib_state, calib_first_frame_time);
            startup_delay_samples[i] = 0;
        }
        
        // Stop camera stream for next iteration
        app_video_stream_off(video_fd);
        
        // Wait a bit before next sample
        vTaskDelay(pdMS_TO_TICKS(110));
    }
    
    // Calculate average and jitter
    if (valid_samples > 0) {
        calibrated_startup_delay_us = total_delay / valid_samples;
        
        // Calculate standard deviation (jitter)
        int64_t variance_sum = 0;
        for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
            if (startup_delay_samples[i] > 0) {
                int64_t diff = startup_delay_samples[i] - calibrated_startup_delay_us;
                variance_sum += diff * diff;
            }
        }
        startup_delay_jitter_us = (int64_t)sqrtf((float)variance_sum / valid_samples);
        
        calibration_done = true;
        
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "Calibration Complete:");
        ESP_LOGI(TAG, "  Average delay: %lld us (%.2f ms)", 
                 calibrated_startup_delay_us, calibrated_startup_delay_us / 1000.0f);
        ESP_LOGI(TAG, "  Jitter (std):  %lld us (±%.2f ms)", 
                 startup_delay_jitter_us, startup_delay_jitter_us / 1000.0f);
        ESP_LOGI(TAG, "  Valid samples: %d/%d", valid_samples, CALIBRATION_SAMPLES);
        ESP_LOGI(TAG, "========================================");
    } else {
        ESP_LOGE(TAG, "Calibration FAILED - no valid samples!");
        calibration_done = false;
    }
    
    // Restore original callback (will be set later by normal init)
    app_video_register_frame_operation_cb(NULL);
    
    return valid_samples > 0 ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Get the calibrated startup delay
 * @return Calibrated delay in microseconds
 */
int64_t app_video_stream_get_calibrated_delay_us(void)
{
    return calibrated_startup_delay_us;
}

/**
 * @brief Get the startup delay jitter (standard deviation)
 * @return Jitter in microseconds
 */
int64_t app_video_stream_get_delay_jitter_us(void)
{
    return startup_delay_jitter_us;
}

/**
 * @brief Check if calibration has been performed
 * @return true if calibration is done
 */
bool app_video_stream_is_calibrated(void)
{
    return calibration_done;
}

/**
 * @brief Callback function to receive video frames, encode to JPEG/H264, and store in ring buffer
 * 
 * Camera runs at 50fps, we only process EVEN frames to get 25fps output.
 * Frame decimation: raw_frame_counter % 2 == 0 → process
 * 
 * Timing analysis for accurate capture timestamp:
 *   - Callback time (esp_timer): When frame transfer is COMPLETE
 *   - Actual capture time ≈ Callback time - transfer_time (~41ms for 1080P)
 */
static void video_frame_callback(uint8_t *camera_buf, uint8_t camera_buf_index, 
                                  uint32_t camera_buf_hes, uint32_t camera_buf_ves, 
                                  size_t camera_buf_len, void *user_data)
{
    // Record frame arrival time immediately at callback entry (frame transfer COMPLETE)
    int64_t frame_complete_time = esp_timer_get_time();
    
    // Count ALL raw frames from camera (for debugging and decimation)
    uint32_t this_raw_frame = raw_frame_counter++;
    ESP_LOGD(TAG, ">>> Raw Frame Arrived: #%lu", (unsigned long)this_raw_frame);

    // Frame decimation: only process every Nth frame
    if (FRAME_DECIMATION > 1 && (this_raw_frame % FRAME_DECIMATION) != 0) {
        return;
    }
    
    // Check if encoding is possible
    if (!capture_active) 
    {
        return;
    }

    // Get trigger state and frame info (GPIO OR UDP)
    trigger_frame_info_t frame_info = {0};
    bool gpio_active = (app_trigger_get_frame_info(&frame_info) == ESP_OK);
    bool udp_active = (g_udp_trigger_active && g_udp_trigger_start_time > 0);
    bool is_triggered = gpio_active || udp_active;

    if (!gpio_active && udp_active) {
        frame_info.is_triggered = true;
        frame_info.timestamp_us = frame_complete_time - g_udp_trigger_start_time;
    }
    
    ESP_LOGD(TAG, "  triggered=%d, frame_num=%lu, skip_remain=%lu",
             is_triggered, (unsigned long)frame_info.frame_number, (unsigned long)frames_to_skip);
    
    // Only process frames when triggered (trigger GPIO must be HIGH)
    if (!is_triggered) 
    {
        return;
    }
    
    // Skip first N frames for AE warmup (let auto-exposure stabilize)
    if (frames_to_skip > 0) 
    {
        frames_to_skip--;
        if (frames_to_skip == 0) {
            ESP_LOGI(TAG, "AE warmup complete, now recording frames");
        }
        return;  // Skip this frame
    }
    
    // Calculate accurate timestamps relative to trigger
    int64_t trigger_start = gpio_active ? app_trigger_get_start_time() : g_udp_trigger_start_time;
    frame_info.callback_time_us = frame_complete_time - trigger_start;

    // Use V4L2 frame start timestamp for accurate capture time.
    // app_video_get_frame_timestamp_us() returns the time MIPI transfer started (frame exposure done).
    // Fall back to callback_time - constant if V4L2 timestamp is unavailable.
    int64_t v4l2_frame_ts = app_video_get_frame_timestamp_us();
    if (v4l2_frame_ts > trigger_start) {
        // V4L2 timestamp is absolute esp_timer time; convert to relative-to-trigger
        frame_info.capture_time_us  = v4l2_frame_ts - trigger_start;
        frame_info.transfer_time_us = frame_complete_time - v4l2_frame_ts;
    } else {
        // Fallback: subtract constant transfer time
        frame_info.transfer_time_us = FRAME_TRANSFER_TIME_US;
        frame_info.capture_time_us  = frame_info.callback_time_us - FRAME_TRANSFER_TIME_US;
    }
    if (frame_info.capture_time_us < 0) 
    {
        frame_info.capture_time_us = 0;  // Clamp to 0 if negative
    }
    
    // Store frame info for later use
    g_current_frame_info = frame_info;
    
    last_capture_time_us = frame_info.capture_time_us;
    
    // OPTIMIZATION: Encode OUTSIDE mutex to minimize critical section
    // Only lock when writing to ring buffer (fast memcpy)
    uint32_t encoded_size = 0;
    esp_err_t ret = ESP_FAIL;
    size_t ts_jpeg_size = 0;
    uint32_t frame_num = 0;
    int64_t capture_time = 0;

    if (current_stream_mode == STREAM_MODE_MJPEG && jpeg_encoder != NULL) 
    {
        // MJPEG encoding (hardware, ~5-10ms) - NO LOCK NEEDED
        ret = jpeg_encoder_process(jpeg_encoder, &jpeg_enc_config, 
                                    camera_buf, camera_buf_len,
                                    jpeg_output_buffer, jpeg_output_buffer_size,
                                    &encoded_size);
        
        if (ret == ESP_OK && encoded_size > 0) 
        {
            // Allocate frame number
            if (gpio_active) {
                frame_num = app_trigger_alloc_frame_number();
            } else {
                frame_num = ++g_udp_frame_counter;

                if (g_udp_first_frame_latency_us < 0) {
                    g_udp_first_frame_latency_us = frame_info.capture_time_us;
                }

                if (g_udp_last_capture_time_us > 0 && frame_info.capture_time_us > g_udp_last_capture_time_us) {
                    int64_t interval = frame_info.capture_time_us - g_udp_last_capture_time_us;
                    g_udp_last_frame_interval_us = interval;
                    g_udp_interval_sum_us += interval;
                    g_udp_interval_samples++;

                    if (interval > (EXPECTED_FRAME_INTERVAL_US * 3) / 2) {
                        uint32_t dropped = (uint32_t)(interval / EXPECTED_FRAME_INTERVAL_US);
                        if (dropped > 1) {
                            g_udp_dropped_frames_est += (dropped - 1);
                        }
                    }
                }
                g_udp_last_capture_time_us = frame_info.capture_time_us;
            }

            capture_time = g_current_frame_info.capture_time_us;
            g_current_frame_info.frame_number = frame_num;

            ESP_LOGD(TAG, "Frame Code: %lu", (unsigned long)frame_num);
            
            // Embed timestamp (fast, ~100us) - NO LOCK NEEDED
            ts_jpeg_size = jpeg_embed_timestamp(
                jpeg_output_buffer, encoded_size,
                jpeg_ts_buffer, capture_time, frame_num);
            
            encoded_frame_size = ts_jpeg_size;
        }
    }

    // CRITICAL SECTION: Only lock for ring buffer write (~500us memcpy)
    if (ret == ESP_OK && ts_jpeg_size > 0 && xSemaphoreTake(stream_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        int64_t now = esp_timer_get_time();
        ring_buffer_add_frame(jpeg_ts_buffer, ts_jpeg_size, now, frame_num);

        // Update snapshot buffer
        if (snapshot_buffer != NULL)
        {
            memcpy(snapshot_buffer, jpeg_ts_buffer, ts_jpeg_size);
            snapshot_size = ts_jpeg_size;
            snapshot_timestamp_us = capture_time;
            snapshot_ready = true;
        }

        xSemaphoreGive(stream_mutex);

        // Notify stream sender task that a new frame is ready
        if (frame_ready_sem != NULL) {
            xSemaphoreGive(frame_ready_sem);
        }
    }

    if (ret == ESP_OK) 
    {
        // FPS calculation (every 25 frames = 1 second at 25fps)
        frame_count++;
        if (frame_count >= 25) 
        {
            int64_t calc_now = esp_timer_get_time();
            float fps = 25.0f * 1000000.0f / (calc_now - last_frame_time);
            uint32_t sent = sent_frame_count;
            sent_frame_count = 0;  // Reset counter
            ESP_LOGI(TAG, "Encode FPS: %.1f, JPEG size: %lu bytes, cached: %d, sent: %lu/25",
                        fps, (unsigned long)encoded_frame_size, ring_buffer_count(), (unsigned long)sent);
            last_frame_time = calc_now;
            frame_count = 0;
        }
    }
}

/**
 * @brief HTTP handler for MJPEG video stream
 * 
 * Workflow for synchronized trigger capture:
 *   1. Client connects to /stream
 *   2. Server sets client_ready = true and waits for trigger
 *   3. External GPIO trigger arrives → camera starts, frames captured
 *   4. Client receives frames starting from Frame 1
 * 
 * This ensures no frames are lost due to client connection timing.
 */
static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t res = ESP_OK;
    char part_buf[128];
    int cached_frames_sent = 0;

    ESP_LOGI(TAG, "Stream client connecting...");

    // Set TCP_NODELAY to disable Nagle's algorithm for lower latency
    int sockfd = httpd_req_to_sockfd(req);
    if (sockfd >= 0) {
        int nodelay = 1;
        setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    }

    // First set up MJPEG response headers and send them immediately
    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "25");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");

    // Send initial boundary immediately to establish HTTP response
    res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    if (res != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send initial boundary");
        return res;
    }
    
    ESP_LOGI(TAG, "Stream client connected, response headers sent");

    // Mark client as ready - trigger handler will now respond to GPIO signals
    client_ready = true;
    waiting_for_trigger = true;
    
    // Check if camera is already running (Trigger arrived BEFORE clients connected)
    if (camera_stream_started) {
        ESP_LOGI(TAG, "Camera already running! Preserving %d cached frames from early trigger", ring_buffer_count());
        // Mark as triggered so we don't think we are waiting
        waiting_for_trigger = false;
    } else {
        // Clear any stale frames from previous sessions ONLY if camera is stopped
        ring_write_index = 0;
        ring_read_index = 0;
        ring_frame_count = 0;
        ESP_LOGI(TAG, "Camera stopped. Ring buffer cleared for fresh start");
    }
    
    ESP_LOGI(TAG, "Waiting for external trigger signal on GPIO %d...", TRIGGER_GPIO_PIN);
    
    // Wait for NEW trigger and frames with timeout (60 seconds)
    // Use frame_ready_sem for near-zero latency wakeup instead of polling
    int wait_count = 0;
    const int max_wait_ms = 60000;
    while (ring_buffer_count() == 0 && wait_count < max_wait_ms) {
        if (frame_ready_sem != NULL) {
            // Block on semaphore — wakes within ~1ms of first frame arriving
            if (xSemaphoreTake(frame_ready_sem, pdMS_TO_TICKS(1000)) == pdTRUE) {
                // Semaphore taken — check if frame is actually ready
                if (ring_buffer_count() > 0) break;
            }
            wait_count += 1000;
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
            wait_count += 5;
        }
        
        // Send a keep-alive comment every 5 seconds to prevent timeout
        if (wait_count % 5000 == 0 && wait_count > 0) {
            ESP_LOGI(TAG, "Still waiting for trigger... (%d s)", wait_count / 1000);
        }
    }
    
    if (ring_buffer_count() == 0) {
        ESP_LOGW(TAG, "Timeout waiting for trigger/frames after %d ms", wait_count);
        client_ready = false;
        waiting_for_trigger = false;
        // FIX Bug 5: Stop camera if it somehow started but produced no frames,
        // so camera_stream_started is not left true for the next connection.
        if (g_video_fd >= 0 && camera_stream_started) {
            app_video_stream_off(g_video_fd);
            camera_stream_started = false;
        }
        ring_write_index = 0;
        ring_read_index = 0;
        ring_frame_count = 0;
        httpd_resp_send_chunk(req, NULL, 0);  // End chunked response
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Trigger received! First frame ready after %d ms, cached: %d", 
             wait_count, ring_buffer_count());

    ESP_LOGI(TAG, "MJPEG Stream starting, cached frames: %d", ring_buffer_count());

    current_stream_mode = STREAM_MODE_MJPEG;
    stream_active = true;

    // Temporary buffer for copying frame data (to release mutex quickly)
    uint8_t *send_buffer = heap_caps_malloc(max_frame_size + 256, MALLOC_CAP_SPIRAM);
    if (send_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate send buffer");
        goto stream_end;
    }

    // Phase 1: Send all cached frames first
    // Copy frames quickly while holding mutex, then send outside mutex
    while (1) 
    {
        size_t frame_size = 0;
        
        if (xSemaphoreTake(stream_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            cached_frame_t *cached = ring_buffer_get_frame();
            if (cached != NULL && cached->size > 0) {
                frame_size = cached->size;
                memcpy(send_buffer, cached->data, frame_size);
            }
            xSemaphoreGive(stream_mutex);
        }
        
        if (frame_size == 0) break;  // No more cached frames
        
        // Send outside mutex to avoid blocking frame callback
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        if (res != ESP_OK) goto stream_cleanup;

        size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, (unsigned int)frame_size);
        res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res != ESP_OK) goto stream_cleanup;

        res = httpd_resp_send_chunk(req, (const char *)send_buffer, frame_size);
        if (res != ESP_OK) goto stream_cleanup;

        cached_frames_sent++;
        vTaskDelay(pdMS_TO_TICKS(2));  // Brief yield
    }
    
    if (cached_frames_sent > 0) {
        ESP_LOGI(TAG, "Sent %d cached frames to client", cached_frames_sent);
    }

    int64_t last_loop_end = esp_timer_get_time();
    int debug_counter = 0;

    // Phase 2: Stream live frames from ring buffer
    // Copy frame data quickly, release mutex, then do blocking HTTP send
    while (stream_active) {
        debug_counter++;
        int64_t loop_start = esp_timer_get_time();

        size_t frame_size = 0;
        uint32_t frame_num = 0;
        
        if (xSemaphoreTake(stream_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            cached_frame_t *frame = ring_buffer_get_frame();
            if (frame != NULL && frame->size > 0) {
                frame_size = frame->size;
                frame_num = frame->frame_number;
                memcpy(send_buffer, frame->data, frame_size);
            }
            xSemaphoreGive(stream_mutex);
        }
        
        if (frame_size > 0) {
            ESP_LOGD(TAG, "TX Frame Code: %lu", (unsigned long)frame_num);

            int64_t t_send_start = esp_timer_get_time();

            // Combine boundary + MIME header into a single send to reduce TCP flushes
            size_t blen = strlen(_STREAM_BOUNDARY);
            memcpy(part_buf, _STREAM_BOUNDARY, blen);
            size_t hlen = snprintf(part_buf + blen, sizeof(part_buf) - blen,
                                   _STREAM_PART, (unsigned int)frame_size);
            res = httpd_resp_send_chunk(req, part_buf, blen + hlen);
            if (res != ESP_OK) break;

            // Send JPEG payload
            res = httpd_resp_send_chunk(req, (const char *)send_buffer, frame_size);
            if (res == ESP_OK) {
                sent_frame_count++;  // Count successfully sent frames
            }

            int64_t t_send_end = esp_timer_get_time();
            float send_cost_ms = (t_send_end - t_send_start) / 1000.0f;

            // Warn only when significantly over one 100Mbps frame budget (~9ms for 120KB)
            if (send_cost_ms > 40.0f) {
                ESP_LOGW(TAG, "[Perf] Net Blocked! Send cost: %.2f ms (Size: %u). TCP/IP or Bandwidth limit.",
                         send_cost_ms, (unsigned)frame_size);
            } else if (debug_counter % 200 == 0) {
                ESP_LOGI(TAG, "[Perf] Send OK: %.2f ms (size=%u, cached=%d)",
                         send_cost_ms, (unsigned)frame_size, ring_buffer_count());
            }

            if (res != ESP_OK) break;

            // Only yield if this send took very long (avoid WDT), otherwise keep sending
            int64_t now = esp_timer_get_time();
            if ((now - loop_start) > 30000) {
                last_loop_end = now;
                vTaskDelay(1);
            }
        } else {
            // No frame ready — wait for callback notification (up to 100ms)
            last_loop_end = esp_timer_get_time();
            if (frame_ready_sem != NULL) {
                xSemaphoreTake(frame_ready_sem, pdMS_TO_TICKS(100));
            } else {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
    }

stream_cleanup:
    if (send_buffer != NULL) {
        heap_caps_free(send_buffer);
    }

stream_end:
    // CRITICAL: Full state reset for clean reconnection
    stream_active = false;
    client_ready = false;
    waiting_for_trigger = false;
    camera_preheating = false;  // Reset preheat state
    frames_to_skip = 0;  // Reset frame skip counter

    // Drain the frame-ready semaphore so it's clean for next connection
    if (frame_ready_sem != NULL) {
        xSemaphoreTake(frame_ready_sem, 0);
    }
    
    // FIX: Race condition — a new trigger rising edge may fire between TRIGGER_STOP
    // (which exits the stream loop) and this cleanup code. In that case,
    // trigger_event_handler_task has already restarted the camera and set
    // camera_stream_started=true for the new recording. Stopping the camera here
    // or clearing the ring buffer would silently kill the new trigger's recording,
    // leaving the next client waiting 60s with no frames → "cannot connect".
    //
    // Solution: check the currently selected trigger source before touching camera or ring buffer.
    //   - Trigger inactive: normal disconnect path — stop camera, clear buffer.
    //   - Trigger active:   new trigger already owns the camera — leave it running.
    if (!is_current_trigger_active()) {
        // Normal path: trigger is not active, clean up fully.
        if (g_video_fd >= 0 && camera_stream_started) {
            app_video_stream_off(g_video_fd);
            camera_stream_started = false;
            ESP_LOGI(TAG, "Camera stopped on client disconnect");
        }

        // FIX Bug 1: Do NOT call app_trigger_reset() here.
        // If a new trigger rising edge has already fired, its g_trigger_start_time
        // has been written by the ISR. Calling app_trigger_reset() would clear it
        // to 0, causing video_frame_callback to see no valid trigger and drop all
        // frames silently → ring buffer stays empty → next client cannot connect.
        // The ISR naturally resets g_trigger_start_time and g_frame_count on each
        // rising edge, so no explicit reset is needed here.

        // Clear ring buffer (stale frames from completed trigger session)
        ring_write_index = 0;
        ring_read_index = 0;
        ring_frame_count = 0;
    } else {
        // New trigger is already active: its rising edge fired during our cleanup.
        // trigger_event_handler_task owns the camera and ring buffer — do not touch them.
        // The next stream_handler call will see camera_stream_started=true and
        // pick up frames directly from the ring buffer.
        ESP_LOGI(TAG, "New trigger active during cleanup — preserving camera and ring buffer");
    }
    
    ESP_LOGI(TAG, "Stream client disconnected (sent %d cached + live frames)", cached_frames_sent);
    return res;
}

/**
 * @brief HTTP handler for raw H264 video stream
 * Note: H264 stream requires special player (VLC, ffplay) or JavaScript MSE
 * Currently not fully implemented - uses MJPEG ring buffer instead
 */
static esp_err_t h264_stream_handler(httpd_req_t *req)
{
    ESP_LOGW(TAG, "H264 streaming not fully implemented, redirecting to MJPEG");
    
    // For now, redirect to MJPEG stream handler
    return stream_handler(req);
}

/**
 * @brief HTTP handler for stream info/status
 */
static esp_err_t stream_info_handler(httpd_req_t *req)
{
    char response[512];
    int len = snprintf(response, sizeof(response),
        "{"
        "\"status\":\"running\","
        "\"mode\":\"%s\","
        "\"width\":%lu,"
        "\"height\":%lu,"
        "\"format\":\"%s\","
        "\"jpeg_quality\":%d,"
        "\"h264_available\":%s,"
        "\"endpoints\":{"
        "\"mjpeg\":\"/stream\","
        "\"h264\":\"/stream/h264\","
        "\"info\":\"/stream/info\""
        "}"
        "}",
        current_stream_mode == STREAM_MODE_MJPEG ? "MJPEG" : "H264",
        (unsigned long)app_video_get_width(),
        (unsigned long)app_video_get_height(),
        "RGB888",
        g_jpeg_quality,
        h264_fd >= 0 ? "true" : "false"
    );
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, response, len);
}

/**
 * @brief HTTP handler for single snapshot (latest frame)
 */
static esp_err_t snapshot_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Snapshot requested from %s", req->uri);
    
    if (!snapshot_ready || snapshot_buffer == NULL || snapshot_size == 0) {
        ESP_LOGW(TAG, "No snapshot available");
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_send(req, "No frame available yet", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }
    
    if (xSemaphoreTake(stream_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Set headers
        httpd_resp_set_type(req, "image/jpeg");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
        
        // Add timestamp header
        char ts_header[32];
        snprintf(ts_header, sizeof(ts_header), "%lld", (long long)snapshot_timestamp_us);
        httpd_resp_set_hdr(req, "X-Timestamp-Us", ts_header);
        
        // Send JPEG data
        esp_err_t res = httpd_resp_send(req, (const char *)snapshot_buffer, snapshot_size);
        
        xSemaphoreGive(stream_mutex);
        
        ESP_LOGI(TAG, "Snapshot sent: %u bytes, timestamp: %lld us", 
                 (unsigned int)snapshot_size, (long long)snapshot_timestamp_us);
        return res;
    }
    
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to acquire mutex");
    return ESP_FAIL;
}

/**
 * @brief HTTP handler for index page with MJPEG/H264 viewer
 */
static esp_err_t index_handler(httpd_req_t *req)
{
    const char *html = 
        "<!DOCTYPE html><html><head><title>ESP32-P4 Camera Stream</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>"
        "body{font-family:Arial;text-align:center;background:#1a1a2e;color:#fff;margin:0;padding:20px;}"
        "h1{color:#0f9;}img,video{border:2px solid #0f9;max-width:100%;height:auto;}"
        ".info{margin:20px;padding:10px;background:#16213e;border-radius:8px;}"
        ".btn{background:#0f9;color:#000;padding:10px 20px;border:none;border-radius:5px;cursor:pointer;margin:5px;}"
        ".btn:hover{background:#0c7;}"
        ".endpoints{text-align:left;display:inline-block;}"
        "</style></head><body>"
        "<h1>🎥 ESP32-P4 Camera Stream</h1>"
        "<div class='info'>"
        "<p><b>Camera:</b> OV2710 | <b>Resolution:</b> 1280x720 @ 25fps</p>"
        "<div class='endpoints'>"
        "<p><b>Available Endpoints:</b></p>"
        "<p>📷 MJPEG Stream: <a href='/stream' style='color:#0f9'>/stream</a></p>"
        "<p>📸 Snapshot: <a href='/snapshot' style='color:#0f9'>/snapshot</a> (single JPEG image)</p>"
        "<p>🎬 H264 Stream: <a href='/stream/h264' style='color:#0f9'>/stream/h264</a></p>"
        "<p>📊 Stream Info: <a href='/stream/info' style='color:#0f9'>/stream/info</a></p>"
        "</div>"
        "</div>"
        "<button class='btn' onclick='location.reload()'>Refresh</button>"
        "<button class='btn' onclick=\"window.open('/snapshot')\">Snapshot</button>"
        "<button class='btn' onclick=\"window.open('/stream/info')\">Info</button>"
        "<div id='stream-container' style='margin-top:20px;'>"
        "<img id='stream' src='/stream' alt='Video Stream' onerror=\"this.alt='Stream unavailable - click Refresh'\">"
        "</div>"
        "<p style='color:#666;font-size:12px;margin-top:20px;'>"
        "MJPEG: Direct browser playback | H264: Use VLC (network stream) or ffplay"
        "</p>"
        "<p style='color:#444;font-size:11px;'>"
        "H264 Command: ffplay http://&lt;ip&gt;/stream/h264"
        "</p>"
        "</body></html>";
    
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, strlen(html));
}

esp_err_t app_video_stream_server_start(int video_fd)
{
    if (stream_httpd != NULL) {
        ESP_LOGW(TAG, "Stream server already running");
        return ESP_OK;
    }

    g_video_fd = video_fd;
    
    // Create mutex
    stream_mutex = xSemaphoreCreateMutex();
    if (stream_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    // Create frame-ready semaphore (binary, counting up to 1 pending notification)
    frame_ready_sem = xSemaphoreCreateBinary();
    if (frame_ready_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create frame_ready semaphore");
        vSemaphoreDelete(stream_mutex);
        return ESP_FAIL;
    }

    // Initialize hardware JPEG encoder
    jpeg_encode_engine_cfg_t encode_eng_cfg = {
        .timeout_ms = 5000,
    };
    esp_err_t ret = jpeg_new_encoder_engine(&encode_eng_cfg, &jpeg_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create JPEG encoder: %s", esp_err_to_name(ret));
        vSemaphoreDelete(stream_mutex);
        return ret;
    }
    ESP_LOGI(TAG, "Hardware JPEG encoder initialized");

    // Configure JPEG encoding parameters
    // RGB888: unambiguous 3-byte format, no layout mismatch, correct colors
    jpeg_enc_config.src_type = JPEG_ENCODE_IN_FORMAT_RGB888;
    jpeg_enc_config.sub_sample = JPEG_DOWN_SAMPLING_YUV444;  // RGB888 requires YUV444
    jpeg_enc_config.image_quality = g_jpeg_quality;
    jpeg_enc_config.width = app_video_get_width();
    jpeg_enc_config.height = app_video_get_height();

    // Allocate JPEG output buffer using JPEG DMA aligned memory
    jpeg_encode_memory_alloc_cfg_t jpeg_mem_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
    };
    // Allocate buffer as 1/4 of raw frame size to handle worst-case JPEG output.
    // 720P Raw ~1.84MB: 1/4 = 460KB. 1080P Raw ~3.95MB: 1/4 = 990KB.
    // JPEG frames are typically 50KB-200KB; 1/4 gives enough headroom.
    size_t raw_frame_size = app_video_get_buf_size();
    jpeg_output_buffer = (uint8_t *)jpeg_alloc_encoder_mem(raw_frame_size / 4, 
                                                           &jpeg_mem_cfg, 
                                                           &jpeg_output_buffer_size);
    if (jpeg_output_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate JPEG output buffer");
        jpeg_del_encoder_engine(jpeg_encoder);
        vSemaphoreDelete(stream_mutex);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "JPEG output buffer allocated: %u bytes", (unsigned int)jpeg_output_buffer_size);

    // Initialize ring buffer for frame caching
    max_frame_size = jpeg_output_buffer_size;
    for (int i = 0; i < FRAME_RING_BUFFER_SIZE; i++) {
        frame_ring_buffer[i].data = heap_caps_malloc(max_frame_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (frame_ring_buffer[i].data == NULL) {
            ESP_LOGE(TAG, "Failed to allocate ring buffer frame %d", i);
            // Free already allocated buffers
            for (int j = 0; j < i; j++) {
                heap_caps_free(frame_ring_buffer[j].data);
            }
            heap_caps_free(jpeg_output_buffer);
            jpeg_del_encoder_engine(jpeg_encoder);
            vSemaphoreDelete(stream_mutex);
            return ESP_ERR_NO_MEM;
        }
        frame_ring_buffer[i].size = 0;
        frame_ring_buffer[i].valid = false;
    }
    ring_write_index = 0;
    ring_read_index = 0;
    ring_frame_count = 0;
    ESP_LOGI(TAG, "Ring buffer initialized: %d frames x %u bytes", 
             FRAME_RING_BUFFER_SIZE, (unsigned int)max_frame_size);

    // Allocate timestamp embedding buffer (JPEG size + header overhead)
    jpeg_ts_buffer_size = max_frame_size + JPEG_TS_HEADER_SIZE;
    jpeg_ts_buffer = heap_caps_malloc(jpeg_ts_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (jpeg_ts_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate timestamp buffer");
        for (int j = 0; j < FRAME_RING_BUFFER_SIZE; j++) {
            heap_caps_free(frame_ring_buffer[j].data);
        }
        heap_caps_free(jpeg_output_buffer);
        jpeg_del_encoder_engine(jpeg_encoder);
        vSemaphoreDelete(stream_mutex);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Timestamp buffer allocated: %u bytes", (unsigned int)jpeg_ts_buffer_size);

    // Allocate snapshot buffer (same size as max JPEG frame + timestamp header)
    snapshot_buffer = heap_caps_malloc(jpeg_ts_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (snapshot_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate snapshot buffer");
        // Continue without snapshot feature
    } else {
        snapshot_size = 0;
        snapshot_ready = false;
        ESP_LOGI(TAG, "Snapshot buffer allocated: %u bytes", (unsigned int)max_frame_size);
    }

    // Configure manual exposure (avoids AE convergence delay at startup)
    if (MANUAL_EXPOSURE_US > 0) {
        // Disable auto exposure first
        esp_err_t ae_ret = app_video_set_auto_exposure(video_fd, false);
        if (ae_ret == ESP_OK) {
            // Set fixed exposure time
            esp_err_t exp_ret = app_video_set_exposure_us(video_fd, MANUAL_EXPOSURE_US);
            if (exp_ret == ESP_OK) {
                ESP_LOGI(TAG, "Manual exposure configured: %d us (%.1f ms)", 
                         MANUAL_EXPOSURE_US, MANUAL_EXPOSURE_US / 1000.0f);
            } else {
                ESP_LOGW(TAG, "Failed to set exposure, using auto mode");
                app_video_set_auto_exposure(video_fd, true);
            }
        } else {
            ESP_LOGW(TAG, "Auto exposure control not supported, using default");
        }
    } else {
        ESP_LOGI(TAG, "Using auto exposure (AE)");
    }

    // Enable capture (independent of HTTP connection)
    capture_active = true;

    // Try to initialize H264 encoder (optional)
    h264_fd = open(ESP_VIDEO_H264_DEVICE_NAME, O_RDWR);
    if (h264_fd >= 0) {
        ESP_LOGI(TAG, "Hardware H264 encoder available at %s", ESP_VIDEO_H264_DEVICE_NAME);
        // Configure H264 encoder format
        struct v4l2_format h264_fmt = {
            .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
            .fmt.pix.width = app_video_get_width(),
            .fmt.pix.height = app_video_get_height(),
            .fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420,
        };
        if (ioctl(h264_fd, VIDIOC_S_FMT, &h264_fmt) != 0) {
            ESP_LOGW(TAG, "Failed to configure H264 encoder input format");
            close(h264_fd);
            h264_fd = -1;
        }
    } else {
        ESP_LOGW(TAG, "Hardware H264 encoder not available (camera using RGB format)");
    }

    // Perform startup latency self-test (calibration)
    // Start video task first (without streaming) for calibration
    ret = app_video_stream_task_start_ex(g_video_fd, 1, NULL, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start video stream task for calibration");
        if (h264_fd >= 0) close(h264_fd);
        heap_caps_free(jpeg_output_buffer);
        jpeg_del_encoder_engine(jpeg_encoder);
        vSemaphoreDelete(stream_mutex);
        return ret;
    }
    
    // Run calibration (measures STREAMON to first frame delay)
    perform_startup_calibration(g_video_fd);

    // AWB warmup: run camera for 3 seconds to let ISP Auto White Balance converge.
    // The ISP hardware retains gain registers after STREAMOFF, so the converged AWB
    // state provides a good starting point when the actual trigger STREAMON happens.
    // Without this, each triggered recording starts with wrong (pink/red) white balance
    // since AWB needs ~1-2s to converge but recordings are only ~400ms long.
    ESP_LOGI(TAG, "AWB warmup: streaming for 3s to let white balance converge...");
    app_video_register_frame_operation_cb(calibration_frame_callback);  // lightweight callback
    if (app_video_stream_on(g_video_fd) == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(3000));  // 3 seconds for AWB convergence
        app_video_stream_off(g_video_fd);
        ESP_LOGI(TAG, "AWB warmup complete");
    } else {
        ESP_LOGW(TAG, "AWB warmup stream failed to start");
    }
    app_video_register_frame_operation_cb(NULL);  // clear callback until real one is registered

    // Configure exposure time for fast triggering (optional)
    // Set manual exposure to 10ms for faster response and consistent timing
    // Note: Shorter exposure = faster response but may need more light
#ifdef CONFIG_TRIGGER_EXPOSURE_US
    app_video_set_auto_exposure(g_video_fd, false);  // Disable auto exposure
    app_video_set_exposure_us(g_video_fd, CONFIG_TRIGGER_EXPOSURE_US);
#endif

    // Query current exposure for debugging (may not be supported by all sensors)
    uint32_t current_exposure_us = 0;
    if (app_video_get_exposure_us(g_video_fd, &current_exposure_us) == ESP_OK) {
        ESP_LOGI(TAG, "Current exposure: %lu us (%.2f ms)", 
                 (unsigned long)current_exposure_us, current_exposure_us / 1000.0f);
    } else {
        ESP_LOGD(TAG, "Exposure query not supported by this sensor (OV2710)");
        ESP_LOGD(TAG, "OV2710 uses internal AE, exposure controlled by AE_LEVEL");
    }

    // Register the actual frame callback after calibration
    app_video_register_frame_operation_cb(video_frame_callback);

    // Create event queue and handler task for trigger events
    // This avoids stack overflow by running heavy V4L2 ops in a task with sufficient stack
    trigger_event_queue = xQueueCreate(4, sizeof(trigger_event_t));
    if (trigger_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create trigger event queue");
        return ESP_FAIL;
    }
    
    // Create handler task with 8KB stack (enough for V4L2 operations)
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        trigger_event_handler_task,
        "trig_handler",
        8192,  // 8KB stack
        NULL,
        5,     // Priority
        &trigger_handler_task,
        1      // Core 1 (same as video task)
    );
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create trigger handler task");
        vQueueDelete(trigger_event_queue);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Trigger event handler task created (8KB stack)");

    // Register lightweight trigger callback
    app_trigger_register_callback(trigger_state_callback);
    
    // After calibration, keep stream OFF and wait for trigger
    // Camera will start when trigger signal arrives
    camera_stream_started = false;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Camera ready, waiting for trigger on GPIO %d", TRIGGER_GPIO_PIN);
    ESP_LOGI(TAG, "  Camera: 720P @ 25fps (exposure-locked from 60fps native)");
    ESP_LOGI(TAG, "  Calibrated startup delay: %.2f ms (±%.2f ms)", 
             calibrated_startup_delay_us / 1000.0f, startup_delay_jitter_us / 1000.0f);
    ESP_LOGI(TAG, "  Expected first frame: %.2f ms after trigger",
             calibrated_startup_delay_us / 1000.0f);
    ESP_LOGI(TAG, "========================================");

    // Configure HTTP server
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 8;
    config.stack_size = 16384; // 16KB stack to be safe
    config.core_id = 0; // Keep on Core 0 for stability
    config.task_priority = 10; // Increase to 10 (higher than standard 5, lower than LwIP 18)
    // FIX Bug 3: Allow server to reclaim oldest idle socket when the pool is full.
    // Without this, a full socket pool causes ALL new connections to be rejected
    // until the server is restarted. This is the last-resort safety net.
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);
    ret = httpd_start(&stream_httpd, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        if (h264_fd >= 0) close(h264_fd);
        heap_caps_free(jpeg_output_buffer);
        jpeg_del_encoder_engine(jpeg_encoder);
        vSemaphoreDelete(stream_mutex);
        return ret;
    }

    // Register URI handlers
    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(stream_httpd, &index_uri);

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(stream_httpd, &stream_uri);

    // Register H264 stream handler
    httpd_uri_t h264_uri = {
        .uri = "/stream/h264",
        .method = HTTP_GET,
        .handler = h264_stream_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(stream_httpd, &h264_uri);

    httpd_uri_t info_uri = {
        .uri = "/stream/info",
        .method = HTTP_GET,
        .handler = stream_info_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(stream_httpd, &info_uri);

    // Register snapshot handler
    httpd_uri_t snapshot_uri = {
        .uri = "/snapshot",
        .method = HTTP_GET,
        .handler = snapshot_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(stream_httpd, &snapshot_uri);

    ESP_LOGI(TAG, "Video stream server started");
    ESP_LOGI(TAG, "  MJPEG stream: http://<device_ip>/stream");
    ESP_LOGI(TAG, "  Snapshot:     http://<device_ip>/snapshot");
    ESP_LOGI(TAG, "  H264 stream:  http://<device_ip>/stream/h264 %s", h264_fd >= 0 ? "(available)" : "(not available - need YUV420 input)");
    ESP_LOGI(TAG, "Open http://<device_ip>/ in browser");
    
    return ESP_OK;
}

esp_err_t app_video_stream_server_stop(void)
{
    stream_active = false;
    
    // Stop video capture task
    if (g_video_fd >= 0) {
        app_video_stream_task_stop(g_video_fd);
        vTaskDelay(pdMS_TO_TICKS(100)); // Give task time to exit
    }
    
    if (stream_httpd != NULL) {
        httpd_stop(stream_httpd);
        stream_httpd = NULL;
    }

    if (h264_fd >= 0) {
        close(h264_fd);
        h264_fd = -1;
    }

    if (jpeg_output_buffer != NULL) {
        heap_caps_free(jpeg_output_buffer);
        jpeg_output_buffer = NULL;
    }

    // Free ring buffer memory
    for (int i = 0; i < FRAME_RING_BUFFER_SIZE; i++) {
        if (frame_ring_buffer[i].data != NULL) {
            heap_caps_free(frame_ring_buffer[i].data);
            frame_ring_buffer[i].data = NULL;
            frame_ring_buffer[i].size = 0;
            frame_ring_buffer[i].valid = false;
        }
    }
    ring_write_index = 0;
    ring_read_index = 0;
    ring_frame_count = 0;

    // Free timestamp embedding buffer
    if (jpeg_ts_buffer != NULL) {
        heap_caps_free(jpeg_ts_buffer);
        jpeg_ts_buffer = NULL;
    }

    // Free snapshot buffer
    if (snapshot_buffer != NULL) {
        heap_caps_free(snapshot_buffer);
        snapshot_buffer = NULL;
    }

    if (jpeg_encoder != NULL) {
        jpeg_del_encoder_engine(jpeg_encoder);
        jpeg_encoder = NULL;
    }

    if (stream_mutex != NULL) {
        vSemaphoreDelete(stream_mutex);
        stream_mutex = NULL;
    }

    g_video_fd = -1;
    ESP_LOGI(TAG, "Video stream server stopped");
    return ESP_OK;
}

httpd_handle_t app_video_stream_get_server(void)
{
    return stream_httpd;
}

void app_video_stream_set_jpeg_quality(int quality)
{
    if (quality < 1)   quality = 1;
    if (quality > 100) quality = 100;
    g_jpeg_quality = quality;
    /* Update the encode config so the next frame uses the new quality */
    jpeg_enc_config.image_quality = quality;
    ESP_LOGI(TAG, "JPEG quality updated to %d", quality);
}

int app_video_stream_get_jpeg_quality(void)
{
    return g_jpeg_quality;
}

void app_video_stream_set_skip_frames(uint32_t n)
{
    g_skip_frames_on_trigger = n;
    ESP_LOGI(TAG, "Skip-frames-on-trigger set to %lu", (unsigned long)n);
}

uint32_t app_video_stream_get_skip_frames(void)
{
    return g_skip_frames_on_trigger;
}

void app_video_stream_set_trigger_mode(app_trigger_mode_t mode)
{
    if (mode != APP_TRIGGER_MODE_GPIO && mode != APP_TRIGGER_MODE_UDP) {
        return;
    }

    // Keep this as UI/reporting preference only.
    // Runtime trigger logic is GPIO OR UDP and does not disable the other source.
    g_trigger_mode = mode;
    ESP_LOGI(TAG, "Trigger mode set to %s", mode == APP_TRIGGER_MODE_UDP ? "UDP" : "GPIO");
}

app_trigger_mode_t app_video_stream_get_trigger_mode(void)
{
    return g_trigger_mode;
}

esp_err_t app_video_stream_udp_trigger_start(void)
{
    if (trigger_event_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    g_udp_trigger_start_time = esp_timer_get_time();
    g_udp_trigger_active = true;

    notify_trigger_state_if_changed(is_current_trigger_active());
    ESP_LOGI(TAG, "UDP trigger start: udp_active=1 gpio_active=%d combined=%d",
             app_trigger_is_active(), is_current_trigger_active());
    return ESP_OK;
}

esp_err_t app_video_stream_udp_trigger_stop(void)
{
    if (trigger_event_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    g_udp_trigger_active = false;
    g_udp_trigger_start_time = 0;
    notify_trigger_state_if_changed(is_current_trigger_active());
    ESP_LOGI(TAG, "UDP trigger stop: udp_active=0 gpio_active=%d combined=%d",
             app_trigger_is_active(), is_current_trigger_active());
    return ESP_OK;
}

void app_video_stream_get_trigger_stats(app_trigger_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    stats->active = is_current_trigger_active();
    stats->frame_count = g_udp_frame_counter;
    stats->dropped_frames_est = g_udp_dropped_frames_est;
    stats->first_frame_latency_us = g_udp_first_frame_latency_us;
    stats->last_frame_interval_us = g_udp_last_frame_interval_us;
    stats->avg_frame_interval_us = (g_udp_interval_samples > 0)
                                   ? (g_udp_interval_sum_us / g_udp_interval_samples)
                                   : 0;
}
