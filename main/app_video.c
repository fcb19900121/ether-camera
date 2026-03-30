/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/errno.h>
#include <errno.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "linux/videodev2.h"
#include "linux/v4l2-controls.h"
#include "esp_video_init.h"
#include "app_video.h"

static const char *TAG = "app_video";

#define MAX_BUFFER_COUNT                (3)
#define MIN_BUFFER_COUNT                (2)
#define VIDEO_TASK_STACK_SIZE           (4 * 1024)
#define VIDEO_TASK_PRIORITY             (4)

typedef struct {
    uint8_t *camera_buffer[MAX_BUFFER_COUNT];
    size_t camera_buf_size;
    uint8_t camera_buf_count;
    uint32_t camera_buf_hes;
    uint32_t camera_buf_ves;
    struct v4l2_buffer v4l2_buf;
    uint8_t camera_mem_mode;
    app_video_frame_operation_cb_t user_camera_video_frame_operation_cb;
    TaskHandle_t video_stream_task_handle;
    uint8_t video_task_core_id;
    bool video_task_delete;
    void *video_task_user_data;
    int video_fd;  // Store video file descriptor
    bool stream_active;  // Track if stream is active
} app_video_t;

static app_video_t app_camera_video;

esp_err_t app_video_main(i2c_master_bus_handle_t i2c_bus_handle)
{
#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR
    esp_video_init_csi_config_t csi_config[] = {
        {
            .sccb_config = {
                .init_sccb = true,
                .i2c_config = {
                    .port      = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_PORT,
                    .scl_pin   = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_SCL_PIN,
                    .sda_pin   = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_SDA_PIN,
                },
                .freq      = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_FREQ,
            },
            .reset_pin = CONFIG_EXAMPLE_MIPI_CSI_CAM_SENSOR_RESET_PIN,
            .pwdn_pin  = CONFIG_EXAMPLE_MIPI_CSI_CAM_SENSOR_PWDN_PIN,
        },
    };

    if (i2c_bus_handle != NULL) {
        csi_config[0].sccb_config.init_sccb = false;
        csi_config[0].sccb_config.i2c_handle = i2c_bus_handle;
    }
#endif

    esp_video_init_config_t cam_config = {
#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR > 0
        .csi      = csi_config,
#endif
    };

    return esp_video_init(&cam_config);
}

int app_video_open(char *dev, video_fmt_t init_fmt)
{
    struct v4l2_format default_format;
    struct v4l2_capability capability;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

#if CONFIG_EXAMPLE_ENABLE_CAM_SENSOR_PIC_VFLIP || CONFIG_EXAMPLE_ENABLE_CAM_SENSOR_PIC_HFLIP
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];
#endif

    int fd = open(dev, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "Open video failed");
        return -1;
    }

    if (ioctl(fd, VIDIOC_QUERYCAP, &capability)) {
        ESP_LOGE(TAG, "failed to get capability");
        goto exit_0;
    }

    ESP_LOGI(TAG, "version: %d.%d.%d", (uint16_t)(capability.version >> 16),
             (uint8_t)(capability.version >> 8),
             (uint8_t)capability.version);
    ESP_LOGI(TAG, "driver:  %s", capability.driver);
    ESP_LOGI(TAG, "card:    %s", capability.card);
    ESP_LOGI(TAG, "bus:     %s", capability.bus_info);

    memset(&default_format, 0, sizeof(struct v4l2_format));
    default_format.type = type;
    if (ioctl(fd, VIDIOC_G_FMT, &default_format) != 0) {
        ESP_LOGE(TAG, "failed to get format");
        goto exit_0;
    }

    ESP_LOGI(TAG, "width=%" PRIu32 " height=%" PRIu32, default_format.fmt.pix.width, default_format.fmt.pix.height);

    app_camera_video.camera_buf_hes = default_format.fmt.pix.width;
    app_camera_video.camera_buf_ves = default_format.fmt.pix.height;

    if (default_format.fmt.pix.pixelformat != init_fmt) {
        struct v4l2_format format = {
            .type = type,
            .fmt.pix.width = default_format.fmt.pix.width,
            .fmt.pix.height = default_format.fmt.pix.height,
            .fmt.pix.pixelformat = init_fmt,
        };

        if (ioctl(fd, VIDIOC_S_FMT, &format) != 0) {
            ESP_LOGE(TAG, "failed to set format");
            goto exit_0;
        }
    }

#if CONFIG_EXAMPLE_ENABLE_CAM_SENSOR_PIC_VFLIP
    controls.ctrl_class = V4L2_CTRL_CLASS_USER;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_VFLIP;
    control[0].value    = 1;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to mirror the frame horizontally and skip this step");
    }
#endif

#if CONFIG_EXAMPLE_ENABLE_CAM_SENSOR_PIC_HFLIP
    controls.ctrl_class = V4L2_CTRL_CLASS_USER;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_HFLIP;
    control[0].value    = 1;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to mirror the frame horizontally and skip this step");
    }
#endif

    return fd;
exit_0:
    close(fd);
    return -1;
}

esp_err_t app_video_set_bufs(int video_fd, uint32_t fb_num, const void **fb)
{
    if (fb_num > MAX_BUFFER_COUNT) {
        ESP_LOGE(TAG, "buffer num is too large");
        return ESP_FAIL;
    } else if (fb_num < MIN_BUFFER_COUNT) {
        ESP_LOGE(TAG, "At least two buffers are required");
        return ESP_FAIL;
    }

    struct v4l2_requestbuffers req;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    memset(&req, 0, sizeof(req));
    req.count = fb_num;
    app_camera_video.camera_buf_count = fb_num;
    req.type = type;

    app_camera_video.camera_mem_mode = req.memory = fb ? V4L2_MEMORY_USERPTR : V4L2_MEMORY_MMAP;

    if (ioctl(video_fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "req bufs failed");
        goto errout_req_bufs;
    }
    for (int i = 0; i < fb_num; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = type;
        buf.memory = req.memory;
        buf.index = i;

        if (ioctl(video_fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "query buf failed");
            goto errout_req_bufs;
        }

        if (req.memory == V4L2_MEMORY_MMAP) {
            app_camera_video.camera_buffer[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, video_fd, buf.m.offset);
            if (app_camera_video.camera_buffer[i] == NULL) {
                ESP_LOGE(TAG, "mmap failed");
                goto errout_req_bufs;
            }
        } else {
            if (!fb[i]) {
                ESP_LOGE(TAG, "frame buffer is NULL");
                goto errout_req_bufs;
            }
            buf.m.userptr = (unsigned long)fb[i];
            app_camera_video.camera_buffer[i] = (uint8_t *)fb[i];
        }

        app_camera_video.camera_buf_size = buf.length;

        if (ioctl(video_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "queue frame buffer failed");
            goto errout_req_bufs;
        }
    }

    return ESP_OK;

errout_req_bufs:
    close(video_fd);
    return ESP_FAIL;
}

esp_err_t app_video_get_bufs(int fb_num, void **fb)
{
    if (fb_num > MAX_BUFFER_COUNT) {
        ESP_LOGE(TAG, "buffer num is too large");
        return ESP_FAIL;
    } else if (fb_num < MIN_BUFFER_COUNT) {
        ESP_LOGE(TAG, "At least two buffers are required");
        return ESP_FAIL;
    }

    for (int i = 0; i < fb_num; i++) {
        if (app_camera_video.camera_buffer[i] != NULL) {
            fb[i] = app_camera_video.camera_buffer[i];
        } else {
            ESP_LOGE(TAG, "frame buffer is NULL");
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

uint32_t app_video_get_buf_size(void)
{
    uint32_t buf_size = app_camera_video.camera_buf_hes * app_camera_video.camera_buf_ves * (APP_VIDEO_FMT == APP_VIDEO_FMT_RGB565 ? 2 : 3);

    return buf_size;
}

uint32_t app_video_get_width(void)
{
    return app_camera_video.camera_buf_hes;
}

uint32_t app_video_get_height(void)
{
    return app_camera_video.camera_buf_ves;
}

static inline esp_err_t video_receive_video_frame(int video_fd)
{
    memset(&app_camera_video.v4l2_buf, 0, sizeof(app_camera_video.v4l2_buf));
    app_camera_video.v4l2_buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    app_camera_video.v4l2_buf.memory = app_camera_video.camera_mem_mode;

    int res = ioctl(video_fd, VIDIOC_DQBUF, &(app_camera_video.v4l2_buf));
    if (res != 0) {
        ESP_LOGE(TAG, "failed to receive video frame, errno=%d (%s)", errno, strerror(errno));
        goto errout;
    }
    
    // V4L2 timestamp not available on ESP32-P4 MIPI CSI driver
    // Use esp_timer instead for accurate frame timing
    if (app_camera_video.v4l2_buf.timestamp.tv_sec == 0 && 
        app_camera_video.v4l2_buf.timestamp.tv_usec == 0) {
        // Fill in with current time as fallback
        int64_t now = esp_timer_get_time();
        app_camera_video.v4l2_buf.timestamp.tv_sec = now / 1000000;
        app_camera_video.v4l2_buf.timestamp.tv_usec = now % 1000000;
    }

    return ESP_OK;

errout:
    return ESP_FAIL;
}

static inline void video_operation_video_frame(int video_fd)
{
    app_camera_video.v4l2_buf.m.userptr = (unsigned long)app_camera_video.camera_buffer[app_camera_video.v4l2_buf.index];
    app_camera_video.v4l2_buf.length = app_camera_video.camera_buf_size;

    uint8_t buf_index = app_camera_video.v4l2_buf.index;

    app_camera_video.user_camera_video_frame_operation_cb(
                        app_camera_video.camera_buffer[buf_index],
                        buf_index,
                        app_camera_video.camera_buf_hes,
                        app_camera_video.camera_buf_ves,
                        app_camera_video.camera_buf_size,
                        app_camera_video.video_task_user_data
                    );
}

static inline esp_err_t video_free_video_frame(int video_fd)
{
    if (ioctl(video_fd, VIDIOC_QBUF, &(app_camera_video.v4l2_buf)) != 0) {
        // This can happen after stream restart - buffer was already re-queued in video_stream_start()
        // Not a critical error, just log as debug
        ESP_LOGD(TAG, "QBUF failed for buffer %d (may be already queued)", (int)app_camera_video.v4l2_buf.index);
        return ESP_OK;  // Return OK since buffer is likely already queued
    }

    return ESP_OK;
}

static inline esp_err_t video_stream_start(int video_fd)
{
    ESP_LOGI(TAG, "Video Stream Start");

    // Re-queue all buffers before STREAMON (required after STREAMOFF)
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (int i = 0; i < app_camera_video.camera_buf_count; i++) {
        struct v4l2_buffer buf = {0};
        buf.type = type;
        buf.memory = app_camera_video.camera_mem_mode;
        buf.index = i;
        if (app_camera_video.camera_mem_mode == V4L2_MEMORY_USERPTR) {
            buf.m.userptr = (unsigned long)app_camera_video.camera_buffer[i];
            buf.length = app_camera_video.camera_buf_size;
        }
        // Ignore error if buffer already queued
        ioctl(video_fd, VIDIOC_QBUF, &buf);
    }

    if (ioctl(video_fd, VIDIOC_STREAMON, &type)) {
        ESP_LOGE(TAG, "failed to start stream");
        goto errout;
    }

    struct v4l2_format format = {0};
    format.type = type;
    if (ioctl(video_fd, VIDIOC_G_FMT, &format) != 0) {
        ESP_LOGE(TAG, "get fmt failed");
        goto errout;
    }

    app_camera_video.stream_active = true;
    return ESP_OK;

errout:
    return ESP_FAIL;
}

static inline esp_err_t video_stream_stop(int video_fd)
{
    ESP_LOGI(TAG, "Video Stream Stop");

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(video_fd, VIDIOC_STREAMOFF, &type)) {
        ESP_LOGE(TAG, "failed to stop stream");
        goto errout;
    }

    app_camera_video.stream_active = false;
    return ESP_OK;

errout:
    return ESP_FAIL;
}

static void video_stream_task(void *arg)
{
    int video_fd = *((int *)arg);
    ESP_LOGI(TAG, "Video stream task started, fd=%d", video_fd);

    while (1) {
        // Wait if stream is not active
        if (!app_camera_video.stream_active) {
            vTaskDelay(pdMS_TO_TICKS(10));  // Reduced from 50ms for faster response
            continue;
        }
        
        esp_err_t ret = video_receive_video_frame(video_fd);
        if (ret != ESP_OK) {
            // Only log error if stream should be active
            if (app_camera_video.stream_active) {
                ESP_LOGE(TAG, "failed to receive video frame, retrying...");
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (app_camera_video.user_camera_video_frame_operation_cb) {
            video_operation_video_frame(video_fd);
        }

        // Only QBUF if stream is still active (stream may have stopped during callback)
        // If stream stopped, buffer will be re-queued on next STREAMON
        if (app_camera_video.stream_active) {
            ret = video_free_video_frame(video_fd);
            if (ret != ESP_OK) {
                // This can happen during stream transitions, just log as debug
                ESP_LOGD(TAG, "QBUF skipped (stream stopping)");
            }
        }

        if (app_camera_video.video_task_delete) {
            app_camera_video.video_task_delete = false;
            video_stream_stop(video_fd);
            vTaskDelete(NULL);
        }
    }
    vTaskDelete(NULL);
}

esp_err_t app_video_stream_task_start(int video_fd, int core_id, void *user_data)
{
    return app_video_stream_task_start_ex(video_fd, core_id, user_data, true);
}

esp_err_t app_video_stream_task_start_ex(int video_fd, int core_id, void *user_data, bool start_stream)
{
    app_camera_video.video_task_core_id = core_id;
    app_camera_video.video_task_user_data = user_data;
    app_camera_video.video_fd = video_fd;  // Store fd in static struct

    if (start_stream) {
        video_stream_start(video_fd);
    }

    BaseType_t result = xTaskCreatePinnedToCore(video_stream_task, "video stream task", VIDEO_TASK_STACK_SIZE, &app_camera_video.video_fd, VIDEO_TASK_PRIORITY, &app_camera_video.video_stream_task_handle, core_id);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "failed to create video stream task");
        goto errout;
    }

    return ESP_OK;

errout:
    video_stream_stop(video_fd);
    return ESP_FAIL;
}

esp_err_t app_video_stream_task_restart(int video_fd)
{
    app_video_set_bufs(video_fd, app_camera_video.camera_buf_count, (const void **)app_camera_video.camera_buffer);

    esp_err_t ret = app_video_stream_task_start(video_fd, app_camera_video.video_task_core_id, app_camera_video.video_task_user_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to restart video stream task");
        goto errout;
    }

    return ESP_OK;

errout:
    return ESP_FAIL;
}

esp_err_t app_video_stream_task_stop(int video_fd)
{
    app_camera_video.video_task_delete = true;

    return ESP_OK;
}

esp_err_t app_video_register_frame_operation_cb(app_video_frame_operation_cb_t operation_cb)
{
    app_camera_video.user_camera_video_frame_operation_cb = operation_cb;

    return ESP_OK;
}

esp_err_t app_video_stream_on(int video_fd)
{
    if (app_camera_video.stream_active) {
        ESP_LOGW(TAG, "Stream already active");
        return ESP_OK;
    }
    return video_stream_start(video_fd);
}

esp_err_t app_video_stream_off(int video_fd)
{
    if (!app_camera_video.stream_active) {
        ESP_LOGW(TAG, "Stream already stopped");
        return ESP_OK;
    }
    return video_stream_stop(video_fd);
}

bool app_video_stream_is_active(void)
{
    return app_camera_video.stream_active;
}

int64_t app_video_get_frame_timestamp_us(void)
{
    // V4L2 timestamp is in struct timeval format (seconds + microseconds)
    struct timeval *tv = &app_camera_video.v4l2_buf.timestamp;
    return (int64_t)tv->tv_sec * 1000000LL + (int64_t)tv->tv_usec;
}

esp_err_t app_video_set_exposure_us(int video_fd, uint32_t exposure_us)
{
    // V4L2_CID_EXPOSURE_ABSOLUTE unit is 100us
    struct v4l2_control ctrl = {
        .id = V4L2_CID_EXPOSURE_ABSOLUTE,
        .value = exposure_us / 100,  // Convert us to 100us units
    };
    
    if (ctrl.value < 1) {
        ctrl.value = 1;  // Minimum 100us
    }
    
    if (ioctl(video_fd, VIDIOC_S_CTRL, &ctrl) != 0) {
        ESP_LOGE(TAG, "Failed to set exposure: %d (%s)", errno, strerror(errno));
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Exposure set to %lu us (%ld x 100us)", 
             (unsigned long)exposure_us, (long)ctrl.value);
    return ESP_OK;
}

esp_err_t app_video_get_exposure_us(int video_fd, uint32_t *exposure_us)
{
    if (exposure_us == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    struct v4l2_control ctrl = {
        .id = V4L2_CID_EXPOSURE_ABSOLUTE,
    };
    
    if (ioctl(video_fd, VIDIOC_G_CTRL, &ctrl) != 0) {
        // Not all sensors support exposure control via V4L2
        ESP_LOGD(TAG, "Exposure control not supported: %d (%s)", errno, strerror(errno));
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    *exposure_us = ctrl.value * 100;  // Convert 100us units to us
    return ESP_OK;
}

esp_err_t app_video_set_auto_exposure(int video_fd, bool auto_exposure)
{
    struct v4l2_control ctrl = {
        .id = V4L2_CID_EXPOSURE_AUTO,
        .value = auto_exposure ? V4L2_EXPOSURE_AUTO : V4L2_EXPOSURE_MANUAL,
    };

    if (ioctl(video_fd, VIDIOC_S_CTRL, &ctrl) != 0) {
        ESP_LOGE(TAG, "Failed to set auto exposure: %d (%s)", errno, strerror(errno));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Auto exposure: %s", auto_exposure ? "ON" : "OFF");
    return ESP_OK;
}
