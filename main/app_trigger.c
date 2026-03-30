/*
 * External Trigger Module Implementation
 * 
 * GPIO interrupt-based camera trigger with timestamp tracking
 */
#include "app_trigger.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "trigger";

// State variables
static volatile bool g_trigger_active = false;
static volatile int64_t g_trigger_start_time = 0;
static volatile uint32_t g_frame_count = 0;
static SemaphoreHandle_t g_trigger_mutex = NULL;
static app_trigger_callback_t g_trigger_callback = NULL;
static volatile bool g_callback_pending = false;
static volatile bool g_callback_state = false;

// Debouncing
static volatile int64_t g_last_isr_time = 0;
static volatile int g_last_level = -1;  // -1 = unknown, 0 = low, 1 = high
#define DEBOUNCE_TIME_US 10000  // 10ms debounce

// Task to handle callback outside ISR context
static TaskHandle_t g_callback_task_handle = NULL;
static volatile bool g_last_callback_state = false;  // Track last state sent to callback

static void trigger_callback_task(void *arg)
{
    bool last_state = false;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (g_trigger_callback) {
            bool current_state = g_callback_state;
            // Only call callback if state actually changed
            if (current_state != last_state) {
                last_state = current_state;
                g_trigger_callback(current_state);
            }
        }
    }
}

// GPIO ISR handler
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    int64_t now = esp_timer_get_time();
    int level = gpio_get_level(TRIGGER_GPIO_PIN);
    
    // Debounce: ignore if same level or too soon after last interrupt
    if (level == g_last_level) {
        return;  // No actual state change
    }
    if ((now - g_last_isr_time) < DEBOUNCE_TIME_US && g_last_level >= 0) {
        return;  // Too soon, likely bounce
    }
    
    g_last_isr_time = now;
    g_last_level = level;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (level == 1) {
        // Rising edge - start capture, reset timestamp
        g_trigger_start_time = now;
        g_frame_count = 0;
        g_trigger_active = true;
        g_callback_state = true;
    } else {
        // Falling edge - mark inactive but DON'T stop immediately
        // Let trigger_event_handler decide when to actually stop
        g_trigger_active = false;
        g_callback_state = false;
    }
    
    // Notify callback task
    if (g_callback_task_handle && g_trigger_callback) {
        vTaskNotifyGiveFromISR(g_callback_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

esp_err_t app_trigger_init(void)
{
    ESP_LOGI(TAG, "Initializing external trigger on GPIO %d", TRIGGER_GPIO_PIN);
    
    // Create mutex
    g_trigger_mutex = xSemaphoreCreateMutex();
    if (g_trigger_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Configure GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIGGER_GPIO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  // Pull down by default
        .intr_type = GPIO_INTR_ANYEDGE,        // Trigger on both edges
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        vSemaphoreDelete(g_trigger_mutex);
        return ret;
    }
    
    // Install GPIO ISR service
    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        vSemaphoreDelete(g_trigger_mutex);
        return ret;
    }
    
    // Add ISR handler
    ret = gpio_isr_handler_add(TRIGGER_GPIO_PIN, gpio_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        vSemaphoreDelete(g_trigger_mutex);
        return ret;
    }
    
    // Check initial state and set last known level for debouncing
    g_last_level = gpio_get_level(TRIGGER_GPIO_PIN);
    g_last_isr_time = esp_timer_get_time();
    
    if (g_last_level == 1) {
        g_trigger_start_time = g_last_isr_time;
        g_frame_count = 0;
        g_trigger_active = true;
        ESP_LOGI(TAG, "Trigger pin is HIGH, capture active");
    } else {
        g_trigger_active = false;
        ESP_LOGI(TAG, "Trigger pin is LOW, waiting for trigger");
    }
    
    ESP_LOGI(TAG, "External trigger initialized successfully");
    return ESP_OK;
}

void app_trigger_deinit(void)
{
    gpio_isr_handler_remove(TRIGGER_GPIO_PIN);
    gpio_reset_pin(TRIGGER_GPIO_PIN);
    
    if (g_trigger_mutex) {
        vSemaphoreDelete(g_trigger_mutex);
        g_trigger_mutex = NULL;
    }
    
    g_trigger_active = false;
    ESP_LOGI(TAG, "External trigger deinitialized");
}

bool app_trigger_is_active(void)
{
    return g_trigger_active;
}

int64_t app_trigger_get_timestamp_us(void)
{
    if (!g_trigger_active) {
        return -1;
    }
    return esp_timer_get_time() - g_trigger_start_time;
}

int64_t app_trigger_get_start_time(void)
{
    return g_trigger_start_time;
}

esp_err_t app_trigger_get_frame_info(trigger_frame_info_t *info)
{
    if (info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Return frame info if we have a valid start time (trigger was activated)
    // Don't depend on g_trigger_active - it may have gone low but camera still running
    // NOTE: This function does NOT allocate a frame number. Call app_trigger_alloc_frame_number()
    //       after successful encoding to get the frame number.
    if (g_trigger_start_time == 0) {
        info->is_triggered = false;
        info->frame_number = 0;
        info->timestamp_us = 0;
        return ESP_ERR_INVALID_STATE;
    }
    
    // Don't need mutex for reading - just get current trigger state
    info->is_triggered = true;  // We have a valid trigger session
    info->frame_number = 0;     // Will be assigned later by app_trigger_alloc_frame_number()
    info->timestamp_us = esp_timer_get_time() - g_trigger_start_time;
    return ESP_OK;
}

uint32_t app_trigger_alloc_frame_number(void)
{
    uint32_t frame_num = 0;
    if (xSemaphoreTake(g_trigger_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Frame numbers start from 1 (not 0)
        frame_num = ++g_frame_count;
        xSemaphoreGive(g_trigger_mutex);
    }
    return frame_num;
}

void app_trigger_manual_start(void)
{
    ESP_LOGI(TAG, "Manual trigger START");
    g_trigger_start_time = esp_timer_get_time();
    g_frame_count = 0;
    g_trigger_active = true;
    if (g_trigger_callback) {
        g_trigger_callback(true);
    }
}

void app_trigger_manual_stop(void)
{
    ESP_LOGI(TAG, "Manual trigger STOP");
    g_trigger_active = false;
    g_trigger_start_time = 0;  // Reset start time to invalidate frame info
    if (g_trigger_callback) {
        g_trigger_callback(false);
    }
}

void app_trigger_reset(void)
{
    // Reset trigger state without calling callback
    g_trigger_active = false;
    g_trigger_start_time = 0;
    g_frame_count = 0;
}

void app_trigger_register_callback(app_trigger_callback_t callback)
{
    g_trigger_callback = callback;
    
    // Create callback task if not exists
    if (g_callback_task_handle == NULL && callback != NULL) {
        xTaskCreate(trigger_callback_task, "trigger_cb", 2048, NULL, 5, &g_callback_task_handle);
    }
    
    ESP_LOGI(TAG, "Trigger callback registered");
}
