/* Ethernet Basic Example with Camera

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "ethernet_init.h"
#include "sdkconfig.h"
#include "lwip/ip4_addr.h"
#include "esp_heap_caps.h"

#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR
#include "app_video.h"
#include "app_video_stream.h"
#endif

#if CONFIG_TRIGGER_ENABLE
#include "app_trigger.h"
#endif

#include "app_console.h"
#include "app_udp_config.h"

static const char *TAG = "eth_cam_example";

#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR
static int g_video_cam_fd = -1;
#endif

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");

    /* Start UDP config service so the host can discover and configure the device */
    app_udp_config_start(event->esp_netif);
}

void app_main(void)
{
    // Initialize NVS (required for console IP storage and wifi/eth config)
    ESP_ERROR_CHECK(app_console_nvs_init());

    // Initialize Ethernet driver
    uint8_t eth_port_cnt = 0;
    esp_eth_handle_t *eth_handles;
    ESP_ERROR_CHECK(example_eth_init(&eth_handles, &eth_port_cnt));

    // Initialize TCP/IP network interface aka the esp-netif (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *eth_netifs[eth_port_cnt];
    esp_eth_netif_glue_handle_t eth_netif_glues[eth_port_cnt];

    // Create instance(s) of esp-netif for Ethernet(s)
    if (eth_port_cnt == 1) {
        // Use ESP_NETIF_DEFAULT_ETH when just one Ethernet interface is used and you don't need to modify
        // default esp-netif configuration parameters.
        esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
        eth_netifs[0] = esp_netif_new(&cfg);
        eth_netif_glues[0] = esp_eth_new_netif_glue(eth_handles[0]);
        // Attach Ethernet driver to TCP/IP stack
        ESP_ERROR_CHECK(esp_netif_attach(eth_netifs[0], eth_netif_glues[0]));

        /* --- NVS-based IP config (takes priority over menuconfig static IP) --- */
        esp_netif_ip_info_t ip_info;
        bool use_static = false;

        if (app_console_load_ip(&ip_info, &use_static) == ESP_OK && use_static) {
            // Stored static IP found in NVS
            ESP_ERROR_CHECK(esp_netif_dhcpc_stop(eth_netifs[0]));
            ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netifs[0], &ip_info));
            ESP_LOGI(TAG, "~~~~~~~~~~~");
            ESP_LOGI(TAG, "Static IP loaded from NVS:");
            ESP_LOGI(TAG, "IP:      " IPSTR, IP2STR(&ip_info.ip));
            ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&ip_info.netmask));
            ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&ip_info.gw));
            ESP_LOGI(TAG, "~~~~~~~~~~~");
        }
#if CONFIG_EXAMPLE_USE_STATIC_IP
        else {
            // Fall back to menuconfig static IP when nothing stored in NVS
            ESP_ERROR_CHECK(esp_netif_dhcpc_stop(eth_netifs[0]));
            IP4_ADDR(&ip_info.ip, CONFIG_EXAMPLE_STATIC_IP_ADDR_1, CONFIG_EXAMPLE_STATIC_IP_ADDR_2,
                     CONFIG_EXAMPLE_STATIC_IP_ADDR_3, CONFIG_EXAMPLE_STATIC_IP_ADDR_4);
            IP4_ADDR(&ip_info.gw, CONFIG_EXAMPLE_STATIC_GW_ADDR_1, CONFIG_EXAMPLE_STATIC_GW_ADDR_2,
                     CONFIG_EXAMPLE_STATIC_GW_ADDR_3, CONFIG_EXAMPLE_STATIC_GW_ADDR_4);
            IP4_ADDR(&ip_info.netmask, CONFIG_EXAMPLE_STATIC_NETMASK_1, CONFIG_EXAMPLE_STATIC_NETMASK_2,
                     CONFIG_EXAMPLE_STATIC_NETMASK_3, CONFIG_EXAMPLE_STATIC_NETMASK_4);
            ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netifs[0], &ip_info));
            ESP_LOGI(TAG, "~~~~~~~~~~~");
            ESP_LOGI(TAG, "Static IP from menuconfig:");
            ESP_LOGI(TAG, "IP:      " IPSTR, IP2STR(&ip_info.ip));
            ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&ip_info.netmask));
            ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&ip_info.gw));
            ESP_LOGI(TAG, "~~~~~~~~~~~");
        }
#endif
    } else {
        // Use ESP_NETIF_INHERENT_DEFAULT_ETH when multiple Ethernet interfaces are used and so you need to modify
        // esp-netif configuration parameters for each interface (name, priority, etc.).
        esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
        esp_netif_config_t cfg_spi = {
            .base = &esp_netif_config,
            .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
        };
        char if_key_str[10];
        char if_desc_str[10];
        char num_str[3];
        for (int i = 0; i < eth_port_cnt; i++) {
            itoa(i, num_str, 10);
            strcat(strcpy(if_key_str, "ETH_"), num_str);
            strcat(strcpy(if_desc_str, "eth"), num_str);
            esp_netif_config.if_key = if_key_str;
            esp_netif_config.if_desc = if_desc_str;
            esp_netif_config.route_prio -= i*5;
            eth_netifs[i] = esp_netif_new(&cfg_spi);
            eth_netif_glues[i] = esp_eth_new_netif_glue(eth_handles[0]);
            // Attach Ethernet driver to TCP/IP stack
            ESP_ERROR_CHECK(esp_netif_attach(eth_netifs[i], eth_netif_glues[i]));
        }
    }

    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    // Start Ethernet driver state machine
    for (int i = 0; i < eth_port_cnt; i++) {
        ESP_ERROR_CHECK(esp_eth_start(eth_handles[i]));
    }

    // Start serial console (eth_netifs[0] is always valid here)
    ESP_LOGI(TAG, "Starting serial console (type 'help' for commands)...");
    ESP_ERROR_CHECK(app_console_start(eth_netifs[0]));

#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR
    esp_err_t ret;
    
    // Initialize external trigger (before camera)
#if CONFIG_TRIGGER_ENABLE
    ESP_LOGI(TAG, "Initializing external trigger on GPIO %d...", CONFIG_TRIGGER_GPIO_PIN);
    ret = app_trigger_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize trigger: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "External trigger initialized");
        ESP_LOGI(TAG, "  - Rising edge: Start capture, reset timestamp");
        ESP_LOGI(TAG, "  - Falling edge: Stop capture");
    }
#endif

    // Initialize and start camera
    ESP_LOGI(TAG, "Initializing camera...");
    ret = app_video_main(NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", ret);
    } else {
        // Open the video device
        g_video_cam_fd = app_video_open(EXAMPLE_CAM_DEV_PATH, APP_VIDEO_FMT);
        if (g_video_cam_fd < 0) {
            ESP_LOGE(TAG, "Camera open failed");
        } else {
            ESP_LOGI(TAG, "Camera opened successfully, fd=%d", g_video_cam_fd);
            // Set the video buffer (use auto-allocated buffer via mapping)
            ESP_ERROR_CHECK(app_video_set_bufs(g_video_cam_fd, EXAMPLE_CAM_BUF_NUM, NULL));
            ESP_LOGI(TAG, "Camera buffer size: %lu bytes", app_video_get_buf_size());
            
            // Start the HTTP video stream server
            ESP_LOGI(TAG, "Starting video stream server...");
            ret = app_video_stream_server_start(g_video_cam_fd);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start video stream server: 0x%x", ret);
            } else {
                ESP_LOGI(TAG, "Video stream server started");
                ESP_LOGI(TAG, "Open http://<ESP32-P4-IP>/stream in browser to view the camera");
            }
        }
    }
#endif

#if CONFIG_EXAMPLE_ETH_DEINIT_AFTER_S >= 0
    // For demonstration purposes, wait and then deinit Ethernet network
    vTaskDelay(pdMS_TO_TICKS(CONFIG_EXAMPLE_ETH_DEINIT_AFTER_S * 1000));
    ESP_LOGI(TAG, "stop and deinitialize Ethernet network...");
    // Stop Ethernet driver state machine and destroy netif
    for (int i = 0; i < eth_port_cnt; i++) {
        ESP_ERROR_CHECK(esp_eth_stop(eth_handles[i]));
        ESP_ERROR_CHECK(esp_eth_del_netif_glue(eth_netif_glues[i]));
        esp_netif_destroy(eth_netifs[i]);
    }
    esp_netif_deinit();
    ESP_ERROR_CHECK(example_eth_deinit(eth_handles, eth_port_cnt));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP, got_ip_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler));
    ESP_ERROR_CHECK(esp_event_loop_delete_default());
#endif // EXAMPLE_ETH_DEINIT_AFTER_S > 0
}
