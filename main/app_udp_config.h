/*
 * UDP Configuration Service
 *
 * On IP acquisition the device broadcasts an announcement packet every N seconds:
 *   {"type":"announce","ip":"192.168.1.103","port":4210,
 *    "trigger_mode":"gpio|udp","jpeg_quality":60,"skip_frames":0,
 *    "width":1280,"height":720}
 *
 * The host sends a JSON command to UDP port 4210 and receives a JSON reply.
 *
 * Supported commands:
 *
 *   Set static IP:
 *     {"cmd":"setip","ip":"192.168.1.103","netmask":"255.255.255.0","gw":"192.168.1.1"}
 *     Reply: {"ok":true,"ip":"192.168.1.103"}
 *
 *   Switch to DHCP:
 *     {"cmd":"dhcp"}
 *     Reply: {"ok":true,"mode":"dhcp"}
 *
 *   Set JPEG quality (1-100):
 *     {"cmd":"set_quality","value":75}
 *     Reply: {"ok":true,"jpeg_quality":75}
 *
 *   Set frames to skip after trigger (0-255):
 *     {"cmd":"set_skip_frames","value":2}
 *     Reply: {"ok":true,"skip_frames":2}

 *   Set trigger mode:
 *     {"cmd":"set_trigger_mode","value":"gpio|udp"}
 *     Reply: {"ok":true,"trigger_mode":"gpio|udp"}

 *   Start/stop UDP trigger (only valid in udp mode):
 *     {"cmd":"trigger_start"}
 *     {"cmd":"trigger_stop"}

 *   Query trigger stats:
 *     {"cmd":"get_trigger_stats"}
 *     Reply: {"ok":true,"active":true,"frame_count":123,...}

 *   Reboot device:
 *     {"cmd":"reset"}
 *     Reply: {"ok":true,"action":"reset","delay_ms":200}
 *
 *   Query current config:
 *     {"cmd":"get_config"}
 *     Reply: {"ok":true,"ip":"...","trigger_mode":"gpio|udp",
 *             "jpeg_quality":60,"skip_frames":0,
 *             "width":1280,"height":720}
 */
#pragma once

#include "esp_err.h"
#include "esp_netif.h"

#ifdef __cplusplus
extern "C" {
#endif

/** UDP port the device listens on and that is announced in broadcasts */
#define UDP_CONFIG_PORT  4210

/** Broadcast announcement interval (seconds) */
#define UDP_ANNOUNCE_INTERVAL_S  5

/**
 * @brief Start the UDP configuration service.
 *
 * Must be called after the network interface has an IP address.
 * Starts:
 *   - A listener task (port UDP_CONFIG_PORT) that handles JSON commands.
 *   - A broadcaster task that sends periodic announce packets.
 *
 * @param netif  The active esp-netif handle (used for IP reads/writes).
 * @return ESP_OK on success.
 */
esp_err_t app_udp_config_start(esp_netif_t *netif);

/**
 * @brief Stop the UDP configuration service and free resources.
 */
void app_udp_config_stop(void);

#ifdef __cplusplus
}
#endif
