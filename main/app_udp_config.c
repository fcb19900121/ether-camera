/*
 * UDP Configuration Service Implementation
 *
 * Listens on UDP_CONFIG_PORT for JSON commands and broadcasts an announce
 * packet every UDP_ANNOUNCE_INTERVAL_S seconds so the host can discover
 * the device's current IP and configuration without knowing it in advance.
 *
 * All JSON parsing is done with a minimal hand-rolled parser to avoid
 * pulling in cJSON (saves ~15 KB of flash).
 */
#include "app_udp_config.h"
#include "app_video_stream.h"
#include "app_video.h"
#include "app_console.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/ip4_addr.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs.h"

static const char *TAG = "udp_cfg";

#define NVS_NAMESPACE_UDP_CFG "udp_cfg"
#define NVS_KEY_JPEG_QUALITY  "jpeg_q"

#define RX_BUF_SIZE   512
#define TX_BUF_SIZE   384

static esp_netif_t *s_netif = NULL;
static volatile bool s_running = false;
static TaskHandle_t s_listen_task  = NULL;
static TaskHandle_t s_announce_task = NULL;
static int s_sock = -1;  /* shared UDP socket (bind + sendto) */
static volatile bool s_pending_reset = false;

static esp_err_t udp_cfg_save_jpeg_quality(int quality)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE_UDP_CFG, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_i32(h, NVS_KEY_JPEG_QUALITY, quality);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

static esp_err_t udp_cfg_load_jpeg_quality(int *quality)
{
    if (quality == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE_UDP_CFG, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return err;
    }

    int32_t q = 0;
    err = nvs_get_i32(h, NVS_KEY_JPEG_QUALITY, &q);
    nvs_close(h);
    if (err != ESP_OK) {
        return err;
    }

    if (q < 1 || q > 100) {
        return ESP_ERR_INVALID_SIZE;
    }

    *quality = (int)q;
    return ESP_OK;
}

static const char *trigger_mode_str(app_trigger_mode_t mode)
{
    return mode == APP_TRIGGER_MODE_UDP ? "udp" : "gpio";
}

/* ------------------------------------------------------------------ */
/*  Minimal JSON helpers                                                */
/* ------------------------------------------------------------------ */

/**
 * Extract string value for a given "key":"value" pair.
 * Returns pointer to start of value in src, writes length to out_len.
 * Returns NULL if key not found.
 */
static const char *json_get_str(const char *src, const char *key, size_t *out_len)
{
    /* search for "key" */
    char needle[48];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = strstr(src, needle);
    if (!p) return NULL;
    p += strlen(needle);
    /* skip : and whitespace */
    while (*p == ' ' || *p == ':') p++;
    if (*p != '"') return NULL;
    p++; /* skip opening quote */
    const char *start = p;
    while (*p && *p != '"') p++;
    *out_len = (size_t)(p - start);
    return start;
}

/**
 * Extract integer value for a given "key":number pair.
 * Returns 0 and sets *found=false if not present.
 */
static int json_get_int(const char *src, const char *key, bool *found)
{
    char needle[48];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = strstr(src, needle);
    if (!p) { *found = false; return 0; }
    p += strlen(needle);
    while (*p == ' ' || *p == ':') p++;
    *found = true;
    return (int)strtol(p, NULL, 10);
}

/* ------------------------------------------------------------------ */
/*  NVS / netif helpers (delegates to app_console)                      */
/* ------------------------------------------------------------------ */

static esp_err_t udp_set_static_ip(const char *ip_str, const char *mask_str, const char *gw_str)
{
    esp_netif_ip_info_t info;
    memset(&info, 0, sizeof(info));

    if (!ip4addr_aton(ip_str, (ip4_addr_t *)&info.ip)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (mask_str && strlen(mask_str) > 0) {
        if (!ip4addr_aton(mask_str, (ip4_addr_t *)&info.netmask)) {
            return ESP_ERR_INVALID_ARG;
        }
    } else {
        IP4_ADDR(&info.netmask, 255, 255, 255, 0);
    }
    if (gw_str && strlen(gw_str) > 0) {
        if (!ip4addr_aton(gw_str, (ip4_addr_t *)&info.gw)) {
            return ESP_ERR_INVALID_ARG;
        }
    } else {
        /* default gateway: same /24, host = 1 */
        uint32_t ip_h = ntohl(info.ip.addr);
        info.gw.addr = htonl((ip_h & 0xFFFFFF00u) | 1u);
    }

    /* Save to NVS (app_console internal helper not exposed; replicate inline) */
    /* We reuse app_console's NVS namespace via the public load API and write
     * directly — the same NVS keys are used so the serial console stays in sync. */
    esp_err_t err = app_console_save_ip(&info);
    if (err != ESP_OK) return err;

    /* Apply immediately */
    esp_netif_dhcpc_stop(s_netif);
    return esp_netif_set_ip_info(s_netif, &info);
}

static esp_err_t udp_set_dhcp(void)
{
    esp_err_t err = app_console_save_dhcp();
    if (err != ESP_OK) return err;

    esp_netif_ip_info_t zero = {0};
    esp_netif_set_ip_info(s_netif, &zero);
    err = esp_netif_dhcpc_start(s_netif);
    if (err == ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) err = ESP_OK;
    return err;
}

/* ------------------------------------------------------------------ */
/*  Build announce / reply JSON                                         */
/* ------------------------------------------------------------------ */

static int build_announce(char *buf, size_t len)
{
    esp_netif_ip_info_t ip;
    esp_netif_get_ip_info(s_netif, &ip);
    char ip_str[16];
    snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip.ip));

    return snprintf(buf, len,
        "{\"type\":\"announce\","
        "\"ip\":\"%s\","
        "\"port\":%d,"
        "\"trigger_mode\":\"%s\","
        "\"jpeg_quality\":%d,"
        "\"skip_frames\":%lu,"
        "\"width\":%lu,"
        "\"height\":%lu}",
        ip_str,
        UDP_CONFIG_PORT,
        trigger_mode_str(app_video_stream_get_trigger_mode()),
        app_video_stream_get_jpeg_quality(),
        (unsigned long)app_video_stream_get_skip_frames(),
        (unsigned long)app_video_get_width(),
        (unsigned long)app_video_get_height());
}

/* ------------------------------------------------------------------ */
/*  Command dispatcher                                                  */
/* ------------------------------------------------------------------ */

static int dispatch_command(const char *rx, char *reply, size_t reply_len)
{
    /* extract "cmd" field */
    size_t cmd_len = 0;
    const char *cmd = json_get_str(rx, "cmd", &cmd_len);
    if (!cmd || cmd_len == 0) {
        // Not a control request packet; do not reply to avoid echo storms
        // when multiple devices share the same UDP port.
        return 0;
    }

    /* ---- set_skip_frames ---- */
    if (cmd_len == 15 && memcmp(cmd, "set_skip_frames", 15) == 0) {
        bool found = false;
        int n = json_get_int(rx, "value", &found);
        if (!found || n < 0 || n > 255) {
            return snprintf(reply, reply_len,
                            "{\"ok\":false,\"error\":\"value must be 0-255\"}");
        }
        app_video_stream_set_skip_frames((uint32_t)n);
        ESP_LOGI(TAG, "skip_frames set to %d via UDP", n);
        return snprintf(reply, reply_len,
                        "{\"ok\":true,\"skip_frames\":%d}", n);
    }

    /* ---- set_trigger_mode ---- */
    if (cmd_len == 16 && memcmp(cmd, "set_trigger_mode", 16) == 0) {
        size_t mode_len = 0;
        const char *mode = json_get_str(rx, "value", &mode_len);
        if (!mode) {
            return snprintf(reply, reply_len,
                            "{\"ok\":false,\"error\":\"missing value\"}");
        }

        if (mode_len == 3 && memcmp(mode, "udp", 3) == 0) {
            app_video_stream_set_trigger_mode(APP_TRIGGER_MODE_UDP);
            return snprintf(reply, reply_len,
                            "{\"ok\":true,\"trigger_mode\":\"udp\"}");
        }
        if (mode_len == 4 && memcmp(mode, "gpio", 4) == 0) {
            app_video_stream_set_trigger_mode(APP_TRIGGER_MODE_GPIO);
            return snprintf(reply, reply_len,
                            "{\"ok\":true,\"trigger_mode\":\"gpio\"}");
        }

        return snprintf(reply, reply_len,
                        "{\"ok\":false,\"error\":\"value must be udp/gpio\"}");
    }

    /* ---- trigger_start ---- */
    if (cmd_len == 13 && memcmp(cmd, "trigger_start", 13) == 0) {
        esp_err_t err = app_video_stream_udp_trigger_start();
        if (err != ESP_OK) {
            if (err == ESP_ERR_INVALID_STATE) {
                return snprintf(reply, reply_len,
                                "{\"ok\":false,\"error\":\"trigger subsystem not ready\"}");
            }
            return snprintf(reply, reply_len,
                            "{\"ok\":false,\"error\":\"%s\"}", esp_err_to_name(err));
        }
        ESP_LOGI(TAG, "UDP trigger started");
        return snprintf(reply, reply_len,
                        "{\"ok\":true,\"trigger\":\"started\"}");
    }

    /* ---- trigger_stop ---- */
    if (cmd_len == 12 && memcmp(cmd, "trigger_stop", 12) == 0) {
        esp_err_t err = app_video_stream_udp_trigger_stop();
        if (err != ESP_OK) {
            if (err == ESP_ERR_INVALID_STATE) {
                return snprintf(reply, reply_len,
                                "{\"ok\":false,\"error\":\"trigger subsystem not ready\"}");
            }
            return snprintf(reply, reply_len,
                            "{\"ok\":false,\"error\":\"%s\"}", esp_err_to_name(err));
        }
        ESP_LOGI(TAG, "UDP trigger stopped");
        return snprintf(reply, reply_len,
                        "{\"ok\":true,\"trigger\":\"stopped\"}");
    }

    /* ---- get_trigger_stats ---- */
    if (cmd_len == 17 && memcmp(cmd, "get_trigger_stats", 17) == 0) {
        app_trigger_stats_t stats = {0};
        app_video_stream_get_trigger_stats(&stats);
        return snprintf(reply, reply_len,
                        "{\"ok\":true,\"active\":%s,\"frame_count\":%lu,\"dropped_frames_est\":%lu,"
                        "\"first_frame_latency_us\":%lld,\"avg_frame_interval_us\":%lld,\"last_frame_interval_us\":%lld}",
                        stats.active ? "true" : "false",
                        (unsigned long)stats.frame_count,
                        (unsigned long)stats.dropped_frames_est,
                        (long long)stats.first_frame_latency_us,
                        (long long)stats.avg_frame_interval_us,
                        (long long)stats.last_frame_interval_us);
    }

    /* ---- reset ---- */
    if (cmd_len == 5 && memcmp(cmd, "reset", 5) == 0) {
        s_pending_reset = true;
        return snprintf(reply, reply_len,
                        "{\"ok\":true,\"action\":\"reset\",\"delay_ms\":200}");
    }

    /* ---- set_quality ---- */
    if (cmd_len == 11 && memcmp(cmd, "set_quality", 11) == 0) {
        bool found = false;
        int q = json_get_int(rx, "value", &found);
        if (!found || q < 1 || q > 100) {
            return snprintf(reply, reply_len,
                            "{\"ok\":false,\"error\":\"value must be 1-100\"}");
        }

        esp_err_t save_err = udp_cfg_save_jpeg_quality(q);
        if (save_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to persist JPEG quality to NVS: %s", esp_err_to_name(save_err));
        }

        app_video_stream_set_jpeg_quality(q);
        ESP_LOGI(TAG, "JPEG quality set to %d via UDP", q);
        return snprintf(reply, reply_len,
                        "{\"ok\":true,\"jpeg_quality\":%d}", q);
    }

    /* ---- setip ---- */
    if (cmd_len == 5 && memcmp(cmd, "setip", 5) == 0) {
        size_t ip_len = 0, mask_len = 0, gw_len = 0;
        const char *ip_v   = json_get_str(rx, "ip",      &ip_len);
        const char *mask_v = json_get_str(rx, "netmask", &mask_len);
        const char *gw_v   = json_get_str(rx, "gw",      &gw_len);

        if (!ip_v || ip_len == 0 || ip_len >= 16) {
            return snprintf(reply, reply_len,
                            "{\"ok\":false,\"error\":\"invalid ip\"}");
        }
        char ip_str[16] = {0}, mask_str[16] = {0}, gw_str[16] = {0};
        memcpy(ip_str,   ip_v,   ip_len   < 15 ? ip_len   : 15);
        if (mask_v && mask_len > 0 && mask_len < 16)
            memcpy(mask_str, mask_v, mask_len);
        if (gw_v && gw_len > 0 && gw_len < 16)
            memcpy(gw_str, gw_v, gw_len);

        esp_err_t err = udp_set_static_ip(ip_str,
                                           mask_len ? mask_str : NULL,
                                           gw_len   ? gw_str   : NULL);
        if (err != ESP_OK) {
            return snprintf(reply, reply_len,
                            "{\"ok\":false,\"error\":\"%s\"}", esp_err_to_name(err));
        }
        ESP_LOGI(TAG, "IP set to %s via UDP", ip_str);
        return snprintf(reply, reply_len,
                        "{\"ok\":true,\"ip\":\"%s\"}", ip_str);
    }

    /* ---- dhcp ---- */
    if (cmd_len == 4 && memcmp(cmd, "dhcp", 4) == 0) {
        esp_err_t err = udp_set_dhcp();
        if (err != ESP_OK) {
            return snprintf(reply, reply_len,
                            "{\"ok\":false,\"error\":\"%s\"}", esp_err_to_name(err));
        }
        ESP_LOGI(TAG, "Switched to DHCP via UDP");
        return snprintf(reply, reply_len,
                        "{\"ok\":true,\"mode\":\"dhcp\"}");
    }

    /* ---- get_config ---- */
    if (cmd_len == 10 && memcmp(cmd, "get_config", 10) == 0) {
        esp_netif_ip_info_t ip;
        esp_netif_get_ip_info(s_netif, &ip);
        char ip_str[16];
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip.ip));
        return snprintf(reply, reply_len,
            "{\"ok\":true,"
            "\"ip\":\"%s\","
            "\"trigger_mode\":\"%s\","
            "\"jpeg_quality\":%d,"
            "\"skip_frames\":%lu,"
            "\"width\":%lu,"
            "\"height\":%lu}",
            ip_str,
            trigger_mode_str(app_video_stream_get_trigger_mode()),
            app_video_stream_get_jpeg_quality(),
            (unsigned long)app_video_stream_get_skip_frames(),
            (unsigned long)app_video_get_width(),
            (unsigned long)app_video_get_height());
    }

    return snprintf(reply, reply_len,
                    "{\"ok\":false,\"error\":\"unknown cmd\"}");
}

/* ------------------------------------------------------------------ */
/*  Listener task                                                       */
/* ------------------------------------------------------------------ */

static void listen_task(void *arg)
{
    char *rx_buf = malloc(RX_BUF_SIZE);
    char *tx_buf = malloc(TX_BUF_SIZE);
    if (!rx_buf || !tx_buf) {
        ESP_LOGE(TAG, "OOM allocating buffers");
        free(rx_buf); free(tx_buf);
        vTaskDelete(NULL);
        return;
    }

    while (s_running) {
        struct sockaddr_in sender;
        socklen_t sender_len = sizeof(sender);
        memset(rx_buf, 0, RX_BUF_SIZE);

        int n = recvfrom(s_sock, rx_buf, RX_BUF_SIZE - 1, 0,
                         (struct sockaddr *)&sender, &sender_len);
        if (n <= 0) {
            if (s_running) vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        rx_buf[n] = '\0';
        ESP_LOGI(TAG, "RX %d bytes from " IPSTR ":%d : %s",
                 n, IP2STR((esp_ip4_addr_t*)&sender.sin_addr.s_addr), ntohs(sender.sin_port), rx_buf);

        // Ignore non-command packets (e.g. peer announce packets/replies on same UDP port).
        // Only packets containing a "cmd" key are treated as control requests.
        if (strstr(rx_buf, "\"cmd\"") == NULL) {
            ESP_LOGD(TAG, "Ignore non-cmd UDP packet");
            continue;
        }

        int reply_len = dispatch_command(rx_buf, tx_buf, TX_BUF_SIZE);
        if (reply_len > 0) {
            sendto(s_sock, tx_buf, reply_len, 0,
                   (struct sockaddr *)&sender, sender_len);
            ESP_LOGI(TAG, "TX reply: %s", tx_buf);
        }

        if (s_pending_reset) {
            ESP_LOGW(TAG, "Reboot requested via UDP command");
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_restart();
        }
    }

    free(rx_buf);
    free(tx_buf);
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/*  Broadcast announce task                                             */
/* ------------------------------------------------------------------ */

static void announce_task(void *arg)
{
    char buf[TX_BUF_SIZE];

    /* Build broadcast destination: 255.255.255.255 : UDP_CONFIG_PORT */
    struct sockaddr_in bcast;
    memset(&bcast, 0, sizeof(bcast));
    bcast.sin_family      = AF_INET;
    bcast.sin_port        = htons(UDP_CONFIG_PORT);
    bcast.sin_addr.s_addr = INADDR_BROADCAST;

    /* Enable broadcast on the socket */
    int on = 1;
    setsockopt(s_sock, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));

    while (s_running) {
        int len = build_announce(buf, sizeof(buf));
        if (len > 0) {
            int sent = sendto(s_sock, buf, len, 0,
                              (struct sockaddr *)&bcast, sizeof(bcast));
            if (sent < 0) {
                ESP_LOGW(TAG, "Broadcast sendto failed: errno %d", errno);
            } else {
                ESP_LOGD(TAG, "Announce: %s", buf);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(UDP_ANNOUNCE_INTERVAL_S * 1000));
    }

    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t app_udp_config_start(esp_netif_t *netif)
{
    if (s_running) return ESP_OK;
    s_netif = netif;

    // Restore persisted JPEG quality (if configured previously).
    int persisted_q = 0;
    if (udp_cfg_load_jpeg_quality(&persisted_q) == ESP_OK) {
        app_video_stream_set_jpeg_quality(persisted_q);
        ESP_LOGI(TAG, "Restored JPEG quality from NVS: %d", persisted_q);
    }

    /* Create a single UDP socket for both RX and TX */
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        return ESP_FAIL;
    }

    /* Allow address reuse */
    int reuse = 1;
    setsockopt(s_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    /* Bind to the config port */
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(UDP_CONFIG_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(s_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: errno %d", errno);
        close(s_sock);
        s_sock = -1;
        return ESP_FAIL;
    }

    /* Set receive timeout so the listen task can check s_running */
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(s_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    s_running = true;

    if (xTaskCreate(listen_task, "udp_listen", 4096, NULL, 5, &s_listen_task) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create listen task");
        s_running = false;
        close(s_sock); s_sock = -1;
        return ESP_FAIL;
    }
    if (xTaskCreate(announce_task, "udp_announce", 3072, NULL, 4, &s_announce_task) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create announce task");
        s_running = false;
        /* listen task will exit on its own */
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "UDP config service started on port %d", UDP_CONFIG_PORT);
    ESP_LOGI(TAG, "  Broadcasting announce every %ds", UDP_ANNOUNCE_INTERVAL_S);
    return ESP_OK;
}

void app_udp_config_stop(void)
{
    s_running = false;
    if (s_sock >= 0) {
        close(s_sock);
        s_sock = -1;
    }
    /* Tasks will exit on next loop iteration */
    s_listen_task   = NULL;
    s_announce_task = NULL;
}
