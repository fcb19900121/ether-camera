/*
 * Serial Console – IP Configuration Module
 *
 * Supported commands (type in idf.py monitor / esp-idf monitor):
 *
 *   setip <ip> [netmask] [gateway]
 *       Example:  setip 192.168.1.101
 *                 setip 192.168.1.101 255.255.255.0 192.168.1.1
 *       Sets a static IP, saves it to NVS, and applies it immediately.
 *
 *   dhcp
 *       Switch back to DHCP, clears the stored static config from NVS.
 *
 *   getip
 *       Print current IP configuration and what is stored in NVS.
 *
 *   help
 *       Show this command list.
 */

#include "app_console.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/ip4_addr.h"

static const char *TAG = "console";

/* NVS namespace / keys */
#define NVS_NAMESPACE   "ip_cfg"
#define NVS_KEY_MODE    "mode"      /* "static" | "dhcp" */
#define NVS_KEY_IP      "ip"
#define NVS_KEY_MASK    "mask"
#define NVS_KEY_GW      "gw"

/* Shared netif handle set by app_console_start() */
static esp_netif_t *s_netif = NULL;

/* ------------------------------------------------------------------ */
/*  NVS helpers                                                         */
/* ------------------------------------------------------------------ */

esp_err_t app_console_nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

esp_err_t app_console_load_ip(esp_netif_ip_info_t *ip_info, bool *is_static)
{
    *is_static = false;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return ESP_ERR_NOT_FOUND;   /* namespace doesn't exist yet */
    }

    /* Check mode */
    char mode[8] = {0};
    size_t mode_len = sizeof(mode);
    err = nvs_get_str(h, NVS_KEY_MODE, mode, &mode_len);
    if (err != ESP_OK || strcmp(mode, "static") != 0) {
        nvs_close(h);
        return ESP_ERR_NOT_FOUND;
    }

    /* Read IP, mask, gw as uint32 (stored in network byte order) */
    uint32_t ip = 0, mask = 0, gw = 0;
    if (nvs_get_u32(h, NVS_KEY_IP,   &ip)   != ESP_OK ||
        nvs_get_u32(h, NVS_KEY_MASK, &mask) != ESP_OK ||
        nvs_get_u32(h, NVS_KEY_GW,   &gw)   != ESP_OK) {
        nvs_close(h);
        return ESP_ERR_NOT_FOUND;
    }
    nvs_close(h);

    ip_info->ip.addr      = ip;
    ip_info->netmask.addr = mask;
    ip_info->gw.addr      = gw;
    *is_static = true;
    return ESP_OK;
}

static esp_err_t save_static_ip(const esp_netif_ip_info_t *ip_info)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    nvs_set_str(h, NVS_KEY_MODE, "static");
    nvs_set_u32(h, NVS_KEY_IP,   ip_info->ip.addr);
    nvs_set_u32(h, NVS_KEY_MASK, ip_info->netmask.addr);
    nvs_set_u32(h, NVS_KEY_GW,   ip_info->gw.addr);
    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static esp_err_t save_dhcp_mode(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    nvs_set_str(h, NVS_KEY_MODE, "dhcp");
    /* erase ip/mask/gw – not strictly necessary but keeps NVS clean */
    nvs_erase_key(h, NVS_KEY_IP);
    nvs_erase_key(h, NVS_KEY_MASK);
    nvs_erase_key(h, NVS_KEY_GW);
    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

/* ------------------------------------------------------------------ */
/*  Apply IP to running netif                                           */
/* ------------------------------------------------------------------ */

static esp_err_t apply_static_ip(const esp_netif_ip_info_t *ip_info)
{
    if (s_netif == NULL) return ESP_ERR_INVALID_STATE;

    /* Must stop DHCP client before setting a static IP */
    esp_netif_dhcpc_stop(s_netif);

    esp_err_t err = esp_netif_set_ip_info(s_netif, ip_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_set_ip_info failed: %s", esp_err_to_name(err));
    }
    return err;
}

static esp_err_t apply_dhcp(void)
{
    if (s_netif == NULL) return ESP_ERR_INVALID_STATE;

    /* Clear any static IP first */
    esp_netif_ip_info_t zero = {0};
    esp_netif_set_ip_info(s_netif, &zero);

    esp_err_t err = esp_netif_dhcpc_start(s_netif);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
        ESP_LOGE(TAG, "dhcpc_start failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Command handlers                                                    */
/* ------------------------------------------------------------------ */

static void print_help(void)
{
    printf("Commands:\n");
    printf("  setip <ip> [netmask] [gateway]\n");
    printf("      Set static IP address.\n");
    printf("      netmask defaults to 255.255.255.0\n");
    printf("      gateway defaults to <same /24>.1\n");
    printf("      Example: setip 192.168.1.101\n");
    printf("               setip 192.168.1.101 255.255.255.0 192.168.1.1\n");
    printf("  dhcp\n");
    printf("      Switch to DHCP mode and clear stored static IP.\n");
    printf("  getip\n");
    printf("      Show current runtime IP and stored NVS config.\n");
    printf("  help\n");
    printf("      Show this help.\n");
}

static void handle_setip(int argc, char **argv)
{
    /* argv[0] = "setip", argv[1] = ip, argv[2] = mask (opt), argv[3] = gw (opt) */
    if (argc < 2) {
        printf("Usage: setip <ip> [netmask] [gateway]\n");
        return;
    }

    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));

    if (!ip4addr_aton(argv[1], (ip4_addr_t *)&ip_info.ip)) {
        printf("ERROR: invalid IP address: %s\n", argv[1]);
        return;
    }

    if (argc >= 3) {
        if (!ip4addr_aton(argv[2], (ip4_addr_t *)&ip_info.netmask)) {
            printf("ERROR: invalid netmask: %s\n", argv[2]);
            return;
        }
    } else {
        IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    }

    if (argc >= 4) {
        if (!ip4addr_aton(argv[3], (ip4_addr_t *)&ip_info.gw)) {
            printf("ERROR: invalid gateway: %s\n", argv[3]);
            return;
        }
    } else {
        /* derive gateway: same /24, host = 1 */
        uint32_t ip_h = ntohl(ip_info.ip.addr);
        ip_info.gw.addr = htonl((ip_h & 0xFFFFFF00u) | 1u);
    }

    esp_err_t err = save_static_ip(&ip_info);
    if (err != ESP_OK) {
        printf("ERROR: NVS save failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = apply_static_ip(&ip_info);
    if (err != ESP_OK) {
        printf("ERROR: apply IP failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf("Static IP set and saved:\n");
    printf("  IP:      " IPSTR "\n", IP2STR(&ip_info.ip));
    printf("  Netmask: " IPSTR "\n", IP2STR(&ip_info.netmask));
    printf("  Gateway: " IPSTR "\n", IP2STR(&ip_info.gw));
    printf("Config takes effect immediately and persists across reboots.\n");
}

static void handle_dhcp(void)
{
    esp_err_t err = save_dhcp_mode();
    if (err != ESP_OK) {
        printf("ERROR: NVS save failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = apply_dhcp();
    if (err != ESP_OK) {
        printf("ERROR: start DHCP failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("DHCP mode enabled and saved.\n");
}

static void handle_getip(void)
{
    if (s_netif) {
        esp_netif_ip_info_t cur;
        esp_netif_get_ip_info(s_netif, &cur);
        printf("Current runtime IP:\n");
        printf("  IP:      " IPSTR "\n", IP2STR(&cur.ip));
        printf("  Netmask: " IPSTR "\n", IP2STR(&cur.netmask));
        printf("  Gateway: " IPSTR "\n", IP2STR(&cur.gw));
    } else {
        printf("(no netif registered)\n");
    }

    esp_netif_ip_info_t stored;
    bool is_static = false;
    if (app_console_load_ip(&stored, &is_static) == ESP_OK && is_static) {
        printf("Stored (NVS) static IP:\n");
        printf("  IP:      " IPSTR "\n", IP2STR(&stored.ip));
        printf("  Netmask: " IPSTR "\n", IP2STR(&stored.netmask));
        printf("  Gateway: " IPSTR "\n", IP2STR(&stored.gw));
    } else {
        printf("Stored mode: DHCP (or not configured)\n");
    }
}

/* ------------------------------------------------------------------ */
/*  REPL task                                                           */
/* ------------------------------------------------------------------ */

#define CONSOLE_LINE_MAX 128
#define CONSOLE_ARGC_MAX 8

static void console_task(void *arg)
{
    char line[CONSOLE_LINE_MAX];

    printf("\neth-cam console ready. Type 'help' for commands.\n");
    printf("eth-cam> ");
    fflush(stdout);

    while (1) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Strip trailing newline / carriage-return */
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }

        if (len == 0) {
            printf("eth-cam> ");
            fflush(stdout);
            continue;
        }

        /* Tokenise */
        char *argv[CONSOLE_ARGC_MAX];
        int   argc = 0;
        char *tok  = strtok(line, " \t");
        while (tok && argc < CONSOLE_ARGC_MAX) {
            argv[argc++] = tok;
            tok = strtok(NULL, " \t");
        }

        if (argc == 0) {
            /* empty after stripping whitespace */
        } else if (strcmp(argv[0], "setip") == 0) {
            handle_setip(argc, argv);
        } else if (strcmp(argv[0], "dhcp") == 0) {
            handle_dhcp();
        } else if (strcmp(argv[0], "getip") == 0) {
            handle_getip();
        } else if (strcmp(argv[0], "help") == 0) {
            print_help();
        } else {
            printf("Unknown command: '%s'. Type 'help' for a list.\n", argv[0]);
        }

        printf("eth-cam> ");
        fflush(stdout);
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t app_console_start(esp_netif_t *netif)
{
    s_netif = netif;

    BaseType_t ret = xTaskCreate(console_task, "console", 4096, NULL,
                                 tskIDLE_PRIORITY + 1, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create console task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
