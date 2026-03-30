#pragma once

#include "esp_err.h"
#include "esp_netif.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize NVS flash (must be called before any NVS operation).
 *        Call once in app_main before esp_netif_init().
 */
esp_err_t app_console_nvs_init(void);

/**
 * @brief Load IP configuration from NVS.
 *
 * @param[out] ip_info   Filled with stored IP/netmask/gw on success.
 * @param[out] is_static Set to true if a stored static IP was found.
 * @return ESP_OK if a stored config was found and loaded, ESP_ERR_NOT_FOUND otherwise.
 */
esp_err_t app_console_load_ip(esp_netif_ip_info_t *ip_info, bool *is_static);

/**
 * @brief Start the serial console REPL task.
 *        Registers "setip", "dhcp", "getip" commands.
 *
 * @param netif  The esp-netif handle used to apply the IP change at runtime.
 */
esp_err_t app_console_start(esp_netif_t *netif);

#ifdef __cplusplus
}
#endif
