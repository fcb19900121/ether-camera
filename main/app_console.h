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

/**
 * @brief Persist a static IP configuration to NVS.
 *
 * Does NOT apply the IP to the running netif — call esp_netif_set_ip_info()
 * separately if you also need to apply it at runtime.
 *
 * @param ip_info  IP / netmask / gateway to save.
 * @return ESP_OK on success.
 */
esp_err_t app_console_save_ip(const esp_netif_ip_info_t *ip_info);

/**
 * @brief Persist DHCP mode to NVS (clears any stored static IP).
 *
 * @return ESP_OK on success.
 */
esp_err_t app_console_save_dhcp(void);

#ifdef __cplusplus
}
#endif
