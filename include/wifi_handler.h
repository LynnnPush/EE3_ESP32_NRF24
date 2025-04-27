#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include "esp_err.h"

/**
 * @brief Initialize Wi-Fi in station mode
 * 
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t wifi_init_sta(void);

/**
 * @brief Connect to the configured Wi-Fi network
 * 
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t wifi_connect(void);

/**
 * @brief Get the local IP address of the device
 * 
 * @param ip_str Buffer to store the IP address string
 * @param max_len Maximum length of the buffer
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t get_local_ip(char *ip_str, size_t max_len);

#endif // WIFI_HANDLER_H