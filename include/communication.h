#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "esp_err.h"

/**
 * @brief Initialize UDP socket for communication
 * 
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t app_udp_init(void);

/**
 * @brief Send a message via UDP
 * 
 * @param dest_ip Destination IP address
 * @param message Message to send
 * @param len Length of the message
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t app_udp_send(const char *dest_ip, const void *message, size_t len);

/**
 * @brief Receive a message via UDP (non-blocking)
 * 
 * @param buffer Buffer to store the received message
 * @param max_len Maximum length of the buffer
 * @param sender_ip Buffer to store the sender's IP address
 * @param ip_len Maximum length of the IP address buffer
 * @param len Length of the received message (output)
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_FOUND if no message is available, otherwise an error code
 */
esp_err_t udp_receive(void *buffer, size_t max_len, char *sender_ip, size_t ip_len, size_t *len);

/**
 * @brief Close UDP socket
 * 
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t udp_close(void);

/**
 * @brief Task for handling UDP communication as a receiver only
 * Listens for incoming UDP messages and processes them
 * 
 * @param pvParameters Task parameters
 */
void udp_receiver_task(void *pvParameters);

#endif // COMMUNICATION_H