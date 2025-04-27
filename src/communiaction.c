#include "communication.h"
#include "config.h"
#include "wifi_handler.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <string.h>
#include <sys/fcntl.h>

#define TAG "UDP"

static int sock = -1;

esp_err_t app_udp_init(void)
{
    struct sockaddr_in dest_addr;
    
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return ESP_FAIL;
    }
    
    // Set socket to non-blocking
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    
    // Bind to any address on the configured port
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);
    
    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        sock = -1;
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "UDP socket initialized, port %d", UDP_PORT);
    return ESP_OK;
}

esp_err_t app_udp_send(const char *dest_ip, const void *message, size_t len)
{
    if (sock < 0) {
        return ESP_ERR_INVALID_STATE;
    }
    
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(dest_ip);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);
    
    int err = sendto(sock, message, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Error sending data: errno %d", errno);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Sent %d bytes to %s", len, dest_ip);
    return ESP_OK;
}

esp_err_t udp_receive(void *buffer, size_t max_len, char *sender_ip, size_t ip_len, size_t *len)
{
    if (sock < 0) {
        return ESP_ERR_INVALID_STATE;
    }
    
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);
    
    int rx_len = recvfrom(sock, buffer, max_len - 1, 0, (struct sockaddr *)&source_addr, &socklen);
    
    if (rx_len < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No data available (non-blocking socket)
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGE(TAG, "Error receiving data: errno %d", errno);
        return ESP_FAIL;
    }
    
    // Null-terminate the received data
    ((char *)buffer)[rx_len] = 0;
    
    // Set output parameters
    *len = rx_len;
    
    if (sender_ip && ip_len > 0) {
        inet_ntop(AF_INET, &source_addr.sin_addr, sender_ip, ip_len);
    }
    
    ESP_LOGI(TAG, "Received %d bytes from %s", rx_len, sender_ip);
    return ESP_OK;
}

esp_err_t udp_close(void)
{
    if (sock >= 0) {
        close(sock);
        sock = -1;
        ESP_LOGI(TAG, "UDP socket closed");
    }
    return ESP_OK;
}

// Task for receiver ESP32 to handle incoming UDP messages
void udp_receiver_task(void *pvParameters)
{
    char rx_buffer[128];
    char sender_ip[16];
    size_t len;
    
    // Get local IP address
    char local_ip[16];
    esp_err_t ret = get_local_ip(local_ip, sizeof(local_ip));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get local IP address");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "UDP Receiver started, local IP: %s", local_ip);
    
    // Initialize UDP
    if (app_udp_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize UDP");
        vTaskDelete(NULL);
        return;
    }
    
    while (1) {
        // Check for incoming messages
        ret = udp_receive(rx_buffer, sizeof(rx_buffer), sender_ip, sizeof(sender_ip), &len);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Received message from Gateway (%s): %s", sender_ip, rx_buffer);
            
            // Process the forwarded message from the Gateway ESP32
            // This is where you would add your application-specific logic
            
            // Example: Send an acknowledgment back to the gateway
            char response[64];
            snprintf(response, sizeof(response), "ACK from %s: received your message", local_ip);
            app_udp_send(sender_ip, response, strlen(response));
        }
        
        // Short delay
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    // This point is never reached, but we clean up anyway
    udp_close();
    vTaskDelete(NULL);
}