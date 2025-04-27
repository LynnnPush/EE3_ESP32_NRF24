/**
 * @title ESP32 Gateway: nRF24L01 to Wi-Fi Forwarder
 * @file main.c
 * @author Shanghong Lin (BUILD1)
 * @original Feiyang Zheng (GAME2)
 * @AI Use Claude3.7 Sonnet for reformatting the code
 * @version 0.1
 * @date 2025-04-27
 * 
 * @brief This application integrates nRF24L01 reception with Wi-Fi forwarding.
 *        It can be configured as either a Gateway (receives from nRF24L01 and forwards via Wi-Fi)
 *        or a Receiver (receives Wi-Fi messages only).
 */

 #include <stdio.h>
 #include <string.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "esp_system.h"
 #include "esp_wifi.h"
 #include "esp_log.h"
 #include "nvs_flash.h"
 
 #include "wifi_handler.h"
 #include "communication.h"
 #include "nrf24_inte.h"
 #include "config.h"
 
 #define TAG "MAIN"
 
 // Function prototypes
 void gateway_task(void *pvParameters);
 
 void app_main(void) {
     // Initialize NVS
     esp_err_t ret = nvs_flash_init();
     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
         ESP_ERROR_CHECK(nvs_flash_erase());
         ret = nvs_flash_init();
     }
     ESP_ERROR_CHECK(ret);
     
     ESP_LOGI(TAG, "ESP32 nRF24L01 to Wi-Fi Gateway");
     
     // Initialize Wi-Fi
     ESP_LOGI(TAG, "Initializing Wi-Fi in station mode...");
     ret = wifi_init_sta();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize Wi-Fi: %s", esp_err_to_name(ret));
         return;
     }
     
     // Connect to Wi-Fi
     ESP_LOGI(TAG, "Connecting to Wi-Fi network %s...", WIFI_SSID);
     ret = wifi_connect();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to connect to Wi-Fi: %s", esp_err_to_name(ret));
         return;
     }
     
     // Get and print our IP address
     char local_ip[16];
     if (get_local_ip(local_ip, sizeof(local_ip)) == ESP_OK) {
         ESP_LOGI(TAG, "Device IP address: %s", local_ip);
     }
     
     // Wait a bit for network to stabilize
     vTaskDelay(1000 / portTICK_PERIOD_MS);
     
 #if DEVICE_MODE == DEVICE_MODE_GATEWAY
     ESP_LOGI(TAG, "Running in GATEWAY mode (nRF24L01 to Wi-Fi forwarder)");
     
    // Initialize nRF24L01 in receiver mode
    #if USE_PIN_CONFIG_BUILD1
     nrf_pins_t pins = PIN_CONFIG_BUILD1;  // Declaration and initialization together
    #else
        nrf_pins_t pins = PIN_CONFIG_FLYNN;   // Declaration and initialization together
    #endif
     
     // Default radio configuration from config.h
     nrf_config_t config = {
         .channel = NRF_CHANNEL,
         .power_rate = NRF_POWER_RATE,
         .address = NRF_ADDRESS,
         .payload_size = NRF_PAYLOAD_SIZE
     };
     
     // Initialize nRF24L01 in RX mode
     printf("Initializing nRF24L01 in receiver mode...\n");
     nrf_init(pins, config, NRF_MODE_RX);
     nrf_check_configuration();
     
     // Create the gateway task
     xTaskCreate(gateway_task, "gateway_task", 4096, NULL, 5, NULL);
     
     ESP_LOGI(TAG, "Gateway started. Waiting for nRF24L01 data to forward...");
     
 #else
     ESP_LOGI(TAG, "Running in RECEIVER mode (Wi-Fi receiver only)");
     
     // Create UDP communication task
     xTaskCreate(udp_receiver_task, "udp_receiver_task", 4096, NULL, 5, NULL);
     
     ESP_LOGI(TAG, "Receiver started. Waiting for UDP messages...");
 #endif
     
     ESP_LOGI(TAG, "Application started successfully");
     
     // Main loop - just keep the system alive
     while (1) {
         vTaskDelay(10000 / portTICK_PERIOD_MS);
     }
 }
 
 // Gateway task: forward nRF24L01 messages to Wi-Fi
void gateway_task(void *pvParameters) {
    uint8_t rx_buffer[NRF_PAYLOAD_SIZE];
    char local_ip[16];
    
    // Get local IP address
    esp_err_t ret = get_local_ip(local_ip, sizeof(local_ip));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get local IP address");
        vTaskDelete(NULL);
        return;
    }
    
    // Initialize UDP
    if (app_udp_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize UDP");
        vTaskDelete(NULL);
        return;
    }
    
    while (1) {
        // Process all available messages in the buffer
        int msg_count = 0;
        
        // Check for incoming nRF24L01 data in a tight loop until buffer is empty
        while (nrf_data_available() && msg_count < 10) {  // Process up to 10 messages per cycle (safety limit)
            // Read data from nRF24L01
            memset(rx_buffer, 0, sizeof(rx_buffer));
            nrf_read_data(rx_buffer, NRF_PAYLOAD_SIZE);
            
            // Ensure null termination if it's a string
            rx_buffer[NRF_PAYLOAD_SIZE - 1] = '\0';
            
            // Log the received data
            ESP_LOGI(TAG, "Received data from nRF24L01: %s", rx_buffer);
            
            // Forward the data via UDP to the receiver ESP32
            ESP_LOGI(TAG, "Forwarding data to %s", RECEIVER_ESP32_IP);
            app_udp_send(RECEIVER_ESP32_IP, rx_buffer, strlen((char*)rx_buffer));
            
            msg_count++;
            
            // Small delay between UDP sends to prevent network congestion
            if (nrf_data_available()) {
                vTaskDelay(5 / portTICK_PERIOD_MS);
            }
        }
        
        if (msg_count > 0) {
            ESP_LOGI(TAG, "Processed %d messages in this cycle", msg_count);
        }
        
        // Shorter delay to check more frequently for new messages
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    
    // This point is never reached, but we clean up anyway
    udp_close();
    vTaskDelete(NULL);
}