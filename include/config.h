#ifndef CONFIG_H
#define CONFIG_H

// =======================================================
// Wi-Fi configuration
// =======================================================
#define WIFI_SSID "LynnsHW"
#define WIFI_PASSWORD "HWlsh123!"
#define WIFI_CONNECT_RETRY_MAX 10

// Communication configuration
#define UDP_PORT 3333

// Device role configuration
// For the Gateway ESP32 that receives from nRF24L01 and forwards to Wi-Fi
#define DEVICE_MODE_GATEWAY    1
// For the Receiver ESP32 that only listens for Wi-Fi messages
#define DEVICE_MODE_RECEIVER   2

// Set the mode for this device
#define DEVICE_MODE DEVICE_MODE_GATEWAY

// Communication partner IP (update with the IP of the Receiver ESP32)
#define RECEIVER_ESP32_IP "192.168.43.137"

// Time intervals (in milliseconds)
#define SEND_INTERVAL_MS 200
#define WIFI_RECONNECT_INTERVAL_MS 5000

// =======================================================
// nRF24L01 configuration
// =======================================================

// Pin configurations (choose one based on your hardware)
#define USE_PIN_CONFIG_BUILD1   1
// #define USE_PIN_CONFIG_FLYNN    0

// Radio communication parameters
#define NRF_CHANNEL 0x67
#define NRF_POWER_RATE 0x06
#define NRF_ADDRESS {0x00, 0x00, 0x00, 0x00, 0x01}
#define NRF_PAYLOAD_SIZE 32

#endif // CONFIG_H