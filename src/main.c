/**
 * @title NRF24L01 Transceiver Application
 * @file main.c
 * @author Shanghong Lin (BUILD1)
 * @original Feiyang Zheng (GAME2)
 * @AI Use Claude3.7 Sonnet for reformatting the code
 * @version 0.1
 * @date 2025-04-24
 * 
 * @brief This application demonstrates the usage of the nRF24L01 transceiver module.
 *        It initializes the nRF24L01 in either transmitter or receiver mode, depending on the mode selected.
 *        In transmitter mode, it sends a predefined message every second.
 *        In receiver mode, it continuously checks for incoming data and prints it to the console.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nrf24_inte.h"

void app_main(void) {
    printf("Starting nRF24L01 Transceiver Application\n");

    // Default pin configuration (choose one based on the hardware)
    nrf_pins_t pins = PIN_CONFIG_BUILD1;
    // nrf_pins_t pins = PIN_CONFIG_FLYNN;
    
    // Default radio configuration
    nrf_config_t config = {
        .channel = 0x67,         // RF channel (must be the same on TX and RX)
        .power_rate = 0x06,      // RF_SETUP: 0dBm output power, 1Mbps data rate
        .address = {0x00, 0x00, 0x00, 0x00, 0x01}, // 5-byte address
        .payload_size = 32       // 32-byte payload
    };
    
    // Choose mode: NRF_MODE_TX for transmitter, NRF_MODE_RX for receiver
    nrf_mode_t mode = NRF_MODE_RX;
    
    // Initialize nRF24L01
    printf("Initializing nRF24L01 in %s mode...\n", 
           mode == NRF_MODE_RX ? "receiver" : "transmitter");
    nrf_init(pins, config, mode);
// while(1)
// {
//     nrf_check_configuration();
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
// }
    nrf_check_configuration();
    

    // Start the appropriate task based on mode
    if (mode == NRF_MODE_TX) {
        xTaskCreate(tx_task, "tx_task", 2048, NULL, 5, NULL);
        printf("Transmitter started. Sending data every second...\n");
    } else {
        xTaskCreate(rx_task, "rx_task", 2048, NULL, 5, NULL);
        printf("Receiver started. Waiting for incoming data...\n");
    }
    
    // The FreeRTOS tasks will now run independently
}