# ESP32 NRF24L01 Transceiver Application

## Hardware Setup

### Pin Configurations (from nrf24_inte.h):
```c
// BUILD1 configuration
#define PIN_CONFIG_BUILD1 {  
    .ce_pin = 4,             
    .csn_pin = 5,            
    .sck_pin = 7,            
    .miso_pin = 16,          
    .mosi_pin = 15
}

// FLYNN configuration
#define PIN_CONFIG_FLYNN {   
    .ce_pin = 9,             
    .csn_pin = 10,           
    .sck_pin = 12,           
    .miso_pin = 13,          
    .mosi_pin = 11
}
```

## Configuration Adaptation
Modify these parameters in `main.c`:
```c
nrf_config_t config = {
    .channel = 0x67,         // RF channel (0-125)
    .power_rate = 0x06,      // RF_SETUP: 0dBm power, 1Mbps data rate
    .address = {0x00,0x00,0x00,0x00,0x01}, // 5-byte address
    .payload_size = 32       // 1-32 bytes
};
```
- **Channel**: Ensure matching between transmitter/receiver
- **Address**: Use unique 5-byte addresses for network separation
- **Payload**: Match payload size between paired devices

## EE3 Receiver Mode Data Format
In receiver mode (NRF_MODE_RX), data packets follow this structure:
```
[<accumulated_items>, <current_weight>, <status_code>]
```
- `accumulated_items`: 8-bit unsigned integer
- `current_weight`: 16-bit signed integer
- `status_code`: 1-bit status code (0: rejected, 1: accepted)

Example payload processing in `rx_task`:
```c
uint8_t rx_buffer[32];
if (nrf_data_available()) {
    nrf_read_data(rx_buffer, 32);
    // Parse EE3 format:
    uint8_t item_count = *(uint8_t*)&rx_buffer[0];
    int16_t weight = *(int16_t*)&rx_buffer[1];  // 16-bit value starting at offset 1
    uint8_t status = rx_buffer[3] & 0x01;       // 1-bit status at offset 3
}
```

## Build & Usage
1. Set up ESP-IDF environment
2. Configure project:
```bash
idf.py set-target esp32
idf.py menuconfig  # Configure serial flasher
```
3. Build and flash:
```bash
idf.py build
idf.py -p PORT flash monitor
```

## Operational Modes
### Transmitter Mode (NRF_MODE_TX)
- Sends data every second
- Modify `tx_task` for custom payloads

### Receiver Mode (NRF_MODE_RX)
- Continuously monitors for packets
- Implements automatic ACK/NACK
- EE3-specific data format as described

## Troubleshooting
Use `nrf_check_configuration()` to verify:
- Radio channel matching
- Address consistency
- Power/rate settings
- FIFO status flags