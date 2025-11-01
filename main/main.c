// Simple UART listener for diagnostics (UART1, RS485)

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/uart.h"

// UART1 configuration
#define UART_NUM            UART_NUM_1
#define BAUDRATE            9600
#define DATA_BITS           UART_DATA_8_BITS
#define PARITY              UART_PARITY_DISABLE
#define STOP_BITS           UART_STOP_BITS_1
#define FLOW_CTRL           UART_HW_FLOWCTRL_DISABLE

// UART pins: TX, RX, RTS (for RS485 DE/RE)
#define UART_TX_PIN         25
#define UART_RX_PIN         26
#define UART_RTS_PIN        27

// Buffer for reading
#define BUF_SIZE            1024

static const char *TAG = "UART_LISTENER";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting UART listener...");
    
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = BAUDRATE,
        .data_bits = DATA_BITS,
        .parity    = PARITY,
        .stop_bits = STOP_BITS,
        .flow_ctrl = FLOW_CTRL,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    
    // Set UART pins
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_RTS_PIN, UART_PIN_NO_CHANGE));
    
    // Set RS485 half-duplex mode
    ESP_ERROR_CHECK(uart_set_mode(UART_NUM, UART_MODE_RS485_HALF_DUPLEX));
    
    ESP_LOGI(TAG, "UART%d configured: %d-8-N-1, RS485 half-duplex", UART_NUM, BAUDRATE);
    ESP_LOGI(TAG, "Pins: TX=%d, RX=%d, RTS=%d", UART_TX_PIN, UART_RX_PIN, UART_RTS_PIN);
    ESP_LOGI(TAG, "Waiting for data...");
    
    uint8_t data[BUF_SIZE];
    uint32_t total_bytes = 0;
    uint32_t packet_count = 0;
    
    while (1) {
        // Check how many bytes are available
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            data[len] = '\0';  // Null-terminate for printing
            
            total_bytes += len;
            packet_count++;
            
            ESP_LOGI(TAG, "=== Packet #%lu, %d bytes ===", (unsigned long)packet_count, len);
            
            // Print as HEX
            ESP_LOGI(TAG, "HEX: ");
            for (int i = 0; i < len; i++) {
                printf("%02X ", data[i]);
                if ((i + 1) % 16 == 0) printf("\n");
            }
            printf("\n");
            
            // Print as ASCII (if printable)
            ESP_LOGI(TAG, "ASCII: ");
            for (int i = 0; i < len; i++) {
                if (data[i] >= 32 && data[i] < 127) {
                    printf("%c", data[i]);
                } else {
                    printf(".");
                }
            }
            printf("\n");
            
            // Print raw data for Modbus analysis
            ESP_LOGI(TAG, "Raw bytes: ");
            for (int i = 0; i < len; i++) {
                printf("%d ", data[i]);
            }
            printf("\n");
            
            ESP_LOGI(TAG, "Total received: %lu bytes in %lu packets\n", 
                     (unsigned long)total_bytes, (unsigned long)packet_count);
        } else {
            // No data received, periodically show status
            static uint32_t idle_count = 0;
            idle_count++;
            if (idle_count % 50 == 0) {  // Every 5 seconds (50 * 100ms)
                size_t buffered = 0;
                uart_get_buffered_data_len(UART_NUM, &buffered);
                ESP_LOGI(TAG, "[Status] Buffered: %d bytes, Total received: %lu bytes", 
                         (int)buffered, (unsigned long)total_bytes);
            }
        }
    }
}