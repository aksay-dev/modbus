// Simple UART loopback test (RX and TX shorted)

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
#define TEST_DELAY_MS       1000

static const char *TAG = "LOOPBACK_TEST";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting UART loopback test (RX-TX shorted)...");
    
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

    // Set UART mode to UART
    ESP_ERROR_CHECK(uart_set_mode(UART_NUM, UART_MODE_UART));
    
    ESP_LOGI(TAG, "UART%d configured: %d-8-N-1", UART_NUM, BAUDRATE);
    ESP_LOGI(TAG, "Pins: TX=%d, RX=%d (shorted)", UART_TX_PIN, UART_RX_PIN);
    ESP_LOGI(TAG, "Starting loopback test...\n");
    
    // Test patterns
    uint8_t test_patterns[][16] = {
        "Hello World!",
        {0x01, 0x02, 0x03, 0x04, 0x05, 0xAA, 0x55, 0xFF},
        {0x00, 0xFF, 0x55, 0xAA, 0x00},
    };
    int pattern_sizes[] = {12, 8, 5};
    int num_patterns = 3;
    
    uint32_t test_count = 0;
    uint32_t success_count = 0;
    uint32_t fail_count = 0;
    
    while (1) {
        // Clear RX buffer
        uart_flush(UART_NUM);
        
        // Select test pattern
        int pattern_idx = test_count % num_patterns;
        uint8_t *tx_data = test_patterns[pattern_idx];
        int tx_len = pattern_sizes[pattern_idx];
        
        // Send data
        int bytes_sent = uart_write_bytes(UART_NUM, tx_data, tx_len);
        ESP_LOGI(TAG, "[Test #%lu] Sent %d bytes:", (unsigned long)(test_count + 1), bytes_sent);
        
        // Print TX data
        printf("TX: ");
        for (int i = 0; i < tx_len; i++) {
            printf("%02X ", tx_data[i]);
        }
        printf("(");
        for (int i = 0; i < tx_len; i++) {
            if (tx_data[i] >= 32 && tx_data[i] < 127) {
                printf("%c", tx_data[i]);
            } else {
                printf(".");
            }
        }
        printf(")\n");
        
        // Wait a bit for data to loop back
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Read received data
        uint8_t rx_data[BUF_SIZE];
        int bytes_received = uart_read_bytes(UART_NUM, rx_data, BUF_SIZE, pdMS_TO_TICKS(100));
        
        // Print RX data
        printf("RX: ");
        for (int i = 0; i < bytes_received; i++) {
            printf("%02X ", rx_data[i]);
        }
        printf("(");
        for (int i = 0; i < bytes_received; i++) {
            if (rx_data[i] >= 32 && rx_data[i] < 127) {
                printf("%c", rx_data[i]);
            } else {
                printf(".");
            }
        }
        printf(")\n");
        
        // Compare TX and RX
        bool match = (bytes_sent == bytes_received);
        if (match) {
            for (int i = 0; i < bytes_sent; i++) {
                if (tx_data[i] != rx_data[i]) {
                    match = false;
                    break;
                }
            }
        }
        
        // Report result
        if (match) {
            ESP_LOGI(TAG, "RESULT: OK (bytes match)\n");
            success_count++;
        } else {
            ESP_LOGE(TAG, "RESULT: FAIL (sent %d, received %d, mismatch)\n", bytes_sent, bytes_received);
            fail_count++;
        }
        
        test_count++;
        
        // Summary every 10 tests
        if (test_count % 10 == 0) {
            ESP_LOGI(TAG, "=== Summary: Total=%lu, OK=%lu, FAIL=%lu ===\n", 
                     (unsigned long)test_count, (unsigned long)success_count, (unsigned long)fail_count);
        }
        
        vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_MS));
    }
}
