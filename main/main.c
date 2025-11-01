// Minimal Modbus RTU slave example on UART1 (RS485 half-duplex)

#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "driver/uart.h"

#include "mbcontroller.h"  // ESP-Modbus controller API

// UART1 (MODBUS RTU Slave)
#define MB_UART_NUM         UART_NUM_1
#define MB_BAUD_RATE        9600
#define MB_PARITY_MODE      UART_PARITY_DISABLE
#define MB_STOP_BITS_MODE   UART_STOP_BITS_1
#define MB_SLAVE_ADDRESS    7

// Modbus (RS485) pins: TX, RX, RTS(DE/RE)
#define MB_UART_TX_PIN      25
#define MB_UART_RX_PIN      26
#define MB_UART_RTS_PIN     27

static const char *TAG = "MB_RTU_SLAVE";

// Ten holding registers starting from 0x0000, values 0..9
static uint16_t holding_registers[10];

void app_main(void)
{
    // Enable debug logging for Modbus port (to see incoming/outgoing frames)
    esp_log_level_set("mb_port.serial", ESP_LOG_DEBUG);
    esp_log_level_set("mbc_serial.slave", ESP_LOG_DEBUG);
    esp_log_level_set("mbs_rtu", ESP_LOG_DEBUG);
    esp_log_level_set("mb_obj.slave", ESP_LOG_DEBUG);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    // Initialize holding registers with values 0..9
    for (uint16_t i = 0; i < (uint16_t)(sizeof(holding_registers) / sizeof(holding_registers[0])); ++i) {
        holding_registers[i] = i;
    }

    void *mbc_slave_handle = NULL;

    // Configure Modbus communication (RTU over UART1)
    mb_communication_info_t comm_config = {
        .ser_opts.port = MB_UART_NUM,
        .ser_opts.mode = MB_RTU,
        .ser_opts.baudrate = MB_BAUD_RATE,
        .ser_opts.parity = MB_PARITY_NONE,
        .ser_opts.uid = MB_SLAVE_ADDRESS,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = MB_STOP_BITS_MODE,
    };

    // Create Modbus RTU slave controller
    ESP_ERROR_CHECK(mbc_slave_create_serial(&comm_config, &mbc_slave_handle));

    // Map holding registers (0x0000..0x0009), read/write access
    mb_register_area_descriptor_t reg_area = {0};
    reg_area.type = MB_PARAM_HOLDING;
    reg_area.start_offset = 0x0000;
    reg_area.address = (void *)holding_registers;
    reg_area.size = sizeof(holding_registers); // 20 bytes for 10 uint16_t registers
    reg_area.access = MB_ACCESS_RW;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(mbc_slave_handle, reg_area));
    ESP_LOGI(TAG, "Holding registers mapped: start=0x%04X, size=%d bytes (%d registers)", 
             reg_area.start_offset, (int)reg_area.size, (int)(reg_area.size / 2));

    // Configure UART pins and RS485 half-duplex mode (AFTER creating controller, BEFORE starting)
    ESP_ERROR_CHECK(uart_set_pin(MB_UART_NUM, MB_UART_TX_PIN, MB_UART_RX_PIN, MB_UART_RTS_PIN, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(MB_UART_NUM, UART_MODE_RS485_HALF_DUPLEX));
    ESP_LOGI(TAG, "UART pins configured: TX=%d, RX=%d, RTS=%d", 
             MB_UART_TX_PIN, MB_UART_RX_PIN, MB_UART_RTS_PIN);

    // Start Modbus controller
    ESP_ERROR_CHECK(mbc_slave_start(mbc_slave_handle));
    ESP_LOGI(TAG, "Modbus RTU slave started: UART%d, 9600-8N1, addr=%d", (int)MB_UART_NUM, MB_SLAVE_ADDRESS);
    ESP_LOGI(TAG, "Holding registers initialized: [0]=%d, [1]=%d, [2]=%d, [3]=%d, [4]=%d", 
             holding_registers[0], holding_registers[1], holding_registers[2], 
             holding_registers[3], holding_registers[4]);

    // Main loop: check for Modbus events (following official example pattern)
    mb_param_info_t reg_info;
    // Use MB_READ_WRITE_MASK like in the official example (includes all read/write events)
    const mb_event_group_t mb_evt_mask = MB_EVENT_HOLDING_REG_RD | MB_EVENT_HOLDING_REG_WR | 
                                         MB_EVENT_INPUT_REG_RD | MB_EVENT_COILS_RD | 
                                         MB_EVENT_COILS_WR | MB_EVENT_DISCRETE_RD;
    #define MB_PAR_INFO_GET_TOUT  10  // Timeout in milliseconds (like in example: 10ms)
    
    ESP_LOGI(TAG, "Entering main event loop, waiting for Modbus requests...");
    
    // Diagnostic counter
    uint32_t loop_count = 0;
    
    while (1) {
        // Check UART buffer first to see if data is arriving
        size_t buffered_size = 0;
        uart_get_buffered_data_len(MB_UART_NUM, &buffered_size);
        if (buffered_size > 0) {
            ESP_LOGI(TAG, "Data detected in UART buffer: %d bytes", (int)buffered_size);
        }
        
        // Get parameter information (will timeout if no event - non-blocking approach)
        // Note: Modbus stack processes requests automatically in background task
        // This just notifies us when registers are accessed
        esp_err_t err = mbc_slave_get_param_info(mbc_slave_handle, &reg_info, MB_PAR_INFO_GET_TOUT);
        
        if (err == ESP_OK) {
            // Process only holding register events
            if (reg_info.type & (MB_EVENT_HOLDING_REG_RD | MB_EVENT_HOLDING_REG_WR)) {
                const char* rw_str = (reg_info.type & MB_EVENT_HOLDING_REG_RD) ? "READ" : "WRITE";
                ESP_LOGI(TAG, "HOLDING %s - ADDR:%u, SIZE:%u bytes (%u registers), INST_ADDR:0x%p", 
                         rw_str, 
                         (unsigned)reg_info.mb_offset,
                         (unsigned)reg_info.size,
                         (unsigned)(reg_info.size / 2),
                         (void*)reg_info.address);
                
                // Log all register values
                ESP_LOGI(TAG, "Register values: [0]=%d, [1]=%d, [2]=%d, [3]=%d, [4]=%d, [5]=%d, [6]=%d, [7]=%d, [8]=%d, [9]=%d",
                         holding_registers[0], holding_registers[1], holding_registers[2],
                         holding_registers[3], holding_registers[4], holding_registers[5],
                         holding_registers[6], holding_registers[7], holding_registers[8], holding_registers[9]);
            }
        } else if (err != ESP_ERR_TIMEOUT) {
            // Only log non-timeout errors (timeout is normal when no events)
            ESP_LOGW(TAG, "Failed to get param info: 0x%x", err);
        }
        
        // Periodically show status (every 5 seconds = 50 iterations * 100ms timeout)
        loop_count++;
        if (loop_count >= 50) {
            size_t buffered_size = 0;
            uart_get_buffered_data_len(MB_UART_NUM, &buffered_size);
            ESP_LOGI(TAG, "[%lu s] Status: UART buffer=%d bytes, waiting for Modbus requests...", 
                     (unsigned long)(esp_timer_get_time() / 1000000), (int)buffered_size);
            loop_count = 0;
        }
        
        // Small delay between iterations (only if no event was processed)
        if (err == ESP_ERR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
