// Simple Modbus Slave Test
// This example demonstrates basic Modbus RTU slave functionality

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/uart.h"
#include "mbcontroller.h"

// Modbus configuration
#define MB_PORT_NUM         UART_NUM_1      // UART port number
#define MB_SLAVE_ADDR       0x03            // Slave address (1)
#define MB_BAUD_RATE        9600            // Baud rate

// UART pins: TX, RX, RTS (for RS485 DE/RE)
#define UART_TX_PIN         25
#define UART_RX_PIN         26
#define UART_RTS_PIN        27

static const char *TAG = "MODBUS_SLAVE";

// Simple register structures
#pragma pack(push, 1)
typedef struct {
    uint16_t value0;  // Holding register at address 0
    uint16_t value1;  // Holding register at address 1
    uint16_t value2;  // Holding register at address 2
} holding_regs_t;

typedef struct {
    uint16_t input0;  // Input register at address 0
    uint16_t input1;  // Input register at address 1
} input_regs_t;
#pragma pack(pop)

// Register storage
static holding_regs_t holding_regs = {100, 200, 300};
static input_regs_t input_regs = {1000, 2000};

static void *mbc_slave_handle = NULL;

void app_main(void) {
    ESP_LOGI(TAG, "Starting Modbus Slave test...");
    
    mb_register_area_descriptor_t reg_area = {0};
    mb_param_info_t reg_info;
    esp_err_t err = ESP_OK;

    // Configure Modbus communication
    mb_communication_info_t comm_config = {
        .ser_opts.port = MB_PORT_NUM,
        .ser_opts.mode = MB_RTU,               // RTU mode
        .ser_opts.baudrate = MB_BAUD_RATE,
        .ser_opts.parity = MB_PARITY_NONE,
        .ser_opts.uid = MB_SLAVE_ADDR,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_1
    };

    // Create Modbus slave controller
    ESP_LOGI(TAG, "Initializing Modbus slave controller...");
    err = mbc_slave_create_serial(&comm_config, &mbc_slave_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create Modbus slave: 0x%x", err);
        return;
    }

    // Set up Holding Registers area (read/write, starting at address 0)
    reg_area.type = MB_PARAM_HOLDING;
    reg_area.start_offset = 0;
    reg_area.address = (void*)&holding_regs;
    reg_area.size = sizeof(holding_regs_t);
    reg_area.access = MB_ACCESS_RW;
    err = mbc_slave_set_descriptor(mbc_slave_handle, reg_area);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set holding registers descriptor: 0x%x", err);
        goto cleanup;
    }
    ESP_LOGI(TAG, "Holding registers configured: address 0, size %d bytes", sizeof(holding_regs_t));

    // Set up Input Registers area (read-only, starting at address 0)
    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = 0;
    reg_area.address = (void*)&input_regs;
    reg_area.size = sizeof(input_regs_t);
    reg_area.access = MB_ACCESS_RO;
    err = mbc_slave_set_descriptor(mbc_slave_handle, reg_area);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set input registers descriptor: 0x%x", err);
        goto cleanup;
    }
    ESP_LOGI(TAG, "Input registers configured: address 0, size %d bytes", sizeof(input_regs_t));

    // Configure UART pins
    ESP_LOGI(TAG, "Configuring UART pins: TX=%d, RX=%d, RTS=%d", UART_TX_PIN, UART_RX_PIN, UART_RTS_PIN);
    err = uart_set_pin(MB_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_RTS_PIN, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: 0x%x", err);
        goto cleanup;
    }

    // Set UART to RS485 half-duplex mode
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set RS485 mode: 0x%x", err);
        goto cleanup;
    }

    // Start Modbus slave
    ESP_LOGI(TAG, "Starting Modbus slave stack...");
    err = mbc_slave_start(mbc_slave_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Modbus slave: 0x%x", err);
        goto cleanup;
    }

    ESP_LOGI(TAG, "Modbus slave initialized successfully!");
    ESP_LOGI(TAG, "Slave address: %d, Port: %d, Baud rate: %d", MB_SLAVE_ADDR, MB_PORT_NUM, MB_BAUD_RATE);
    ESP_LOGI(TAG, "Waiting for Modbus master requests...");

    // Main loop: monitor Modbus events
    mb_event_group_t event;
    const mb_event_group_t read_mask = MB_EVENT_INPUT_REG_RD | MB_EVENT_HOLDING_REG_RD;
    const mb_event_group_t write_mask = MB_EVENT_HOLDING_REG_WR;
    const mb_event_group_t all_mask = read_mask | write_mask;

    while (1) {
        // Check for Modbus events
        event = mbc_slave_check_event(mbc_slave_handle, all_mask);
        
        if (event) {
            // Get parameter information
            err = mbc_slave_get_param_info(mbc_slave_handle, &reg_info, pdMS_TO_TICKS(100));
            if (err == ESP_OK) {
                const char* type_str = "";
                
                if (reg_info.type & MB_EVENT_HOLDING_REG_RD) type_str = "HOLDING REG READ";
                else if (reg_info.type & MB_EVENT_HOLDING_REG_WR) type_str = "HOLDING REG WRITE";
                else if (reg_info.type & MB_EVENT_INPUT_REG_RD) type_str = "INPUT REG READ";

                ESP_LOGI(TAG, "%s: addr=%d, size=%d, offset=0x%04X, timestamp=%lu us",
                         type_str, reg_info.mb_offset, reg_info.size, reg_info.mb_offset, reg_info.time_stamp);

                // Print register values after write operations
                if (reg_info.type & MB_EVENT_HOLDING_REG_WR) {
                    ESP_LOGI(TAG, "  Holding regs: [0]=%d, [1]=%d, [2]=%d",
                             holding_regs.value0, holding_regs.value1, holding_regs.value2);
                }
            }
        }

        // Update input registers periodically (simulate sensor readings)
        static uint32_t counter = 0;
        counter++;
        if (counter % 10000 == 0) {
            input_regs.input0++;
            input_regs.input1 += 2;
            ESP_LOGI(TAG, "Input regs updated: [0]=%d, [1]=%d", input_regs.input0, input_regs.input1);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay
    }

cleanup:
    if (mbc_slave_handle) {
        ESP_LOGI(TAG, "Destroying Modbus slave controller...");
        mbc_slave_delete(mbc_slave_handle);
        mbc_slave_handle = NULL;
    }
    ESP_LOGI(TAG, "Modbus slave test ended");
}
