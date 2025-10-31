// Minimal Modbus RTU slave example on UART1 (RS485 half-duplex)

#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

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
    reg_area.size = sizeof(holding_registers); // bytes
    reg_area.access = MB_ACCESS_RW;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(mbc_slave_handle, reg_area));

    // Configure UART pins and RS485 half-duplex mode
    ESP_ERROR_CHECK(uart_set_pin(MB_UART_NUM, MB_UART_TX_PIN, MB_UART_RX_PIN, MB_UART_RTS_PIN, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(MB_UART_NUM, UART_MODE_RS485_HALF_DUPLEX));

    // Start Modbus controller
    ESP_ERROR_CHECK(mbc_slave_start(mbc_slave_handle));
    ESP_LOGI(TAG, "Modbus RTU slave started: UART%d, 9600-8N1, addr=%d", (int)MB_UART_NUM, MB_SLAVE_ADDRESS);

    // Idle loop; Modbus stack runs in its own tasks
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
