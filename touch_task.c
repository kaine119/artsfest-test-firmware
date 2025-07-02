#include "touch_task.h"

#include <stdio.h>
#include "pico/binary_info.h"
#include <hardware/i2c.h>
#include <hardware/gpio.h>

#include "FreeRTOS.h"
#include <queue.h>

void touch_task(void *params) {
    printf("[touch_task] started\n");

    TickType_t lastSleep = 0;
    uint16_t last_active_pads = 0;
    uint16_t current_active_pads;
    QueueHandle_t touchMsgQueue = (QueueHandle_t) params;
    
    touch_initialize_hw();
    touch_initialize_device();

    printf("[touch_task] polling pads...\n");

    while (true) {
        current_active_pads = touch_fetch_contacted_pads();
        
        if (last_active_pads != current_active_pads) {
            xQueueSend(touchMsgQueue, &current_active_pads, 0);
        }

        last_active_pads = current_active_pads;
        xTaskDelayUntil(&lastSleep, 10 / portTICK_PERIOD_MS);
    }
}

void touch_initialize_hw() {
    i2c_init(i2c1, 100000);

    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL_PIN);
    gpio_pull_up(I2C_SDA_PIN);

    gpio_init(TOUCH_RST_PIN);
    gpio_set_dir(TOUCH_RST_PIN, GPIO_OUT);
    gpio_put(TOUCH_RST_PIN, true);

    bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));
}

void touch_initialize_device() {
    printf("[touch_task] initializing pads...\n");

    uint8_t bytes_to_write[5];
    uint8_t read_result[5];

    touch_reset_device();
    
    // SENSOR_EN: Enable all sensors
    bytes_to_write[0] = 0b11111111;
    bytes_to_write[1] = 0b11111111;
    touch_write_register(0x00, bytes_to_write, 2);
    
    // FSS_EN: Enable flank suppression for all sensors
    bytes_to_write[0] = 0b00000000;
    bytes_to_write[1] = 0b00000000;
    touch_write_register(0x02, bytes_to_write, 2);

    // TOGGLE_EN: Turn off all GPIO outputs
    // LED_ON_EN: Turn off GPIO extened LED ON durations
    bytes_to_write[0] = 0b00000000;
    bytes_to_write[1] = 0b00000000;
    touch_write_register(0x04, bytes_to_write, 1);
    touch_write_register(0x06, bytes_to_write, 1);

    // SENSITIVITY0-4: Set sensitivity (00 - 04 for each channel, in decreasing sensitivity)
    bytes_to_write[0] = 0b01010101;
    bytes_to_write[1] = 0b01010101;
    bytes_to_write[2] = 0b01010101;
    bytes_to_write[3] = 0b01010101;
    touch_write_register(0x08, bytes_to_write, 4);

    // SPO_CFG: Enable SPO0 and SPO1 as touch sensors
    bytes_to_write[0] = 0b00010001;
    touch_write_register(0x4c, bytes_to_write, 1);
    
    // DEVICE_CFG2: No auto reset, auto thresholding, no EMC solution, no guard, no active shield
    bytes_to_write[0] = 0b00001000;
    touch_write_register(0x4f, bytes_to_write, 1);
    
    // CTRL_CMD: Command device to calculate CRC
    touch_write_ctrl_cmd(3);

    // CALC_CRC: Get computed CRC from previous step
    touch_read_register(0x94, read_result, 2);
    printf("reg 0x94 (CALC_CRC): %08b %08b\n", read_result[0], read_result[1]);

    // CONFIG_CRC: Write CRC to device
    bytes_to_write[0] = read_result[0];
    bytes_to_write[1] = read_result[1];
    touch_write_register(0x7e, bytes_to_write, 2);

    // Save configuration data to NV memory
    touch_write_ctrl_cmd(2);
    touch_reset_device();

    // Print status
    touch_read_register(0x00, read_result, 2);
    printf("reg 0x00 (sensor_en): %08b %08b\n", read_result[0], read_result[1]);
    touch_read_register(0x02, read_result, 2);
    printf("reg 0x02 (fss_en): %08b %08b\n", read_result[0], read_result[1]);
    touch_read_register(0x04, read_result, 2);
    printf("reg 0x04 (toggle_en): %08b %08b\n", read_result[0], read_result[1]);
    touch_read_register(0x06, read_result, 2);
    printf("reg 0x06 (led_on_en): %08b %08b\n", read_result[0], read_result[1]);
    touch_read_register(0x08, read_result, 4);
    printf("reg 0x08 (sensitivity): %08b %08b %08b %08b\n", read_result[0], read_result[1], read_result[2], read_result[3]);
    touch_read_register(0x4c, read_result, 1);
    printf("reg 0x4c (SPO_CFG): %08b\n", read_result[0]);
    touch_read_register(0x4d, read_result, 1);
    printf("reg 0x4d (DEVICE_CFG0): %08b\n", read_result[0]);
    touch_read_register(0x4e, read_result, 1);
    printf("reg 0x4e (DEVICE_CFG1): %08b\n", read_result[0]);
    touch_read_register(0x4f, read_result, 1);
    printf("reg 0x4f (DEVICE_CFG2): %08b\n", read_result[0]);
    touch_read_register(0x50, read_result, 1);
    printf("reg 0x50 (DEVICE_CFG3): %08b\n", read_result[0]);
}

uint16_t touch_fetch_contacted_pads() {
    uint8_t rxdata[2];
    touch_read_register(0xAA, rxdata, 2);

    // printf("button_stat: %08b %08b\r", rxdata[0], rxdata[1]);

    return rxdata[1] << 8 | rxdata[0];
}


void touch_read_register(uint8_t reg_addr, uint8_t* rxdata, size_t size) {
    int write_result = PICO_ERROR_GENERIC;
    while (write_result == PICO_ERROR_GENERIC) {
        write_result = i2c_write_blocking(i2c1, TOUCH_SLAVE_ADDR, &reg_addr, 1, true);
    }
    i2c_read_blocking(i2c1, TOUCH_SLAVE_ADDR, rxdata, size, false);
}

void touch_write_register(uint8_t reg_addr, uint8_t* txdata, size_t size) {
    int write_result = PICO_ERROR_GENERIC;
    while (write_result == PICO_ERROR_GENERIC) {
        write_result = i2c_write_burst_blocking(i2c1, TOUCH_SLAVE_ADDR, &reg_addr, 1);
    }
    i2c_write_blocking(i2c1, TOUCH_SLAVE_ADDR, txdata, size, false);
}

void touch_write_ctrl_cmd(uint8_t cmd) {
    uint8_t byte_tx = cmd;
    uint8_t byte_rx = 255;

    // Write command to CTRL_CMD register
    touch_write_register(0x86, &byte_tx, 1);

    // Wait for device to clear CTRL_CMD register before returning
    while (byte_rx != 0) {
        touch_read_register(0x86, &byte_rx, 1);
    }
}

void touch_reset_device() {
    // Reset device
    gpio_put(TOUCH_RST_PIN, false);
    gpio_put(TOUCH_RST_PIN, true);
}