#ifndef _TOUCH_TASK_H
#define _TOUCH_TASK_H

#include <FreeRTOS.h>
#include <queue.h>

#define I2C_SDA_PIN PICO_DEFAULT_I2C_SDA_PIN
#define I2C_SCL_PIN PICO_DEFAULT_I2C_SCL_PIN
#define TOUCH_RST_PIN 15

#define TOUCH_SLAVE_ADDR (0x37)

// typedef struct TouchMessage_t {
//     // TODO: fill in
// } TouchMessage_t;

void touch_initialize_hw();
void touch_initialize_device();
uint16_t touch_fetch_contacted_pads();

void touch_read_register(uint8_t reg_addr, uint8_t* rxdata, size_t size);
void touch_write_register(uint8_t reg_addr, uint8_t* txdata, size_t size);
void touch_write_ctrl_cmd(uint8_t cmd);
void touch_reset_device();

void touch_task(void* params);

#endif