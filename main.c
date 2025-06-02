#include <stdio.h>
#include <pico/stdlib.h>
#include "touch_task.h"
#include "audio_task.h"

#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t touch_task_handle;
TaskHandle_t audio_task_handle;

QueueHandle_t key_event_queue;

int main() {
    stdio_init_all();

    key_event_queue = xQueueCreate(20, sizeof(uint16_t));

    xTaskCreate(touch_task, "touch_task", 4096, key_event_queue, 1, &touch_task_handle);
    xTaskCreate(audio_task, "audio_task", 4096, key_event_queue, 2, &audio_task_handle);

    vTaskStartScheduler();
    while (1) {}
}
