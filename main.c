#include <stdio.h>
#include <pico/stdlib.h>
#include "touch_task.h"
#include "audio_task.h"

#include "FreeRTOS.h"
#include "task.h"

//Pin allocations
#define OCT_UP 18
#define OCT_DOWN 19
#define TONE_SEL 20

#define DEBOUNCE_INT 50

TaskHandle_t touch_task_handle;
TaskHandle_t audio_task_handle;

QueueHandle_t key_event_queue;

volatile absolute_time_t lastClick;

void setupButton(uint gpio){
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_pull_up(gpio);
}

void buttonIrq(uint gpio, uint32_t events) {
    //Deboucing
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(lastClick, now) > DEBOUNCE_INT * 1000 && (events & GPIO_IRQ_EDGE_FALL)) {
        lastClick = now;

        //Check which button triggered the interupt
        if(gpio == OCT_UP){
            uint16_t oct_up = 128;
            xQueueSend(key_event_queue, &oct_up, 0);
        }else if(gpio == OCT_DOWN){
            uint16_t oct_down = 64;
            xQueueSend(key_event_queue, &oct_down, 0);
        }else if(gpio == TONE_SEL){
            printf("Tone change \n");
        }else{
            printf("Interupt \n");
        }
    }
}

int main() {
    stdio_init_all();

    //Button stuff
    setupButton(OCT_UP);
    setupButton(OCT_DOWN);
    setupButton(TONE_SEL);
    lastClick = get_absolute_time();
    
    gpio_set_irq_enabled_with_callback(OCT_UP, GPIO_IRQ_EDGE_FALL, true, &buttonIrq);
    gpio_set_irq_enabled(OCT_DOWN, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(TONE_SEL, GPIO_IRQ_EDGE_FALL, true);

    key_event_queue = xQueueCreate(20, sizeof(uint16_t));

    xTaskCreate(touch_task, "touch_task", 4096, key_event_queue, 1, &touch_task_handle);
    xTaskCreate(audio_task, "audio_task", 4096, key_event_queue, 2, &audio_task_handle);

    vTaskStartScheduler();
    while (1) {}
}
