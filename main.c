#include <stdio.h>
#include <pico/stdlib.h>
#include "touch_task.h"
#include "audio_task.h"

#include "FreeRTOS.h"
#include "task.h"

//Pin allocations
#define OCT_UP 21
#define OCT_DOWN 18
#define TONE_SEL 22

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
            uint16_t oct_up = (1 << 6);
            if(xQueueSend(key_event_queue, &oct_up, 0) != pdTRUE){
                printf("oct up failed");
            }else{
                printf("oct up\n");
            }
        }else if(gpio == OCT_DOWN){
            uint16_t oct_down = (1 << 5);
            if(xQueueSend(key_event_queue, &oct_down, 0) != pdTRUE){
                printf("oct down failed");
            }else{
                printf("oct down\n");
            }
        }else if(gpio == TONE_SEL){
            uint16_t tone_sel = (1 << 7);
            if(xQueueSend(key_event_queue, &tone_sel, 0 != pdTRUE)){
                printf("tone changed");
            }else{
                printf("tone change failed\n");
            }
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
