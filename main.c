#include <stdio.h>
#include <pico/stdlib.h>
#include "touch_task.h"
#include "audio_task.h"

#include "FreeRTOS.h"
#include "task.h"

//Pin allocations
#define BUTTON1_PIN 18 //octUp
#define BUTTON2_PIN 19 //octDown
#define BUTTON3_PIN 20 //toneSel
#define ADC_PIN 29 //vol

#define DEBOUNCE_INT 50

TaskHandle_t touch_task_handle;
TaskHandle_t audio_task_handle;

QueueHandle_t key_event_queue;

volatile absolute_time_t lastClick;

void buttonIrq(uint gpio, uint32_t events) {
    //Deboucing
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(lastClick, now) > DEBOUNCE_INT * 1000 && (events & GPIO_IRQ_EDGE_FALL)) {
        lastClick = now;

        //Check which button triggered the interupt
        if(gpio == BUTTON1_PIN){
            printf("Button 1 \n");
        }else if(gpio == BUTTON2_PIN){
            printf("Button 2 \n");
        }else if(gpio == BUTTON3_PIN){
            printf("Button 3 \n");
        }else{
            printf("Interupt \n");
        }
    }
}

int main() {
    stdio_init_all();

    gpio_init(BUTTON1_PIN);
    gpio_set_dir(BUTTON1_PIN, GPIO_IN);
    gpio_pull_up(BUTTON1_PIN);

    lastClick = get_absolute_time();
    
    gpio_set_irq_enabled_with_callback(BUTTON1_PIN, GPIO_IRQ_EDGE_FALL, true, &buttonIrq);
    gpio_set_irq_enabled(BUTTON2_PIN, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BUTTON3_PIN, GPIO_IRQ_EDGE_FALL, true);

    key_event_queue = xQueueCreate(20, sizeof(uint16_t));

    xTaskCreate(touch_task, "touch_task", 4096, key_event_queue, 1, &touch_task_handle);
    xTaskCreate(audio_task, "audio_task", 4096, key_event_queue, 2, &audio_task_handle);

    vTaskStartScheduler();
    while (1) {}
}
