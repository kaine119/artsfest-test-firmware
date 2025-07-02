#ifndef AUDIO_TASK_H
#define AUDIO_TASK_H

#define I2S_DATA_PIN 10
#define I2S_CLOCK_PIN_BASE 11 // BCK = CLK_PIN_BASE, LRCK = CLK_PIN_BASE + 1

#define SAMPLES_PER_BUFFER 256
#define I2S_SAMPLE_RATE 48000

void audio_task(void* params);

#endif
