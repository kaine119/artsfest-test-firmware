#include <stdio.h>
#include <pico/stdlib.h>
#include "pico/audio_i2s.h"
#include "pico/audio.h"
#include "pico/binary_info/code.h"
#include <math.h>
#include "hardware/adc.h"
#include "hardware/gpio.h"

#include <FreeRTOS.h>
#include <queue.h>

#include <audio_task.h>

#define VOL_PIN 26 

#define PIN_SPK_SHUTDOWN_N 1
#define PIN_DAC_XSMT 9

const float FREQ_TABLE[13] = {
    261.63, // C4
    277.18, // C#4
    293.66, // D4
    311.13, // D#4
    329.63, // E4
    349.23, // F4
    369.99, // F#4
    392,    // G4
    415.3,  // G#4
    440,    // A4
    466,    // Bb4
    493.88, // B4
    523.26  // C5
};

const uint8_t key_order[13] = {0, 1, 2, 3, 15, 14, 13, 12, 11, 10, 9, 8, 7};

static int8_t note_to_play = -1; // -1 if no notes
static int octave = 0;
static int tone =0; // 0 = sine, 1 = sawtooth, 2 = triangle, 3 = square
static bool octave_pressed = false;

bi_decl(bi_3pins_with_names(PICO_AUDIO_I2S_DATA_PIN, "I2S DIN", PICO_AUDIO_I2S_CLOCK_PIN_BASE, "I2S BCK", PICO_AUDIO_I2S_CLOCK_PIN_BASE+1, "I2S LRCK"));

struct audio_buffer_pool *init_audio() {

    static audio_format_t audio_format = {
            .format = AUDIO_BUFFER_FORMAT_PCM_S16,
            .sample_freq = I2S_SAMPLE_RATE,
            .channel_count = 1,
    };

    static struct audio_buffer_format producer_format = {
            .format = &audio_format,
            .sample_stride = 2
    };

    struct audio_buffer_pool *producer_pool = audio_new_producer_pool(&producer_format, 3,
                                                                      SAMPLES_PER_BUFFER); // todo correct size
    bool __unused ok;
    const struct audio_format *output_format;
    struct audio_i2s_config config = {
            .data_pin = I2S_DATA_PIN,              // DOUT from MCU
            .clock_pin_base = I2S_CLOCK_PIN_BASE,  // BCLK
                                                   // LRCLK = BCLK + 1
            .dma_channel = 0,
            .pio_sm = 0,
    };

    output_format = audio_i2s_setup(&audio_format, &config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }

    ok = audio_i2s_connect(producer_pool);
    if (!ok) {
        printf("[audio_task] failed to initialize producer pool\n");
    }
    assert(ok);
    audio_i2s_set_enabled(true);
    return producer_pool;
}

static void handle_key_event(uint16_t key_event) {

    if (key_event == 0) {
        octave_pressed = false;
        note_to_play = -1;
    }
    else if ((key_event & (1 << 5)) && !octave_pressed) {
        octave_pressed = true;
        octave -= 1;
    } else if ((key_event & (1 << 6)) && !octave_pressed) {
        octave_pressed = true;
        octave += 1;
    } else if((key_event & (1 << 7))){
        if(++tone > 3){tone = 0;}
    } else {
        for (int i = 0; i < 13; i++) {
            if (key_event & (1 << key_order[i])) {
                note_to_play = i;
            }
        }
    }
}

int16_t calculate_sample(uint16_t adc_raw) {
    int absolute_max_volume = 18000;
    int max_volume = (adc_raw * absolute_max_volume) / 4095;
    static float vol = 0;
    static float freq = 880;
    uint mask;

    // Get tone frequency from key-freq map
    if (note_to_play != -1) {
        if (octave >= 0) freq = FREQ_TABLE[note_to_play] * (1 << octave);
        else             freq = FREQ_TABLE[note_to_play] / (1 << -octave);
    }

    // Increase / decrease volume over time for attack and decay envelopes
    // Volume is changed every sample by a increment/decrement
    // calculated based on an attack time.
    static uint attack_sample_count = 0;
    float attack = 0.01;
    float attack_sample_total = attack * I2S_SAMPLE_RATE; // == attack / sample_period
    float attack_progress = attack_sample_count / attack_sample_total;
    float attack_incr = max_volume / attack_sample_total;

    static uint decay_sample_count = 0;
    float decay = 1;
    float decay_sample_total = decay * I2S_SAMPLE_RATE; // == decay / sample_period
    float decay_progress = decay_sample_count / decay_sample_total;
    float decay_incr = max_volume / decay_sample_total;


    if (note_to_play != -1 && attack_sample_count <= attack_sample_total) {
        decay_sample_count = 0;
        // vol = MIN(max_volume, vol + attack_incr);
        vol = MAX(vol, attack_progress * max_volume);
        attack_sample_count++;
    } else if (note_to_play == -1 && decay_sample_count <= decay_sample_total) {
        attack_sample_count = 0;
        // vol = MAX(0, vol - decay_incr);
        vol = MIN(vol, max_volume - decay_progress * max_volume);

        decay_sample_count++;
    }

    static uint count = 0;
    uint total_waveform_samples = I2S_SAMPLE_RATE / (freq);
    if (count >= total_waveform_samples) {
        count = 0;
    }
    count++;
    switch(tone){
        case 0:
            return round(vol * sinf(2 * M_PI * count / total_waveform_samples));//sinwave
            break;
        case 1:
            return round(vol * (2*count / total_waveform_samples - 1)); //sawtooth
            break;
        case 2:
            return round(vol * (1 - 2 * abs(2 * count / total_waveform_samples - 1))); //Triangle
            break;
        case 3:
            return vol * (count / total_waveform_samples < 0.5) ? 1 : -1; //Square
            break;
        default:
            tone = 0;
            return round(vol * sinf(2 * M_PI * count / total_waveform_samples));//sinwave
            break;
    }
}

void audio_task(void* params) {

    //ADC stuff
    adc_init();
    adc_gpio_init(VOL_PIN); 
    adc_select_input(0);
    
    gpio_init(PIN_DAC_XSMT);
    gpio_set_dir(PIN_DAC_XSMT, GPIO_OUT);
    gpio_put(PIN_DAC_XSMT, true);

    gpio_init(PIN_SPK_SHUTDOWN_N);
    gpio_set_dir(PIN_SPK_SHUTDOWN_N, GPIO_OUT);
    gpio_put(PIN_SPK_SHUTDOWN_N, true);
    
    printf("[audio_task] Starting audio_task\n");

    struct audio_buffer_pool *ap = init_audio();
    

    QueueHandle_t key_event_queue = (QueueHandle_t) params;
    uint16_t key_event;

    while (true) {
        if (xQueueReceive(key_event_queue, &key_event, 0) == pdPASS) {
            handle_key_event(key_event);
        }

        struct audio_buffer *buffer = take_audio_buffer(ap, true);
        int16_t *samples = (int16_t *) buffer->buffer->bytes;
        uint16_t adc_value = adc_read();
        for (uint i = 0; i < buffer->max_sample_count; i++) {
            samples[i] = calculate_sample(adc_value);
        }
        buffer->sample_count = buffer->max_sample_count;
        give_audio_buffer(ap, buffer);
    }
    puts("\n");
}

