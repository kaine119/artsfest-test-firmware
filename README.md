# Firmware for Artsfest 2025 PCB PoC board

extremely not-discount stylophone lmao

## Getting started

This project is built on the Raspberry Pi Pico SDK, you'll need an environment for that set up first.

* Follow the instructions [here](https://rptl.io/pico-get-started) to get your Pico SDK environment set up
* If you're using the vscode extension, use the buttons in vscode
* If not:
    * You'll need to have `PICO_SDK_PATH` on your $PATH, as per the setup instructions
    * To build (similar to most cmake projects):
        ```bash
        mkdir -p build
        cd build
        cmake ..
        make
        ```
* If cmake complains about a missing `PICO_EXTRAS_PATH`, clone [the `pico-extras` repo](https://github.com/raspberrypi/pico-extras) and place it next to the `pico-sdk` folder. You may need to find this yourself
* To flash, either use the button in vscode, or to do it manually:
    * hold BOOTSEL on the Pico, then press reset (or plug it in)
    * copy `build/artsfest-test.uf2` to the newly attached thumbstick named `RPI-RP2`
    * ...or use picotool (figure that out yourself)

You may need to reset the Pico to bootloader if the vscode extension (or picotool) isn't able to do it automatically.

## Hardware

Major parts list:
* MCU: Raspberry Pi RP2040 (currently in the form of a RPi Pico)
* Touch sensor: Infineon (formerly Cypress) CY8CMBR3116 16-way capacitive touch sensing controller
* DAC: Texas Instruments PCM5100A I2S DAC. This feeds both the headphone and the speaker amp
* Headphone amp: TI TPA6139A2RGTR, Speaker amp: TI TPA6211A1DGN
* Headphone jack: generic 3.5mm TRRS jack with normally-closed contacts on Tip and R1

For RPi Pico pinout, refer to [pinout.xyz](https://pico.pinout.xyz)

### Touch sensor

Pinout (configured in `touch_task.h`): 

| touch-board | RP2040          | 
| ---         | ---             |
| GND         | GND (duh)       |
| SDA         | GP4             | 
| SCL         | GP5             | 
| HI_N        | (not connected) |
| RST_N       | GP15            |
| VDD         | 3v3             |


The touch controller is connected to the MCU via I2C, and the controller has a *lot* of registers on it. Check out the registers manual [here](https://www.infineon.com/dgdl/Infineon-CY8CMBR3xxx_CapSense_Express_Controllers_Registers_TRM-AdditionalTechnicalInformation-v06_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0f90b2ad7da7). 

In brief:

* Write required settings to to the settings registers
* Write a CRC that validates the settings map.

    More legit operation would be to use Infineon's helper software to generate a CRC offline, then write it directly to the device. I can't be bothered to do that, and one of the commands on the on-chip command interface calculates the CRC for you. Here we calculate that CRC, read it from the device, and write it back to the device. It's not great, but it works well enough.

* Save the setting map and reset.

Sensor state can then be read from another register. This is implemented in `touch_task.c`.

### Audio

Pinout:

| audio-board | RP2040                      | 
| ---         | ---                         |
| 3V3         | 3V3                         |
| GND         | GND                         | 
| BCK         | GP8                         | 
| DIN         | GP7                         | 
| LRC (LRCLK) | GP9                         | 
| XSM (XSMT)  | pull up to 5V to unmute DAC | 
| 5V          | 5V - speaker amp power      | 
| (on the other side:) |
| S_S (speaker shutdown) | pull up to 5V to unmute speaker amp |
| P_T (phone tip) | connected to phone tip switch |
| P_G (phone gain) | NC for now |
| P_M (phone mute) | pull up to 3V to unmute headphone amp |



The DAC takes audio samples over I2S. The RP2040 doesn't have a built-in I2S device, but the PIO cores can be (ab)used to act as an I2S device, and the RPi Foundation has an implementation of this in their `pico-extras` libraries. Usage of this implementation is copied from [`pico-playground`](https://github.com/raspberrypi/pico-playground/tree/master/audio/sine_wave), and can be found in `audio_task.c`.

## Software

[FreeRTOS](https://docs.freertos.org/Documentation/00-Overview) is used to manage the tasks of talking to the touch controller and outputting samples to the DAC. 

* The entry point, `main()` in `main.c`, sets up `touch_task`, `audio_task`, and the `key_event_queue` used for inter-task communication, and starts the scheduler.
* `touch_task` sets up the touch controller and polls it for touch sensor values, sending it as an `uint16_t` over `key_event_queue`
* `audio_task` sets up the RP2040's PIO device for I2S, and transmits samples to the DAC. Samples are generated in `calculate_samples`, which takes in sensor values received from `key_event_queue`. 

### Audio framework

Samples are fed into the PIO device through a DMA block. Every time `audio_task` runs, it calculates a block of samples and writes it to the DMA buffer. Most of the DMA stuff is abstracted away with the `pico/audio` libraries, so we only need to worry about generating samples to write to the buffer.

### Sample generation

Sample generation is fairly basic for now:

* The frequency to output is looked up from a frequency table (monophonic only; no chords here)
* Counters are used to keep track of progress through attack and decay envelopes, which increase and decrease the volume
* Another counter is used to keep track of phase progress through a waveform, which is also updated per sample
    * This counter is reset at the end of each waveform duration, which is frequency dependent. In effect, 0 < phase < 1. (or 2pi or whatever) 
* Value is calculated using any periodic function (sine, map directly to produce a sawtooth, ...), scaled and returned

# TODO

* [ ] Octave adjustment
* [ ] Toggle through different tones
* [ ] More, fancier tones (FM synthesis? filters??)
* [ ] Also how are we gonna provide envelope values, etc?