/*
ADC controlled DDS tone generator
with keypad mute / record / playback

Slide potentiometer:
ADC 0 ~ 4095
maps to
frequency 0 ~ 10 kHz
*/

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "pt_cornell_rp2040_v1_4.h"

// ==========================================
// ADC
// ==========================================
#define ADC_PIN 26
#define ADC_MUX 0

// ==========================================
// DDS parameters
// ==========================================
#define two32 4294967296.0
#define Fs 50000
#define DELAY 20

volatile unsigned int phase_accum_main = 0;
volatile unsigned int phase_incr_main = 0;

volatile float current_frequency = 0.0f;

// ==========================================
// Amplitude / mute
// ==========================================
volatile float amplitude = 1.0f;
volatile float target_amplitude = 1.0f;

volatile int mute = 0;

#define AMP_STEP 0.1f

// ==========================================
// DAC / SPI
// ==========================================
uint16_t DAC_data;

#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define SPI_PORT spi0

// ==========================================
// LED
// ==========================================
#define LED_PIN 25

// ==========================================
// ISR timing GPIO
// ==========================================
#define ISR_GPIO 2

// ==========================================
// Alarm
// ==========================================
#define ALARM_NUM 0

#define ALARM_IRQ timer_hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)

// ==========================================
// Sine table
// ==========================================
#define sine_table_size 256

volatile int sin_table[sine_table_size];

// ==========================================
// Keypad
// ==========================================
#define BASE_KEYPAD_PIN 9
#define KEYROWS 4
#define NUMKEYS 12

unsigned int keycodes[NUMKEYS] = {0x57, 0x6E, 0x5E, 0x3E, 0x6D, 0x5D,
                                  0x3D, 0x6B, 0x5B, 0x3B, 0x67, 0x37};

unsigned int scancodes[KEYROWS] = {0xE, 0xD, 0xB, 0x7};

unsigned int button = 0x70;

// ==========================================
// Debounce states
// ==========================================
#define NOT_PRESSED 0
#define MAYBE_PRESSED 1
#define PRESSED 2
#define MAYBE_NOT_PRESSED 3

// ==========================================
// System modes
// ==========================================
#define MODE_SYNTH 0
#define MODE_RECORD_READY 1
#define MODE_RECORDING 2
#define MODE_PLAYBACK 3

volatile int system_mode = MODE_SYNTH;

// ==========================================
// Recording
// ==========================================

// 100 Hz recording
// 500 samples = about 5 seconds
#define MAX_RECORD_SAMPLES 500

float recorded_frequency[10][MAX_RECORD_SAMPLES];

int recorded_length[10] = {0};

volatile int recording_key = -1;
volatile int record_index = 0;

// ==========================================
// Playback
// ==========================================
volatile int playback_key = -1;

// ==========================================
// DDS ISR
// ==========================================
static void alarm_irq(void) {
  // --------------------------------------
  // ISR timing signal HIGH
  // --------------------------------------
  gpio_put(ISR_GPIO, 1);

  // Clear alarm IRQ
  hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

  // Schedule next interrupt
  timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

  // Update DDS phase
  phase_accum_main += phase_incr_main;

  // Lookup sine table
  int sample = (int)(amplitude * sin_table[phase_accum_main >> 24]);

  // Convert signed sine sample
  // into 12-bit DAC value
  DAC_data = DAC_config_chan_A | ((sample + 2048) & 0x0fff);

  // Send to DAC
  spi_write16_blocking(SPI_PORT, &DAC_data, 1);

  // --------------------------------------
  // ISR timing signal LOW
  // --------------------------------------
  gpio_put(ISR_GPIO, 0);
}

// ==========================================
// ADC protothread
// ==========================================
static PT_THREAD(protothread_adc(struct pt* pt)) {
  PT_BEGIN(pt);

  static unsigned int adc_val;

  while (1) {
    // Toggle LED
    gpio_put(LED_PIN, !gpio_get(LED_PIN));

    // ----------------------------------
    // Read potentiometer
    // ----------------------------------
    adc_val = adc_read();

    // ADC 0-4095
    // -> frequency 0-10000 Hz
    current_frequency = (float)adc_val * 10000.0f / 4095.0f;

    // ----------------------------------
    // Potentiometer controls frequency
    // except during playback
    // ----------------------------------
    if (system_mode == MODE_SYNTH || system_mode == MODE_RECORD_READY ||
        system_mode == MODE_RECORDING) {
      phase_incr_main = (unsigned int)(current_frequency * two32 / Fs);
    }

    // ----------------------------------
    // Smooth amplitude ramp
    // ----------------------------------
    if (amplitude < target_amplitude) {
      amplitude += AMP_STEP;

      if (amplitude > target_amplitude) {
        amplitude = target_amplitude;
      }
    } else if (amplitude > target_amplitude) {
      amplitude -= AMP_STEP;

      if (amplitude < target_amplitude) {
        amplitude = target_amplitude;
      }
    }

    // Debug output
    printf("ADC: %u, Frequency: %.2f Hz, Amp: %.2f, Mode: %d\n", adc_val,
           current_frequency, amplitude, system_mode);

    // 20 ms = 50 Hz update
    PT_YIELD_usec(20000);
  }

  PT_END(pt);
}

// ==========================================
// Keypad protothread
// ==========================================
static PT_THREAD(protothread_keypad(struct pt* pt)) {
  PT_BEGIN(pt);

  static int i;
  static uint32_t keypad;

  static int debounce_state = NOT_PRESSED;
  static int stored_key = -1;

  while (1) {
    // ==================================
    // Scan keypad
    // ==================================
    for (i = 0; i < KEYROWS; i++) {
      // Drive one row LOW
      // and others HIGH
      gpio_put_masked(0xF << BASE_KEYPAD_PIN, scancodes[i] << BASE_KEYPAD_PIN);

      // Small settling delay
      sleep_us(1);

      // Read GPIO 9-15
      keypad = (gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F;

      // Check if any column went LOW
      if ((~keypad) & button) {
        break;
      }
    }

    // ==================================
    // Determine key
    // ==================================
    if ((~keypad) & button) {
      for (i = 0; i < NUMKEYS; i++) {
        if (keypad == keycodes[i]) {
          break;
        }
      }

      if (i == NUMKEYS) {
        i = -1;
      }
    } else {
      i = -1;
    }

    // ==================================
    // Debounce state machine
    // ==================================
    switch (debounce_state) {
      // --------------------------------
      // No valid key pressed
      // --------------------------------
      case NOT_PRESSED:

        if (i != -1) {
          stored_key = i;
          debounce_state = MAYBE_PRESSED;
        }

        break;

      // --------------------------------
      // Maybe a valid press
      // --------------------------------
      case MAYBE_PRESSED:

        if (i == stored_key) {
          // Confirmed press
          debounce_state = PRESSED;
          printf("Key %d pressed\n", stored_key);

          // =========================
          // Begin recording
          // if we are waiting for
          // a key 1-9
          // =========================
          if (system_mode == MODE_RECORD_READY && stored_key >= 1 &&
              stored_key <= 9) {

            system_mode = MODE_RECORDING;
            recording_key = stored_key;
            record_index = 0;
            
            printf("Start recording key %d\n", recording_key);
          }
        } else {
          // Probably bounce
          debounce_state = NOT_PRESSED;
        }

        break;

      // --------------------------------
      // Confirmed pressed
      // --------------------------------
      case PRESSED:

        if (i != stored_key) {
          debounce_state = MAYBE_NOT_PRESSED;
        }

        break;

      // --------------------------------
      // Maybe released
      // --------------------------------
      case MAYBE_NOT_PRESSED:

        if (i == stored_key) {
          // Bounce:
          // actually still pressed
          debounce_state = PRESSED;
        } else {
          // =========================
          // Confirmed release
          // =========================
          printf("Key %d released\n", stored_key);

          // -------------------------
          // Key 0
          // mute toggle
          // -------------------------
          if (stored_key == 0) {
            mute = !mute;

            if (mute) {
              target_amplitude = 0.0f;

              printf("MUTE\n");
            } else {
              target_amplitude = 1.0f;

              printf("UNMUTE\n");
            }
          }

          // -------------------------
          // * key
          // enter Record Mode
          //
          // According to this keypad:
          // key index 10 = *
          // -------------------------
          else if (stored_key == 10) {
            system_mode = MODE_RECORD_READY;

            printf("RECORD MODE READY\n");
          }

          // -------------------------
          // Finished recording
          // -------------------------
          else if (system_mode == MODE_RECORDING &&
                   stored_key == recording_key) {
            recorded_length[recording_key] = record_index;

            printf("Finished recording key %d, samples = %d\n", recording_key,
                   record_index);

            system_mode = MODE_SYNTH;

            recording_key = -1;
            record_index = 0;
          }

          // -------------------------
          // Playback keys 1-9
          // -------------------------
          else if (system_mode == MODE_SYNTH && stored_key >= 1 &&
                   stored_key <= 9) {
            // Only playback if
            // this key has data
            if (recorded_length[stored_key] > 0) {
              playback_key = stored_key;

              system_mode = MODE_PLAYBACK;

              printf("Start playback key %d\n", playback_key);
            } else {
              printf("Key %d has no recording\n", stored_key);
            }
          }

          // Reset debounce
          stored_key = -1;

          debounce_state = NOT_PRESSED;
        }

        break;
    }

    // Approximately 33 Hz keypad scan
    PT_YIELD_usec(30000);
  }

  PT_END(pt);
}

// ==========================================
// Recording protothread
// ==========================================
static PT_THREAD(protothread_record(struct pt* pt)) {
  PT_BEGIN(pt);

  while (1) {
    if (system_mode == MODE_RECORDING) {
      if (record_index < MAX_RECORD_SAMPLES) {
        recorded_frequency[recording_key][record_index] = current_frequency;

        record_index++;
      }
    }

    // 10 ms
    // = approximately 100 Hz
    PT_YIELD_usec(10000);
  }

  PT_END(pt);
}

// ==========================================
// Playback protothread
// ==========================================
static PT_THREAD(protothread_playback(struct pt* pt)) {
  PT_BEGIN(pt);

  static int playback_index;
  static float playback_frequency;

  while (1) {
    // ==================================
    // Start playback
    // ==================================
    if (system_mode == MODE_PLAYBACK) {
      playback_index = 0;

      while (playback_index < recorded_length[playback_key]) {
        // Read recorded frequency
        playback_frequency = recorded_frequency[playback_key][playback_index];

        // Convert frequency back
        // into DDS phase increment
        phase_incr_main = (unsigned int)(playback_frequency * two32 / Fs);

        playback_index++;

        // Playback at same 100 Hz
        // rate used when recording
        PT_YIELD_usec(10000);
      }

      printf("Playback finished: key %d\n", playback_key);

      // Return to normal synth
      system_mode = MODE_SYNTH;

      playback_key = -1;
    }

    PT_YIELD_usec(1000);
  }

  PT_END(pt);
}

// ==========================================
// main
// ==========================================
int main() {
  // --------------------------------------
  // stdio
  // --------------------------------------
  stdio_init_all();
  printf("ADC + Keypad DDS Synth\n");

  // ======================================
  // ADC setup
  // ======================================
  adc_init();
  adc_gpio_init(ADC_PIN);
  adc_select_input(ADC_MUX);

  // ======================================
  // SPI setup
  // ======================================
  spi_init(SPI_PORT, 20000000);
  spi_set_format(SPI_PORT, 16, 0, 0, 0);
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

  // ======================================
  // LED
  // ======================================
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put(LED_PIN, true);

  // ======================================
  // ISR timing GPIO
  // ======================================
  gpio_init(ISR_GPIO);
  gpio_set_dir(ISR_GPIO, GPIO_OUT);
  gpio_put(ISR_GPIO, 0);

  // ======================================
  // Keypad setup
  // ======================================

  // Initialize GPIO 9-15
  gpio_init_mask(0x7F << BASE_KEYPAD_PIN);

  // Column pins:
  // GPIO 13, 14, 15
  // inputs
  gpio_set_dir(BASE_KEYPAD_PIN + 4, GPIO_IN);
  gpio_set_dir(BASE_KEYPAD_PIN + 5, GPIO_IN);
  gpio_set_dir(BASE_KEYPAD_PIN + 6, GPIO_IN);

  // Row pins:
  // GPIO 9,10,11,12
  // outputs
  gpio_set_dir_out_masked(0xF << BASE_KEYPAD_PIN);

  // Default all rows HIGH
  gpio_put_masked(0xF << BASE_KEYPAD_PIN,
                  0xF << BASE_KEYPAD_PIN);

  // Column pull-ups
  gpio_pull_up(BASE_KEYPAD_PIN + 4);
  gpio_pull_up(BASE_KEYPAD_PIN + 5);
  gpio_pull_up(BASE_KEYPAD_PIN + 6);

  // ======================================
  // Build sine lookup table
  // ======================================
  for (int ii = 0; ii < sine_table_size; ii++) {
    sin_table[ii] =
        (int)(2047 * sin((float)ii * 6.283 / (float)sine_table_size));
  }

  // ======================================
  // Timer interrupt
  // ======================================
  hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
  irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
  irq_set_enabled(ALARM_IRQ, true);
  timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

  // ======================================
  // Protothreads
  // ======================================
  pt_add_thread(protothread_adc);
  pt_add_thread(protothread_keypad);
  pt_add_thread(protothread_record);
  pt_add_thread(protothread_playback);

  // Start scheduler
  pt_schedule_start;
}