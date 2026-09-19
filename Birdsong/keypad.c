/*
 * ECE 5730 Lab 1 -- Birdsong synthesizer
 *
 * DDS tone generator driven by a slide potentiometer, with a 4x3 keypad
 * for muting, recording, and playing back frequency envelopes.
 *
 * Controls
 *   slide pot : frequency 0-10 kHz, or playback volume when the switch is on
 *   0         : toggle mute
 *   *         : toggle Record Mode -- then hold 1-9 to record, release to stop
 *   1-9       : play back that key's recording
 *   #         : toggle Compose Mode -- press 1-9 to build a sequence,
 *               press # again to play the sequence back
 *   switch    : repurpose the pot from frequency to volume
 *
 * Timing
 *   synthesis ISR  50 kHz   (GPIO 2 is asserted for the duration)
 *   record/playback 100 Hz
 *   ADC thread      100 Hz
 *   keypad thread   33 Hz
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

// ============================================================
// Pin assignments
// ============================================================
#define ADC_PIN 26  // slide potentiometer wiper
#define ADC_MUX 0   // GPIO 26 is ADC channel 0

#define PIN_MISO 4  // unused: the MCP4822 is write-only
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7  // wires to SDI on the DAC
#define SPI_PORT spi0

#define LED_PIN 25    // heartbeat
#define ISR_GPIO 2    // scope probe point for ISR timing
#define SWITCH_PIN 3  // volume-mode toggle switch, active low

#define BASE_KEYPAD_PIN 9  // GPIO 9-12 rows (out), 13-15 columns (in)

// ============================================================
// DDS
// ============================================================
#define two32 4294967296.0  // 2^32, one full turn of the phase accumulator
#define Fs 50000            // synthesis sample rate
#define DELAY 20            // 1/Fs in microseconds

#define sine_table_size 256

volatile int sin_table[sine_table_size];

volatile unsigned int phase_accum_main = 0;  // owned by the ISR
volatile unsigned int phase_incr_main = 0;  // written by adc or playback thread

volatile float current_frequency = 0.0f;

// ============================================================
// DAC
// ============================================================
uint16_t DAC_data;

#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

// ============================================================
// Amplitude
// ============================================================
// amplitude chases target_amplitude one AMP_STEP per ADC pass so that
// muting ramps over ~200 ms instead of cutting the waveform mid-cycle.
volatile float amplitude = 1.0f;
volatile float target_amplitude = 1.0f;

volatile int mute = 0;

#define AMP_STEP 0.1f

// When the switch is on, the pot sets volume instead of frequency.
volatile int volume_mode = 0;

// ============================================================
// Alarm
// ============================================================
#define ALARM_NUM 0
#define ALARM_IRQ timer_hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)

// ============================================================
// Keypad
// ============================================================
#define KEYROWS 4
#define NUMKEYS 12

// One entry per key: the 7-bit pattern seen on GPIO 9-15 while that key
// is held and its row is being driven low. Index maps to the key face:
// 0-9 are the digits, 10 is *, 11 is #.
unsigned int keycodes[NUMKEYS] = {0x57, 0x6E, 0x5E, 0x3E, 0x6D, 0x5D,
                                  0x3D, 0x6B, 0x5B, 0x3B, 0x67, 0x37};

// Drives exactly one row low at a time.
unsigned int scancodes[KEYROWS] = {0xE, 0xD, 0xB, 0x7};

// Mask covering just the three column bits.
unsigned int button = 0x70;

// A key must read the same across two consecutive scans before a press or
// release is accepted, which rejects the 5-10 ms of contact bounce.
#define NOT_PRESSED 0
#define MAYBE_PRESSED 1
#define PRESSED 2
#define MAYBE_NOT_PRESSED 3

// ============================================================
// System modes
// ============================================================
// A single mode variable rather than a set of flags, so contradictory
// states are unrepresentable.
#define MODE_SYNTH 0
#define MODE_RECORD_READY 1
#define MODE_RECORDING 2
#define MODE_PLAYBACK 3
#define MODE_COMPOSE_READY 4
#define MODE_COMPOSE_PLAYBACK 5

volatile int system_mode = MODE_SYNTH;

// ============================================================
// Recording
// ============================================================
// Frequency envelopes, not waveforms: 100 samples/sec is plenty to
// capture how fast a hand can move the slider.
// Index 0 is unused -- key 0 toggles mute and is never recorded.
#define MAX_RECORD_SAMPLES 500  // 5 seconds at 100 Hz

float recorded_frequency[10][MAX_RECORD_SAMPLES];
int recorded_length[10] = {0};

volatile int recording_key = -1;
volatile int record_index = 0;

// ============================================================
// Playback
// ============================================================
// Skipping PLAYBACK_SPEED samples per pass compresses playback by that
// factor. The lab requires 8-10x so that hand-speed slider sweeps come
// out at birdsong speed.
// playback_index is global because the keypad thread resets it when a
// playback starts, while the playback thread advances it.
#define PLAYBACK_SPEED 8

volatile int playback_key = -1;
volatile int playback_index = 0;

// ============================================================
// Compose
// ============================================================
// A second layer above recording: this stores an order of keys, where
// each key already holds its own frequency envelope.
#define MAX_SEQUENCE 32

int sequence[MAX_SEQUENCE];
int sequence_length = 0;
int sequence_index = 0;

// ============================================================
// Synthesis ISR -- runs every 20 us, preempts every thread
// ============================================================
static void alarm_irq(void) {
  // Assert the scope probe on entry, clear it on exit: the pulse width is
  // the ISR execution time.
  gpio_put(ISR_GPIO, 1);

  hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

  // The alarm is one-shot, so rearm it here or this never fires again.
  timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

  phase_accum_main += phase_incr_main;

  // Top 8 bits index the table; the rest is truncated phase.
  int sample = (int)(amplitude * sin_table[phase_accum_main >> 24]);

  // Table holds -2047..+2047; the DAC wants 0..4095, so shift to offset
  // binary and mask to 12 bits before OR-ing in the config bits.
  DAC_data = DAC_config_chan_A | ((sample + 2048) & 0x0fff);

  spi_write16_blocking(SPI_PORT, &DAC_data, 1);

  gpio_put(ISR_GPIO, 0);
}

// ============================================================
// ADC thread -- 100 Hz
// ============================================================
// Matched to the record and playback threads so the whole chain shares
// one time base: the frequency this thread computes is what the record
// thread samples.
static PT_THREAD(protothread_adc(struct pt* pt)) {
  PT_BEGIN(pt);

  static unsigned int adc_val;

  // The thread runs faster than the serial port can keep up with, so the
  // status line is printed only every PRINT_EVERY passes.
  static int print_counter = 0;
  const int PRINT_EVERY = 10;  // 10 lines per second

  while (1) {
    gpio_put(LED_PIN, !gpio_get(LED_PIN));

    volume_mode = !gpio_get(SWITCH_PIN);

    adc_val = adc_read();
    current_frequency = (float)adc_val * 10000.0f / 4095.0f;

    // The pot means one thing or the other, never both at once.
    if (volume_mode) {
      // No ramp here: the slider already moves continuously.
      mute = 0;
      target_amplitude = (float)adc_val / 4095.0f;
      amplitude = target_amplitude;
    } else if (system_mode == MODE_SYNTH || system_mode == MODE_RECORD_READY ||
               system_mode == MODE_RECORDING) {
      // Playback modes are excluded: the playback thread owns the
      // phase increment while a recording is being replayed.
      phase_incr_main = (unsigned int)(current_frequency * two32 / Fs);
    }

    // Ramp toward the target so mute and unmute don't click.
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

    print_counter++;
    if (print_counter >= PRINT_EVERY) {
      printf("ADC: %u, Frequency: %.2f Hz, Amp: %.2f, Mode: %d\n", adc_val,
             current_frequency, amplitude, system_mode);
      print_counter = 0;
    }

    PT_YIELD_usec(10000);
  }

  PT_END(pt);
}

// ============================================================
// Keypad thread -- 33 Hz
// ============================================================
static PT_THREAD(protothread_keypad(struct pt* pt)) {
  PT_BEGIN(pt);

  static int i;
  static uint32_t keypad;

  static int debounce_state = NOT_PRESSED;
  static int stored_key = -1;

  while (1) {
    // --------------------------------------------------------
    // Scan: drive one row low, read the columns, stop on a hit
    // --------------------------------------------------------
    for (i = 0; i < KEYROWS; i++) {
      gpio_put_masked(0xF << BASE_KEYPAD_PIN, scancodes[i] << BASE_KEYPAD_PIN);

      // Line capacitance needs a moment to settle before reading.
      sleep_us(1);

      keypad = (gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F;

      // Columns idle high, so a zero anywhere means a key is down.
      if ((~keypad) & button) {
        break;
      }
    }

    // --------------------------------------------------------
    // Resolve the pattern to a key index, or -1
    // --------------------------------------------------------
    if ((~keypad) & button) {
      for (i = 0; i < NUMKEYS; i++) {
        if (keypad == keycodes[i]) {
          break;
        }
      }

      // Ran off the end: an unrecognised pattern, e.g. two keys at once.
      if (i == NUMKEYS) {
        i = -1;
      }
    } else {
      i = -1;
    }

    // --------------------------------------------------------
    // Debounce
    // --------------------------------------------------------
    switch (debounce_state) {
      case NOT_PRESSED:
        if (i != -1) {
          stored_key = i;
          debounce_state = MAYBE_PRESSED;
        }
        break;

      case MAYBE_PRESSED:
        if (i == stored_key) {
          debounce_state = PRESSED;
          printf("Key %d pressed\n", stored_key);

          // Recording is the one action that starts on press rather than
          // release, so the held interval is what gets captured.
          if (system_mode == MODE_RECORD_READY && stored_key >= 1 &&
              stored_key <= 9) {
            system_mode = MODE_RECORDING;
            recording_key = stored_key;
            record_index = 0;

            printf("Start recording key %d\n", recording_key);
          }
        } else {
          debounce_state = NOT_PRESSED;
        }
        break;

      case PRESSED:
        if (i != stored_key) {
          debounce_state = MAYBE_NOT_PRESSED;
        }
        break;

      case MAYBE_NOT_PRESSED:
        if (i == stored_key) {
          debounce_state = PRESSED;
        } else {
          // ====================================================
          // Confirmed release -- everything else dispatches here
          // ====================================================
          printf("Key %d released\n", stored_key);

          // ----- any release ends a recording -----
          // Checked first so that pressing * or # mid-recording still
          // closes the recording cleanly, instead of falling into a
          // mode-toggle branch and silently discarding it. Not checking
          // stored_key == recording_key either: brushing a second key
          // would otherwise strand system_mode in MODE_RECORDING with no
          // way out.
          if (system_mode == MODE_RECORDING) {
            if (recording_key >= 1 && recording_key <= 9) {
              recorded_length[recording_key] = record_index;
              printf("Finished recording key %d, samples = %d\n", recording_key,
                     record_index);
            }

            // Outside the guard, so a bad key index can't trap us here.
            system_mode = MODE_SYNTH;
            recording_key = -1;
            record_index = 0;
          }

          // ----- 0: mute -----
          else if (stored_key == 0) {
            mute = !mute;

            if (mute) {
              target_amplitude = 0.0f;
              printf("MUTE\n");
            } else {
              target_amplitude = 1.0f;
              printf("UNMUTE\n");
            }
          }

          // ----- *: record mode -----
          // Ignored while composing so a stray press can't discard the
          // sequence the user has been building.
          else if (stored_key == 10) {
            if (system_mode == MODE_COMPOSE_READY ||
                system_mode == MODE_COMPOSE_PLAYBACK) {
              printf("Ignored: in compose mode\n");
            } else if (system_mode == MODE_RECORD_READY) {
              system_mode = MODE_SYNTH;
              printf("EXIT RECORD MODE\n");
            } else {
              system_mode = MODE_RECORD_READY;
              printf("RECORD MODE READY\n");
            }
          }

          // ----- #: compose mode -----
          // Ignored while in record mode, mirroring how * is ignored while
          // composing: the two modes shouldn't clobber each other.
          else if (stored_key == 11) {
            if (system_mode == MODE_RECORD_READY) {
              printf("Ignored: in record mode\n");
            } else if (system_mode == MODE_COMPOSE_READY) {
              if (sequence_length > 0) {
                system_mode = MODE_COMPOSE_PLAYBACK;
                sequence_index = 0;
                playback_key = sequence[0];
                playback_index = 0;
                printf("Sequence (%d): ", sequence_length);
                for (int k = 0; k < sequence_length; k++) {
                  printf("%d ", sequence[k]);
                }
                printf("\n");
              } else {
                system_mode = MODE_SYNTH;
                printf("Sequence is empty\n");
              }

            } else if (system_mode == MODE_COMPOSE_PLAYBACK) {
              system_mode = MODE_SYNTH;
              printf("COMPOSE PLAYBACK ABORTED\n");
            } else {
              system_mode = MODE_COMPOSE_READY;
              sequence_length = 0;
              printf("COMPOSE MODE READY\n");
            }
          }

          // ----- 1-9 while composing: append to the sequence -----
          // Must come before the playback case below, or composing would
          // trigger playback instead of recording the key.
          else if (system_mode == MODE_COMPOSE_READY && stored_key >= 1 &&
                   stored_key <= 9) {
            if (sequence_length < MAX_SEQUENCE) {
              sequence[sequence_length] = stored_key;
              sequence_length++;
              printf("Added key %d to sequence (%d)\n", stored_key,
                     sequence_length);
            } else {
              printf("Sequence full\n");
            }
          }

          // ----- 1-9 while idle: play that key back -----
          else if (system_mode == MODE_SYNTH && stored_key >= 1 &&
                   stored_key <= 9) {
            if (recorded_length[stored_key] > 0) {
              playback_key = stored_key;
              playback_index = 0;
              system_mode = MODE_PLAYBACK;

              printf("Start playback key %d\n", playback_key);
            } else {
              printf("Key %d has no recording\n", stored_key);
            }
          }

          stored_key = -1;
          debounce_state = NOT_PRESSED;
        }
        break;
    }

    PT_YIELD_usec(30000);
  }

  PT_END(pt);
}

// ============================================================
// Record thread -- 100 Hz
// ============================================================
static PT_THREAD(protothread_record(struct pt* pt)) {
  PT_BEGIN(pt);

  while (1) {
    // Key bounds are checked here too: a write to index -1 would corrupt
    // whatever sits below the array.
    if (system_mode == MODE_RECORDING && recording_key >= 1 &&
        recording_key <= 9) {
      if (record_index < MAX_RECORD_SAMPLES) {
        recorded_frequency[recording_key][record_index] = current_frequency;
        record_index++;
      } else {
        // Stop and say so, rather than silently dropping samples.
        recorded_length[recording_key] = MAX_RECORD_SAMPLES;
        system_mode = MODE_SYNTH;
        recording_key = -1;
        printf("Recording buffer full\n");
      }
    }

    PT_YIELD_usec(10000);
  }

  PT_END(pt);
}

// ============================================================
// Playback thread -- 100 Hz
// ============================================================
// One sample per scheduler pass, not a loop with a yield inside it:
// protothreads resume by jumping to a switch case, so a yield nested in
// a loop would skip both the mode check and the loop condition.
static PT_THREAD(protothread_playback(struct pt* pt)) {
  PT_BEGIN(pt);

  static float playback_frequency;

  while (1) {
    if ((system_mode == MODE_PLAYBACK ||
         system_mode == MODE_COMPOSE_PLAYBACK) &&
        playback_key >= 1 && playback_key <= 9) {
      if (playback_index < recorded_length[playback_key]) {
        playback_frequency = recorded_frequency[playback_key][playback_index];
        phase_incr_main = (unsigned int)(playback_frequency * two32 / Fs);

        playback_index += PLAYBACK_SPEED;
      } else {
        if (system_mode == MODE_COMPOSE_PLAYBACK) {
          // Chain straight into the next key rather than returning to
          // MODE_SYNTH, so the sequence plays as one continuous phrase.
          sequence_index++;

          if (sequence_index < sequence_length) {
            playback_key = sequence[sequence_index];
            playback_index = 0;
          } else {
            system_mode = MODE_SYNTH;
            playback_key = -1;
            printf("Sequence finished\n");
          }
        } else {
          system_mode = MODE_SYNTH;
          printf("Playback finished: key %d\n", playback_key);
          playback_key = -1;
        }
      }
    }

    PT_YIELD_usec(10000);
  }

  PT_END(pt);
}

// ============================================================
// main
// ============================================================
int main() {
  stdio_init_all();
  printf("ADC + Keypad DDS Synth\n");

  // --------------------------------------------------------
  // ADC
  // --------------------------------------------------------
  adc_init();
  adc_gpio_init(ADC_PIN);
  adc_select_input(ADC_MUX);

  // --------------------------------------------------------
  // SPI to the DAC
  // --------------------------------------------------------
  spi_init(SPI_PORT, 20000000);
  spi_set_format(SPI_PORT, 16, 0, 0, 0);
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

  // --------------------------------------------------------
  // Plain GPIO
  // --------------------------------------------------------
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put(LED_PIN, true);

  gpio_init(ISR_GPIO);
  gpio_set_dir(ISR_GPIO, GPIO_OUT);
  gpio_put(ISR_GPIO, 0);

  gpio_init(SWITCH_PIN);
  gpio_set_dir(SWITCH_PIN, GPIO_IN);
  gpio_pull_up(SWITCH_PIN);

  // --------------------------------------------------------
  // Keypad: rows drive, columns read
  // --------------------------------------------------------
  gpio_init_mask(0x7F << BASE_KEYPAD_PIN);

  gpio_set_dir(BASE_KEYPAD_PIN + 4, GPIO_IN);
  gpio_set_dir(BASE_KEYPAD_PIN + 5, GPIO_IN);
  gpio_set_dir(BASE_KEYPAD_PIN + 6, GPIO_IN);

  gpio_set_dir_out_masked(0xF << BASE_KEYPAD_PIN);

  // Idle state: no row driven low.
  gpio_put_masked(0xF << BASE_KEYPAD_PIN, 0xF << BASE_KEYPAD_PIN);

  // Columns float otherwise, so hold them high and let a key pull one low.
  gpio_pull_up(BASE_KEYPAD_PIN + 4);
  gpio_pull_up(BASE_KEYPAD_PIN + 5);
  gpio_pull_up(BASE_KEYPAD_PIN + 6);

  // --------------------------------------------------------
  // Sine table
  // --------------------------------------------------------
  // Built once at startup so the ISR only ever does a lookup.
  for (int ii = 0; ii < sine_table_size; ii++) {
    sin_table[ii] =
        (int)(2047 * sin((float)ii * 6.283 / (float)sine_table_size));
  }

  // --------------------------------------------------------
  // Timer interrupt
  // --------------------------------------------------------
  hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
  irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
  irq_set_enabled(ALARM_IRQ, true);

  // Arming the alarm starts synthesis; it runs independently from here on.
  timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

  // --------------------------------------------------------
  // Threads
  // --------------------------------------------------------
  pt_add_thread(protothread_adc);
  pt_add_thread(protothread_keypad);
  pt_add_thread(protothread_record);
  pt_add_thread(protothread_playback);

  pt_schedule_start;  // never returns
}