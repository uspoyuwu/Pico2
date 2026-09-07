/*
 * ADC controlled DDS tone generator + Keypad Recorder
 *
 * Function:
 *   * : toggle Record Mode
 *   0 : toggle tone ON/OFF
 *   1-9:
 *       Record Mode:
 *          hold key -> record potentiometer frequency
 *          release  -> stop recording
 *
 *       Normal Mode:
 *          press key -> play recording once
 *
 * Frequency is stored at approximately 100 Hz.
 * DDS synthesis runs at 50 kHz.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "hardware/adc.h"

#include "pt_cornell_rp2040_v1_4.h"


// ============================================================
// ADC
// ============================================================
#define ADC_PIN 26
#define ADC_MUX 0

#define SAMPLE_RATE      100
#define SAMPLE_PERIOD_US 10000


// ============================================================
// DDS parameters
// ============================================================
#define two32 4294967296.0
#define Fs    50000
#define DELAY 20

volatile uint32_t phase_accum_main = 0;
volatile uint32_t phase_incr_main  = 0;

volatile uint32_t current_frequency = 0;


// ============================================================
// DAC / SPI
// ============================================================
uint16_t DAC_data;

#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0


// ============================================================
// LED
// ============================================================
#define LED_PIN 25


// ============================================================
// ISR timing GPIO
// ============================================================
#define ISR_GPIO 2


// ============================================================
// Alarm
// ============================================================
#define ALARM_NUM 0
#define ALARM_IRQ timer_hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)


// ============================================================
// Sine table
// ============================================================
#define sine_table_size 256

volatile int sin_table[sine_table_size];


// ============================================================
// Keypad
// ============================================================
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

#define KEY_0    0
#define KEY_1    1
#define KEY_2    2
#define KEY_3    3
#define KEY_4    4
#define KEY_5    5
#define KEY_6    6
#define KEY_7    7
#define KEY_8    8
#define KEY_9    9
#define KEY_STAR 10
#define KEY_HASH 11

unsigned int keycodes[NUMKEYS] = {0x57, 0x6E, 0x5E, 0x3E, 0x6D, 0x5D,
                                  0x3D, 0x6B, 0x5B, 0x3B, 0x67, 0x37};

unsigned int scancodes[KEYROWS] = {0xE, 0xD, 0xB, 0x7};

unsigned int button = 0x70;


// ============================================================
// Recording parameters
// ============================================================
#define MAX_RECORD_SECONDS 10
#define MAX_SAMPLES        (SAMPLE_RATE * MAX_RECORD_SECONDS)


// ============================================================
// Recording memory
// ============================================================
uint16_t recordings[10][MAX_SAMPLES];

uint16_t record_lengths[10] = {0};


// ============================================================
// Recording state
// ============================================================
volatile bool record_mode  = false;
volatile bool recording    = false;
volatile int  record_key   = -1;
volatile int  record_index = 0;


// ============================================================
// Playback state
// ============================================================
volatile bool playing        = false;
volatile int  playback_key   = -1;
volatile int  playback_index = 0;


// ============================================================
// Tone ON/OFF
// ============================================================
volatile bool tone_on = true;


// ============================================================
// Keypad scan
// ============================================================
static int scan_keypad(void)
{
    int i;
    uint32_t keypad = 0x7F;

    for (i = 0; i < KEYROWS; i++) {

        gpio_put_masked((0xF << BASE_KEYPAD_PIN), (scancodes[i] << BASE_KEYPAD_PIN));

        sleep_us(1);

        keypad = (gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F;

        if ((~keypad) & button) {
            break;
        }
    }

    if ((~keypad) & button) {

        for (i = 0; i < NUMKEYS; i++) {
            if (keypad == keycodes[i]) {
                return i;
            }
        }
    }

    return -1;
}


// ============================================================
// Keypad debounce
// ============================================================
#define KEYPAD_SCAN_US 5000
#define DEBOUNCE_COUNT 6

static int debounce_key(int raw_key)
{
    static int candidate_key = -1;
    static int stable_key    = -1;
    static int count         = 0;

    if (raw_key == candidate_key) {
        count++;
    }
    else {
        candidate_key = raw_key;
        count = 1;
    }

    if (count >= DEBOUNCE_COUNT && candidate_key != stable_key) {
        stable_key = candidate_key;
        return stable_key;
    }

    return -2;
}


// ============================================================
// DDS synthesis ISR
// ============================================================
static void alarm_irq(void)
{
    // ISR timing measurement START
    gpio_put(ISR_GPIO, 1);

    // Clear interrupt
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Schedule next interrupt
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // DDS phase accumulator
    phase_accum_main += phase_incr_main;

    // Sine lookup
    DAC_data = DAC_config_chan_A |
               ((sin_table[phase_accum_main >> 24] + 2048) & 0x0fff);

    // Send to DAC
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);

    // ISR timing measurement END
    gpio_put(ISR_GPIO, 0);
}


// ============================================================
// ADC / Recording / Playback thread
// ============================================================
static PT_THREAD(protothread_adc(struct pt *pt))
{
    PT_BEGIN(pt);

    static uint16_t adc_val;
    static uint16_t frequency;

    // LED heartbeat
    static int led_counter = 0;

    // printf heartbeat
    static int printf_counter = 0;

    while (1) {

        // ====================================================
        // LED heartbeat
        // ====================================================
        //
        // Thread runs every 10 ms.
        // 50 * 10 ms = 500 ms.
        //
        led_counter++;

        if (led_counter >= 50) {
            gpio_put(LED_PIN, !gpio_get(LED_PIN));
            led_counter = 0;
        }


        // ====================================================
        // Read ADC
        // ====================================================
        adc_val = adc_read();

        // ADC 0-4095 -> 0-10000 Hz
        frequency = (uint16_t)(((uint32_t)adc_val * 10000) / 4095);


        // ====================================================
        // RECORDING
        // ====================================================
        if (recording) {

            if (record_index < MAX_SAMPLES) {
                recordings[record_key][record_index] = frequency;
                record_index++;
            }
            else {
                recording = false;
                record_lengths[record_key] = MAX_SAMPLES;
                record_key = -1;

                printf("\nMaximum recording length reached.\n");
            }
        }


        // ====================================================
        // PLAYBACK
        // ====================================================
        else if (playing) {

            if (playback_key >= 0 && playback_key < 10) {

                if (playback_index < record_lengths[playback_key]) {
                    frequency = recordings[playback_key][playback_index];
                    playback_index++;
                }
                else {
                    playing      = false;
                    playback_key = -1;
                    frequency    = 0;

                    printf("\nPlayback finished.\n");
                }
            }
        }


        // ====================================================
        // Update DDS
        // ====================================================
        if (!tone_on) {
            phase_incr_main = 0;
        }
        else {
            phase_incr_main = (uint32_t)((double)frequency * two32 / Fs);
        }

        current_frequency = frequency;


        // ====================================================
        // printf heartbeat
        // ====================================================
        //
        // ADC thread runs at 100 Hz.
        // 10 iterations = 100 ms.
        //
        printf_counter++;

        if (printf_counter >= 10) {

            // printf("[RUNNING] ADC=%u  Frequency=%u Hz  "
            //        "Tone=%s  RecordMode=%s  Recording=%s  Playing=%s\n",
            //        adc_val, frequency,
            //        tone_on ? "ON" : "OFF",
            //        record_mode ? "ON" : "OFF",
            //        recording ? "YES" : "NO",
            //        playing ? "YES" : "NO");

            printf_counter = 0;
        }


        // ====================================================
        // 100 Hz update
        // ====================================================
        PT_YIELD_usec(SAMPLE_PERIOD_US);
    }

    PT_END(pt);
}


// ============================================================
// Keypad thread
// ============================================================
static PT_THREAD(protothread_keypad(struct pt *pt))
{
    PT_BEGIN(pt);

    static int raw_key;
    static int event;

    while (1) {

        // Scan keypad
        raw_key = scan_keypad();

        // Debounce
        event = debounce_key(raw_key);


        // ====================================================
        // New debounced event
        // ====================================================
        if (event != -2) {

            // =================================================
            // PRESS EVENT
            // =================================================
            if (event >= 0) {

                // ---------------------------------------------
                // STAR
                // ---------------------------------------------
                if (event == KEY_STAR) {

                    record_mode = !record_mode;

                    recording = false;
                    playing   = false;

                    printf("\n*** Record Mode: %s ***\n",
                           record_mode ? "ON" : "OFF");
                }

                // ---------------------------------------------
                // ZERO
                // ---------------------------------------------
                else if (event == KEY_0) {

                    tone_on = !tone_on;

                    printf("\n*** Tone: %s ***\n", tone_on ? "ON" : "OFF");
                }

                // ---------------------------------------------
                // KEYS 1-9
                // ---------------------------------------------
                else if (event >= KEY_1 && event <= KEY_9) {

                    int key = event;

                    // =========================================
                    // RECORD MODE
                    // =========================================
                    if (record_mode) {

                        playing = false;

                        recording    = true;
                        record_key   = key;
                        record_index = 0;

                        printf("\n*** Recording key %d ***\n", key);
                    }

                    // =========================================
                    // NORMAL MODE
                    // =========================================
                    else {

                        if (record_lengths[key] > 0) {
                            recording      = false;
                            playing        = true;
                            playback_key   = key;
                            playback_index = 0;

                            printf("\n*** Playing key %d ***\n", key);
                        }
                        else {
                            printf("\n*** Key %d has no recording ***\n", key);
                        }
                    }
                }
            }

            // =================================================
            // RELEASE EVENT
            // =================================================
            else if (event == -1) {

                if (recording) {

                    recording = false;

                    if (record_key >= 0 && record_key < 10) {

                        record_lengths[record_key] = record_index;

                        printf("\n*** Finished recording key %d: %d samples ***\n",
                               record_key, record_index);
                    }

                    record_key = -1;
                }
            }
        }


        // ====================================================
        // Wait 5 ms
        // ====================================================
        PT_YIELD_usec(KEYPAD_SCAN_US);
    }

    PT_END(pt);
}


// ============================================================
// MAIN
// ============================================================
int main(void)
{
    // ========================================================
    // STDIO
    // ========================================================
    stdio_init_all();

    printf("\n========================================\n"
           " ADC controlled DDS + Keypad Recorder\n"
           " System starting...\n"
           "========================================\n");


    // ========================================================
    // ADC setup
    // ========================================================
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_MUX);


    // ========================================================
    // SPI setup
    // ========================================================
    spi_init(SPI_PORT, 20000000);
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SPI);


    // ========================================================
    // LED
    // ========================================================
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);


    // ========================================================
    // ISR timing GPIO
    // ========================================================
    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);


    // ========================================================
    // Build sine table
    // ========================================================
    for (int i = 0; i < sine_table_size; i++) {
        sin_table[i] = (int)(2047 * sin((float)i * 6.283185 / sine_table_size));
    }


    // ========================================================
    // Timer interrupt
    // ========================================================
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    irq_set_enabled(ALARM_IRQ, true);

    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;


    // ========================================================
    // Keypad GPIO setup
    // ========================================================
    gpio_init_mask(0x7F << BASE_KEYPAD_PIN);

    // Column pins
    gpio_set_dir(BASE_KEYPAD_PIN + 4, GPIO_IN);
    gpio_set_dir(BASE_KEYPAD_PIN + 5, GPIO_IN);
    gpio_set_dir(BASE_KEYPAD_PIN + 6, GPIO_IN);

    // Row pins
    gpio_set_dir_out_masked(0xF << BASE_KEYPAD_PIN);

    // Initial row state
    gpio_put_masked(0xF << BASE_KEYPAD_PIN, 0xF << BASE_KEYPAD_PIN);

    // Pull-ups on columns
    gpio_pull_up(BASE_KEYPAD_PIN + 4);
    gpio_pull_up(BASE_KEYPAD_PIN + 5);
    gpio_pull_up(BASE_KEYPAD_PIN + 6);


    // ========================================================
    // Add threads
    // ========================================================
    pt_add_thread(protothread_adc);
    pt_add_thread(protothread_keypad);


    // ========================================================
    // Start scheduler
    // ========================================================
    printf("Starting scheduler...\n");

    pt_schedule_start;
}