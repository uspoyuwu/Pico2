/*
ADC controlled DDS tone generator

Slide potentiometer:
ADC 0 ~ 4095
maps to
frequency 0 ~ 10 kHz
*/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "hardware/adc.h"

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
#define Fs    50000
#define DELAY 20

volatile unsigned int phase_accum_main = 0;
volatile unsigned int phase_incr_main  = 0;

// ==========================================
// DAC / SPI
// ==========================================
uint16_t DAC_data;

#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
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
// DDS ISR
// ==========================================
static void alarm_irq(void)
{
    // ISR timing signal
    gpio_put(ISR_GPIO, 1);

    // Clear alarm IRQ
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Schedule next interrupt
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // Update DDS phase
    phase_accum_main += phase_incr_main;

    // Lookup sine value
    DAC_data = DAC_config_chan_A |
               ((sin_table[phase_accum_main >> 24] + 2048) & 0x0fff);

    // Send to DAC
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);

    gpio_put(ISR_GPIO, 0);
}

// ==========================================
// ADC protothread
// ==========================================
static PT_THREAD(protothread_adc(struct pt *pt))
{
    PT_BEGIN(pt);

    static unsigned int adc_val;
    static float frequency;

    while (1)
    {
        // toggle gpio 25
        gpio_put(LED_PIN, !gpio_get(LED_PIN));

        // Read potentiometer
        adc_val = adc_read();

        // ADC 0-4095 -> frequency 0-10000 Hz
        frequency = (float)adc_val * 10000.0f / 4095.0f;

        // Convert frequency to DDS phase increment
        phase_incr_main = (unsigned int)(frequency * two32 / Fs);

        printf("ADC: %u, Frequency: %.2f Hz\n", adc_val, frequency);

        // ADC does NOT need to update at 50 kHz.
        // 20 ms gives responsive potentiometer control.
        PT_YIELD_usec(20000);
    }

    PT_END(pt);
}

// ==========================================
// main
// ==========================================
int main()
{
    // --------------------------------------
    // stdio
    // --------------------------------------
    stdio_init_all();
    printf("ADC controlled DDS\n");

    // --------------------------------------
    // ADC setup
    // --------------------------------------
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_MUX);

    // --------------------------------------
    // SPI setup
    // --------------------------------------
    spi_init(SPI_PORT, 20000000);
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

    // --------------------------------------
    // LED GPIO
    // --------------------------------------
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, true);

    // --------------------------------------
    // ISR timing GPIO
    // --------------------------------------
    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);

    // --------------------------------------
    // Build sine lookup table
    // --------------------------------------
    for (int ii = 0; ii < sine_table_size; ii++)
    {
        sin_table[ii] = (int)(2047 * sin((float)ii * 6.283 / (float)sine_table_size));
    }

    // --------------------------------------
    // Timer interrupt
    // --------------------------------------
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    irq_set_enabled(ALARM_IRQ, true);
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // --------------------------------------
    // Protothread
    // --------------------------------------
    pt_add_thread(protothread_adc);
    pt_schedule_start;
}