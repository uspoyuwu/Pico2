/**
 * PWM LED Brightness Control
 *
 * Based on V. Hunter Adams' PWM demo
 *
 * HARDWARE CONNECTIONS:
 *   - GPIO 6 (pin 9) ---> motor board PWM terminal (or LED)
 *
 * Enter a value between 0 and 6000 through Serial
 * to control the LED brightness.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/uart.h"

#include "pt_cornell_rp2040_v1_4.h"

// PWM configuration for a 150 MHz system clock:
// PWM frequency = 150 MHz / (25 * 6001) ~= 1 kHz
#define WRAPVAL 6000
#define CLKDIV 25.0f

// PWM output on GPIO 6 = physical pin 9 (PWM slice 3, channel A)
#define PWM_OUT 6

// UART0 is used by the Cornell protothread serial helpers.
// Pico UART0 default pins: GPIO 0 = TX, GPIO 1 = RX.
#define UART_BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// PWM slice number
uint slice_num;

// PWM duty cycle control
volatile int control = 0;      // start with the motor OFF
volatile int old_control = 0;


// =====================================================
// PWM Interrupt Service Routine
// =====================================================
void on_pwm_wrap() {

    // Clear PWM interrupt flag
    pwm_clear_irq(slice_num);

    // Update PWM duty cycle if changed
    if (control != old_control) {

        old_control = control;

        // Automatically select PWM Channel A or B
        pwm_set_gpio_level(PWM_OUT, control);
    }
}


// =====================================================
// Serial Input Protothread
// =====================================================
static PT_THREAD(protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);

    static int test_in;

    while (1) {

        // Ask user for duty cycle
        sprintf(
            pt_serial_out_buffer,
            "Input LED brightness (0-6000): "
        );

        serial_write;

        // Non-blocking serial input
        serial_read;

        // Convert string to integer
        if (sscanf(pt_serial_in_buffer, "%d", &test_in) != 1) {
            sprintf(
                pt_serial_out_buffer,
                "Invalid input!\r\n"
            );
            serial_write;
            continue;
        }

        // Validate input range
        if (test_in > WRAPVAL || test_in < 0) {

            sprintf(
                pt_serial_out_buffer,
                "Please enter a value between 0 and 6000.\r\n"
            );

            serial_write;
            continue;
        }

        // Update duty cycle
        control = test_in;

        // Print current brightness
        sprintf(
            pt_serial_out_buffer,
            "PWM Level: %d / %d (%.1f%%)\r\n",
            test_in,
            WRAPVAL,
            (100.0f * test_in) / (WRAPVAL + 1)
        );

        serial_write;
    }

    PT_END(pt);
}


// =====================================================
// Main Function
// =====================================================
int main() {

    // Initialize standard I/O
    stdio_init_all();

    // Configure the UART that serial_read and serial_write use.
    uart_init(uart0, UART_BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);

    // -------------------------------------------------
    // PWM Configuration
    // -------------------------------------------------

    // Set GPIO 6 to PWM function
    gpio_set_function(PWM_OUT, GPIO_FUNC_PWM);

    // Get PWM slice number
    slice_num = pwm_gpio_to_slice_num(PWM_OUT);

    // Clear pending PWM interrupt
    pwm_clear_irq(slice_num);

    // Enable PWM interrupt
    pwm_set_irq_enabled(slice_num, true);

    // Register PWM interrupt handler
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);

    // Enable PWM interrupt
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Configure PWM period
    pwm_set_wrap(slice_num, WRAPVAL);

    // Configure clock divider
    pwm_set_clkdiv(slice_num, CLKDIV);

    // Initial duty cycle = 0 (motor off at boot)
    pwm_set_gpio_level(PWM_OUT, control);

    // Start PWM
    pwm_set_mask_enabled(1u << slice_num);

    // -------------------------------------------------
    // Protothread Configuration
    // -------------------------------------------------

    // Add serial input thread
    pt_add_thread(protothread_serial);

    // Start protothread scheduler
    pt_schedule_start;
}