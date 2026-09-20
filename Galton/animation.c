/**
 * Hunter Adams (vha3@cornell.edu)
 *
 * Digital Galton Board - Week 1
 *
 * Week 1:
 *  - Rotary encoder controls a number displayed on VGA
 *  - One ball falls onto one peg
 *  - Gravity accelerates the ball downward
 *  - Ball uses collision physics to bounce from the peg
 *  - DMA generates a short sound effect on collision
 *  - Ball respawns at the top after leaving the bottom
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> VGA Green lo-bit
 *  - GPIO 19 ---> VGA Green hi-bit
 *  - GPIO 20 ---> VGA Blue
 *  - GPIO 21 ---> VGA Red
 *  - RP2040 GND ---> VGA-GND
 *
 */

// Include the VGA graphics library
#include "VGA/vga16_graphics_v3.h"

// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
#include "pico/sync.h"

// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/spi.h"

// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"


// === the fixed point macros ========================================

typedef signed int fix15 ;

#define multfix15(a,b) \
    ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))

#define float2fix15(a) ((fix15)((a)*32768.0))

#define fix2float15(a) ((float)(a)/32768.0)

#define absfix15(a) abs(a)

#define int2fix15(a) ((fix15)(a << 15))

#define fix2int15(a) ((int)(a >> 15))

#define char2fix15(a) (fix15)(((fix15)(a)) << 15)

#define divfix(a,b) \
    (fix15)(div_s64s64( \
    (((signed long long)(a)) << 15), \
    ((signed long long)(b))))


// Wall detection
#define hitBottom(b) (b>int2fix15(380))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(100))
#define hitRight(a) (a>int2fix15(540))


// ================================================================
// === Galton Board parameters
// ================================================================

// Parameters from Week 1 figure

#define BALL_RADIUS 4
#define PEG_RADIUS 6

#define PEG_X 320
#define PEG_Y 200

#define GRAVITY float2fix15(0.37)
#define BOUNCINESS float2fix15(0.5)


// ================================================================
// === Rotary encoder
// ================================================================

// TEMPORARY:
// Replace these with the actual GPIO pins used by your rotary encoder.
#define ROTARY_A_PIN 10
#define ROTARY_B_PIN 11

int encoder_value = 0 ;


// ================================================================
// === Audio DMA
// ================================================================

// SPI DAC connections from the professor's Audio DMA Demo

#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7

#define SPI_PORT spi0

// DAC configuration
#define DAC_config_chan_A 0b0011000000000000

// Short sound-effect buffer
#define SOUND_SAMPLES 128

unsigned short DAC_data[SOUND_SAMPLES] ;


// DMA channels
int data_chan ;
int ctrl_chan ;

unsigned short *address_pointer = &DAC_data[0] ;


// ================================================================
// === the color of the ball
// ================================================================

char color = WHITE ;


// ================================================================
// === Ball
// ================================================================

// Keep the original professor variable names

fix15 boid0_x ;
fix15 boid0_y ;
fix15 boid0_vx ;
fix15 boid0_vy ;


// ================================================================
// === Peg collision bookkeeping
// ================================================================

int last_peg = -1 ;


// Create a semaphore
semaphore_t draw_semaphore ;


// ================================================================
// === Rotary encoder state
// ================================================================

int encoder_last_state ;


// ================================================================
// === Create a boid
// ================================================================

void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy, int direction)
{
    // Start near the top of the screen
    *x = int2fix15(320) ;
    *y = int2fix15(110) ;

    // Zero initial y velocity
    *vy = int2fix15(0) ;

    // Small randomized x velocity
    int random_velocity = (rand() % 5) - 2 ;

    *vx = int2fix15(random_velocity) ;

    // Prevent exactly zero x velocity
    if (*vx == 0) {
        *vx = int2fix15(1) ;
    }

    // New ball has not hit a peg yet
    last_peg = -1 ;
}


// ================================================================
// === Draw the peg
// ================================================================

void drawPeg()
{
    fillCircle(PEG_X, PEG_Y, PEG_RADIUS, WHITE) ;
}


// ================================================================
// === Draw the boundaries
// ================================================================

void drawArena()
{
    drawVLine(100, 100, 280, WHITE) ;
    drawVLine(540, 100, 280, WHITE) ;
    drawHLine(100, 100, 440, WHITE) ;
    drawHLine(100, 380, 440, WHITE) ;
}


// ================================================================
// === DMA sound effect
// ================================================================

void initDMA()
{
    // Initialize SPI
    spi_init(SPI_PORT, 20000000) ;

    // 16-bit SPI, mode 0
    spi_set_format(SPI_PORT, 16, 0, 0, 0) ;

    // Map SPI signals to GPIO
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI) ;
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI) ;
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI) ;


    // Generate a short decaying sound effect
    for (int i = 0; i < SOUND_SAMPLES; i++) {

        float t = (float)i / (float)SOUND_SAMPLES ;

        // Short sine burst with decreasing amplitude
        float envelope = 1.0f - t ;

        float sine =
            sinf(2.0f * 3.14159265f * 8.0f * t) ;

        int sample =
            (int)(2047.0f + 1800.0f * envelope * sine) ;

        if (sample < 0) sample = 0 ;
        if (sample > 4095) sample = 4095 ;

        DAC_data[i] =
            DAC_config_chan_A | (sample & 0x0fff) ;
    }


    // Claim two DMA channels, following the professor's DMA demo
    data_chan = dma_claim_unused_channel(true) ;
    ctrl_chan = dma_claim_unused_channel(true) ;


    // ------------------------------------------------------------
    // Control channel
    // ------------------------------------------------------------

    dma_channel_config c =
        dma_channel_get_default_config(ctrl_chan) ;

    channel_config_set_transfer_data_size(&c, DMA_SIZE_32) ;

    channel_config_set_read_increment(&c, false) ;

    channel_config_set_write_increment(&c, false) ;

    // Control channel starts the data channel
    channel_config_set_chain_to(&c, data_chan) ;


    dma_channel_configure(
        ctrl_chan,
        &c,
        &dma_hw->ch[data_chan].read_addr,
        &address_pointer,
        1,
        false
    ) ;


    // ------------------------------------------------------------
    // Data channel
    // ------------------------------------------------------------

    dma_channel_config c2 =
        dma_channel_get_default_config(data_chan) ;

    channel_config_set_transfer_data_size(
        &c2,
        DMA_SIZE_16
    ) ;

    channel_config_set_read_increment(
        &c2,
        true
    ) ;

    channel_config_set_write_increment(
        &c2,
        false
    ) ;


    // Audio-rate DMA pacing
    dma_timer_set_fraction(0, 0x0017, 0xffff) ;

    channel_config_set_dreq(
        &c2,
        0x3b
    ) ;


    // IMPORTANT:
    // Do not chain the data channel back to the control channel.
    // This makes the sound effect play once instead of continuously.


    dma_channel_configure(
        data_chan,
        &c2,
        &spi_get_hw(SPI_PORT)->dr,
        DAC_data,
        SOUND_SAMPLES,
        false
    ) ;
}


// ================================================================
// === Trigger one DMA sound effect
// ================================================================

void triggerSound()
{
    // Do not restart the DMA if the previous sound is still playing
    if (!dma_channel_is_busy(data_chan)) {

        // Reset the read address
        dma_hw->ch[data_chan].read_addr =
            (uintptr_t)DAC_data ;

        // Start the data channel
        dma_start_channel_mask(1u << data_chan) ;
    }
}


// ================================================================
// === Rotary encoder initialization
// ================================================================

void initRotaryEncoder()
{
    gpio_init(ROTARY_A_PIN) ;
    gpio_init(ROTARY_B_PIN) ;

    gpio_set_dir(ROTARY_A_PIN, GPIO_IN) ;
    gpio_set_dir(ROTARY_B_PIN, GPIO_IN) ;

    gpio_pull_up(ROTARY_A_PIN) ;
    gpio_pull_up(ROTARY_B_PIN) ;


    encoder_last_state =
        (gpio_get(ROTARY_A_PIN) << 1) |
        gpio_get(ROTARY_B_PIN) ;
}


// ================================================================
// === Read rotary encoder
// ================================================================

void readRotaryEncoder()
{
    int current_state =
        (gpio_get(ROTARY_A_PIN) << 1) |
        gpio_get(ROTARY_B_PIN) ;


    if (current_state != encoder_last_state) {

        // Clockwise transitions
        if ((encoder_last_state == 0b00 &&
             current_state == 0b01) ||

            (encoder_last_state == 0b01 &&
             current_state == 0b11) ||

            (encoder_last_state == 0b11 &&
             current_state == 0b10) ||

            (encoder_last_state == 0b10 &&
             current_state == 0b00)) {

            encoder_value++ ;
        }


        // Counterclockwise transitions
        else if ((encoder_last_state == 0b00 &&
                  current_state == 0b10) ||

                 (encoder_last_state == 0b10 &&
                  current_state == 0b11) ||

                 (encoder_last_state == 0b11 &&
                  current_state == 0b01) ||

                 (encoder_last_state == 0b01 &&
                  current_state == 0b00)) {

            encoder_value-- ;
        }


        encoder_last_state = current_state ;
    }
}


// ================================================================
// === Collision physics
// ================================================================

void collisionPhysics(
    fix15* x,
    fix15* y,
    fix15* vx,
    fix15* vy)
{
    // ------------------------------------------------------------
    // Update position using current velocity
    // ------------------------------------------------------------

    *x = *x + *vx ;
    *y = *y + *vy ;


    // ------------------------------------------------------------
    // Compute distance between ball and peg
    // ------------------------------------------------------------

    fix15 dx =
        *x - int2fix15(PEG_X) ;

    fix15 dy =
        *y - int2fix15(PEG_Y) ;


    // Bounding-box check
    if ((absfix15(dx) <
         int2fix15(BALL_RADIUS + PEG_RADIUS)) &&

        (absfix15(dy) <
         int2fix15(BALL_RADIUS + PEG_RADIUS))) {


        // Distance = sqrt(dx^2 + dy^2)
        float distance_float =
            sqrtf(
                fix2float15(multfix15(dx, dx) +
                            multfix15(dy, dy))
            ) ;


        fix15 distance =
            float2fix15(distance_float) ;


        // --------------------------------------------------------
        // Check for collision
        // --------------------------------------------------------

        if (distance <
            int2fix15(BALL_RADIUS + PEG_RADIUS)) {


            // Avoid divide by zero
            if (distance == 0) {
                distance = int2fix15(1) ;
            }


            // ----------------------------------------------------
            // Normal vector
            // ----------------------------------------------------

            fix15 normal_x =
                divfix(dx, distance) ;

            fix15 normal_y =
                divfix(dy, distance) ;


            // ----------------------------------------------------
            // Collision intermediate term
            // ----------------------------------------------------

            fix15 intermediate_term =
                multfix15(
                    int2fix15(-2),
                    (
                        multfix15(normal_x, *vx) +
                        multfix15(normal_y, *vy)
                    )
                ) ;


            // ----------------------------------------------------
            // Move ball outside collision distance
            // ----------------------------------------------------

            fix15 collision_distance =
                int2fix15(
                    BALL_RADIUS + PEG_RADIUS + 1
                ) ;


            *x =
                int2fix15(PEG_X) +
                multfix15(normal_x, collision_distance) ;

            *y =
                int2fix15(PEG_Y) +
                multfix15(normal_y, collision_distance) ;


            // ----------------------------------------------------
            // Update velocity
            // ----------------------------------------------------

            if (intermediate_term > 0) {

                *vx =
                    *vx +
                    multfix15(
                        normal_x,
                        intermediate_term
                    ) ;

                *vy =
                    *vy +
                    multfix15(
                        normal_y,
                        intermediate_term
                    ) ;
            }


            // ----------------------------------------------------
            // New peg collision
            // ----------------------------------------------------

            if (last_peg != 0) {

                // DMA "thunk"
                triggerSound() ;

                // Remove energy from the ball
                *vx =
                    multfix15(
                        BOUNCINESS,
                        *vx
                    ) ;

                *vy =
                    multfix15(
                        BOUNCINESS,
                        *vy
                    ) ;

                last_peg = 0 ;
            }
        }
    }


    // ------------------------------------------------------------
    // Ball has moved away from peg
    // ------------------------------------------------------------

    if ((absfix15(*x - int2fix15(PEG_X)) >
         int2fix15(BALL_RADIUS + PEG_RADIUS + 2)) ||

        (absfix15(*y - int2fix15(PEG_Y)) >
         int2fix15(BALL_RADIUS + PEG_RADIUS + 2))) {

        last_peg = -1 ;
    }


    // ------------------------------------------------------------
    // Respawn when ball reaches bottom
    // ------------------------------------------------------------

    if (hitBottom(*y)) {

        spawnBoid(
            x,
            y,
            vx,
            vy,
            0
        ) ;
    }


    // ------------------------------------------------------------
    // Bounce from left/right/top
    // ------------------------------------------------------------

    if (hitTop(*y)) {

        *vy = -*vy ;

        *y =
            *y +
            int2fix15(5) ;
    }


    if (hitRight(*x)) {

        *vx = -*vx ;

        *x =
            *x -
            int2fix15(5) ;
    }


    if (hitLeft(*x)) {

        *vx = -*vx ;

        *x =
            *x +
            int2fix15(5) ;
    }


    // ------------------------------------------------------------
    // Apply gravity
    // ------------------------------------------------------------

    *vy =
        *vy +
        GRAVITY ;
}


// ================================================================
// === Rotary encoder thread
// ================================================================

static PT_THREAD (protothread_encoder(struct pt *pt))
{
    PT_BEGIN(pt) ;

    while(1) {

        readRotaryEncoder() ;

        PT_YIELD_usec(1000) ;
    }

    PT_END(pt) ;
}


// ================================================================
// === Animation on core 0
// ================================================================

static PT_THREAD (protothread_anim(struct pt *pt))
{
    PT_BEGIN(pt) ;


    // Seed random number generator
    srand(time_us_32()) ;


    // Spawn one ball
    spawnBoid(
        &boid0_x,
        &boid0_y,
        &boid0_vx,
        &boid0_vy,
        0
    ) ;


    while(1) {

        // Wait for the next VGA frame
        PT_YIELD_UNTIL(
            pt,
            draw_start_signal()
        ) ;


        // Clear the frame
        clearLowFrame(
            0,
            BLACK
        ) ;


        // Update ball physics
        collisionPhysics(
            &boid0_x,
            &boid0_y,
            &boid0_vx,
            &boid0_vy
        ) ;


        // Draw the peg
        drawPeg() ;


        // Draw the ball
        fillCircle(
            fix2int15(boid0_x),
            fix2int15(boid0_y),
            BALL_RADIUS,
            color
        ) ;


        // Draw the boundaries
        drawArena() ;


        // Display rotary encoder value
        char encoder_string[32] ;

        sprintf(
            encoder_string,
            "Encoder: %d",
            encoder_value
        ) ;


        drawTextTiny8(
            110,
            110,
            encoder_string,
            WHITE,
            BLACK
        ) ;
    }

    PT_END(pt) ;
}


// ================================================================
// === Main
// ================================================================

// USE ONLY C-sdk library

int main()
{
    // Overclock
    set_sys_clock_khz(
        150000,
        true
    ) ;


    // Initialize stdio
    stdio_init_all() ;


    // Initialize VGA
    initVGA() ;


    // Initialize rotary encoder
    initRotaryEncoder() ;


    // Initialize DMA audio
    initDMA() ;


    // Add threads
    pt_add_thread(
        protothread_encoder
    ) ;

    pt_add_thread(
        protothread_anim
    ) ;


    // Start scheduler
    pt_schedule_start ;
}
