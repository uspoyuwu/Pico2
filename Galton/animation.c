/**
 * Digital Galton Board - Week 1
 *
 * Based on V. Hunter Adams (vha3@cornell.edu) VGA and DMA audio demos.
 *
 * Week 1 deliverables:
 *  - Rotary encoder controls a number displayed on VGA
 *  - One ball falls onto one peg
 *  - Gravity accelerates the ball downward
 *  - Ball uses collision physics to bounce from the peg
 *  - DMA generates a short sound effect on collision
 *  - Ball respawns at the top after leaving the bottom
 *
 * HARDWARE CONNECTIONS
 *  - GPIO  5 ---> MCP4822 CS
 *  - GPIO  6 ---> MCP4822 SCK
 *  - GPIO  7 ---> MCP4822 SDI (MOSI)
 *  - GPIO 10 ---> Rotary encoder channel A
 *  - GPIO 11 ---> Rotary encoder channel B
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> VGA Green lo-bit
 *  - GPIO 19 ---> VGA Green hi-bit
 *  - GPIO 20 ---> VGA Blue
 *  - GPIO 21 ---> VGA Red
 *  - RP2350 GND ---> VGA GND, encoder COM, DAC GND
 *  - RP2350 3V3 ---> DAC VDD (LDAC tied to GND)
 */

// Include the VGA graphics library
#include "VGA/vga16_graphics_v3.h"

// Include standard libraries
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Include Pico libraries
#include "pico/divider.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

// Include hardware libraries
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/pll.h"
#include "hardware/spi.h"

// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// ================================================================
// === Fixed point macros
// ================================================================

typedef signed int fix15;

#define multfix15(a, b) \
  ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))

#define float2fix15(a) ((fix15)((a) * 32768.0))
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)((a) << 15))
#define fix2int15(a) ((int)((a) >> 15))

#define divfix(a, b) \
  (fix15)(div_s64s64((((signed long long)(a)) << 15), ((signed long long)(b))))

// ================================================================
// === Arena geometry
// ================================================================

#define ARENA_LEFT 100
#define ARENA_RIGHT 540
#define ARENA_TOP 100
#define ARENA_BOTTOM 380

#define hitBottom(b) ((b) > int2fix15(ARENA_BOTTOM))
#define hitTop(b) ((b) < int2fix15(ARENA_TOP))
#define hitLeft(a) ((a) < int2fix15(ARENA_LEFT))
#define hitRight(a) ((a) > int2fix15(ARENA_RIGHT))

// ================================================================
// === Galton board parameters
// ================================================================

#define BALL_RADIUS 4
#define PEG_RADIUS 6

// Sum of radii: the centre-to-centre distance at contact
#define CONTACT_RADIUS (BALL_RADIUS + PEG_RADIUS)

#define PEG_X 320
#define PEG_Y 200

// Ball release point
#define SPAWN_X 320
#define SPAWN_Y 110

#define GRAVITY float2fix15(0.37)

// Coefficient of restitution: 1.0 = perfectly elastic, 0.0 = no bounce
#define BOUNCINESS float2fix15(0.5)

// ================================================================
// === Rotary encoder
// ================================================================

#define ROTARY_A_PIN 26
#define ROTARY_B_PIN 27

// A mechanical encoder walks through all four quadrature states
// between physical detents, so four sub-steps make one click.
#define SUBSTEPS_PER_DETENT 4

volatile int encoder_value = 0;

static int encoder_last_state;
static int encoder_accumulator;

// Quadrature transition table, indexed by (previous_state << 2) | current_state
// where state = (A << 1) | B.
//   +1 : clockwise      00 -> 01 -> 11 -> 10 -> 00
//   -1 : counterclockwise
//    0 : no change, or an illegal transition caused by contact bounce
static const int encoder_table[16] = {0,  +1, -1, 0,  -1, 0,  0,  +1,
                                      +1, 0,  0,  -1, 0,  -1, +1, 0};

// ================================================================
// === Audio DMA
// ================================================================

#define PIN_CS 5
#define PIN_SCK 6
#define PIN_SDI 7

#define SPI_PORT spi0

// MCP4822: channel A, 1x gain (2.048 V full scale), output active
#define DAC_config_chan_A 0b0011000000000000

// Target audio sample rate, and the DMA timer used to pace it
#define AUDIO_RATE 44100
#define AUDIO_DMA_TIMER 0

// ~23 ms of audio at 44.1 kHz: long enough for a speaker cone to move
#define SOUND_SAMPLES 1024

// Pitch and decay rate of the "thunk"
#define CLICK_FREQ_HZ 1200.0f
#define CLICK_DECAY 5.0f

unsigned short DAC_data[SOUND_SAMPLES];

int data_chan;

// ================================================================
// === Ball state
// ================================================================

fix15 boid0_x;
fix15 boid0_y;
fix15 boid0_vx;
fix15 boid0_vy;

char color = WHITE;

// Edge detection for the collision sound: holds the index of the peg
// currently being touched, or -1 when the ball is in free flight.
int last_peg = -1;

// ================================================================
// === Create a ball
// ================================================================

void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy, int direction) {
  *x = int2fix15(SPAWN_X);
  *y = int2fix15(SPAWN_Y);

  // Zero initial vertical velocity
  *vy = 0;

  // Small randomised horizontal velocity, in quarter-pixel steps
  // over the range [-1.0, +1.0] pixels per frame, never exactly zero.
  int steps = (rand() % 8) - 4;
  if (steps >= 0) steps += 1;

  *vx = multfix15(int2fix15(steps), float2fix15(0.25));

  // A freshly spawned ball has not touched a peg yet
  last_peg = -1;
}

// ================================================================
// === Drawing helpers
// ================================================================

void drawPeg() { fillCircle(PEG_X, PEG_Y, PEG_RADIUS, WHITE); }

void drawArena() {
  drawVLine(ARENA_LEFT, ARENA_TOP, ARENA_BOTTOM - ARENA_TOP, WHITE);
  drawVLine(ARENA_RIGHT, ARENA_TOP, ARENA_BOTTOM - ARENA_TOP, WHITE);
  drawHLine(ARENA_LEFT, ARENA_TOP, ARENA_RIGHT - ARENA_LEFT, WHITE);
  drawHLine(ARENA_LEFT, ARENA_BOTTOM, ARENA_RIGHT - ARENA_LEFT, WHITE);
}

// ================================================================
// === DMA sound effect
// ================================================================

void initDMA() {
  // Initialise SPI at 20 MHz
  spi_init(SPI_PORT, 20000000);

  // 16 bits per transfer, mode 0. The MCP4822 command word is 16 bits,
  // and hardware-managed CS frames each word automatically.
  spi_set_format(SPI_PORT, 16, 0, 0, 0);

  // Map SPI signals to GPIO. The MCP4822 has no data output, so MISO
  // is deliberately left unmapped and GPIO 4 stays free.
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SDI, GPIO_FUNC_SPI);

  // Build the click: a sine burst under an exponentially decaying
  // envelope. Exponential decay reads as a percussive "thunk";
  // a linear ramp sounds like the tone was cut off.
  for (int i = 0; i < SOUND_SAMPLES; i++) {
    float t = (float)i / (float)SOUND_SAMPLES;

    float envelope = expf(-CLICK_DECAY * t);

    float sine =
        sinf(2.0f * 3.14159265f * CLICK_FREQ_HZ * (float)i / (float)AUDIO_RATE);

    int sample = (int)(2047.0f + 1800.0f * envelope * sine);

    if (sample < 0) sample = 0;
    if (sample > 4095) sample = 4095;

    DAC_data[i] = DAC_config_chan_A | (sample & 0x0fff);
  }

  // Claim a channel the VGA driver is not already using
  data_chan = dma_claim_unused_channel(true);

  dma_channel_config c = dma_channel_get_default_config(data_chan);

  channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);

  // Pace the transfers at the audio rate. The timer divides the system
  // clock by (numerator / denominator), so derive the numerator from the
  // clock actually in use rather than assuming a fixed frequency --
  // overclocking later would otherwise shift the pitch silently.
  uint32_t numerator =
      (uint32_t)(((uint64_t)AUDIO_RATE << 16) / clock_get_hz(clk_sys));

  dma_timer_set_fraction(AUDIO_DMA_TIMER, numerator, 0xffff);

  channel_config_set_dreq(&c, dma_get_timer_dreq(AUDIO_DMA_TIMER));

  // Chaining a channel to itself disables chaining, so the buffer plays
  // once per trigger instead of looping forever.
  channel_config_set_chain_to(&c, data_chan);

  dma_channel_configure(
      data_chan, &c,
      &spi_get_hw(SPI_PORT)->dr,  // write address: SPI data register
      DAC_data,                   // read address: the click waveform
      SOUND_SAMPLES,
      false  // do not start yet
  );
}

// Play the click once. Costs the CPU a single register write.
void triggerSound() {
  if (!dma_channel_is_busy(data_chan)) {
    // Rewind the read pointer and start in one operation
    dma_channel_set_read_addr(data_chan, DAC_data, true);
  }
}

// ================================================================
// === Rotary encoder
// ================================================================

void initRotaryEncoder() {
  gpio_init(ROTARY_A_PIN);
  gpio_init(ROTARY_B_PIN);

  gpio_set_dir(ROTARY_A_PIN, GPIO_IN);
  gpio_set_dir(ROTARY_B_PIN, GPIO_IN);

  // The encoder's common pin is grounded, so the inputs need pull-ups
  gpio_pull_up(ROTARY_A_PIN);
  gpio_pull_up(ROTARY_B_PIN);

  encoder_last_state = (gpio_get(ROTARY_A_PIN) << 1) | gpio_get(ROTARY_B_PIN);
  encoder_accumulator = 0;
}

// Sample both channels and advance the state machine. Periodic sampling
// rather than edge interrupts is what absorbs contact bounce: a bounce
// produces a +1 and a -1 that cancel in the accumulator.
void readRotaryEncoder() {
  int current_state = (gpio_get(ROTARY_A_PIN) << 1) | gpio_get(ROTARY_B_PIN);

  if (current_state == encoder_last_state) {
    return;
  }

  int index = (encoder_last_state << 2) | current_state;

  encoder_accumulator += encoder_table[index];

  encoder_last_state = current_state;

  // One physical detent spans a full quadrature cycle, so only report
  // a click once four consistent sub-steps have accumulated.
  if (encoder_accumulator >= SUBSTEPS_PER_DETENT) {
    encoder_value++;
    encoder_accumulator = 0;
  } else if (encoder_accumulator <= -SUBSTEPS_PER_DETENT) {
    encoder_value--;
    encoder_accumulator = 0;
  }
}

// ================================================================
// === Collision physics
// ================================================================

void collisionPhysics(fix15* x, fix15* y, fix15* vx, fix15* vy) {
  // --- Integrate position ---------------------------------------
  *x = *x + *vx;
  *y = *y + *vy;

  // --- Separation from the peg ----------------------------------
  fix15 dx = *x - int2fix15(PEG_X);
  fix15 dy = *y - int2fix15(PEG_Y);

  // Cheap bounding-box reject before the expensive distance.
  // The box is widened by the current speed, because a fast ball can
  // cross most of the contact region within a single frame.
  fix15 box_x = int2fix15(CONTACT_RADIUS) + absfix15(*vx);
  fix15 box_y = int2fix15(CONTACT_RADIUS) + absfix15(*vy);

  if ((absfix15(dx) < box_x) && (absfix15(dy) < box_y)) {
    float distance_float =
        sqrtf(fix2float15(multfix15(dx, dx) + multfix15(dy, dy)));

    fix15 distance = float2fix15(distance_float);

    if (distance < int2fix15(CONTACT_RADIUS)) {
      // Guard against a divide by zero on a dead-centre hit
      if (distance == 0) {
        distance = float2fix15(0.001);
        dx = distance;
      }

      // --- Unit normal, pointing from peg centre to ball -----
      fix15 normal_x = divfix(dx, distance);
      fix15 normal_y = divfix(dy, distance);

      // --- Push the ball just clear of the peg --------------
      // Without this the ball can stay inside the peg and
      // re-trigger the collision on every frame.
      fix15 collision_distance = int2fix15(CONTACT_RADIUS + 1);

      *x = int2fix15(PEG_X) + multfix15(normal_x, collision_distance);
      *y = int2fix15(PEG_Y) + multfix15(normal_y, collision_distance);

      // --- Reflect the velocity -----------------------------
      // v' = v - (1 + e)(n . v) n
      //
      // Only the normal component is scaled by the coefficient of
      // restitution. Scaling the whole vector would also damp the
      // tangential component, which is friction, not bounciness.
      fix15 normal_velocity =
          multfix15(normal_x, *vx) + multfix15(normal_y, *vy);

      // Act only when the ball is still moving into the peg,
      // otherwise a grazing contact can be reflected twice.
      if (normal_velocity < 0) {
        fix15 one_plus_e = int2fix15(1) + BOUNCINESS;

        fix15 impulse = -multfix15(one_plus_e, normal_velocity);

        *vx = *vx + multfix15(normal_x, impulse);
        *vy = *vy + multfix15(normal_y, impulse);
      }

      // --- Sound, once per contact --------------------------
      // The edge detection gates only the sound effect. The
      // reflection above must run on every overlapping frame.
      if (last_peg != 0) {
        triggerSound();
        last_peg = 0;
      }
    }
  }

  // Re-arm the sound once the ball has clearly left the peg
  if ((absfix15(*x - int2fix15(PEG_X)) > int2fix15(CONTACT_RADIUS + 2)) ||
      (absfix15(*y - int2fix15(PEG_Y)) > int2fix15(CONTACT_RADIUS + 2))) {
    last_peg = -1;
  }

  // --- Respawn at the bottom ------------------------------------
  if (hitBottom(*y)) {
    spawnBoid(x, y, vx, vy, 0);
  }

  // --- Arena walls ----------------------------------------------
  if (hitTop(*y)) {
    *vy = -*vy;
    *y = *y + int2fix15(5);
  }

  if (hitRight(*x)) {
    *vx = -*vx;
    *x = *x - int2fix15(5);
  }

  if (hitLeft(*x)) {
    *vx = -*vx;
    *x = *x + int2fix15(5);
  }

  // --- Gravity --------------------------------------------------
  *vy = *vy + GRAVITY;
}

// ================================================================
// === Rotary encoder thread
// ================================================================

static PT_THREAD(protothread_encoder(struct pt* pt)) {
  PT_BEGIN(pt);

  while (1) {
    readRotaryEncoder();

    // 1 kHz sampling: fast enough to catch every quadrature edge
    // by hand, slow enough to cost almost nothing
    PT_YIELD_usec(1000);
  }

  PT_END(pt);
}

// ================================================================
// === Animation thread
// ================================================================

static PT_THREAD(protothread_anim(struct pt* pt)) {
  PT_BEGIN(pt);

  srand(time_us_32());

  spawnBoid(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy, 0);

  while (1) {
    // Wait for the VGA driver to release the next frame
    PT_YIELD_UNTIL(pt, draw_start_signal());

    clearLowFrame(0, BLACK);

    collisionPhysics(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy);

    drawPeg();

    fillCircle(fix2int15(boid0_x), fix2int15(boid0_y), BALL_RADIUS, color);

    drawArena();

    char encoder_string[32];
    sprintf(encoder_string, "Encoder: %d", encoder_value);
    drawTextTiny8(110, 110, encoder_string, WHITE, BLACK);
  }

  PT_END(pt);
}

// ================================================================
// === Main
// ================================================================

int main() {
  set_sys_clock_khz(150000, true);

  stdio_init_all();

  initVGA();

  initRotaryEncoder();

  initDMA();

  pt_add_thread(protothread_encoder);
  pt_add_thread(protothread_anim);

  pt_schedule_start;
}