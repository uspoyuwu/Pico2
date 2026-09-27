/**
 * Digital Galton Board - Week 2
 *
 * Based on V. Hunter Adams (vha3@cornell.edu) VGA and DMA audio demos.
 *
 * Week 2 deliverables (cumulative on week 1):
 *  - 16-row Galton board with at least 10 balls animated
 *  - Each ball sounds a thunk when it strikes a NEW peg
 *  - Each ball respawns at the top after falling through the bottom
 *  - Rotary encoder adjusts the number of balls in flight
 *  - Display shows balls in flight, total fallen since reset, time since boot
 *  - Histogram of the bin each ball lands in, normalised to the space below
 *
 * ENCODER DECODING
 * A single GPIO interrupt source: the falling edge of channel A. Channel B
 * is not an interrupt source; it is read inside the handler, where its
 * level gives the direction of travel.
 *
 * The falling edge is chosen because the contact shorts the pin straight
 * to ground, giving a sharp edge, while the rising edge has to be pulled
 * up through the ~50 kOhm internal pull-up and is slowed badly by any
 * debounce capacitor.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO  5 ---> MCP4822 CS
 *  - GPIO  6 ---> MCP4822 SCK
 *  - GPIO  7 ---> MCP4822 SDI (MOSI)
 *  - GPIO 26 ---> Rotary encoder channel A
 *  - GPIO 27 ---> Rotary encoder channel B
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
#include "hardware/gpio.h"
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
// === Screen and board geometry
// ================================================================

#define SCREEN_W 640
#define SCREEN_H 480

// Playing field. There is no bottom wall: balls are counted into a bin
// and respawned when they cross COUNT_LINE_Y. The side walls line up
// with the outer edges of the bins (320 +/- 17*36/2 = 14 and 626), so a
// ball that does reach a wall is bounced back towards the lattice
// rather than sliding down outside it.
#define ARENA_LEFT 14
#define ARENA_RIGHT 626
#define ARENA_TOP 35

// --- peg lattice ------------------------------------------------
// Row r holds r+1 pegs, so the whole triangle is 1+2+...+16 = 136 pegs.
// Successive rows are offset by half a spacing, which is what makes a
// ball arriving at a peg face a choice of two gaps below it.
#define NUM_ROWS 16
#define NUM_PEGS ((NUM_ROWS * (NUM_ROWS + 1)) / 2)

#define PEG_TOP_Y 45      // y of the single peg in row 0
#define ROW_SPACING 21    // vertical gap between rows
#define PEG_SPACING 36    // horizontal gap between pegs in a row
#define BOARD_CENTRE_X 320

// Bottom row sits at 45 + 15*21 = 360 and spans 320 +/- 7.5*36,
// i.e. x = 50 to 590.
//
// The ratio of the two spacings is what sets how far the distribution
// spreads. A ball has to move half a peg spacing (18 px) sideways per
// row to change column, and it only has the time it takes to fall
// ROW_SPACING to do it. Pack the rows too close and it arrives at the
// next peg barely deflected, the sixteen rows stop behaving like
// sixteen independent trials, and the histogram collapses into the
// middle few bins. 36:21 is roughly the reference implementation's
// ratio and gives a full-width bell.

// --- bins and histogram -----------------------------------------
// A ball makes NUM_ROWS left/right decisions, so there are NUM_ROWS+1
// possible outcomes. Bin centres sit half a spacing off the bottom-row
// peg centres, i.e. under each gap, with one outer bin at each end.
#define NUM_BINS (NUM_ROWS + 1)
#define BIN_WIDTH PEG_SPACING
#define HIST_LEFT (BOARD_CENTRE_X - (NUM_BINS * BIN_WIDTH) / 2)

// Balls are counted and respawned when they cross this line, which sits
// just below the last peg row so balls never overlap the histogram.
#define COUNT_LINE_Y 378

#define HIST_BASE_Y 470                            // baseline of the bars
#define HIST_TOP_Y 386                             // tallest a bar may be
#define HIST_MAX_HEIGHT (HIST_BASE_Y - HIST_TOP_Y) // 84 px of headroom

// ================================================================
// === Galton board parameters (Fig. 2 defaults)
// ================================================================

// A small particle, matching the reference. Smaller balls also pass
// through the 24 px gaps between pegs more freely and cost less to draw
// when hundreds are on screen at once.
#define BALL_RADIUS 2
#define PEG_RADIUS 6

// Sum of radii: the centre-to-centre distance at contact
#define CONTACT_RADIUS (BALL_RADIUS + PEG_RADIUS)

// Release point, just above the single peg of row 0. Kept the same
// distance above it wherever the triangle sits, so the speed at the
// first impact does not change with the layout.
#define SPAWN_X BOARD_CENTRE_X
#define SPAWN_Y (PEG_TOP_Y - 18)

#define GRAVITY float2fix15(0.75)

// Coefficient of restitution: 1.0 = perfectly elastic, 0.0 = no bounce
#define BOUNCINESS float2fix15(0.5)

// Cap on horizontal speed, in pixels per frame.
//
// The triangle widens by half a peg spacing (18 px) on each side per
// row. A ball leaving a peg with zero vertical speed takes
// sqrt(2 * ROW_SPACING / GRAVITY) ~= 7.5 frames to reach the next row,
// so 18 / 7.5 = 2.4 px/frame is what it takes to change column. The cap
// sits a little above that: too tight and the extreme paths that fill
// the tails of the bell get clipped, too loose and a ball can outrun
// the lattice, escape into the open space beside it where nothing
// deflects it, and pile up in an outer bin.
#define MAX_VX float2fix15(3.5)

// ================================================================
// === Balls
// ================================================================

// Upper bound on what the encoder can dial in. The arrays are sized for
// this; how many are actually animated is num_balls.
//
// The reference implementation runs a couple of hundred particles at
// once. At that count the plain nested collision loop below does
// 400 * 136 = 54,000 checks per frame, which is where the week 3
// optimisation work starts: dropping sqrtf, comparing squared
// distances, and only testing pegs in nearby rows.
#define MAX_BALLS 400
#define DEFAULT_BALLS 10

// Structure-of-arrays rather than an array of structs: the physics loop
// walks one field at a time, so keeping each field contiguous is kinder
// to the cache and easier to vectorise later.
fix15 ball_x[MAX_BALLS];
fix15 ball_y[MAX_BALLS];
fix15 ball_vx[MAX_BALLS];
fix15 ball_vy[MAX_BALLS];

// Index of the peg each ball is currently in contact with, or -1 for a
// ball in free flight. Per ball, because the sound must fire once per
// NEW peg and two balls are generally touching different pegs.
int ball_last_peg[MAX_BALLS];

int num_balls = DEFAULT_BALLS;

// Running totals shown on screen
uint32_t total_fallen = 0;
uint32_t bin_count[NUM_BINS];

// ================================================================
// === Palette
// ================================================================

// The peg lattice is already a triangle, so dressing it as a tree costs
// nothing: green pegs for the foliage, a sparse scatter of baubles to
// break up the grid, white balls for falling snow and red bars
// underneath for the presents.
//
// Verify these names against vga16_graphics_v3.h before building. The
// palette is 16 entries built from one red bit, one blue bit and two
// green bits, and the spellings may differ.
//
// The foliage is one flat green. Banding it across the two darker
// levels was tried and abandoned: on this board MED_GREEN is
// indistinguishable from GREEN and DARK_GREEN does not show against
// black at all, which points at the low green bit on GPIO 18 not
// reaching the divider. If that gets fixed, banding is worth
// revisiting -- green is the only channel here with more than one bit,
// so it is the only one that can carry shading.
#define TREE_COLOUR GREEN

#define ORNAMENT_A RED
#define ORNAMENT_B YELLOW

#define STAR_COLOUR YELLOW
#define BALL_COLOUR WHITE
#define HIST_COLOUR RED
#define HIST_BASE_COLOUR WHITE
#define TEXT_COLOUR WHITE

char color = BALL_COLOUR;

// ================================================================
// === Peg lattice
// ================================================================

// Fixed-point copies are what the physics reads; integer copies are what
// the drawing code reads. Both are computed once at boot.
fix15 peg_x[NUM_PEGS];
fix15 peg_y[NUM_PEGS];
int peg_px[NUM_PEGS];
int peg_py[NUM_PEGS];

// Colour is fixed per peg and worked out once at boot. Recomputing the
// banding and ornament pattern for 136 pegs every frame would be pure
// waste -- this way drawing is a straight array read.
char peg_colour[NUM_PEGS];

// ================================================================
// === Rotary encoder
// ================================================================

#define ROTARY_A_PIN 26
#define ROTARY_B_PIN 27

// Written by the interrupt handler, read by the animation thread.
volatile int encoder_value = DEFAULT_BALLS;

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

// ~93 ms per bell. Length is a real trade-off: a longer ring sounds
// fuller but keeps the DMA channel busy, and with a hundred balls
// striking pegs constantly most hits would then land on a busy channel
// and fall silent. For a jingle bell short is also correct -- they are
// bright little shakes, not tubular bells.
#define BELL_SAMPLES 4096

// Three bells at different pitches. A single repeated sample reads as
// one machine clicking; three pitches picked at random read as a
// handful of bells on a string. C6, E flat 6 and G6 make a triad, so
// overlapping rings stay consonant.
#define NUM_BELLS 3

static const float bell_pitch[NUM_BELLS] = {1046.5f, 1244.5f, 1568.0f};

// A bell is not a harmonic series. What makes metal sound like metal is
// that its partials sit at irrational ratios to the fundamental, and
// that the high ones fade first -- the strike is bright, the tail is
// dark. Integer ratios would sound like an organ pipe instead.
#define NUM_PARTIALS 6

//                                              ratio  amp   decay
static const float partial_ratio[NUM_PARTIALS] = {1.00f, 1.83f, 2.41f,
                                                  3.17f, 4.63f, 5.89f};
static const float partial_amp[NUM_PARTIALS] = {1.00f, 0.60f, 0.45f,
                                                0.32f, 0.22f, 0.15f};
static const float partial_decay[NUM_PARTIALS] = {3.0f, 4.0f, 5.5f,
                                                  7.0f, 9.0f, 11.0f};

unsigned short DAC_data[NUM_BELLS][BELL_SAMPLES];

int data_chan;

// ================================================================
// === Surface roughness
// ================================================================

// The peg lattice is staggered so that the gap between two pegs in one
// row sits directly above a peg in the next. A ball falling through a
// gap therefore arrives at the next peg dead centre, where the contact
// normal points straight up, and a perfectly smooth peg reflects it
// straight back up again. Nothing in the geometry decides whether it
// should go left or right, so the ball stays in the centre column and
// most of the sixteen rows never produce a deflection at all. The
// sixteen trials the central limit theorem needs simply do not happen,
// and the histogram collapses into the middle bins.
//
// A real board breaks that symmetry with microscopic irregularity in
// the peg surface and the ball. This models the same thing: at each
// contact the normal is rotated by a small random angle, as if the ball
// had struck a facet rather than an ideal cylinder. The normal stays a
// unit vector, so no energy is created, and each contact becomes an
// independent trial with a random sign -- which is exactly the
// assumption the central limit theorem rests on.
#define ROUGHNESS_TABLE_SIZE 64

// Widest tilt of the contact normal, in radians. 0.52 is 30 degrees.
// Larger values scatter harder and widen the distribution; smaller
// values narrow it and, near zero, restore the degenerate behaviour
// described above.
#define MAX_TILT_RADIANS 0.52f

fix15 tilt_cos[ROUGHNESS_TABLE_SIZE];
fix15 tilt_sin[ROUGHNESS_TABLE_SIZE];

// A 32-bit xorshift. rand() is called several times per collision at
// these particle counts, and it is far too slow for that; this is three
// shifts and three xors.
static uint32_t rng_state = 0x12345678u;

static inline uint32_t xorshift32(void) {
  rng_state ^= rng_state << 13;
  rng_state ^= rng_state >> 17;
  rng_state ^= rng_state << 5;
  return rng_state;
}

// Sweep the whole tilt range once into the table. Picking an entry at
// random then draws a uniform angle, with no trigonometry at run time.
void initRoughness() {
  for (int i = 0; i < ROUGHNESS_TABLE_SIZE; i++) {
    float fraction = (float)i / (float)(ROUGHNESS_TABLE_SIZE - 1);

    float angle = MAX_TILT_RADIANS * (2.0f * fraction - 1.0f);

    tilt_cos[i] = float2fix15(cosf(angle));
    tilt_sin[i] = float2fix15(sinf(angle));
  }
}

// ================================================================
// === Build the peg lattice
// ================================================================

void initPegs() {
  int n = 0;

  for (int r = 0; r < NUM_ROWS; r++) {
    int y = PEG_TOP_Y + r * ROW_SPACING;

    // Row r has r+1 pegs, centred on the board. The (r * PEG_SPACING)/2
    // term is the half-spacing offset that interleaves the rows.
    int row_left = BOARD_CENTRE_X - (r * PEG_SPACING) / 2;

    for (int i = 0; i <= r; i++) {
      int x = row_left + i * PEG_SPACING;

      peg_px[n] = x;
      peg_py[n] = y;
      peg_x[n] = int2fix15(x);
      peg_y[n] = int2fix15(y);

      // --- Colour ---------------------------------------------
      // Flat green foliage, with a sparse scatter of baubles over it.
      // The hash is deterministic, so the pattern is the same on every
      // boot and the physics is untouched -- an ornament is an ordinary
      // peg wearing a different colour. Skipping the top two rows keeps
      // the area around the star clear.
      peg_colour[n] = TREE_COLOUR;

      if (r >= 2 && ((r * 5 + i * 3) % 7) == 0) {
        peg_colour[n] = ((r + i) & 1) ? ORNAMENT_A : ORNAMENT_B;
      }

      n++;
    }
  }
}

// ================================================================
// === Create a ball
// ================================================================

void spawnBall(int i) {
  // A small random offset on the release point as well as on the
  // velocity, so balls do not all follow the same trajectory out of the
  // gate. Most of the randomness now comes from the peg surface, but
  // spreading the release still helps the first couple of rows.
  int offset = (int)(xorshift32() % 7) - 3;  // -3 .. +3 px

  ball_x[i] = int2fix15(SPAWN_X + offset);
  ball_y[i] = int2fix15(SPAWN_Y);

  // Zero initial vertical velocity, per the week 1 specification
  ball_vy[i] = 0;

  // Small randomised horizontal velocity in quarter-pixel steps,
  // never exactly zero.
  int steps = (int)(xorshift32() % 5) - 2;  // -2 .. +2
  if (steps == 0) steps = 1;

  ball_vx[i] = multfix15(int2fix15(steps), float2fix15(0.25));

  // A freshly spawned ball has not touched a peg yet
  ball_last_peg[i] = -1;
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

  // Build the bells once, at boot. Each is a sum of inharmonic partials,
  // every one with its own decay rate, so the tone darkens as it rings
  // out. Nothing here runs again at animation time.
  for (int b = 0; b < NUM_BELLS; b++) {
    // Pass one: find the loudest point. Partials start in phase but
    // drift apart, so the peak lands somewhere unpredictable in the
    // first few milliseconds. Measuring it rather than guessing lets the
    // bell use the DAC's full range without ever clipping.
    float peak = 0.0f;

    for (int i = 0; i < BELL_SAMPLES; i++) {
      float t = (float)i / (float)BELL_SAMPLES;
      float seconds = (float)i / (float)AUDIO_RATE;

      float value = 0.0f;

      for (int p = 0; p < NUM_PARTIALS; p++) {
        float freq = bell_pitch[b] * partial_ratio[p];
        float phase = 2.0f * 3.14159265f * freq * seconds;

        value += partial_amp[p] * expf(-partial_decay[p] * t) * sinf(phase);
      }

      float magnitude = value < 0.0f ? -value : value;
      if (magnitude > peak) peak = magnitude;
    }

    if (peak < 0.0001f) peak = 1.0f;  // never divide by zero

    float scale = 1900.0f / peak;

    // Pass two: write the samples, scaled to fill the range.
    for (int i = 0; i < BELL_SAMPLES; i++) {
      float t = (float)i / (float)BELL_SAMPLES;
      float seconds = (float)i / (float)AUDIO_RATE;

      float value = 0.0f;

      for (int p = 0; p < NUM_PARTIALS; p++) {
        float freq = bell_pitch[b] * partial_ratio[p];
        float phase = 2.0f * 3.14159265f * freq * seconds;

        value += partial_amp[p] * expf(-partial_decay[p] * t) * sinf(phase);
      }

      int sample = (int)(2047.0f + scale * value);

      if (sample < 0) sample = 0;
      if (sample > 4095) sample = 4095;

      DAC_data[b][i] = DAC_config_chan_A | (sample & 0x0fff);
    }
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
  // overclocking in week 3 would otherwise shift the pitch silently.
  uint32_t numerator =
      (uint32_t)(((uint64_t)AUDIO_RATE << 16) / clock_get_hz(clk_sys));

  dma_timer_set_fraction(AUDIO_DMA_TIMER, numerator, 0xffff);

  channel_config_set_dreq(&c, dma_get_timer_dreq(AUDIO_DMA_TIMER));

  // Chaining a channel to itself disables chaining, so the buffer plays
  // once per trigger instead of looping for ever.
  channel_config_set_chain_to(&c, data_chan);

  dma_channel_configure(
      data_chan, &c,
      &spi_get_hw(SPI_PORT)->dr,  // write address: SPI data register
      DAC_data[0],                // read address: the first bell
      BELL_SAMPLES,
      false  // do not start yet
  );
}

// Ring one bell, chosen at random from the three. Costs the CPU two
// register writes.
//
// If the previous bell is still ringing the hit is dropped rather than
// restarting mid-waveform, which would produce an audible click. At high
// particle counts that means most collisions are silent, which is what
// keeps it sounding like bells rather than static.
void triggerSound() {
  if (!dma_channel_is_busy(data_chan)) {
    int b = (int)(xorshift32() % NUM_BELLS);

    // The transfer count reads back as zero once a run finishes, so it
    // has to be reloaded before the channel is retriggered.
    dma_channel_set_trans_count(data_chan, BELL_SAMPLES, false);

    dma_channel_set_read_addr(data_chan, DAC_data[b], true);
  }
}

// ================================================================
// === Rotary encoder
// ================================================================

// Interrupt handler for the falling edge of channel A.
//
// The two channels are 90 degrees out of phase, so at the moment A goes
// low the level of B differs between the two directions of travel, and
// reading B alone resolves direction. B needs no interrupt of its own.
//
// If the count runs backwards on the bench, swap the two branches (or
// swap the A and B wires).
void encoder_isr(uint gpio, uint32_t events) {
  if (gpio_get(ROTARY_B_PIN)) {
    encoder_value--;
  } else {
    encoder_value++;
  }
}

void initRotaryEncoder() {
  gpio_init(ROTARY_A_PIN);
  gpio_init(ROTARY_B_PIN);

  gpio_set_dir(ROTARY_A_PIN, GPIO_IN);
  gpio_set_dir(ROTARY_B_PIN, GPIO_IN);

  // The encoder's common pin is grounded, so both inputs need pull-ups
  gpio_pull_up(ROTARY_A_PIN);
  gpio_pull_up(ROTARY_B_PIN);

  // One interrupt source only: the falling edge of channel A. Channel B
  // stays a plain input, sampled inside the handler.
  gpio_set_irq_enabled_with_callback(ROTARY_A_PIN, GPIO_IRQ_EDGE_FALL, true,
                                     &encoder_isr);
}

// ================================================================
// === Drawing
// ================================================================

void drawPegs() {
  for (int n = 0; n < NUM_PEGS; n++) {
    fillCircle(peg_px[n], peg_py[n], PEG_RADIUS, peg_colour[n]);
  }

  // Peg 0 is the only peg in the top row, so it is the apex. A plain
  // oversized circle there just looked like a bigger peg; crossed spikes
  // through a small core read as a star instead.
  //
  // This is cosmetic only. The physics still sees PEG_RADIUS at this
  // position, so the star deflects balls exactly like its neighbours and
  // the statistics are unaffected.
  int sx = peg_px[0];
  int sy = peg_py[0];

  drawVLine(sx, sy - 8, 17, STAR_COLOUR);
  drawHLine(sx - 8, sy, 17, STAR_COLOUR);

  fillCircle(sx, sy, 4, STAR_COLOUR);
}

// The walls are active in the physics but deliberately not drawn: the
// reference implementation has no visible frame, and a box drawn at
// ARENA_TOP would run straight through the text readout.
void drawArena() {
  drawVLine(ARENA_LEFT, ARENA_TOP, COUNT_LINE_Y - ARENA_TOP, WHITE);
  drawVLine(ARENA_RIGHT, ARENA_TOP, COUNT_LINE_Y - ARENA_TOP, WHITE);
  drawHLine(ARENA_LEFT, ARENA_TOP, ARENA_RIGHT - ARENA_LEFT, WHITE);
}

// Draw the histogram, rescaled so the tallest bar exactly fills the
// available vertical space. Rescaling every frame is what keeps the plot
// readable as the counts grow without bound.
void drawHistogram() {
  // Baseline
  drawHLine(HIST_LEFT, HIST_BASE_Y, NUM_BINS * BIN_WIDTH, HIST_BASE_COLOUR);

  // Tallest bin sets the scale
  uint32_t peak = 1;  // never zero, so the division below is always safe
  for (int b = 0; b < NUM_BINS; b++) {
    if (bin_count[b] > peak) peak = bin_count[b];
  }

  for (int b = 0; b < NUM_BINS; b++) {
    if (bin_count[b] == 0) continue;

    // 64-bit intermediate: counts can grow large over a long run and
    // count * HIST_MAX_HEIGHT would overflow 32 bits eventually.
    int h = (int)(((uint64_t)bin_count[b] * HIST_MAX_HEIGHT) / peak);

    if (h < 1) h = 1;  // a bin with any balls in it should be visible

    // One pixel of margin each side so adjacent bars stay distinct
    fillRect(HIST_LEFT + b * BIN_WIDTH + 1, HIST_BASE_Y - h, BIN_WIDTH - 2, h,
             HIST_COLOUR);
  }
}

// ================================================================
// === Physics
// ================================================================

// Advance one ball by one frame: move it, resolve any peg collision,
// bounce it off the walls, count it into a bin if it has fallen through,
// and apply gravity.
void updateBall(int i) {
  // --- Integrate position ---------------------------------------
  ball_x[i] = ball_x[i] + ball_vx[i];
  ball_y[i] = ball_y[i] + ball_vy[i];

  // --- Peg collisions -------------------------------------------
  for (int n = 0; n < NUM_PEGS; n++) {
    fix15 dx = ball_x[i] - peg_x[n];
    fix15 dy = ball_y[i] - peg_y[n];

    // Cheap bounding-box reject before the expensive distance. The box
    // is widened by the current speed, because a fast ball can cross
    // most of the contact region within a single frame.
    fix15 box_x = int2fix15(CONTACT_RADIUS) + absfix15(ball_vx[i]);
    fix15 box_y = int2fix15(CONTACT_RADIUS) + absfix15(ball_vy[i]);

    if (absfix15(dx) >= box_x) continue;
    if (absfix15(dy) >= box_y) continue;

    float distance_float =
        sqrtf(fix2float15(multfix15(dx, dx) + multfix15(dy, dy)));

    fix15 distance = float2fix15(distance_float);

    if (distance >= int2fix15(CONTACT_RADIUS)) continue;

    // Guard against a divide by zero on a dead-centre hit
    if (distance == 0) {
      distance = float2fix15(0.001);
      dx = distance;
    }

    // --- Unit normal, pointing from peg centre to ball ----------
    fix15 normal_x = divfix(dx, distance);
    fix15 normal_y = divfix(dy, distance);

    // --- Push the ball just clear of the peg --------------------
    // Without this the ball can stay inside the peg and re-trigger the
    // collision on every frame.
    fix15 collision_distance = int2fix15(CONTACT_RADIUS + 1);

    ball_x[i] = peg_x[n] + multfix15(normal_x, collision_distance);
    ball_y[i] = peg_y[n] + multfix15(normal_y, collision_distance);

    // --- Tilt the normal ----------------------------------------
    // The true normal is used above to push the ball clear, because
    // that is pure geometry. The reflection below uses a tilted copy,
    // standing in for a peg surface that is not perfectly smooth.
    {
      int t = xorshift32() & (ROUGHNESS_TABLE_SIZE - 1);

      fix15 c = tilt_cos[t];
      fix15 s = tilt_sin[t];

      fix15 tilted_x = multfix15(normal_x, c) - multfix15(normal_y, s);
      fix15 tilted_y = multfix15(normal_x, s) + multfix15(normal_y, c);

      normal_x = tilted_x;
      normal_y = tilted_y;
    }

    // --- Reflect the velocity -----------------------------------
    // Elastic reflection about the contact normal, then damp the whole
    // velocity vector by the coefficient of restitution. This is the
    // lab pseudocode's form.
    //
    // Damping the whole vector rather than the normal component alone
    // is what keeps successive rows independent: it wipes out most of
    // the sideways momentum a ball carries into a contact, so the
    // outcome of one row barely influences the next. Damping only the
    // normal component leaves the tangential speed intact, the walk
    // acquires a memory, and the distribution narrows.
    fix15 normal_velocity =
        multfix15(normal_x, ball_vx[i]) + multfix15(normal_y, ball_vy[i]);

    // Act only while the ball is still moving into the peg, otherwise a
    // grazing contact can be reflected twice.
    if (normal_velocity < 0) {
      fix15 impulse = -multfix15(int2fix15(2), normal_velocity);

      ball_vx[i] = ball_vx[i] + multfix15(normal_x, impulse);
      ball_vy[i] = ball_vy[i] + multfix15(normal_y, impulse);

      ball_vx[i] = multfix15(BOUNCINESS, ball_vx[i]);
      ball_vy[i] = multfix15(BOUNCINESS, ball_vy[i]);
    }

    // --- Sound, once per NEW peg --------------------------------
    // The reflection above must run on every overlapping frame; only
    // the sound is gated. Holding the peg index rather than a flag is
    // what lets a ball slide from one peg straight onto the next and
    // still sound the second one.
    if (ball_last_peg[i] != n) {
      triggerSound();
      ball_last_peg[i] = n;
    }

    // One peg per frame is enough. Resolving a second contact in the
    // same frame would use a position the first contact already moved.
    break;
  }

  // --- Walls ----------------------------------------------------
  if (ball_y[i] < int2fix15(ARENA_TOP)) {
    ball_vy[i] = -ball_vy[i];
    ball_y[i] = int2fix15(ARENA_TOP + 1);
  }

  if (ball_x[i] < int2fix15(ARENA_LEFT)) {
    ball_vx[i] = -ball_vx[i];
    ball_x[i] = int2fix15(ARENA_LEFT + 1);
  }

  if (ball_x[i] > int2fix15(ARENA_RIGHT)) {
    ball_vx[i] = -ball_vx[i];
    ball_x[i] = int2fix15(ARENA_RIGHT - 1);
  }

  // --- Cap horizontal speed -------------------------------------
  // Applied after every collision and wall bounce, so no path through
  // the frame can leave a ball fast enough to escape the lattice.
  if (ball_vx[i] > MAX_VX) ball_vx[i] = MAX_VX;
  if (ball_vx[i] < -MAX_VX) ball_vx[i] = -MAX_VX;

  // --- Fell through: count it into a bin and respawn -------------
  if (ball_y[i] > int2fix15(COUNT_LINE_Y)) {
    int x = fix2int15(ball_x[i]);

    int b = (x - HIST_LEFT) / BIN_WIDTH;

    if (b < 0) b = 0;
    if (b >= NUM_BINS) b = NUM_BINS - 1;

    bin_count[b]++;
    total_fallen++;

    spawnBall(i);
  }

  // --- Gravity --------------------------------------------------
  // Applied last, so the position update at the top of the next frame
  // uses the velocity from the end of this one (semi-implicit Euler,
  // which stays stable where explicit Euler would gain energy).
  ball_vy[i] = ball_vy[i] + GRAVITY;
}

// ================================================================
// === Animation thread
// ================================================================

static PT_THREAD(protothread_anim(struct pt* pt)) {
  PT_BEGIN(pt);

  srand(time_us_32());

  // Every slot starts out populated; num_balls decides how many are
  // actually animated, so raising the count never shows a stale ball.
  for (int i = 0; i < MAX_BALLS; i++) {
    spawnBall(i);
  }

  static int previous_num_balls;
  previous_num_balls = num_balls;

  static char line[48];

  // --- frame timing -------------------------------------------
  // The VGA driver releases a frame every 16667 us. Anything longer
  // than that in the loop below is a missed deadline.
  static uint32_t frame_start;
  static uint32_t frame_us;
  static uint32_t worst_us;
  static uint32_t warmup;

  frame_us = 0;
  worst_us = 0;
  warmup = 0;

  while (1) {
    // Wait for the VGA driver to release the next frame
    PT_YIELD_UNTIL(pt, draw_start_signal());

    frame_start = time_us_32();

    // --- Read the encoder ---------------------------------------
    // Clamped here rather than in the interrupt handler, so the ISR
    // stays as short as possible.
    int requested = encoder_value;

    if (requested < 1) {
      requested = 1;
      encoder_value = 1;
    }
    if (requested > MAX_BALLS) {
      requested = MAX_BALLS;
      encoder_value = MAX_BALLS;
    }

    // Balls coming back into play start a fresh drop rather than
    // resuming from wherever they were when they were switched off.
    if (requested > previous_num_balls) {
      for (int i = previous_num_balls; i < requested; i++) {
        spawnBall(i);
      }
    }

    num_balls = requested;
    previous_num_balls = requested;

    // --- Physics ------------------------------------------------
    for (int i = 0; i < num_balls; i++) {
      updateBall(i);
    }

    // --- Draw ---------------------------------------------------
    clearLowFrame(0, BLACK);

    drawPegs();

    for (int i = 0; i < num_balls; i++) {
      fillCircle(fix2int15(ball_x[i]), fix2int15(ball_y[i]), BALL_RADIUS,
                 color);
    }

    drawHistogram();

    // --- Readout ------------------------------------------------
    uint32_t ms = to_ms_since_boot(get_absolute_time());

    uint32_t seconds = ms / 1000;
    uint32_t hours = seconds / 3600;
    uint32_t minutes = (seconds / 60) % 60;

    seconds = seconds % 60;

    sprintf(line, "Total particles dropped: %lu", (unsigned long)total_fallen);
    drawTextTiny8(10, 10, line, TEXT_COLOUR, BLACK);

    sprintf(line, "Active particles: %d", num_balls);
    drawTextTiny8(10, 22, line, TEXT_COLOUR, BLACK);

    sprintf(line, "Bounciness: %.2f", fix2float15(BOUNCINESS));
    drawTextTiny8(10, 34, line, TEXT_COLOUR, BLACK);

    sprintf(line, "Gravity: %.2f", fix2float15(GRAVITY));
    drawTextTiny8(10, 46, line, TEXT_COLOUR, BLACK);

    sprintf(line, "Time elapsed: %lu:%02lu:%02lu", (unsigned long)hours,
            (unsigned long)minutes, (unsigned long)seconds);
    drawTextTiny8(10, 58, line, TEXT_COLOUR, BLACK);

    // --- frame timing readout -----------------------------------
    // Measured before this text is drawn, so it lags by one frame.
    // FPS is capped by the VGA driver at 60, so the useful number is
    // the microsecond figure: 16667 is the whole budget.
    {
      uint32_t fps = frame_us ? (1000000u / frame_us) : 0;
      if (fps > 60) fps = 60;

      sprintf(line, "Frame: %lu us  FPS: %lu", (unsigned long)frame_us,
              (unsigned long)fps);
      drawTextTiny8(10, 70, line, TEXT_COLOUR, BLACK);

      sprintf(line, "Worst: %lu us", (unsigned long)worst_us);
      drawTextTiny8(10, 82, line, TEXT_COLOUR, BLACK);
    }

    // --- close the frame ----------------------------------------
    frame_us = time_us_32() - frame_start;

    // The first frames after boot are unrepresentative, so let the
    // system settle before the worst case starts being recorded.
    if (warmup < 120) {
      warmup++;
    } else if (frame_us > worst_us) {
      worst_us = frame_us;
    }
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

  initPegs();

  initRoughness();

  rng_state = time_us_32() | 1u;  // xorshift must never start at zero

  // Registers the GPIO interrupt; the encoder runs entirely from it.
  initRotaryEncoder();

  initDMA();

  pt_add_thread(protothread_anim);

  pt_schedule_start;
}