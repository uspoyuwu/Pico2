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

// Physics parameters. Variables rather than constants, because week 3
// makes them adjustable from the encoder.
fix15 gravity = float2fix15(0.75);

// Coefficient of restitution: 1.0 = perfectly elastic, 0.0 = no bounce
fix15 bounciness = float2fix15(0.5);

// Limits, so a few turns of the encoder cannot produce a board that
// does nothing. At zero bounciness balls stick to the pegs; at zero
// gravity they never fall.
#define MIN_BOUNCINESS float2fix15(0.05)
#define MAX_BOUNCINESS float2fix15(0.95)
#define MIN_GRAVITY float2fix15(0.05)
#define MAX_GRAVITY float2fix15(2.00)

// One detent of the encoder moves either parameter by this much.
#define PARAM_STEP float2fix15(0.01)

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
// Measured, not guessed. With row-pruned collisions, erase-only drawing
// and a dirty-flagged readout, the deadline was first missed at 1624
// balls on hardware -- against about 1700 predicted from the per-ball
// cost at 156 balls, so the cost is linear and there is no cache cliff.
//
// Memory is not what stops it. Each ball needs 28 bytes -- four fix15
// values, its last peg, and two frames of trail -- so this costs 45 kB
// against roughly 174 kB free once the VGA driver has taken its 307 kB.
// There is room for four times as many; the CPU is the limit.
#define MAX_BALLS 1624
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

// Where each ball was actually painted on the last two frames, so those
// pixels can be erased rather than clearing the whole screen. Two frames
// of history because the driver may be page-flipping; see the draw
// section of the animation thread.
#define TRAIL_FRAMES 2

short trail_x[TRAIL_FRAMES][MAX_BALLS];
short trail_y[TRAIL_FRAMES][MAX_BALLS];
int trail_count[TRAIL_FRAMES];

// Counts down the frames still owing a full repaint of the static
// scene. Set to TRAIL_FRAMES so that a page-flipping driver receives it
// in every buffer; anything that invalidates the screen sets it again.
int rebuild_scene = TRAIL_FRAMES;

// Same idea for the two parameter lines of the readout, which are
// expensive to format and almost never change. Week 3 will make them
// adjustable; setting this to TRAIL_FRAMES is all that will be needed
// to get the new value on screen.
int params_dirty = TRAIL_FRAMES;

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

// Index of the first peg of each row. Row r holds r+1 pegs, so its
// first index is r(r+1)/2 -- but a lookup is clearer at the point of
// use than that expression inline, and it costs 16 bytes.
int row_base[NUM_ROWS];

// ================================================================
// === Rotary encoder
// ================================================================

#define ROTARY_A_PIN 26
#define ROTARY_B_PIN 27

// Accumulated detents since the animation thread last looked.
//
// A delta rather than a value, because the encoder no longer drives one
// fixed quantity: the button selects which parameter it adjusts, and
// they have different ranges and step sizes. The thread converts the
// delta into whatever is currently selected.
volatile int32_t encoder_delta = 0;

// Which parameter the encoder adjusts. Advanced by the push-button.
enum {
  PARAM_BALLS = 0,
  PARAM_BOUNCINESS,
  PARAM_GRAVITY,
  PARAM_COUNT
};

int selected_param = PARAM_BALLS;

// ================================================================
// === Push-button
// ================================================================

#define BUTTON_PIN 28

void initButton() {
  gpio_init(BUTTON_PIN);
  gpio_set_dir(BUTTON_PIN, GPIO_IN);

  // Pull-up, so an open switch reads high and a closed one reads low.
  //
  // No debounce code and no interrupt: the button is sampled once a
  // frame, and a 16.7 ms sampling interval is longer than the few
  // milliseconds a contact spends chattering, so the frame rate filters
  // it for free. (The encoder cannot be handled this way -- a fast turn
  // would be missed between samples -- which is why that one is on an
  // interrupt.)
  gpio_pull_up(BUTTON_PIN);
}

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

// ~93 ms per ring. Length is a real trade-off: a longer ring sounds
// fuller but keeps the DMA channel busy, and with a hundred balls
// striking pegs constantly most hits would then land on a busy channel
// and fall silent. Short is also correct here -- a sleigh bell is a
// bright shake, not a tolling bell.
#define BELL_SAMPLES 4096

// Three different clusters, picked at random per hit. A single repeated
// sample reads as one machine clicking; three read as a handful of
// bells on a strap.
#define NUM_BELLS 3

// Sleigh bells sit far higher than a musical bell -- these are C7, D7
// and E7. Dropping an octave turns the sound into a glockenspiel.
static const float bell_pitch[NUM_BELLS] = {2093.0f, 2349.0f, 2637.0f};

// A sleigh bell is never one bell. Each buffer stacks three of them at
// slightly different sizes, which beat against each other and produce
// the shimmer that a single ring cannot.
#define BELL_CLUSTER 3

static const float cluster_detune[BELL_CLUSTER] = {1.00f, 1.12f, 1.27f};

// A bell is not a harmonic series. What makes metal sound like metal is
// that its partials sit at irrational ratios to the fundamental, and
// that the high ones fade first -- the strike is bright, the tail is
// dark. Integer ratios would sound like an organ pipe instead. These
// ratios are packed closer together than a tubular bell's, which is
// what gives a small bell its dense, jangling tone.
#define NUM_PARTIALS 5

//                                                ratio  amp   decay
static const float partial_ratio[NUM_PARTIALS] = {1.00f, 1.27f, 1.61f,
                                                  2.04f, 2.58f};
static const float partial_amp[NUM_PARTIALS] = {1.00f, 0.70f, 0.50f, 0.35f,
                                                0.25f};
static const float partial_decay[NUM_PARTIALS] = {5.0f, 6.0f, 7.5f, 9.0f,
                                                  11.0f};

// The loose metal pellet rattling inside the shell. Without this the
// attack is too clean and the result sounds synthetic; a few
// milliseconds of noise underneath the strike is most of what sells it
// as a real bell.
#define NOISE_AMP 0.45f
#define NOISE_DECAY 50.0f

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

    row_base[r] = n;

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

  // Build the bells once, at boot. Each is a cluster of three shells,
  // every shell a sum of inharmonic partials with its own decay rate,
  // plus a short noise burst for the pellet. Nothing here runs again at
  // animation time.
  //
  // This takes a moment -- roughly a third of a million sinf calls per
  // pass -- so expect a brief pause before the display appears.
  for (int b = 0; b < NUM_BELLS; b++) {
    // Two passes over the same expression. The first measures the
    // loudest point: partials start in phase but drift apart, so the
    // peak lands somewhere unpredictable in the first few milliseconds.
    // Measuring it rather than guessing lets the bell use the DAC's
    // full range without ever clipping.
    float peak = 0.0f;
    float scale = 1.0f;

    for (int pass = 0; pass < 2; pass++) {
      // The pellet noise has to be identical in both passes or the
      // second would be scaled against a peak that is not its own, so
      // the generator is reseeded to the same value each time.
      uint32_t noise_state = 0xC0FFEEu + (uint32_t)b;

      for (int i = 0; i < BELL_SAMPLES; i++) {
        float t = (float)i / (float)BELL_SAMPLES;
        float seconds = (float)i / (float)AUDIO_RATE;

        // Envelopes depend on the partial and the time, not on which
        // shell of the cluster, so they are worth hoisting out of the
        // inner loop -- five expf calls per sample instead of fifteen.
        float envelope[NUM_PARTIALS];
        for (int p = 0; p < NUM_PARTIALS; p++) {
          envelope[p] = expf(-partial_decay[p] * t);
        }

        float value = 0.0f;

        for (int c = 0; c < BELL_CLUSTER; c++) {
          float base = bell_pitch[b] * cluster_detune[c];

          for (int p = 0; p < NUM_PARTIALS; p++) {
            // Each mode starts at its own phase. With every partial
            // starting at zero they all swing positive together, the
            // waveform ends up lopsided, and half the DAC's range goes
            // unused -- measured, the negative side only reached about
            // half as far as the positive one. Real bell modes are not
            // phase-locked either, so offsetting them is both louder
            // and closer to the physics.
            float offset =
                2.0f * 3.14159265f * (float)((c * 3 + p * 5) % 7) / 7.0f;

            float phase =
                2.0f * 3.14159265f * base * partial_ratio[p] * seconds +
                offset;

            value += partial_amp[p] * envelope[p] * sinf(phase);
          }
        }

        // Pellet rattle: a few milliseconds of noise under the strike.
        noise_state ^= noise_state << 13;
        noise_state ^= noise_state >> 17;
        noise_state ^= noise_state << 5;

        float noise = (float)(int32_t)noise_state / 2147483648.0f;

        value += NOISE_AMP * expf(-NOISE_DECAY * t) * noise;

        if (pass == 0) {
          float magnitude = value < 0.0f ? -value : value;
          if (magnitude > peak) peak = magnitude;
        } else {
          int sample = (int)(2047.0f + scale * value);

          if (sample < 0) sample = 0;
          if (sample > 4095) sample = 4095;

          DAC_data[b][i] = DAC_config_chan_A | (sample & 0x0fff);
        }
      }

      if (pass == 0) {
        if (peak < 0.0001f) peak = 1.0f;  // never divide by zero
        scale = 1900.0f / peak;
      }
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
    encoder_delta--;
  } else {
    encoder_delta++;
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

void drawStar() {
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

void drawPegs() {
  for (int n = 0; n < NUM_PEGS; n++) {
    fillCircle(peg_px[n], peg_py[n], PEG_RADIUS, peg_colour[n]);
  }

  drawStar();
}

// Repaint the peg the erase of a ball at (px, py) could have bitten
// into. Without this, a ball that passes over a peg leaves a black
// notch in it that never heals, because the pegs are no longer redrawn
// every frame.
//
// Only one peg is ever checked. The closest two pegs in the lattice sit
// 27.7 px apart, and a point within REPAIR_RADIUS of two of them would
// need them within 2 * REPAIR_RADIUS = 18 px, so a ball can overlap at
// most one. Rounding to the nearest row and column therefore finds the
// only candidate there is -- verified by sweeping every pixel of the
// board against a full search, with no misses.
//
// The overlap test still matters: most erases happen in open space and
// repair nothing at all.
#define REPAIR_RADIUS (PEG_RADIUS + BALL_RADIUS + 1)

static void repairPegsNear(int px, int py) {
  // Round rather than truncate, so a ball just above a row snaps to
  // that row instead of the one before it. Negative values truncate
  // towards zero in C, which lands at or below 0, and the clamp covers
  // that.
  int r = (py - PEG_TOP_Y + ROW_SPACING / 2) / ROW_SPACING;

  if (r < 0) r = 0;
  if (r > NUM_ROWS - 1) r = NUM_ROWS - 1;

  int row_left = BOARD_CENTRE_X - (r * PEG_SPACING) / 2;

  int c = (px - row_left + PEG_SPACING / 2) / PEG_SPACING;

  if (c < 0) c = 0;
  if (c > r) c = r;

  int n = row_base[r] + c;

  int dx = px - peg_px[n];
  int dy = py - peg_py[n];

  if (dx * dx + dy * dy > REPAIR_RADIUS * REPAIR_RADIUS) return;

  fillCircle(peg_px[n], peg_py[n], PEG_RADIUS, peg_colour[n]);

  // The star is drawn on top of peg 0, so repainting that peg wipes
  // the spikes.
  if (n == 0) drawStar();
}

// Balls are drawn as small squares, not circles.
//
// fillCircle at radius 2 covers 13 pixels but measured at about 6.8 us
// a call -- roughly a thousand cycles, or 78 cycles a pixel. Almost all
// of that is the function working out spans for a shape this small. A
// single fillRect paints the same area as one run and, at four pixels
// across, the difference is invisible on screen.
#define BALL_DRAW_SIZE (BALL_RADIUS * 2)

static inline void paintBall(int px, int py, char colour) {
  fillRect(px - BALL_RADIUS, py - BALL_RADIUS, BALL_DRAW_SIZE, BALL_DRAW_SIZE,
           colour);
}

static void eraseBallAt(int px, int py) {
  paintBall(px, py, BLACK);

  repairPegsNear(px, py);
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
// Height each bar was last computed at, and how many frames it still
// owes a repaint.
//
// A bar cannot be updated by painting just the difference. If the
// driver page-flips, the buffer being drawn into holds what was drawn
// two frames ago, so a slice painted on top of "what is already there"
// is painted on top of the wrong thing and the old bar shows through.
// Repainting the whole column instead needs no knowledge of what was
// there before, and doing it for TRAIL_FRAMES frames puts it into every
// buffer.
int previous_bar_height[NUM_BINS];
int bar_dirty[NUM_BINS];

void drawHistogram() {
  // Tallest bin sets the scale
  uint32_t peak = 1;  // never zero, so the division below is always safe
  for (int b = 0; b < NUM_BINS; b++) {
    if (bin_count[b] > peak) peak = bin_count[b];
  }

  for (int b = 0; b < NUM_BINS; b++) {
    // 64-bit intermediate: counts can grow large over a long run and
    // count * HIST_MAX_HEIGHT would overflow 32 bits eventually.
    int h = (int)(((uint64_t)bin_count[b] * HIST_MAX_HEIGHT) / peak);

    // A bin with any balls in it should be visible
    if (h < 1 && bin_count[b] > 0) h = 1;

    if (h != previous_bar_height[b]) {
      previous_bar_height[b] = h;
      bar_dirty[b] = TRAIL_FRAMES;
    }

    if (bar_dirty[b] == 0) continue;

    bar_dirty[b]--;

    // One pixel of margin each side so adjacent bars stay distinct
    int x = HIST_LEFT + b * BIN_WIDTH + 1;
    int w = BIN_WIDTH - 2;

    // Clear the full column, then draw the bar. Unconditional, so the
    // result does not depend on what this buffer happened to hold.
    fillRect(x, HIST_BASE_Y - HIST_MAX_HEIGHT, w, HIST_MAX_HEIGHT, BLACK);

    if (h > 0) {
      fillRect(x, HIST_BASE_Y - h, w, h, HIST_COLOUR);
    }
  }
}

// The parts of the screen that never change: background, pegs, star and
// the histogram baseline. Called at boot and whenever the display has to
// be rebuilt from scratch.
void drawStaticScene() {
  clearLowFrame(0, BLACK);

  drawPegs();

  drawHLine(HIST_LEFT, HIST_BASE_Y, NUM_BINS * BIN_WIDTH, HIST_BASE_COLOUR);

  // Nothing is on screen any more, so every bar owes a repaint.
  for (int b = 0; b < NUM_BINS; b++) {
    previous_bar_height[b] = -1;
    bar_dirty[b] = TRAIL_FRAMES;
  }

  // The parameter lines were wiped with everything else, and they are
  // only drawn on demand, so they have to be asked for again.
  params_dirty = TRAIL_FRAMES;
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
  //
  // The old version tested this ball against all 136 pegs every frame.
  // That is almost entirely wasted work: the pegs sit on a regular
  // lattice, so the handful that could possibly be in contact can be
  // computed directly from the ball's position instead of searched for.
  //
  // Row r sits at PEG_TOP_Y + r * ROW_SPACING, so the nearest row is a
  // division. Within a row the pegs are PEG_SPACING apart starting at
  // row_left, so the nearest column is another. Checking one either
  // side of each covers everything a ball can reach in one frame: the
  // widest contact box is CONTACT_RADIUS plus the per-frame travel,
  // around 16 px vertically and 12 horizontally, and the neighbouring
  // row and column are 21 and 36 px away.
  //
  // Worst case is 9 pegs instead of 136.
  int ball_px = fix2int15(ball_x[i]);
  int ball_py = fix2int15(ball_y[i]);

  int centre_row = (ball_py - PEG_TOP_Y) / ROW_SPACING;

  int first_row = centre_row - 1;
  int last_row = centre_row + 1;

  if (first_row < 0) first_row = 0;
  if (last_row > NUM_ROWS - 1) last_row = NUM_ROWS - 1;

  int hit_peg = -1;

  for (int r = first_row; r <= last_row && hit_peg < 0; r++) {
    int row_left = BOARD_CENTRE_X - (r * PEG_SPACING) / 2;

    int centre_col = (ball_px - row_left) / PEG_SPACING;

    int first_col = centre_col - 1;
    int last_col = centre_col + 1;

    // Row r holds r+1 pegs, indices 0..r. Negative divisions truncate
    // towards zero in C, so a ball left of the row can produce a zero
    // or negative column; clamping covers both.
    if (first_col < 0) first_col = 0;
    if (last_col > r) last_col = r;

    for (int c = first_col; c <= last_col; c++) {
      int n = row_base[r] + c;

      fix15 dx = ball_x[i] - peg_x[n];
      fix15 dy = ball_y[i] - peg_y[n];

      // Cheap bounding-box reject before the expensive distance. The
      // box is widened by the current speed, because a fast ball can
      // cross most of the contact region within a single frame.
      fix15 box_x = int2fix15(CONTACT_RADIUS) + absfix15(ball_vx[i]);
      fix15 box_y = int2fix15(CONTACT_RADIUS) + absfix15(ball_vy[i]);

      if (absfix15(dx) >= box_x) continue;
      if (absfix15(dy) >= box_y) continue;

      // Compare squared distances, so the common case of "near but not
      // touching" costs two multiplies instead of a square root. The
      // root is only taken once contact is confirmed, which is rare.
      fix15 distance_squared = multfix15(dx, dx) + multfix15(dy, dy);

      if (distance_squared >= int2fix15(CONTACT_RADIUS * CONTACT_RADIUS)) {
        continue;
      }

      float distance_float = sqrtf(fix2float15(distance_squared));

      fix15 distance = float2fix15(distance_float);

      // Guard against a divide by zero on a dead-centre hit
      if (distance == 0) {
        distance = float2fix15(0.001);
        dx = distance;
      }

      // --- Unit normal, pointing from peg centre to ball ----------
      fix15 normal_x = divfix(dx, distance);
      fix15 normal_y = divfix(dy, distance);

      // --- Push the ball just clear of the peg --------------------
      // Without this the ball can stay inside the peg and re-trigger
      // the collision on every frame.
      fix15 collision_distance = int2fix15(CONTACT_RADIUS + 1);

      ball_x[i] = peg_x[n] + multfix15(normal_x, collision_distance);
      ball_y[i] = peg_y[n] + multfix15(normal_y, collision_distance);

      // --- Tilt the normal ----------------------------------------
      // The true normal is used above to push the ball clear, because
      // that is pure geometry. The reflection below uses a tilted copy,
      // standing in for a peg surface that is not perfectly smooth.
      {
        int t = xorshift32() & (ROUGHNESS_TABLE_SIZE - 1);

        fix15 tc = tilt_cos[t];
        fix15 ts = tilt_sin[t];

        fix15 tilted_x = multfix15(normal_x, tc) - multfix15(normal_y, ts);
        fix15 tilted_y = multfix15(normal_x, ts) + multfix15(normal_y, tc);

        normal_x = tilted_x;
        normal_y = tilted_y;
      }

      // --- Reflect the velocity -----------------------------------
      // Elastic reflection about the contact normal, then damp the
      // whole velocity vector by the coefficient of restitution. This
      // is the lab pseudocode's form.
      //
      // Damping the whole vector rather than the normal component
      // alone is what keeps successive rows independent: it wipes out
      // most of the sideways momentum a ball carries into a contact,
      // so the outcome of one row barely influences the next. Damping
      // only the normal component leaves the tangential speed intact,
      // the walk acquires a memory, and the distribution narrows.
      fix15 normal_velocity =
          multfix15(normal_x, ball_vx[i]) + multfix15(normal_y, ball_vy[i]);

      // Act only while the ball is still moving into the peg,
      // otherwise a grazing contact can be reflected twice.
      if (normal_velocity < 0) {
        fix15 impulse = -multfix15(int2fix15(2), normal_velocity);

        ball_vx[i] = ball_vx[i] + multfix15(normal_x, impulse);
        ball_vy[i] = ball_vy[i] + multfix15(normal_y, impulse);

        ball_vx[i] = multfix15(bounciness, ball_vx[i]);
        ball_vy[i] = multfix15(bounciness, ball_vy[i]);
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
      // same frame would use a position the first contact already
      // moved. Recording the index ends the outer row loop too.
      hit_peg = n;
      break;
    }
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
  ball_vy[i] = ball_vy[i] + gravity;
}

// ================================================================
// === Parameter control
// ================================================================

// Throw away the distribution collected so far.
//
// Changing the physics invalidates every ball already counted: a
// histogram mixing balls dropped at two different bounciness values is
// a picture of neither. The ball count is different -- it changes how
// fast samples arrive, not what is being sampled -- so it does not
// trigger a reset.
void resetStatistics() {
  for (int b = 0; b < NUM_BINS; b++) {
    bin_count[b] = 0;
  }

  total_fallen = 0;

  // Repaint from scratch, which is what actually clears the old bars
  // off the screen.
  rebuild_scene = TRAIL_FRAMES;
}

static fix15 clampFix(fix15 value, fix15 low, fix15 high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

// Apply a number of detents to whichever parameter is selected.
void applyEncoder(int32_t detents) {
  switch (selected_param) {
    case PARAM_BALLS: {
      // A fixed step of one would need 1600 detents to cross the range,
      // so the step grows with the count. The result is roughly
      // constant effort per unit of proportional change.
      int step = 1;
      if (num_balls >= 500) {
        step = 50;
      } else if (num_balls >= 100) {
        step = 10;
      }

      int requested = num_balls + (int)detents * step;

      if (requested < 1) requested = 1;
      if (requested > MAX_BALLS) requested = MAX_BALLS;

      num_balls = requested;
      break;
    }

    case PARAM_BOUNCINESS:
      bounciness = clampFix(bounciness + (fix15)(detents * PARAM_STEP),
                            MIN_BOUNCINESS, MAX_BOUNCINESS);
      resetStatistics();
      break;

    case PARAM_GRAVITY:
      gravity = clampFix(gravity + (fix15)(detents * PARAM_STEP), MIN_GRAVITY,
                         MAX_GRAVITY);
      resetStatistics();
      break;
  }
}

// ================================================================
// === Missed-deadline indicator
// ================================================================

// The Pico 2's on-board LED. Nothing to wire.
#define DEADLINE_LED_PIN 25

// The VGA driver releases a frame every 16667 us, so any frame whose
// work takes longer than that has missed its deadline.
#define FRAME_BUDGET_US 16667

// Once lit the LED stays lit for this many frames, so a single missed
// frame is still visible. At 60 fps this is a third of a second; a
// single-frame flash would be invisible.
#define LED_HOLD_FRAMES 20

void initDeadlineLED() {
  gpio_init(DEADLINE_LED_PIN);
  gpio_set_dir(DEADLINE_LED_PIN, GPIO_OUT);
  gpio_put(DEADLINE_LED_PIN, 0);
}

// How often the whole scene is repainted regardless, in frames. Once a
// second at 60 fps. See the note where it is used.
#define REFRESH_INTERVAL_FRAMES 60

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

  // Where the frame actually goes. Guessing which phase dominates has
  // been wrong twice now -- the collision scan was cut by 35x and the
  // pixel count by 40x, and neither produced the saving those numbers
  // implied -- so measure the three phases separately.
  static uint32_t physics_start;
  static uint32_t draw_start;
  static uint32_t text_start;

  static uint32_t physics_us;
  static uint32_t draw_us;
  static uint32_t text_us;

  // Last values actually painted, so a line can be skipped when it has
  // not moved.
  static uint32_t drawn_seconds;
  static int clock_dirty;

  // Button edge detection, sampled once a frame.
  static int button_was_down;

  static uint32_t missed_frames;
  static int led_hold;
  static int refresh_countdown;

  missed_frames = 0;
  led_hold = 0;
  refresh_countdown = REFRESH_INTERVAL_FRAMES;

  drawn_seconds = 0xffffffffu;
  clock_dirty = TRAIL_FRAMES;
  button_was_down = 0;

  frame_us = 0;
  worst_us = 0;
  warmup = 0;

  physics_us = 0;
  draw_us = 0;
  text_us = 0;

  while (1) {
    // Wait for the VGA driver to release the next frame
    PT_YIELD_UNTIL(pt, draw_start_signal());

    frame_start = time_us_32();

    // --- Read the button ----------------------------------------
    // Sampled once a frame; a press is the transition from up to down,
    // not the down state, or holding it would cycle continuously.
    int button_down = !gpio_get(BUTTON_PIN);

    if (button_down && !button_was_down) {
      selected_param = (selected_param + 1) % PARAM_COUNT;
      params_dirty = TRAIL_FRAMES;
    }

    button_was_down = button_down;

    // --- Read the encoder ---------------------------------------
    // Take the accumulated detents and subtract exactly what was taken,
    // rather than zeroing: an interrupt landing between the two lines
    // would otherwise be lost.
    int32_t detents = encoder_delta;
    encoder_delta -= detents;

    if (detents != 0) {
      applyEncoder(detents);
      params_dirty = TRAIL_FRAMES;
    }

    // Balls coming back into play start a fresh drop rather than
    // resuming from wherever they were when they were switched off.
    if (num_balls > previous_num_balls) {
      for (int i = previous_num_balls; i < num_balls; i++) {
        spawnBall(i);
      }
    }

    previous_num_balls = num_balls;

    // --- Physics ------------------------------------------------
    physics_start = time_us_32();

    for (int i = 0; i < num_balls; i++) {
      updateBall(i);
    }

    physics_us = time_us_32() - physics_start;

    draw_start = time_us_32();

    // --- Draw ---------------------------------------------------
    //
    // Clearing the whole frame and repainting everything cost more than
    // the physics did. Almost none of it changes: the pegs are fixed,
    // the histogram moves a pixel at a time, and the balls are 156 dots
    // four pixels across on a 640x480 screen.
    //
    // So the background is painted once and left alone, and each frame
    // only erases where the balls were and redraws where they are.
    //
    // The erase covers the frame drawn TRAIL_FRAMES ago, not the one
    // just before. The driver page-flips -- the histogram ghosting
    // proved it -- so the buffer being drawn into now holds what was
    // drawn two frames back, and that is the only stale content in it.
    // Erasing the intervening frame as well would be wasted work on
    // pixels belonging to the other buffer.
    if (rebuild_scene) {
      drawStaticScene();

      // The trail history is deliberately NOT cleared here.
      //
      // A rebuild spans two frames, one per buffer. The second of them
      // clears the other buffer, but the balls the first one drew are
      // still sitting in this one, and the frame after next is the one
      // that has to erase them. Zeroing the history at this point threw
      // those positions away and left that batch of balls on screen for
      // good. Clearing the screen already makes the next erase a no-op
      // where it matters, so there is nothing to gain by it.
      rebuild_scene--;
    } else {
      int stale = TRAIL_FRAMES - 1;

      for (int i = 0; i < trail_count[stale]; i++) {
        eraseBallAt(trail_x[stale][i], trail_y[stale][i]);
      }
    }

    // Shift the history along, then record this frame into slot 0.
    for (int s = TRAIL_FRAMES - 1; s > 0; s--) {
      trail_count[s] = trail_count[s - 1];

      for (int i = 0; i < trail_count[s]; i++) {
        trail_x[s][i] = trail_x[s - 1][i];
        trail_y[s][i] = trail_y[s - 1][i];
      }
    }

    trail_count[0] = num_balls;

    for (int i = 0; i < num_balls; i++) {
      int px = fix2int15(ball_x[i]);
      int py = fix2int15(ball_y[i]);

      trail_x[0][i] = (short)px;
      trail_y[0][i] = (short)py;

      paintBall(px, py, BALL_COLOUR);
    }

    drawHistogram();

    draw_us = time_us_32() - draw_start;

    text_start = time_us_32();

    // --- Readout ------------------------------------------------
    uint32_t ms = to_ms_since_boot(get_absolute_time());

    uint32_t seconds = ms / 1000;
    uint32_t hours = seconds / 3600;
    uint32_t minutes = (seconds / 60) % 60;

    seconds = seconds % 60;

    sprintf(line, "Total particles dropped: %lu", (unsigned long)total_fallen);
    drawTextTiny8(10, 10, line, TEXT_COLOUR, BLACK);

    // All three parameter lines share one dirty flag: they change only
    // when the encoder is turned or the button is pressed, and the
    // selection marker means a button press has to repaint all of them.
    if (params_dirty) {
      params_dirty--;

      sprintf(line, "%c Active particles: %d    ",
              selected_param == PARAM_BALLS ? '>' : ' ', num_balls);
      drawTextTiny8(10, 22, line, TEXT_COLOUR, BLACK);

      sprintf(line, "%c Bounciness: %.2f",
              selected_param == PARAM_BOUNCINESS ? '>' : ' ',
              fix2float15(bounciness));
      drawTextTiny8(10, 34, line, TEXT_COLOUR, BLACK);

      sprintf(line, "%c Gravity: %.2f",
              selected_param == PARAM_GRAVITY ? '>' : ' ',
              fix2float15(gravity));
      drawTextTiny8(10, 46, line, TEXT_COLOUR, BLACK);
    }
    if (seconds != drawn_seconds) {
      drawn_seconds = seconds;
      clock_dirty = TRAIL_FRAMES;
    }

    if (clock_dirty) {
      clock_dirty--;

      sprintf(line, "Time elapsed: %lu:%02lu:%02lu", (unsigned long)hours,
              (unsigned long)minutes, (unsigned long)seconds);
      drawTextTiny8(10, 58, line, TEXT_COLOUR, BLACK);
    }

    // --- frame timing readout -----------------------------------
    // Split by phase, because a single total says nothing about where
    // the time is going. Each figure lags by one frame: it was measured
    // before this text was drawn.
    //
    // FPS is capped by the VGA driver at 60, so the useful number is
    // the microsecond figure: 16667 is the whole budget.
    {
      sprintf(line, "Frame: %lu us   Worst: %lu us", (unsigned long)frame_us,
              (unsigned long)worst_us);
      drawTextTiny8(10, 70, line, TEXT_COLOUR, BLACK);

      sprintf(line, "Physics %lu  Draw %lu  Text %lu  Miss %lu",
              (unsigned long)physics_us, (unsigned long)draw_us,
              (unsigned long)text_us, (unsigned long)missed_frames);
      drawTextTiny8(10, 82, line, TEXT_COLOUR, BLACK);
    }

    text_us = time_us_32() - text_start;

    // --- close the frame ----------------------------------------
    frame_us = time_us_32() - frame_start;

    // The first frames after boot are unrepresentative, so let the
    // system settle before the worst case starts being recorded.
    if (warmup < 120) {
      warmup++;
    } else if (frame_us > worst_us) {
      worst_us = frame_us;
    }

    // --- Missed deadline ----------------------------------------
    // Measured at the very end, so it covers everything the frame did.
    // The hold is what makes it usable: a miss lasts one frame, and a
    // sixtieth of a second of LED is not something an eye can catch.
    if (warmup >= 120 && frame_us > FRAME_BUDGET_US) {
      missed_frames++;
      led_hold = LED_HOLD_FRAMES;

      // Erasing only works while the buffer being drawn holds exactly
      // what was drawn TRAIL_FRAMES ago. Overrun a frame and the thread
      // misses a release, the buffers step out of that relationship,
      // and the balls drawn into the skipped frame are never erased --
      // they stay on screen for good. Rebuilding is the only way back,
      // since there is no way to ask the driver which buffer this is.
      rebuild_scene = TRAIL_FRAMES;
    }

    // Belt and braces. Not every disturbance shows up as an overrun
    // measured here -- an interrupt landing between the deadline check
    // and the next release will not -- so the scene is also rebuilt on
    // a timer. Once a second bounds how long any artefact can survive,
    // and a full repaint amortised over 60 frames is a few thousand
    // pixels a frame against the 300,000 this replaced.
    refresh_countdown--;

    if (refresh_countdown <= 0) {
      refresh_countdown = REFRESH_INTERVAL_FRAMES;
      rebuild_scene = TRAIL_FRAMES;
    }

    if (led_hold > 0) {
      led_hold--;
      gpio_put(DEADLINE_LED_PIN, 1);
    } else {
      gpio_put(DEADLINE_LED_PIN, 0);
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

  initButton();

  initDeadlineLED();

  initDMA();

  pt_add_thread(protothread_anim);

  pt_schedule_start;
}