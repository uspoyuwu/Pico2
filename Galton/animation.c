/**
 * Digital Galton Board -- Week 3
 *
 * Built from the structure of V. Hunter Adams's VGA animation and DMA
 * examples: one protothread advances the simulation once per VGA frame,
 * while a DMA channel sends the short MCP4822 sound effect.
 *
 * GPIO connections
 *   VGA: HSync 16, VSync 17, green 18/19, blue 20, red 21
 *   MCP4822 DAC: CS 5, SCK 6, SDI/MOSI 7
 *   Rotary encoder: A 26, B 27, push-button 28
 *   Deadline LED: 25
 */

#include "VGA/vga16_graphics_v3.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/divider.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/vreg.h"

#include "pt_cornell_rp2040_v1_4.h"

// ================================================================
// === Fixed point helpers (16.15 format)
// ================================================================

typedef signed int fix15;

#define multfix15(a, b) \
  ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define divfix(a, b) \
  ((fix15)div_s64s64(((signed long long)(a)) << 15, (signed long long)(b)))
#define int2fix15(a) ((fix15)((a) << 15))
#define float2fix15(a) ((fix15)((a) * 32768.0f))
#define fix2int15(a) ((int)((a) >> 15))
#define fix2float15(a) ((float)(a) / 32768.0f)
#define absfix15(a) abs(a)

// ================================================================
// === Hardware assignment
// ================================================================

#define DAC_CS_PIN 5
#define DAC_SCK_PIN 6
#define DAC_MOSI_PIN 7
#define DAC_SPI_PORT spi0

#define ENCODER_A_PIN 26
#define ENCODER_B_PIN 27
#define ENCODER_BUTTON_PIN 28
#define DEADLINE_LED_PIN 25

// ================================================================
// === System clock
// ================================================================

// The VGA driver was written against 150 MHz. Raising this is the last
// large gain available: everything in the frame is CPU-bound, so the
// ceiling scales almost linearly with the clock.
//
// WHETHER THIS WORKS DEPENDS ON THE DRIVER. Its three PIO state machines
// have to run at the pixel clock, and they get there by dividing the
// system clock. If the .pio init routines compute that divider from
// clock_get_hz(clk_sys), raising the clock here is all that is needed,
// because main sets the clock before calling initVGA. If instead they
// hard-code a divider for 150 MHz, the picture will roll or lose sync
// and the divider in hsync.pio, vsync.pio and rgb.pio has to be scaled
// by the same factor.
//
// Step this up gradually and watch the screen: 150 (known good), then
// 200, then 250. Nothing else needs changing -- the audio DMA timer
// already derives its divider from clock_get_hz, and time_us_32 runs
// off a separate 1 MHz reference, so the timing figures stay valid.
// 200 MHz was tried and the monitor lost sync: the driver's PIO state
// machines take their clock divider from a value fixed for 150 MHz, so
// raising the system clock raises the pixel clock with it and the signal
// stops meeting the VGA timing the monitor expects.
//
// Making this work means scaling sm_config_set_clkdiv in hsync.pio,
// vsync.pio and rgb.pio by the same factor -- a change to the supplied
// driver, not to this file.
#define SYS_CLOCK_KHZ 150000

// ================================================================
// === Board geometry and parameters
// ================================================================

#define NUM_ROWS 16
#define NUM_PEGS ((NUM_ROWS * (NUM_ROWS + 1)) / 2)
#define NUM_BINS (NUM_ROWS + 1)

#define BOARD_CENTER_X 320
#define PEG_TOP_Y 105
#define ROW_SPACING 17
#define PEG_SPACING 36
#define PEG_RADIUS 6
#define BALL_RADIUS 2
#define CONTACT_RADIUS (PEG_RADIUS + BALL_RADIUS)

#define ARENA_LEFT 14
#define ARENA_RIGHT 626
#define ARENA_TOP 82
#define COUNT_LINE_Y 382
#define SPAWN_X BOARD_CENTER_X
#define SPAWN_Y (PEG_TOP_Y - 18)

#define HIST_LEFT (BOARD_CENTER_X - (NUM_BINS * PEG_SPACING) / 2)
#define HIST_BASE_Y 468
#define HIST_TOP_Y 394
#define HIST_HEIGHT (HIST_BASE_Y - HIST_TOP_Y)

// Measured at 150 MHz: the deadline is first missed at 9450 balls,
// against 9475 predicted from the fixed cost and the per-ball cost, so
// the model holds to a third of a percent. Set just under it.
//
// At 20 bytes a ball the RAM ran out at about the same point. Packing
// the state to 10 bytes moved that limit past 21,000, so the processor
// is now the only thing stopping this -- raising it further needs a
// faster clock, and that needs the driver's PIO dividers changed too.
#define MAX_BALLS 9400
#define INITIAL_BALLS 300

#define MIN_BOUNCINESS float2fix15(0.05f)
#define MAX_BOUNCINESS float2fix15(0.95f)
#define MIN_GRAVITY float2fix15(0.10f)
#define MAX_GRAVITY float2fix15(1.50f)
#define BOUNCINESS_STEP float2fix15(0.05f)
#define GRAVITY_STEP float2fix15(0.05f)
#define MAX_HORIZONTAL_SPEED float2fix15(3.5f)

#define FRAME_BUDGET_US 16667
#define DISPLAY_COLOUR WHITE

// Balls are stored packed and worked on unpacked.
//
// Twenty bytes a ball made RAM the binding constraint once the CPU had
// been sped up. Ten bytes moves the limit from about 10,500 balls to
// over 21,000, past anything the processor can animate.
//
// The stored format is 11.5 fixed point: eleven integer bits cover the
// 640 by 480 screen with room to spare, and five fractional bits give
// 1/32 of a pixel. Six fractional bits would only reach 512 and could
// not hold an x of 639.
//
// Crucially the physics is NOT done in this format. updateBall unpacks
// into ordinary fix15, runs exactly the arithmetic it ran before, and
// packs the result back. That matters because the contact normal is a
// unit vector: its components live in [-1, 1], where five fractional
// bits would leave just 32 distinct values and make every bounce angle
// coarse. Quantising only the stored state costs 1/32 px of position
// and velocity, which nothing can see.
#define STORE_SHIFT 10  // fix15 has 15 fractional bits, stored has 5

typedef struct {
  int16_t x;
  int16_t y;
  int16_t vx;
  int16_t vy;
  int16_t last_peg;
} Ball;

// The unpacked form, used only inside one call to updateBall.
typedef struct {
  fix15 x;
  fix15 y;
  fix15 vx;
  fix15 vy;
  int last_peg;
} BallWork;

static inline void loadBall(const Ball *b, BallWork *w) {
  // Multiply rather than shift: left-shifting a negative value is not
  // defined by the standard, and the compiler emits the same shift.
  w->x = (fix15)b->x * (1 << STORE_SHIFT);
  w->y = (fix15)b->y * (1 << STORE_SHIFT);
  w->vx = (fix15)b->vx * (1 << STORE_SHIFT);
  w->vy = (fix15)b->vy * (1 << STORE_SHIFT);
  w->last_peg = b->last_peg;
}

static inline void storeBall(const BallWork *w, Ball *b) {
  b->x = (int16_t)(w->x >> STORE_SHIFT);
  b->y = (int16_t)(w->y >> STORE_SHIFT);
  b->vx = (int16_t)(w->vx >> STORE_SHIFT);
  b->vy = (int16_t)(w->vy >> STORE_SHIFT);
  b->last_peg = (int16_t)w->last_peg;
}

typedef struct {
  fix15 x;
  fix15 y;
  int px;
  int py;
} Peg;

static Ball balls[MAX_BALLS];
static Peg pegs[NUM_PEGS];
static int row_start[NUM_ROWS];

// Counters are per core.
//
// Both cores run the physics, and both land balls in bins. A shared
// counter would be read-modify-written from two cores at once and would
// silently lose counts -- the histogram would drift low with no visible
// symptom. Each core keeps its own tally and the two are summed when the
// display needs them, which costs 17 additions a frame.
static uint32_t bin_count[2][NUM_BINS];
static uint32_t total_fallen[2];

static uint32_t binTotal(int bin) {
  return bin_count[0][bin] + bin_count[1][bin];
}

static uint32_t droppedTotal(void) {
  return total_fallen[0] + total_fallen[1];
}

static int ball_count = INITIAL_BALLS;
static fix15 bounciness = float2fix15(0.50f);
static fix15 gravity = float2fix15(0.75f);

// ================================================================
// === Frame timing breakdown
// ================================================================

// Six phases, measured separately.
//
// A single total says nothing about where the time goes, and guessing
// has a poor record here. Splitting it answers the question that decides
// how to use the second core: if the fixed cost (clear, pegs, histogram,
// text) dominates, the work should be split by screen region; if the
// per-ball cost (physics, balls) dominates, it should be split by ball
// index.
//
// time_us_32 resolves to one microsecond, so a phase that costs less
// than that reads as 0 or flickers between 0 and 1. That is a real
// answer: the phase is negligible.
//
// File scope rather than local, because drawReadout has to read them.
static uint32_t t_physics;
static uint32_t t_clear;
static uint32_t t_pegs;
static uint32_t t_balls;
static uint32_t t_hist;
static uint32_t t_text;

// ================================================================
// === Random surface roughness
// ================================================================

// Tilting the reflection normal breaks the otherwise perfectly symmetric
// peg collision. The table is computed once at boot; animation is fixed point.
#define TILT_TABLE_SIZE 32
#define MAX_TILT_RADIANS 0.52f

static fix15 tilt_cos[TILT_TABLE_SIZE];
static fix15 tilt_sin[TILT_TABLE_SIZE];

// One generator per core, for the same reason as the bin counters: the
// three shifts and three xors are a read-modify-write on a shared word,
// and two cores interleaving them would corrupt the state. Separate
// seeds also keep the two streams from running in lockstep.
static uint32_t random_state[2] = {0x6d2b79f5u, 0x9e3779b9u};

static uint32_t nextRandom(void) {
  // get_core_num reads one SIO register, a single cycle.
  uint32_t *s = &random_state[get_core_num()];
  *s ^= *s << 13;
  *s ^= *s >> 17;
  *s ^= *s << 5;
  return *s;
}

static void initRoughnessTable(void) {
  for (int i = 0; i < TILT_TABLE_SIZE; i++) {
    float fraction = (float)i / (float)(TILT_TABLE_SIZE - 1);
    float angle = MAX_TILT_RADIANS * (2.0f * fraction - 1.0f);
    tilt_cos[i] = float2fix15(cosf(angle));
    tilt_sin[i] = float2fix15(sinf(angle));
  }
}

// ================================================================
// === Audio: a finite DMA-triggered thunk
// ================================================================

#define DAC_COMMAND_A 0b0011000000000000
#define AUDIO_SAMPLE_RATE 44100
#define AUDIO_DMA_TIMER 0
#define THUNK_SAMPLES 512

static unsigned short thunk_samples[THUNK_SAMPLES];
static int audio_dma_channel;

static void buildThunkSamples(void) {
  for (int i = 0; i < THUNK_SAMPLES; i++) {
    float time = (float)i / (float)AUDIO_SAMPLE_RATE;
    float envelope = expf(-110.0f * time);
    float tone = sinf(2.0f * 3.14159265f * 900.0f * time) +
                 0.35f * sinf(2.0f * 3.14159265f * 1470.0f * time);
    int sample = 2048 + (int)(1500.0f * envelope * tone);
    if (sample < 0) sample = 0;
    if (sample > 4095) sample = 4095;
    thunk_samples[i] = DAC_COMMAND_A | (sample & 0x0fff);
  }
}

static void initAudio(void) {
  spi_init(DAC_SPI_PORT, 20000000);
  spi_set_format(DAC_SPI_PORT, 16, 0, 0, 0);
  gpio_set_function(DAC_CS_PIN, GPIO_FUNC_SPI);
  gpio_set_function(DAC_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(DAC_MOSI_PIN, GPIO_FUNC_SPI);

  buildThunkSamples();
  audio_dma_channel = dma_claim_unused_channel(true);

  dma_channel_config config =
      dma_channel_get_default_config(audio_dma_channel);
  channel_config_set_transfer_data_size(&config, DMA_SIZE_16);
  channel_config_set_read_increment(&config, true);
  channel_config_set_write_increment(&config, false);
  channel_config_set_chain_to(&config, audio_dma_channel);

  uint16_t timer_numerator = (uint16_t)(
      ((uint64_t)AUDIO_SAMPLE_RATE << 16) / clock_get_hz(clk_sys));
  dma_timer_set_fraction(AUDIO_DMA_TIMER, timer_numerator, 0xffff);
  channel_config_set_dreq(&config, dma_get_timer_dreq(AUDIO_DMA_TIMER));

  dma_channel_configure(audio_dma_channel, &config,
                        &spi_get_hw(DAC_SPI_PORT)->dr, thunk_samples,
                        THUNK_SAMPLES, false);
}

static void playThunk(void) {
  // Core 0 only. The busy check and the two register writes are not
  // atomic as a group, so two cores arriving together could both see an
  // idle channel and both retrigger it, restarting the waveform part way
  // through and producing a click. Most collisions are silent at these
  // ball counts anyway, so losing core 1's share costs nothing audible.
  if (get_core_num() != 0) return;

  if (!dma_channel_is_busy(audio_dma_channel)) {
    dma_channel_set_trans_count(audio_dma_channel, THUNK_SAMPLES, false);
    dma_channel_set_read_addr(audio_dma_channel, thunk_samples, true);
  }
}

// ================================================================
// === Rotary encoder and button
// ================================================================

typedef enum {
  ADJUST_BALL_COUNT,
  ADJUST_BOUNCINESS,
  ADJUST_GRAVITY,
  ADJUST_COUNT
} AdjustMode;

static volatile int32_t encoder_steps;
static AdjustMode adjust_mode = ADJUST_BALL_COUNT;

static void encoderInterrupt(uint gpio, uint32_t events) {
  (void)gpio;
  (void)events;
  encoder_steps += gpio_get(ENCODER_B_PIN) ? -1 : 1;
}

static void initEncoder(void) {
  gpio_init(ENCODER_A_PIN);
  gpio_init(ENCODER_B_PIN);
  gpio_init(ENCODER_BUTTON_PIN);
  gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
  gpio_set_dir(ENCODER_B_PIN, GPIO_IN);
  gpio_set_dir(ENCODER_BUTTON_PIN, GPIO_IN);
  gpio_pull_up(ENCODER_A_PIN);
  gpio_pull_up(ENCODER_B_PIN);
  gpio_pull_up(ENCODER_BUTTON_PIN);
  gpio_set_irq_enabled_with_callback(ENCODER_A_PIN, GPIO_IRQ_EDGE_FALL,
                                     true, &encoderInterrupt);
}

static void initDeadlineLed(void) {
  gpio_init(DEADLINE_LED_PIN);
  gpio_set_dir(DEADLINE_LED_PIN, GPIO_OUT);
  gpio_put(DEADLINE_LED_PIN, 0);
}

// ================================================================
// === Board creation and ball spawning
// ================================================================

static void initPegs(void) {
  int peg_index = 0;
  for (int row = 0; row < NUM_ROWS; row++) {
    int left = BOARD_CENTER_X - (row * PEG_SPACING) / 2;
    row_start[row] = peg_index;
    for (int column = 0; column <= row; column++) {
      Peg *peg = &pegs[peg_index++];
      peg->px = left + column * PEG_SPACING;
      peg->py = PEG_TOP_Y + row * ROW_SPACING;
      peg->x = int2fix15(peg->px);
      peg->y = int2fix15(peg->py);
    }
  }
}

static fix15 randomHorizontalVelocity(void) {
  // A uniformly selected value in approximately [-0.5, +0.5] pixels/frame.
  fix15 velocity = ((fix15)(nextRandom() & 0xffff) - int2fix15(1)) >> 1;
  if (absfix15(velocity) < float2fix15(0.03f)) {
    velocity = velocity < 0 ? float2fix15(-0.10f) : float2fix15(0.10f);
  }
  return velocity;
}

static void spawnBall(BallWork *ball) {
  int horizontal_offset = (int)(nextRandom() % 7) - 3;
  ball->x = int2fix15(SPAWN_X + horizontal_offset);
  ball->y = int2fix15(SPAWN_Y);
  ball->vx = randomHorizontalVelocity();
  ball->vy = 0;
  ball->last_peg = -1;
}

static void spawnAllBalls(void) {
  BallWork w;
  for (int i = 0; i < MAX_BALLS; i++) {
    spawnBall(&w);
    storeBall(&w, &balls[i]);
  }
}

// ================================================================
// === Physics
// ================================================================

static int clampInt(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static fix15 clampFix(fix15 value, fix15 low, fix15 high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

// Check only the nearby 3-by-3 peg neighbourhood, not all 136 pegs.
static int findCollidingPeg(const BallWork *ball) {
  int ball_x = fix2int15(ball->x);
  int ball_y = fix2int15(ball->y);
  int center_row = (ball_y - PEG_TOP_Y) / ROW_SPACING;
  int first_row = clampInt(center_row - 1, 0, NUM_ROWS - 1);
  int last_row = clampInt(center_row + 1, 0, NUM_ROWS - 1);

  for (int row = first_row; row <= last_row; row++) {
    int left = BOARD_CENTER_X - (row * PEG_SPACING) / 2;
    int center_column = (ball_x - left) / PEG_SPACING;
    int first_column = clampInt(center_column - 1, 0, row);
    int last_column = clampInt(center_column + 1, 0, row);

    for (int column = first_column; column <= last_column; column++) {
      int index = row_start[row] + column;
      fix15 dx = ball->x - pegs[index].x;
      fix15 dy = ball->y - pegs[index].y;
      fix15 contact = int2fix15(CONTACT_RADIUS);

      if (absfix15(dx) >= contact || absfix15(dy) >= contact) continue;
      fix15 distance_squared = multfix15(dx, dx) + multfix15(dy, dy);
      if (distance_squared < int2fix15(CONTACT_RADIUS * CONTACT_RADIUS)) {
        return index;
      }
    }
  }
  return -1;
}

static void reflectFromPeg(BallWork *ball, int peg_index) {
  Peg *peg = &pegs[peg_index];
  fix15 dx = ball->x - peg->x;
  fix15 dy = ball->y - peg->y;
  fix15 distance = float2fix15(sqrtf(fix2float15(
      multfix15(dx, dx) + multfix15(dy, dy))));

  if (distance == 0) {
    distance = float2fix15(0.001f);
    dx = distance;
  }

  fix15 normal_x = divfix(dx, distance);
  fix15 normal_y = divfix(dy, distance);
  fix15 separation = int2fix15(CONTACT_RADIUS + 1);
  ball->x = peg->x + multfix15(normal_x, separation);
  ball->y = peg->y + multfix15(normal_y, separation);

  int tilt_index = nextRandom() & (TILT_TABLE_SIZE - 1);
  fix15 tilted_x = multfix15(normal_x, tilt_cos[tilt_index]) -
                    multfix15(normal_y, tilt_sin[tilt_index]);
  fix15 tilted_y = multfix15(normal_x, tilt_sin[tilt_index]) +
                    multfix15(normal_y, tilt_cos[tilt_index]);
  fix15 normal_velocity =
      multfix15(tilted_x, ball->vx) + multfix15(tilted_y, ball->vy);

  if (normal_velocity < 0) {
    fix15 impulse = -multfix15(int2fix15(2), normal_velocity);
    ball->vx += multfix15(tilted_x, impulse);
    ball->vy += multfix15(tilted_y, impulse);
    ball->vx = multfix15(ball->vx, bounciness);
    ball->vy = multfix15(ball->vy, bounciness);
  }

  if (ball->last_peg != peg_index) {
    playThunk();
    ball->last_peg = peg_index;
  }
}

static void bounceFromWalls(BallWork *ball) {
  if (ball->y < int2fix15(ARENA_TOP)) {
    ball->y = int2fix15(ARENA_TOP);
    ball->vy = -ball->vy;
  }
  if (ball->x < int2fix15(ARENA_LEFT)) {
    ball->x = int2fix15(ARENA_LEFT);
    ball->vx = -ball->vx;
  }
  if (ball->x > int2fix15(ARENA_RIGHT)) {
    ball->x = int2fix15(ARENA_RIGHT);
    ball->vx = -ball->vx;
  }
  ball->vx = clampFix(ball->vx, -MAX_HORIZONTAL_SPEED, MAX_HORIZONTAL_SPEED);
}

static void recordFallenBall(BallWork *ball) {
  int x = fix2int15(ball->x);
  int bin = clampInt((x - HIST_LEFT) / PEG_SPACING, 0, NUM_BINS - 1);

  int core = get_core_num();
  bin_count[core][bin]++;
  total_fallen[core]++;

  spawnBall(ball);
}

static void updateBall(BallWork *ball) {
  ball->x += ball->vx;
  ball->y += ball->vy;

  int peg_index = findCollidingPeg(ball);
  if (peg_index >= 0) {
    reflectFromPeg(ball, peg_index);
  } else {
    ball->last_peg = -1;
  }

  bounceFromWalls(ball);
  if (ball->y > int2fix15(COUNT_LINE_Y)) {
    recordFallenBall(ball);
    return;
  }

  ball->vy += gravity;
}

// ================================================================
// === Dual-core physics
// ================================================================

// Balls never interact -- updateBall reads only that ball, the peg
// lattice, and the two parameters -- so the loop splits cleanly down the
// middle. Core 0 takes the first half, core 1 the second. Nothing needs
// locking because the only writes are to each core's own ball range and
// to its own counters.
//
// Where core 1 starts, written by core 0 before each frame and read by
// core 1 after the handshake, so it is never read while it is changing.
static volatile int core1_first_ball;
static volatile int core1_last_ball;

static void updateBallRange(int first, int last) {
  BallWork w;

  for (int i = first; i < last; i++) {
    loadBall(&balls[i], &w);
    updateBall(&w);
    storeBall(&w, &balls[i]);
  }
}

// Core 1 spends its whole life here: blocked on the FIFO, awake only
// while there is a half-frame of physics to run.
static void core1_entry(void) {
  while (1) {
    // Blocking pop parks the core until core 0 sends the go signal.
    // No polling, no timers.
    multicore_fifo_pop_blocking();

    updateBallRange(core1_first_ball, core1_last_ball);

    // Tell core 0 this half is finished.
    multicore_fifo_push_blocking(1);
  }
}

static void updateSimulation(void) {
  int split = ball_count / 2;

  core1_first_ball = split;
  core1_last_ball = ball_count;

  // Release core 1, then do our own half while it works.
  multicore_fifo_push_blocking(1);

  updateBallRange(0, split);

  // Both halves must be complete before anything draws, or the frame
  // would show some balls a step behind the others.
  multicore_fifo_pop_blocking();
}

// ================================================================
// === Parameter control
// ================================================================

static void resetStatistics(void) {
  for (int i = 0; i < NUM_BINS; i++) {
    bin_count[0][i] = 0;
    bin_count[1][i] = 0;
  }
  total_fallen[0] = 0;
  total_fallen[1] = 0;
}

static void applyEncoderSteps(int32_t steps) {
  if (steps == 0) return;

  switch (adjust_mode) {
    case ADJUST_BALL_COUNT: {
      // Three sizes rather than two, because the range now runs to 1810
      // and ten at a time would still need 180 detents to cross it.
      int step_size = 1;
      if (ball_count >= 500) {
        step_size = 50;
      } else if (ball_count >= 100) {
        step_size = 10;
      }

      int new_count = clampInt(ball_count + (int)steps * step_size, 1,
                               MAX_BALLS);
      if (new_count != ball_count) {
        if (new_count > ball_count) {
          BallWork w;
          for (int i = ball_count; i < new_count; i++) {
            spawnBall(&w);
            storeBall(&w, &balls[i]);
          }
        }
        ball_count = new_count;
        resetStatistics();
      }
      break;
    }

    case ADJUST_BOUNCINESS: {
      fix15 new_value = clampFix(bounciness + steps * BOUNCINESS_STEP,
                                 MIN_BOUNCINESS, MAX_BOUNCINESS);
      if (new_value != bounciness) {
        bounciness = new_value;
        resetStatistics();
      }
      break;
    }

    case ADJUST_GRAVITY: {
      fix15 new_value = clampFix(gravity + steps * GRAVITY_STEP,
                                 MIN_GRAVITY, MAX_GRAVITY);
      if (new_value != gravity) {
        gravity = new_value;
        resetStatistics();
      }
      break;
    }

    case ADJUST_COUNT:
      break;
  }
}

static void pollUserControls(void) {
  static bool button_was_down;
  bool button_down = !gpio_get(ENCODER_BUTTON_PIN);
  if (button_down && !button_was_down) {
    adjust_mode = (AdjustMode)((adjust_mode + 1) % ADJUST_COUNT);
  }
  button_was_down = button_down;

  int32_t steps = encoder_steps;
  encoder_steps -= steps;
  applyEncoderSteps(steps);
}

// ================================================================
// === Drawing
// ================================================================

// The frame buffer the driver is currently letting us write to. It swaps
// every frame; the driver exports it, so nothing in the driver changes.
extern char *current_draw_buffer;

// Write one pixel, with no range check. Everything that calls this knows
// its coordinates are on screen.
static inline void putPixel(char *row_base, int x, char color) {
  char *b = row_base + (x >> 1);
  if (x & 1) {
    *b = (char)((*b & 0x0f) | (color << 4));
  } else {
    *b = (char)((*b & 0xf0) | color);
  }
}

// The outline of a radius-6 peg: what a filled circle of radius 6 covers
// minus what one of radius 5 covers, which is a ring one or two pixels
// thick. Thirty-eight pixels against the hundred and twenty-four a solid
// peg needs, so a hollow peg is the cheaper one to draw as well as the
// one that lets the balls behind it show through.
static const signed char peg_ring[38][2] = {
    {-2, -6}, {-1, -6}, {0, -6}, {1, -6}, {-4, -5}, {-3, -5}, {2, -5},
    {3, -5},  {-5, -4}, {-4, -4}, {3, -4}, {4, -4}, {-5, -3}, {4, -3},
    {-6, -2}, {5, -2},  {-6, -1}, {5, -1}, {-6, 0}, {5, 0},   {-6, 1},
    {5, 1},   {-6, 2},  {5, 2},   {-5, 3}, {4, 3},  {-5, 4},  {-4, 4},
    {3, 4},   {4, 4},   {-4, 5},  {-3, 5}, {2, 5},  {3, 5},   {-2, 6},
    {-1, 6},  {0, 6},   {1, 6}};

static inline void drawPeg(int cx, int cy, char color) {
  for (int k = 0; k < 38; k++) {
    char *row = current_draw_buffer + 320 * (cy + peg_ring[k][1]);
    putPixel(row, cx + peg_ring[k][0], color);
  }
}

static void drawPegs(void) {
  for (int i = 0; i < NUM_PEGS; i++) {
    drawPeg(pegs[i].px, pegs[i].py, DISPLAY_COLOUR);
  }
}

// Paint one ball.
//
// fillCircle is a general routine: any radius, anywhere, so every call
// works out spans, square-roots them, range-checks, and aligns odd and
// even pixels. At radius 2 that machinery costs far more than the
// drawing -- measured at 5.28 us a ball, 57% of the whole frame.
//
// A radius-2 circle is always the same sixteen pixels, so they can just
// be written:
//
//     .##.     the driver's fillCircle produces exactly this,
//     ####     so the ball looks identical on screen
//     ####
//     ####
//     .##.
//
// Two pixels share a byte in this 4 bpp buffer. Snapping the centre to
// an even column makes each 4-pixel row two whole bytes, which can be
// stored outright; only the 2-pixel top and bottom rows straddle a byte
// pair and need read-modify-write. The snap moves a ball by at most one
// pixel, which is invisible at this size.
static inline void drawBall(int cx, int cy, char color) {
  // Nothing here range-checks per pixel, so reject anything near an edge
  // before writing. Balls live in the middle of the screen anyway.
  if (cx < 2 || cx > 636 || cy < 2 || cy > 477) return;

  cx &= ~1;

  char both = (char)(color | (color << 4));
  char *p = current_draw_buffer + 320 * cy + (cx >> 1);

  // The three 4-pixel rows: two whole bytes each
  *(p - 321) = both;
  *(p - 320) = both;
  *(p - 1) = both;
  *(p) = both;
  *(p + 319) = both;
  *(p + 320) = both;

  // Top row: pixel cx-1 is the high nibble of the byte before, pixel cx
  // is the low nibble of this one.
  char *top = p - 640;
  *(top - 1) = (char)((*(top - 1) & 0x0f) | (color << 4));
  *(top) = (char)((*(top) & 0xf0) | color);

  char *bottom = p + 640;
  *(bottom - 1) = (char)((*(bottom - 1) & 0x0f) | (color << 4));
  *(bottom) = (char)((*(bottom) & 0xf0) | color);
}

static void drawBalls(void) {
  for (int i = 0; i < ball_count; i++) {
    // Stored coordinates are 11.5, so the pixel position is a shift of
    // five rather than fifteen.
    drawBall(balls[i].x >> 5, balls[i].y >> 5, DISPLAY_COLOUR);
  }
}

static void drawHistogram(void) {
  // Summed once into a local array rather than twice through binTotal,
  // since both the scaling pass and the drawing pass need the values.
  uint32_t totals[NUM_BINS];

  uint32_t largest_bin = 1;
  for (int i = 0; i < NUM_BINS; i++) {
    totals[i] = binTotal(i);
    if (totals[i] > largest_bin) largest_bin = totals[i];
  }

  for (int i = 0; i < NUM_BINS; i++) {
    int height = (int)((totals[i] * HIST_HEIGHT) / largest_bin);
    if (height == 0 && totals[i] > 0) height = 1;
    int x = HIST_LEFT + i * PEG_SPACING + 1;

    // Outlined rather than solid, to match the hollow pegs. A bar one or
    // two pixels tall has no interior, so draw those as a plain line
    // instead -- drawRect would put its top and bottom edges on the same
    // row and leave a gap at the sides.
    if (height <= 2) {
      drawHLine(x, HIST_BASE_Y - height, PEG_SPACING - 2, DISPLAY_COLOUR);
    } else {
      drawRect(x, HIST_BASE_Y - height, PEG_SPACING - 2, height,
               DISPLAY_COLOUR);
    }
  }
  drawHLine(HIST_LEFT, HIST_BASE_Y, NUM_BINS * PEG_SPACING, DISPLAY_COLOUR);
}

static const char *activeMarker(AdjustMode mode) {
  return adjust_mode == mode ? ">" : " ";
}

static void drawReadout(uint32_t frame_us, uint32_t missed_frames) {
  char line[80];
  uint32_t seconds = to_ms_since_boot(get_absolute_time()) / 1000;

  sprintf(line, "Dropped: %lu   Time: %02lu:%02lu:%02lu",
          (unsigned long)droppedTotal(), (unsigned long)(seconds / 3600),
          (unsigned long)((seconds / 60) % 60), (unsigned long)(seconds % 60));
  drawTextTiny8(8, 8, line, DISPLAY_COLOUR, BLACK);

  sprintf(line, "%s Balls: %d", activeMarker(ADJUST_BALL_COUNT), ball_count);
  drawTextTiny8(8, 20, line, DISPLAY_COLOUR, BLACK);
  sprintf(line, "%s Bounce: %.2f", activeMarker(ADJUST_BOUNCINESS),
          fix2float15(bounciness));
  drawTextTiny8(8, 32, line, DISPLAY_COLOUR, BLACK);
  sprintf(line, "%s Gravity: %.2f", activeMarker(ADJUST_GRAVITY),
          fix2float15(gravity));
  drawTextTiny8(8, 44, line, DISPLAY_COLOUR, BLACK);

  sprintf(line, "Frame: %lu us  Misses: %lu", (unsigned long)frame_us,
          (unsigned long)missed_frames);
  drawTextTiny8(8, 56, line, DISPLAY_COLOUR, BLACK);

  // The breakdown. These lag by one frame, because t_text cannot be known
  // until this function has finished and the other five were measured
  // before it started. Nothing here changes fast enough for that to
  // matter.
  //
  // Reading them: Phys and Ball grow with the ball count, the other four
  // do not. Note the figures at two very different counts and the fixed
  // cost falls straight out of the difference.
  sprintf(line, "Phys %lu  Clr %lu  Peg %lu", (unsigned long)t_physics,
          (unsigned long)t_clear, (unsigned long)t_pegs);
  drawTextTiny8(8, 68, line, DISPLAY_COLOUR, BLACK);

  sprintf(line, "Ball %lu  Hist %lu  Txt %lu", (unsigned long)t_balls,
          (unsigned long)t_hist, (unsigned long)t_text);
  drawTextTiny8(8, 80, line, DISPLAY_COLOUR, BLACK);
}

// Draw the frame, timing each phase.
//
// The phases are deliberately left as separate calls rather than merged:
// the point of this version is to find out where the time goes, and a
// merged loop would hide it.
static void drawFrame(uint32_t previous_frame_us, uint32_t missed_frames) {
  uint32_t mark = time_us_32();

  clearLowFrame(0, BLACK);
  uint32_t after_clear = time_us_32();
  t_clear = after_clear - mark;

  drawPegs();
  uint32_t after_pegs = time_us_32();
  t_pegs = after_pegs - after_clear;

  drawBalls();
  uint32_t after_balls = time_us_32();
  t_balls = after_balls - after_pegs;

  drawHistogram();
  uint32_t after_hist = time_us_32();
  t_hist = after_hist - after_balls;

  drawReadout(previous_frame_us, missed_frames);
  t_text = time_us_32() - after_hist;
}

// ================================================================
// === Animation protothread (teacher animation-demo structure)
// ================================================================

static PT_THREAD(protothread_anim(struct pt *pt)) {
  PT_BEGIN(pt);

  spawnAllBalls();

  static uint32_t previous_frame_us = 0;
  static uint32_t missed_frames = 0;

  while (1) {
    PT_YIELD_UNTIL(pt, draw_start_signal());

    uint32_t frame_start = time_us_32();

    pollUserControls();
    updateSimulation();

    uint32_t after_physics = time_us_32();
    t_physics = after_physics - frame_start;

    drawFrame(previous_frame_us, missed_frames);

    previous_frame_us = time_us_32() - frame_start;
    if (previous_frame_us > FRAME_BUDGET_US) {
      missed_frames++;
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

int main(void) {
  // Above the default 150 MHz the core needs more headroom on its supply.
  // The voltage has to settle before the clock is raised, hence the
  // pause -- changing both at once is how an overclock fails to boot.
  if (SYS_CLOCK_KHZ > 150000) {
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(10);
  }

  // The false means "proceed even if the exact frequency is not
  // achievable", so a bad value degrades rather than hanging at boot.
  set_sys_clock_khz(SYS_CLOCK_KHZ, false);

  stdio_init_all();
  initVGA();

  random_state[0] = time_us_32() | 1u;
  random_state[1] = (time_us_32() * 2654435761u) | 1u;

  initRoughnessTable();
  initPegs();
  initAudio();
  initEncoder();
  initDeadlineLed();

  // Core 1 reads the peg lattice and the tilt table, so it must not
  // start until both exist. It blocks on the FIFO immediately and does
  // nothing until the first frame hands it work.
  multicore_launch_core1(core1_entry);

  pt_add_thread(protothread_anim);
  pt_schedule_start;
}