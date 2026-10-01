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
#include "pico/stdlib.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

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

// This is a conservative 150 MHz starting count. The frame-time display
// and LED identify a board that needs a lower maximum after hardware test.
#define MAX_BALLS 300
#define INITIAL_BALLS MAX_BALLS

#define MIN_BOUNCINESS float2fix15(0.05f)
#define MAX_BOUNCINESS float2fix15(0.95f)
#define MIN_GRAVITY float2fix15(0.10f)
#define MAX_GRAVITY float2fix15(1.50f)
#define BOUNCINESS_STEP float2fix15(0.05f)
#define GRAVITY_STEP float2fix15(0.05f)
#define MAX_HORIZONTAL_SPEED float2fix15(3.5f)

#define FRAME_BUDGET_US 16667
#define DISPLAY_COLOUR WHITE

typedef struct {
  fix15 x;
  fix15 y;
  fix15 vx;
  fix15 vy;
  int last_peg;
} Ball;

typedef struct {
  fix15 x;
  fix15 y;
  int px;
  int py;
} Peg;

static Ball balls[MAX_BALLS];
static Peg pegs[NUM_PEGS];
static int row_start[NUM_ROWS];
static uint32_t bin_count[NUM_BINS];
static uint32_t total_fallen;

static int ball_count = INITIAL_BALLS;
static fix15 bounciness = float2fix15(0.50f);
static fix15 gravity = float2fix15(0.75f);

// ================================================================
// === Random surface roughness
// ================================================================

// Tilting the reflection normal breaks the otherwise perfectly symmetric
// peg collision. The table is computed once at boot; animation is fixed point.
#define TILT_TABLE_SIZE 32
#define MAX_TILT_RADIANS 0.52f

static fix15 tilt_cos[TILT_TABLE_SIZE];
static fix15 tilt_sin[TILT_TABLE_SIZE];
static uint32_t random_state = 0x6d2b79f5u;

static uint32_t nextRandom(void) {
  random_state ^= random_state << 13;
  random_state ^= random_state >> 17;
  random_state ^= random_state << 5;
  return random_state;
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

static void spawnBall(Ball *ball) {
  int horizontal_offset = (int)(nextRandom() % 7) - 3;
  ball->x = int2fix15(SPAWN_X + horizontal_offset);
  ball->y = int2fix15(SPAWN_Y);
  ball->vx = randomHorizontalVelocity();
  ball->vy = 0;
  ball->last_peg = -1;
}

static void spawnAllBalls(void) {
  for (int i = 0; i < MAX_BALLS; i++) {
    spawnBall(&balls[i]);
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
static int findCollidingPeg(const Ball *ball) {
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

static void reflectFromPeg(Ball *ball, int peg_index) {
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

static void bounceFromWalls(Ball *ball) {
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

static void recordFallenBall(Ball *ball) {
  int x = fix2int15(ball->x);
  int bin = clampInt((x - HIST_LEFT) / PEG_SPACING, 0, NUM_BINS - 1);
  bin_count[bin]++;
  total_fallen++;
  spawnBall(ball);
}

static void updateBall(Ball *ball) {
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

static void updateSimulation(void) {
  for (int i = 0; i < ball_count; i++) {
    updateBall(&balls[i]);
  }
}

// ================================================================
// === Parameter control
// ================================================================

static void resetStatistics(void) {
  for (int i = 0; i < NUM_BINS; i++) {
    bin_count[i] = 0;
  }
  total_fallen = 0;
}

static void applyEncoderSteps(int32_t steps) {
  if (steps == 0) return;

  switch (adjust_mode) {
    case ADJUST_BALL_COUNT: {
      int step_size = ball_count >= 100 ? 10 : 1;
      int new_count = clampInt(ball_count + (int)steps * step_size, 1,
                               MAX_BALLS);
      if (new_count != ball_count) {
        if (new_count > ball_count) {
          for (int i = ball_count; i < new_count; i++) spawnBall(&balls[i]);
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

static void drawPegs(void) {
  for (int i = 0; i < NUM_PEGS; i++) {
    fillCircle(pegs[i].px, pegs[i].py, PEG_RADIUS, DISPLAY_COLOUR);
  }
}

static void drawBalls(void) {
  for (int i = 0; i < ball_count; i++) {
    fillCircle(fix2int15(balls[i].x), fix2int15(balls[i].y), BALL_RADIUS,
               DISPLAY_COLOUR);
  }
}

static void drawHistogram(void) {
  uint32_t largest_bin = 1;
  for (int i = 0; i < NUM_BINS; i++) {
    if (bin_count[i] > largest_bin) largest_bin = bin_count[i];
  }

  for (int i = 0; i < NUM_BINS; i++) {
    int height = (int)((bin_count[i] * HIST_HEIGHT) / largest_bin);
    if (height == 0 && bin_count[i] > 0) height = 1;
    int x = HIST_LEFT + i * PEG_SPACING + 1;
    fillRect(x, HIST_BASE_Y - height, PEG_SPACING - 2, height,
             DISPLAY_COLOUR);
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
          (unsigned long)total_fallen, (unsigned long)(seconds / 3600),
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
}

static void drawFrame(uint32_t previous_frame_us, uint32_t missed_frames) {
  clearLowFrame(0, BLACK);
  drawPegs();
  drawBalls();
  drawHistogram();
  drawReadout(previous_frame_us, missed_frames);
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
  // Keep the 150 MHz clock used by the supplied VGA PIO configuration.
  set_sys_clock_khz(150000, true);
  stdio_init_all();
  initVGA();

  random_state = time_us_32() | 1u;
  initRoughnessTable();
  initPegs();
  initAudio();
  initEncoder();
  initDeadlineLed();

  pt_add_thread(protothread_anim);
  pt_schedule_start;
}
