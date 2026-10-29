/**
 * Lab 3 (1D helicopter) — week 2 version
 * Based on V. Hunter Adams' MPU6050 IMU demo (vha3@cornell.edu)
 *
 * What this adds on top of the demo:
 *   1. Arm angle from a complementary filter of accel + gyro (degrees,
 *      -90 = hanging straight down, 0 = horizontal, +90 = straight up)
 *   2. Gyro bias calibration at boot (keep the arm still while it runs)
 *   3. PID control: control = base + Kp*error + I - Kd*rate,  I += Ki*error*dt
 *      [Week 3 stage 1: D term.  Stage 2: I term with anti-windup.
 *       Ki = Kd = 0 behaves exactly like week 2]
 *   4. VGA: top plot = angle (white) + target (yellow),
 *           bottom plot = low-passed motor command (green)
 *   5. Serial commands to change parameters while running
 *
 * The motor starts DISABLED. Type "m 1" in the serial terminal to enable.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2350 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2350 GND ---> MPU6050 GND
 *  - GPIO 6 (pin 9) ---> motor board PWM terminal
 *  - RP2350 GND ---> motor board PWM GND terminal
 *  - GPIO 0 / GPIO 1 ---> USB-serial RX / TX (UART0, 115200 baud)
 *  - GPIO 2 (pin 4) ---> push button ---> GND (pin 3)   [Week 3 demo button]
 *
 * DEMO BUTTON (Week 3, stage 3)
 *  - Hold the button:   motor OFF, arm falls to its rest position
 *  - Release (t = 0):   motor ON, target   0 deg
 *  - t = 5 s:           target +30 deg
 *  - t = 10 s:          target -30 deg
 *  - t = 15 s:          target   0 deg  (keeps hovering afterwards)
 *  - Typing "a" or "m" on the serial port cancels the sequence.
 */

// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
// Include custom libraries
#include "VGA/vga16_graphics_v3.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"

////////////////////////////////////////////////////////////////////////////
////////////////////// IMU ORIENTATION — CHECK THESE ////////////////////////
////////////////////////////////////////////////////////////////////////////
// These depend on how the IMU is glued to your arm. Find them with the
// original demo (or with "r" below, which prints raw readings):
//
//  ACCEL_ALONG_AXIS: the accel axis that points ALONG the arm, toward the
//      motor. With the arm horizontal it reads ~0 g; hanging down it reads
//      about +/-1 g.
//  ACCEL_UP_AXIS: the accel axis that reads about +/-1 g when the arm is
//      horizontal (perpendicular to the arm, in the plane it swings in).
//  The two *_SIGN values make: horizontal -> along = 0,  up = +1 g
//                              hanging   -> along = -1 g, up = 0
//
//  GYRO_AXIS: the gyro axis parallel to the hinge. It is the one that
//      changes when you swing the arm; the other two stay near 0.
//  GYRO_SIGN: make it +1 or -1 so that swinging the arm UP gives a
//      POSITIVE rate. If this is wrong the filtered angle lags badly or
//      drifts the opposite way from the accel angle.
//
// Axis index: 0 = x, 1 = y, 2 = z
// Measured on our arm: horizontal -> z = +0.89 g, arm tilted down -> x positive
#define ACCEL_ALONG_AXIS 0
#define ACCEL_ALONG_SIGN -1.0f
#define ACCEL_UP_AXIS    2
#define ACCEL_UP_SIGN    1.0f
#define GYRO_AXIS        1
#define GYRO_SIGN        1.0f

////////////////////////////////////////////////////////////////////////////
/////////////////////////// CONTROL PARAMETERS /////////////////////////////
////////////////////////////////////////////////////////////////////////////
#define DT        0.001f      // ISR period: 1 kHz PWM wrap
#define RAD2DEG   57.29578f
#define WRAPVAL   6000        // PWM counts per period
#define CLKDIV    25.0f       // 150 MHz / (25 * 6001) = ~1 kHz
#define PWM_PIN   6           // motor board PWM terminal (physical pin 9)
#define BUTTON_PIN 2          // [Week 3] demo button (physical pin 4) to GND

// Complementary filter weight on the gyro path (0..1). Closer to 1 trusts
// the gyro more: smoother, but slower to correct drift.
volatile float alpha = 0.995f ;

// Controller state, shared between the ISR and the threads.
// RP2350 has a hardware FPU, so float math in the ISR is fast.
volatile float angle_deg   = -90.0f ;  // filtered arm angle
volatile float accel_deg   = -90.0f ;  // accel-only angle (for debugging)
volatile float gyro_deg    = -90.0f ;  // gyro-only angle: pure integration, drifts
volatile float gyro_part   = 0.0f ;    // alpha * (angle + rate*dt)   (filter term 1)
volatile float accel_part  = 0.0f ;    // (1 - alpha) * accel angle   (filter term 2)
volatile float target_deg  = 0.0f ;    // where we want the arm
volatile float Kp          = 20.0f ;   // PWM counts per degree of error
volatile float Kd          = 0.0f ;    // [Week 3] PWM counts per (deg/s) of rotation
volatile float Ki          = 0.0f ;    // [Week 3] PWM counts per (deg * s) of accumulated error
volatile float i_term      = 0.0f ;    // [Week 3] integral term (PWM counts), kept between ISRs
volatile float I_max       = 1000.0f ; // [Week 3] max |I term| in PWM counts ("l" command)
#define KI_MAX  200.0f                 // [Week 3] largest Ki the "i" command accepts
volatile float p_term      = 0.0f ;    // [Week 3] Kp * error, for display
volatile float d_term      = 0.0f ;    // [Week 3] -Kd * rate, for display
volatile float rate_disp   = 0.0f ;    // [Week 3] gyro rate (deg/s), low-passed for display
volatile float base_pwm    = 0.0f ;    // feed-forward / hover offset
volatile int   motor_on    = 0 ;       // 0 = motor forced off
volatile float control     = 0.0f ;    // command sent to PWM (0..WRAPVAL)
volatile float motor_disp  = 0.0f ;    // low-passed control, for display
float gyro_bias = 0.0f ;               // deg/s, measured at boot

// [Week 3] Demo button sequence state
#define DEMO_IDLE 0                    // not running (normal serial control)
#define DEMO_HOLD 1                    // button held: motor off
#define DEMO_RUN  2                    // released: stepping through targets
volatile int   demo_state = DEMO_IDLE ;
volatile float demo_time  = 0.0f ;     // seconds since the button was released

////////////////////////////////////////////////////////////////////////////
//////////////////////////////// DISPLAY ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#define LEFT_EDGE 81
#define PLOT_WIDTH 530
#define PLOT_HEIGHT 150
#define TOP_PLOT_BOTTOM 230      // angle plot spans y = 80..230
#define BOTTOM_PLOT_BOTTOM 430   // motor plot spans y = 280..430

float angle_buf[PLOT_WIDTH] ;
float motor_buf[PLOT_WIDTH] ;
float accel_buf[PLOT_WIDTH] ;    // accel-only angle history
float gyro_buf[PLOT_WIDTH] ;     // gyro-only angle history
volatile int head = 0 ;          // oldest sample in the circular buffers
volatile int threshold = 10 ;    // store one sample every N ISR calls
volatile int throttle = 0 ;
char screentext[96] ;

// Arrays in which raw measurements will be stored (15.16 fixed point)
fix15 acceleration[3], gyro[3] ;

uint slice_num ;

#define clampf(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

// I2C bus recovery. If the Pico resets in the middle of an I2C read, the
// MPU6050 (which stays powered) can be left holding SDA low, waiting for
// clocks that never come. i2c_write_blocking() then waits forever.
// Fix: drive SCL by hand (open-drain style) until the IMU lets go of SDA,
// then send a STOP condition. Standard I2C recovery (9 clock pulses).
static void i2c_line(uint pin, int high) {
    if (high) gpio_set_dir(pin, GPIO_IN) ;               // released: pull-up makes it high
    else    { gpio_put(pin, 0) ; gpio_set_dir(pin, GPIO_OUT) ; }   // driven low
    sleep_us(5) ;
}
static int i2c_bus_recover(void) {   // returns 1 if SDA was stuck low
    gpio_init(SDA_PIN) ; gpio_pull_up(SDA_PIN) ;
    gpio_init(SCL_PIN) ; gpio_pull_up(SCL_PIN) ;
    i2c_line(SDA_PIN, 1) ;
    i2c_line(SCL_PIN, 1) ;
    int stuck = !gpio_get(SDA_PIN) ;
    for (int k = 0; k < 9 && !gpio_get(SDA_PIN); k++) {
        i2c_line(SCL_PIN, 0) ;
        i2c_line(SCL_PIN, 1) ;
    }
    // STOP: SDA goes low -> high while SCL is high
    i2c_line(SCL_PIN, 0) ;
    i2c_line(SDA_PIN, 0) ;
    i2c_line(SCL_PIN, 1) ;
    i2c_line(SDA_PIN, 1) ;
    return stuck ;
}

// Angle from the accelerometer alone (degrees)
static inline float accel_angle(void) {
    float along = ACCEL_ALONG_SIGN * fix2float15(acceleration[ACCEL_ALONG_AXIS]) ;
    float up    = ACCEL_UP_SIGN    * fix2float15(acceleration[ACCEL_UP_AXIS]) ;
    return atan2f(along, up) * RAD2DEG ;
}

// Gyro rate about the hinge, bias removed (deg/s)
static inline float gyro_rate(void) {
    return GYRO_SIGN * fix2float15(gyro[GYRO_AXIS]) - gyro_bias ;
}

////////////////////////////////////////////////////////////////////////////
/////////////////////////// 1 kHz CONTROL ISR //////////////////////////////
////////////////////////////////////////////////////////////////////////////
void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(slice_num) ;
    // GPIO 14 is high while the ISR runs: put a scope on it to see how
    // much of the 1 ms budget the ISR uses
    gpio_put(14, 1) ;

    // 1. Read the IMU (accel in g, gyro in deg/s, 15.16 fixed point)
    mpu6050_read_raw(acceleration, gyro) ;

    // 2. Complementary filter
    float a_deg = accel_angle() ;
    float rate  = gyro_rate() ;
    float g_part = alpha * (angle_deg + rate * DT) ;   // gyro path
    float a_part = (1.0f - alpha) * a_deg ;            // accel path
    float new_angle = g_part + a_part ;                // = complementary output
    accel_deg  = a_deg ;
    gyro_deg  += rate * DT ;                           // gyro alone, for comparison
    gyro_part  = g_part ;
    rate_disp  = rate_disp + (rate - rate_disp) * (1.0f / 16.0f) ;
    accel_part = a_part ;
    angle_deg  = new_angle ;

    // 3. PID control (with a constant feed-forward term)   [Week 3]
    //    P pushes toward the target; D brakes any rotation;
    //    I slowly adds up any error that stays, until it is gone.
    //    D uses the gyro rate directly: rate = d(angle)/dt, so
    //    d(error)/dt = -rate when the target is constant.
    float u = 0.0f ;
    if (motor_on) {
        float error = target_deg - new_angle ;
        float p = Kp * error ;
        float d = -Kd * rate ;

        // Integral with anti-windup: if the motor is already maxed out
        // (or off) in the direction the error asks for, don't keep adding,
        // otherwise I grows huge and the arm overshoots badly later.
        float u_try = base_pwm + p + i_term + d ;
        int saturated = (u_try >= (float)WRAPVAL && error > 0.0f) ||
                        (u_try <= 0.0f && error < 0.0f) ;
        if (!saturated) i_term += Ki * error * DT ;
        i_term = clampf(i_term, -I_max, I_max) ;

        p_term = p ;
        d_term = d ;
        u = base_pwm + p + i_term + d ;
        u = clampf(u, 0.0f, (float)WRAPVAL) ;
    }
    else {
        p_term = 0.0f ;
        d_term = 0.0f ;
        i_term = 0.0f ;      // start fresh every time the motor turns on
    }
    control = u ;

    // 4. Drive the motor
    pwm_set_gpio_level(PWM_PIN, (uint16_t)u) ;

    // 5. Low-pass the command for display (time constant ~16 ms)
    motor_disp = motor_disp + (u - motor_disp) * (1.0f / 16.0f) ;

    // 6. Save samples for the VGA plots
    throttle += 1 ;
    if (throttle >= threshold) {
        throttle = 0 ;
        angle_buf[head] = new_angle ;
        accel_buf[head] = a_deg ;
        gyro_buf[head]  = gyro_deg ;
        motor_buf[head] = motor_disp ;
        head = ((head + 1) == PLOT_WIDTH) ? 0 : (head + 1) ;
    }

    gpio_put(14, 0) ;
}

////////////////////////////////////////////////////////////////////////////
//////////////////////////////// VGA ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// Map an angle (-90..+90 deg) to a screen row in the top plot
static inline int angle_to_y(float deg) {
    float y = TOP_PLOT_BOTTOM - (deg + 90.0f) / 180.0f * PLOT_HEIGHT ;
    return (int)clampf(y, TOP_PLOT_BOTTOM - PLOT_HEIGHT, TOP_PLOT_BOTTOM) ;
}

// Map a motor command (0..WRAPVAL) to a screen row in the bottom plot
static inline int motor_to_y(float m) {
    float y = BOTTOM_PLOT_BOTTOM - m / (float)WRAPVAL * PLOT_HEIGHT ;
    return (int)clampf(y, BOTTOM_PLOT_BOTTOM - PLOT_HEIGHT, BOTTOM_PLOT_BOTTOM) ;
}

void drawAxes() {
    setTextSize(1) ;
    setTextColor(WHITE) ;

    // Top plot: angle
    drawVLine(80, TOP_PLOT_BOTTOM - PLOT_HEIGHT, PLOT_HEIGHT, CYAN) ;
    drawHLine(75, angle_to_y(90),  5, CYAN) ;
    drawHLine(75, angle_to_y(0),   5, CYAN) ;
    drawHLine(75, angle_to_y(-90), 5, CYAN) ;
    setCursor(45, angle_to_y(90) - 4) ;  writeString("+90") ;
    setCursor(57, angle_to_y(0) - 4) ;   writeString("0") ;
    setCursor(45, angle_to_y(-90) - 4) ; writeString("-90") ;
    setCursor(LEFT_EDGE, 62) ;
    writeString("Angle: white=complementary  red=accel-only  magenta=gyro-only  yellow=target") ;

    // Bottom plot: motor command
    drawVLine(80, BOTTOM_PLOT_BOTTOM - PLOT_HEIGHT, PLOT_HEIGHT, CYAN) ;
    drawHLine(75, motor_to_y(WRAPVAL), 5, CYAN) ;
    drawHLine(75, motor_to_y(WRAPVAL / 2), 5, CYAN) ;
    drawHLine(75, motor_to_y(0), 5, CYAN) ;
    setCursor(40, motor_to_y(WRAPVAL) - 4) ;     writeString("6000") ;
    setCursor(40, motor_to_y(WRAPVAL / 2) - 4) ; writeString("3000") ;
    setCursor(57, motor_to_y(0) - 4) ;           writeString("0") ;
    setCursor(LEFT_EDGE, 262) ;
    writeString("Motor command (low-passed PWM counts)") ;
}

static PT_THREAD (protothread_draw(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static int start ;
    static int i ;

    drawAxes() ;
    copy_buffer_to_other() ;

    while (1) {
        // Wait for a buffer swap
        PT_YIELD_UNTIL(pt, draw_start_signal()) ;
        gpio_put(15, !gpio_get(15)) ;

        clearLowFrame(0, BLACK) ;
        drawAxes() ;

        // Status line
        setTextColor2(WHITE, BLACK) ;
        setCursor(10, 10) ;
        sprintf(screentext, "Kp=%.1f  Ki=%.1f  Kd=%.2f  base=%.0f  target=%.1f deg  motor %s",
                Kp, Ki, Kd, base_pwm, target_deg, motor_on ? "ON " : "OFF") ;
        writeString(screentext) ;
        // Demo status, top right
        setCursor(470, 24) ;
        if      (demo_state == DEMO_RUN)  sprintf(screentext, "DEMO t=%5.1f s ", demo_time) ;
        else if (demo_state == DEMO_HOLD) sprintf(screentext, "DEMO: button held") ;
        else                              sprintf(screentext, "                 ") ;
        writeString(screentext) ;
        setCursor(10, 24) ;
        sprintf(screentext, "angle=%6.1f  accel-only=%6.1f  gyro-only=%6.1f  cmd=%4.0f",
                angle_deg, accel_deg, gyro_deg, motor_disp) ;
        writeString(screentext) ;
        setCursor(10, 48) ;
        sprintf(screentext, "cmd = base %5.0f + P %6.0f + I %5.0f/%4.0f + D %6.0f (rate %6.1f)",
                base_pwm, p_term, i_term, I_max, d_term, rate_disp) ;
        writeString(screentext) ;
        setCursor(10, 38) ;
        sprintf(screentext, "filter: %.3f*(gyro path)=%7.2f + %.3f*accel=%6.2f -> %6.1f",
                alpha, gyro_part, 1.0f - alpha, accel_part, gyro_part + accel_part) ;
        writeString(screentext) ;

        // Target line
        drawHLine(LEFT_EDGE, angle_to_y(target_deg), PLOT_WIDTH, YELLOW) ;

        // Data, oldest --> newest
        start = head ;
        for (i = 0; i < PLOT_WIDTH; i++) {
            drawPixel(LEFT_EDGE + i, angle_to_y(accel_buf[start]), RED) ;
            drawPixel(LEFT_EDGE + i, angle_to_y(gyro_buf[start]), MAGENTA) ;
            drawPixel(LEFT_EDGE + i, angle_to_y(angle_buf[start]), WHITE) ;
            drawPixel(LEFT_EDGE + i, motor_to_y(motor_buf[start]), GREEN) ;
            start = ((start + 1) == PLOT_WIDTH) ? 0 : (start + 1) ;
        }
    }
    PT_END(pt) ;
}

////////////////////////////////////////////////////////////////////////////
///////////////////////////// SERIAL COMMANDS //////////////////////////////
////////////////////////////////////////////////////////////////////////////
//   p 20     set Kp
//   d 2      set Kd   [Week 3]
//   i 20     set Ki   [Week 3]  (0..200)
//   l 1000   set the max |I term| in PWM counts  [Week 3]
//   z        reboot the Pico (re-inits the IMU and recalibrates the gyro)
//   a 0      set target angle in degrees (-90 down, 0 horizontal)
//   b 2500   set base (feed-forward) PWM, 0..6000
//   f 0.995  set complementary filter alpha, 0..1
//   m 1      motor on   (m 0 = motor off)
//   t 10     plot speed: store one sample every N ms
//   r        print raw accel/gyro (for finding the axes above)
//   g        reset the gyro-only angle to the current filtered angle
//   ?        print current settings
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char cmd ;
    static float val ;
    static int n ;

    sprintf(pt_serial_out_buffer,
            "\r\nCommands: p <Kp>, i <Ki>, d <Kd>, l <Imax>, a <deg>, b <base>, f <alpha>, m <0|1>, t <ms>, r, g, z, ?\r\n") ;
    serial_write ;

    while (1) {
        sprintf(pt_serial_out_buffer, "cmd> ") ;
        serial_write ;
        serial_read ;
        n = sscanf(pt_serial_in_buffer, " %c %f", &cmd, &val) ;
        if (n < 1) continue ;

        if (cmd == 'z') {
            // Reboot the Pico (same as the reset button). The boot code then
            // recovers the I2C bus, resets the MPU6050 and recalibrates the gyro.
            // Note: the IMU itself stays powered; unplug USB for a full power cycle.
            motor_on = 0 ;
            pwm_set_gpio_level(PWM_PIN, 0) ;
            sprintf(pt_serial_out_buffer, "rebooting... keep the arm still\r\n") ;
            serial_write ;
            sleep_ms(50) ;               // let the UART finish sending
            watchdog_reboot(0, 0, 10) ;  // reboot in 10 ms
            while (1) tight_loop_contents() ;
        }
        else if (cmd == 'i' && n == 2) {
            Ki = clampf(val, 0.0f, KI_MAX) ;
        }
        else if (cmd == 'l' && n == 2) {
            I_max = clampf(val, 0.0f, (float)WRAPVAL) ;
        }
        else if (cmd == 'd' && n == 2) {
            Kd = val ;
        }
        else if (cmd == 'p' && n == 2) {
            Kp = val ;
        }
        else if (cmd == 'a' && n == 2) {
            demo_state = DEMO_IDLE ;            // a manual target cancels the demo
            target_deg = clampf(val, -90.0f, 90.0f) ;
        }
        else if (cmd == 'b' && n == 2) {
            base_pwm = clampf(val, 0.0f, (float)WRAPVAL) ;
        }
        else if (cmd == 'f' && n == 2) {
            alpha = clampf(val, 0.0f, 1.0f) ;
        }
        else if (cmd == 'm' && n == 2) {
            demo_state = DEMO_IDLE ;            // manual motor control cancels the demo
            motor_on = (val != 0.0f) ;
        }
        else if (cmd == 't' && n == 2) {
            if (val >= 1.0f) threshold = (int)val ;
        }
        else if (cmd == 'g') {
            gyro_deg = angle_deg ;   // gyro-only drifts; line it back up
        }
        else if (cmd == 'r') {
            sprintf(pt_serial_out_buffer,
                    "accel g: x=%5.2f y=%5.2f z=%5.2f | gyro dps: x=%7.1f y=%7.1f z=%7.1f\r\n",
                    fix2float15(acceleration[0]), fix2float15(acceleration[1]),
                    fix2float15(acceleration[2]), fix2float15(gyro[0]),
                    fix2float15(gyro[1]), fix2float15(gyro[2])) ;
            serial_write ;
            continue ;
        }
        else if (cmd != '?') {
            sprintf(pt_serial_out_buffer, "unknown command\r\n") ;
            serial_write ;
            continue ;
        }

        sprintf(pt_serial_out_buffer,
                "Kp=%.2f Ki=%.2f Kd=%.3f base=%.0f target=%.1f alpha=%.4f motor=%s angle=%.1f I=%.0f Imax=%.0f bias=%.2f\r\n",
                Kp, Ki, Kd, base_pwm, target_deg, alpha, motor_on ? "ON" : "OFF",
                angle_deg, i_term, I_max, gyro_bias) ;
        serial_write ;
    }
    PT_END(pt) ;
}

////////////////////////////////////////////////////////////////////////////
/////////////////////////// DEMO BUTTON THREAD /////////////////////////////
////////////////////////////////////////////////////////////////////////////
// [Week 3, stage 3] Polls the button every 10 ms. The button pulls GPIO 2
// to GND, so pressed = 0. Requires 2 readings in a row (20 ms) to count a
// press or a release, which ignores contact bounce.
static PT_THREAD (protothread_demo(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static uint64_t t_release ;

    while (1) {
        PT_YIELD_usec(10000) ;

        if (!gpio_get(BUTTON_PIN)) {
            PT_YIELD_usec(10000) ;
            if (gpio_get(BUTTON_PIN)) continue ;      // just a glitch

            // Pressed and held: motor off, arm falls to rest
            demo_state = DEMO_HOLD ;
            motor_on   = 0 ;
            target_deg = 0.0f ;
            printf("DEMO: button held, motor off\r\n") ;

            // Wait for a clean release (high on two readings in a row)
            while (1) {
                PT_YIELD_UNTIL(pt, gpio_get(BUTTON_PIN)) ;
                PT_YIELD_usec(10000) ;
                if (gpio_get(BUTTON_PIN)) break ;
            }

            // Released: this is t = 0
            t_release  = time_us_64() ;
            demo_time  = 0.0f ;
            target_deg = 0.0f ;
            motor_on   = 1 ;
            demo_state = DEMO_RUN ;
            printf("DEMO: released, t = 0, target 0\r\n") ;
        }

        if (demo_state == DEMO_RUN) {
            demo_time = (float)(time_us_64() - t_release) * 1e-6f ;
            if      (demo_time <  5.0f) target_deg =   0.0f ;
            else if (demo_time < 10.0f) target_deg =  30.0f ;
            else if (demo_time < 15.0f) target_deg = -30.0f ;
            else                        target_deg =   0.0f ;
            if (demo_time >= 20.0f) demo_state = DEMO_IDLE ;  // done, keep hovering at 0
        }
    }
    PT_END(pt) ;
}

////////////////////////////////////////////////////////////////////////////
//////////////////////////////// STARTUP ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void core1_entry() {
    gpio_init(15) ;
    gpio_set_dir(15, GPIO_OUT) ;
    gpio_put(15, 0) ;
    pt_add_thread(protothread_draw) ;
    pt_schedule_start ;
}

int main() {
    set_sys_clock_khz(150000, true) ;
    stdio_init_all() ;
    initVGA() ;

    // Debug pin: high while the ISR runs
    gpio_init(14) ;
    gpio_set_dir(14, GPIO_OUT) ;
    gpio_put(14, 0) ;

    //////////////////////////// I2C + IMU ////////////////////////////////
    int was_stuck = i2c_bus_recover() ;   // un-stick the bus if the IMU was mid-transfer
    printf("\r\nI2C bus: %s, SDA now %s\r\n",
           was_stuck ? "was STUCK (SDA low), recovered" : "OK",
           gpio_get(SDA_PIN) ? "high (good)" : "STILL LOW -- check wiring") ;
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;
    gpio_pull_up(SDA_PIN) ;
    gpio_pull_up(SCL_PIN) ;

    // Full device reset of the MPU6050 (PWR_MGMT_1 bit 7), with a timeout
    // so a missing/miswired IMU can't hang the boot
    uint8_t dev_reset[] = {0x6B, 0x80} ;
    i2c_write_timeout_us(I2C_CHAN, ADDRESS, dev_reset, 2, false, 2000) ;
    sleep_ms(100) ;

    // WHO_AM_I (register 0x75) should read back 0x68
    uint8_t reg = 0x75, who = 0 ;
    i2c_write_timeout_us(I2C_CHAN, ADDRESS, &reg, 1, true, 2000) ;
    int got = i2c_read_timeout_us(I2C_CHAN, ADDRESS, &who, 1, false, 2000) ;
    if (got == 1) printf("MPU6050 WHO_AM_I = 0x%02x (expect 0x68)\r\n", who) ;
    else          printf("MPU6050 did not answer -- check VCC, GND, SDA=GP8, SCL=GP9\r\n") ;

    mpu6050_reset() ;
    sleep_ms(100) ;   // let the sensor settle after wake-up

    // Gyro bias: average 1000 readings (~1.5 s). Keep the arm STILL.
    printf("\r\nCalibrating gyro, keep the arm still...\r\n") ;
    float sum = 0.0f ;
    for (int k = 0; k < 1000; k++) {
        mpu6050_read_raw(acceleration, gyro) ;
        sum += GYRO_SIGN * fix2float15(gyro[GYRO_AXIS]) ;
        sleep_us(1000) ;
    }
    gyro_bias = sum / 1000.0f ;
    angle_deg = accel_angle() ;   // start the filter at the accel angle
    gyro_deg  = angle_deg ;       // gyro-only starts at the same place
    accel_deg = angle_deg ;
    for (int k = 0; k < PLOT_WIDTH; k++) {
        angle_buf[k] = angle_deg ;
        accel_buf[k] = angle_deg ;
        gyro_buf[k]  = angle_deg ;
        motor_buf[k] = 0.0f ;
    }
    printf("gyro bias = %.3f deg/s, start angle = %.1f deg\r\n", gyro_bias, angle_deg) ;

    /////////////////////////////// PWM ///////////////////////////////////
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM) ;
    slice_num = pwm_gpio_to_slice_num(PWM_PIN) ;   // GPIO 6 = slice 3, channel A

    pwm_clear_irq(slice_num) ;
    pwm_set_irq_enabled(slice_num, true) ;
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap) ;
    irq_set_enabled(PWM_IRQ_WRAP, true) ;

    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;
    pwm_set_gpio_level(PWM_PIN, 0) ;   // motor off at boot
    pwm_set_mask_enabled((1u << slice_num)) ;

    /////////////////////////// DEMO BUTTON ///////////////////////////////
    gpio_init(BUTTON_PIN) ;
    gpio_set_dir(BUTTON_PIN, GPIO_IN) ;
    gpio_pull_up(BUTTON_PIN) ;          // not pressed = high

    ////////////////////////////// THREADS ////////////////////////////////
    multicore_reset_core1() ;
    multicore_launch_core1(core1_entry) ;

    pt_add_thread(protothread_demo) ;
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;
}