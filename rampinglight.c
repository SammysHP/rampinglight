#include <avr/io.h>
#include <util/delay.h>
// #include <avr/interrupt.h>
// #include <avr/eeprom.h>
// #include <avr/sleep.h>

#define PWM_PIN PB1
#define VOLTAGE_PIN PB2

#define RAMP_TIME 3
#define RAMP_SIZE sizeof(ramp_values)
#define RAMP_VALUES 5,5,5,5,5,6,6,6,6,7,7,8,8,9,10,11,12,13,14,15,17,18,20,22,23,25,28,30,32,35,38,41,44,47,51,55,59,63,67,71,76,81,86,92,97,103,109,116,122,129,136,144,151,159,167,176,185,194,203,213,223,233,244,255
#define TURBO_PWM 255

#define RAMPING_STOPPED 0
#define RAMPING_UP 1
#define RAMPING_DOWN -1

#define CBD_BYTES 3
#define CBD_PATTERN 0xAA

/*
#define ADC_CHANNEL 0x01    // MUX 01 corresponds with PB2
#define ADC_DIDR    ADC1D   // Digital input disable bit corresponding with PB2
#define ADC_PRSCL   0x06    // clk/64
*/

/**
 * States of the state machine.
 */
enum State {
  kRamping,  // Ramping up and down
  kFrozen,   // Frozen ramping level
  kTurbo,    // Full power
  kConfig,   // Config menu
};

/**
 * TODO Doc and split into options and runtime state
 */
struct Options {
  uint8_t fixed_modes : 1;
  uint8_t mode_memory : 1;
  uint8_t freeze_on_high : 1;
  uint8_t start_high : 1;
  uint8_t ramping_up : 1;
};

const uint8_t __flash ramp_values[] = { RAMP_VALUES };

uint8_t cold_boot_detect[CBD_BYTES] __attribute__((section(".noinit")));
struct Options options __attribute__((section(".noinit")));
enum State state __attribute__((section(".noinit")));
uint8_t output __attribute__((section(".noinit")));
uint8_t fast_presses __attribute__((section(".noinit")));

/**
 * Busy wait delay with ms resolution. This function allows to choose the duration during runtime.
 *
 * @param duration Wait duration in ms.
 */
void delay_ms(uint16_t duration) {
  while (duration--) {
    _delay_ms(1);
  }
}

/**
 * Set output, i.e. compare register.
 *
 * @param pwm Raw PWM level.
 */
void set_pwm(uint8_t pwm) {
  OCR0B = pwm;
}

/**
 * Set output to a level as defined in the ramp table. The table is indexed
 * starting from 1 so that level 0 can be used to disable the output.
 *
 * @param level Index in ramp_values, starting at 1.
 */
void set_level(uint8_t level) {
  if (level == 0) {
    set_pwm(0);
  } else {
    set_pwm(ramp_values[level - 1]);
  }
}

/**
 * Blink or strobe the LED on low intensity.
 *
 * @param count Number of flashes.
 * @param speed Duration of a single flash.
 */
void blink(uint8_t count, uint16_t speed) {
  while (count--) {
    set_pwm(40);
    delay_ms(speed);
    set_pwm(0);
    delay_ms(speed);
    delay_ms(speed);
  }
}

/**
 * Entry point.
 */
int main(void) {
  uint8_t coldboot = 0;

  options.fixed_modes = 0;
  options.mode_memory = 0;
  options.freeze_on_high = 0;
  options.start_high = 0;

  // Phase correct PWM, system clock without prescaler
  TCCR0A = (1 << COM0B1) | (1 << WGM00);
  TCCR0B = (1 << CS00);

  // Set PWM pin to output
  DDRB |= (1 << PWM_PIN);

  // Cold boot detection
  for (int i = CBD_BYTES - 1; i >= 0; --i) {
    if (cold_boot_detect[i] != CBD_PATTERN) {
      coldboot = 1;
    }
    cold_boot_detect[i] = CBD_PATTERN;
  }

  if (coldboot) {  // Initialize state after the flashlight was switched off for some time
    state = kRamping;
    fast_presses = 0;
    options.ramping_up = 1;
    output = 1;
  } else {  // User has tapped the power button
    ++fast_presses;

    // Input handling
    switch (state) {
      case kRamping:
        state = kFrozen;
        break;

      case kFrozen:
        state = kRamping;
        break;

      case kTurbo:
        state = kRamping;
        break;
    }

    if (fast_presses == 2) {
      state = kTurbo;
    }
  }

  set_level(output);

  while (1) {
    switch (state) {
      case kRamping:
        if (output == RAMP_SIZE) {
          delay_ms(1000);
        }

        options.ramping_up =
            (options.ramping_up && output < RAMP_SIZE) ||
            (!options.ramping_up && output == 1);

        if (output == RAMP_SIZE && options.freeze_on_high) {
          state = kFrozen;
          break;
        }

        if (options.ramping_up) {
          ++output;
        } else {
          --output;
        }

        set_level(output);
        delay_ms(RAMP_TIME*1000/RAMP_SIZE);
        break;

      case kFrozen:
        break;

      case kTurbo:
        // set_pwm(TURBO_PWM);
        blink(20, 500/20);
        break;

      case kConfig:
        // set_level(0);
        // _delay_ms(1000);
        // fast_presses = 0;

        // toggle_options((options ^ 0b00000001), 1);
        // toggle_options((options ^ 0b00000010), 2);
        // toggle_options((options ^ 0b00000100), 3);
        // toggle_options(DEFAULTS, 4);
        break;
    }

    // TODO Low voltage protection
  }

  // TODO Use ISR(TIMER0_OVF_vect) to reset fast_presses (use fast PWM)
}
