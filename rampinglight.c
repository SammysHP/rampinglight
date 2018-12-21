/*
 * Copyright (c) 2018 Sven Karsten Greiner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
// #include <avr/sleep.h>

#define PWM_PIN PB1
#define VOLTAGE_PIN PB2

#define RAMP_TIME 3
#define RAMP_SIZE sizeof(ramp_values)
#define RAMP_VALUES 5,5,5,5,5,6,6,6,6,7,7,8,8,9,10,11,12,13,14,15,17,18,20,22,23,25,28,30,32,35,38,41,44,47,51,55,59,63,67,71,76,81,86,92,97,103,109,116,122,129,136,144,151,159,167,176,185,194,203,213,223,233,244,255
#define TURBO_PWM 255

#define FIXED_SIZE sizeof(fixed_values)
#define FIXED_VALUES 5,35,118,255

#define CBD_BYTES 4
#define CBD_PATTERN 0xAA

#define EEPROM_SIZE 64
#define EEPROM_OPTIONS (EEPROM_SIZE-1)

/*
#define ADC_CHANNEL 0x01    // MUX 01 corresponds with PB2
#define ADC_DIDR    ADC1D   // Digital input disable bit corresponding with PB2
#define ADC_PRSCL   0x06    // clk/64
*/

/**
 * States of the state machine.
 */
enum State {
  kDefault,    // Special decision state
  kRamping,    // Ramping up and down
  kFrozen,     // Frozen ramping level
  kTurbo,      // Full power
  kFixed,      // Fixed mode
  kBattcheck,  // Battery level
  kConfig,     // Config menu
};

/**
 * Persistent configuration of the flashlight
 */
typedef union {
  uint8_t raw;
  struct {
    unsigned fixed_mode : 1;
    unsigned mode_memory : 1; // TODO
    unsigned freeze_on_high : 1;
    unsigned start_high : 1;
  };
} Options;

const uint8_t __flash ramp_values[] = { RAMP_VALUES };
const uint8_t __flash fixed_values[] = { FIXED_VALUES };

uint8_t microticks = 0;  // TODO Put into register?
uint8_t ticks = 0;

uint8_t cold_boot_detect[CBD_BYTES] __attribute__((section(".noinit")));
Options options __attribute__((section(".noinit")));
enum State state __attribute__((section(".noinit")));
uint8_t output __attribute__((section(".noinit")));
uint8_t fast_presses __attribute__((section(".noinit")));
uint8_t ramping_up __attribute__((section(".noinit")));

/**
 * Busy wait delay with ms resolution. This function allows to choose the
 * duration during runtime.
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
 * @param level Index in ramp_values or fixed_values depending on fixed_mode
 */
void set_level(uint8_t level) {
  if (level == 0) {
    set_pwm(0);
  } else {
    if (options.fixed_mode) {
      set_pwm(fixed_values[level - 1]);
    } else {
      set_pwm(ramp_values[level - 1]);
    }
  }
}

/**
 * Blink or strobe the LED on low intensity.
 *
 * @param count Number of flashes.
 * @param speed Duration of a single flash.
 */
void blink(uint8_t count, uint16_t speed) {
  const uint8_t old_pwm = OCR0B;
  while (count--) {
    set_pwm(40);
    delay_ms(speed);
    set_pwm(0);
    delay_ms(speed);
    delay_ms(speed);
  }
  OCR0B = old_pwm;
}

/**
 * Write current options to EEPROM.
 */
void save_options(void) {
  // Not necessary to use eeprom_update_byte() because options will usually be
  // written only if they have changed.
  // Store inverted so that a read from uninitialized EEPROM results in 0.
  eeprom_write_byte((uint8_t *)EEPROM_OPTIONS, ~options.raw);
}

/**
 * Restore state from EEPROM.
 */
void restore_state(void) {
  options.raw = ~eeprom_read_byte((uint8_t *)EEPROM_OPTIONS);
}

/**
 * User interface to toggle an option.
 *
 * @param new_opts New options
 * @param flashes  Number of flashes
 */
void toggle_option(uint8_t new_opts, uint8_t flashes) {
  blink(flashes, 250);
  uint8_t old_options = options.raw;
  options.raw = new_opts;
  save_options();
  blink(32, 500/32);
  options.raw = old_options;
  save_options();
  _delay_ms(1000);
}

/**
 * Timer0 overflow interrupt handler. With phase correct PWM this interrupt is
 * executed with a frequency of F_CPU/510.
 */
ISR(TIM0_OVF_vect) {
  if (!--microticks) {
    ++ticks;
  }

  if (ticks == 15) {
    fast_presses = 0;
  }
}

/**
 * Entry point.
 */
int main(void) {
  uint8_t coldboot = 0;

  restore_state();

  // Phase correct PWM, system clock without prescaler
  // Frequency will be F_CPU/510
  TCCR0A = (1 << COM0B1) | (1 << WGM00);
  TCCR0B = (1 << CS00);

  // Enable timer overflow interrupt
  TIMSK0 |= (1 << TOIE0);

  // Set PWM pin to output
  DDRB |= (1 << PWM_PIN);

  sei();

  // Cold boot detection
  for (int i = CBD_BYTES - 1; i >= 0; --i) {
    if (cold_boot_detect[i] != CBD_PATTERN) {
      coldboot = 1;
    }
    cold_boot_detect[i] = CBD_PATTERN;
  }

  if (coldboot) {  // Initialize state after the flashlight was switched off for some time
    state = kDefault;
    fast_presses = 0;
    ramping_up = 1;

    if (options.start_high) {
      output = options.fixed_mode ? FIXED_SIZE : RAMP_SIZE;
    } else {
      output = 1;
    }
  } else {  // User has tapped the power button
    ++fast_presses;

    // TODO Optimize overflow handling
    if (fast_presses > 10) {
      fast_presses = 10;
    }

    // Input handling
    if (options.fixed_mode) {
      switch (fast_presses) {
        case FIXED_SIZE + 1:
          state = kBattcheck;
          break;

        case 10:
          state = kConfig;
          break;

        default:
          output = (output % FIXED_SIZE) + 1;
          state = kFixed;
          break;
      }
    } else {
      switch (fast_presses) {
        case 2:
          state = kTurbo;
          break;

        case 3:
          state = kBattcheck;
          break;

        case 10:
          state = kConfig;
          break;

        default:
          switch (state) {
            case kRamping:
              state = kFrozen;
              break;

            default:
              state = kRamping;
              break;
          }
          break;
      }
    }
  }

  set_level(output);

  while (1) {
    switch (state) {
      case kDefault:
        // This is a special state that does nothing but deciding which is the
        // correct state for the current mode (ramping vs fixed)
        state = options.fixed_mode ? kFixed : kRamping;
        continue;  // Skip main loop cycle and continue with correct mode

      case kRamping:
        ramping_up =
            (ramping_up && output < RAMP_SIZE) ||
            (!ramping_up && output == 1);

        output += ramping_up ? 1 : -1;
        set_level(output);

        if (options.freeze_on_high && output == RAMP_SIZE) {
          state = kFrozen;
          break;
        }

        if (output == RAMP_SIZE) {
          delay_ms(1000);
        } else {
          delay_ms(RAMP_TIME*1000/RAMP_SIZE);
        }

        break;

      case kFrozen:
        break;

      case kTurbo:
        // TODO
        // set_pwm(TURBO_PWM);
        blink(20, 500/20);
        break;

      case kFixed:
        break;

      case kBattcheck:
        // TODO
        blink(20, 1000/20);
        break;

      case kConfig:
        set_level(0);
        _delay_ms(1000);

        state = kDefault;

        // This assumes that the bit field starts at the least significant bit
        uint8_t opts = options.raw;
        toggle_option(opts ^ 0b00000001, 1);  // Fixed mode
        toggle_option(opts ^ 0b00000010, 2);  // Mode memory
        toggle_option(opts ^ 0b00000100, 3);  // Freeze on high
        toggle_option(opts ^ 0b00001000, 4);  // Start with high

        break;
    }

    // if (ticks == 50) {
    //   // TODO Low voltage protection
    //   blink(5, 20);
    // }
  }
}
