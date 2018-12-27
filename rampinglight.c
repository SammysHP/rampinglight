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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

// Optional features
#define BATTCHECK
#define LOW_VOLTAGE_PROTECTION

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/atomic.h>

#define PWM_PIN PB1
#define VOLTAGE_ADC_CHANNEL 0x01  // ADC1/PB2

#define FLASH_TIME 20
#define FLICKER_TIME 2

#define BAT_LOW  141  // ~3.2 V
#define BAT_CRIT 120  // ~2.7 V
#define BAT_75P  175  // ~4.0 V
#define BAT_50P  167  // ~3.8 V
#define BAT_25P  154  // ~3.5 V

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
#define EEPROM_OUTPUT_WL_BYTES 16

/**
 * States of the state machine.
 */
enum State {
  kDefault,    // Special decision state
  kRamping,    // Ramping up and down
  kFrozen,     // Frozen ramping level
  kTurbo,      // Full power
  kFixed,      // Fixed mode
  kConfig,     // Config menu
#ifdef BATTCHECK
  kBattcheck,  // Battery level
#endif  // ifdef BATTCHECK
};

/**
 * Persistent configuration of the flashlight
 */
typedef union {
  uint8_t raw;
  struct {
    unsigned fixed_mode : 1;
    unsigned mode_memory : 1;
    unsigned freeze_on_high : 1;
    unsigned start_high : 1;
  };
} Options;

const uint8_t __flash ramp_values[] = { RAMP_VALUES };
const uint8_t __flash fixed_values[] = { FIXED_VALUES };
const uint8_t __flash voltage_table[] = { 0, BAT_25P, BAT_50P, BAT_75P };

uint8_t cold_boot_detect[CBD_BYTES] __attribute__((section(".noinit")));
enum State state __attribute__((section(".noinit")));
uint8_t output __attribute__((section(".noinit")));
uint8_t fast_presses __attribute__((section(".noinit")));
uint8_t ramping_up __attribute__((section(".noinit")));

register Options options asm("r7");
register uint8_t output_eeprom asm("r6");
register uint8_t output_eeprom_pos asm("r5");
register uint8_t microticks asm("r4");
register uint8_t ticks asm("r3");

/**
 * Busy wait delay with 10 ms resolution. This function allows to choose the
 * duration during runtime.
 *
 * @param duration Wait duration in n*10 ms.
 */
void delay_10ms(uint8_t duration) {
  while (duration--) {
    _delay_ms(10);
  }
}

/**
 * Busy wait one second. Saves some space because call does not require setup
 * of arguments.
 */
void delay_s(void) {
  delay_10ms(100);
}

/**
 * Set output, i.e. compare register.
 *
 * @param pwm Raw PWM level.
 */
static inline void set_pwm(uint8_t pwm) {
  OCR0B = pwm;
  if (!pwm) {
    TCCR0A &= ~(1 << COM0B1);
  } else {
    TCCR0A |= (1 << COM0B1);
  }
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
 * @param speed Duration of a single flash in n*10 ms.
 */
void blink(uint8_t count, const uint8_t speed) {
  const uint8_t old_pwm = OCR0B;
  while (count--) {
    set_pwm(40);
    delay_10ms(speed);
    set_pwm(0);
    delay_10ms(speed);
    delay_10ms(speed);
  }
  set_pwm(old_pwm);
}

/**
 * Internal function to erase or dirty write a byte to EEPROM.
 *
 * @param address Address in EEPROM
 * @param data    Data that should be written to address
 * @param eecr    Write (EEPM1) or erase (EEPM0) operation
 */
static void eeprom_erase_or_write_byte(const uint8_t address, const uint8_t data, const uint8_t eecr) {
  while (EECR & (1 << EEPE));
  EECR = eecr;
  EEARL = address;
  EEDR = data;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    EECR |= (1<<EEMPE);
    EECR |= (1<<EEPE);
  }
}

/**
 * Dirty write a byte to EEPROM without erase cycle.
 *
 * @param address Address in EEPROM
 * @param data    Data that should be written to address
 */
void eeprom_onlywrite_byte(const uint8_t address, const uint8_t data) {
  eeprom_erase_or_write_byte(address, data, (1 << EEPM1));
}

/**
 * Erase a byte in EEPROM without writing anything to it.
 *
 * @param address Address in EEPROM
 */
void eeprom_erase_byte(const uint8_t address) {
  eeprom_erase_or_write_byte(address, 0, (1 << EEPM0));
}

/**
 * Write current options to EEPROM.
 */
void save_options(void) {
  // Not necessary to use eeprom_update_byte() because options will usually be
  // written only if they have changed.
  // Using non-atomic erase and write to save some bytes by not using avr-libc
  // method eeprom_write_byte().
  // Store inverted so that a read from uninitialized EEPROM results in 0.
  eeprom_erase_byte(EEPROM_OPTIONS);
  eeprom_onlywrite_byte(EEPROM_OPTIONS, ~options.raw);
}

/**
 * Write current output level to EEPROM with wear leveling.
 */
void save_output(void) {
  if (!options.mode_memory) return;

  if (!output_eeprom_pos) {
    uint8_t i = EEPROM_OUTPUT_WL_BYTES;
    do {
      --i;
      eeprom_erase_byte(i);
    } while (i);
  }

  // Store inverted so that an output of 0 (invalid in this code) can be used
  // to detect unused bytes in the EEPROM (0xFF)
  eeprom_onlywrite_byte(output_eeprom_pos, ~output);
}

/**
 * Restore state from EEPROM.
 */
void restore_state(void) {
  options.raw = ~eeprom_read_byte((uint8_t *)EEPROM_OPTIONS);
  if (!options.mode_memory) return;

  // From back to front find the first byte that is not uninitialized EEPROM
  output_eeprom_pos = EEPROM_OUTPUT_WL_BYTES - 1;
  while (output_eeprom_pos && !(output_eeprom = ~eeprom_read_byte((uint8_t *)output_eeprom_pos))) {
    --output_eeprom_pos;
  }
  if (output_eeprom_pos == EEPROM_OUTPUT_WL_BYTES - 1) {
    output_eeprom_pos = 0;
  } else {
    ++output_eeprom_pos;
  }
}

/**
 * User interface to toggle an option.
 *
 * @param new_opts New options
 * @param flashes  Number of flashes
 */
void toggle_option(uint8_t new_opts, uint8_t flashes) {
  blink(flashes, FLASH_TIME);
  uint8_t old_options = options.raw;
  options.raw = new_opts;
  save_options();
  blink(24, FLICKER_TIME);
  options.raw = old_options;
  save_options();
  delay_s();
}

#if defined(LOW_VOLTAGE_PROTECTION) || defined(BATTCHECK)
/**
 * Measure battery voltage.
 *
 * @return 8 bit battery voltage as ADC value
 */
static uint8_t battery_voltage(void) {
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADCH;
}
#endif  // if defined(LOW_VOLTAGE_PROTECTION) || defined(BATTCHECK)

/**
 * Timer0 overflow interrupt handler.
 * Frequency will be F_CPU/(8*256) = 2343.75 Hz.
 *
 * One microtick: ~0.427 ms
 * One tick (256 microticks): ~0.109227 s
 */
ISR(TIM0_OVF_vect) {
  if (!--microticks) {
    ++ticks;
  }

  if (ticks == 4) {  // ~440 ms
    fast_presses = 0;
  }
}

/**
 * Entry point.
 */
int main(void) {
  microticks = 0;
  ticks = 0;

  // Fast PWM, system clock with /8 prescaler
  // Frequency will be F_CPU/(8*256) = 2343.75 Hz
  TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
  TCCR0B = (1 << CS01);

  // Enable timer overflow interrupt
  TIMSK0 |= (1 << TOIE0);  // TODO Optimization: no or

  // Set PWM pin to output
  DDRB |= (1 << PWM_PIN);  // TODO Optimization: no or

#if defined(LOW_VOLTAGE_PROTECTION) || defined(BATTCHECK)
  ADMUX =
    (1 << REFS0) |        // Internal 1.1 V reference
    (1 << ADLAR) |        // Left adjust
    VOLTAGE_ADC_CHANNEL;  // ADC MUX channel

  ADCSRA =
    (1 << ADEN) |                 // Enable ADC
    (1 << ADSC) |                 // Start conversion
    (1 << ADPS2) | (1 << ADPS1);  // Set /64 prescaler
#endif  // if defined(LOW_VOLTAGE_PROTECTION) || defined(BATTCHECK)

  restore_state();

  sei();  // Restore state before enabling interrupts (EEPROM read)!

  // Cold boot detection
  uint8_t coldboot = 0;
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

    if (options.mode_memory && output_eeprom) {
      output = output_eeprom;
    } else {
      if (options.start_high) {
        output = options.fixed_mode ? FIXED_SIZE : RAMP_SIZE;
      } else {
        output = 1;
      }
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
#ifdef BATTCHECK
        case FIXED_SIZE + 1:
          state = kBattcheck;
          break;
#endif  // ifdef BATTCHECK

        case 10:
          state = kConfig;
          break;

        default:
          output = (output % FIXED_SIZE) + 1;
          state = kFixed;
          save_output();
          break;
      }
    } else {
      switch (fast_presses) {
        case 2:
          state = kTurbo;
          break;

#ifdef BATTCHECK
        case FIXED_SIZE + 1:
          state = kBattcheck;
          break;
#endif  // ifdef BATTCHECK

        case 10:
          state = kConfig;
          break;

        default:
          switch (state) {
            case kRamping:
              state = kFrozen;
              save_output();
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
        if (options.fixed_mode) {
          state = kFixed;
        } else {
          if (options.mode_memory) {
            state = kFrozen;
          } else {
            state = kRamping;
          }
        }
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
          delay_s();
        } else {
          delay_10ms(RAMP_TIME*100/RAMP_SIZE);
        }

        break;

      case kFrozen:
        break;

      case kTurbo:
        set_pwm(TURBO_PWM);
        break;

      case kFixed:
        break;

#ifdef BATTCHECK
      case kBattcheck:
        set_pwm(0);

        const uint8_t voltage = battery_voltage();

        uint8_t i = sizeof(voltage_table) - 1;
        while (voltage < voltage_table[i]) {
          --i;
        }

        blink(i+1, FLASH_TIME);
        delay_s();
        break;
#endif  // ifdef BATTCHECK

      case kConfig:
        set_level(0);
        delay_s();

        state = kDefault;

        // This assumes that the bit field starts at the least significant bit
        uint8_t opts = options.raw;
        toggle_option(opts ^ 0b00000001, 1);  // Fixed mode
        toggle_option(opts ^ 0b00000010, 2);  // Mode memory
        toggle_option(opts ^ 0b00000100, 3);  // Freeze on high
        toggle_option(opts ^ 0b00001000, 4);  // Start with high

        break;
    }

#ifdef LOW_VOLTAGE_PROTECTION
    if ((ticks & 0x7F) == 14) {  // Every ~14 s starting after ~1.5 s
      // TODO Take several measurements for noise filtering?
      const uint8_t voltage = battery_voltage();
      if (voltage <= BAT_CRIT) {
        set_pwm(0);
        while (1) {
          // Do not go to sleep, but flash every few seconds to notify the user
          // that the flashlight is still turned on but the battery is dying.
          // TODO If free space in flash, disable as many components as possible
          blink(3, FLASH_TIME/2);
          uint8_t delay_seconds;
          asm volatile (
            "ldi %[i], 5"    "\n\t"
            "0:"             "\n\t"
            "rcall  delay_s" "\n\t"
            "dec    %[i]"    "\n\t"
            "brne   0b"      "\n\t"
              : [i] "=&r" (delay_seconds)
          );
        }
      } else if (voltage <= BAT_LOW) {
        blink(16, FLICKER_TIME);
      }
    }
#endif  // ifdef LOW_VOLTAGE_PROTECTION
  }
}
