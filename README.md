# Firmware for ATtiny13-based flashlights

This firmware is compatible with the driver commonly found in Convoy flashlights like the S2+ or C8+. More specifically it expects an ATtiny13 microcontroller, single PWM output on PB1, (optionally) voltage devider on PB2 and a single on/off switch. The microcontroller must run at 4.8 MHz (fuses H:FF L:75).


## Features

- Two modes of operation: ramping and fixed levels
- Turbo mode: immediately switch to maximum output
- Battery check: one to four flashes
- Low voltage protection: flicker every 15 seconds if the voltage is low and short flashes if the voltage is critical
- Mode memory: start with last frozen ramp value or with last fixed level (off-time memory)
- Stop at high: in ramping UI, stop if ramping reaches maximum output
- Start at high: after the flashlight was off, start with the highest output
- Runtime configuration: options can be toggled via configuration menu


## Operation

### Configuration menu

1. Turn the light on
1. Shortly tap the switch 10+ times, the light will turn off
1. The light starts flashing from one to four, each group followed by a short burst of flashes. Turn off the light during the burst to toggle the option. Options are:
    1. Ramping or fixed levels
    1. Mode memory on or off
    1. Freeze on high
    1. Start on high

The default is: ramping UI, no mode memory, do not freeze on high, start on low


### Ramping UI

Turn the light on and it will start ramping up and down. Short tap the switch to freeze the current brightness. Tap again to resume ramping.

Tap two times to go into turbo mode. There is no timer, so make sure to monitor the temperature. Tap again to go back to ramping.


### Fixed level UI

There are four brightness levels in the fixed mode:

1. Low
1. Medium
1. High
1. Turbo

Turn the light on, it starts with a fixed output level. Tap the button to switch to next mode. After the highest mode it will start with the lowest mode again.


### Battery check

Tap five times to enter battery check mode. The flashlight starts blinking one to four times. The number of flashes corresponds to the remaining capacity / current voltage:

- 4 flashes: above 4.0 V (75%)
- 3 flashes: above 3.8 V (50%)
- 2 flashes: above 3.5 V (25%)
- 1 flashes: below 3.5 V

Tap again to return to normal flashlight mode.


### Low voltage protection

If the voltage is below 3.2 V the flashlight will flicker for half a second every 15 seconds. If the voltage is below 2.7 V the light will turn off and flash regularly to notify the operator that the light is still turned on but the battery is empty.


## Development

TODO


## License and acknowledgments

Thanks to ToyKeeper who was a great inspiration while writing this code. Her [flashlight firmware repository](https://launchpad.net/flashlight-firmware) contains a huge collection of flashlight firmwares.

Copyright (c) 2018 Sven Karsten Greiner  
This code is licensed under GPL 3 or any later version, see COPYING for details.