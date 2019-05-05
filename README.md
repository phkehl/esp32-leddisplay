# ESP32 (esp-idf) component for driving dumb LED displays using I2S parallel mode DMA

## Introduction

This is a *leddisplay* component for the ESP-IDF [1] for ESP32. It can be used
to drive "dumb" LED displays (a.k.a. "Px displays").

See [leddisplay.h](include/leddisplay.h) for the API.

This code is meant for directly connecting the ESP32 to a display (possibly via
a level shifter). The pinout configuration is available via the sdkconfig ("make
menuconfig"). See [Kconfig](Kconfig) for the defaults.

See [leddisplay.c](src/leddisplay.c) for more information on the code, for
references to the origin of the concept and code, and for copyright and license
information.

There is an example in the *examples* directory ([leddisplay_test.c](examples/leddisplay_test/main/leddisplay_test.c)).

Happy hacking!

## Usage

How to use:

Clone this repository into *components/leddisplay* of your esp-adf project:

```
cd /path/to/your/project
git clone https://github.com/phkehl/esp32-leddisplay.git components/leddisplay
```

Or add it as a git submodule:

```
cd /path/to/your/project
git submodule add -b master https://github.com/phkehl/esp32-leddisplay.git components/leddisplay
```

## Notes

- This has been tested with a 64x32 1/16 scan type display. It may or may not
  work with other configurations.

- The code is relatively slow, in particular the pixel based API (`leddisplay_xy_rgb()`),
 so perhaps increase the CPU clock in sdkconfig (e.g. `CONFIG_ESP32_DEFAULT_CPU_FREQ_240=y`,
 `CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ=240`).

## See also

Some generic LED display stuff:

- https://www.sparkfun.com/sparkx/blog/2650

Similar libraries, mostly for the Arduino environment:

- https://github.com/2dom/PxMatrix
  - https://www.instructables.com/id/RGB-LED-Matrix-With-an-ESP8266/
- https://github.com/hzeller/rpi-rgb-led-matrix
- http://docs.pixelmatix.com/SmartMatrix/
  - https://github.com/pixelmatix/SmartMatrix/tree/teensylc
- https://github.com/mrfaptastic/ESP32-RGB64x32MatrixPanel-I2S-DMA

## References

- [1] https://github.com/espressif/esp-idf

