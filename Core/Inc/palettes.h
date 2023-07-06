/*
 * Color palettes for FastLED effects (65-73).
 */

// From ColorWavesWithPalettes by Mark Kriegsman: https://gist.github.com/kriegsman/8281905786e8b2632aeb
// Unfortunately, these are stored in RAM!

// Color order: GRB with gamma correction

#ifndef __PALETTES_H__
#define __PALETTES_H__

#include <stdint.h>

void fill_pattern(uint32_t *led_strip, const uint32_t pattern[], uint8_t num_leds, uint8_t num_pattern);
void fill_pattern_reverse(uint32_t *led_strip, const uint32_t pattern[], uint8_t num_leds, uint8_t num_pattern);
void fill_gradient(uint32_t *led_strip, const uint8_t gp[], uint8_t num_leds);

#endif // _PALETTES_H