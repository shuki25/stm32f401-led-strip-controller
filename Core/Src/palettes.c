/*
 * palettes.c
 * Created Date: 2023-07-03
 * Author: Josh Butler, MD, MHI
 * 
 * This file contains the functions for managing the color palattes.
 * 
 **/

#include "palettes.h"
#include "gamma8.h"

void fill_pattern(uint32_t *led_strip, const uint32_t pattern[], uint8_t num_leds, uint8_t num_pattern) {
    uint8_t i;
    uint8_t j;
    uint8_t k;
    
    for (i = 0; i < num_leds; i++) {
        j = i % num_pattern;
        led_strip[i] = pattern[j];
    }
}

void fill_pattern_reverse(uint32_t *led_strip, const uint32_t pattern[], uint8_t num_leds, uint8_t num_pattern) {
    uint8_t i;
    uint8_t j;
    uint8_t k;
    
    for (i = 0; i < num_leds; i++) {
        j = i % num_pattern;
        k = i / num_pattern;
        led_strip[i] = pattern[(num_pattern - j)];
    }
}

void fill_gradient(uint32_t *led_strip, const uint8_t gp[], uint8_t num_leds) {
    uint8_t num_segments = 0;
    uint8_t r, g, b;
    
    while (gp[num_segments * 4] != 255 && num_segments < 16) {
        num_segments++;
    }
    if (gp[num_segments * 4] != 255) {
        // Invalid gradient palette, cannot interpolate
        return;
    }
    
    // Calculate the step size for each segment
    float step_size = (float)(num_leds - 1) / num_segments;

    // index of the led strip
    uint8_t led_index = 0;
    
    // Loop through each segment
    for (uint32_t i = 0; i < num_segments; i++) {
        // Get the start and end position of the segment
        uint8_t start_pos = gp[i * 4];
        uint8_t end_pos = gp[(i + 1) * 4];

        // Get the color values at the start and end positions
        uint8_t start_r = gp[i * 4 + 1];
        uint8_t start_g = gp[i * 4 + 2];
        uint8_t start_b = gp[i * 4 + 3];
        uint8_t end_r = gp[(i + 1) * 4 + 1];
        uint8_t end_g = gp[(i + 1) * 4 + 2];
        uint8_t end_b = gp[(i + 1) * 4 + 3];

        float step_pos = (((float)end_pos - (float)start_pos) / 256.0) * (float)num_leds;
        
        // Calculate the color step sizes for each component
        float step_r = (float)(end_r - start_r) / step_pos;
        float step_g = (float)(end_g - start_g) / step_pos;
        float step_b = (float)(end_b - start_b) / step_pos;

        
        // Loop through the positions within the segment
        for (uint32_t j = 0; j <= (uint8_t)step_pos; j++) {
            // Calculate the interpolated color values
            if (step_r < 0) {
                r = start_r - (uint8_t)(step_r * j * -1);
            } else {
                r = start_r + (uint8_t)(step_r * j);
            }
            if (step_g < 0) {
                g = start_g - (uint8_t)(step_g * j * -1);
            } else {
                g = start_g + (uint8_t)(step_g * j);
            }
            if (step_b < 0) {
                b = start_b - (uint8_t)(step_b * j * -1);
            } else {
                b = start_b + (uint8_t)(step_b * j);
            }

            // Set the color in the palette
            led_strip[led_index++] = (0x00 << 24) | (g << 16) | (r << 8) | b;
            if (led_index >= num_leds) {
                return;
            }
        }
    }
    
    return;
}