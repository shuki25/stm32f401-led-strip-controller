/*
 * color_patterns.h
 * Created Date: 2023-07-03
 * Author: Josh Butler, MD, MHI
 * 
 * This file color definitions for the LED strip and color patterns.
 * 
 **/

#ifndef _COLOR_PATTERNS_H_
#define _COLOR_PATTERNS_H_


/* Color defines in Green, Red, Blue order */
#define COLOR_BLACK         0x000000
#define COLOR_WHITE         0xFFFFFF
#define COLOR_RED           0x00FF00
#define COLOR_LIME          0xFF0000
#define COLOR_GREEN         0x800000
#define COLOR_FOREST_GREEN  0x8B2222
#define COLOR_BLUE          0x0000FF
#define COLOR_NAVY_BLUE     0x000080
#define COLOR_CYAN          0xFFFF00
#define COLOR_MAGENTA       0x00FFFF
#define COLOR_YELLOW        0xFFFF00
#define COLOR_ORANGE        0xA5FF00
#define COLOR_ORANGE_RED    0x45FF00
#define COLOR_HOT_PINK      0x69FFB4
#define COLOR_PURPLE        0x008080
#define COLOR_TEAL          0x800080
#define COLOR_BROWN         0x2AA52A

/* Color patterns */
const uint32_t pattern_rainbow[] = {
    COLOR_RED,
    COLOR_ORANGE,
    COLOR_YELLOW,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_PURPLE
};

const uint32_t pattern_rainbow_reverse[] = {
    COLOR_PURPLE,
    COLOR_BLUE,
    COLOR_GREEN,
    COLOR_YELLOW,
    COLOR_ORANGE,
    COLOR_RED
};

const uint32_t pattern_rainbow_cycle[] = {
    COLOR_RED,
    COLOR_ORANGE,
    COLOR_YELLOW,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_PURPLE,
    COLOR_PURPLE,
    COLOR_BLUE,
    COLOR_GREEN,
    COLOR_YELLOW,
    COLOR_ORANGE,
    COLOR_RED
};

const uint32_t *pattern_list[] = {
    pattern_rainbow,
    pattern_rainbow_reverse,
    pattern_rainbow_cycle
};

const char *pattern_name_list[] = {
    "Rainbow",
    "Rainbow Reverse",
    "Rainbow Cycle"
};

const uint8_t pattern_list_size = sizeof(pattern_list) / sizeof(pattern_list[0]);


#endif /* _COLOR_PATTERNS_H_ */