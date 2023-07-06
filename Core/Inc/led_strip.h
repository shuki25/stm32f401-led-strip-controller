/*
* seven_segment.h
* Created on: 2023-06-03
* Author: Joshua Butler, MD, MHI
*
* This file contains the function prototypes for the seven segment display
* functions using WS2818B LED strip.
*
*/

#ifndef LED_STRIP_H_
#define LED_STRIP_H_

#define LED_STRIP_ROTATE_LEFT  0
#define LED_STRIP_ROTATE_RIGHT 1

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include "main.h"
#include "misc.h"

    typedef enum {
        LED_STRIP_ERROR,
        LED_STRIP_OK,
        LED_STRIP_MALLOC_FAILED
    } led_strip_error_t;

    typedef struct {
        TIM_HandleTypeDef *htim;
        uint32_t channel;
        uint8_t counter_period;
        uint8_t duty_high;
        uint8_t duty_low;
        uint8_t max_brightness;
        uint8_t max_num_leds;
        uint8_t num_leds;
        uint8_t data_sent_flag;
        uint8_t sacrificial_led_flag;
        uint8_t **primary;
        uint8_t **secondary;
        uint8_t **working;
        uint8_t **mod;
        uint16_t *pwm_data;
    } led_strip_t;

    led_strip_error_t led_strip_init(led_strip_t *led_obj, TIM_HandleTypeDef *htim, const uint32_t channel, const uint8_t counter_period, const uint8_t num_leds, const uint8_t sacrificial_led_flag);
    void led_strip_set_primary_LED(led_strip_t *led_obj, uint8_t LEDnum, uint8_t Red, uint8_t Green, uint8_t Blue);
    void led_strip_set_secondary_LED(led_strip_t *led_obj, uint8_t LEDnum, uint8_t Red, uint8_t Green, uint8_t Blue);
    void led_strip_set_LED(led_strip_t *led_obj, uint8_t LEDnum, uint8_t Red, uint8_t Green, uint8_t Blue);
    void led_strip_set_brightness(led_strip_t *led_obj, uint8_t brightness);
    void led_strip_rotate(led_strip_t *led_obj, uint8_t direction);
    void led_strip_clear(led_strip_t *led_obj);
    void led_strip_fill(led_strip_t *led_obj, uint8_t Red, uint8_t Green, uint8_t Blue);
    void led_strip_fill_range(led_strip_t *led_obj, uint8_t Red, uint8_t Green, uint8_t Blue, uint8_t start, uint8_t end);
    void led_strip_WS2812_send(led_strip_t *led_obj);

#ifdef __cplusplus
}
#endif
#endif /* SEVEN_SEGMENT_H_ */
