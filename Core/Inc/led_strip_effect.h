/*
 * led_strip_effect.h
 * Created Date: 2023-07-02
 * Author: Josh Butler, MD, MHI
 * 
 * This file contains the prototypes for controlling the light effect of LED strip.
 * 
 */

#ifndef _LED_STRIP_EFFECT_H_
#define _LED_STRIP_EFFECT_H_

#define GREEN(x) ((x & 0xff0000) >> 16)
#define RED(x) ((x & 0x00ff00) >> 8)
#define BLUE(x) (x & 0x0000ff)

#define NBR_FX 2

#ifdef __cplusplus
extern "C" {
#endif
    
#include <stddef.h>
    
#include "main.h"
#include "led_strip.h"
  
    typedef enum {
        LED_STRIP_EFFECT_ERROR,
        LED_STRIP_EFFECT_OK,
        LED_STRIP_EFFECT_MALLOC_FAILED
    } led_strip_effect_error_t;
    
    typedef struct
    {
        uint8_t effect_id;
        led_strip_t *led;
        uint8_t num_leds;
        uint32_t *primary;
        uint32_t *secondary;
        uint32_t solid_color;
        uint8_t speed;
        uint8_t duty_cycle;
        uint8_t brightness;
        uint8_t intensity;
        uint8_t delay_time;
        uint8_t loop_count;
        union {
            struct
            {
                uint8_t initialized : 1;
                uint8_t is_running : 1;
                uint8_t is_loop : 1;
                uint8_t is_done : 1;
                uint8_t need_update : 1;
            };
            uint8_t flags;
        };
    } led_strip_effect_t;
    
    led_strip_effect_error_t led_strip_effect_init(led_strip_effect_t *effect, led_strip_t *led, uint8_t num_leds);
    void fx_get_name(char *name, uint8_t effect_id, uint8_t max_len);
    void fx_update(led_strip_effect_t *effect);
    void fx_solid(led_strip_effect_t *effect);
    void fx_police_wagtag(led_strip_effect_t *effect);

#ifdef __cplusplus
}
#endif

#endif /* _LED_STRIP_EFFECT_H_ */
