/*
 * led_strip_effect.c
 * Created Date: 2023-07-02
 * Author: Josh Butler, MD, MHI
 * 
 * This file contains the functions for controlling the light effect of LED strip.
 * 
 */

#include <string.h>
#include "led_strip_effect.h"
#include "palettes.h"
#include "misc.h"


void (*fx_list[NBR_FX])(led_strip_effect_t *);
void(*fx_list[NBR_FX])(led_strip_effect_t *) = {
    fx_solid,
    fx_police_wagtag
};

char *fx_name_list[NBR_FX] = {"Solid", "Police"};

led_strip_effect_error_t led_strip_effect_init(led_strip_effect_t *effect, led_strip_t *led, uint8_t num_leds)
{
    if (effect == NULL || led == NULL)
    {
        return LED_STRIP_EFFECT_ERROR;
    }
    else
    {
        memset(effect, 0, sizeof(led_strip_effect_t));
//        effect->effect_id = 0;
//        effect->led = NULL;
//        effect->speed = 0;
//        effect->duty_cycle = 0;
//        effect->brightness = 0;
//        effect->intensity = 0;
//        effect->fade_time = 0;

        effect->led = led;
        effect->num_leds = num_leds;
        effect->primary = (uint32_t *)rtos_malloc(sizeof(uint32_t) * num_leds);
        effect->secondary = (uint32_t *)rtos_malloc(sizeof(uint32_t) * num_leds);
        
        if (effect->primary == NULL || effect->secondary == NULL)
        {
            return LED_STRIP_EFFECT_MALLOC_FAILED;
        }
        
        memset(effect->primary, 0, sizeof(uint32_t) * num_leds);
        memset(effect->secondary, 0, sizeof(uint32_t) * num_leds);
        
        effect->solid_color = 0x000000;
        effect->flags = 0x00;
        
        return LED_STRIP_EFFECT_OK;
    }
}

void fx_update(led_strip_effect_t *effect)
{
    fx_list[effect->effect_id](effect);
}

void fx_solid(led_strip_effect_t *effect)
{
    if (!effect->initialized) {
        effect->is_loop = 0;
        effect->initialized = 1;
    }
    else if (effect->need_update) {
        effect->need_update = 0;
    }
    else {
        return;
    }
    
    led_strip_fill(effect->led, RED(effect->solid_color), GREEN(effect->solid_color), BLUE(effect->solid_color));
    led_strip_set_brightness(effect->led, effect->brightness);
    led_strip_WS2812_send(effect->led);
}

void fx_get_name(char *name, uint8_t effect_id, uint8_t max_len)
{
    uint8_t len = strlen(fx_name_list[effect_id]);
    if (len > max_len) {
        len = max_len;
    }
    if (effect_id >= NBR_FX) {
        return;
    }
    strncpy(name, fx_name_list[effect_id], len);
    name[len] = '\0';
}


void fx_police_wagtag(led_strip_effect_t *effect)
{
    if (!effect->initialized) {
        effect->is_loop = 1;
        effect->initialized = 1;
        effect->loop_count = 1;
        effect->delay_time = 250;
    }
    else if (effect->need_update) {
        effect->need_update = 0;
    }
    
    uint8_t half = effect->num_leds / 2;
    led_strip_clear(effect->led);
    
    if (effect->loop_count) {
        for (uint8_t i = 0; i < half; i++) {
            led_strip_set_LED(effect->led, i, 128, 0, 0);
            led_strip_set_LED(effect->led, i + half, 0, 0, 0);
        }
    } else {
        for (uint8_t i = 0; i < half; i++) {
            led_strip_set_LED(effect->led, i, 0, 0, 0);
            led_strip_set_LED(effect->led, i + half, 0, 0, 128);
        }
    }
    led_strip_set_brightness(effect->led, effect->brightness);
    led_strip_WS2812_send(effect->led);
    effect->loop_count = !effect->loop_count;
}
