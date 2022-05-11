
RGB_MATRIX_EFFECT(CustomEffect)

#ifdef RGB_MATRIX_CUSTOM_EFFECT_IMPLS

#include "rgb_matrix_user.h"

static void CustomEffect_complex_init(effect_params_t* params)
{
    rgb_matrix_user_init();
}

static bool CustomEffect_complex_run(effect_params_t* params)
{
    rgb_matrix_user_tick();

    uint8_t  NUM_LED= rgb_matrix_user_numLED();
    RGB_MATRIX_USE_LIMITS(led_min, led_max);
    for (uint8_t i = led_min; i < led_max; i++)
        if (i < NUM_LED)
            rgb_matrix_user_render(i);
    for (uint8_t i = led_min; i < led_max; i++)
        if (i < NUM_LED)
        {
            RGB col= rgb_matrix_user_getColor(i);
            rgb_matrix_set_color(i, col.r, col.g, col.b);
        }
    return rgb_matrix_check_finished_leds(led_max);
}

static bool CustomEffect(effect_params_t* params)
{
    if (params->init)
        CustomEffect_complex_init(params);
    return CustomEffect_complex_run(params);
}

#endif // RGB_MATRIX_CUSTOM_EFFECT_IMPLS