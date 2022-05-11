
#include "rgb_matrix_user.h"
#include "quantum/rgb_matrix/rgb_matrix_types.h"
#include "quantum/rgb_matrix/rgb_matrix.h"
#include "quantum/led.h"
#include "tmk_core/protocol/host.h"

#include <math.h>

#define RGB_NUM_COL                 (210)       // default QMK settings is 224
#define RGB_NUM_ROW                 (56)        // default QMK settings is 64

#define NUM_LED                     (119)       // may different from DRIVER_LED_TOTAL as some manufacturer may use the same value for both ISO and ANSI layout
#define NUM_KEYS                    (99)
#define MAX_KEY_SCAN                (99)

#define F5_KEY_IDX                  (5)
#define F7_KEY_IDX                  (7)
#define ENTER_KEY_IDX               (66)
#define NUMPAD_ENTER_KEY_IDX        (86)
#define SPACE_KEY_IDX               (90)
#define BACKSPACE_KEY_IDX           (31)
#define NUM_LOCK_KEY_IDX            (32)
#define CAPS_LOCK_KEY_IDX           (54)
#define NUM_LOCK_LED_KEY_IDX        NO_LED
#define CAPS_LOCK_LED_KEY_IDX       NO_LED
#define SCROLL_LOCK_LED_KEY_IDX     NO_LED

typedef struct float3_s
{
	float x;
	float y;
	float z;
} float3;

float clamp(float x, float minVal, float maxVal)
{
    return (x > maxVal ? maxVal : (x < minVal ? minVal : x));
}

float square(float x)
{
    return x * x;
}

float smoothStep(float t)
{
    return t*t*(3.0f - 2.0f*t);
}

float lerp(float from, float to, float t)
{
	return (to - from) * t + from;
}

float3 lerp_f3(float3 from, float3 to, float t)
{
    float3 ret=
    {
        lerp(from.x, to.x, t),
        lerp(from.y, to.y, t),
        lerp(from.z, to.z, t)
    };
    return ret;
}

float3 add_f3(float3 a, float3 b)
{
    float3 ret=
    {
        a.x + b.x,
        a.y + b.y,
        a.z + b.z,
    };
    return ret;
}

float3 add_f3_s(float3 f3, float s)
{
    float3 ret=
    {
        f3.x + s,
        f3.y + s,
        f3.z + s,
    };
    return ret;
}

float3 mul_f3_s(float3 f3, float s)
{
    float3 ret=
    {
        f3.x * s,
        f3.y * s,
        f3.z * s,
    };
    return ret;
}

RGB led_buffer[NUM_LED];

float       render_pass_weight_ctrl = 0.0f;
float       render_pass_weight_shift= 0.0f;
float       render_pass_weight_alt  = 0.0f;

#define UPDATE_KEY_PRESS_NUM_PER_FRAME  (33)
#define PULSE_NUM_MAX                   (16)
#define PULSE_TRAVEL_TIME               (0.5f)

#define RIPPLE_NUM_MAX                   (16)
#define RIPPLE_TRAVEL_TIME               (0.35f)

uint32_t    timer_val = 0;
uint32_t    timer_tick= 0;
uint32_t    timer_tick_last= 0;

uint8_t     key_pressed_state[NUM_KEYS];    // 0 => not pressed
uint8_t     scan_to_key_idx[MAX_KEY_SCAN];
uint8_t     update_key_press_idx= 0;
float       pulse_life[PULSE_NUM_MAX];      // ring buffer
uint8_t     pulse_next_free_idx = 0;
uint8_t     pulse_active_idx    = 0;
uint8_t     pulse_active_num    = 0;

typedef struct RippleData_s
{
	float life; // remain life in secodn
	float x;    // origin X
	float y;    // origin Y
} RippleData;

RippleData  ripple_data[RIPPLE_NUM_MAX];  // ring buffer
uint8_t     ripple_next_free_idx    = 0;
uint8_t     ripple_active_idx       = 0;
uint8_t     ripple_active_num       = 0;

#define IDLE_WAKE_UP_SLASH_TIME             (0.7f)
#define IDLE_WAKE_UP_SHOW_VERT_LINE_TIME    (0.2f)
#define IDLE_WAKE_UP_MOVE_VERT_LINE_TIME    (0.45f)
#define IDLE_ENTER_IDLE_TIME                (0.25f)
#define IDLE_TIME_THRESHOLD                 (14.0f * 60.0f)
enum IdleState
{
    IdleState_Idle,
    IdleState_WakingUpFromIdle,
    IdleState_WakingUpFromEnteringIdle,
    IdleState_Awake,
    IdleState_EnteringIdle,
    IdleState_Num
};
uint8_t     idle_state= IdleState_WakingUpFromIdle;
float       idle_state_timer= 0.0f;
float       idle_timer      = 0.0f;


static void tick_led(void);
static void render_led(int i);

void rgb_matrix_user_init(void)
{
    for (uint8_t i = 0; i < NUM_LED; i++)
    {
        led_buffer[i].r = 0;
        led_buffer[i].g = 0;
        led_buffer[i].b = 0;
    }
    
    // init key press state
    for (uint8_t i = 0; i < NUM_KEYS; ++i)
        key_pressed_state[i]= 0;
    
    // initialize key map table
    for (uint8_t i = 0; i < MAX_KEY_SCAN; ++i)
        scan_to_key_idx[i]= NO_LED;
    for (uint8_t y = 0; y < MATRIX_ROWS; ++y)
        for (uint8_t x = 0; x < MATRIX_COLS; ++x)
        {
            uint8_t idx= g_led_config.matrix_co[y][x];
            if (idx < MAX_KEY_SCAN)
                scan_to_key_idx[idx]= idx;
        }
    
    // init pulse
    for(uint8_t i=0; i<PULSE_NUM_MAX; ++i)
        pulse_life[i]= 0.0f;

    // init ripple
    for(uint8_t i=0; i<RIPPLE_NUM_MAX; ++i)
        ripple_data[i].life= 0.0f;

    timer_val= timer_read32();
}

void rgb_matrix_user_tick(void)
{
    tick_led();
}

void add_reactive_ripple(float originX, float originY)
{
    ripple_data[ripple_next_free_idx].life  = RIPPLE_TRAVEL_TIME;
    ripple_data[ripple_next_free_idx].x     = originX;
    ripple_data[ripple_next_free_idx].y     = originY;
    ripple_next_free_idx                    = (ripple_next_free_idx + 1) % RIPPLE_NUM_MAX;
    ++ripple_active_num;
}

static bool cal_breath_col(float3 col0, float3 col1, float colT, float layerAlpha, float* accum_inv_alpha, float3* outCol)
{
    float invAlpha= *accum_inv_alpha;
    float3	col = lerp_f3(col0, col1, colT);
    *outCol     = add_f3(*outCol, mul_f3_s(col, layerAlpha * invAlpha));

    invAlpha            *= 1.0f - layerAlpha;
    *accum_inv_alpha    = invAlpha;
    return layerAlpha < 1.0f;
}

static void change_idle_state(uint8_t newState)
{
	if (!(idle_state == IdleState_EnteringIdle && idle_state == IdleState_WakingUpFromEnteringIdle))
		idle_state_timer= 0.0f;
	idle_state = newState;
}

void rgb_matrix_user_process(uint8_t row, uint8_t col, bool pressed)
{
    uint8_t key_scan= g_led_config.matrix_co[row][col];

    // update idle state
    {
    	if (idle_state == IdleState_Idle)
            change_idle_state(IdleState_WakingUpFromIdle);
        else if (idle_state == IdleState_EnteringIdle)
            change_idle_state(IdleState_WakingUpFromEnteringIdle);

        // reset idle state
        idle_timer= 0;
    }

    if (key_scan >= MAX_KEY_SCAN)
        return;    // should not enter this case, need to update the scan_to_key_idx table

    uint8_t led_idx= scan_to_key_idx[key_scan];
    if (led_idx >= NUM_KEYS)
        return;    // not a key

    if (pressed)
        key_pressed_state[led_idx]= 255;
    else
    {
        key_pressed_state[led_idx]= 254;

        // launch pulse when releasing back space key
        if (key_scan == BACKSPACE_KEY_IDX) 
        {
            pulse_life[pulse_next_free_idx] = PULSE_TRAVEL_TIME;
            pulse_next_free_idx             = (pulse_next_free_idx + 1) % PULSE_NUM_MAX;
            ++pulse_active_num;
        }
        else if (key_scan == F5_KEY_IDX)  // F5
            add_reactive_ripple(22.0f, 2.0f);
        else if (key_scan == F7_KEY_IDX)  // F7
            add_reactive_ripple(30.0f, 2.0f);
        else if (key_scan == ENTER_KEY_IDX)  // enter
            add_reactive_ripple(54.0f,15.0f);
        else if (key_scan == NUMPAD_ENTER_KEY_IDX)  // numpad enter
            add_reactive_ripple(72.0f,19.0f);
        else if (key_scan == SPACE_KEY_IDX)  // space bar
            add_reactive_ripple(27.0f,22.0f);
    }
    return;
}

void rgb_matrix_user_sleep(void)
{
    if (idle_state == IdleState_Awake)
    {
        idle_timer= IDLE_TIME_THRESHOLD - 0.1f;
    }
}

static void tick_led()
{
    uint32_t timer_val_new  = timer_read32();
    uint32_t deltaT_ms      = TIMER_DIFF_32(timer_val_new, timer_val);
    deltaT_ms               = deltaT_ms > 100 ? 100 : deltaT_ms;       // clamp it in case of lag or warp around, in ms
    float    deltaT         = deltaT_ms * 0.001f;                      // convert to seconds
    timer_tick              += deltaT_ms;
    timer_tick              = timer_tick % (43200 * 1000);             // warp around 0.5 day, .tick unit is ms
    timer_val               = timer_val_new;

    // update modifier weight
    {
        const float     blendTime   = 0.125f;
        const float     blendSpeed	= 1.0f/blendTime;
        float           blendDelta  = blendSpeed * deltaT;

        uint8_t modifers= get_mods();
        bool isHeldCtrl = modifers & MOD_MASK_CTRL  ;
        bool isHeldShift= modifers & MOD_MASK_SHIFT ;
        bool isHeldAlt  = modifers & MOD_MASK_ALT   ;
    
        render_pass_weight_ctrl = clamp(render_pass_weight_ctrl + (isHeldCtrl   ? blendDelta : -blendDelta) , 0.0f, 1.0f);
        render_pass_weight_shift= clamp(render_pass_weight_shift+ (isHeldShift  ? blendDelta : -blendDelta) , 0.0f, 1.0f);
        render_pass_weight_alt  = clamp(render_pass_weight_alt  + (isHeldAlt    ? blendDelta : -blendDelta) , 0.0f, 1.0f);
    }

    // update key press
    {
        uint8_t idx= update_key_press_idx;
        uint8_t updateVal= deltaT_ms * (NUM_KEYS / UPDATE_KEY_PRESS_NUM_PER_FRAME) / 2; // fade out in 0.5 sec
        for(uint8_t i=0; i<UPDATE_KEY_PRESS_NUM_PER_FRAME; ++i)
        {
            uint8_t key_val= key_pressed_state[idx];
            if (key_val > 0 && key_val < 255)
            {
                if (key_val < updateVal)
                    key_pressed_state[idx]= 0;
                else
                    key_pressed_state[idx]= key_val - updateVal;
            }
            idx= (idx + 1) % NUM_KEYS;
        }
        update_key_press_idx= idx;
    }

    // tick pulse
    {
        int idx= pulse_active_idx;
        for(int i=0; i<pulse_active_num; ++i)
        {
            float life= pulse_life[idx];
            if (life <= 0.0f)
                break;
            life -= deltaT;
            if (life <= 0.0f)
            {
                --pulse_active_num;
                pulse_active_idx= (pulse_active_idx + 1) % PULSE_NUM_MAX;
                life= 0.0f;
            }
            pulse_life[idx]= life;
            idx= (idx + 1) % PULSE_NUM_MAX;
        }
    }

    // tick ripple
    {
        int idx= ripple_active_idx;
        for(int i=0; i<ripple_active_num; ++i)
        {
            float life= ripple_data[idx].life;
            if (life <= 0.0f)
                break;
            life -= deltaT;
            if (life <= 0.0f)
            {
                --ripple_active_num;
                ripple_active_idx= (ripple_active_idx + 1) % RIPPLE_NUM_MAX;
                life= 0.0f;
            }
            ripple_data[idx].life= life;
            idx= (idx + 1) % RIPPLE_NUM_MAX;
        }
    }

    // tick idle
    {
        if (idle_state == IdleState_WakingUpFromIdle)
        {
            idle_state_timer+= deltaT;
            if (idle_state_timer >= (IDLE_WAKE_UP_SLASH_TIME + IDLE_WAKE_UP_SHOW_VERT_LINE_TIME + IDLE_WAKE_UP_MOVE_VERT_LINE_TIME))
                change_idle_state(IdleState_Awake);
        }
        else if (idle_state == IdleState_WakingUpFromEnteringIdle)
        {
            idle_state_timer-= deltaT;
            if (idle_state_timer <= 0.0f)
                change_idle_state(IdleState_Awake);
        }
        else if (idle_state == IdleState_Awake)
        {
            float prevTimer	= idle_timer;
            float currTimer	= prevTimer + deltaT;
            idle_timer		= currTimer;
            if (currTimer >= IDLE_TIME_THRESHOLD && prevTimer< IDLE_TIME_THRESHOLD)
                change_idle_state(IdleState_EnteringIdle);
        }
        else if (idle_state == IdleState_EnteringIdle)
        {
            idle_state_timer+= deltaT;
            if (idle_state_timer >= IDLE_ENTER_IDLE_TIME)
                change_idle_state(IdleState_Idle);
        }
    }
}

static void render_led(int i)
{
    // render target system values
    const float renderTargetWidth   = 76.0f;
    const float renderTargetHeight  = 24.0f;
    const float NUM_PIXEL_PER_KEY   = 4.0f;
    float       x                   = ((float)g_led_config.point[i].x) * (renderTargetWidth /((float)(RGB_NUM_COL -1))   );
    float       y                   = ((float)g_led_config.point[i].y) * (renderTargetHeight/((float)(RGB_NUM_ROW -1))   );
    float3      rgb                 = {0, 0, 0};
    bool        isContinueRenderPass= true;

    // get timer
    float       timer       = timer_tick * 0.001f;                               // convert to seconds
    float       accum_inv_alpha = 1.0f;

    // idle animation
    {
        if (idle_state == IdleState_Idle)
        {
            led_buffer[i].r = 0;
            led_buffer[i].g = 0;
            led_buffer[i].b = 0;
            return;
        }
        else if (idle_state == IdleState_WakingUpFromIdle)
        {
            if (idle_state_timer < IDLE_WAKE_UP_SLASH_TIME)
            {
                float		t				= idle_state_timer * (1.0f / IDLE_WAKE_UP_SLASH_TIME );
                float		m				= 4.0f;							// line - slope
                float		c				= lerp(70.0f, -350.0f, t);      // line - intercept
                float		a				= m;							// line equation general form
                float		b				=-1.0f;							// line equation general form
                float		disToLineFactor	= 1.0f/sqrtf(a*a+b*b);		    // denominator of distance to line equation
                const float	lineHalfWidth	= 10.5f;
                
				float dis   = fabsf(a*x + b*y + c) * disToLineFactor;
				float alpha	= fminf(dis/lineHalfWidth, 1.0f);
				alpha		*= alpha;
                rgb.x= 0.0f;
                rgb.y= 0.0f;
                rgb.z= 0.0f;
                accum_inv_alpha = 1.0f - alpha;
            }
            else if (idle_state_timer < (IDLE_WAKE_UP_SLASH_TIME + IDLE_WAKE_UP_SHOW_VERT_LINE_TIME))
            {
                led_buffer[i].r = 0;
                led_buffer[i].g = 0;
                led_buffer[i].b = 0;
                return;
            }
            else
            {
	            const float	keyWidth= 4.0f;
                float t	= (idle_state_timer - (IDLE_WAKE_UP_SLASH_TIME + IDLE_WAKE_UP_SHOW_VERT_LINE_TIME))/ IDLE_WAKE_UP_MOVE_VERT_LINE_TIME;
                float rtHalfWidth   = renderTargetWidth * 0.5f;
                float blackLineMaxX = (rtHalfWidth + keyWidth) * t;
                float blackLineMinX = blackLineMaxX - keyWidth;

                float x_center= fabsf(x - rtHalfWidth);
                float alpha   = clamp((x_center - blackLineMinX)/(blackLineMaxX - blackLineMinX), 0.0f, 1.0f);
                rgb.x= 0;
                rgb.y= 0;
                rgb.z= 0;
                accum_inv_alpha = 1.0f - alpha;
            }
        }
        else if (idle_state == IdleState_WakingUpFromEnteringIdle ||
                 idle_state == IdleState_EnteringIdle)
        {
            float	t			= idle_state_timer * (1.0f / IDLE_ENTER_IDLE_TIME);
            
            const float	keyHeight= 4.0f;
            float rtHalfHeight   = renderTargetHeight * 0.5f;
            float blackLineMaxY = (rtHalfHeight + keyHeight) * t;
            float blackLineMinY = blackLineMaxY - keyHeight;

            float y_center= fabsf(y - rtHalfHeight);
            float alpha   = 1.0f - clamp((y_center - blackLineMinY)/(blackLineMaxY - blackLineMinY), 0.0f, 1.0f);
            rgb.x= 0;
            rgb.y= 0;
            rgb.z= 0;
            accum_inv_alpha = 1.0f - alpha;
        }
    }

    // pulse launched from back space
    {
        const float pulseRadius     = 2.0f;
        const float pulseStartPos   = 50.0f;
        const float pulseEndPos     = -pulseRadius;
        int idx= pulse_active_idx;
        for(int n=0; n<pulse_active_num; ++n)
        {
            float life  = pulse_life[idx];
            float t     = life * (1.0f/PULSE_TRAVEL_TIME);
            float pulsePos= lerp(pulseEndPos, pulseStartPos, t);
            if (fabsf(x - pulsePos  ) <= pulseRadius &&
                fabsf(y - 6         ) <= pulseRadius)
            {
                rgb.x   = 1.0f;
                rgb.y   = 1.0f;
                rgb.z   = 1.0f;
                isContinueRenderPass= false;
                break;
            }
            idx= (idx + 1) % PULSE_NUM_MAX;
        }
    }

    // reactive ripple
    {
        const float rippleRadius    = 4.0f; // include fade out regin
        const float rippleTravelDis = 28.0f;
        int idx= ripple_active_idx;
        for(int n=0; n<ripple_active_num; ++n)
        {
            float life      = ripple_data[idx].life;
            float originX   = ripple_data[idx].x;
            float originY   = ripple_data[idx].y;
            float t         = life * (1.0f/RIPPLE_TRAVEL_TIME);
            float travelledDis  = rippleTravelDis * (1.0f - t);
            float disToOrigin   = sqrtf(square(x - originX) + square(y - originY));
            float disToRipple   = fabsf(travelledDis - disToOrigin);
            float alpha         = 1.0f - square(fminf(disToRipple * (1.0f / rippleRadius), 1.0f));
            alpha               *= sqrtf(t);
            if (alpha > 0.0f)
            {
                rgb             = add_f3_s(rgb, alpha * accum_inv_alpha);   // assume using white color
                accum_inv_alpha *= 1.0f - alpha;
            }
            idx= (idx + 1) % RIPPLE_NUM_MAX;
        }
    }

    // modifier blink
    {
        // blink settings
        const float     changeColorTime = 0.4f;
        float           speed           = 1.0f/changeColorTime;

        // calculate blink weight
        float blinkWeight   = fmodf(timer * speed, 2.0f);
        blinkWeight         = blinkWeight > 1.0f ? 2.0f - blinkWeight : blinkWeight;
        blinkWeight         = smoothStep(blinkWeight);

        // ctrl
        if (isContinueRenderPass && render_pass_weight_ctrl > 0.0f)
        {
            float3	col0= {1.0f, 0.075f, 0.075f};
            float3	col1= {0.35f, 0.025f, 0.025f};
            isContinueRenderPass = cal_breath_col(col0, col1, blinkWeight, render_pass_weight_ctrl, &accum_inv_alpha, &rgb);
        }
        
        // shift
        if (isContinueRenderPass && render_pass_weight_shift > 0.0f)
        {
            float3	col0= {0.075f, 0.075f, 1.0f};
            float3	col1= {0.02f, 0.02f, 0.35f};
            isContinueRenderPass = cal_breath_col(col0, col1, blinkWeight, render_pass_weight_shift, &accum_inv_alpha, &rgb);
        }
        
        // alt
        if (isContinueRenderPass && render_pass_weight_alt > 0.0f)
        {
            float3	col0= {1.0f, 0.9f, 0.1f};
            float3	col1= {0.5f, 0.4f, 0.04f};
            isContinueRenderPass = cal_breath_col(col0, col1, blinkWeight, render_pass_weight_alt, &accum_inv_alpha, &rgb);
        }
    }

    // color scroll
    if (isContinueRenderPass)
    {
		const float3    scrollColor[]= { 
			{1.0f , 0.0f , 0.0f }, // red
			{1.0f , 0.5f , 0.0f }, // orange
			{1.0f , 1.0f , 0.0f }, // yellow
			{0.5f , 0.75f, 1.0f }, // cyan 1
			{0.25f, 0.5f , 1.0f }, // cyan 2
			{0.0f , 0.0f , 1.0f }, // blue
			{0.5f , 0.0f , 0.75f}, // purple
			{1.0f , 0.0f , 0.75f}, // pink
		};
        const int       numColor    	    = sizeof(scrollColor)/sizeof(float3);
        const float     distanceBetweenColor= 10.0f;
        const float     changeColorTime     = 1.5f;
        float	idxOffset	= 1.0f/(distanceBetweenColor * NUM_PIXEL_PER_KEY);
        float   speed       = 1.0f/changeColorTime;
        float	idxStart	= numColor - fmodf(timer * speed, (float)numColor);
        {
            float	idxT= fmodf(idxStart + x * idxOffset, (float)numColor);
            int		idx0= (int)idxT;
            int		idx1= (idx0 + 1) % numColor;
            float3	col0= scrollColor[idx0];
            float3	col1= scrollColor[idx1];

            float	t	= idxT - (float)idx0;
            float3	col = lerp_f3(col0, col1, t);
            rgb         = add_f3(rgb, mul_f3_s(col, accum_inv_alpha));
        }
    }

    // reactive key
    if (i < NUM_KEYS)
    {
        if (key_pressed_state[i] > 0)
        {
            float t         = key_pressed_state[i] * (1.0f / 255.0f);
            float3 white    = { 1.0f, 1.0f, 1.0f };
            rgb             = lerp_f3(rgb, white, t);
        }
    }

    // blink lock keys
    uint8_t kbled = host_keyboard_leds();
    uint8_t keyID= i;
    bool isOffNumLock   = (kbled & (1<<USB_LED_NUM_LOCK   ))==0;
    bool isOffCapLock   = (kbled & (1<<USB_LED_CAPS_LOCK  ))==0;
    bool isOffScrollLock= (kbled & (1<<USB_LED_SCROLL_LOCK))==0;
    if (isOffNumLock || !isOffCapLock)
    {
        const float blinkSpeed  = 2.0f;
        float blinkTimer        = fmodf(timer * blinkSpeed, 1.0f);
        float blinkMultiplier   = blinkTimer > 0.5f ? 1.0f : 0.1f;
        if ((keyID == NUM_LOCK_KEY_IDX && isOffNumLock    ) ||
            (keyID == CAPS_LOCK_KEY_IDX && (!isOffCapLock) ) )
            rgb= mul_f3_s(rgb, blinkMultiplier);
    }
  
    // dim dedicated indicator LED if not active
    if (((keyID == NUM_LOCK_LED_KEY_IDX   ) && isOffNumLock     ) ||
        ((keyID == CAPS_LOCK_LED_KEY_IDX  ) && isOffCapLock     ) ||
        ((keyID == SCROLL_LOCK_LED_KEY_IDX) && isOffScrollLock  ) )
    {
        float dimFactor= 0.1f;
        rgb= mul_f3_s(rgb, dimFactor);
    }

    // convert from float range [0, 1] to int range [0, 255] and compute power consumption
    {
        uint8_t ro= (uint8_t)(rgb.x * 255.0f);
        uint8_t go= (uint8_t)(rgb.y * 255.0f);
        uint8_t bo= (uint8_t)(rgb.z * 255.0f);

        led_buffer[i].r = ro;
        led_buffer[i].g = go;
        led_buffer[i].b = bo;
    }
}

void rgb_matrix_user_render(int ledIdx)
{
    render_led(ledIdx);
}

RGB  rgb_matrix_user_getColor(int ledIdx)
{
    return led_buffer[ledIdx];
}

uint8_t  rgb_matrix_user_numLED(void)
{
    return NUM_LED;
}