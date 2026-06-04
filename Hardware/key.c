#include "key.h"

// 声明外部变量，与 empty.c 或其他文件通信
extern volatile u8 car_started;
extern volatile u8 task_mode;       // 题目的档位，1-5档

typedef enum
{
    KEY_STATE_IDLE = 0,
    KEY_STATE_DEBOUNCE_PRESS,
    KEY_STATE_PRESSED,
    KEY_STATE_DEBOUNCE_RELEASE
} KeyState;

typedef struct
{
    KeyState state;
    uint8_t debounce_cnt;
} KeyFsm;

#define KEY_DEBOUNCE_TICKS   (3U)

static uint8_t key_is_pressed(uint32_t pin_level, uint32_t pressed_level)
{
    if (pressed_level == 0U)
    {
        return (pin_level == 0U) ? 1U : 0U;
    }

    // DL_GPIO_readPins 返回的是位掩码，不是 1/0
    return (pin_level != 0U) ? 1U : 0U;
}

static void key_fsm_update(KeyFsm *fsm, uint32_t pin_level, uint32_t pressed_level, void (*on_pressed)(void))
{
    const uint8_t is_pressed = key_is_pressed(pin_level, pressed_level);

    switch (fsm->state)
    {
        case KEY_STATE_IDLE:
            if (is_pressed)
            {
                fsm->state = KEY_STATE_DEBOUNCE_PRESS;
                fsm->debounce_cnt = 0U;
            }
            break;

        case KEY_STATE_DEBOUNCE_PRESS:
            if (is_pressed)
            {
                if (++fsm->debounce_cnt >= KEY_DEBOUNCE_TICKS)
                {
                    fsm->state = KEY_STATE_PRESSED;
                    on_pressed();
                }
            }
            else
            {
                fsm->state = KEY_STATE_IDLE;
            }
            break;

        case KEY_STATE_PRESSED:
            if (!is_pressed)
            {
                fsm->state = KEY_STATE_DEBOUNCE_RELEASE;
                fsm->debounce_cnt = 0U;
            }
            break;

        case KEY_STATE_DEBOUNCE_RELEASE:
            if (!is_pressed)
            {
                if (++fsm->debounce_cnt >= KEY_DEBOUNCE_TICKS)
                {
                    fsm->state = KEY_STATE_IDLE;
                }
            }
            else
            {
                fsm->state = KEY_STATE_PRESSED;
            }
            break;

        default:
            fsm->state = KEY_STATE_IDLE;
            fsm->debounce_cnt = 0U;
            break;
    }
}

static void key_a_on_pressed(void)
{
    // 只有未发车时才允许选单
    if (car_started == 0U)
    {
        task_mode++;
        if (task_mode > 7U)
        {
            task_mode = 1U;
        }
    }
}

static void key_b_on_pressed(void)
{
    if (car_started == 0U)
    {
        car_started = 1U;
    }
    else
    {
        // 运行中按下则急停
        car_started = 0U;
        Set_PWM_L(0);
        Set_PWM_R(0);
    }
}

// 非阻塞按键扫描：建议在 while(1) 或固定周期中断里周期调用
void Key(void)
{
    static KeyFsm key_a_fsm = { KEY_STATE_IDLE, 0U };
    static uint8_t key_init_done = 0U;
    static uint32_t b_idle_level = 0U;
    static uint8_t b_press_latched = 0U;


    // A键：上拉输入，按下为低电平
    const uint32_t btnA = DL_GPIO_readPins(KEY_key_PORT, KEY_key_PIN); // 题目翻页
    // B键：电平极性由上电空闲值自适应，兼容高/低电平按下
    const uint32_t btnB = DL_GPIO_readPins(KEY_key_2_PORT, KEY_key_2_PIN); // 发车/急停

    if (key_init_done == 0U)
    {
        // 上电首次扫描先等待按键释放，防止上电瞬间误触发
        if (key_is_pressed(btnA, 0U) != 0U)
        {
            key_a_fsm.state = KEY_STATE_PRESSED;
            key_a_fsm.debounce_cnt = 0U;
        }

        b_idle_level = btnB;
        b_press_latched = 0U;

        key_init_done = 1U;
    }

    key_fsm_update(&key_a_fsm, btnA, 0U, key_a_on_pressed);

    // B键仅在“从空闲进入按下”时触发一次，松手后解除锁存，防止抖动导致来回切换
    if (btnB != b_idle_level)
    {
        if (b_press_latched == 0U)
        {
            b_press_latched = 1U;
            key_b_on_pressed();
        }
    }
    else
    {
        b_press_latched = 0U;
    }
}


