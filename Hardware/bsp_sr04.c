#include "board.h"

/* 最近一次测距结果，单位 cm；在 ECHO 下降沿中断中更新。 */
volatile float distance = 0.0f;
/* 测距完成标志：0 表示等待 ECHO 下降沿，1 表示本次测距完成。 */
volatile uint8_t SR04_Flag = 0U;

#define SR04_NB_PERIOD_CALLS 3U
#define SR04_NB_TIMEOUT_CALLS 3U

static volatile uint8_t sr04_nb_busy = 0U;
static volatile uint8_t sr04_nb_new_data = 0U;
static volatile float sr04_nb_last_distance = 0.0f;
static uint8_t sr04_nb_period_cnt = 0U;
static uint8_t sr04_nb_timeout_cnt = 0U;

/**
 * @brief 初始化 HC-SR04 超声波模块。
 *
 * TIMER_SR04 在 SysConfig 中配置为 1 MHz 计数频率，1 个计数约等于 1 us。
 * ECHO 使用 GPIOA 共享中断，与编码器同属 GROUP1，实际入口在 encoder.c 的
 * GROUP1_IRQHandler() 中调用 SR04_HandleEchoInterrupt()。
 */
void SR04_Init(void)
{
    /* 空闲时 TRIG 保持低电平。 */
    SR04_TRIG(0);

    /* 测距前先停止并清零专用计时器，避免残留计数影响第一次测量。 */
    DL_TimerA_stopCounter(TIMER_SR04_INST);
    DL_TimerA_setTimerCount(TIMER_SR04_INST, TIMER_SR04_INST_LOAD_VALUE);

    /* ECHO 为 PA7，需要同时捕获上升沿和下降沿：上升沿开始计时，下降沿结束计时。 */
    DL_GPIO_setLowerPinsPolarity(SR04_PORT, DL_GPIO_PIN_7_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(SR04_PORT, SR04_Echo_PIN);

    /* 使能 SR04 所在 GPIOA 中断。 */
    NVIC_ClearPendingIRQ(SR04_INT_IRQN);
    NVIC_EnableIRQ(SR04_INT_IRQN);
}

/**
 * @brief 开始记录 ECHO 高电平宽度。
 */
void Open_Timer(void)
{
    DL_TimerA_setTimerCount(TIMER_SR04_INST, TIMER_SR04_INST_LOAD_VALUE);
    DL_TimerA_startCounter(TIMER_SR04_INST);
}

/**
 * @brief 读取当前 ECHO 高电平持续时间。
 *
 * @return TIMER_SR04 当前计数值，单位约为 us。
 */
uint32_t Get_TIMER_Count(void)
{
    uint32_t count = DL_TimerA_getTimerCount(TIMER_SR04_INST);

    if (count > TIMER_SR04_INST_LOAD_VALUE) {
        return 0U;
    }

    return TIMER_SR04_INST_LOAD_VALUE - count;
}

/**
 * @brief 停止超声波计时器。
 */
void Close_Timer(void)
{
    DL_TimerA_stopCounter(TIMER_SR04_INST);
}

static void SR04_StartMeasureNonBlocking(void)
{
    SR04_Flag = 0U;
    sr04_nb_busy = 1U;
    sr04_nb_timeout_cnt = 0U;
    DL_TimerA_setTimerCount(TIMER_SR04_INST, TIMER_SR04_INST_LOAD_VALUE);
    DL_GPIO_clearInterruptStatus(SR04_PORT, SR04_Echo_PIN);
    NVIC_ClearPendingIRQ(SR04_INT_IRQN);
    NVIC_EnableIRQ(SR04_INT_IRQN);

    SR04_TRIG(0);
    delay_1us(2);
    SR04_TRIG(1);
    delay_1us(15);
    SR04_TRIG(0);
}

/**
 * @brief 处理 ECHO 引脚边沿中断。
 *
 * 该函数不直接作为中断函数注册，而是在 encoder.c 的 GROUP1_IRQHandler()
 * 中被调用，用于避免多个文件重复定义 GROUP1_IRQHandler()。
 */
void SR04_HandleEchoInterrupt(void)
{
    uint32_t sr04_int = DL_GPIO_getEnabledInterruptStatus(SR04_PORT, SR04_Echo_PIN);

    /* 本次 GROUP1 中断不是 SR04_ECHO 触发，直接返回。 */
    if ((sr04_int & SR04_Echo_PIN) == 0U) {
        return;
    }

    if (SR04_ECHO()) {
        /* 上升沿：ECHO 变高，开始计时。 */
        SR04_Flag = 0U;
        sr04_nb_busy = 1U;
        distance = 0.0f;
        Open_Timer();
    } else {
        /* 下降沿：ECHO 变低，停止计时并换算距离。 */
        Close_Timer();
        SR04_Flag = 1U;
        /* HC-SR04 经验公式：距离(cm) = 高电平时间(us) / 58。 */
        distance = (float) Get_TIMER_Count() / 58.0f;
        sr04_nb_last_distance = distance;
        sr04_nb_new_data = 1U;
        sr04_nb_busy = 0U;
        sr04_nb_timeout_cnt = 0U;
    }

    DL_GPIO_clearInterruptStatus(SR04_PORT, SR04_Echo_PIN);
}

float SR04_GetLengthNonBlocking(void)
{
    float result = 0.0f;

    if (sr04_nb_new_data != 0U) {
        result = sr04_nb_last_distance;
        sr04_nb_new_data = 0U;
    }

    if (sr04_nb_busy != 0U) {
        sr04_nb_timeout_cnt++;
        if (sr04_nb_timeout_cnt >= SR04_NB_TIMEOUT_CALLS) {
            Close_Timer();
            SR04_Flag = 0U;
            sr04_nb_busy = 0U;
            sr04_nb_timeout_cnt = 0U;
            sr04_nb_period_cnt = 0U;
        }
        return result;
    }

    sr04_nb_period_cnt++;
    if (sr04_nb_period_cnt >= SR04_NB_PERIOD_CALLS) {
        sr04_nb_period_cnt = 0U;
        SR04_StartMeasureNonBlocking();
    }

    return result;
}

/**
 * @brief 获取一次滤波后的超声波距离。
 *
 * 连续测量 5 次，保留有效值；有效值不少于 3 个时，排序后去掉最大值和最小值，
 * 返回剩余值的平均距离。若有效测量不足 3 次，返回 0。
 *
 * @return 距离，单位 cm。
 */
float SR04_GetLength(void)
{
    /* 存放最多 5 次有效测距结果。 */
    float distances[5] = {0.0f};
    uint8_t valid_count = 0U;

    for (uint8_t i = 0U; i < 5U; i++) {
        /* 600 * 50 us = 30 ms，超过常见 HC-SR04 回波等待时间则认为超时。 */
        uint32_t timeout = 600U;

        /* 开始一次新测量前清状态、清计数、清中断标志。 */
        SR04_Flag = 0U;
        DL_TimerA_setTimerCount(TIMER_SR04_INST, TIMER_SR04_INST_LOAD_VALUE);
        DL_GPIO_clearInterruptStatus(SR04_PORT, SR04_Echo_PIN);
        NVIC_ClearPendingIRQ(SR04_INT_IRQN);
        NVIC_EnableIRQ(SR04_INT_IRQN);

        /* TRIG 输出不少于 10 us 的高电平触发测距。 */
        SR04_TRIG(0);
        delay_1us(2);
        SR04_TRIG(1);
        delay_1us(15);
        SR04_TRIG(0);

        /* 等待 ECHO 下降沿中断置位 SR04_Flag，或等待超时。 */
        while ((SR04_Flag == 0U) && (timeout > 0U)) {
            delay_1us(50);
            timeout--;
        }

        /* 超时说明没有收到完整回波，本次结果丢弃。 */
        if (timeout == 0U) {
            Close_Timer();
            continue;
        }

        /* 保存有效结果，两次测量之间留出短间隔，避免回波串扰。 */
        distances[valid_count++] = distance;
        delay_ms(10);
    }

    /* 有效值太少时不做去极值平均。 */
    if (valid_count < 3U) {
        return 0.0f;
    }

    /* 简单冒泡排序，方便去掉最大值和最小值。 */
    for (uint8_t i = 0U; i < valid_count - 1U; i++) {
        for (uint8_t j = i + 1U; j < valid_count; j++) {
            if (distances[i] > distances[j]) {
                float temp = distances[i];
                distances[i] = distances[j];
                distances[j] = temp;
            }
        }
    }

    /* 去掉排序后的首尾值，对中间值求平均。 */
    float sum = 0.0f;
    for (uint8_t i = 1U; i < valid_count - 1U; i++) {
        sum += distances[i];
    }

    return sum / (float) (valid_count - 2U);
}
