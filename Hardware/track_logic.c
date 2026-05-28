#include "track_logic.h"
#include "xunji.h"

uint8_t Track_ReadSensorMask(void)
{
    uint8_t mask = 0U;

    /* 通道 1 放到 bit0，通道 8 放到 bit7，后续判断就不用反复读 GPIO。 */
    for (uint8_t channel = 1U; channel <= 8U; channel++) {
        if (digital(channel)) {
            mask |= (uint8_t) (1U << (channel - 1U));
        }
    }

    return mask;
}

uint8_t Track_CountActive(uint8_t mask)
{
    uint8_t count = 0U;

    /* 统计有多少个 bit 为 1，也就是当前有多少路压到黑线。 */
    while (mask != 0U) {
        count += (uint8_t) (mask & 1U);
        mask >>= 1U;
    }

    return count;
}

TrackPattern Track_ClassifyMask(uint8_t mask)
{
    /* 将 8 路探头按左外侧、左中、中心、右中、右外侧分组，方便识别路口形态。 */
    const uint8_t leftOuter = mask & (TRACK_CH1_MASK | TRACK_CH2_MASK);
    const uint8_t leftMid = mask & TRACK_CH3_MASK;
    const uint8_t center = mask & (TRACK_CH4_MASK | TRACK_CH5_MASK);
    const uint8_t rightMid = mask & TRACK_CH6_MASK;
    const uint8_t rightOuter = mask & (TRACK_CH7_MASK | TRACK_CH8_MASK);
    const uint8_t active = Track_CountActive(mask);

    if (mask == 0U) {
        /* 全白/无黑线：车可能已经冲出线外，或正在经过断线区域。 */
        return TRACK_PATTERN_LOST;
    }

    if (active >= 7U) {
        /* 7 路以上同时为黑，单独作为大面积黑线处理，避免和普通十字混淆。 */
        return TRACK_PATTERN_FULL_BLACK;
    }

    if ((active >= 5U) ||
        ((leftOuter != 0U) && (center != 0U) && (rightOuter != 0U))) {
        /* 多路同时触发，或左右外侧和中间同时触发，通常是十字/横线。 */
        return TRACK_PATTERN_CROSS;
    }

    if ((leftOuter != 0U) && (leftMid != 0U) && (center == 0U) &&
        (rightMid == 0U) && (rightOuter == 0U)) {
        /* 左侧探头连续触发，但中心和右侧没有线，说明左边出现锐角入口。 */
        return TRACK_PATTERN_LEFT_SHARP;
    }

    if ((rightOuter != 0U) && (rightMid != 0U) && (center == 0U) &&
        (leftMid == 0U) && (leftOuter == 0U)) {
        /* 右侧探头连续触发，但中心和左侧没有线，说明右边出现锐角入口。 */
        return TRACK_PATTERN_RIGHT_SHARP;
    }

    if ((leftOuter != 0U) && (center != 0U) && (rightOuter == 0U)) {
        /* 左外侧和中心同时有线，右外侧没有线，可作为左环岛入口候选。 */
        return TRACK_PATTERN_LEFT_ROUNDABOUT_ENTRY;
    }

    if ((rightOuter != 0U) && (center != 0U) && (leftOuter == 0U)) {
        /* 右外侧和中心同时有线，左外侧没有线，可作为右环岛入口候选。 */
        return TRACK_PATTERN_RIGHT_ROUNDABOUT_ENTRY;
    }

    /* 剩下的情况都交给普通循迹误差计算处理。 */
    return TRACK_PATTERN_NORMAL;
}

TrackPattern Track_GetPattern(void)
{
    /* 实时版本：读取传感器，再复用纯分类函数。 */
    return Track_ClassifyMask(Track_ReadSensorMask());
}

const char *Track_PatternName(TrackPattern pattern)
{
    /* 串口调试时可直接打印当前识别到的路况名称。 */
    switch (pattern) {
        case TRACK_PATTERN_LOST:
            return "LOST";
        case TRACK_PATTERN_NORMAL:
            return "NORMAL";
        case TRACK_PATTERN_CROSS:
            return "CROSS";
        case TRACK_PATTERN_LEFT_SHARP:
            return "LEFT_SHARP";
        case TRACK_PATTERN_RIGHT_SHARP:
            return "RIGHT_SHARP";
        case TRACK_PATTERN_LEFT_ROUNDABOUT_ENTRY:
            return "LEFT_RING_IN";
        case TRACK_PATTERN_RIGHT_ROUNDABOUT_ENTRY:
            return "RIGHT_RING_IN";
        case TRACK_PATTERN_FULL_BLACK:
            return "FULL_BLACK";
        default:
            return "UNKNOWN";
    }
}
