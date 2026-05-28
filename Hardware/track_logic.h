#ifndef __TRACK_LOGIC_H__
#define __TRACK_LOGIC_H__

#include <stdint.h>

/* 8 路循迹探头组合后识别出来的路况类型。 */
typedef enum {
    TRACK_PATTERN_LOST = 0,              /* 8 路都没有检测到黑线，认为丢线。 */
    TRACK_PATTERN_NORMAL,                /* 普通循迹状态，交给 Xunji_Speed() 处理。 */
    TRACK_PATTERN_CROSS,                 /* 十字路口或大面积横线。 */
    TRACK_PATTERN_LEFT_SHARP,            /* 左侧锐角/急左转入口。 */
    TRACK_PATTERN_RIGHT_SHARP,           /* 右侧锐角/急右转入口。 */
    TRACK_PATTERN_LEFT_ROUNDABOUT_ENTRY, /* 左侧环岛入口特征。 */
    TRACK_PATTERN_RIGHT_ROUNDABOUT_ENTRY,/* 右侧环岛入口特征。 */
    TRACK_PATTERN_FULL_BLACK             /* 接近全黑，通常是停车线/大面积标志线。 */
} TrackPattern;

/* bit0~bit7 分别对应循迹通道 1~8，digital() 返回 1 表示检测到黑线。 */
#define TRACK_CH1_MASK (1U << 0)
#define TRACK_CH2_MASK (1U << 1)
#define TRACK_CH3_MASK (1U << 2)
#define TRACK_CH4_MASK (1U << 3)
#define TRACK_CH5_MASK (1U << 4)
#define TRACK_CH6_MASK (1U << 5)
#define TRACK_CH7_MASK (1U << 6)
#define TRACK_CH8_MASK (1U << 7)

/* 读取当前 8 路循迹状态，并打包成 bitmask。 */
uint8_t Track_ReadSensorMask(void);

/* 统计 bitmask 中有几路探头检测到黑线。 */
uint8_t Track_CountActive(uint8_t mask);

/* 纯逻辑分类函数：输入 8 路 bitmask，输出路况类型，便于后续做离线测试。 */
TrackPattern Track_ClassifyMask(uint8_t mask);

/* 实时读取传感器并返回当前路况类型。 */
TrackPattern Track_GetPattern(void);

/* 调试用：把路况枚举转成字符串，可用于串口打印。 */
const char *Track_PatternName(TrackPattern pattern);

/* mode 6 的增强循迹状态机：普通循迹 + 十字 + 锐角 + 环岛入口处理。 */
void Track_Mode6Enhanced(void);

#endif
