#include "motor.h"

void Set_PWM_L(int pwmA)
{
    if (pwmA == 0)
    {
        // Left motor: PWM C1, direction AIN1/AIN2.
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
        return;
    }

    if (pwmA > 0)
    {
        DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, ABS(pwmA), GPIO_PWM_0_C1_IDX);
    }
    else
    {
        DL_GPIO_setPins(AIN_PORT, AIN_AIN2_PIN);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, ABS(pwmA), GPIO_PWM_0_C1_IDX);
    }
}

void Set_PWM_R(int pwmB)
{
    if (pwmB == 0)
    {
        // Right motor: PWM C0, direction BIN1/BIN2.
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
        return;
    }

    if (pwmB > 0)
    {
        DL_GPIO_setPins(BIN_PORT, BIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, ABS(pwmB), GPIO_PWM_0_C0_IDX);
    }
    else
    {
        DL_GPIO_setPins(BIN_PORT, BIN_BIN2_PIN);
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, ABS(pwmB), GPIO_PWM_0_C0_IDX);
    }
}
