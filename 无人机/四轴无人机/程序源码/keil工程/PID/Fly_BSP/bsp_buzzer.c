/**
 * ============================================================
 * bsp_buzzer.c — 蜂鸣器驱动
 * ============================================================
 * 硬件: GPIO输出, 高电平鸣响 (SET=ON, RESET=OFF)
 * 接口: BuzzerInterface_t { .on, .off, .toggle }
 * 实例: UserBuzzer
 * ============================================================
 */

#include "bsp_buzzer.h"
#include "gpio.h"

static void BuzzerOn(void)
{
    HAL_GPIO_WritePin(User_BUZZER_GPIO_Port,User_BUZZER_Pin,GPIO_PIN_SET);
}

static void BuzzerOff(void)
{
    HAL_GPIO_WritePin(User_BUZZER_GPIO_Port,User_BUZZER_Pin,GPIO_PIN_RESET);
}

static void BuzzerToggle(void)
{
    HAL_GPIO_TogglePin(User_BUZZER_GPIO_Port,User_BUZZER_Pin);
}

BuzzerInterface_t UserBuzzer = {
    .on = BuzzerOn,
    .off = BuzzerOff,
    .toggle = BuzzerToggle
};

