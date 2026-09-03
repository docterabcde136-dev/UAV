/**
 * @file    bsp_buzzer.c
 * @brief   板级蜂鸣器驱动实现
 * @note    使用 GPIO 推挽输出控制有源蜂鸣器（PA4）
 *          高电平鸣响（GPIO_PIN_SET=ON），低电平关闭（GPIO_PIN_RESET=OFF）
 */

#include "bsp_buzzer.h"
#include "gpio.h"

/**
 * @brief  蜂鸣器开启（输出高电平）
 */
static void BuzzerOn(void)
{
    HAL_GPIO_WritePin(User_BUZZER_GPIO_Port, User_BUZZER_Pin, GPIO_PIN_SET);
}

/**
 * @brief  蜂鸣器关闭（输出低电平）
 */
static void BuzzerOff(void)
{
    HAL_GPIO_WritePin(User_BUZZER_GPIO_Port, User_BUZZER_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  蜂鸣器状态翻转
 */
static void Buzzeroggle(void)
{
    HAL_GPIO_TogglePin(User_BUZZER_GPIO_Port, User_BUZZER_Pin);
}

/* 蜂鸣器接口结构体实例 -------------------------------------------------------*/
/* 通过函数指针向上层 APP 暴露统一的 on/off/toggle 操作接口 */

BuzzerInterface_t UserBuzzer = {
    .on     = BuzzerOn,     /* 开启蜂鸣器 */
    .off    = BuzzerOff,    /* 关闭蜂鸣器 */
    .toggle = Buzzeroggle   /* 翻转蜂鸣器状态 */
};
