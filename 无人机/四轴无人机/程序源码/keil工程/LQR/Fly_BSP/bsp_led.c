/**
 * @file    bsp_led.c
 * @brief   板级 LED 驱动实现
 * @note    使用 GPIO 推挽输出控制两个用户 LED（LED1=PB14, LED2=PB15）
 *          低电平点亮（GPIO_PIN_RESET=ON），高电平熄灭（GPIO_PIN_SET=OFF）
 */

#include "bsp_led.h"
#include "gpio.h"

/* LED1 (PB14) 控制函数 -------------------------------------------------------*/

/**
 * @brief  点亮 LED1（输出低电平）
 */
static void Led1On(void)
{
    HAL_GPIO_WritePin(User_LED1_GPIO_Port, User_LED1_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  熄灭 LED1（输出高电平）
 */
static void Led1Off(void)
{
    HAL_GPIO_WritePin(User_LED1_GPIO_Port, User_LED1_Pin, GPIO_PIN_SET);
}

/**
 * @brief  翻转 LED1 电平（亮灭切换）
 */
static void Led1Toggle(void)
{
    HAL_GPIO_TogglePin(User_LED1_GPIO_Port, User_LED1_Pin);
}

/* LED2 (PB15) 控制函数 -------------------------------------------------------*/

/**
 * @brief  点亮 LED2（输出低电平）
 */
static void Led2On(void)
{
    HAL_GPIO_WritePin(User_LED2_GPIO_Port, User_LED2_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  熄灭 LED2（输出高电平）
 */
static void Led2Off(void)
{
    HAL_GPIO_WritePin(User_LED2_GPIO_Port, User_LED2_Pin, GPIO_PIN_SET);
}

/**
 * @brief  翻转 LED2 电平（亮灭切换）
 */
static void Led2Toggle(void)
{
    HAL_GPIO_TogglePin(User_LED2_GPIO_Port, User_LED2_Pin);
}

/* LED 接口结构体实例 ----------------------------------------------------------*/
/* 通过函数指针实现多态，上层 APP 无需关心底层 GPIO 操作细节 */

LedInterface_t UserLed1 = {
    .on     = Led1On,      /* 点亮 LED1 */
    .off    = Led1Off,     /* 熄灭 LED1 */
    .toggle = Led1Toggle   /* 翻转 LED1 状态 */
};

LedInterface_t UserLed2 = {
    .on     = Led2On,      /* 点亮 LED2 */
    .off    = Led2Off,     /* 熄灭 LED2 */
    .toggle = Led2Toggle   /* 翻转 LED2 状态 */
};
