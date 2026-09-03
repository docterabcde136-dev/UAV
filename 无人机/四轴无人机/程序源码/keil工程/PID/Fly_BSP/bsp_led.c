/**
 * ============================================================
 * bsp_led.c — LED驱动 (两路用户指示灯)
 * ============================================================
 * 硬件: GPIO输出, 低电平点亮 (RESET=ON, SET=OFF)
 * 接口: LedInterface_t { .on, .off, .toggle }
 * 实例: UserLed1, UserLed2
 * ============================================================
 */

#include "bsp_led.h"
#include "gpio.h"


static void Led1On(void)
{
    HAL_GPIO_WritePin(User_LED1_GPIO_Port,User_LED1_Pin,GPIO_PIN_RESET);
}

static void Led1Off(void)
{
    HAL_GPIO_WritePin(User_LED1_GPIO_Port,User_LED1_Pin,GPIO_PIN_SET);
}

static void Led1Toggle(void)
{
    HAL_GPIO_TogglePin(User_LED1_GPIO_Port,User_LED1_Pin);
}


static void Led2On(void)
{
    HAL_GPIO_WritePin(User_LED2_GPIO_Port,User_LED2_Pin,GPIO_PIN_RESET);
}

static void Led2Off(void)
{
    HAL_GPIO_WritePin(User_LED2_GPIO_Port,User_LED2_Pin,GPIO_PIN_SET);
}

static void Led2Toggle(void)
{
    HAL_GPIO_TogglePin(User_LED2_GPIO_Port,User_LED2_Pin);
}


//驱动挂载
LedInterface_t UserLed1 = {
    .on = Led1On,
    .off = Led1Off,
    .toggle = Led1Toggle
};

LedInterface_t UserLed2 = {
    .on = Led2On,
    .off = Led2Off,
    .toggle = Led2Toggle
};


