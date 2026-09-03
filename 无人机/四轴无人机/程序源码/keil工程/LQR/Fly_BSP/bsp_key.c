/**
 * @file    bsp_key.c
 * @brief   板级按键驱动实现
 * @note    使用 GPIO 输入模式检测用户按键（PB13）
 *          实现单击、双击、长按三种按键状态识别
 *          按键按下为低电平 (keyValue()==0)，弹起为高电平 (keyValue()==1)
 */

#include "bsp_key.h"
#include "gpio.h"

/**
 * @brief  读取按键当前电平
 * @retval 0=按下（低电平），1=弹起（高电平）
 */
static uint8_t keyValue(void)
{
    return HAL_GPIO_ReadPin(User_KEY_GPIO_Port, User_KEY_Pin);
}

/**
 * @brief  按键状态扫描（状态机识别单击/双击/长按）
 * @param  freq: 调用频率（Hz），用于将计数值转换为毫秒时间基准
 * @retval UserKeyState_t 按键事件类型：
 *         - key_stateless: 无按键事件
 *         - single_click:  单击（按下后 800ms 内未再次按下）
 *         - double_click:  双击（50~300ms 内连续两次按下）
 *         - long_click:    长按（持续按下超过 500ms）
 * @note   该函数需以固定频率（freq Hz）周期调用以保证时间判断准确
 */
static UserKeyState_t key_scan(uint16_t freq)
{
    static uint16_t time_core;        /* 走时核心：记录按键弹起后的时间计数 */
    static uint16_t long_press_time;  /* 长按计时：记录按键持续按下的时间计数 */
    static uint8_t  press_flag = 0;   /* 按键按下标记：0=未检测到按下，1=已被按下过一次 */
    static uint8_t  check_once = 0;   /* 识别完成标记：1=已完成一次事件识别，需等待弹起后重启 */

    /* 将调用频率转换为每次计数所对应的毫秒数 */
    float Count_time = (((float)(1.0f / (float)freq)) * 1000.0f);

    /* 状态复位：上一次事件识别完成后，重置所有状态变量 */
    if (check_once)
    {
        press_flag      = 0;          /* 清除按下标记 */
        time_core       = 0;          /* 清除弹起计时 */
        long_press_time = 0;          /* 清除长按计时 */
    }
    /* 检测按键弹起后允许进入下一轮扫描 */
    if (check_once && 1 == keyValue()) check_once = 0;

    /* 按键按下检测：低电平有效 */
    if (0 == keyValue() && check_once == 0)
    {
        press_flag = 1;               /* 标记按键已被按下 */
        long_press_time++;            /* 累计长按计时 */
    }

    /* 长按判定：持续按下超过 500ms */
    if (long_press_time > (uint16_t)(500.0f / Count_time))
    {
        check_once = 1;               /* 标记事件已识别 */
        return long_click;            /* 返回长按事件 */
    }

    /* 按键弹起后开始计时（用于判断单击/双击间隔） */
    if (press_flag && 1 == keyValue())
    {
        time_core++;
    }

    /* 双击判定：50ms~300ms 内检测到第二次按下 */
    if (press_flag && (time_core > (uint16_t)(50.0f / Count_time)
                    && time_core < (uint16_t)(300.0f / Count_time)))
    {
        if (0 == keyValue())          /* 在时间窗口内再次按下 */
        {
            check_once = 1;           /* 标记事件已识别 */
            return double_click;      /* 返回双击事件 */
        }
    }
    /* 单击判定：弹起超过 300ms 仍未再次按下 */
    else if (press_flag && time_core > (uint16_t)(300.0f / Count_time))
    {
        check_once = 1;               /* 标记事件已识别 */
        return single_click;          /* 返回单击事件 */
    }

    return key_stateless;             /* 无按键事件 */
}

/* 按键接口结构体实例 ---------------------------------------------------------*/
/* 通过函数指针向上层 APP 暴露按键状态获取接口 */

KeyInterface_t UserKey = {
    .getKeyState = key_scan,          /* 按键扫描状态机函数 */
};
