/**
 * @file    bsp_key.h
 * @brief   板级按键驱动接口定义（状态机识别单击/双击/长按）
 */

#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include <stdint.h>

/* 按键状态枚举 */
typedef enum {
	key_stateless,           /* 无按键事件 */
	single_click,            /* 单击 */
	double_click,            /* 双击 */
	long_click               /* 长按 */
} UserKeyState_t;

/* 按键接口结构体 */
typedef struct {
	UserKeyState_t (*getKeyState)(uint16_t freq); /* 获取按键状态（需传入调用频率 Hz） */
} KeyInterface_t, *pKeyInterface_t;

extern KeyInterface_t UserKey; /* 用户按键 (PB13) */

#endif /* __BSP_KEY_H */
