/**
 * @file    bsp_buzzer.h
 * @brief   板级蜂鸣器驱动接口定义
 */

#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

typedef struct {
	void (*on)(void);        /* 蜂鸣器开启 */
	void (*off)(void);       /* 蜂鸣器关闭 */
	void (*toggle)(void);    /* 蜂鸣器状态翻转 */
} BuzzerInterface_t, *pBuzzeInterface_t;

extern BuzzerInterface_t UserBuzzer; /* 用户蜂鸣器 (PA4) */

#endif /* __BSP_BUZZER_H */
