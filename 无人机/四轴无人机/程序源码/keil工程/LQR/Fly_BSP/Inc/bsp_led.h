/**
 * @file    bsp_led.h
 * @brief   板级 LED 驱动接口定义
 * @note    接口使用示例：
 *          pLedInterface_t led1 = &UserLed1;
 *          led1->on();  // 点亮 LED1
 */

#ifndef __BSP_LED_H
#define __BSP_LED_H

typedef struct {
	void (*on)(void);        /* 点亮 LED */
	void (*off)(void);       /* 熄灭 LED */
	void (*toggle)(void);    /* 翻转 LED 状态 */
} LedInterface_t, *pLedInterface_t;

extern LedInterface_t UserLed1;  /* 用户 LED1 (PB14) */
extern LedInterface_t UserLed2;  /* 用户 LED2 (PB15) */

#endif /* __BSP_LED_H */
