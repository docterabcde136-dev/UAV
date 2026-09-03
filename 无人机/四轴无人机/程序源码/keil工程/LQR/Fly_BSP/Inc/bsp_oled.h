/**
 * @file    bsp_oled.h
 * @brief   OLED 显示屏驱动接口定义（SSD1306 兼容, 128x64, I2C 接口）
 */

#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

/* OLED 操作接口 */
typedef struct {
	void (*init)(void);                                          /* 硬件初始化 */
	void (*ShowChar)(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode); /* 显示字符 */
	void (*ShowNumber)(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size); /* 显示数字 */
	void (*ShowString)(uint8_t x, uint8_t y, const char *p);    /* 显示字符串 */
	void (*ShowFloat)(uint8_t show_x, uint8_t show_y, const float needtoshow,
	                  uint8_t zs_num, uint8_t xs_num);           /* 显示浮点数 */
	void (*RefreshGram)(void);                                   /* 刷新 GRAM 到屏幕 */
	void (*Clear)(void);                                         /* 清屏 */
} OLEDInterface_t, *pOLEDInterface_t;

extern OLEDInterface_t UserOLED; /* OLED 接口实例 */

#endif
