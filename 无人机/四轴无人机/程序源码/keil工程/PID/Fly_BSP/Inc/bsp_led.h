#ifndef __BSP_LED_H
#define __BSP_LED_H

/*
* 使用说明：
* 已对外提供了3个接口：UserLed1、UserLed2、UserBuzzer
* 使用示例：想开启LED1灯
*    pLedInterface_t led1 = &UserLed1;
*    led1 -> on();  //开启LED1
*/

typedef struct {
    void (*on)(void);
    void (*off)(void);
    void (*toggle)(void);
}LedInterface_t,*pLedInterface_t;

extern LedInterface_t UserLed1,UserLed2;

#endif /* __BSP_LED_H */

