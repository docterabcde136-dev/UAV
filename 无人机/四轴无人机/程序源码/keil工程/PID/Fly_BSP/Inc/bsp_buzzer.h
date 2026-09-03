#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

/*
* 使用说明：
* 已对外提供了1个接口：UserBuzzer
* 使用示例：想让蜂鸣器鸣响
*    pBuzzerInterface_t buzzer = &UserBuzzer;
*    buzzer -> on();  //开启蜂鸣器
*/

typedef struct {
    void (*on)(void);
    void (*off)(void);
    void (*toggle)(void);
}BuzzerInterface_t,*pBuzzerInterface_t;

extern BuzzerInterface_t UserBuzzer;

#endif /* __BSP_BUZZER_H */

