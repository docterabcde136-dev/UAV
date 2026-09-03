#ifndef __OLED_H
#define __OLED_H	

#include <stdint.h>

typedef struct {
	void (*init)(void);
    void (*ShowChar)(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode);
    void (*ShowNumber)(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
    void (*ShowString)(uint8_t x,uint8_t y,const char *p);
	void (*ShowFloat)(uint8_t show_x,uint8_t show_y,const float needtoshow,uint8_t zs_num,uint8_t xs_num);
	void (*RefreshGram)(void);
	void (*Clear)(void);
}OLEDInterface_t,*pOLEDInterface_t;

extern OLEDInterface_t UserOLED;

#endif  
	 
