#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include <stdint.h>

typedef enum {
	key_stateless,
	single_click,
	double_click,
	long_click
}UserKeyState_t;

typedef struct {
    UserKeyState_t (*getKeyState)(uint16_t freq);
}KeyInterface_t,*pKeyInterface_t;

extern KeyInterface_t UserKey;

#endif /* __BSP_KEY_H */
