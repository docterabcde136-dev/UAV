/**************************************************************************
作者：平衡小车之家 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/

#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H

#include <stdint.h>

typedef struct{
	void (*show)(float* showdata,uint8_t len);
}DataScopeInterface_t,*pDataScopeInterface_t;

extern DataScopeInterface_t UserDataScope;

#endif 

