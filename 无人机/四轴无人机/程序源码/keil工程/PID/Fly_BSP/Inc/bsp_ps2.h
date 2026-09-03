/**
 * ============================================================
 * bsp_ps2.h — PS2无线手柄USB-HID驱动头文件
 * ============================================================
 * 支持的设备:
 *   有线模式:   VID=0x0810 PID=0x0001
 *   无线PC模式:  VID=0x0079 PID=0x0126
 *   无线安卓模式: VID=0x045E PID=0x028E
 *
 * 数据接口: PS2INFO_t
 *   LX/LY/RX/RY: 4个模拟摇杆 (0~255, 中位128)
 *   getKeyEvent(): 按键事件 (单击/双击/长按)
 *   getKeyState(): 按键状态 (按下/松开)
 *
 * 按键枚举: SELECT, START, L1~R2, 十字键, 4颜色键, 摇杆按下
 * ============================================================
 */

#ifndef __BSP_PS2_H
#define __BSP_PS2_H

#include <stdint.h>

//有线ps2手柄设备标识符
#define Wired_PS2_VID 0x0810
#define Wired_PS2_PID 0x0001

//无线ps2手柄安卓模式设备标识符
#define Wireless_Android_PS2_VID 0x045E
#define Wireless_Android_PS2_PID 0x028E

//无线ps2手柄PC模式设备标识符
#define Wireless_PC_PS2_VID 0x0079
#define Wireless_PC_PS2_PID 0x0126

//PS2手柄类型定义
typedef enum{
	UnKnown_Dev,          //未知设备
	Wired_PS2,            //有线ps2手柄
	Wiredless_PC_PS2,     //无线ps2手柄pc模式
	Wiredless_Android_PS2,//无线ps2手柄安卓模式
}PS2_TYPE_t;

//表示按键的状态值
#define PS2KEY_PressDOWN 1  //按键按下
#define PS2KEY_PressUP   0  //按键松开

//PS2按键状态(无状态、单击、双击、长按)
typedef enum{
    PS2KEYSTATE_NONE,
    PS2KEYSTATE_SINGLECLICK,
    PS2KEYSTATE_DOUBLECLICK,
    PS2KEYSTATE_LONGCLICK
}PS2KEY_State_t;

//PS2手柄对象
typedef struct{
	uint8_t LX;      //4个方向摇杆值,取值0~255
	uint8_t LY;
	uint8_t RX;
	uint8_t RY;
	PS2KEY_State_t (*getKeyEvent)(uint8_t keybit); //获取按键事件,有单击、双击、长按3种事件可以获取,入口参数为按键的枚举键值
	uint8_t (*getKeyState)(uint8_t keybit);        //获取按键的状态,0表示按下,1表示松开,入口参数为按键的枚举键值
}PS2INFO_t;

//PS2按键位置枚举(bit0~bit15分别为下面的0~15)
enum 
{
	PS2KEY_SELECT	   = 0, //选择按键
	PS2KEY_LROCKER      , //左右摇杆按下键值
	PS2KEY_RROCKER      ,
	PS2KEY_START        , //开始按键
	PS2KEY_UP           , //左按键区域
	PS2KEY_RIGHT        ,
	PS2KEY_DOWN         ,
	PS2KEY_LEFT         ,
	PS2KEY_L2           ,	//左右扳机按键值
	PS2KEY_R2           ,
	PS2KEY_L1           ,  
	PS2KEY_R1           ,
	PS2KEY_1GREEN       , //右按键区域
	PS2KEY_2RED         , 
	PS2KEY_3BLUE        , 
	PS2KEY_4PINK           
};

#endif /* __BSP_PS2_H */
