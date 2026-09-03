/**
 * ============================================================
 * ps2_task.c — PS2无线手柄控制任务
 * ============================================================
 * 功能:
 *   将 PS2 手柄输入映射为四轴飞行器控制指令
 *
 * 按键映射 (核心):
 *   START 长按 = 启动飞行     START 单击/双击 = 停止飞行
 *   SELECT 长按 = 切换不定高模式
 *   左摇杆 = 前后左右平移     右摇杆(Y) = 高度  (X) = 旋转
 *   L1 长按 = 无头模式切换    R2 长按 = 系统复位
 *   十字键 = 微调零点
 *
 * 特色:
 *   - 支持左右手油门切换 (L2+左摇杆按下=左手, L2+右摇杆按下=右手)
 *   - 飞行中: 左摇杆+颜色键控制避障/跟随/步进
 *   - 地面时: 颜色键进入电机测试模式 (单击=测试, 双击=换向, 长按=保存)
 * ============================================================
 */

#include "bsp_ps2.h"
#include "usbh_hid.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"

#include "balance_task.h"

//数据处理回调函数声明
extern USBH_StatusTypeDef USBH_HID_PS2_Decode(USBH_HandleTypeDef *phost);
extern void WriteFlyControlQueue(FlyControlType_t val,uint8_t writeEnv,BaseType_t* HigherPriorityTask);

extern void StopFlyAction(void);
extern void StartFlyAction(void);
extern void FlyAction_EnterUNUSEHeightMode(void);
extern void FlyAction_AdjustDiffAngle(uint8_t changeNum);
extern void FlyAction_HeadLessModeChange(void);
extern void FlyAction_SaveDiffAngleParam(void);
extern void FlyAction_ChangeLidarAvoidMode(uint8_t id);

//电机测试模式
extern void FlyAction_TestMotorMode(void);
extern void TestMotorMode_TestA(uint8_t operateNum);
extern void TestMotorMode_TestB(uint8_t operateNum);
extern void TestMotorMode_TestC(uint8_t operateNum);
extern void TestMotorMode_TestD(uint8_t operateNum);

extern void set_targetpos_lc307(uint8_t flag);
extern void start_lc307_pos(void);

//系统复位
void ResetSystem(uint8_t isFromISR);

//左右手油门
#define LeftHandThrottle  1
#define RightHandThrottle 0 
static uint8_t ThrottleHand = RightHandThrottle;

//usb数据读取后,最终进入此回调函数.数据解码在此函数进行(任务环境)
void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
	//PS2数据数据解码
	USBH_HID_PS2_Decode(phost);
	
	//任务使用到的队列
	extern QueueHandle_t g_xQueueFlyControl;
	
	//ps2手柄信息
	extern PS2INFO_t ps2_info;
	PS2INFO_t* ps2 = &ps2_info;
	
	////////////////// 根据PS2手柄数据控制四轴飞行器 /////////////////////////////
	FlyControlType_t controlVal = { 0 };//定义控制量
	
	PS2KEY_State_t ps2_keystate = PS2KEYSTATE_NONE;
	
	extern EventGroupHandle_t g_xEventFlyAction; //四轴动作事件组
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	
	//start按键
	ps2_keystate = ps2->getKeyEvent(PS2KEY_START);
	if( PS2KEYSTATE_LONGCLICK == ps2_keystate )
	{
		//长按启动按键
		StartFlyAction();
	}
	else if( PS2KEYSTATE_SINGLECLICK == ps2_keystate || PS2KEYSTATE_DOUBLECLICK == ps2_keystate )
	{
		//单击或双击启动按键
		StopFlyAction();
	}
	
	//select按键
	ps2_keystate = ps2->getKeyEvent(PS2KEY_SELECT);
	if( PS2KEYSTATE_LONGCLICK == ps2_keystate )
	{
		FlyAction_EnterUNUSEHeightMode();
	}
	
	//右下扳机长按 -- 复位
	ps2_keystate = ps2->getKeyEvent(PS2KEY_R2);
	if( PS2KEYSTATE_LONGCLICK == ps2_keystate )
	{
		ResetSystem(0);
	}
	
	//左上下扳机 -- 电机测试功能(简单的油门加减)
	ps2_keystate = ps2->getKeyEvent(PS2KEY_L1);
	if( PS2KEYSTATE_LONGCLICK == ps2_keystate  ) 
	{
		FlyAction_HeadLessModeChange();
	}
	
	ps2_keystate = ps2->getKeyEvent(PS2KEY_L2);
	if( PS2KEYSTATE_LONGCLICK == ps2_keystate )
	{
		
	}
	
	//左右微调
	ps2_keystate = ps2->getKeyEvent(PS2KEY_UP);
	if( PS2KEYSTATE_SINGLECLICK == ps2_keystate || PS2KEYSTATE_DOUBLECLICK == ps2_keystate )
		FlyAction_AdjustDiffAngle(3);
	
	ps2_keystate = ps2->getKeyEvent(PS2KEY_DOWN);
	if( PS2KEYSTATE_SINGLECLICK == ps2_keystate || PS2KEYSTATE_DOUBLECLICK == ps2_keystate ) 
		FlyAction_AdjustDiffAngle(4);
	
	ps2_keystate = ps2->getKeyEvent(PS2KEY_LEFT);
	if( PS2KEYSTATE_SINGLECLICK == ps2_keystate || PS2KEYSTATE_DOUBLECLICK == ps2_keystate )
		FlyAction_AdjustDiffAngle(1);
	
	ps2_keystate = ps2->getKeyEvent(PS2KEY_RIGHT);
	if( PS2KEYSTATE_SINGLECLICK == ps2_keystate || PS2KEYSTATE_DOUBLECLICK == ps2_keystate ) 
		FlyAction_AdjustDiffAngle(2);
	

	
	//右摇杆长按,进入电机测试模式
	ps2_keystate = ps2->getKeyEvent(PS2KEY_RROCKER);
	if( PS2KEYSTATE_LONGCLICK == ps2_keystate )
		FlyAction_TestMotorMode();
	
	//电机测试模式下,右盘按键可对电机进行正反转的设置与测试
	if( uxBits&StartFly_Event )
	{
		//飞行状态下,左摇杆长按,切换避障模式
		ps2_keystate = ps2->getKeyEvent(PS2KEY_LROCKER);
		if( PS2KEYSTATE_LONGCLICK == ps2_keystate )
		{	//长按清除状态
			FlyAction_ChangeLidarAvoidMode(0);
		}
		else if( PS2KEYSTATE_SINGLECLICK == ps2_keystate )
		{	//单击启动跟随
			FlyAction_ChangeLidarAvoidMode(2);
		}
		else if( PS2KEYSTATE_DOUBLECLICK == ps2_keystate )
		{	//双击启动避障
			FlyAction_ChangeLidarAvoidMode(1);
		}
		
		ps2_keystate = ps2->getKeyEvent(PS2KEY_1GREEN);
		if( PS2KEYSTATE_SINGLECLICK == ps2_keystate || PS2KEYSTATE_DOUBLECLICK == ps2_keystate )
		{
			set_targetpos_lc307(2);
		}
		
		ps2_keystate = ps2->getKeyEvent(PS2KEY_2RED);
		if( PS2KEYSTATE_SINGLECLICK == ps2_keystate || PS2KEYSTATE_DOUBLECLICK == ps2_keystate )
		{
			set_targetpos_lc307(0);
		}
		
		ps2_keystate = ps2->getKeyEvent(PS2KEY_3BLUE);
		if( PS2KEYSTATE_SINGLECLICK == ps2_keystate || PS2KEYSTATE_DOUBLECLICK == ps2_keystate )
		{
			set_targetpos_lc307(3);
		}
		
		ps2_keystate = ps2->getKeyEvent(PS2KEY_4PINK);
		if( PS2KEYSTATE_SINGLECLICK == ps2_keystate || PS2KEYSTATE_DOUBLECLICK == ps2_keystate )
		{
			set_targetpos_lc307(1);
		}
	}
	else
	{
		
		//左摇杆长按,保存用户微调参数
		ps2_keystate = ps2->getKeyEvent(PS2KEY_LROCKER);
		if( PS2KEYSTATE_LONGCLICK == ps2_keystate )
			FlyAction_SaveDiffAngleParam();
		
		ps2_keystate = ps2->getKeyEvent(PS2KEY_1GREEN);
		if( PS2KEYSTATE_SINGLECLICK == ps2_keystate )
			TestMotorMode_TestA(0); //测试电机转动
		else if( PS2KEYSTATE_DOUBLECLICK == ps2_keystate )
			TestMotorMode_TestA(1); //修改电机方向
		else if( PS2KEYSTATE_LONGCLICK == ps2_keystate )
			TestMotorMode_TestA(2); //保存修改
		
		ps2_keystate = ps2->getKeyEvent(PS2KEY_2RED);
		if( PS2KEYSTATE_SINGLECLICK == ps2_keystate )
			TestMotorMode_TestB(0); //测试电机转动
		else if( PS2KEYSTATE_DOUBLECLICK == ps2_keystate )
			TestMotorMode_TestB(1); //修改电机方向
		else if( PS2KEYSTATE_LONGCLICK == ps2_keystate )
			TestMotorMode_TestB(2); //保存修改
		
		ps2_keystate = ps2->getKeyEvent(PS2KEY_3BLUE);
		if( PS2KEYSTATE_SINGLECLICK == ps2_keystate )
			TestMotorMode_TestC(0); //测试电机转动
		else if( PS2KEYSTATE_DOUBLECLICK == ps2_keystate )
			TestMotorMode_TestC(1); //修改电机方向
		else if( PS2KEYSTATE_LONGCLICK == ps2_keystate )
			TestMotorMode_TestC(2); //保存修改
		
		ps2_keystate = ps2->getKeyEvent(PS2KEY_4PINK);
		if( PS2KEYSTATE_SINGLECLICK == ps2_keystate )
			TestMotorMode_TestD(0); //测试电机转动
		else if( PS2KEYSTATE_DOUBLECLICK == ps2_keystate )
			TestMotorMode_TestD(1); //修改电机方向
		else if( PS2KEYSTATE_LONGCLICK == ps2_keystate )
			TestMotorMode_TestD(2); //保存修改
	}

	
	//检查是否需要写入控制队列的变量
	uint8_t WriteFlag = 0;
	static uint8_t lastWriteFlag = 0;
	
	//标记控制来源
	controlVal.source = PS2Cmd; 
	 
	//左右手油门切换
	extern void BeepTips(void);
	if( ps2->getKeyState(PS2KEY_L2) == PS2KEY_PressDOWN )
	{
		//切换至左手油门
		if( ps2->getKeyState(PS2KEY_LROCKER) == PS2KEY_PressDOWN )
			BeepTips(),
			ThrottleHand = LeftHandThrottle;
		
		//切换至右手油门
		if( ps2->getKeyState(PS2KEY_RROCKER) == PS2KEY_PressDOWN )
			BeepTips(),
			ThrottleHand = RightHandThrottle;
	}
	
	//控制速度
	static float control_step = 10.0f;
	
	//右上扳机长按，高速档与低速档的切换
	ps2_keystate = ps2->getKeyEvent(PS2KEY_R1);
	if( PS2KEYSTATE_SINGLECLICK == ps2_keystate ) 
	{
		static uint8_t flag = 0;
		flag = !flag;
		BeepTips();
		
		start_lc307_pos();
	}
	
	//右手油门
	if( RightHandThrottle == ThrottleHand )
	{
		//左右摇杆控制 -- 前后左右
		if( ps2->LX <= 5 ) controlVal.pitch = angle_to_rad(control_step);
		else if( ps2->LX > 250 ) controlVal.pitch = -angle_to_rad(control_step);
		else 
		{
			controlVal.pitch = 0 , WriteFlag++;
		}
		
		if( ps2->LY <= 5 ) controlVal.roll = angle_to_rad(control_step);//前推
		else if( ps2->LY > 250 ) controlVal.roll = -angle_to_rad(control_step);//后推
		else 
		{
			controlVal.roll = 0 , WriteFlag++;
		}
		
		//右手摇杆Y轴控制 -- 高度
		if( ps2->RY<=5 ) controlVal.height = 0.003f;
		else if( ps2->RY>250 ) controlVal.height = -0.003f;
		else controlVal.height = 0 , WriteFlag++;
		
		//右手摇杆X轴 -- 控制旋转
		if( ps2->RX<=5 ) controlVal.gyroz = -angle_to_rad(1.0f);
		else if( ps2->RX > 250 ) controlVal.gyroz = angle_to_rad(1.0f);
		else controlVal.gyroz = 0 , WriteFlag++;
	}
	
	//左手油门
	else if( LeftHandThrottle == ThrottleHand )
	{
		if( ps2->RX <= 5 ) controlVal.pitch = angle_to_rad(control_step);
		else if( ps2->RX > 250 ) controlVal.pitch = -angle_to_rad(control_step);
		else controlVal.pitch = 0 , WriteFlag++;
		
		if( ps2->RY <= 5 ) controlVal.roll = angle_to_rad(control_step);
		else if( ps2->RY > 250 ) controlVal.roll = -angle_to_rad(control_step);
		else controlVal.roll = 0 , WriteFlag++;
		
		if( ps2->LY<=5 ) controlVal.height = 0.003f;
		else if( ps2->LY>250 ) controlVal.height = -0.003f;
		else controlVal.height = 0 , WriteFlag++;
		
		if( ps2->LX<=5 ) controlVal.gyroz = -angle_to_rad(1.0f);
		else if( ps2->LX > 250 ) controlVal.gyroz = angle_to_rad(1.0f);
		else controlVal.gyroz = 0 , WriteFlag++;
	}

	//写入控制队列
	if( WriteFlag != 4 || (lastWriteFlag!=4 && WriteFlag == 4 ) )
	{
		WriteFlyControlQueue(controlVal,0,NULL);
	}
	lastWriteFlag = WriteFlag;
	
	////////////////// 根据PS2手柄数据控制四轴飞行器 END/////////////////////////////
}

