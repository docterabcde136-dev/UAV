#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "Variable.h"

uint8_t Motor_time=4;



void Motor_Init(void)
{
	PWM_Init();
}


void Motor_Control(void)
{
  if(Lock==0 && connect_protect==0)
  {
			Set_Setcompare1(PWM_OUT1);
			Set_Setcompare2(PWM_OUT2);
			Set_Setcompare3(PWM_OUT3);
			Set_Setcompare4(PWM_OUT4);
	
  }
	else
	{
		Set_Setcompare1(0);
		Set_Setcompare2(0);
		Set_Setcompare3(0);
		Set_Setcompare4(0);
	}

}
