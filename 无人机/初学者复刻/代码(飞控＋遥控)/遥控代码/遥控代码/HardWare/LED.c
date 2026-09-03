#include "stm32f10x.h"                  // Device header
#include "Variable.h"

#define LED_LDRIFT(x) GPIO_WriteBit(GPIOB,GPIO_Pin_7,(BitAction)(x))
#define LED_RDRIFT(x) GPIO_WriteBit(GPIOA,GPIO_Pin_5,(BitAction)(x))
#define LED_UNLOCK(x) GPIO_WriteBit(GPIOB,GPIO_Pin_1,(BitAction)(x))
#define LED_LOCK(x)   GPIO_WriteBit(GPIOB,GPIO_Pin_10,(BitAction)(x))

uint8_t LLbit=0;
uint8_t LRbit=0;

void LED_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	/*配置GPIO*/
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_7|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	
//	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
//	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	
}


void LED_Control(void)
{
	/*PCB上有个失误，上锁与解锁的LED弄反了了*/
	if(Key_LED==7) 
	{
		LED_LOCK(1) ;
		LED_UNLOCK(0);
	}
	if(Key_LED==8)
	{
		LED_UNLOCK(1);
		LED_LOCK(0);
	}

	if((Encoder_number/4)>2)
	{
		LED_LDRIFT(1);
		LED_RDRIFT(0);
	}
	if((Encoder_number/4)<-2)
	{
		LED_LDRIFT(0);
		LED_RDRIFT(1);
	}
	if((Encoder_number/4)<=2 && (Encoder_number/4)>=-2)
	{
		LED_LDRIFT(1);
		LED_RDRIFT(1);
	}
	
}

