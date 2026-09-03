#include "stm32f10x.h"                  // Device header
#include "Delay.h"  

void MyI2C_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_10,(BitAction)BitValue);
	Delay_us(10);
}
void MyI2C_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_11,(BitAction)BitValue);
	Delay_us(10);
}

uint8_t MyI2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11);
	Delay_us(10);
	return BitValue;
}

void MyI2C_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11);
}

void MyI2C_Start(void)
{
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(0);
}

void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for(i=0;i<8;i++)
	{
		MyI2C_W_SDA(Byte&(0x80>>i));
		MyI2C_W_SCL(1);
		MyI2C_W_SCL(0);
	}
}

uint8_t MyI2C_Receive(void)
{
	MyI2C_W_SDA(1);//释放sda给从机
	uint8_t i,R_Byte=0x00;//R_Byte为接收到的数据
	
	for(i=0;i<8;i++)
	{
		MyI2C_W_SCL(1);
		if(MyI2C_R_SDA()==1) {R_Byte|=(0x80>>i);}
		MyI2C_W_SCL(0);
	}

	return R_Byte;
}

void MyI2C_Send_ACK(uint8_t send_ack)//发送应答：从机给主机发数据，主机需要给一个应答
{
		MyI2C_W_SDA(send_ack);
		MyI2C_W_SCL(1);
		MyI2C_W_SCL(0);
}

uint8_t MyI2C_Receive_ACK(void)//接收应答：主机给从机发数据，主机需要接收一个应答
{
	uint8_t Receive_ack=0;
	MyI2C_W_SDA(1);//释放sda，允许从机发数据
	
	MyI2C_W_SCL(1);
	Receive_ack=MyI2C_R_SDA();
	MyI2C_W_SCL(0);
	
	return Receive_ack;
}
