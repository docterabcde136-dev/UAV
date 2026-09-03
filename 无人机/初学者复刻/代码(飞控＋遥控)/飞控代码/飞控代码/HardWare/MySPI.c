#include "stm32f10x.h"                  // Device header
#include "NRF24L01_Reg.H"	

void MySPI_W_MOSI(uint8_t Value)
{
	GPIO_WriteBit(MOSI_Port,MOSI_Pin,(BitAction)Value);
}

void MySPI_W_SCLK(uint8_t Value)
{
	GPIO_WriteBit(SCK_Port,SCK_Pin,(BitAction)Value);
}

void MySPI_W_CSN(uint8_t Value)
{
	GPIO_WriteBit(CSN_Port,CSN_Pin,(BitAction)Value);
}

void MySPI_W_CE(uint8_t Value)
{
	GPIO_WriteBit(CE_Port,CE_Pin,(BitAction)Value);
}

uint8_t MySPI_R_MISO(void)
{
	return GPIO_ReadInputDataBit(MISO_Port,MISO_Pin);
}

//交换一个字节
uint8_t MySPI_SwapData(uint8_t Byte)//1111 1010
{
	uint8_t i,ReceiveByte=0x00;
	for(i=0;i<8;i++)
	{
		//SCLK高电平前放置好数据
		MySPI_W_MOSI(Byte&(0x80>>i));
		//拉高SCLK，开始交换数据
		MySPI_W_SCLK(1);
		if(MySPI_R_MISO()==1)
		{
			ReceiveByte=ReceiveByte|(0x80>>i);
		}
		//拉低SCLK，放置下一个数据
		MySPI_W_SCLK(0);
	}
	return ReceiveByte;
}
