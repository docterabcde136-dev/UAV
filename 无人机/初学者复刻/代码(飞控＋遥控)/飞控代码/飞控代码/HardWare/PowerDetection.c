#include "stm32f10x.h"                  // Device header
#include "Variable.h"


void AD_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	/*ADC时钟分频*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;//模拟输入模式
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	/*分组*/
	ADC_RegularChannelConfig(ADC1,ADC_Channel_9,1,ADC_SampleTime_55Cycles5);
	/*ADC结构体*/
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;
	ADC_InitStructure.ADC_NbrOfChannel=1;
	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;
	ADC_InitStructure.ADC_ScanConvMode=DISABLE;
	ADC_Init(ADC1,&ADC_InitStructure);
	/*开始ADC*/
	ADC_Cmd(ADC1,ENABLE);
	/*校验ADC*/
	//复位校准
	ADC_ResetCalibration(ADC1);
	//等待复位完成
	while(ADC_GetResetCalibrationStatus(ADC1)==SET);
	//开始校准
	ADC_StartCalibration(ADC1);
	//等待校准完成
	while(ADC_GetCalibrationStatus(ADC1)==SET);
}

uint16_t Get_ADC(void)
{
	/*开始转换*/
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
  /*等待转换完成*/
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	/*获取转换结果*/
	return ADC_GetConversionValue(ADC1);
}


/*初始化ADC函数*/
void GetBattery_Init(void)
{
	AD_Init();
}

///*获取一次电量(平均前)*/
float GetBattery(void)
{
  float temp_battery;
	temp_battery=(3.3*2*Get_ADC()/4096);
	return temp_battery;
}

/*获取平均电量(读取10次adc对应的电量值，再取平均数)*/
void GetCur_Power(void)
{
		if(++Battery_num<=100)
		{
			Battery_temp=GetBattery();
			Battery_total+=Battery_temp;
		}
		else
		{
			Battery_average=(Battery_total/100);
			Battery_total=0;
			Battery_num=0;
		}
}
