/**
 * ============================================================
 * bsp_N10.c — N10 360°激光雷达驱动
 * ============================================================
 * 功能: 360°激光雷达数据解析
 * 规格: 1圈500点, 角度分辨率0.8°
 * 处理:
 *   - 角度增量计算 + 索引映射
 *   - 重复点取平均 + 漏点插值填充
 *   - 距离数据滤波
 * ============================================================
 */

#include "bsp_N10.h"
#include "stdio.h"
#include "math.h"

//1个雷达点云数据信息
typedef struct {
	uint16_t distance;//距离信息
	uint8_t  peak;    //强度信息
}N10_OnePointCloud_t;

//DMA使用数组，readonly
OriData_N10_t DMABuf_oridata_N10;

//1圈雷达点云数据，角度分辨率0.8°.
N10_OnePointCloud_t N10_PointCloud_Data[450];

void N10_DataHandle(uint8_t* oribuf)
{
	//1帧数据校验
	uint8_t CrcVal = 0;
	for(uint8_t i=0;i<57;i++) CrcVal+=oribuf[i];
	
	if( CrcVal!=oribuf[57] )
	{
		/* 数据校验失败的处理 */
	}

	//起始角度
	float startangle,endangle,angleIncr;

	//计算当前数据帧的起始角度
	startangle = (float)((uint16_t)(oribuf[5]<<8 | oribuf[6]))/100.f;
	endangle = (float)((uint16_t)(oribuf[55]<<8 | oribuf[56]))/100.f;
	
	//计算角度每个数据之间的角度增量值：角度范围/分辨率增量区间
	if( startangle > endangle ) angleIncr = (endangle + 360.f - startangle)/15.0f;
	else                        angleIncr = (endangle - startangle)/15.0f;
	
	//数值下标索引值
	uint16_t index = 0;            //当前角度对应的数组下标值
	uint16_t lostindex = 0;        //用于检查是否有跳过下标的情况
	static uint16_t lastindex = 0;//用于保存上一次的下标，辅组判断是否完成1圈的数据接收、是否有跳过下标
	
	//转换下标使用的临时变量
	float tmpanlge;
	
	for(uint8_t i=0;i<16;i++)
	{
		tmpanlge = startangle + (i*angleIncr);      //1帧数据12度,包含16个数据,使用角度增量递增
		if( tmpanlge > 360.0f ) tmpanlge -= 360.0f; //角度超出360°后从0继续开始
		
		index = round(tmpanlge/0.8f);//根据角度计算数组下标的位置，数组下标*0.8 = 实际的雷达角度
		index = index%450;           //防止数组溢出，使用450个数据表示0~360°，取决于雷达分辨率，N10为0.8°
		
		//索引值完成了1次轮询,说明处理好了1圈雷达的数据.
		if( index<lastindex )
		{
			/* 完成了1圈的数据处理，通知需要使用的任务，此时雷达数据可用了 */
			
			/* 计算1圈数据接收完成的频率，N10为10Hz */
			//printf("%d\r\n",debug->UpdateFreq(&debugpriv));
			
			/* 检查是否存在遗漏点 */
			lostindex = (index + 450) - lastindex;
		}
		else
		{	/* 检查是否存在遗漏点 */
			lostindex = index  - lastindex;
		}
		
		//计算当前角度下的距离与强度信息
		N10_OnePointCloud_t TmpPonit = { 0 };
		TmpPonit.distance = oribuf[7+(i*3)] << 8 | oribuf[8+(i*3)];
		TmpPonit.peak = oribuf[9+(i*3)];
		
		//根据索引值的不同情况进行数据赋值
		if( 1 == lostindex || lostindex >= 3 ) //1为正常情况，3为雷达数据本身存在漏点，大于3则为刚上电瞬间的随机情况.这些情况直接赋值
		{
			N10_PointCloud_Data[ index ].distance = TmpPonit.distance;
			N10_PointCloud_Data[ index ].peak = TmpPonit.peak;
		}
		else if( 0 == lostindex ) //出现下标重复的情况,则将两次结果取平均
		{
			N10_PointCloud_Data[ index ].distance = (N10_PointCloud_Data[ index ].distance + TmpPonit.distance)/2;
			N10_PointCloud_Data[ index ].peak    =  (N10_PointCloud_Data[ index ].peak + TmpPonit.peak)/2;
		}
		else if( 2 == lostindex ) //若出现遗漏的点，将遗漏点的两边数值取平均，再赋值到遗漏点处
		{
			//先赋值当前角度
			N10_PointCloud_Data[ index ].distance = TmpPonit.distance;
			N10_PointCloud_Data[ index ].peak = TmpPonit.peak;
			
			//漏掉的上一个角度索引赋值,将左右两边值取平均作为该点的值.
			N10_PointCloud_Data[ (lastindex + 1)%450 ].distance = ( N10_PointCloud_Data[ index ].distance + N10_PointCloud_Data[lastindex].distance ) / 2 ;
			N10_PointCloud_Data[ (lastindex + 1)%450 ].peak =     ( N10_PointCloud_Data[ index ].peak     + N10_PointCloud_Data[lastindex].peak ) / 2 ;
		}
		
		//保存本次索引值
		lastindex = index;
	}
	
}
