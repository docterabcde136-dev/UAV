/**
 * ============================================================
 * bsp_imu.h — IMU驱动接口定义 (ICM20948)
 * ============================================================
 * 数据结构:
 *   IMU_DATA_t     — 9轴原始数据 (陀螺仪+加速度计+磁力计)
 *   ATTITUDE_DATA_t — 姿态角 (横滚/俯仰/偏航, 单位弧度)
 * 接口:
 *   IMUInterface_t — 标准IMU操作接口
 *     Init/DeInit: 初始化与去初始化
 *     Update_9axisVal: 读9轴原始值
 *     UpdateAttitude: 姿态解算 (Mahony互补滤波)
 *     UpdateZeroPoint: 零点标定
 * ============================================================
 */

#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include <stdint.h>

typedef struct{ //中间私有变量
	float x;
	float y;
	float z;
}PrivateBuf_t;
	
typedef struct{
	PrivateBuf_t gyro; //3轴角速度
	PrivateBuf_t accel;//3轴加速度
	PrivateBuf_t magn;//3轴磁力计
}IMU_DATA_t;

typedef struct{
	float roll;
	float pitch;
	float yaw;
}ATTITUDE_DATA_t;


//IMU操作接口
typedef struct {
    uint8_t (*Init)(void);   //返回值：1错误 0无异常
    uint8_t (*DeInit)(void); //返回值：1错误 0无异常
	
	void (*UpdateZeroPoint_axis)(const IMU_DATA_t* point);               //更新9轴数据零点
	void (*UpdateZeroPoint_attitude)(const ATTITUDE_DATA_t* attitude);   //更新姿态零点
    void (*Update_9axisVal)(IMU_DATA_t* imudata);                         //更新9轴数据
	void (*UpdateAttitude)(IMU_DATA_t imudata,ATTITUDE_DATA_t *attitude); //更新姿态角
	
}IMUInterface_t,*pIMUInterface_t;

extern IMUInterface_t UserICM20948;

#endif /* __BSP_IMU_H */
