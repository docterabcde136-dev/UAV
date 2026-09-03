/**
 * @file    bsp_imu.h
 * @brief   IMU 惯性测量单元驱动接口定义（ICM20948 9轴传感器）
 */

#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include <stdint.h>

/* 三轴数据结构体 */
typedef struct {
	float x;                 /* X 轴 */
	float y;                 /* Y 轴 */
	float z;                 /* Z 轴 */
} PrivateBuf_t;

/* IMU 9 轴原始数据 */
typedef struct {
	PrivateBuf_t gyro;       /* 3轴陀螺仪角速度 (rad/s) */
	PrivateBuf_t accel;      /* 3轴加速度计 (m/s²) */
	PrivateBuf_t magn;       /* 3轴磁力计 (uT) */
} IMU_DATA_t;

/* 姿态角数据 */
typedef struct {
	float roll;              /* 横滚角 (rad) */
	float pitch;             /* 俯仰角 (rad) */
	float yaw;               /* 偏航角 (rad) */
} ATTITUDE_DATA_t;

/* IMU 操作接口 */
typedef struct {
	uint8_t (*Init)(void);   /* 初始化（返回 0=成功, 1=异常） */
	uint8_t (*DeInit)(void); /* 反初始化（返回 0=成功, 1=异常） */

	void (*UpdateZeroPoint_axis)(const IMU_DATA_t* point);               /* 校准 9 轴零点 */
	void (*UpdateZeroPoint_attitude)(const ATTITUDE_DATA_t* attitude);   /* 校准姿态零点 */
	void (*Update_9axisVal)(IMU_DATA_t* imudata);                         /* 读取 9 轴原始数据 */
	void (*UpdateAttitude)(IMU_DATA_t imudata, ATTITUDE_DATA_t* attitude); /* Mahony 姿态解算 */
} IMUInterface_t, *pIMUInterface_t;

extern IMUInterface_t UserICM20948; /* ICM20948 IMU 接口实例 */

#endif /* __BSP_IMU_H */
