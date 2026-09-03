/**
 * @file    bsp_imu.c
 * @brief   IMU 惯性测量单元驱动（ICM20948 + AK09916 磁力计）
 * @note    ICM20948: 9 轴 IMU（3 轴加速度计 + 3 轴陀螺仪 + 3 轴磁力计 AK09916）
 *          I2C 地址: ICM20948=0x68, AK09916=0x0C
 *          采用 bypass 模式直接访问 AK09916 磁力计
 *          姿态解算: Mahony 互补滤波（加速度计 + 陀螺仪融合）
 */

#include "bsp_imu.h"

/* ICM20948 寄存器定义 */
#include "icm20948_reg.h"

/* IMU 依赖 I2C 底层驱动 */
#include "bsp_iic.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

/**
 * @brief  ICM20948 初始化函数
 * @retval 0=成功, 1=失败（WHO_AM_I 校验失败或 I2C 通信异常）
 * @note   初始化序列：复位 -> 配置 Bank0~2 寄存器（电源、时钟、量程、滤波）
 *         -> bypass 模式使能 -> AK09916 磁力计初始化 (100Hz)
 */
static uint8_t ICM20948_Init(void)
{
	pIICInterface_t iicdev = &UserII2Dev; /* 获取 I2C 设备接口 */
	IIC_Status_t check_state = IIC_OK;    /* 累积 I2C 操作错误状态 */
	uint8_t writebuf = 0;

	/* === 选择 Bank 0 === */
	writebuf = REG_VAL_SELECT_BANK_0;
	check_state += iicdev->write_reg(ICM20948_DEV << 1, REG_BANK_SEL, &writebuf, 1, 500);

	/* 读取 WHO_AM_I 寄存器，验证设备身份 */
	check_state += iicdev->read_reg(ICM20948_DEV << 1, WHO_AM_I, &writebuf, 1, 500);
	if (0xEA != writebuf) return 1;       /* ICM20948 WHO_AM_I 应为 0xEA */

	/* 复位 ICM20948（PWR_MGMT_1 bit7=1: 设备复位） */
	writebuf = (1 << 7);
	check_state += iicdev->write_reg(ICM20948_DEV << 1, PWR_MGMT_1, &writebuf, 1, 500);
	iicdev->delay_ms(100);                /* 等待复位完成 */

	/* 配置 USER_CTRL 寄存器
	 * bit7=0: 禁用 DMP
	 * bit6=0: 禁用 FIFO
	 * bit5=0: 禁用 I2C Master（使用 bypass 模式访问磁力计）
	 * bit4=0: I2C_IF_DIS
	 * bit3=0: 禁用 DMP 复位
	 * bit2=0: 禁用 SRAM 复位
	 * bit1=0: 禁用 I2C Master 复位 */
	writebuf = 0x00;
	check_state += iicdev->write_reg(ICM20948_DEV << 1, USER_CTRL, &writebuf, 1, 500);

	/* 退出睡眠模式，使能温度传感器，自动选择时钟源 */
	writebuf = 0x01;
	check_state += iicdev->write_reg(ICM20948_DEV << 1, PWR_MGMT_1, &writebuf, 1, 100);

	/* === 选择 Bank 2（陀螺仪和加速度计配置） === */
	writebuf = REG_VAL_SELECT_BANK_2;
	check_state += iicdev->write_reg(ICM20948_DEV << 1, REG_BANK_SEL, &writebuf, 1, 100);

	/* 陀螺仪采样率分频: ODR = 1.1kHz / (1 + 4) = 220Hz
	 * 需配合 FCHOICE=1（旁路 DLPF）或 DLPF_CFG 配置 */
	writebuf = 0x04;
	check_state += iicdev->write_reg(ICM20948_DEV << 1, GYRO_SMPLRT_DIV, &writebuf, 1, 100);

	/* GYRO_CONFIG_1:
	 * FS_SEL[2:1]=3: 量程 ±2000dps
	 * FCHOICE[0]=1:   旁路 DLPF 滤波器
	 * DLPFCFG[5:3]=3: DLPF 3dB BW=11.6Hz, NBW=17.8Hz */
	writebuf = (3 << 1) | (1 << 0) | (3 << 3);
	check_state += iicdev->write_reg(ICM20948_DEV << 1, GYRO_CONFIG_1, &writebuf, 1, 100);

	/* 加速度计采样率分频: ODR = 1.125kHz / (1 + 4) ≈ 225Hz */
	writebuf = 0x04;
	check_state += iicdev->write_reg(ICM20948_DEV << 1, ACCEL_SMPLRT_DIV_2, &writebuf, 1, 100);

	/* ACCEL_CONFIG:
	 * FS_SEL[2:1]=0:   量程 ±2g
	 * FCHOICE[0]=1:    旁路 DLPF
	 * DLPFCFG[5:3]=5:  滤波配置 */
	writebuf = (0 << 1) | (1 << 0) | (5 << 3);
	check_state += iicdev->write_reg(ICM20948_DEV << 1, ACCEL_CONFIG, &writebuf, 1, 100);

	/* === 选择 Bank 0，配置磁力计 === */
	writebuf = REG_VAL_SELECT_BANK_0;
	check_state += iicdev->write_reg(ICM20948_DEV << 1, REG_BANK_SEL, &writebuf, 1, 100);

	/* 使能 I2C bypass 模式，允许主机直接通过 ICM20948 访问 AK09916 磁力计 */
	writebuf = (1 << 1);                  /* INT_PIN_CFG bit1=1: BYPASS_EN */
	check_state += iicdev->write_reg(ICM20948_DEV << 1, INT_PIN_CFG, &writebuf, 1, 100);

	/* 验证 AK09916 磁力计身份 */
	check_state += iicdev->read_reg(AK09916_DEV << 1, WIA, &writebuf, 1, 100);
	if (0x09 != writebuf) return 1;       /* AK09916 WHO_AM_I 应为 0x09 */

	/* 配置磁力计采样模式（CNTL2 寄存器）
	 * 0x00: 关闭测量
	 * 0x01: 单次测量
	 * 0x02: 连续模式1 (10Hz)
	 * 0x04: 连续模式2 (20Hz)
	 * 0x06: 连续模式3 (50Hz)
	 * 0x08: 连续模式4 (100Hz) */
	writebuf = (1 << 3);                  /* 100Hz 连续采样模式 */
	check_state += iicdev->write_reg(AK09916_DEV << 1, CNTL2, &writebuf, 1, 100);
	/* AK09916: 量程 ±4900uT, 分辨率 16 位 */

	/* 检查初始化过程中是否有 I2C 通信错误 */
	if (0 != check_state) return 1;

	return 0;
}

/**
 * @brief  ICM20948 反初始化（进入睡眠模式）
 * @retval 0=成功, 1=失败
 */
static uint8_t ICM20948_DeInit(void)
{
	pIICInterface_t iicdev = &UserII2Dev;
	IIC_Status_t check_state = IIC_OK;
	uint8_t writebuf = 0;

	/* 选择 Bank 0 */
	writebuf = REG_VAL_SELECT_BANK_0;
	check_state += iicdev->write_reg(ICM20948_DEV << 1, REG_BANK_SEL, &writebuf, 1, 500);

	/* 验证设备身份 */
	check_state += iicdev->read_reg(ICM20948_DEV << 1, WHO_AM_I, &writebuf, 1, 500);
	if (0xEA != writebuf) return 1;

	/* 进入睡眠模式（PWR_MGMT_1 bit6=1: SLEEP） */
	writebuf = (1 << 6);
	check_state += iicdev->write_reg(ICM20948_DEV << 1, PWR_MGMT_1, &writebuf, 1, 500);

	if (0 != check_state) return 1;
	return 0;
}

/* 零偏校准值（零点偏移）--------------------------------------------------------*/
static IMU_DATA_t      ZeroPoint    = { 0 }; /* 加速度计和陀螺仪的零点偏移 */
static ATTITUDE_DATA_t ZeroAttitude = { 0 }; /* 姿态角零点偏移 */

/**
 * @brief  读取 ICM20948 9 轴传感器原始数据
 * @param  data: 输出参数，存放读取并转换后的 IMU 数据
 * @note   加速度计量程 ±2g → 转换系数 0.000598 m/s²/LSB
 *          陀螺仪量程 ±2000dps → 转换系数 0.001065 rad/s/LSB
 *          磁力计量程 ±4900uT → 转换系数 0.1495 uT/LSB
 *          读取后自动减去零偏 (ZeroPoint)
 */
static void ImuUpdate(IMU_DATA_t *data)
{
	pIICInterface_t iicdev = &UserII2Dev;

	/* 连续读取加速度计 (6 字节) + 陀螺仪 (6 字节) = 12 字节 */
	uint8_t tmpbuf[12];
	iicdev->read_reg(ICM20948_DEV << 1, ACCEL_XOUT_H, tmpbuf, 12, 100);

	/* 提取加速度计 3 轴原始值（MSB 在前） */
	data->accel.x = (short)(tmpbuf[0] << 8 | tmpbuf[1]);
	data->accel.y = (short)(tmpbuf[2] << 8 | tmpbuf[3]);
	data->accel.z = (short)(tmpbuf[4] << 8 | tmpbuf[5]);

	/* 加速度计原始值 → m/s²（量程 ±2g, 16 位分辨率）
	 * 系数 = (2 * 2 * 9.8) / 65536 = 0.00059814453125 */
	data->accel.x *= 0.00059814453125f;
	data->accel.y *= 0.00059814453125f;
	data->accel.z *= 0.00059814453125f;

	/* 以下为加速度计校准补偿（注释掉，如需启用请取消注释） */
	// data->accel.x = 1.0292f * data->accel.x - 0.3405f;
	// data->accel.y = 1.0090f * data->accel.y - 0.1166f;
	// data->accel.z = 0.9926f * data->accel.z + 0.0506f;

	/* 提取陀螺仪 3 轴原始值 */
	data->gyro.x = (short)(tmpbuf[6]  << 8 | tmpbuf[7]);
	data->gyro.y = (short)(tmpbuf[8]  << 8 | tmpbuf[9]);
	data->gyro.z = (short)(tmpbuf[10] << 8 | tmpbuf[11]);

	/* 陀螺仪原始值 → rad/s
	 * 系数 = (2000 * π / 180) / 65536 = 0.000532 * (π/180) = 0.001065 */
	data->gyro.x *= 0.06103515625f;       /* 转换为 dps */
	data->gyro.y *= 0.06103515625f;
	data->gyro.z *= 0.06103515625f;

	data->gyro.x *= 0.01745329252f;       /* dps 转换为 rad/s (× π/180) */
	data->gyro.y *= 0.01745329252f;
	data->gyro.z *= 0.01745329252f;

	/* 减去陀螺仪零偏 */
	data->gyro.x -= ZeroPoint.gyro.x;
	data->gyro.y -= ZeroPoint.gyro.y;
	data->gyro.z -= ZeroPoint.gyro.z;

	/* 读取磁力计数据（7 字节数据 + 1 字节 ST2 状态） */
	uint8_t magnbuf[8];
	iicdev->read_reg(AK09916_DEV << 1, HXL, magnbuf, 8, 100);

	/* 检查磁力计数据溢出标志（ST2 bit3: HOFL） */
	if (0 == ((magnbuf[7] >> 3) & 0x01))
	{
		/* 提取磁力计 3 轴原始值（注意: AK09916 小端序，低字节在前） */
		data->magn.x = (short)(magnbuf[1] << 8 | magnbuf[0]);
		data->magn.y = (short)(magnbuf[3] << 8 | magnbuf[2]);
		data->magn.z = (short)(magnbuf[5] << 8 | magnbuf[4]);

		/* 磁力计原始值 → uT（量程 ±4900uT, 16 位分辨率）
		 * 系数 = 4900 / 32768 = 0.1495361328125 */
		data->magn.x *= 0.1495361328125f;
		data->magn.y *= 0.1495361328125f;
		data->magn.z *= 0.1495361328125f;
	}
	else
	{
		/* 数据溢出不可靠，保留上一帧数据，不更新 */
	}
}

/**
 * @brief  设定 IMU 轴零点偏移（用于陀螺仪和加速度计校准）
 * @param  point: 包含零点偏差的 IMU 数据结构体指针
 */
static void setZeroPoint_axis(const IMU_DATA_t* point)
{
	memcpy(&ZeroPoint, point, sizeof(IMU_DATA_t));
}

/**
 * @brief  设定姿态角零点偏移（用于姿态初始化对准）
 * @param  attitude: 包含姿态零点偏差的结构体指针
 */
static void setZeroPoint_attitude(const ATTITUDE_DATA_t* attitude)
{
	memcpy(&ZeroAttitude, attitude, sizeof(ATTITUDE_DATA_t));
}

/* Mahony 互补滤波参数 ---------------------------------------------------------*/
static const float Kp = 1.000f;           /* 比例系数（加速度计修正权重） */
static const float Ki = 0.001f;           /* 积分系数（消除陀螺仪长期漂移） */

/**
 * @brief  姿态更新（Mahony 互补滤波算法）
 * @param  imudata:  输入的 IMU 原始数据（陀螺仪 + 加速度计）
 * @param  attitude: 输出的姿态角（roll, pitch, yaw，单位 rad）
 * @note   使用四元数表示姿态，融合加速度计和陀螺仪数据
 *          陀螺仪提供高频动态响应，加速度计提供低频绝对参考（重力方向）
 *          PI 控制器修正陀螺仪积分漂移
 *          采样周期 SamplePeriod = 0.005s (200Hz)
 */
static void attitudeUpdate(IMU_DATA_t imudata, ATTITUDE_DATA_t *attitude)
{
	static float eInt[3] = { 0 };          /* PI 控制器的积分误差累积 */
	static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; /* 四元数初始值（无旋转） */
	float norm;
	float vx, vy, vz;                        /* 加速度计估算的重力方向向量 */
	float ex, ey, ez;                        /* 姿态误差（叉积） */
	float pa, pb, pc, pd;                    /* 四元数更新前的备份 */
	float SamplePeriod = 0.005;              /* 采样周期 5ms (200Hz) */

	/* 提取传感器数据 */
	float gx = imudata.gyro.x;               /* 陀螺仪: X 轴角速度 (rad/s) */
	float gy = imudata.gyro.y;               /* 陀螺仪: Y 轴角速度 (rad/s) */
	float gz = imudata.gyro.z;               /* 陀螺仪: Z 轴角速度 (rad/s) */
	float ax = imudata.accel.x;              /* 加速度计: X 轴 (m/s²) */
	float ay = imudata.accel.y;              /* 加速度计: Y 轴 (m/s²) */
	float az = imudata.accel.z;              /* 加速度计: Z 轴 (m/s²) */

	/* 预计算四元数乘积项（减少重复计算） */
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	/* 加速度计归一化（只关心方向，不关心幅值） */
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm < 1e-10f) return;               /* 加速度太小，跳过本帧 */
	ax /= norm;
	ay /= norm;
	az /= norm;

	/* 根据当前四元数估算重力方向在机体坐标系中的投影 */
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = (q0q0 - q1q1 - q2q2 + q3q3);

	/* 计算姿态误差：加速度计测量值与估算重力方向的叉积 */
	ex = ay * vz - az * vy;
	ey = az * vx - ax * vz;
	ez = ax * vy - ay * vx;

	/* PI 控制器：累积积分误差 + 比例修正 */
	eInt[0] += ex;
	eInt[1] += ey;
	eInt[2] += ez;

	gx += Kp * ex + Ki * eInt[0];            /* 用 PI 输出修正陀螺仪角速度 */
	gy += Kp * ey + Ki * eInt[1];
	gz += Kp * ez + Ki * eInt[2];

	/* 一阶龙格-库塔法更新四元数 */
	pa = q0;
	pb = q1;
	pc = q2;
	pd = q3;
	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * (0.5f * SamplePeriod);
	q1 = pb + ( pa * gx + pc * gz - pd * gy) * (0.5f * SamplePeriod);
	q2 = pc + ( pa * gy - pb * gz + pd * gx) * (0.5f * SamplePeriod);
	q3 = pd + ( pa * gz + pb * gy - pc * gx) * (0.5f * SamplePeriod);

	/* 四元数归一化（防止数值漂移导致非单位四元数） */
	norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 /= norm;
	q1 /= norm;
	q2 /= norm;
	q3 /= norm;

	/* 四元数 → 欧拉角转换
	 * roll:  绕 X 轴旋转（横滚角）
	 * pitch: 绕 Y 轴旋转（俯仰角）
	 * yaw:   绕 Z 轴旋转（偏航角） */
	attitude->roll  = atan2(2 * q2 * q3 + 2 * q0 * q1,
	                        -2 * q1 * q1 - 2 * q2 * q2 + 1);
	attitude->pitch = asin(-2 * q1 * q3 + 2 * q0 * q2);
	attitude->yaw   = atan2(2 * (q1 * q2 + q0 * q3),
	                        q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);

	/* 减去姿态零点偏移 */
	attitude->roll  -= ZeroAttitude.roll;
	attitude->pitch -= ZeroAttitude.pitch;
	attitude->yaw   -= ZeroAttitude.yaw;
}

/* IMU 接口结构体实例 ---------------------------------------------------------*/
/* 通过函数指针向上层 APP 暴露统一的 IMU 操作接口 */

IMUInterface_t UserICM20948 = {
	.Init                   = ICM20948_Init,            /* IMU 初始化 */
	.DeInit                 = ICM20948_DeInit,          /* IMU 反初始化 */
	.UpdateZeroPoint_axis   = setZeroPoint_axis,        /* 设定轴零点偏移 */
	.UpdateZeroPoint_attitude = setZeroPoint_attitude,  /* 设定姿态零点偏移 */
	.Update_9axisVal        = ImuUpdate,                /* 读取 9 轴传感器数据 */
	.UpdateAttitude         = attitudeUpdate            /* Mahony 姿态解算 */
};


#if 0 /* 以下为 ICM20948 I2C Master 模式访问磁力计的备用方法，仅供参考 */
/**
 * @brief  通过 ICM20948 的 I2C Master 模式读取 AK09916 的 WHO_AM_I
 * @retval 1=设备正确(0x09), 0=设备异常
 * @note   此方法使用 ICM20948 内部 I2C Master 控制器间接访问磁力计
 *          当前代码使用的是 bypass 模式（更简单直接）
 */
uint8_t Check_AK09916_WHO_AM_I(void)
{
	pIICInterface_t iicdev = &UserII2Dev;
	uint8_t write_buf = 0;

	/* 选择 Bank 3（I2C Master 配置寄存器所在 Bank） */
	write_buf = REG_VAL_SELECT_BANK_3;
	iicdev->write_reg(ICM20948_DEV << 1, REG_BANK_SEL, &write_buf, 1, 100);

	/* 设置 I2C SLV0 要访问的从设备地址（bit7=1 表示读操作） */
	write_buf = AK09916_DEV | (1 << 7);
	iicdev->write_reg(ICM20948_DEV << 1, I2C_SLV0_ADDR, &write_buf, 1, 100);

	/* 设置 I2C SLV0 要读取的寄存器地址 */
	write_buf = WIA;
	iicdev->write_reg(ICM20948_DEV << 1, I2C_SLV0_REG, &write_buf, 1, 100);

	/* 使能 SLV0 读取（bit7=1: 使能, bit0=1: 读取 1 字节） */
	write_buf = (1 << 7) | (1 << 0);
	iicdev->write_reg(ICM20948_DEV << 1, I2C_SLV0_CTRL, &write_buf, 1, 100);

	/* 切回 Bank 0 */
	write_buf = REG_VAL_SELECT_BANK_0;
	iicdev->write_reg(ICM20948_DEV << 1, REG_BANK_SEL, &write_buf, 1, 100);

	/* 使能 I2C Master 模式 */
	iicdev->read_reg(ICM20948_DEV << 1, USER_CTRL, &write_buf, 1, 100);
	write_buf |= (1 << 5);               /* 使能 I2C MST */
	iicdev->write_reg(ICM20948_DEV << 1, USER_CTRL, &write_buf, 1, 100);

	/* 等待读取完成（此处可加入延时或中断等待） */

	/* 禁用 I2C Master 模式 */
	iicdev->read_reg(ICM20948_DEV << 1, USER_CTRL, &write_buf, 1, 100);
	write_buf &= ~(1 << 5);              /* 禁用 I2C MST */
	iicdev->write_reg(ICM20948_DEV << 1, USER_CTRL, &write_buf, 1, 100);

	/* 从 EXT_SLV_SENS_DATA_00 寄存器读取 SLV0 获取的数据 */
	iicdev->read_reg(ICM20948_DEV << 1, EXT_SLV_SENS_DATA_00, &write_buf, 1, 100);

	/* AK09916 WHO_AM_I 应为 0x09 */
	if (write_buf == 0x09)
		write_buf = 1;
	else
		write_buf = 0;

	return write_buf;
}
#endif
