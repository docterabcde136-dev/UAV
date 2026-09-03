#ifndef __SHOW_TASK_H
#define __SHOW_TASK_H

#include <stdint.h>

typedef struct{
	float roll;
	float pitch;
	float yaw;
	float gyrox;
	float gyroy;
	float gyroz;
	float accelx;
	float accely;
	float accelz;
	float m1;
	float m2;
	float m3;
	float m4;
	float balanceTaskFreq;
	float height;
	float spa06Height;
	float spa06NofilterHeight;
	float c_roll;
	float c_pitch;
	float c_yaw;
	float c_height;
	float posx;
	float posy;
	float speedx;
	float speedy;
	float zero_roll;
	float zero_pitch;
	float targetX;
	float targetY;
	uint16_t lidar_freq;
	float lidar_qian;
	float lidar_hou;
	float lidar_zuo;
	float lidar_you;
}APPShowType_t;

#endif

