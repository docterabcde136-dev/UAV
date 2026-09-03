#ifndef __LIDAR_TASK_H
#define __LIDAR_TASK_H

#include <stdint.h>

// === 输出结构体 ===
typedef struct {
    float  center_angle;   // 中间角度（度），范围 [0, 360)
    float  avg_distance;   // 该区域平均距离（mm）
    int    point_count;    // 有效点数
    uint8_t valid;         // 1=找到有效区域，0=无
} LidarNearestDenseRegion;

extern LidarNearestDenseRegion LidarFollowRegion;

#endif
