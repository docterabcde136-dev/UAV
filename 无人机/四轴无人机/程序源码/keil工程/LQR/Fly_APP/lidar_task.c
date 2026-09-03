#include "lidar_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include "bsp_stl06n.h"
#include "KD_Tree.h"
#include "bsp_RTOSdebug.h"

#include <stdio.h>
#include <string.h>

#include "usart.h"
#include "show_task.h"
#include "balance_task.h"

//#define USE_PYTHON_DEBUG

//static pRtosDebugInterface_t freq_debug = &RTOSTaskDebug;
//static RtosDebugPrivateVar debugpriv = { 0 };

extern QueueHandle_t g_xQueueLidarBuffer;

//雷达原始数据,用于进行坐标计算
static Stl06NAngleBuffer_t LidarRaw;


#if defined USE_PYTHON_DEBUG
#pragma pack(1)
//1个雷达点云数据信息
typedef struct {
	uint16_t distance;//距离信息
	uint8_t  peak;    //强度信息
}SendPointCloud_t;

typedef struct{
	uint8_t Head1;
	uint8_t Head2;
	Stl06NPointStructDef buf[500];
	uint8_t BccCheck;
	uint8_t End;
}Lidarmsg_t;
#pragma pack()

Lidarmsg_t send_to_python;

uint8_t Lidarmsg_BCC(const Lidarmsg_t* msg)
{
    uint8_t bcc = 0;
    const uint8_t* data = (const uint8_t*)msg;
    for (size_t i = 0; i < sizeof(Lidarmsg_t) - 2; i++) {
        bcc ^= data[i];
    }
    return bcc;
}
#endif


// 可变参数宏定义
#define LIDAR_BUFFER_SIZE 500    //雷达数组长度
#define ANGLE_RESOLUTION 0.72f   //雷达分辨率 360°/分辨率 = 数组长度
#define OBSTACLE_THRESHOLD_PERCENT 60.0f //百分比阈值

//检索指定范围、指定距离内是否存在障碍物
uint8_t detect_obstacle(const Stl06NPointStructDef* lidar_buffer, float start_angle, float end_angle, float distance_threshold) {
    // 确保角度范围有效，允许溢出
    if (start_angle < 0.0f || end_angle < 0.0f || start_angle > 360.0f || end_angle > 360.0f) {
        return 0;
    }

    // 计算起始和结束索引
    int start_index = (int)(start_angle / ANGLE_RESOLUTION);
    int end_index = (int)(end_angle / ANGLE_RESOLUTION);

    // 统计小于阈值的点数和总点数
    int points_below_threshold = 0;
    int total_points = 0;

    if (start_index <= end_index) {
        // 没有跨0°的情况
        total_points = end_index - start_index + 1;
        for (int i = start_index; i <= end_index; i++) {
            if (i < LIDAR_BUFFER_SIZE && lidar_buffer[i].distance < distance_threshold) {
                points_below_threshold++;
            }
        }
    } else {
        // 跨0°的情况（例如330°到60°）
        // 第一段：从start_index到数组末尾
        for (int i = start_index; i < LIDAR_BUFFER_SIZE; i++) {
            if (lidar_buffer[i].distance < distance_threshold) {
                points_below_threshold++;
            }
            total_points++;
        }
        // 第二段：从0到end_index
        for (int i = 0; i <= end_index; i++) {
            if (lidar_buffer[i].distance < distance_threshold) {
                points_below_threshold++;
            }
            total_points++;
        }
    }

    // 确保有有效点数
    if (total_points <= 0) {
        return 0;
    }

    // 计算百分比
    float percentage = (float)points_below_threshold / total_points * 100.0f;

    // 判断是否超过阈值百分比
    return percentage >= OBSTACLE_THRESHOLD_PERCENT;
}

///// 分桶统计算法 //////
#define DISTANCE_BIN_SIZE       100.0f    // 每100mm一个桶
#define MIN_VALID_DISTANCE      50.0f     // 设置雷达近点的过滤噪声
#define MAX_VALID_DISTANCE      10000.0f   // 设置雷达最远点过滤噪声
#define MAX_BINS                200       // 0~20m / 100mm = 200个桶

typedef struct {
    int   count;   // 该桶内点数
    float sum;     // 距离总和（用于求平均）
} DistanceBin;

//获取指定角度范围的主导距离平均值
uint8_t get_dominant_distance_avg(
    const Stl06NPointStructDef* lidar_buffer,
    float start_angle,
    float end_angle,
    float* avg_distance)
{
    // --- 参数检查 ---
    if (lidar_buffer == NULL || avg_distance == NULL) {
        return 0;
    }
    if (start_angle < 0.0f || end_angle < 0.0f ||
        start_angle > 360.0f || end_angle > 360.0f) {
        return 0;
    }

    *avg_distance = -1.0f;

    // --- 计算索引 ---
    int start_index = (int)(start_angle / ANGLE_RESOLUTION);
    int end_index   = (int)(end_angle   / ANGLE_RESOLUTION);

    // --- 初始化统计桶 ---
    DistanceBin bins[MAX_BINS];
    memset(bins, 0, sizeof(bins));  // 全部清零
    int total_points = 0;

    // --- 辅助函数：处理单个点（内联替代 lambda）---
    #define PROCESS_POINT(idx) do { \
        if ((idx) >= LIDAR_BUFFER_SIZE) break; \
        float d = lidar_buffer[(idx)].distance; \
        if (d < MIN_VALID_DISTANCE || d > MAX_VALID_DISTANCE) break; \
        int bin_idx = (int)(d / DISTANCE_BIN_SIZE); \
        if (bin_idx >= MAX_BINS) bin_idx = MAX_BINS - 1; \
        bins[bin_idx].count++; \
        bins[bin_idx].sum += d; \
        total_points++; \
    } while(0)

    // --- 遍历点云---
    if (start_index <= end_index) {
        // 不跨0°
        for (int i = start_index; i <= end_index; i++) {
            PROCESS_POINT(i);
        }
    } else {
        // 跨0°：分两段
        for (int i = start_index; i < LIDAR_BUFFER_SIZE; i++) {
            PROCESS_POINT(i);
        }
        for (int i = 0; i <= end_index; i++) {
            PROCESS_POINT(i);
        }
    }

    #undef PROCESS_POINT  // 清理宏

    // --- 无有效点 ---
    if (total_points == 0) {
        return 0;
    }

    // --- 找到计数最多的桶 ---
    int max_count = 0;
    int best_bin = -1;
    for (int i = 0; i < MAX_BINS; i++) {
        if (bins[i].count > max_count) {
            max_count = bins[i].count;
            best_bin = i;
        }
    }

    if (best_bin == -1) {
        return 0;
    }

    // --- 计算平均距离并返回 ---
    *avg_distance = bins[best_bin].sum / (float)bins[best_bin].count;
    return 1;
}


#define WINDOW_ANGLE          30.0f     // 滑动窗口大小（度）

//雷达跟随数据处理对象
LidarNearestDenseRegion LidarFollowRegion;

/**
 * @brief 在一圈点云中找出“最近且最密集”的区域，返回中间角度
 * @param buffer    激光雷达原始数据
 * @param result    输出结果
 * @return uint8_t  1=成功，0=失败（无有效点）
 */
uint8_t find_nearest_dense_region(const Stl06NPointStructDef* buffer,
                                  LidarNearestDenseRegion* result)
{
	static float LastAngle = 0;
	static uint8_t filterCnt = 0;
	
    if (!buffer || !result) return 0;

    // 计算窗口包含的点数
    int window_points = (int)(WINDOW_ANGLE / ANGLE_RESOLUTION);  // ≈41点
    if (window_points < 1) window_points = 1;

    float best_score = 1e9f;      // 评分：越小越好
    int   best_start_idx = -1;
    float best_avg_dist = 0.0f;
    int   best_count = 0;

    // 滑动窗口遍历（环形）
    for (int start = 0; start < LIDAR_BUFFER_SIZE; start++) {
        float sum_dist = 0.0f;
        int   count = 0;

        // 窗口内累加
        for (int i = 0; i < window_points && count < window_points; i++) {
            int idx = (start + i) % LIDAR_BUFFER_SIZE;  // 环形取模
            float d = buffer[idx].distance;

            if (d >= MIN_VALID_DISTANCE && d <= MAX_VALID_DISTANCE) {
                sum_dist += d;
                count++;
            }
        }

        if (count == 0) continue;

        float avg_dist = sum_dist / count;

        // 评分函数：距离越近 + 点越多 → 分数越低
        // 可调权重：距离优先 or 密度优先
        float score = avg_dist * 1.0f + (10000.0f / (count + 1));  // 平衡距离和点数

        if (score < best_score) {
            best_score = score;
            best_start_idx = start;
            best_avg_dist = avg_dist;
            best_count = count;
        }
    }

    // 未找到有效区域
    if (best_start_idx == -1) {
        result->valid = 0;
        return 0;
    }

    // 计算中间角度
    int mid_idx = (best_start_idx + window_points / 2) % LIDAR_BUFFER_SIZE;
    result->center_angle = mid_idx * ANGLE_RESOLUTION;
	
	//角度归一化
	while(result->center_angle>=180.0f) result->center_angle-=360.0f;
	while(result->center_angle<-180.0f) result->center_angle+=360.0f;

	//大于30度的突变,进行滤波处理
	if( fabs(result->center_angle-LastAngle)>30.0f )
	{
		filterCnt++;
		if( filterCnt >= 8 )
		{
			//突变保持500ms,则确认为非干扰
			filterCnt=0;//更新本轮目标
		}
		else
		{
			result->center_angle = LastAngle;
		}
	}
	else
	{
		filterCnt=0;
	}
	
	LastAngle = result->center_angle;
	
    result->avg_distance = best_avg_dist;
    result->point_count  = best_count;
    result->valid        = 1;

    return 1;
}


//必要的外部对象
void avoid_targetpos_lc307(uint8_t flag);
extern EventGroupHandle_t g_xEventFlyAction;

void CoordinateHandleTask(void* param)
{
	extern APPShowType_t appshow;
	while(1)
	{
		//等待队列数据
		BaseType_t queuestate = xQueueReceive(g_xQueueLidarBuffer,&LidarRaw,portMAX_DELAY);
		
		if( queuestate == pdPASS )
		{
			//获取飞行器相关的事件组
			EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);

			
			//跟随模式开启,搜索离雷达最近的距离以及角度
			if( uxBits & lidar_follow_mode )
			{
				find_nearest_dense_region(LidarRaw.buffer,&LidarFollowRegion);
			}
			
			//避障模式开启,规避悬停状态下的靠近障碍物
			else if( uxBits & lidar_avoid_mode )
			{
				//前
				if( detect_obstacle(LidarRaw.buffer,330,30,550) )
				{
					avoid_targetpos_lc307(3);
				}
				
				//后
				if( detect_obstacle(LidarRaw.buffer,150,210,550) )
				{
					avoid_targetpos_lc307(2);
				}
				
				//左
				if( detect_obstacle(LidarRaw.buffer,240,300,550) )
				{
					avoid_targetpos_lc307(0);
				}
				
				//右
				if( detect_obstacle(LidarRaw.buffer,60,120,550) )
				{
					avoid_targetpos_lc307(1);
				}
			}
		}
	}
}


