#include "KD_Tree.h"
#include <string.h>

#include "bsp_RTOSdebug.h"

Node nodes[MAX_NODES]; // 静态分配节点数组
Node last_nodes[MAX_NODES]; // 静态分配节点数组
int nextAvailableNodeIndex = 0; // 跟踪下一个可用节点的索引

// 用于获取下一个可用节点的索引
int getNextAvailableNodeIndex(void) {
	return nextAvailableNodeIndex++; // 分配后指针+1
}

void resetNodes(void) {
	memset(nodes, 0, sizeof(Node) * MAX_NODES);
	memset(last_nodes, 0, sizeof(Node) * MAX_NODES);
    nextAvailableNodeIndex = 0;
}

// 快速排序比较函数
int compare_x(const void* a, const void* b) {
    float diff = ((Point*)a)->x - ((Point*)b)->x;
    return (diff > 0) - (diff < 0); // 避免浮点精度问题
}

int compare_y(const void* a, const void* b) {
    float diff = ((Point*)a)->y - ((Point*)b)->y;
    return (diff > 0) - (diff < 0);
}

// 使用快速排序替代插入排序
void insertionSort(Point points[], int split, int start, int end) {
    if (split == 0) {
        qsort(&points[start], end - start + 1, sizeof(Point), compare_x);
    } else {
        qsort(&points[start], end - start + 1, sizeof(Point), compare_y);
    }
}

Point* findMedian(Point points[], int split, int start, int end) // 查找中位数.
{
    int n = end - start + 1;
    Point* address = 0;

    insertionSort(points, split, start, end);
    if (n % 2 != 0)
    {
        address = &points[start + (n-1)/2];
    }
    else
    {
        address = &points[start + n/2];
    }

    return address;
}

Node* buildKDTree(Point points[], int depth, int start, int end) // 创建KD-Tree.
{
    if (start <= end)
    {
        int nodeIndex = getNextAvailableNodeIndex(); // 获取下一个可用节点的索引
        Node* node = &nodes[nodeIndex];
        int split = depth % 2;
        node -> point = *findMedian(points, split, start, end);

        int temp = end;
        int n = end - start + 1;
         // 左子树.
        if (n % 2 != 0)
        {
            end = start + (n-1)/2 - 1;
        }
        else
        {
            end = start + n/2 - 1;
        }
        node -> left = buildKDTree(points, depth+1, start, end);
         // 右子树.
        if (n % 2 != 0)
        {
            start = start + (n-1)/2 + 1;
        }
        else
        {
            start = start + n/2 + 1;
        }
        end = temp;
        node -> right = buildKDTree(points, depth+1, start, end);

        return node;
    }
    else
    {
        return NULL;
    }
}


void findMinDistance (Node* node, Point point, int depth, float d_min[]) // 查找最短距离.
{
    while (node != NULL)
    {
        float d_x = point.x - node -> point.x;
        float d_y = point.y - node -> point.y;
        float d = fabs(d_x) + fabs(d_y); // 计算曼哈顿距离.
        if (d <= d_min[0])
        {
			d_min[0] = d;
            d_min[1] = d_x;
            d_min[2] = d_y;	
        }
        if (depth % 2 != 0)
        {
            if (point.y <= node -> point.y)
            {
                node = node -> left;
            }
            else
            {
                node = node -> right;
            }
            depth += 1;
        }
        else
        {
            if (point.x <= node -> point.x)
            {
                node = node -> left;
            }
            else
            {
                node = node -> right;
            }
            depth += 1;
        }
    }
}

Node* root; // 上一时刻点云数据建立的树的根节点
Point points[MAX_NODES]; // 当前时刻的点云数据
Point points_copy[MAX_NODES]; // 中间变量
Point temp[MAX_NODES] = {0};  // 中间变量

#define PI 3.1415626f

static void points_reset(void)
{
	resetNodes();
	root = NULL;
	memset(points,0,sizeof(Point)*MAX_NODES);
	memset(points_copy,0,sizeof(Point)*MAX_NODES);
	memset(temp,0,sizeof(Point)*MAX_NODES);
}

// 三角函数查找表,分辨率0.3度,总长 360/0.3 = 1200
static float sin_table[1200] = { 0 };
static float cos_table[1200] = { 0 };

// 初始化三角函数表
void init_trig_table(void) {
    for (int i = 0; i < 1200; i++) {
        sin_table[i] = sin(i * 0.3f * PI / 180.0f);
        cos_table[i] = cos(i * 0.3f * PI / 180.0f);
    }
}

float sin_tab(float rad)
{
	float angle = rad * 180.0f / PI;
	int angle_idx = 0;
	angle_idx = (int)( angle / 0.3f);
	angle_idx = (angle_idx+1200)%1200;   
	return sin_table[angle_idx];
}

float cos_tab(float rad)
{
	float angle = rad * 180.0f / PI;
	int angle_idx = 0;
	angle_idx = (int)( angle / 0.3f);
	angle_idx = (angle_idx+1200)%1200;   
	return cos_table[angle_idx];
}

float sinAngle_tab(float angle)
{
	int angle_idx = 0;
	angle_idx = (int)( angle / 0.3f);
	angle_idx = (angle_idx+1200)%1200;   
	return sin_table[angle_idx];
}

float cosAngle_tab(float angle)
{
	int angle_idx = 0;
	angle_idx = (int)( angle / 0.3f);
	angle_idx = (angle_idx+1200)%1200;   
	return cos_table[angle_idx];
}

//精度测试
//void test_trig(void)
//{
//	int angle_idx = roundf(50.0f / 0.3f);
//	angle_idx = (angle_idx+1200)%1200;                   //正负数处理
//	
//	printf("sin50:%.3f\t%.3f\r\n",sin_table[angle_idx],sin(50*(PI/180.0f)));
//	printf("cos50:%.3f\t%.3f\r\n",cos_table[angle_idx],cos(50*(PI/180.0f)));

//	angle_idx = (int)( -70 / 0.3f);
//	angle_idx = (angle_idx+1200)%1200;                   //正负数处理
//	printf("sin-70:%.3f\t%.3f\r\n",sin_table[angle_idx],sin(-70*(PI/180.0f)));
//	printf("cos-70:%.3f\t%.3f\r\n",cos_table[angle_idx],cos(-70*(PI/180.0f)));
//}


//static pRtosDebugInterface_t freq_debug = &RTOSTaskDebug;
//static RtosDebugPrivateVar debugpriv = { 0 };

extern float get_imuYaw(void);

void Positioning(Stl06NPointStructDef* PointCloud, int length, float resolution, int* result,uint8_t startFlag)
{
	static int cnt = 0; // 计数器
	float x_bias = 0; // x方向上的累计偏移量
	float y_bias = 0; // y方向上的累计偏移量
	float max_error = 10000; // 偏移量阈值
	static float x_error = 0; // x方向上的偏移量
	static float y_error = 0; // y方向上的偏移量
	static int x = 0, y = 0; // 位置的估计
	static float x1_mid = 0; // 模板点云数据的质心坐标（x方向）
	static float y1_mid = 0; // 模板点云数据的质心坐标（y方向）
	static float x2_mid = 0; // 目标点云数据的质心坐标（x方向）
	static float y2_mid = 0; // 目标点云数据的质心坐标（y方向）
	int iter = 0; // 迭代次数
	int real_cnt = 0; // 计数器

	//未开始
	if(startFlag == 0)
	{
		cnt = 0;
		x_error = y_error = 0;
		x1_mid = y1_mid = x2_mid = y2_mid = 0;
		x = y = 0;
		result[0] = 0;
		result[1] = 0;
		result[2] = 0;
		max_error = 10000;
		x_bias = y_bias = 0;
		points_reset();
		return;
	}

	for (int i = 0; i < length; i++)
	{
		if (PointCloud[i].distance > 50)
		{
			int angle_idx = roundf((i*resolution) / 0.3f); // 查表法获取三角函数值,节省时间
			angle_idx = (angle_idx + 1200) % 1200; // 正负数处理
			points[real_cnt].x = PointCloud[i].distance*sin_table[angle_idx];
			points[real_cnt].y = PointCloud[i].distance*cos_table[angle_idx];
			real_cnt += 1;
		}
	} // 滤除距离太近（小于5cm）和距离太远（超过扫描距离，数据为0）的点

	if (cnt < 3)
	{
		x1_mid = 0;
		y1_mid = 0;
		for (int j = 0; j < real_cnt; j++) x1_mid += points[j].x, y1_mid += points[j].y;
		x1_mid /= real_cnt; // 计算模板点云数据的质心坐标（x方向）
		y1_mid /= real_cnt; // 计算模板点云数据的质心坐标（y方向）
		for (int j = 0; j < real_cnt; j++) points[j].x -= x1_mid, points[j].y -= y1_mid; // 去中心化
		resetNodes(); // 释放内存
		root = buildKDTree(points, 0, 0, real_cnt-1); // 将模板数据以KD-Tree的形式存储
	} // 为了保证数据的稳定性，前 2 次数据放弃，第 3 次数据作为初始模板数据，第 4 次数据开始处理。
	else
	{
		x2_mid = 0;
		y2_mid = 0;
		for (int j = 0; j < real_cnt; j++) x2_mid += points[j].x, y2_mid += points[j].y;
		x2_mid /= real_cnt; // 计算目标点云数据的质心坐标（x方向）
		y2_mid /= real_cnt; // 计算目标点云数据的质心坐标（y方向）
		for (int j = 0; j < real_cnt; j++) points[j].x -= x2_mid, points[j].y -= y2_mid; // 去中心化
		for (int j = 0; j < real_cnt; j++) points_copy[j].x = points[j].x, points_copy[j].y = points[j].y; // 复制一份目标点云数据

		x_bias = x1_mid - x2_mid; // 初始化x方向上的累计偏移量
		y_bias = y1_mid - y2_mid; // 初始化y方向上的累计偏移量

		while (max_error > 0.1f) // 要求最大偏移量小于阈值
		{
			iter += 1; // 累计迭代次数
			x_error = 0; // x方向上的偏移量初始化
			y_error = 0; // y方向上的偏移量初始化
			for (int i = 0; i < real_cnt; i++)
			{
				float d_min[] = {3e3, 0, 0}; // 初始化
				findMinDistance(root, points[i], 0, d_min); // 查找每个点与模板点云的最短距离
				temp[i].x = d_min[1]; // 存储x方向的偏移量
				temp[i].y = d_min[2]; // 存储y方向的偏移量
			}
			insertionSort(temp, 0, 0, real_cnt-1); // 插入排序
			x_error = temp[(int)(real_cnt/2)].x; // 用中位数作为整体的偏移量
			insertionSort(temp, 1, 0, real_cnt-1); // 插入排序
			y_error = temp[(int)(real_cnt/2)].y; // 用中位数作为整体的偏移量
			max_error = fabs(x_error) + fabs(y_error); // 计算最大累计偏移量（定义最大偏移量为两个方向偏移量的绝对值之和）
			for (int j = 0; j < real_cnt; j++) points[j].x -= x_error, points[j].y -= y_error; 
			x_bias -= x_error;
			y_bias -= y_error;
			if(iter > 50)
			{
				x_bias = y_bias = 0;
				break;
			} // 设定最大迭代次数为100次，防止陷入死循环无法跳出
		}
	}

	if(fabs(x_bias) > 50 || fabs(y_bias) > 50) // 当任意一个方向上的累计偏移量达到阈值的时候，记录当前位置，更新模板点云数据
	{
		x1_mid = x2_mid;
		y1_mid = y2_mid;
		x += (int)x_bias; // 将当前位置作为新的坐标原点
		y += (int)y_bias; // 将当前位置作为新的坐标原点
		x_bias=0;
		y_bias=0;
		resetNodes(); // 初始化静态内存
		root = buildKDTree(points_copy, 0, 0, real_cnt-1); // 更新模板点云数据
	}

	if(cnt < 5) cnt++;

	// 输出结果
	result[0] = x + x_bias; // 位置坐标（x方向）
	result[1] = y + y_bias; // 位置坐标（y方向）
	result[2] = iter; // 迭代次数
}

