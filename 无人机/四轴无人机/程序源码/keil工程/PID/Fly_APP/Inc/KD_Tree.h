/**
 * ============================================================
 * KD_Tree.h — 2D KD-Tree + ICP点云配准定位算法头文件
 * ============================================================
 * 数据结构:
 *   Point: 二维点 (x, y)
 *   Node:  KD-Tree节点 (点+左右子树指针)
 *   MAX_NODES = 1200 (静态分配, 对应雷达一圈点数)
 *
 * 核心API:
 *   buildKDTree()   — 构建KD-Tree
 *   findMinDistance() — 查找最近邻 (曼哈顿距离)
 *   Positioning()   — ICP定位主函数
 *   init_trig_table() — 三角函数查表初始化 (分辨率0.3°)
 * ============================================================
 */

#ifndef __KD_TREE_H
#define __KD_TREE_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include "bsp_stl06n.h"

typedef struct point // 定义二维点的结构体.
{
	float x, y;
} Point;

typedef struct node // 定义节点的结构体.
{
	Point point;
	struct node* left;
	struct node* right;
} Node;


#define MAX_NODES 1200 // 最大节点数
extern Point points[MAX_NODES];
extern int nextAvailableNodeIndex;


void insertionSort(Point points[], int split, int start, int end); // 插入排序.

Point* findMedian(Point points[], int split, int start, int end); // 查找中位数.

Node* buildKDTree(Point points[], int depth, int start, int end); // 创建KD-Tree.

void preorder(Node* node); // 先序遍历.

void findMinDistance (Node* node, Point point, int depth, float d_min[]); // 查找最短距离.

void resetNodes(void); // 释放内存.

void Positioning(Stl06NPointStructDef* PointCloud, int length, float resolution, int* result,uint8_t startFlag);

void init_trig_table(void);

#endif 

