/*
 * cqueue.h
 *
 *  Created on: 2022-1-4
 *      Author: Administrator
 */

#ifndef CQUEUE_H_
#define CQUEUE_H_
#include "system_init.h"
#include <stdbool.h>
#define NODE_DATAINFO_SIZE		20
#define NODE_DATA_SIZE			512
#define MAX_QUEUE				10
/*
 * 队列的结点结构，特为数据上报设定
 */
typedef struct Node{
	uint8_t data_info[NODE_DATAINFO_SIZE];
    uint8_t data[NODE_DATA_SIZE];
    struct Node *next;
} Node, *Queue;

//队列的结构，嵌套
typedef struct{
    Queue front;		// 头
    Queue rear;			// 尾
    uint8_t size;		// 个数
    uint8_t max_size;	// 最大个数
} LinkQueue;

void initQueue(LinkQueue *queue,uint8_t maxSize);
bool isEmpty(LinkQueue *queue);
void insertQueue(LinkQueue *queue, uint8_t *data_info,uint8_t *data);
void deleteQueue(LinkQueue *queue);
void traversal(LinkQueue queue);
void destoryQueue(LinkQueue *queue);

#endif /* CQUEUE_H_ */
