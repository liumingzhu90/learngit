/*
 * cqueue.c
 *
 *  Created on: 2022-1-4
 *      Author: Administrator
 */

#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
#include "cqueue.h"
//#include "malloc.h"
/*
 * 初始化队列
 * 参数：队列指针，队列最大容量
 * 说明：开始必然是空队列，队尾指针和队头指针都指向头结点
 */
Node node;
void initQueue(LinkQueue *queue,uint8_t maxSize)
{
    //初始化头结点
    queue->front = queue->rear = &node;

    if (NULL == queue->front) {
        return ;
    }
    queue->size = 0;
    queue->max_size = maxSize;
    queue->front->next = NULL;
}
/*
 * 判断队列是否为空
 */
bool isEmpty(LinkQueue *queue)
{
    if(queue->rear == queue->front){
    	queue->size = 0;
    	return true;
    }else{
    	return false;
    }
}
/*
 * 插入队列数据
 * 参数：队列指针，消息信息指针，消息内容指针
 * 说明：为了数据上报特设定的格式；
 * 	入队,只在一端入队，另一端出队，同样入队不需要判满
 */
Node nq[MAX_QUEUE+1];
void insertQueue(LinkQueue *queue, uint8_t *data_info,uint8_t *data)
{
//    Queue q = (Queue)malloc(sizeof(Node));
//
//    if (NULL == q) {
//        exit(0);
//    }
    if(queue->size >= queue->max_size){
    	queue->size = queue->max_size;
    	deleteQueue(queue);
    }

    //插入数据
	memset(nq[queue->size].data_info,0,sizeof(nq[queue->size].data_info));
	memset(nq[queue->size].data,0,sizeof(nq[queue->size].data));
    //插入数据
    memcpy(nq[queue->size].data_info,data_info,strlen(data_info)+2);
    memcpy(nq[queue->size].data,data,strlen(data)+2);
    nq[queue->size].next = NULL;

    //rear 总是指向队尾元素
    queue->rear->next = &nq[queue->size];
    queue->rear = &nq[queue->size];
    queue->size++;
}
/*
 * 删除队头数据，出队
 * 参数：队列指针
 * 说明：出队，需要判空
 */
void deleteQueue(LinkQueue *queue)
{
    Queue q = NULL;

    if (!isEmpty(queue)) {
        q = queue->front->next;
        queue->front->next = q->next;

        //这句很关键，不能丢
        if (queue->rear == q) {
            queue->rear = queue->front;
        }
        queue->size--;
        memset(q->data_info,0,sizeof(q->data_info));
        memset(q->data,0,sizeof(q->data));
//        free(q);
    }
}
/*
 * 遍历输出
 * 参数：队列变量
 */
void traversal(LinkQueue queue)
{
    uint16_t i = 1;
    Queue q = queue.front->next;

    while (q != NULL) {
        fprintf(USART1_STREAM,"队列第%d个元素是：info:%s\r\ndata:%s\r\n", i, q->data_info,q->data);
        q = q->next;
        i++;
    }
}

//销毁
/*
 * 销毁队列
 * 参数：队列指针
 */
void destoryQueue(LinkQueue *queue)
{
    while (queue->front != NULL) {
        queue->rear = queue->front->next;
//        free(queue->front);
        queue->front = queue->rear;
    }
    queue->size = 0;
    fprintf(USART1_STREAM,"销毁成功\r\n");
}

