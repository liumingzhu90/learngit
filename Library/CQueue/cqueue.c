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
 * ��ʼ������
 * ����������ָ�룬�����������
 * ˵������ʼ��Ȼ�ǿն��У���βָ��Ͷ�ͷָ�붼ָ��ͷ���
 */
Node node;
void initQueue(LinkQueue *queue,uint8_t maxSize)
{
    //��ʼ��ͷ���
    queue->front = queue->rear = &node;

    if (NULL == queue->front) {
        return ;
    }
    queue->size = 0;
    queue->max_size = maxSize;
    queue->front->next = NULL;
}
/*
 * �ж϶����Ƿ�Ϊ��
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
 * �����������
 * ����������ָ�룬��Ϣ��Ϣָ�룬��Ϣ����ָ��
 * ˵����Ϊ�������ϱ����趨�ĸ�ʽ��
 * 	���,ֻ��һ����ӣ���һ�˳��ӣ�ͬ����Ӳ���Ҫ����
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

    //��������
	memset(nq[queue->size].data_info,0,sizeof(nq[queue->size].data_info));
	memset(nq[queue->size].data,0,sizeof(nq[queue->size].data));
    //��������
    memcpy(nq[queue->size].data_info,data_info,strlen(data_info)+2);
    memcpy(nq[queue->size].data,data,strlen(data)+2);
    nq[queue->size].next = NULL;

    //rear ����ָ���βԪ��
    queue->rear->next = &nq[queue->size];
    queue->rear = &nq[queue->size];
    queue->size++;
}
/*
 * ɾ����ͷ���ݣ�����
 * ����������ָ��
 * ˵�������ӣ���Ҫ�п�
 */
void deleteQueue(LinkQueue *queue)
{
    Queue q = NULL;

    if (!isEmpty(queue)) {
        q = queue->front->next;
        queue->front->next = q->next;

        //���ܹؼ������ܶ�
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
 * �������
 * ���������б���
 */
void traversal(LinkQueue queue)
{
    uint16_t i = 1;
    Queue q = queue.front->next;

    while (q != NULL) {
        fprintf(USART1_STREAM,"���е�%d��Ԫ���ǣ�info:%s\r\ndata:%s\r\n", i, q->data_info,q->data);
        q = q->next;
        i++;
    }
}

//����
/*
 * ���ٶ���
 * ����������ָ��
 */
void destoryQueue(LinkQueue *queue)
{
    while (queue->front != NULL) {
        queue->rear = queue->front->next;
//        free(queue->front);
        queue->front = queue->rear;
    }
    queue->size = 0;
    fprintf(USART1_STREAM,"���ٳɹ�\r\n");
}

