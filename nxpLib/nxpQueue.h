#ifndef NXPLIB_NXPQUEUE_H
#define NXPLIB_NXPQUEUE_H

#include <stdio.h>
#include <stdlib.h>

/* ����: ֻ�����ڱ��һ��(��βrear)���в��������������һ��(��ͷfront)����ɾ�����������Ա�
 * ����������Ϊ���  ɾ���������Ϊ����   ���о����Ƚ��ȳ����ص�
 */

/*=====���е���ӡ�����ʾ��ͼ========
 *
 *  ���� ----------------- ���
 *   <--- a1,a2,a3,...,an <---
 *      -----------------
 *
 *================================*/

typedef enum
{
    OK=0, //��ȷ
    ERROR=1,   //����
    TRUE=2,  //Ϊ��
    FALSE=3   //Ϊ��
}status;

typedef struct {
	//unsigned int canbusId;
	//unsigned char canbusData[8];
	unsigned char data;
}ElemType;   //�궨����е���������

#define MAX_SIZE 200

/*һ��ʹ������洢���еĳ�Ϊ��̬˳�����
 *����ʹ�ö�̬�����ָ��ĳ�Ϊ��̬˳�����*/
// ��������Ƕ�̬˳����С�
typedef struct QUEUE
{
    ElemType *pBase;    //����ָ��
    int front;      //��ͷ����
    int rear;       //��β����
    int maxSize;    //��ǰ������������
}queue;

//�����ն��� queueCapacity-��������
status initQueue(queue *PQueue,int queueCapacity);
//���ٶ���
void destroyQueue(queue *PQueue);
//��ն���
void clearQueue(queue *PQueue);
//�ж϶����Ƿ�Ϊ��
status isEmpityQueue(queue *PQueue);
//�ж϶����Ƿ�Ϊ��
status isFullQueue(queue *PQueue);
//��ö��г���
int getQueueLen(queue *PQueue);
//��Ԫ����� [�Ƚ��ȳ�ԭ���ڶ�β��λ�ò���] element-Ҫ����Ԫ��
status enQueue(queue *PQueue,ElemType element);
//��Ԫ�س���,ͬʱ������ӵ�Ԫ�� [�Ƚ��ȳ�ԭ���ڶ�ͷ��λ��ɾ��]
status deQueue(queue *PQueue,ElemType *pElement);
//��������
void queueTraverse(queue *PQueue);


//--------------------------------------------------------------------------
typedef struct {
	unsigned int canbusId;
	unsigned char canbusData[8];
}ElemTypeCan;   //??????????


typedef struct QUEUECAN
{
	ElemTypeCan *pBase;    
    int front;      
    int rear;       
    int maxSize;    
}queueCan;

status initQueueCan(queueCan *PQueue,int queueCapacity);

void destroyQueueCan(queueCan *PQueue);

void clearQueueCan(queueCan *PQueue);

status isEmpityQueueCan(queueCan *PQueue);

status isFullQueueCan(queueCan *PQueue);

int getQueueLenCan(queueCan *PQueue);

status enQueueCan(queueCan *PQueue,ElemTypeCan element);

status deQueueCan(queueCan *PQueue,ElemTypeCan *pElement);

void queueTraverseCan(queueCan *PQueue);
#endif

