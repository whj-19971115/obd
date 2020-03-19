#include "nxpQueue.h"
#include "string.h"


//�������� queueCapacity-��������
status initQueue(queue *PQueue,int queueCapacity)
{
    //������ָ������ڴ�
    PQueue->pBase = (ElemType *)malloc(sizeof(ElemType)*queueCapacity);
    if(!PQueue->pBase)
    {
        return ERROR;
    }

    PQueue->front = 0; //�ʼ����ʱ����ͷ����Ϊ0
    PQueue->rear = 0; //�ʼ����ʱ����β����Ϊ0
    PQueue->maxSize = queueCapacity;

    return OK;
}

//���ٶ���
void destroyQueue(queue *PQueue)
{
    free(PQueue);  //�ͷŶ�������ָ��ָ����ڴ�
    PQueue = NULL;    //��������ָ������ָ��NULL,�����ΪҰָ��
}

//��ն���
void clearQueue(queue *PQueue)
{
    PQueue->front = 0; //��ͷ������0
    PQueue->rear = 0; //��β������0
}

//�ж϶����Ƿ�Ϊ��
status isEmpityQueue(queue *PQueue)
{
    if( PQueue->front == PQueue->rear )  //��ͷ==��β��˵��Ϊ��
        return TRUE;

    return FALSE;
}

/*
 *��ѭ��������,���������͡��ӿա��������п�������ͬ�ģ�����front==rear��
 *��������£��޷������ǡ����������ǡ��ӿա���
 *���������⣬��3�ֿ��ܵĴ�������
 *��1������һ����־�������ǡ����������ǡ��ӿա����������/����ǰ����Ƿ񡰶�����/���ӿա���
 *��2����һ������������ʱ����������ʡȥһ��ָ�롣
 *��3������һ��Ԫ�ؿռ䣬��Լ����ͷָ���ڶ�βָ�����һλ��ʱ����Ϊ���������ı�־��
 *��������������Ϊ��(PQueue->rear+1)%MAX_SIZE == PQueue->front��
 *  ����������˵�3�ִ�������
 */
//�ж϶����Ƿ�Ϊ��
status isFullQueue(queue *PQueue)
{
    if( (PQueue->rear+1)%PQueue->maxSize == PQueue->front )  //������
        return TRUE;

    return FALSE;
}

//��ö��г���
int getQueueLen(queue *PQueue)
{
    //��������£����г���Ϊ��β��ͷָ��֮��������βָ����������ֵʱ��Ҫ%
    return (PQueue->rear - PQueue->front + PQueue->maxSize)%PQueue->maxSize;
}

//��Ԫ����� [�Ƚ��ȳ�ԭ���ڶ�β��λ�ò���] element-Ҫ����Ԫ��
status enQueue(queue *PQueue,ElemType element)
{
#if 1
    if(isFullQueue(PQueue)==TRUE)
    {
//        printf("��������,�����ٲ���Ԫ����,�ȳ�һ���ٽ�!\n");
//        return FALSE;
        ElemType elemTemp;
        deQueue(PQueue, &elemTemp);
    }

    //������������Ԫ��
//    PQueue->pBase[PQueue->rear] = element;
    memcpy(&PQueue->pBase[PQueue->rear], &element, sizeof(ElemType));
    PQueue->rear = (PQueue->rear+1) % PQueue->maxSize; //��rear�����µĺ��ʵ�ֵ
#endif
    return TRUE;
}

//��Ԫ�س���,ͬʱ������ӵ�Ԫ�� [�Ƚ��ȳ�ԭ���ڶ�ͷ��λ��ɾ��]
status deQueue(queue *PQueue,ElemType *pElement)
{
    //�������Ϊ��,�򷵻�false
    if(isEmpityQueue(PQueue)==TRUE)
    {
//        printf("����Ϊ�գ�����ʧ��!\n");
        return FALSE;
    }

//    *pElement = PQueue->pBase[PQueue->front];       //�Ƚ��ȳ�
    memcpy(pElement, &PQueue->pBase[PQueue->front],sizeof(ElemType));
    PQueue->front = (PQueue->front+1) % PQueue->maxSize; //�Ƶ���һλ��

    return TRUE;
}

//��������
void queueTraverse(queue *PQueue)
{
    int i = PQueue->front;           //��ͷ��ʼ����
    while(i != PQueue->rear)     //���û�е���rearλ�ã���ѭ��
    {
//        printf("%d  ", PQueue->pBase[i]);
        i = (i+1) % PQueue->maxSize;              //�Ƶ���һλ��
    }
    printf("\n");
}


//---------------------------------------------------------------------
//???? queueCapacity-????
status initQueueCan(queueCan *PQueue,int queueCapacity)
{
    //?????????
    PQueue->pBase = (ElemTypeCan *)malloc(sizeof(ElemTypeCan)*queueCapacity);
    if(!PQueue->pBase)
    {
        return ERROR;
    }

    PQueue->front = 0; //??????,?????0
    PQueue->rear = 0; //??????,?????0
    PQueue->maxSize = queueCapacity;

    return OK;
}

//????
void destroyQueueCan(queueCan *PQueue)
{
    free(PQueue);  //?????????????
    PQueue = NULL;    //??????????NULL,???????
}

//????
void clearQueueCan(queueCan *PQueue)
{
    PQueue->front = 0; //?????0
    PQueue->rear = 0; //?????0
}

//????????
status isEmpityQueueCan(queueCan *PQueue)
{
    if( PQueue->front == PQueue->rear )  //??==??,????
        return TRUE;

    return FALSE;
}


status isFullQueueCan(queueCan *PQueue)
{
    if( (PQueue->rear+1)%PQueue->maxSize == PQueue->front )  //???
        return TRUE;

    return FALSE;
}

//??????
int getQueueLenCan(queueCan *PQueue)
{
    //?????,?????????????,??????????????,?%
    return (PQueue->rear - PQueue->front + PQueue->maxSize)%PQueue->maxSize;
}

//????? [??????:????????] element-?????
status enQueueCan(queueCan *PQueue,ElemTypeCan element)
{
#if 1
    if(isFullQueueCan(PQueue)==TRUE)
    {
//        printf("????,????????,??????!\n");
//        return FALSE;
        ElemTypeCan elemTemp;
        deQueueCan(PQueue, &elemTemp);
    }

    //?????????
//    PQueue->pBase[PQueue->rear] = element;
    memcpy(&PQueue->pBase[PQueue->rear], &element, sizeof(ElemType));
    PQueue->rear = (PQueue->rear+1) % PQueue->maxSize; //?rear????????
#endif
    return TRUE;
}

//?????,????????? [??????:????????]
status deQueueCan(queueCan *PQueue,ElemTypeCan *pElement)
{
    //??????,???false
    if(isEmpityQueueCan(PQueue)==TRUE)
    {
//        printf("????,????!\n");
        return FALSE;
    }

//    *pElement = PQueue->pBase[PQueue->front];       //????
    memcpy(pElement, &PQueue->pBase[PQueue->front],sizeof(ElemType));
    PQueue->front = (PQueue->front+1) % PQueue->maxSize; //??????

    return TRUE;
}

//????
void queueTraverseCan(queueCan *PQueue)
{
    int i = PQueue->front;           //??????
    while(i != PQueue->rear)     //??????rear??,???
    {
//        printf("%d  ", PQueue->pBase[i]);
        i = (i+1) % PQueue->maxSize;              //??????
    }
    printf("\n");
}
