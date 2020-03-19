#include "nxpQueue.h"
#include "string.h"


//创建队列 queueCapacity-队列容量
status initQueue(queue *PQueue,int queueCapacity)
{
    //给数组指针分配内存
    PQueue->pBase = (ElemType *)malloc(sizeof(ElemType)*queueCapacity);
    if(!PQueue->pBase)
    {
        return ERROR;
    }

    PQueue->front = 0; //最开始创建时，队头索引为0
    PQueue->rear = 0; //最开始创建时，队尾索引为0
    PQueue->maxSize = queueCapacity;

    return OK;
}

//销毁队列
void destroyQueue(queue *PQueue)
{
    free(PQueue);  //释放队列数组指针指向的内存
    PQueue = NULL;    //队列数组指针重新指向NULL,避免成为野指针
}

//清空队列
void clearQueue(queue *PQueue)
{
    PQueue->front = 0; //队头索引清0
    PQueue->rear = 0; //队尾索引清0
}

//判断队列是否为空
status isEmpityQueue(queue *PQueue)
{
    if( PQueue->front == PQueue->rear )  //队头==队尾，说明为空
        return TRUE;

    return FALSE;
}

/*
 *在循环队列中,“队满”和“队空”的条件有可能是相同的，都是front==rear，
 *这种情况下，无法区别是“队满”还是“队空”。
 *针对这个问题，有3种可能的处理方法：
 *（1）另设一个标志以区别是“队满”还是“队空”。（即入队/出队前检查是否“队满”/“队空”）
 *（2）设一个计数器，此时甚至还可以省去一个指针。
 *（3）少用一个元素空间，即约定队头指针在队尾指针的下一位置时就作为“队满”的标志，
 *即“队满”条件为：(PQueue->rear+1)%MAX_SIZE == PQueue->front。
 *  【这里采用了第3种处理方法】
 */
//判断队列是否为满
status isFullQueue(queue *PQueue)
{
    if( (PQueue->rear+1)%PQueue->maxSize == PQueue->front )  //队列满
        return TRUE;

    return FALSE;
}

//获得队列长度
int getQueueLen(queue *PQueue)
{
    //正常情况下，队列长度为队尾队头指针之差，但如果首尾指针跨容量最大值时，要%
    return (PQueue->rear - PQueue->front + PQueue->maxSize)%PQueue->maxSize;
}

//新元素入队 [先进先出原则：在队尾的位置插入] element-要插入元素
status enQueue(queue *PQueue,ElemType element)
{
#if 1
    if(isFullQueue(PQueue)==TRUE)
    {
//        printf("队列已满,不能再插入元素了,先出一个再进!\n");
//        return FALSE;
        ElemType elemTemp;
        deQueue(PQueue, &elemTemp);
    }

    //向队列中添加新元素
//    PQueue->pBase[PQueue->rear] = element;
    memcpy(&PQueue->pBase[PQueue->rear], &element, sizeof(ElemType));
    PQueue->rear = (PQueue->rear+1) % PQueue->maxSize; //将rear赋予新的合适的值
#endif
    return TRUE;
}

//新元素出队,同时保存出队的元素 [先进先出原则：在队头的位置删除]
status deQueue(queue *PQueue,ElemType *pElement)
{
    //如果队列为空,则返回false
    if(isEmpityQueue(PQueue)==TRUE)
    {
//        printf("队列为空，出队失败!\n");
        return FALSE;
    }

//    *pElement = PQueue->pBase[PQueue->front];       //先进先出
    memcpy(pElement, &PQueue->pBase[PQueue->front],sizeof(ElemType));
    PQueue->front = (PQueue->front+1) % PQueue->maxSize; //移到下一位置

    return TRUE;
}

//遍历队列
void queueTraverse(queue *PQueue)
{
    int i = PQueue->front;           //从头开始遍历
    while(i != PQueue->rear)     //如果没有到达rear位置，就循环
    {
//        printf("%d  ", PQueue->pBase[i]);
        i = (i+1) % PQueue->maxSize;              //移到下一位置
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
