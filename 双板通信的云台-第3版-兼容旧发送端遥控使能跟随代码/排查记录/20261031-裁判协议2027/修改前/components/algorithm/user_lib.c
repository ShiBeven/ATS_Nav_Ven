#include "user_lib.h"
#include "arm_math.h"

fp32 invSqrt(fp32 num)
{
    fp32 halfnum = 0.5f * num;
    fp32 y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(fp32 *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}
void ramp_calc_min(ramp_function_source_t *ramp_source_type, fp32 input)
{
    ramp_source_type->input = input;
    ramp_source_type->out -= ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}

void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

void abs_limit(fp32 *num, fp32 Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

fp32 sign(fp32 value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

void VAL_LIMIT(float *Value, float minValue, float maxValue)
{
    if (*Value < minValue)
        *Value = minValue;
    else if (*Value > maxValue)
        *Value = maxValue;
    else
        *Value = *Value;
}

int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

fp32 theta_format(fp32 Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}

void InitQueue(Queue *Q)
{
    Q->base = (ElemType *)malloc(sizeof(ElemType) * MAXSIZE);
    // assert(Q->base != NULL);
    Q->front = Q->rear = 0;
}

// 入队操作

void EnQueue(Queue *Q, ElemType x)

{
    if (((Q->rear + 1) % MAXSIZE) == Q->front)

        return;
    Q->base[Q->rear] = x;

    // 更改尾指针的指向
    Q->rear = (Q->rear + 1) % MAXSIZE;
}

// 出队操作
void DeQueue(Queue *Q)

{
    if (Q->front == Q->rear)
        return;
    Q->front = (Q->front + 1) % MAXSIZE;
}

// 获取队头元素

void GetHdad(Queue *Q, ElemType *v)
{
    if (Q->front == Q->rear)

        return;

    // 如果队列不为空，获取队头元素
    *v = Q->base[Q->front];
}

// 获取队列长度（元素个数）
int Length(Queue *Q)
{
    int len = Q->rear - Q->front;
    len = (len > 0) ? len : MAXSIZE + len;

    return len;
}

// 清空队列

void ClearQueue(Queue *Q)

{
    Q->front = Q->rear = 0;
}

void DestroyQueue(Queue *Q)

{
    free(Q->base);
    Q->base = NULL;
}


/*数组实现队列*/

void Queueinit(struct queue *q)
{
	q->front = q->tail = 0;
	q->empty = 1;
}

void enqueue(struct queue *q, float value)
{
	if (!q->empty && q->front == q->tail) {
//		return 0;
	}
	q->empty = 0;
	q->data[q->tail] = value;
	q->tail = (q->tail + 1) % QUEUE_SIZE;
}

int dequeue(struct queue *q)
{
	if (q->empty)
		return -1;
	int value = q->data[q->front];
	q->front = (q->front + 1) % QUEUE_SIZE;
	if (q->front == q->tail)
		q->empty = 1;
	return value;
}


//QueueObj PitchAngle=
//{
//    .nowLength =0,
//    .queueLength = 100,
//    .queue = {0},
//    .queueTotal = 0,
//    .full_flag=0
//};
void  LoopQueuePitch(uint8_t queue_len,float Data)
{
//    if(queue_len>PitchAngle.queueLength)
//        queue_len=PitchAngle.queueLength;


//    if(PitchAngle.nowLength<queue_len)
//    {
//        PitchAngle.queue[PitchAngle.nowLength] = Data;
//        PitchAngle.nowLength++;
//    }
//    else
//    {
//        for(uint16_t i=0; i<queue_len-1; i++)
//        {
//            PitchAngle.queue[i] = PitchAngle.queue[i+1];
//            //更新队列
//        }
//        PitchAngle.queue[queue_len-1] = Data;
//    }

}


//QueueObj YawAngle=
//{
//    .nowLength =0,
//    .queueLength = 100,
//    .queue = {0},
//    .queueTotal = 0,
//    .full_flag=0
//};

void  LoopQueueYaw(uint8_t queue_len,float Data)
{
//    if(queue_len>YawAngle.queueLength)
//        queue_len=YawAngle.queueLength;


//    if(YawAngle.nowLength<queue_len)
//    {
//        YawAngle.queue[YawAngle.nowLength] = Data;
//        YawAngle.nowLength++;
//    }
//    else
//    {
//        for(uint16_t i=0; i<queue_len-1; i++)
//        {
//            YawAngle.queue[i] = YawAngle.queue[i+1];
//            //更新队列
//        }
//        YawAngle.queue[queue_len-1] = Data;
//    }

}

void Clear_Queue(QueueObj* queue)
{
    for(uint16_t i=0; i<queue->queueLength; i++)
    {
        queue->queue[i]=0;
    }
    queue->nowLength = 0;
    queue->queueTotal = 0;
    queue->full_flag=0;
}


