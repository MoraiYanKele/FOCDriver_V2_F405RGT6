#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

#include <stdint.h>
#include <string.h>

// 环形缓冲区结构体
typedef struct 
{
  uint8_t *buffer;    // 指向实际存储数据的内存区域
  uint32_t size;        // 缓冲区的总大小 (必须是2的幂，以便用位运算优化)
  volatile uint32_t readIndex;       // 读指针 (数据从这里读出)
  volatile uint32_t writeIndex;      // 写指针 (数据从这里写入)

} RingBuffer_t;


uint32_t RoundUpToPowerOfTwo(uint32_t size);
void RingBuffer_Init(RingBuffer_t *ringBuffer, uint8_t *buffer, uint32_t size);
uint32_t RingBuffer_Write(RingBuffer_t *ringBuffer, const uint8_t *data, uint32_t length);
uint32_t RingBuffer_Read(RingBuffer_t *ringBuffer, uint8_t *data, uint32_t length);
uint32_t RingBuffer_GetLength(const RingBuffer_t *ringBuffer);
uint32_t RingBuffer_GetRemain(RingBuffer_t *ringBuffer);
void RingBuffer_Clear(RingBuffer_t *ringBuffer);




#endif // __RINGBUFFER_H__