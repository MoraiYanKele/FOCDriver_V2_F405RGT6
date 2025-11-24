#include "RingBuffer.h"
#include <string.h>
#include <assert.h>

/**
 * @brief  判断一个数是否为2的幂
 * @param  size: 要判断的数字
 * @retval 1: 是2的幂
 * @retval 0: 不是2的幂
 */
static inline uint8_t IsPowerOfTwo(uint32_t size) 
{
  return (size != 0) && ((size & (size - 1)) == 0);
}

/**
 * @brief  将一个数向上取整到最接近的2的幂
 * @note   这个函数提供给用户，用于在分配缓冲区之前计算出最优的缓冲区大小。
 *         例如: RoundUpToPowerOfTwo(1000) 会返回 1024。
 * @param  size: 原始大小
 * @retval 向上取整后的大小
 */
uint32_t RoundUpToPowerOfTwo(uint32_t size) 
{
  if (size == 0) return 1; // 至少为1
  if (IsPowerOfTwo(size)) return size; // 如果已经是2的幂，直接返回
  size--;
  size |= size >> 1;
  size |= size >> 2;
  size |= size >> 4;
  size |= size >> 8;
  size |= size >> 16;
  return size + 1;
}

/**
 * @brief  初始化环形缓冲区
 * @note   为了达到最高的运行效率，缓冲区的size必须是2的幂。
 *         如果size不是2的幂，初始化将失败。
 *         请在外部使用 RoundUpToPowerOfTwo() 来计算并分配一个合适大小的缓冲区。
 * @param  ringBuffer: 环形缓冲区对象指针
 * @param  buffer: 外部提供的缓冲区内存指针
 * @param  size: 缓冲区的实际大小，必须为2的幂
 * @retval None
 */
void RingBuffer_Init(RingBuffer_t *ringBuffer, uint8_t *buffer, uint32_t size)
{
  // 检查：确保传入的size是2的幂，这是性能优化的关键。
  if (!IsPowerOfTwo(size))
  {
    // 如果不是，初始化失败。将size置0来标记这是一个无效的缓冲区。
    ringBuffer->buffer = buffer; // 仍然指向它，但size为0
    ringBuffer->size = 0;
    ringBuffer->readIndex = 0;
    ringBuffer->writeIndex  = 0;
    return;
  }

  ringBuffer->buffer = buffer;
  ringBuffer->size = size;
  ringBuffer->readIndex = 0;
  ringBuffer->writeIndex  = 0;
}

/**
 * @brief  获取环形缓冲区中已存储的数据长度
 * @param  ringBuffer: 环形缓冲区对象指针
 * @retval 当前已存储的数据长度（字节）
 */
uint32_t RingBuffer_GetLength(const RingBuffer_t *ringBuffer)
{
  // (write - read) & (size - 1) 是一个经典算法，可以正确处理writeIndex回环绕过readIndex的情况
  return (ringBuffer->writeIndex - ringBuffer->readIndex) & (ringBuffer->size - 1);
}

/**
 * @brief  获取环形缓冲区剩余可写入的空间大小
 * @note   由于我们用一个字节来区分空/满状态，所以总容量会比size小1。
 * @param  ringBuffer: 环形缓冲区对象指针
 * @retval 剩余可写入的空间大小（字节）
 */
uint32_t RingBuffer_GetRemain(RingBuffer_t *ringBuffer)
{
  return (ringBuffer->size - RingBuffer_GetLength(ringBuffer) - 1);
}

/**
 * @brief  向环形缓冲区中写入数据
 * @param  ringBuffer: 环形缓冲区对象指针
 * @param  data: 需要写入的数据的指针
 * @param  length: 需要写入的数据长度
 * @retval 实际写入的数据长度。如果空间不足，可能为0。
 */
uint32_t RingBuffer_Write(RingBuffer_t *ringBuffer, const uint8_t *data, uint32_t length)
{
  if (RingBuffer_GetRemain(ringBuffer) < length)
  {
    return 0; // 缓冲区空间不足，无法写入, 返回0
  }

  // 计算从当前写指针到缓冲区末尾的连续空间大小
  uint32_t part1_len = ringBuffer->size - ringBuffer->writeIndex;

  if (length <= part1_len)
  {
    // 情况1: 数据可以一次性全部写入，不会回环
    memcpy(ringBuffer->buffer + ringBuffer->writeIndex, data, length);
  }
  else 
  {
    // 情况2: 数据需要分两次写入，需要回环
    // 先填满从 writeIndex 到末尾的空间
    memcpy(ringBuffer->buffer + ringBuffer->writeIndex, data, part1_len);
    // 再把剩余的数据从缓冲区的开头写入
    memcpy(ringBuffer->buffer, data + part1_len, length - part1_len);
  }

  // 统一更新 writeIndex，利用位运算实现回环
  ringBuffer->writeIndex = (ringBuffer->writeIndex + length) & (ringBuffer->size - 1);
  
  return length;
}

/**
 * @brief  从环形缓冲区中读取数据
 * @param  ringBuffer: 环形缓冲区对象指针
 * @param  data: 用于接收读取数据的缓冲区的指针
 * @param  length: 希望读取的数据长度
 * @retval 实际读取到的数据长度。如果数据不足，会小于length。
 */
uint32_t RingBuffer_Read(RingBuffer_t *ringBuffer, uint8_t *data, uint32_t length)
{
  uint32_t available_len = RingBuffer_GetLength(ringBuffer);
  if (available_len < length)
  {
    // 如果期望读取的长度超过了实际有的数据，则只读取实际有的数据
    length = available_len;
  }
  
  if (length == 0)
  {
    return 0;
  }

  // 计算从当前读指针到缓冲区末尾的连续数据长度
  uint32_t part1_len = ringBuffer->size - ringBuffer->readIndex;

  if (length <= part1_len)
  {
    // 情况1: 数据是连续的，可以一次性全部读取
    memcpy(data, ringBuffer->buffer + ringBuffer->readIndex, length);
  }
  else 
  {
    // 情况2: 数据是回环的，需要分两次读取
    // 先读取从 readIndex 到末尾的数据
    memcpy(data, ringBuffer->buffer + ringBuffer->readIndex, part1_len);
    // 再从缓冲区的开头读取剩余的数据
    memcpy(data + part1_len, ringBuffer->buffer, length - part1_len);
  }

  // 统一更新 readIndex，利用位运算实现回环
  ringBuffer->readIndex = (ringBuffer->readIndex + length) & (ringBuffer->size - 1);
  
  return length;
}



void RingBuffer_Clear(RingBuffer_t *ringBuffer)
{
  ringBuffer->readIndex = 0;
  ringBuffer->writeIndex = 0;
} 