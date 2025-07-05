#include "MT6701.h"
#include "stdio.h"

//////////////////////////////////////////////////////////////////////////////////

//驱动	  芯片：MT6701

//********************************************************************************

//////////////////////////////////////////////////////////////////////////////////

//本驱动程序使用的是MCU硬件SPI接口驱动



/********************************************************************/
//函数：u8 SPIx_ReadWriteByte(u8 TxData)
//函数功能： SPI读写函数
//SPI1 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
//*******************************************************************/

static float angle_data_prev = 0; //上次位置
static float full_rotation_offset = 0; //转过的整圈数

static uint8_t SPIx_ReadWriteByte(uint8_t TxData)
{

    uint8_t Rxdata;
    HAL_SPI_TransmitReceive(&STM32_SPI_PORT, &TxData, &Rxdata, 1, HAL_MAX_DELAY);
    return Rxdata;
}


static void MT6701_Read_RAW(uint8_t* pBuffer)
{
    uint16_t i;
    MT6701_CSN_CLR;//MT6701_CSN=0  //片选
    for(i = 0; i < 4; i++) {
        pBuffer[i] = SPIx_ReadWriteByte(0xFF); //循环读入字节数据
    }
    MT6701_CSN_SET; //MT6701_CSN=1
    
}

/*!
 *  @brief  Return position of encoder
 *  @return Angle value of encoder position
 */
float angleRead(void)
{
    float angle_f = 0.0f;
    uint8_t data[3];
    uint16_t angle_u16;

    MT6701_Read_RAW(data);
    angle_u16 = (uint16_t)(data[1] >> 2); //原始值
    angle_u16 |= ((uint16_t)data[0] << 6);
    angle_f = (float)angle_u16 * (_2PI / 16384.0f);
    return angle_f;
}

float GetAngle(float rawAngle) 
{

  float d_angle = rawAngle - angle_data_prev;
  if(abs(d_angle) > (0.8 * _2PI)) {
    full_rotation_offset += (d_angle > 0 ? -_2PI : _2PI);
  }
  angle_data_prev = rawAngle;
  
  return (full_rotation_offset + (rawAngle / (float)_2PI)*_2PI);
}

//angle_raw返回原始角度数据，angle转换后的角度值：0-360,field_status磁场强度；
void mt6701_read(uint16_t*angle_raw, float*angle, uint8_t*field_status)
{
    float angle_f = 0.0f;
    uint8_t status;
    uint8_t data[3];
    uint16_t angle_u16;

    MT6701_Read_RAW(data);
    angle_u16 = (uint16_t)(data[1] >> 2); //原始值
    angle_u16 |= ((uint16_t)data[0] << 6);
    status = (data[2] >> 6);
    status |= (data[1] & 0x03) << 2;
    if(angle_raw != NULL) {
        *angle_raw = angle_u16;
    }

    if(angle != NULL) {

        angle_f = (float)angle_u16 * (360.0f / 16384.0f);
        *angle = angle_f;
    }
    if(field_status != NULL) {
        *field_status = status & 0x03;
    }
}



/*
// 获取MT6701原始数据
uint16_t MT6701_GetRawData(void)
{
    uint16_t rawData;
    uint16_t txData = 0xFFFF;
    uint16_t timeOut = 200;

    while (HAL_SPI_GetState(&STM32_SPI_PORT) != HAL_SPI_STATE_READY)
    {
        if (timeOut-- == 0)
        {
            printf("SPI state error!\r\n");
            return 0; // 在超时时直接返回，避免继续执行后续代码
        }
    }

    MT6701_CS_Enable();

    HAL_StatusTypeDef spiStatus = HAL_SPI_TransmitReceive(&STM32_SPI_PORT, (uint8_t *)&txData, (uint8_t *)&rawData, 1, HAL_MAX_DELAY);
    if (spiStatus != HAL_OK)
    {
        MT6701_CS_Disable();
        printf("MT6701 read data error!\r\n");
        return 0; // 在SPI传输错误时直接返回，避免继续执行后续代码
    }

    MT6701_CS_Disable();

    return rawData >> 2; // 取高14位的角度数据
}

// 获得原始角度，无圈数累加
float MT6701_GetRawAngle(void)
{
    uint16_t rawData = MT6701_GetRawData();
    return (float)(rawData / 16384.0f * 360);
}

 */
