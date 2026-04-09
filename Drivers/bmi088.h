#ifndef __BMI088_H
#define __BMI088_H

#include "stm32f10x.h"

/* BMI088 I2C地址（7位） */
#define BMI088_ACC_ADDR     0x19  /* SDO1=GND */
#define BMI088_GYRO_ADDR    0x69  /* SDO2=GND */

/* 加速度计寄存器 */
#define BMI088_ACC_CHIP_ID      0x00  /* 芯片ID，应为0x1E */
#define BMI088_ACC_DATA         0x12  /* X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB */
#define BMI088_ACC_CONF         0x40  /* 带宽/ODR配置 */
#define BMI088_ACC_RANGE        0x41  /* 量程配置 */
#define BMI088_ACC_PWR_CTRL     0x7D  /* 电源控制 */
#define BMI088_ACC_PWR_CONF     0x7C  /* 省电配置 */
#define BMI088_ACC_SOFTRESET    0x7E  /* 软复位 */

/* 陀螺仪寄存器 */
#define BMI088_GYRO_CHIP_ID     0x00  /* 芯片ID，应为0x0F */
#define BMI088_GYRO_DATA        0x02  /* X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB */
#define BMI088_GYRO_RANGE       0x0F  /* 量程配置 */
#define BMI088_GYRO_BANDWIDTH   0x10  /* 带宽配置 */
#define BMI088_GYRO_LPM1        0x11  /* 低功耗模式 */
#define BMI088_GYRO_SOFTRESET   0x14  /* 软复位 */

/* 量程选择 */
#define BMI088_ACC_RANGE_3G     0x00
#define BMI088_ACC_RANGE_6G     0x01
#define BMI088_ACC_RANGE_12G    0x02
#define BMI088_ACC_RANGE_24G    0x03

#define BMI088_GYRO_RANGE_2000  0x00  /* ±2000°/s */
#define BMI088_GYRO_RANGE_1000  0x01  /* ±1000°/s */
#define BMI088_GYRO_RANGE_500   0x02  /* ±500°/s */
#define BMI088_GYRO_RANGE_250   0x03  /* ±250°/s */
#define BMI088_GYRO_RANGE_125   0x04  /* ±125°/s */

/* 原始数据结构 */
typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} BMI088_RawData;

uint8_t BMI088_Init(void);
uint8_t BMI088_ReadAccel(int16_t *x, int16_t *y, int16_t *z);
uint8_t BMI088_ReadGyro(int16_t *x, int16_t *y, int16_t *z);
void BMI088_ReadAll(BMI088_RawData *data);

#endif