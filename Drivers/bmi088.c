#include "bmi088.h"
#include "bsp_i2c.h"
#include "delay.h"

// 上电校准
static BMI088_RawData calibra[100];
static BMI088_RawData bias;

/* 验证芯片ID */
static uint8_t BMI088_CheckID(void) {
    uint8_t acc_id = BSP_I2C_ReadByte(BMI088_ACC_ADDR, BMI088_ACC_CHIP_ID);
    uint8_t gyro_id = BSP_I2C_ReadByte(BMI088_GYRO_ADDR, BMI088_GYRO_CHIP_ID);
    
    /* 加速度计ID应为0x1E，陀螺仪ID应为0x0F */
    if (acc_id == 0x1E && gyro_id == 0x0F)
        return 1;
    return 0;
}

/* 初始化BMI088 */
uint8_t BMI088_Init(void) {
    uint8_t tmp;
    
    /* 1. 加速度计软复位 */
    BSP_I2C_WriteByte(BMI088_ACC_ADDR, BMI088_ACC_SOFTRESET, 0xB6);
    Delay_ms(50);  /* 等待复位完成 */
    
    /* 2. 陀螺仪软复位 */
    BSP_I2C_WriteByte(BMI088_GYRO_ADDR, BMI088_GYRO_SOFTRESET, 0xB6);
    Delay_ms(50);
    
    /* 3. 验证芯片ID */
    if (!BMI088_CheckID())
        return 0;
    
    /* 4. 加速度计配置：±24g量程，100Hz ODR，正常带宽 */
		BSP_I2C_WriteByte(BMI088_ACC_ADDR, BMI088_ACC_PWR_CTRL, 0x04);  // 使能加速度计
		Delay_ms(50);
    BSP_I2C_WriteByte(BMI088_ACC_ADDR, BMI088_ACC_PWR_CONF, 0x00);  /* 正常模式 */
    Delay_ms(50);
    
    tmp = (0x0A << 4) | 0x0A;  /* BWP=5(正常带宽), ODR=100Hz */
    BSP_I2C_WriteByte(BMI088_ACC_ADDR, BMI088_ACC_CONF, tmp);
    BSP_I2C_WriteByte(BMI088_ACC_ADDR, BMI088_ACC_RANGE, BMI088_ACC_RANGE_12G);
    
    /* 5. 陀螺仪配置：±2000°/s量程，100Hz ODR */
    BSP_I2C_WriteByte(BMI088_GYRO_ADDR, BMI088_GYRO_RANGE, BMI088_GYRO_RANGE_500);
    BSP_I2C_WriteByte(BMI088_GYRO_ADDR, BMI088_GYRO_BANDWIDTH, 0x05);  /* 100Hz, 32kHz采样 */
    BSP_I2C_WriteByte(BMI088_GYRO_ADDR, BMI088_GYRO_LPM1, 0x00);       /* 正常模式 */
		
		// Delay_ms(500);
    
    return 1;
}

/* 读取加速度计原始数据（16位有符号，小端格式） */
uint8_t BMI088_ReadAccel(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buf[6];
    
    BSP_I2C_ReadBytes(BMI088_ACC_ADDR, BMI088_ACC_DATA, buf, 6);
    
    *x = (int16_t)((buf[1] << 8) | buf[0]);
    *y = (int16_t)((buf[3] << 8) | buf[2]);
    *z = (int16_t)((buf[5] << 8) | buf[4]);
    
    return 1;
}

/* 读取陀螺仪原始数据 */
uint8_t BMI088_ReadGyro(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buf[6];
    
    BSP_I2C_ReadBytes(BMI088_GYRO_ADDR, BMI088_GYRO_DATA, buf, 6);
    
    *x = (int16_t)((buf[1] << 8) | buf[0]);
    *y = (int16_t)((buf[3] << 8) | buf[2]);
    *z = (int16_t)((buf[5] << 8) | buf[4]);
    
    return 1;
}

/* 一次性读取六轴数据 */
void BMI088_ReadAll(BMI088_RawData *data) {
    BMI088_ReadAccel(&data->acc_x, &data->acc_y, &data->acc_z);
    BMI088_ReadGyro(&data->gyro_x, &data->gyro_y, &data->gyro_z);
}

/* 自检函数：检查传感器是否正常响应 */
uint8_t BMI088_SelfTest(void) {
    uint8_t acc_id, gyro_id;
    
    acc_id = BSP_I2C_ReadByte(BMI088_ACC_ADDR, BMI088_ACC_CHIP_ID);
    gyro_id = BSP_I2C_ReadByte(BMI088_GYRO_ADDR, BMI088_GYRO_CHIP_ID);
    
    if (acc_id == 0x1E && gyro_id == 0x0F)
        return 1;
    else
        return 0;
}

/* 设置加速度计量程
 * range: BMI088_ACC_RANGE_3G / 6G / 12G / 24G
 */
void BMI088_SetAccelRange(uint8_t range) {
    if (range <= BMI088_ACC_RANGE_24G) {
        BSP_I2C_WriteByte(BMI088_ACC_ADDR, BMI088_ACC_RANGE, range);
    }
}

/* 设置陀螺仪量程
 * range: BMI088_GYRO_RANGE_2000 / 1000 / 500 / 250 / 125
 */
void BMI088_SetGyroRange(uint8_t range) {
    if (range <= BMI088_GYRO_RANGE_125) {
        BSP_I2C_WriteByte(BMI088_GYRO_ADDR, BMI088_GYRO_RANGE, range);
    }
}

/* 获取加速度计当前量程对应的灵敏度（LSB/g）
 * 用于将原始值转换为实际物理量（g）
 */
float BMI088_GetAccelSensitivity(void) {
    uint8_t range_reg;
    range_reg = BSP_I2C_ReadByte(BMI088_ACC_ADDR, BMI088_ACC_RANGE);
    
    switch (range_reg & 0x03) {
        case BMI088_ACC_RANGE_3G:   return 10920.0f;  /* ±3g: 10920 LSB/g */
        case BMI088_ACC_RANGE_6G:   return 5460.0f;   /* ±6g: 5460 LSB/g */
        case BMI088_ACC_RANGE_12G:  return 2730.0f;   /* ±12g: 2730 LSB/g */
        case BMI088_ACC_RANGE_24G:  return 1365.0f;   /* ±24g: 1365 LSB/g */
        default:                    return 1365.0f;
    }
}

/* 获取陀螺仪当前量程对应的灵敏度（LSB/°/s）
 * 用于将原始值转换为实际物理量（°/s）
 */
float BMI088_GetGyroSensitivity(void) {
    uint8_t range_reg;
    range_reg = BSP_I2C_ReadByte(BMI088_GYRO_ADDR, BMI088_GYRO_RANGE);
    
    switch (range_reg & 0x07) {
        case BMI088_GYRO_RANGE_2000: return 16.384f;  /* ±2000°/s: 16.384 LSB/°/s */
        case BMI088_GYRO_RANGE_1000: return 32.768f;  /* ±1000°/s: 32.768 LSB/°/s */
        case BMI088_GYRO_RANGE_500:  return 65.536f;  /* ±500°/s: 65.536 LSB/°/s */
        case BMI088_GYRO_RANGE_250:  return 131.072f; /* ±250°/s: 131.072 LSB/°/s */
        case BMI088_GYRO_RANGE_125:  return 262.144f; /* ±125°/s: 262.144 LSB/°/s */
        default:                     return 16.384f;
    }
}

/* 将加速度计原始值转换为实际加速度值（单位：g）
 * 返回结果存入传入的指针
 */
void BMI088_ConvertAccelToG(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                            float *g_x, float *g_y, float *g_z) {
    float sensitivity = BMI088_GetAccelSensitivity();
    
    *g_x = (float)raw_x / sensitivity;
    *g_y = (float)raw_y / sensitivity;
    *g_z = (float)raw_z / sensitivity;
}

/* 将陀螺仪原始值转换为实际角速度值（单位：°/s）
 * 返回结果存入传入的指针
 */
void BMI088_ConvertGyroToDPS(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                             float *dps_x, float *dps_y, float *dps_z) {
    float sensitivity = BMI088_GetGyroSensitivity();
    
    *dps_x = (float)raw_x / sensitivity;
    *dps_y = (float)raw_y / sensitivity;
    *dps_z = (float)raw_z / sensitivity;
}