#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include "stm32f10x.h"

/* I2C1: PB6(SCL), PB7(SDA) */
#define BSP_I2C_PORT         GPIOB
#define BSP_I2C_SCL_PIN      GPIO_Pin_6
#define BSP_I2C_SDA_PIN      GPIO_Pin_7
#define BSP_I2C_SPEED        400000  /* 400kHz，BMI088支持快速模式 */
#define BSP_I2C_TIMEOUT      10000

void BSP_I2C_Init(void);
void BSP_I2C_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
void BSP_I2C_WriteBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
uint8_t BSP_I2C_ReadByte(uint8_t dev_addr, uint8_t reg_addr);
void BSP_I2C_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);

#endif