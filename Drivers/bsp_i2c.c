#include "bsp_i2c.h"

static void I2C_WaitEvent(I2C_TypeDef* I2Cx, uint32_t event) {
    uint32_t timeout = BSP_I2C_TIMEOUT;
    while (I2C_CheckEvent(I2Cx, event) != SUCCESS) {
        if (timeout-- == 0) break;
    }
}

void BSP_I2C_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* PB6(SCL), PB7(SDA) 复用开漏输出 */
    GPIO_InitStructure.GPIO_Pin = BSP_I2C_SCL_PIN | BSP_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* I2C1配置 */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = BSP_I2C_SPEED;
    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_Cmd(I2C1, ENABLE);
}

/* 单字节写入寄存器 */
void BSP_I2C_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    I2C_GenerateSTART(I2C1, ENABLE);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);
    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Transmitter);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    I2C_SendData(I2C1, reg_addr);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    I2C_SendData(I2C1, data);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    I2C_GenerateSTOP(I2C1, ENABLE);
}

/* 多字节写入（用于初始化配置） */
void BSP_I2C_WriteBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
    uint8_t i;
    I2C_GenerateSTART(I2C1, ENABLE);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);
    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Transmitter);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    I2C_SendData(I2C1, reg_addr);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    for (i = 0; i < len; i++) {
        I2C_SendData(I2C1, data[i]);
        I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    }
    I2C_GenerateSTOP(I2C1, ENABLE);
}

/* 单字节读取 */
uint8_t BSP_I2C_ReadByte(uint8_t dev_addr, uint8_t reg_addr) {
    uint8_t data;
    
    /* 先发送寄存器地址 */
    I2C_GenerateSTART(I2C1, ENABLE);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);
    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Transmitter);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    I2C_SendData(I2C1, reg_addr);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    
    /* 重复起始 + 读取 */
    I2C_GenerateSTART(I2C1, ENABLE);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);
    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Receiver);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
    
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED);
    data = I2C_ReceiveData(I2C1);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    
    return data;
}

/* 多字节连续读取（读取六轴数据时使用） */
void BSP_I2C_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
    uint8_t i;
    
    I2C_GenerateSTART(I2C1, ENABLE);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);
    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Transmitter);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    I2C_SendData(I2C1, reg_addr);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    
    I2C_GenerateSTART(I2C1, ENABLE);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);
    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Receiver);
    I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
    
    for (i = 0; i < len; i++) {
        if (i == len - 1) {
            I2C_AcknowledgeConfig(I2C1, DISABLE);   /* 最后一字节不响应 */
        }
        I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED);
        data[i] = I2C_ReceiveData(I2C1);
    }
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}