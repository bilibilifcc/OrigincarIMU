# Origincar IMU

这是一个基于BMI088**模块**和STM32F103C6T6的IIC、UART桥接器。

STM32读取BMI088的数据，通过串口转USB发送给上位机进行处理，数据均为原始数据。

## 1.串口协议定义

波特率**921600**，其他默认。

| byte_index | 数据  | 备注                               |
| ---------- | ----- | ---------------------------------- |
| 0          | 0xFF  | 固定帧头                           |
| 1          | [cnt] | 不包括帧头、校验位、这位的数据长度 |
| [2,2+12)   |       | acc_xyz、gyro_xyz、类型int16_t     |
| 14         |       | 校验和，C代码如下                  |

 ``` c
 uint8_t CheckSum(uint8_t *Buf, uint8_t Len) {
 	uint8_t sum = 0;
 	for (uint8_t i = 0; i < Len; i++) {
 		sum += Buf[i];
 	}
 	return sum;
 }
 ```