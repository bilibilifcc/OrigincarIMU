/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *   Converted to pure C by [Your Name]
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file attitude_estimator_q.h
 *
 * Attitude estimator (quaternion based) - Pure C implementation
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Converted to C by [Your Name]
 */

#ifndef ATTITUDE_ESTIMATOR_Q_H
#define ATTITUDE_ESTIMATOR_Q_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/* 常量定义                                                                   */
/*===========================================================================*/

#define ATT_EST_Q_EO_MAX_STD_DEV      100.0f      /* 估计姿态的最大允许标准差 */
#define ATT_EST_Q_DT_MIN              0.00001f    /* 最小时间步长 */
#define ATT_EST_Q_DT_MAX              0.02f       /* 最大时间步长 */
#define ATT_EST_Q_ONE_G               9.80665f    /* 重力加速度 (m/s^2) */
#define ATT_EST_Q_SPIN_RATE_THRESH    0.175f      /* 陀螺仪偏置估计的角速度阈值 (rad/s) */
#define ATT_EST_Q_FIFTY_DPS           0.873f      /* 50度/秒转换为弧度/秒 */

/* 外部航向源模式 */
#define ATT_EXT_HDG_M_NONE            0   /* 不使用外部航向 */
#define ATT_EXT_HDG_M_VISION          1   /* 使用视觉里程计航向 */
#define ATT_EXT_HDG_M_MOCAP           2   /* 使用动作捕捉航向 */

/*===========================================================================*/
/* 数据结构定义                                                               */
/*===========================================================================*/

/**
 * @brief 三维向量结构体
 */
typedef struct {
    float x;
    float y;
    float z;
} Vector3f;

/**
 * @brief 四元数结构体
 */
typedef struct {
    float w;    /* q0 */
    float x;    /* q1 */
    float y;    /* q2 */
    float z;    /* q3 */
} Quaternion;

/**
 * @brief 欧拉角结构体 (弧度)
 */
typedef struct {
    float roll;     /* 横滚角，绕X轴 */
    float pitch;    /* 俯仰角，绕Y轴 */
    float yaw;      /* 偏航角，绕Z轴 */
} EulerAngles;

/**
 * @brief 传感器数据结构体
 */
typedef struct {
    uint64_t timestamp;             /* 时间戳 (微秒) */
    Vector3f gyro;                  /* 陀螺仪数据 (rad/s) */
    Vector3f accel;                 /* 加速度计数据 (m/s^2) */
} SensorData;

/**
 * @brief 磁力计数据结构体
 */
typedef struct {
    uint64_t timestamp;             /* 时间戳 (微秒) */
    Vector3f mag;                   /* 磁场数据 (Gauss) */
} MagnetometerData;

/**
 * @brief GPS位置数据结构体
 */
typedef struct {
    uint64_t timestamp;             /* 时间戳 (微秒) */
    double latitude;                /* 纬度 (度) */
    double longitude;               /* 经度 (度) */
    float eph;                      /* 水平位置精度估计 */
} GpsData;

/**
 * @brief 视觉/动作捕捉里程计数据结构体
 */
typedef struct {
    uint64_t timestamp;             /* 时间戳 (微秒) */
    uint64_t timestamp_sample;      /* 采样时间戳 (微秒) */
    Quaternion q;                   /* 姿态四元数 */
    float orientation_variance[3];  /* 姿态方差 [roll, pitch, yaw] */
    bool valid;                     /* 数据有效标志 */
} OdometryData;

/**
 * @brief 本地位置数据结构体
 */
typedef struct {
    uint64_t timestamp;             /* 时间戳 (微秒) */
    Vector3f velocity;              /* 速度 (m/s) */
    bool v_xy_valid;                /* 水平速度有效 */
    bool v_z_valid;                 /* 垂直速度有效 */
    float eph;                      /* 水平位置精度估计 */
} LocalPositionData;

/**
 * @brief 输出姿态数据结构体
 */
typedef struct {
    uint64_t timestamp;             /* 发布时的时间戳 */
    uint64_t timestamp_sample;      /* 传感器采样时间戳 */
    Quaternion q;                   /* 姿态四元数 */
    EulerAngles euler;              /* 欧拉角 (可选) */
} AttitudeOutput;

/**
 * @brief 姿态估计器参数结构体
 */
typedef struct {
    float w_acc;                    /* 加速度计修正权重 */
    float w_mag;                    /* 磁力计修正权重 */
    float w_ext_hdg;                /* 外部航向修正权重 */
    float w_gyro_bias;              /* 陀螺仪偏置估计权重 */
    float mag_decl;                 /* 磁偏角 (弧度) */
    int mag_decl_auto;              /* 自动磁偏角使能 */
    int ext_hdg_mode;               /* 外部航向模式 */
    int acc_comp;                   /* 加速度补偿使能 */
    float bias_max;                 /* 最大陀螺仪偏置 (rad/s) */
    int has_mag;                    /* 系统是否有磁力计 */
} AttitudeEstimatorParams;

/**
 * @brief 姿态估计器对象 (面向对象风格)
 */
typedef struct AttitudeEstimatorQ AttitudeEstimatorQ;

struct AttitudeEstimatorQ {
    /* 公有数据 (只读) */
    AttitudeOutput output;          /* 当前估计的姿态输出 */
    bool initialized;               /* 估计器是否已初始化 */
    bool data_good;                 /* 传感器数据是否良好 */
    
    /* 私有数据 (不应直接访问) */
    /* 使用不透明指针隐藏内部实现细节 */
    void *private_data;
};

/*===========================================================================*/
/* 公共接口函数 (模拟C++类的成员函数)                                         */
/*===========================================================================*/

/**
 * @brief 创建姿态估计器实例 (构造函数)
 * @return 姿态估计器指针，失败返回NULL
 */
AttitudeEstimatorQ* attitude_estimator_q_create(void);

/**
 * @brief 销毁姿态估计器实例 (析构函数)
 * @param est 姿态估计器指针
 */
void attitude_estimator_q_destroy(AttitudeEstimatorQ *est);

/**
 * @brief 初始化姿态估计器
 * @param est 姿态估计器指针
 * @return true:成功, false:失败
 */
bool attitude_estimator_q_init(AttitudeEstimatorQ *est);

/**
 * @brief 设置估计器参数
 * @param est 姿态估计器指针
 * @param params 参数结构体
 */
void attitude_estimator_q_set_params(AttitudeEstimatorQ *est, const AttitudeEstimatorParams *params);

/**
 * @brief 获取当前参数
 * @param est 姿态估计器指针
 * @param params 输出参数结构体
 */
void attitude_estimator_q_get_params(const AttitudeEstimatorQ *est, AttitudeEstimatorParams *params);

/**
 * @brief 更新IMU传感器数据
 * @param est 姿态估计器指针
 * @param data 传感器数据
 */
void attitude_estimator_q_update_imu(AttitudeEstimatorQ *est, const SensorData *data);

/**
 * @brief 更新磁力计数据
 * @param est 姿态估计器指针
 * @param mag 磁力计数据
 */
void attitude_estimator_q_update_mag(AttitudeEstimatorQ *est, const MagnetometerData *mag);

/**
 * @brief 更新GPS数据 (用于自动磁偏角计算)
 * @param est 姿态估计器指针
 * @param gps GPS数据
 */
void attitude_estimator_q_update_gps(AttitudeEstimatorQ *est, const GpsData *gps);

/**
 * @brief 更新视觉里程计数据
 * @param est 姿态估计器指针
 * @param odom 里程计数据
 */
void attitude_estimator_q_update_vision(AttitudeEstimatorQ *est, const OdometryData *odom);

/**
 * @brief 更新动作捕捉里程计数据
 * @param est 姿态估计器指针
 * @param odom 里程计数据
 */
void attitude_estimator_q_update_mocap(AttitudeEstimatorQ *est, const OdometryData *odom);

/**
 * @brief 更新本地位置数据 (用于加速度补偿)
 * @param est 姿态估计器指针
 * @param lpos 本地位置数据
 */
void attitude_estimator_q_update_local_pos(AttitudeEstimatorQ *est, const LocalPositionData *lpos);

/**
 * @brief 执行姿态更新 (每次IMU数据到来后调用)
 * @param est 姿态估计器指针
 * @return true:更新成功并输出新姿态, false:更新失败
 */
bool attitude_estimator_q_update(AttitudeEstimatorQ *est);

/**
 * @brief 重置估计器状态
 * @param est 姿态估计器指针
 */
void attitude_estimator_q_reset(AttitudeEstimatorQ *est);

/**
 * @brief 强制重新初始化姿态
 * @param est 姿态估计器指针
 * @return true:成功, false:失败
 */
bool attitude_estimator_q_reinit(AttitudeEstimatorQ *est);

/*===========================================================================*/
/* 工具函数                                                                   */
/*===========================================================================*/

/**
 * @brief 角度归一化到 [-PI, PI]
 * @param angle 输入角度 (弧度)
 * @return 归一化后的角度
 */
static inline float wrap_pi(float angle)
{
    while (angle > M_PI) {
        angle -= 2.0f * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
    return angle;
}

/**
 * @brief 数值限幅
 * @param val 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅后的值
 */
static inline float constrain_float(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/**
 * @brief 获取最小值的宏
 */
#define min(a, b) ((a) < (b) ? (a) : (b))

/**
 * @brief 获取最大值的宏
 */
#define max(a, b) ((a) > (b) ? (a) : (b))

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_ESTIMATOR_Q_H */