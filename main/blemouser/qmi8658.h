/*
 * QMI8658 6-DOF IMU 传感器库 - ESP-IDF C语言版本
 * 适用于ESP32平台的C语言驱动
 * 
 * 特性:
 * - 3轴陀螺仪和3轴加速度计
 * - 多种测量范围和采样率  
 * - I2C通信接口
 * - 中断和唤醒功能
 * 
 * 版本: 1.0
 * 日期: 2024
 */

#ifndef QMI8658_H
#define QMI8658_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

// QMI8658 设备地址
#define QMI8658_ADDRESS_LOW  0x6A    // SDO接地
#define QMI8658_ADDRESS_HIGH 0x6B    // SDO接VDD

// 寄存器地址
#define QMI8658_WHO_AM_I     0x00    // 设备ID寄存器，应返回0x05
#define QMI8658_REVISION_ID  0x01    // 版本ID寄存器
#define QMI8658_CTRL1        0x02    // 控制寄存器1
#define QMI8658_CTRL2        0x03    // 控制寄存器2 - 加速度计配置
#define QMI8658_CTRL3        0x04    // 控制寄存器3 - 陀螺仪配置
#define QMI8658_CTRL4        0x05    // 控制寄存器4
#define QMI8658_CTRL5        0x06    // 控制寄存器5 - 低功耗模式
#define QMI8658_CTRL6        0x07    // 控制寄存器6
#define QMI8658_CTRL7        0x08    // 控制寄存器7 - 传感器使能
#define QMI8658_CTRL8        0x09    // 控制寄存器8
#define QMI8658_CTRL9        0x0A    // 控制寄存器9

// 数据寄存器
#define QMI8658_TEMP_L       0x33    // 温度低字节
#define QMI8658_TEMP_H       0x34    // 温度高字节
#define QMI8658_AX_L         0x35    // 加速度X轴低字节
#define QMI8658_AX_H         0x36    // 加速度X轴高字节  
#define QMI8658_AY_L         0x37    // 加速度Y轴低字节
#define QMI8658_AY_H         0x38    // 加速度Y轴高字节
#define QMI8658_AZ_L         0x39    // 加速度Z轴低字节
#define QMI8658_AZ_H         0x3A    // 加速度Z轴高字节
#define QMI8658_GX_L         0x3B    // 陀螺仪X轴低字节
#define QMI8658_GX_H         0x3C    // 陀螺仪X轴高字节
#define QMI8658_GY_L         0x3D    // 陀螺仪Y轴低字节
#define QMI8658_GY_H         0x3E    // 陀螺仪Y轴高字节
#define QMI8658_GZ_L         0x3F    // 陀螺仪Z轴低字节
#define QMI8658_GZ_H         0x40    // 陀螺仪Z轴高字节

// 状态寄存器
#define QMI8658_STATUS0      0x2D    // 状态寄存器0
#define QMI8658_STATUS1      0x2E    // 状态寄存器1
#define QMI8658_TIMESTAMP_L  0x30    // 时间戳低字节
#define QMI8658_TIMESTAMP_M  0x31    // 时间戳中字节
#define QMI8658_TIMESTAMP_H  0x32    // 时间戳高字节

// 加速度计量程配置 (CTRL2)
#define QMI8658_ACCEL_RANGE_2G    0x00    // ±2g
#define QMI8658_ACCEL_RANGE_4G    0x01    // ±4g  
#define QMI8658_ACCEL_RANGE_8G    0x02    // ±8g
#define QMI8658_ACCEL_RANGE_16G   0x03    // ±16g

// 加速度计采样率配置 (CTRL2)
#define QMI8658_ACCEL_ODR_8000HZ  0x00    // 8000 Hz
#define QMI8658_ACCEL_ODR_4000HZ  0x01    // 4000 Hz
#define QMI8658_ACCEL_ODR_2000HZ  0x02    // 2000 Hz
#define QMI8658_ACCEL_ODR_1000HZ  0x03    // 1000 Hz
#define QMI8658_ACCEL_ODR_500HZ   0x04    // 500 Hz
#define QMI8658_ACCEL_ODR_250HZ   0x05    // 250 Hz
#define QMI8658_ACCEL_ODR_125HZ   0x06    // 125 Hz
#define QMI8658_ACCEL_ODR_62_5HZ  0x07    // 62.5 Hz
#define QMI8658_ACCEL_ODR_31_25HZ 0x08    // 31.25 Hz
#define QMI8658_ACCEL_ODR_LOWPWR  0x09    // 低功耗模式

// 陀螺仪量程配置 (CTRL3)
#define QMI8658_GYRO_RANGE_16DPS   0x00   // ±16 dps
#define QMI8658_GYRO_RANGE_32DPS   0x01   // ±32 dps
#define QMI8658_GYRO_RANGE_64DPS   0x02   // ±64 dps
#define QMI8658_GYRO_RANGE_128DPS  0x03   // ±128 dps
#define QMI8658_GYRO_RANGE_256DPS  0x04   // ±256 dps
#define QMI8658_GYRO_RANGE_512DPS  0x05   // ±512 dps
#define QMI8658_GYRO_RANGE_1024DPS 0x06   // ±1024 dps
#define QMI8658_GYRO_RANGE_2048DPS 0x07   // ±2048 dps

// 陀螺仪采样率配置 (CTRL3)
#define QMI8658_GYRO_ODR_8000HZ   0x00    // 8000 Hz
#define QMI8658_GYRO_ODR_4000HZ   0x01    // 4000 Hz
#define QMI8658_GYRO_ODR_2000HZ   0x02    // 2000 Hz
#define QMI8658_GYRO_ODR_1000HZ   0x03    // 1000 Hz
#define QMI8658_GYRO_ODR_500HZ    0x04    // 500 Hz
#define QMI8658_GYRO_ODR_250HZ    0x05    // 250 Hz
#define QMI8658_GYRO_ODR_125HZ    0x06    // 125 Hz
#define QMI8658_GYRO_ODR_62_5HZ   0x07    // 62.5 Hz
#define QMI8658_GYRO_ODR_31_25HZ  0x08    // 31.25 Hz

// 传感器使能标志 (CTRL7)
#define QMI8658_ENABLE_GYRO       0x01    // 启用陀螺仪
#define QMI8658_ENABLE_ACCEL      0x02    // 启用加速度计
#define QMI8658_ENABLE_MOTION     0x04    // 启用运动检测

// 低功耗模式配置 (CTRL5)
#define QMI8658_LP_MODE_NORMAL    0x00    // 正常模式
#define QMI8658_LP_MODE_LOW       0x01    // 低功耗模式

// 传感器配置结构体
typedef struct {
    i2c_port_t i2c_port;          // I2C端口号
    uint8_t device_address;        // 设备地址
    uint8_t accel_range;           // 加速度计量程
    uint8_t gyro_range;            // 陀螺仪量程
    uint16_t accel_lsb_div;        // 加速度计LSB除数
    uint16_t gyro_lsb_div;         // 陀螺仪LSB除数
    bool initialized;              // 初始化状态
} qmi8658_handle_t;

// 6轴数据结构
typedef struct {
    float accel_x, accel_y, accel_z;   // 加速度计数据 (m/s²)
    float gyro_x, gyro_y, gyro_z;      // 陀螺仪数据 (dps)
    float temperature;                 // 温度 (°C)
    uint32_t timestamp;                // 内部时间戳
} qmi8658_data_t;

// API函数声明

/**
 * @brief 初始化QMI8658传感器
 * @param handle 传感器句柄
 * @param i2c_port I2C端口号
 * @param sda_pin SDA引脚
 * @param scl_pin SCL引脚
 * @param device_addr 设备地址
 * @return true 初始化成功，false 初始化失败
 */
bool qmi8658_init(qmi8658_handle_t *handle, i2c_port_t i2c_port, 
                  int sda_pin, int scl_pin, uint8_t device_addr);

/**
 * @brief 设置加速度计量程
 * @param handle 传感器句柄
 * @param range 量程配置
 * @return true 设置成功，false 设置失败
 */
bool qmi8658_set_accel_range(qmi8658_handle_t *handle, uint8_t range);

/**
 * @brief 设置陀螺仪量程
 * @param handle 传感器句柄
 * @param range 量程配置
 * @return true 设置成功，false 设置失败
 */
bool qmi8658_set_gyro_range(qmi8658_handle_t *handle, uint8_t range);

/**
 * @brief 设置加速度计采样率
 * @param handle 传感器句柄
 * @param odr 采样率配置
 * @return true 设置成功，false 设置失败
 */
bool qmi8658_set_accel_odr(qmi8658_handle_t *handle, uint8_t odr);

/**
 * @brief 设置陀螺仪采样率
 * @param handle 传感器句柄
 * @param odr 采样率配置
 * @return true 设置成功，false 设置失败
 */
bool qmi8658_set_gyro_odr(qmi8658_handle_t *handle, uint8_t odr);

/**
 * @brief 启用传感器
 * @param handle 传感器句柄
 * @param enable_flags 启用标志
 * @return true 启用成功，false 启用失败
 */
bool qmi8658_enable_sensors(qmi8658_handle_t *handle, uint8_t enable_flags);

/**
 * @brief 读取陀螺仪数据 (dps)
 * @param handle 传感器句柄
 * @param x X轴数据
 * @param y Y轴数据
 * @param z Z轴数据
 * @return true 读取成功，false 读取失败
 */
bool qmi8658_read_gyro_dps(qmi8658_handle_t *handle, float *x, float *y, float *z);

/**
 * @brief 读取加速度计数据 (mg)
 * @param handle 传感器句柄
 * @param x X轴数据
 * @param y Y轴数据
 * @param z Z轴数据
 * @return true 读取成功，false 读取失败
 */
bool qmi8658_read_accel_mg(qmi8658_handle_t *handle, float *x, float *y, float *z);

/**
 * @brief 读取温度数据
 * @param handle 传感器句柄
 * @param temp 温度值
 * @return true 读取成功，false 读取失败
 */
bool qmi8658_read_temperature(qmi8658_handle_t *handle, float *temp);

/**
 * @brief 获取WHO AM I寄存器值
 * @param handle 传感器句柄
 * @return WHO AM I值 (应为0x05)
 */
uint8_t qmi8658_get_who_am_i(qmi8658_handle_t *handle);

/**
 * @brief 软复位传感器
 * @param handle 传感器句柄
 * @return true 复位成功，false 复位失败
 */
bool qmi8658_reset(qmi8658_handle_t *handle);

/**
 * @brief 检查数据是否准备就绪
 * @param handle 传感器句柄
 * @return true 数据就绪，false 数据未就绪
 */
bool qmi8658_is_data_ready(qmi8658_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif // QMI8658_H
