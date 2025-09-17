#ifndef BLE_HID_MOUSE_H
#define BLE_HID_MOUSE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BLE HID鼠标配置结构体
 * 
 * 包含所有鼠标功能的配置参数
 */
typedef struct {
    float mouse_sensitivity;            /**< 鼠标灵敏度 - 控制陀螺仪数据到鼠标移动的映射比例 */
    float gyro_threshold;               /**< 陀螺仪阈值 - 滤除小于此值的陀螺仪数据以避免抖动 */
    uint16_t sample_rate_ms;            /**< 采样间隔(毫秒) - 控制鼠标数据发送频率 */
    char device_name[32];               /**< 蓝牙设备名称 - 连接时显示的名称 */
} ble_hid_mouse_config_t;

/**
 * @brief 默认鼠标配置宏
 * 
 * 提供合理的默认参数值：
 * - 鼠标灵敏度: 2.2
 * - 陀螺仪阈值: 5.0
 * - 采样间隔: 10ms (约100Hz)
 * - 设备名称: "XiaoZhi-Mouse"
 */
#define BLE_HID_MOUSE_DEFAULT_CONFIG() { \
    .mouse_sensitivity = 2.2f, \
    .gyro_threshold = 5.0f, \
    .sample_rate_ms = 10, \
    .device_name = "XiaoZhi-Mouse" \
}

/**
 * @brief 初始化BLE HID鼠标功能
 * 
 * 该函数初始化蓝牙协议栈、HID配置文件、传感器和必要的硬件接口。
 * 
 * @param config 鼠标配置参数结构体指针，如果为NULL则使用默认配置
 * @return esp_err_t ESP_OK表示成功，其他值表示错误
 */
esp_err_t ble_hid_mouse_init(const ble_hid_mouse_config_t *config);

#ifdef __cplusplus
}
#endif

#endif // BLE_HID_MOUSE_H
