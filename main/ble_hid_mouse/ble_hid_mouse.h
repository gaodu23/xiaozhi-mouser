#ifndef BLE_HID_MOUSE_H
#define BLE_HID_MOUSE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 配置结构
typedef struct {
    float mouse_sensitivity;            // 鼠标灵敏度
    float gyro_threshold;               // 陀螺仪阈值
    uint16_t sample_rate_ms;            // 采样间隔
    char device_name[32];               // 设备名称
} ble_hid_mouse_config_t;

// 默认配置（单一模式）
#define BLE_HID_MOUSE_DEFAULT_CONFIG() { \
    .mouse_sensitivity = 370.0f, \
    .gyro_threshold = 0.1f, \
    .sample_rate_ms = 10, \
    .device_name = "XiaoZhi-Mouse" \
}

// API函数声明
esp_err_t ble_hid_mouse_init(const ble_hid_mouse_config_t *config);

#ifdef __cplusplus
}
#endif

#endif // BLE_HID_MOUSE_H
