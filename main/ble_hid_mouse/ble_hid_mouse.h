#ifndef BLE_HID_MOUSE_H
#define BLE_HID_MOUSE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 蓝牙鼠标模式枚举
typedef enum {
    BLE_MOUSE_MODE_PC = 0,        // 电脑模式
    BLE_MOUSE_MODE_PHONE = 1,     // 手机模式  
    BLE_MOUSE_MODE_TABLET = 2,    // 平板模式
    BLE_MOUSE_MODE_MAX
} ble_mouse_mode_t;

// 初始化参数结构
typedef struct {
    float mouse_sensitivity;            // 鼠标灵敏度
    float gyro_threshold;               // 陀螺仪阈值
    uint16_t sample_rate_ms;            // 采样间隔
    char device_name[32];               // 设备名称
    ble_mouse_mode_t mode;              // 当前模式
} ble_hid_mouse_config_t;

// 默认配置
#define BLE_HID_MOUSE_DEFAULT_CONFIG() { \
    .mouse_sensitivity = 50, \
    .gyro_threshold = 0.2f, \
    .sample_rate_ms = 20, \
    .device_name = "Mouse-XZ-PC", \
    .mode = BLE_MOUSE_MODE_PC \
}

// API函数声明
esp_err_t ble_hid_mouse_init(const ble_hid_mouse_config_t *config);
esp_err_t ble_hid_mouse_start(void);
esp_err_t ble_hid_mouse_stop(void);
esp_err_t ble_hid_mouse_deinit(void);
bool ble_hid_mouse_is_connected(void);
bool ble_hid_mouse_is_initialized(void);

// 滚轮控制API
esp_err_t ble_hid_mouse_scroll_up(void);
esp_err_t ble_hid_mouse_scroll_down(void);

// 音量控制API
esp_err_t ble_hid_mouse_volume_up(void);
esp_err_t ble_hid_mouse_volume_down(void);

// 灵敏度控制API
esp_err_t ble_hid_mouse_set_sensitivity(float sensitivity);
float ble_hid_mouse_get_sensitivity(void);

// 模式切换API
esp_err_t ble_hid_mouse_set_mode(ble_mouse_mode_t mode);
ble_mouse_mode_t ble_hid_mouse_get_mode(void);

// 配置获取API
esp_err_t ble_hid_mouse_get_config(ble_hid_mouse_config_t *config);

#ifdef __cplusplus
}
#endif

#endif // BLE_HID_MOUSE_H
