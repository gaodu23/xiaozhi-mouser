#include "bluetooth_mouse.h"
#include <string.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_hidd_prf_api.h>
#include <esp_bt_defs.h>

#define MOUSE_TAG "BT_MOUSE"

// HID鼠标报告描述符
const uint8_t BluetoothMouse::hid_mouse_descriptor_[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,        // Usage (Mouse)
    0xa1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xa1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x03,        //     Usage Maximum (0x03)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x03,        //     Report Count (3)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x05,        //     Report Size (5)
    0x81, 0x03,        //     Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0xc0,              //   End Collection
    0xc0,              // End Collection
};

BluetoothMouse* BluetoothMouse::instance_ = nullptr;

BluetoothMouse::BluetoothMouse()
    : initialized_(false), connected_(false)
{
    instance_ = this;
    strcpy(device_name_, "Gezipai Mouse");
}

BluetoothMouse::~BluetoothMouse()
{
    Deinitialize();
    instance_ = nullptr;
}

bool BluetoothMouse::Initialize()
{
    if (initialized_) {
        ESP_LOGW(MOUSE_TAG, "蓝牙鼠标已初始化");
        return true;
    }

    ESP_LOGI(MOUSE_TAG, "初始化蓝牙鼠标...");

    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 释放经典蓝牙内存
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    // 初始化蓝牙控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(MOUSE_TAG, "蓝牙控制器初始化失败: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(MOUSE_TAG, "蓝牙控制器启用失败: %s", esp_err_to_name(ret));
        return false;
    }

    // 初始化蓝牙协议栈
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(MOUSE_TAG, "蓝牙协议栈初始化失败: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(MOUSE_TAG, "蓝牙协议栈启用失败: %s", esp_err_to_name(ret));
        return false;
    }

    // 注册GAP回调
    ret = esp_bt_gap_register_callback(GapEventHandler);
    if (ret != ESP_OK) {
        ESP_LOGE(MOUSE_TAG, "GAP回调注册失败: %s", esp_err_to_name(ret));
        return false;
    }

    // 注册HID设备回调
    ret = esp_bt_hidd_register_callback(HiddEventHandler);
    if (ret != ESP_OK) {
        ESP_LOGE(MOUSE_TAG, "HID设备回调注册失败: %s", esp_err_to_name(ret));
        return false;
    }

    // 初始化HID设备
    esp_hidd_app_param_t app_param = {0};
    app_param.name = device_name_;
    app_param.description = "Gezipai Bluetooth Mouse";
    app_param.provider = "Gezipai";
    app_param.subclass = 0x80; // 指向设备
    app_param.desc_list = hid_mouse_descriptor_;
    app_param.desc_list_len = sizeof(hid_mouse_descriptor_);

    ret = esp_bt_hidd_device_register_app(&app_param, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(MOUSE_TAG, "HID设备注册失败: %s", esp_err_to_name(ret));
        return false;
    }

    // 设置设备为可发现和可连接
    ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    if (ret != ESP_OK) {
        ESP_LOGE(MOUSE_TAG, "设置扫描模式失败: %s", esp_err_to_name(ret));
        return false;
    }

    // 设置设备名称
    ret = esp_bt_dev_set_device_name(device_name_);
    if (ret != ESP_OK) {
        ESP_LOGE(MOUSE_TAG, "设置设备名称失败: %s", esp_err_to_name(ret));
        return false;
    }

    // 设置CoD (Class of Device) 为鼠标
    esp_bt_cod_t cod;
    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    cod.minor = ESP_BT_COD_MINOR_DEV_PERIPHERAL_POINTING;
    cod.service = ESP_BT_COD_SRVC_NONE;
    ret = esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);
    if (ret != ESP_OK) {
        ESP_LOGE(MOUSE_TAG, "设置CoD失败: %s", esp_err_to_name(ret));
        return false;
    }

    initialized_ = true;
    ESP_LOGI(MOUSE_TAG, "蓝牙鼠标初始化成功");
    return true;
}

void BluetoothMouse::Deinitialize()
{
    if (!initialized_) {
        return;
    }

    ESP_LOGI(MOUSE_TAG, "销毁蓝牙鼠标...");

    esp_bt_hidd_device_unregister_app();
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();

    initialized_ = false;
    connected_ = false;
    ESP_LOGI(MOUSE_TAG, "蓝牙鼠标销毁完成");
}

void BluetoothMouse::Move(int8_t x, int8_t y)
{
    if (!connected_) {
        return;
    }
    SendMouseReport(0, x, y, 0);
}

void BluetoothMouse::Scroll(int8_t wheel)
{
    if (!connected_) {
        return;
    }
    SendMouseReport(0, 0, 0, wheel);
}

void BluetoothMouse::LeftClick()
{
    LeftPress();
    vTaskDelay(pdMS_TO_TICKS(50));
    LeftRelease();
}

void BluetoothMouse::RightClick()
{
    RightPress();
    vTaskDelay(pdMS_TO_TICKS(50));
    RightRelease();
}

void BluetoothMouse::MiddleClick()
{
    MiddlePress();
    vTaskDelay(pdMS_TO_TICKS(50));
    MiddleRelease();
}

void BluetoothMouse::LeftPress()
{
    if (!connected_) return;
    SendMouseReport(0x01, 0, 0, 0);
}

void BluetoothMouse::LeftRelease()
{
    if (!connected_) return;
    SendMouseReport(0x00, 0, 0, 0);
}

void BluetoothMouse::RightPress()
{
    if (!connected_) return;
    SendMouseReport(0x02, 0, 0, 0);
}

void BluetoothMouse::RightRelease()
{
    if (!connected_) return;
    SendMouseReport(0x00, 0, 0, 0);
}

void BluetoothMouse::MiddlePress()
{
    if (!connected_) return;
    SendMouseReport(0x04, 0, 0, 0);
}

void BluetoothMouse::MiddleRelease()
{
    if (!connected_) return;
    SendMouseReport(0x00, 0, 0, 0);
}

void BluetoothMouse::SetDeviceName(const char* name)
{
    if (name && strlen(name) < sizeof(device_name_)) {
        strcpy(device_name_, name);
        if (initialized_) {
            esp_bt_dev_set_device_name(device_name_);
        }
    }
}

void BluetoothMouse::SendMouseReport(uint8_t buttons, int8_t x, int8_t y, int8_t wheel)
{
    if (!connected_) {
        return;
    }

    uint8_t report[4] = {buttons, (uint8_t)x, (uint8_t)y, (uint8_t)wheel};
    esp_bt_hidd_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x00, sizeof(report), report);
}

void BluetoothMouse::GapEventHandler(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(MOUSE_TAG, "认证成功: %s", param->auth_cmpl.device_name);
            } else {
                ESP_LOGE(MOUSE_TAG, "认证失败，状态: %d", param->auth_cmpl.stat);
            }
            break;
        case ESP_BT_GAP_PIN_REQ_EVT:
            ESP_LOGI(MOUSE_TAG, "PIN码请求");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 0, pin_code);
            break;
        default:
            break;
    }
}

void BluetoothMouse::HiddEventHandler(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    if (!instance_) return;
    
    switch (event) {
        case ESP_HIDD_INIT_FINISH_EVT:
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                ESP_LOGI(MOUSE_TAG, "HID设备初始化完成");
            } else {
                ESP_LOGE(MOUSE_TAG, "HID设备初始化失败");
            }
            break;
        case ESP_HIDD_CONNECT_EVT:
            ESP_LOGI(MOUSE_TAG, "HID设备已连接");
            instance_->connected_ = true;
            break;
        case ESP_HIDD_DISCONNECT_EVT:
            ESP_LOGI(MOUSE_TAG, "HID设备已断开连接");
            instance_->connected_ = false;
            // 重新设置为可发现状态
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            break;
        case ESP_HIDD_DEINIT_FINISH_EVT:
            ESP_LOGI(MOUSE_TAG, "HID设备销毁完成");
            break;
        default:
            break;
    }
}
