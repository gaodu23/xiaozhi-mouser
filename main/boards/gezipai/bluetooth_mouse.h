#ifndef _BLUETOOTH_MOUSE_H_
#define _BLUETOOTH_MOUSE_H_

#include <esp_log.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#include <esp_bt_device.h>
#include <esp_hidd_prf_api.h>

class BluetoothMouse 
{
public:
    BluetoothMouse();
    ~BluetoothMouse();
    
    // 初始化蓝牙鼠标
    bool Initialize();
    
    // 销毁蓝牙鼠标
    void Deinitialize();
    
    // 鼠标移动 (相对移动)
    void Move(int8_t x, int8_t y);
    
    // 鼠标滚轮
    void Scroll(int8_t wheel);
    
    // 鼠标按键
    void LeftClick();
    void RightClick();
    void MiddleClick();
    void LeftPress();
    void LeftRelease();
    void RightPress();
    void RightRelease();
    void MiddlePress();
    void MiddleRelease();
    
    // 获取连接状态
    bool IsConnected() const { return connected_; }
    
    // 设置设备名称
    void SetDeviceName(const char* name);

private:
    static void GapEventHandler(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
    static void HiddEventHandler(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);
    
    // 发送鼠标报告
    void SendMouseReport(uint8_t buttons, int8_t x, int8_t y, int8_t wheel);
    
    bool initialized_;
    bool connected_;
    char device_name_[32];
    
    // HID报告描述符 - 鼠标
    static const uint8_t hid_mouse_descriptor_[];
    static BluetoothMouse* instance_;
};

#endif // _BLUETOOTH_MOUSE_H_
