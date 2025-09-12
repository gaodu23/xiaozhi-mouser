/*
 * 蓝牙鼠标使用示例
 * 
 * 这个文件展示了如何在Gezipai开发板上使用蓝牙鼠标功能
 */

#include "gezipai_board.cc"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 蓝牙鼠标测试任务
void bluetooth_mouse_test_task(void *parameter)
{
    GezipaiBoard* board = static_cast<GezipaiBoard*>(parameter);
    BluetoothMouse* mouse = board->GetBluetoothMouse();
    
    if (!mouse) {
        ESP_LOGE("MOUSE_TEST", "蓝牙鼠标未初始化");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI("MOUSE_TEST", "开始蓝牙鼠标测试...");
    
    while (true) {
        // 等待设备连接
        if (!mouse->IsConnected()) {
            ESP_LOGI("MOUSE_TEST", "等待蓝牙连接...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        ESP_LOGI("MOUSE_TEST", "设备已连接，开始测试鼠标功能");
        
        // 测试鼠标移动 - 画一个正方形
        ESP_LOGI("MOUSE_TEST", "测试鼠标移动 - 画正方形");
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 50; j++) {
                switch (i) {
                    case 0: mouse->Move(2, 0); break;  // 向右移动
                    case 1: mouse->Move(0, 2); break;  // 向下移动
                    case 2: mouse->Move(-2, 0); break; // 向左移动
                    case 3: mouse->Move(0, -2); break; // 向上移动
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // 测试鼠标点击
        ESP_LOGI("MOUSE_TEST", "测试左键点击");
        mouse->LeftClick();
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        ESP_LOGI("MOUSE_TEST", "测试右键点击");
        mouse->RightClick();
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        ESP_LOGI("MOUSE_TEST", "测试中键点击");
        mouse->MiddleClick();
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 测试滚轮
        ESP_LOGI("MOUSE_TEST", "测试滚轮向上");
        for (int i = 0; i < 5; i++) {
            mouse->Scroll(1);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        ESP_LOGI("MOUSE_TEST", "测试滚轮向下");
        for (int i = 0; i < 5; i++) {
            mouse->Scroll(-1);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        // 暂停10秒后重复测试
        ESP_LOGI("MOUSE_TEST", "测试完成，10秒后重复...");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// 主应用程序中使用蓝牙鼠标的示例
void app_main() 
{
    // 创建Gezipai开发板实例（蓝牙鼠标会自动启动）
    GezipaiBoard board;
    
    // 创建测试任务（可选）
    xTaskCreate(bluetooth_mouse_test_task, "mouse_test", 4096, &board, 5, NULL);
    
    // 主循环 - 在这里可以添加其他功能
    while (true) {
        BluetoothMouse* mouse = board.GetBluetoothMouse();
        if (mouse && mouse->IsConnected()) {
            ESP_LOGI("APP", "蓝牙鼠标已连接");
            
            // 在这里可以根据需要控制鼠标
            // 例如：
            // mouse->Move(10, 0);     // 向右移动10像素
            // mouse->LeftClick();     // 左键点击
            // mouse->Scroll(1);       // 向上滚动
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* 
 * 使用说明：
 * 
 * 1. 开发板启动后，蓝牙鼠标会自动初始化并开始广播
 * 2. 在电脑或手机上搜索"Gezipai Mouse"设备并连接
 * 3. 连接成功后，可以通过代码控制鼠标操作
 * 4. 支持的操作：
 *    - Move(x, y): 相对移动
 *    - LeftClick(), RightClick(), MiddleClick(): 鼠标点击
 *    - LeftPress(), LeftRelease(): 鼠标按下/释放
 *    - Scroll(wheel): 滚轮操作
 * 5. 通过IsConnected()检查连接状态
 */
