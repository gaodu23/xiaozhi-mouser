# Gezipai 开发板

## 功能特性

- **音频编解码**: ES8311音频编解码器，支持16KHz采样率
- **蓝牙鼠标**: 内置蓝牙HID鼠标功能，开机自启动
- **WiFi连接**: 支持WiFi无线网络连接
- **LED指示**: 内置LED指示灯

## 蓝牙鼠标功能

Gezipai开发板集成了完整的蓝牙鼠标功能：

### 特性
- 开机自动启动蓝牙鼠标服务
- 设备名称：`Gezipai Mouse`
- 支持标准HID鼠标协议
- 支持鼠标移动、点击、滚轮操作

### 使用方法
1. 开发板启动后，蓝牙鼠标自动初始化
2. 在目标设备上搜索并连接 "Gezipai Mouse"
3. 通过代码控制鼠标操作

### API接口
```cpp
// 获取蓝牙鼠标实例
BluetoothMouse* mouse = board.GetBluetoothMouse();

// 鼠标移动
mouse->Move(x, y);

// 鼠标点击
mouse->LeftClick();
mouse->RightClick();
mouse->MiddleClick();

// 鼠标按键按下/释放
mouse->LeftPress();
mouse->LeftRelease();

// 滚轮操作
mouse->Scroll(1);  // 向上滚动
mouse->Scroll(-1); // 向下滚动

// 检查连接状态
bool connected = mouse->IsConnected();
```

## 编译配置命令

**配置编译目标为 ESP32S3：**

```bash
idf.py set-target esp32s3
```

**打开 menuconfig：**

```bash
idf.py menuconfig
```

**选择板子：**

```
Xiaozhi Assistant -> Board Type -> gezipai
```

```

**编译：**
    
bash
idf.py build
```

**下载：**
idf.py build flash monitor

进行下载和显示日志


**固件生成：**

```bash
idf.py merge-bin
```
