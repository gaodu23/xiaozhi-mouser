/*
 * ESP-IDF BLE HID 体感鼠标 - 单一模式精简版
 */

/* 系统头文件 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* FreeRTOS头文件 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ESP系统头文件 */
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

/* BLE相关头文件 */
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_hidd_prf_api.h"
#include "hid_dev.h"

/* 驱动程序头文件 */
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* 传感器头文件 */
#include "qmi8658.h"

/* 项目头文件 */
#include "ble_hid_mouse.h"
#include "boards/gezipai/config.h"

/* 日志标签 */
static const char *TAG = "BLE_HID_MOUSE";

/* 按键和ADC相关参数 */
#define BUTTON_DEBOUNCE_TIME 30      /* 从50ms减少到30ms */
#define ADC_SAMPLES 5                /* ADC采样次数 */

/* 按键ADC阈值 */
#define BUTTON_RIGHT_MIN 0
#define BUTTON_RIGHT_MAX 100

#define BUTTON_UP_MIN 200
#define BUTTON_UP_MAX 500

#define BUTTON_LEFT_MIN 1500
#define BUTTON_LEFT_MAX 2100

#define BUTTON_DOWN_MIN 2300
#define BUTTON_DOWN_MAX 2900

#define BUTTON_IDLE_MIN 3800
#define BUTTON_IDLE_MAX 5000

/* 按键状态定义 */
typedef enum
{
    BUTTON_STATE_IDLE = 0,   /* 空闲状态 */
    BUTTON_STATE_LEFT,       /* 左键按下 */
    BUTTON_STATE_RIGHT,      /* 右键按下 */
    BUTTON_STATE_SCROLL_UP,  /* 滚轮向上 */
    BUTTON_STATE_SCROLL_DOWN /* 滚轮向下 */
} button_state_t;

/* 全局变量 */
static ble_hid_mouse_config_t g_config; /* 鼠标配置 */
static uint16_t hid_conn_id = 0;        /* HID连接ID */
static bool sec_conn = false;           /* 安全连接标志 */

/* I2C和IMU相关变量 */
static i2c_master_bus_handle_t i2c_master_bus = NULL; /* I2C总线句柄 */
static qmi8658_handle_t *imu_sensor = NULL;           /* IMU传感器句柄 */
static bool imu_initialized = false;                  /* IMU初始化标志 */

/* 任务句柄 */
static TaskHandle_t mouse_task_handle = NULL; /* 鼠标任务句柄 */

/* ADC相关变量 */
static adc_oneshot_unit_handle_t adc1_handle; /* ADC单次转换句柄 */
static bool adc_initialized = false;          /* ADC初始化标志 */

/* 按键状态变量 */
static button_state_t current_button_state = BUTTON_STATE_IDLE; /* 当前按键状态 */
static button_state_t last_button_state = BUTTON_STATE_IDLE;    /* 上一次按键状态 */
static uint32_t button_state_start_time = 0;                    /* 按键状态开始时间 */

/* 卡尔曼滤波器结构定义 */
typedef struct {
    float x;      /* 状态估计 */
    float p;      /* 估计误差协方差 */
    float q;      /* 过程噪声协方差 */
    float r;      /* 测量噪声协方差 */
    float k;      /* 卡尔曼增益 */
} kalman_filter_t;

/**
 * @brief 初始化卡尔曼滤波器
 *
 * @param filter 卡尔曼滤波器指针
 * @param q 过程噪声协方差
 * @param r 测量噪声协方差
 * @param p 初始估计误差协方差
 * @param initial_value 初始状态估计值
 */
static void kalman_init(kalman_filter_t *filter, float q, float r, float p, float initial_value)
{
    filter->x = initial_value;
    filter->p = p;
    filter->q = q;
    filter->r = r;
}

/**
 * @brief 卡尔曼滤波器更新
 *
 * @param filter 卡尔曼滤波器指针
 * @param measurement 测量值
 * @return float 滤波后的估计值
 */
static float kalman_update(kalman_filter_t *filter, float measurement)
{
    /* 预测步骤 */
    filter->p = filter->p + filter->q;
    
    /* 更新步骤 */
    filter->k = filter->p / (filter->p + filter->r);
    filter->x = filter->x + filter->k * (measurement - filter->x);
    filter->p = (1 - filter->k) * filter->p;
    
    return filter->x;
}

/* 函数声明 */
static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

/* BLE HID服务UUID */
static uint8_t hidd_service_uuid128[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0xA1};

/* BLE广播数据 */
static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x03C2, /* HID Mouse appearance */
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = hidd_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)};

/* BLE广播参数 */
static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min = 0x40,
    .adv_int_max = 0x60,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY};



/**
 * @brief 初始化IMU传感器
 */
static bool init_imu_sensor()
{
    /* 分配IMU传感器内存 */
    if (imu_sensor == NULL)
    {
        imu_sensor = malloc(sizeof(qmi8658_handle_t));
        if (!imu_sensor)
        {
            ESP_LOGE(TAG, "分配IMU失败");
            return false;
        }
    }

    /* 初始化I2C总线 */
    if (i2c_master_bus == NULL)
    {
        i2c_master_bus_config_t cfg = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_NUM_1,
            .scl_io_num = QMI8658_I2C_SCL_PIN,
            .sda_io_num = QMI8658_I2C_SDA_PIN,
            .flags.enable_internal_pullup = true};

        if (i2c_new_master_bus(&cfg, &i2c_master_bus) != ESP_OK)
        {
            ESP_LOGE(TAG, "I2C失败");
            return false;
        }
    }

    /* 初始化QMI8658传感器 */
    if (!qmi8658_init(imu_sensor, i2c_master_bus, QMI8658_ADDRESS_LOW))
    {
        ESP_LOGE(TAG, "QMI8658初始化失败");
        return false;
    }

    /* 配置传感器参数 - 提高采样率 */
    qmi8658_set_gyro_range(imu_sensor, QMI8658_GYRO_RANGE_256DPS);
    qmi8658_set_gyro_odr(imu_sensor, QMI8658_GYRO_ODR_500HZ); // 提高到500Hz
    qmi8658_set_accel_range(imu_sensor, QMI8658_ACCEL_RANGE_4G);
    qmi8658_set_accel_odr(imu_sensor, QMI8658_ACCEL_ODR_500HZ); // 提高到500Hz
    qmi8658_enable_sensors(imu_sensor, QMI8658_ENABLE_GYRO | QMI8658_ENABLE_ACCEL);

    imu_initialized = true;
    return true;
}

/**
 * @brief 初始化按钮ADC
 */
static void init_button_adc()
{
    if (adc_initialized)
    {
        return;
    }

    /* 初始化ADC单次转换单元 */
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));

    /* 配置ADC通道 */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BUTTON_ADC_CHANNEL, &chan_cfg));

    adc_initialized = true;
}

/**
 * @brief 读取按钮状态
 *
 * @return button_state_t 按钮状态
 */
static button_state_t read_button_state()
{
    /* 对ADC值进行多次采样并取平均值 */
    int sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++)
    {
        int raw;
        if (adc_oneshot_read(adc1_handle, BUTTON_ADC_CHANNEL, &raw) == ESP_OK)
            sum += raw;
        vTaskDelay(1);
    }
    int adc_raw = sum / ADC_SAMPLES;

    /* 根据ADC值确定按钮状态 */
    button_state_t st;
    if (adc_raw > BUTTON_IDLE_MIN)
        st = BUTTON_STATE_IDLE;
    else if (adc_raw <= BUTTON_UP_MAX)
        st = BUTTON_STATE_SCROLL_UP;
    else if (adc_raw >= BUTTON_RIGHT_MIN && adc_raw <= BUTTON_RIGHT_MAX)
        st = BUTTON_STATE_RIGHT;
    else if (adc_raw >= BUTTON_LEFT_MIN && adc_raw <= BUTTON_LEFT_MAX)
        st = BUTTON_STATE_LEFT;
    else if (adc_raw >= BUTTON_DOWN_MIN && adc_raw <= BUTTON_DOWN_MAX)
        st = BUTTON_STATE_SCROLL_DOWN;
    else
        st = BUTTON_STATE_IDLE;

    return st;
}

/**
 * @brief 鼠标传感器任务
 *
 * @param arg 任务参数
 */
static void mouse_sensor_task(void *arg)
{
    /* 初始化卡尔曼滤波器 */
    kalman_filter_t kalman_x, kalman_y, kalman_z;
    kalman_init(&kalman_x, 0.01, 0.1, 1.0, 0.0); /* X轴滤波器 - 实际为YAW */
    kalman_init(&kalman_y, 0.01, 0.1, 1.0, 0.0); /* Y轴滤波器 */
    kalman_init(&kalman_z, 0.01, 0.1, 1.0, 0.0); /* Z轴滤波器 - 实际为PITCH */
    
    /* 跟踪角度变化 */
    float angle_x = 0.0f, angle_y = 0.0f, angle_z = 0.0f;
    float last_angle_x = 0.0f, last_angle_z = 0.0f;
    
    /* 调试信息计时 */
    uint32_t last_debug_time = 0;
    
    while (1)
    {
        if (sec_conn)
        {
            /* 读取陀螺仪和加速度计数据 */
            float gx, gy, gz;
            float ax, ay, az;
            
            qmi8658_read_gyro_dps(imu_sensor, &gx, &gy, &gz);
            qmi8658_read_accel_mg(imu_sensor, &ax, &ay, &az);

            /* 计算时间增量(秒) */
            float dt = g_config.sample_rate_ms / 1000.0f;
            
            /* 更新角度 - 通过积分角速度（坐标系已调整） */
            angle_x += gx * dt;  // 实际是YAW (原来的ROLL)
            angle_y += gy * dt;  // 保持不变
            angle_z += gz * dt;  // 实际是PITCH (原来的YAW)
            
            /* 使用加速度计数据计算姿态角 (调整为正确的物理意义) */
            // 因为IMU安装方向不同，需要调整加速度计数据的解释
            float accel_angle_x = atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / M_PI;  // 实际是YAW
            float accel_angle_z = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI; // 实际是PITCH
            
            /* 互补滤波融合数据 - 加速度计只能校正Roll和Pitch */
            float yaw = 0.98f * angle_x + 0.02f * accel_angle_x;   // 实际是YAW (原来的Roll)
            float pitch = 0.98f * angle_z + 0.02f * accel_angle_z; // 实际是PITCH (原来的Yaw)
            
            /* 卡尔曼滤波融合 */
            float filtered_yaw = kalman_update(&kalman_x, yaw);     // 实际是YAW
            float filtered_pitch = kalman_update(&kalman_z, pitch); // 实际是PITCH
            
            /* 计算角度变化 */
            float delta_yaw = filtered_yaw - last_angle_x;    // YAW变化
            float delta_pitch = filtered_pitch - last_angle_z; // PITCH变化
            
            /* 更新上次角度 */
            last_angle_x = filtered_yaw;   // 保存YAW
            last_angle_z = filtered_pitch; // 保存PITCH
            
            /* 将角度变化转换为鼠标移动 - pitch控制鼠标上下(Y轴)，yaw控制鼠标左右(X轴) */
            int16_t mx = 0, my = 0;
            
            // YAW控制鼠标左右移动(X轴)
            if (fabs(delta_yaw) > g_config.gyro_threshold) {
                mx = (int16_t)(delta_yaw * g_config.mouse_sensitivity);
            }
            
            // PITCH控制鼠标上下移动(Y轴)
            if (fabs(delta_pitch) > g_config.gyro_threshold) {
                my = (int16_t)(delta_pitch * g_config.mouse_sensitivity);
            }
            
            /* 每1秒输出一次详细调试信息 */
            uint32_t current_time = esp_log_timestamp();
            if (current_time - last_debug_time > 1000)
            {
                ESP_LOGI(TAG, "====== 鼠标传感器调试信息 ======");
                ESP_LOGI(TAG, "滤波后角度(deg): YAW=%.2f PITCH=%.2f", 
                         filtered_yaw, filtered_pitch);
                ESP_LOGI(TAG, "角度变化: dYAW=%.2f dPITCH=%.2f", delta_yaw, delta_pitch);
                ESP_LOGI(TAG, "鼠标移动: X=%d Y=%d", mx, my);
                ESP_LOGI(TAG, "================================");
                
                last_debug_time = current_time;
            }

            /* 以下代码保持不变 */
            /* 读取按键状态并处理消抖 */
            button_state_t new_state = read_button_state();
            uint32_t now_time = esp_log_timestamp();
            if (new_state != current_button_state && now_time - button_state_start_time > BUTTON_DEBOUNCE_TIME)
            {
                last_button_state = current_button_state;
                current_button_state = new_state;
                button_state_start_time = now_time;
            }

            /* 处理按键状态 - 左键和右键按下期间都保持按下状态，可以实现拖拽 */
            uint8_t buttons = 0;
            if (current_button_state == BUTTON_STATE_LEFT)
                buttons = 1; // 左键按下
            else if (current_button_state == BUTTON_STATE_RIGHT)
                buttons = 2; // 右键按下

            /* 滚轮事件处理 - 支持单击和长按 */
            static uint32_t last_wheel_time = 0;
            /* 使用前面已定义的current_time变量 */

            /* 滚轮向上滚动 */
            if (current_button_state == BUTTON_STATE_SCROLL_UP)
            {
                /* 状态变化时立即触发一次滚动 */
                if (current_button_state != last_button_state)
                {
                    esp_hidd_send_mouse_value(hid_conn_id, 0, 0, 0, 1);
                    last_wheel_time = current_time;
                }
                /* 长按时，每隔500ms触发一次滚动 */
                else if (current_time - last_wheel_time > 500)
                {
                    esp_hidd_send_mouse_value(hid_conn_id, 0, 0, 0, 1);
                    last_wheel_time = current_time;
                }
            }
            /* 滚轮向下滚动 */
            else if (current_button_state == BUTTON_STATE_SCROLL_DOWN)
            {
                /* 状态变化时立即触发一次滚动 */
                if (current_button_state != last_button_state)
                {
                    esp_hidd_send_mouse_value(hid_conn_id, 0, 0, 0, -1);
                    last_wheel_time = current_time;
                }
                /* 长按时，每隔500ms触发一次滚动 */
                else if (current_time - last_wheel_time > 500)
                {
                    esp_hidd_send_mouse_value(hid_conn_id, 0, 0, 0, -1);
                    last_wheel_time = current_time;
                }
            }

            static uint32_t last_time = 0;
            uint32_t currenttime = esp_log_timestamp();

            if (currenttime - last_time > g_config.sample_rate_ms)
            {
                /* 鼠标移动和按键事件始终发送，支持拖拽操作 */
                esp_hidd_send_mouse_value(hid_conn_id, buttons, mx, my, 0);
                last_time = currenttime;
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(5)); // 从10ms减少到5ms，更快检测连接
        }
    }
}

/**
 * @brief GAP事件处理回调
 *
 * @param event GAP事件类型
 * @param param 事件参数
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        /* 广播数据设置完成，开始广播 */
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        /* 检查广播启动是否成功 */
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "广播启动失败 %d", param->adv_start_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_SEC_REQ_EVT:
        /* 响应安全请求 */
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        /* 响应密钥请求 */
        esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, 0);
        break;

    case ESP_GAP_BLE_NC_REQ_EVT:
        /* 响应数字比较请求 */
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        /* 认证完成事件处理 */
        if (param->ble_security.auth_cmpl.success)
        {
            ESP_LOGI(TAG, "蓝牙认证成功，设置安全连接标志");
            sec_conn = true;
        }
        else
        {
            ESP_LOGE(TAG, "蓝牙认证失败，断开连接");
            sec_conn = false;
            esp_ble_gap_disconnect(param->ble_security.auth_cmpl.bd_addr);
        }
        break;

    default:
        break;
    }
}

/**
 * @brief HID事件回调
 *
 * @param event HID事件类型
 * @param param 事件参数
 */
static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch (event)
    {
    case ESP_HIDD_EVENT_REG_FINISH:
        /* HID设备注册完成 */
        if (param->init_finish.state == ESP_HIDD_INIT_OK)
        {
            /* 设置设备名称并配置广播数据 */
            esp_ble_gap_set_device_name(g_config.device_name);
            esp_ble_gap_config_adv_data(&hidd_adv_data);
        }
        break;

    case ESP_HIDD_EVENT_BLE_CONNECT:
        /* BLE连接事件 */
        ESP_LOGI(TAG, "BLE已连接 - 连接ID: %d", param->connect.conn_id);
        hid_conn_id = param->connect.conn_id;
        sec_conn = false;
        break;

    case ESP_HIDD_EVENT_BLE_DISCONNECT:
        /* BLE断开连接事件 */
        ESP_LOGI(TAG, "BLE已断开连接");
        sec_conn = false;
        hid_conn_id = 0;
        /* 重新开始广播 */
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;

    default:
        break;
    }
}

/**
 * @brief 初始化BLE HID鼠标
 *
 * @param config 鼠标配置参数
 * @return esp_err_t 操作结果
 */
esp_err_t ble_hid_mouse_init(const ble_hid_mouse_config_t *config)
{
    ESP_LOGI(TAG, "初始化BLE HID鼠标 - 设备名称: %s", config->device_name);

    /* 保存配置 */
    memcpy(&g_config, config, sizeof(ble_hid_mouse_config_t));

    ESP_LOGI(TAG, "鼠标配置 - 灵敏度: %.2f, 阈值: %.2f",
             g_config.mouse_sensitivity, g_config.gyro_threshold);
    /* 初始化NVS闪存 */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 释放经典蓝牙内存 */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    /* 初始化蓝牙控制器 */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    /* 初始化蓝牙协议栈 */
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    /* 初始化HID配置 */
    ESP_ERROR_CHECK(esp_hidd_profile_init());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_hidd_register_callbacks(hidd_event_callback));

    /* 设置安全参数 */
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    /* 初始化按钮ADC */
    init_button_adc();

    /* 初始化IMU传感器 */
    if (init_imu_sensor())
    {
        ESP_LOGI(TAG, "IMU传感器初始化成功");
    }
    else
    {
        ESP_LOGE(TAG, "IMU传感器初始化失败");
    }

    /* 创建鼠标任务 - 提高优先级 */
    ESP_LOGI(TAG, "创建鼠标传感器任务");
    xTaskCreate(mouse_sensor_task, "mouse_task", 4096, NULL, 10, &mouse_task_handle);

    return ESP_OK;
}
