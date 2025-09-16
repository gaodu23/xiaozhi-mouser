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
#define BUTTON_DEBOUNCE_TIME 50      /* 按键消抖时间(ms) */
#define ADC_SAMPLES 5                /* ADC采样次数 */
#define CALIBRATION_DURATION_MS 2000 /* 陀螺仪校准持续时间(ms) */

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

/* 陀螺仪校准变量 */
static bool gyro_calibrated = false; /* 陀螺仪校准标志 */
static float gyro_offset_x = 0;      /* X轴偏移量 */
static float gyro_offset_y = 0;      /* Y轴偏移量 */
static float gyro_offset_z = 0;      /* Z轴偏移量 */

/* 按键状态变量 */
static button_state_t current_button_state = BUTTON_STATE_IDLE; /* 当前按键状态 */
static button_state_t last_button_state = BUTTON_STATE_IDLE;    /* 上一次按键状态 */
static uint32_t button_state_start_time = 0;                    /* 按键状态开始时间 */

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
 * @brief 陀螺仪校准
 *
 * @return esp_err_t 校准结果
 */
static esp_err_t gyro_calibration()
{
    if (!imu_initialized || !imu_sensor)
        return ESP_ERR_INVALID_STATE;

    /* 采样并累计陀螺仪数据 */
    float sx = 0, sy = 0, sz = 0;
    int cnt = 0;
    uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start) < CALIBRATION_DURATION_MS)
    {
        float gx, gy, gz;
        if (qmi8658_read_gyro_rads(imu_sensor, &gx, &gy, &gz) == ESP_OK)
        {
            sx += gx;
            sy += gy;
            sz += gz;
            cnt++;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    /* 检查是否获得足够的样本 */
    if (cnt < 10)
        return ESP_ERR_INVALID_RESPONSE;

    /* 计算偏移量 */
    gyro_offset_x = sx / cnt;
    gyro_offset_y = sy / cnt;
    gyro_offset_z = sz / cnt;
    gyro_calibrated = true;

    return ESP_OK;
}

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

    /* 配置传感器参数 */
    qmi8658_set_gyro_range(imu_sensor, QMI8658_GYRO_RANGE_256DPS);
    qmi8658_set_gyro_odr(imu_sensor, QMI8658_GYRO_ODR_250HZ);
    qmi8658_set_accel_range(imu_sensor, QMI8658_ACCEL_RANGE_4G);
    qmi8658_set_accel_odr(imu_sensor, QMI8658_ACCEL_ODR_250HZ);
    qmi8658_enable_sensors(imu_sensor, QMI8658_ENABLE_GYRO | QMI8658_ENABLE_ACCEL);

    /* 如果未校准，进行陀螺仪校准 */
    if (!gyro_calibrated)
    {
        if (gyro_calibration() != ESP_OK)
        {
            gyro_offset_x = gyro_offset_y = gyro_offset_z = 0;
            gyro_calibrated = true;
        }
    }
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

    /* 主循环 */
    while (1)
    {
        /* 检查IMU初始化状态 */
        if (!imu_initialized || !imu_sensor)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        /* 读取陀螺仪和加速度计数据 */
        float gx, gy, gz, ax, ay, az;
        esp_err_t gyro_err = qmi8658_read_gyro_rads(imu_sensor, &gx, &gy, &gz);
        esp_err_t accel_err = qmi8658_read_accel_mg(imu_sensor, &ax, &ay, &az);
        
        if (gyro_err != ESP_OK || accel_err != ESP_OK)
        {
            ESP_LOGE(TAG, "传感器读取失败 - 陀螺仪: %d, 加速度: %d", gyro_err, accel_err);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        
        // 每5秒输出一次传感器数据，避免日志过多
        static uint32_t last_log_time = 0;
        uint32_t current_log_time = esp_log_timestamp();
        if (current_log_time - last_log_time > 5000) {
            ESP_LOGI(TAG, "传感器数据 - 陀螺仪(rad/s): X=%.3f Y=%.3f Z=%.3f", gx, gy, gz);
            ESP_LOGI(TAG, "传感器数据 - 加速度(mg): X=%.1f Y=%.1f Z=%.1f", ax, ay, az);
            last_log_time = current_log_time;
        }

        /* 应用陀螺仪校准偏移量 */
        if (gyro_calibrated)
        {
            gx -= gyro_offset_x;
            gy -= gyro_offset_y;
            gz -= gyro_offset_z;
        }

        /* 计算鼠标移动值 */
        int8_t mx = (fabs(gx) > g_config.gyro_threshold) ? (int8_t)(gx * g_config.mouse_sensitivity) : 0;
        int8_t my = (fabs(gz) > g_config.gyro_threshold) ? (int8_t)(gz * g_config.mouse_sensitivity) : 0;

        /* 每5秒输出一次阈值和计算结果 */
        if (current_log_time - last_log_time > 5000) {
            ESP_LOGI(TAG, "阈值判断 - 阈值=%.3f, X轴=%.3f, Z轴=%.3f", 
                    g_config.gyro_threshold, fabs(gx), fabs(gz));
            ESP_LOGI(TAG, "鼠标移动值 - X=%d, Y=%d, 灵敏度=%.2f", 
                    mx, my, g_config.mouse_sensitivity);
        }

        /* 读取按键状态并处理消抖 */
        button_state_t new_state = read_button_state();
        uint32_t now_time = esp_log_timestamp();
        if (new_state != current_button_state && now_time - button_state_start_time > BUTTON_DEBOUNCE_TIME)
        {
            last_button_state = current_button_state;
            current_button_state = new_state;
            button_state_start_time = now_time;
        }

        /* 发送HID报告 */
        if (sec_conn)
        {
            /* 处理按键状态 - 左键和右键按下期间都保持按下状态，可以实现拖拽 */
            uint8_t buttons = 0;
            if (current_button_state == BUTTON_STATE_LEFT)
                buttons = 1; // 左键按下
            else if (current_button_state == BUTTON_STATE_RIGHT)
                buttons = 2; // 右键按下

            /* 滚轮事件处理 - 支持单击和长按 */
            static uint32_t last_wheel_time = 0;
            uint32_t current_time = esp_log_timestamp();

            /* 滚轮向上滚动 */
            if (current_button_state == BUTTON_STATE_SCROLL_UP)
            {
                /* 状态变化时立即触发一次滚动 */
                if (current_button_state != last_button_state)
                {
                    esp_hidd_send_mouse_value(hid_conn_id, 0, 0, 0, 1);
                    last_wheel_time = current_time;
                }
                /* 长按时，每隔200ms触发一次滚动 */
                else if (current_time - last_wheel_time > 200)
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
                /* 长按时，每隔200ms触发一次滚动 */
                else if (current_time - last_wheel_time > 200)
                {
                    esp_hidd_send_mouse_value(hid_conn_id, 0, 0, 0, -1);
                    last_wheel_time = current_time;
                }
            }

            /* 鼠标移动和按键事件始终发送，支持拖拽操作 */
            esp_hidd_send_mouse_value(hid_conn_id, buttons, mx, my, 0);
            
            /* 每5秒输出一次发送状态 */
            if (current_log_time - last_log_time > 5000) {
                ESP_LOGI(TAG, "HID发送 - 按键=%d, 移动: X=%d, Y=%d", buttons, mx, my);
            }
        } else {
            /* 每5秒输出一次连接状态 */
            if (current_log_time - last_log_time > 5000) {
                ESP_LOGI(TAG, "未连接 - 安全连接标志=%d, 连接ID=%d", sec_conn, hid_conn_id);
            }
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
    if (init_imu_sensor()) {
        ESP_LOGI(TAG, "IMU传感器初始化成功");
    } else {
        ESP_LOGE(TAG, "IMU传感器初始化失败");
    }

    /* 创建鼠标任务 */
    ESP_LOGI(TAG, "创建鼠标传感器任务");
    xTaskCreate(mouse_sensor_task, "mouse_task", 4096, NULL, 6, &mouse_task_handle);

    return ESP_OK;
}
