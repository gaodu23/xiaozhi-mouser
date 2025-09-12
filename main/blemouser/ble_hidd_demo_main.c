/*
 * ESP-IDF BLE HID 体感鼠标示例
 * 基于QMI8658陀螺仪传感器实现手势控制鼠标移动
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "hid_dev.h"
#include "qmi8658.h"

// // 硬件配置
// #define SDA_PIN 11
// #define SCL_PIN 12
// #define BUTTON_ADC_CHANNEL ADC_CHANNEL_8  // GPIO9对应ADC1_CHANNEL_8 (ESP32S3)

// 体感鼠标参数
#define MOUSE_SENSITIVITY 0.1f
#define GYRO_THRESHOLD 0.8f
#define MOUSE_MAX_MOVE 10
#define MOUSE_SAMPLE_RATE 20

// 按键检测参数
#define BUTTON_DEBOUNCE_TIME 50
#define ADC_SAMPLES 5
#define BUTTON_IDLE_MIN 3900
#define BUTTON_UP_MAX 200
#define BUTTON_RIGHT_MIN 200
#define BUTTON_RIGHT_MAX 600
#define BUTTON_LEFT_MIN 1600
#define BUTTON_LEFT_MAX 2100
#define BUTTON_DOWN_MIN 2400
#define BUTTON_DOWN_MAX 2900

// 按键状态定义
typedef enum {
    BUTTON_STATE_IDLE = 0,
    BUTTON_STATE_LEFT = 1,
    BUTTON_STATE_RIGHT = 2,
    BUTTON_STATE_VOLUME_UP = 3,
    BUTTON_STATE_VOLUME_DOWN = 4
} button_state_t;

#define HID_DEMO_TAG "MOUSER"

// 全局变量
static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
static qmi8658_handle_t imu_sensor;
static bool imu_initialized = false;
static TaskHandle_t mouse_task_handle = NULL;


// 按键相关变量
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static button_state_t current_button_state = BUTTON_STATE_IDLE;
static button_state_t last_button_state = BUTTON_STATE_IDLE;
static uint32_t button_state_start_time = 0;

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME "MOU"
static uint8_t hidd_service_uuid128[] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x03C2,           // HID鼠标设备
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x30,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


// HID设备事件回调函数
static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH:
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);
            }
            break;
        case ESP_HIDD_EVENT_BLE_CONNECT:
            hid_conn_id = param->connect.conn_id;
            break;
        case ESP_HIDD_EVENT_BLE_DISCONNECT:
            sec_conn = false;
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        default:
            break;
    }
}

// GAP事件处理函数
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = param->ble_security.auth_cmpl.success;
        if (sec_conn) {
            ESP_LOGI(HID_DEMO_TAG, "BLE连接已建立");
        }
        break;
    default:
        break;
    }
}

// 初始化IMU传感器
bool init_imu_sensor() {
    if (!qmi8658_init(&imu_sensor, I2C_NUM_0, SDA_PIN, SCL_PIN, QMI8658_ADDRESS_LOW)) {
        ESP_LOGE(HID_DEMO_TAG, "IMU传感器初始化失败");
        return false;
    }
    
    // 配置传感器
    qmi8658_set_gyro_range(&imu_sensor, QMI8658_GYRO_RANGE_512DPS);
    qmi8658_set_gyro_odr(&imu_sensor, QMI8658_GYRO_ODR_125HZ);
    qmi8658_set_accel_range(&imu_sensor, QMI8658_ACCEL_RANGE_2G);
    qmi8658_set_accel_odr(&imu_sensor, QMI8658_ACCEL_ODR_125HZ);
    qmi8658_enable_sensors(&imu_sensor, QMI8658_ENABLE_GYRO | QMI8658_ENABLE_ACCEL);
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    ESP_LOGI(HID_DEMO_TAG, "IMU传感器初始化成功");
    return true;
}

// 初始化ADC用于按键检测
bool init_button_adc() {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BUTTON_ADC_CHANNEL, &config));

    // ADC校准（可选）
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);
    if (ret != ESP_OK) {
        adc1_cali_handle = NULL;
    }
    
    return true;
}

// 读取按键ADC值
uint32_t read_button_adc() {
    uint32_t sum = 0;
    int adc_raw;
    
    for (int i = 0; i < ADC_SAMPLES; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BUTTON_ADC_CHANNEL, &adc_raw));
        sum += adc_raw;
    }
    return sum / ADC_SAMPLES;
}

// 根据ADC值判断按键状态
button_state_t get_button_state_from_adc(uint32_t adc_value) {
    if (adc_value >= BUTTON_IDLE_MIN) {
        return BUTTON_STATE_IDLE;
    } else if (adc_value <= BUTTON_UP_MAX) {
        return BUTTON_STATE_VOLUME_UP;
    } else if (adc_value >= BUTTON_RIGHT_MIN && adc_value <= BUTTON_RIGHT_MAX) {
        return BUTTON_STATE_RIGHT;
    } else if (adc_value >= BUTTON_LEFT_MIN && adc_value <= BUTTON_LEFT_MAX) {
        return BUTTON_STATE_LEFT;
    } else if (adc_value >= BUTTON_DOWN_MIN && adc_value <= BUTTON_DOWN_MAX) {
        return BUTTON_STATE_VOLUME_DOWN;
    }
    
    return BUTTON_STATE_IDLE;
}

// 检测按键状态变化
button_state_t detect_button_press() {
    uint32_t adc_value = read_button_adc();
    button_state_t new_state = get_button_state_from_adc(adc_value);
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    if (new_state != current_button_state) {
        current_button_state = new_state;
        button_state_start_time = current_time;
        return BUTTON_STATE_IDLE; // 状态刚变化，还未稳定
    }
    
    // 检查状态是否已稳定足够时间
    if ((current_time - button_state_start_time) >= BUTTON_DEBOUNCE_TIME) {
        if (current_button_state != last_button_state) {
            last_button_state = current_button_state;
            return current_button_state;
        }
    }
    
    return BUTTON_STATE_IDLE;
}

static float constrain(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

void mouse_sensor_task(void *pvParameters) {
    float gyro_x, gyro_y, gyro_z;
    int8_t mouse_x, mouse_y;
    uint8_t mouse_buttons = 0;
    
    while (1) {
        if (sec_conn && imu_initialized) {
            // 读取陀螺仪数据
            if (qmi8658_read_gyro_dps(&imu_sensor, &gyro_x, &gyro_y, &gyro_z)) {
                float mouse_x_raw = gyro_y * MOUSE_SENSITIVITY;
                float mouse_y_raw = -gyro_x * MOUSE_SENSITIVITY;
                
                if (fabs(gyro_x) < GYRO_THRESHOLD) mouse_y_raw = 0;
                if (fabs(gyro_y) < GYRO_THRESHOLD) mouse_x_raw = 0;
                
                mouse_x = (int8_t)constrain(mouse_x_raw, -MOUSE_MAX_MOVE, MOUSE_MAX_MOVE);
                mouse_y = (int8_t)constrain(mouse_y_raw, -MOUSE_MAX_MOVE, MOUSE_MAX_MOVE);
            } else {
                mouse_x = 0;
                mouse_y = 0;
            }
            
            // 检测按键状态
            button_state_t button_pressed = detect_button_press();
            mouse_buttons = 0;
            
            switch (button_pressed) {
                case BUTTON_STATE_LEFT:
                    mouse_buttons = 0x01;
                    break;
                case BUTTON_STATE_RIGHT:
                    mouse_buttons = 0x02;
                    break;
                case BUTTON_STATE_VOLUME_UP:
                    esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, true);
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, false);
                    break;
                case BUTTON_STATE_VOLUME_DOWN:
                    esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, true);
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, false);
                    break;
                default:
                    break;
            }
            
            // 发送鼠标数据
            if (mouse_x != 0 || mouse_y != 0 || mouse_buttons != 0) {
                esp_hidd_send_mouse_value(hid_conn_id, mouse_buttons, mouse_x, mouse_y);
                
                if (mouse_buttons != 0) {
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    esp_hidd_send_mouse_value(hid_conn_id, 0, 0, 0);
                }
            }
        }
        
        vTaskDelay(MOUSE_SAMPLE_RATE / portTICK_PERIOD_MS);
    }
}

void hid_demo_task(void *pvParameters) {
    while(1) {
        // 简单的状态监控，仅在状态变化时输出
        static bool last_ready_state = false;
        bool current_ready = (sec_conn && imu_initialized);
        
        if (current_ready != last_ready_state) {
            if (current_ready) {
                ESP_LOGI(HID_DEMO_TAG, "体感鼠标已就绪");
            } else {
                ESP_LOGI(HID_DEMO_TAG, "等待连接或初始化...");
            }
            last_ready_state = current_ready;
        }
        
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}


void app_main(void) {
    esp_err_t ret;

    // 初始化NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化BLE
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_hidd_profile_init());

    // 注册回调函数
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    // 设置安全参数
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    // 初始化硬件
    init_button_adc();
    imu_initialized = init_imu_sensor();

    // 创建任务
    xTaskCreate(&mouse_sensor_task, "mouse_task", 4096, NULL, 6, &mouse_task_handle);
    xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 4, NULL);
    
    ESP_LOGI(HID_DEMO_TAG, "体感鼠标系统启动完成");
}
