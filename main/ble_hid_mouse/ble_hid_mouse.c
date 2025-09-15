/*
 * ESP-IDF BLE HID 体感鼠标示例
 * 基于官方BLE HID示例和QMI8658陀螺仪传感器实现手势控制鼠标移动
 * 使用Bluedroid协议栈
 */

#include "ble_hid_mouse.h"
#include "esp_hidd_prf_api.h"
#include "hid_dev.h"
#include "boards/gezipai/config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_bt.h"

#include "qmi8658.h"
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
#include "driver/i2c_master.h"

// NVS存储的键
#define NVS_NAMESPACE "ble_mouse"
#define NVS_KEY_MODE "mode"
#define NVS_KEY_BOND_PC "bond_pc"
#define NVS_KEY_BOND_PHONE "bond_phone"
#define NVS_KEY_BOND_TABLET "bond_tablet"

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

// 陀螺仪校准参数
#define CALIBRATION_DURATION_MS 2000
#define CALIBRATION_SAMPLES 100

// 按键状态定义
typedef enum {
    BUTTON_STATE_IDLE = 0,
    BUTTON_STATE_LEFT = 1,
    BUTTON_STATE_RIGHT = 2,
    BUTTON_STATE_SCROLL_UP = 3,
    BUTTON_STATE_SCROLL_DOWN = 4
} button_state_t;

static const char *TAG = "BLE_HID_MOUSE";

// 不同模式的设备配置
typedef struct {
    char device_name[32];
    esp_mac_type_t mac_type;        // 使用系统MAC地址类型
    uint8_t service_uuid[16];
} mode_config_t;

static const mode_config_t mode_configs[BLE_MOUSE_MODE_MAX] = {
    [BLE_MOUSE_MODE_PC] = {
        .device_name = "XiaoZhi-Mouse-PC",
        .mac_type = ESP_MAC_WIFI_STA,    // PC模式使用WiFi STA MAC
        .service_uuid = {0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x01}
    },
    [BLE_MOUSE_MODE_PHONE] = {
        .device_name = "XiaoZhi-Mouse-Phone", 
        .mac_type = ESP_MAC_WIFI_SOFTAP, // 手机模式使用WiFi SoftAP MAC
        .service_uuid = {0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x02}
    },
    [BLE_MOUSE_MODE_TABLET] = {
        .device_name = "XiaoZhi-Mouse-Tablet",
        .mac_type = ESP_MAC_BT,          // 平板模式使用蓝牙 MAC
        .service_uuid = {0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x03}
    }
};

// 核心状态变量
static ble_hid_mouse_config_t g_config;
static bool g_initialized = false;
static bool g_started = false;
static uint16_t hid_conn_id = 0;
static bool sec_conn = false;

// 硬件设备句柄
static i2c_master_bus_handle_t i2c_master_bus = NULL;
static qmi8658_handle_t *imu_sensor = NULL;
static bool imu_initialized = false;
static TaskHandle_t mouse_task_handle = NULL;
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool adc_initialized = false;

// 陀螺仪校准和按键状态
static bool gyro_calibrated = false;
static float gyro_offset_x = 0.0f, gyro_offset_y = 0.0f, gyro_offset_z = 0.0f;
static button_state_t current_button_state = BUTTON_STATE_IDLE;
static button_state_t last_button_state = BUTTON_STATE_IDLE;
static uint32_t button_state_start_time = 0;

// 函数声明
static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static esp_err_t gyro_calibration(void);
static bool init_imu_sensor(void);
static void init_button_adc(void);
static button_state_t read_button_state(void);
static esp_err_t ble_hid_mouse_save_bond(esp_bd_addr_t bond_addr);
static esp_err_t ble_hid_mouse_load_and_manage_bonds(void);

// HID服务UUID - 动态设置
static uint8_t hidd_service_uuid128[16];

// BLE广播数据
static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x03C2,       // HID鼠标外观
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// BLE广播参数 - 优化为更稳定的连接参数
static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x40,     // 增加广播间隔，减少功耗，提高稳定性
    .adv_int_max        = 0x60,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// GAP事件处理程序
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "BLE广播启动失败: %d", param->adv_start_cmpl.status);
        } else {
            ESP_LOGI(TAG, "BLE广播启动成功，等待连接...");
        }
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        // 同意安全请求，开始配对过程
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        ESP_LOGI(TAG, "同意BLE安全请求");
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        ESP_LOGI(TAG, "GAP_PASSKEY_REQ_EVT");
        // 对于无输入输出设备，通常使用Just Works配对，密码为0
        esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, 0);
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        // 对于无输入输出设备，自动接受数字比较
        ESP_LOGI(TAG, "GAP_NC_REQ_EVT, 自动接受");
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        ESP_LOGI(TAG, "BLE认证完成，success: %d", param->ble_security.auth_cmpl.success);
        
        if (param->ble_security.auth_cmpl.success) {
            ESP_LOGI(TAG, "BLE安全连接建立成功");
            sec_conn = true;

            // 保存新的配对信息
            ble_hid_mouse_save_bond(param->ble_security.auth_cmpl.bd_addr);
        } else {
            ESP_LOGW(TAG, "BLE安全连接建立失败，失败原因: %d", param->ble_security.auth_cmpl.fail_reason);
            // 认证失败，确保断开连接
            sec_conn = false;
            esp_ble_gap_disconnect(param->ble_security.auth_cmpl.bd_addr);
        }
        break;
    case ESP_GAP_BLE_KEY_EVT:
        // 在配对过程中接收密钥
        ESP_LOGI(TAG, "GAP_KEY_EVT, key type: %d", param->ble_security.ble_key.key_type);
        break;
    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
        ESP_LOGI(TAG, "本地隐私设置完成, status: %d", param->local_privacy_cmpl.status);
        break;
    default:
        break;
    }
}

static bool init_imu_sensor() {
    if (imu_sensor == NULL) {
        imu_sensor = (qmi8658_handle_t *)malloc(sizeof(qmi8658_handle_t));
        if (imu_sensor == NULL) {
            ESP_LOGE(TAG, "无法分配QMI8658内存");
            return false;
        }
    }
    
    // 创建I2C总线
    if (i2c_master_bus == NULL) {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_NUM_1,
            .scl_io_num = QMI8658_I2C_SCL_PIN,
            .sda_io_num = QMI8658_I2C_SDA_PIN,
            .flags.enable_internal_pullup = true
        };
        
        if (i2c_new_master_bus(&i2c_bus_cfg, &i2c_master_bus) != ESP_OK) {
            ESP_LOGE(TAG, "I2C总线初始化失败");
            return false;
        }
    }
    
    if (!qmi8658_init(imu_sensor, i2c_master_bus, QMI8658_ADDRESS_LOW)) {
        ESP_LOGE(TAG, "QMI8658传感器初始化失败");
        return false;
    }
    
    // 配置传感器
    qmi8658_set_gyro_range(imu_sensor, QMI8658_GYRO_RANGE_128DPS);
    qmi8658_set_gyro_odr(imu_sensor, QMI8658_GYRO_ODR_125HZ);
    qmi8658_set_accel_range(imu_sensor, QMI8658_ACCEL_RANGE_2G);
    qmi8658_set_accel_odr(imu_sensor, QMI8658_ACCEL_ODR_125HZ);
    qmi8658_enable_sensors(imu_sensor, QMI8658_ENABLE_GYRO | QMI8658_ENABLE_ACCEL);
    
    vTaskDelay(pdMS_TO_TICKS(100));
    return true;
}

static void init_button_adc() {
    if (adc_initialized) {
        ESP_LOGI(TAG, "ADC已经初始化，跳过重复初始化");
        return;
    }
    
    adc_oneshot_unit_init_cfg_t init_config = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));
    
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BUTTON_ADC_CHANNEL, &config));
    
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle));
    
    adc_initialized = true;
    ESP_LOGI(TAG, "ADC初始化完成");
}

static button_state_t read_button_state() {
    int adc_sum = 0;
    
    for (int i = 0; i < ADC_SAMPLES; i++) {
        int adc_raw;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BUTTON_ADC_CHANNEL, &adc_raw));
        adc_sum += adc_raw;
        vTaskDelay(1);
    }
    int adc_raw = adc_sum / ADC_SAMPLES;
    
    // 每2秒打印一次ADC值，便于观察但不会产生太多日志
    static uint32_t last_print_time = 0;
    uint32_t current_time = esp_log_timestamp();
    if (current_time - last_print_time > 2000) {
        ESP_LOGI(TAG, "ADC当前值: %d", adc_raw);
        last_print_time = current_time;
    }
    
    button_state_t state;
    if (adc_raw > BUTTON_IDLE_MIN) state = BUTTON_STATE_IDLE;
    else if (adc_raw <= BUTTON_UP_MAX) state = BUTTON_STATE_SCROLL_UP;
    else if (adc_raw >= BUTTON_RIGHT_MIN && adc_raw <= BUTTON_RIGHT_MAX) state = BUTTON_STATE_RIGHT;
    else if (adc_raw >= BUTTON_LEFT_MIN && adc_raw <= BUTTON_LEFT_MAX) state = BUTTON_STATE_LEFT;
    else if (adc_raw >= BUTTON_DOWN_MIN && adc_raw <= BUTTON_DOWN_MAX) state = BUTTON_STATE_SCROLL_DOWN;
    else state = BUTTON_STATE_IDLE;
    
    return state;
}

static esp_err_t gyro_calibration(void) {
    if (!imu_initialized || imu_sensor == NULL) {
        ESP_LOGE(TAG, "IMU未初始化，无法进行校准");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "开始陀螺仪校准，请保持设备静止 %d 秒...", CALIBRATION_DURATION_MS / 1000);
    
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    int valid_samples = 0;
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) < CALIBRATION_DURATION_MS) {
        float gyro_x, gyro_y, gyro_z;
        if (qmi8658_read_gyro_rads(imu_sensor, &gyro_x, &gyro_y, &gyro_z) == ESP_OK) {
            sum_x += gyro_x;
            sum_y += gyro_y;
            sum_z += gyro_z;
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    if (valid_samples < 10) {
        ESP_LOGE(TAG, "校准失败：有效样本数量不足 (%d)", valid_samples);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    gyro_offset_x = sum_x / valid_samples;
    gyro_offset_y = sum_y / valid_samples;
    gyro_offset_z = sum_z / valid_samples;
    gyro_calibrated = true;
    
    ESP_LOGI(TAG, "陀螺仪校准完成！偏移量: X=%.2f, Y=%.2f, Z=%.2f rad/s", 
             gyro_offset_x, gyro_offset_y, gyro_offset_z);
    
    return ESP_OK;
}

// 鼠标传感器任务
static void mouse_sensor_task(void* arg) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // 开机自动校准陀螺仪
    if (!gyro_calibrated) {
        if (gyro_calibration() != ESP_OK) {
            gyro_offset_x = gyro_offset_y = gyro_offset_z = 0.0f;
            gyro_calibrated = true;
        }
    }
    
    while (1) {
        if (!imu_initialized || imu_sensor == NULL) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // 读取传感器数据
        float gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z;
        if (qmi8658_read_gyro_rads(imu_sensor, &gyro_x, &gyro_y, &gyro_z) != ESP_OK ||
            qmi8658_read_accel_mg(imu_sensor, &acc_x, &acc_y, &acc_z) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // 应用校准偏移量
        if (gyro_calibrated) {
            gyro_x -= gyro_offset_x;
            gyro_y -= gyro_offset_y;
            gyro_z -= gyro_offset_z;
        }
        
        // 计算鼠标移动量
        int8_t mouse_x = (fabs(gyro_x) > g_config.gyro_threshold) ? 
                         (int8_t)(1.2 * gyro_x * g_config.mouse_sensitivity) : 0;
        int8_t mouse_y = (fabs(gyro_z) > g_config.gyro_threshold) ? 
                         (int8_t)(1.2 * gyro_z * g_config.mouse_sensitivity) : 0;
        
        // 处理按键状态
        button_state_t new_button_state = read_button_state();
        uint32_t current_time = esp_log_timestamp();
        
        if (new_button_state != current_button_state && 
            current_time - button_state_start_time > BUTTON_DEBOUNCE_TIME) {
            last_button_state = current_button_state;
            current_button_state = new_button_state;
            button_state_start_time = current_time;
            
            // 输出按键状态变化的调试信息
            const char* button_names[] = {"空闲", "左键", "右键", "上滚轮", "下滚轮"};
            ESP_LOGI(TAG, "按键状态变化: %s -> %s", 
                     button_names[last_button_state], 
                     button_names[current_button_state]);
        }
        
        // 发送HID数据
        if (sec_conn) {
            uint8_t buttons = 0;
            if (current_button_state == BUTTON_STATE_LEFT) buttons = 1;
            else if (current_button_state == BUTTON_STATE_RIGHT) buttons = 2;
            else if (current_button_state == BUTTON_STATE_SCROLL_UP && 
                     current_button_state != last_button_state) {
                ESP_LOGI(TAG, "发送鼠标滚轮上滚命令");
                esp_hidd_send_mouse_value_with_wheel(hid_conn_id, 0, 0, 0, 1);
            } else if (current_button_state == BUTTON_STATE_SCROLL_DOWN && 
                       current_button_state != last_button_state) {
                ESP_LOGI(TAG, "发送鼠标滚轮下滚命令");
                esp_hidd_send_mouse_value_with_wheel(hid_conn_id, 0, 0, 0, -1);
            }
            
            if (mouse_x != 0 || mouse_y != 0 || buttons != 0) {
                // 只在有鼠标移动或按钮事件时输出日志，避免日志过多
                if (buttons != 0) {
                    ESP_LOGI(TAG, "发送鼠标按键事件: buttons=%d", buttons);
                }
                else if (abs(mouse_x) > 5 || abs(mouse_y) > 5) {
                    ESP_LOGI(TAG, "发送鼠标移动事件: x=%d, y=%d", mouse_x, mouse_y);
                }
                esp_hidd_send_mouse_value(hid_conn_id, buttons, mouse_x, mouse_y);
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(g_config.sample_rate_ms));
    }
}

static void hid_demo_task(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    while (1) {
        if (!imu_initialized) {
            imu_initialized = init_imu_sensor();
            if (imu_initialized && mouse_task_handle == NULL) {
                xTaskCreate(&mouse_sensor_task, "mouse_task", 4096, NULL, 6, &mouse_task_handle);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// HIDD事件回调函数
void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param) {
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH:
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                // 使用当前模式的设备名称
                const mode_config_t *mode_cfg = &mode_configs[g_config.mode];
                esp_ble_gap_set_device_name(mode_cfg->device_name);
                esp_ble_gap_config_adv_data(&hidd_adv_data);
                ESP_LOGI(TAG, "HID服务注册完成，设备名: %s", mode_cfg->device_name);
            } else {
                ESP_LOGE(TAG, "HID服务注册失败");
            }
            break;
        case ESP_HIDD_EVENT_BLE_CONNECT:
            hid_conn_id = param->connect.conn_id;
            sec_conn = false; // 重置安全连接状态，等待认证完成
            ESP_LOGI(TAG, "BLE客户端连接，conn_id: %d. 等待主机发起安全请求...", hid_conn_id);
            // 不主动发起安全请求，等待主机发起
            // 主机连接后，如果已有配对，则直接使用；如果没有，则发起配对
            // 如果主机发起安全请求，由gap_event_handler处理
            break;
        case ESP_HIDD_EVENT_BLE_DISCONNECT:
            sec_conn = false;
            hid_conn_id = 0;
                        
            ESP_LOGI(TAG, "BLE客户端断开连接，重新开始广播");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        default:
            break;
    }
}

esp_err_t ble_hid_mouse_init(const ble_hid_mouse_config_t *config) {
    if (g_initialized) return ESP_ERR_INVALID_STATE;
    
    memcpy(&g_config, config, sizeof(ble_hid_mouse_config_t));
    
    // 从NVS读取保存的模式（如果有的话）
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret == ESP_OK) {
        uint8_t saved_mode = 0;
        ret = nvs_get_u8(nvs_handle, NVS_KEY_MODE, &saved_mode);
        if (ret == ESP_OK && saved_mode < BLE_MOUSE_MODE_MAX) {
            g_config.mode = (ble_mouse_mode_t)saved_mode;
            ESP_LOGI(TAG, "从NVS恢复模式: %d", saved_mode);
        }
        nvs_close(nvs_handle);
    }
    
    // 根据模式设置UUID和MAC地址
    const mode_config_t *mode_cfg = &mode_configs[g_config.mode];
    memcpy(hidd_service_uuid128, mode_cfg->service_uuid, 16);
    hidd_adv_data.service_uuid_len = sizeof(hidd_service_uuid128);
    hidd_adv_data.p_service_uuid = hidd_service_uuid128;
    
    // 从系统获取对应类型的MAC地址并设置为蓝牙地址
    uint8_t mac_addr[6];
    ret = esp_read_mac(mac_addr, mode_cfg->mac_type);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "读取MAC地址失败，将使用默认MAC: %s", esp_err_to_name(ret));
    } else {
        // 设置蓝牙设备的MAC地址
        ret = esp_base_mac_addr_set(mac_addr);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "设置MAC地址失败，将使用默认MAC: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "已设置MAC地址 (%s): %02x:%02x:%02x:%02x:%02x:%02x", 
                     mode_cfg->mac_type == ESP_MAC_WIFI_STA ? "WiFi STA" :
                     mode_cfg->mac_type == ESP_MAC_WIFI_SOFTAP ? "WiFi AP" :
                     mode_cfg->mac_type == ESP_MAC_BT ? "Bluetooth" : "Unknown",
                     mac_addr[0], mac_addr[1], mac_addr[2],
                     mac_addr[3], mac_addr[4], mac_addr[5]);
        }
    }
    
    // 初始化NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 初始化蓝牙
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // 加载并管理配对信息
    ble_hid_mouse_load_and_manage_bonds();

    ESP_ERROR_CHECK(esp_hidd_profile_init());
    
    // 注册回调
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_hidd_register_callbacks(hidd_event_callback));
    
    // 设置安全参数 - 恢复为安全连接并绑定
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_BOND; // 请求安全连接和绑定
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;             // 无输入输出能力
    uint8_t key_size = 16;                                // 密钥大小
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    g_initialized = true;
    return ESP_OK;
}

esp_err_t ble_hid_mouse_start(void) {
    if (!g_initialized || g_started) return ESP_ERR_INVALID_STATE;
    
    init_button_adc();
    imu_initialized = init_imu_sensor();
    
    xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 4, NULL);
    if (imu_initialized) {
        xTaskCreate(&mouse_sensor_task, "mouse_task", 4096, NULL, 6, &mouse_task_handle);
    }
    
    g_started = true;
    return ESP_OK;
}

esp_err_t ble_hid_mouse_stop(void) {
    if (!g_started) return ESP_ERR_INVALID_STATE;
    
    // 停止广播
    esp_ble_gap_stop_advertising();
    
    // 删除鼠标任务
    if (mouse_task_handle) {
        vTaskDelete(mouse_task_handle);
        mouse_task_handle = NULL;
    }
    
    // 重置连接状态
    if (sec_conn) {
        ESP_LOGI(TAG, "重置连接状态");
        sec_conn = false;
        hid_conn_id = 0;
    }
    
    g_started = false;
    return ESP_OK;
}

esp_err_t ble_hid_mouse_deinit(void) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    
    if (g_started) ble_hid_mouse_stop();
    
    if (imu_sensor) {
        free(imu_sensor);
        imu_sensor = NULL;
    }
    if (i2c_master_bus) {
        i2c_del_master_bus(i2c_master_bus);
        i2c_master_bus = NULL;
    }
    if (adc1_cali_handle) {
        adc_cali_delete_scheme_curve_fitting(adc1_cali_handle);
        adc1_cali_handle = NULL;
    }
    if (adc1_handle) {
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = NULL;
    }
    adc_initialized = false;
    
    g_initialized = false;
    return ESP_OK;
}

bool ble_hid_mouse_is_connected(void) {
    return sec_conn && imu_initialized;
}

bool ble_hid_mouse_is_initialized(void) {
    return g_initialized;
}

esp_err_t ble_hid_mouse_scroll_up(void) {
    if (!g_initialized || !sec_conn) return ESP_ERR_INVALID_STATE;
    esp_hidd_send_mouse_value_with_wheel(hid_conn_id, 0, 0, 0, 1);
    return ESP_OK;
}

esp_err_t ble_hid_mouse_scroll_down(void) {
    if (!g_initialized || !sec_conn) return ESP_ERR_INVALID_STATE;
    esp_hidd_send_mouse_value_with_wheel(hid_conn_id, 0, 0, 0, -1);
    return ESP_OK;
}

esp_err_t ble_hid_mouse_volume_up(void) {
    if (!g_initialized || !sec_conn) return ESP_ERR_INVALID_STATE;
    esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, true);
    vTaskDelay(pdMS_TO_TICKS(10)); // 短暂延迟
    esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, false);
    return ESP_OK;
}

esp_err_t ble_hid_mouse_volume_down(void) {
    if (!g_initialized || !sec_conn) return ESP_ERR_INVALID_STATE;
    esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, true);
    vTaskDelay(pdMS_TO_TICKS(10)); // 短暂延迟
    esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, false);
    return ESP_OK;
}

esp_err_t ble_hid_mouse_set_sensitivity(float sensitivity) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    if (sensitivity < 10.0f || sensitivity > 100.0f) return ESP_ERR_INVALID_ARG;
    g_config.mouse_sensitivity = sensitivity;
    return ESP_OK;
}

float ble_hid_mouse_get_sensitivity(void) {
    return g_config.mouse_sensitivity;
}

esp_err_t ble_hid_mouse_get_config(ble_hid_mouse_config_t *config) {
    if (!g_initialized || config == NULL) return ESP_ERR_INVALID_ARG;
    memcpy(config, &g_config, sizeof(ble_hid_mouse_config_t));
    return ESP_OK;
}

// 根据当前模式获取NVS中对应的配对信息键
static const char* get_nvs_key_for_current_mode(void) {
    switch (g_config.mode) {
        case BLE_MOUSE_MODE_PC:
            return NVS_KEY_BOND_PC;
        case BLE_MOUSE_MODE_PHONE:
            return NVS_KEY_BOND_PHONE;
        case BLE_MOUSE_MODE_TABLET:
            return NVS_KEY_BOND_TABLET;
        default:
            return NULL;
    }
}

// 保存当前模式的配对地址
static esp_err_t ble_hid_mouse_save_bond(esp_bd_addr_t bond_addr) {
    const char* key = get_nvs_key_for_current_mode();
    if (key == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "打开NVS失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_blob(nvs_handle, key, bond_addr, sizeof(esp_bd_addr_t));
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs_handle);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "已为模式 %d 保存新的配对地址", g_config.mode);
        } else {
            ESP_LOGE(TAG, "NVS提交失败: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "保存配对地址到NVS失败: %s", esp_err_to_name(ret));
    }

    nvs_close(nvs_handle);
    return ret;
}

// 加载当前模式的配对信息，并清理其他模式的配对
static esp_err_t ble_hid_mouse_load_and_manage_bonds(void) {
    esp_bd_addr_t bonded_addr_for_current_mode;
    bool bond_for_current_mode_exists = false;

    // 1. 尝试从NVS加载当前模式的配对地址
    const char* key = get_nvs_key_for_current_mode();
    if (key != NULL) {
        nvs_handle_t nvs_handle;
        esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
        if (ret == ESP_OK) {
            size_t length = sizeof(esp_bd_addr_t);
            ret = nvs_get_blob(nvs_handle, key, bonded_addr_for_current_mode, &length);
            if (ret == ESP_OK && length == sizeof(esp_bd_addr_t)) {
                bond_for_current_mode_exists = true;
                ESP_LOGI(TAG, "为当前模式 %d 加载了已保存的配对地址", g_config.mode);
            }
            nvs_close(nvs_handle);
        }
    }

    // 2. 获取所有已配对的设备列表
    int dev_num = esp_ble_get_bond_device_num();
    if (dev_num <= 0) {
        ESP_LOGI(TAG, "没有已存在的配对信息，无需管理。");
        return ESP_OK;
    }

    esp_ble_bond_dev_t *bond_dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    if (!bond_dev_list) {
        ESP_LOGE(TAG, "无法为配对设备列表分配内存");
        return ESP_ERR_NO_MEM;
    }
    esp_ble_get_bond_device_list(&dev_num, bond_dev_list);

    // 3. 遍历并清理不属于当前模式的配对
    ESP_LOGI(TAG, "开始清理不相关的配对信息...");
    for (int i = 0; i < dev_num; i++) {
        bool should_remove = true;
        if (bond_for_current_mode_exists) {
            if (memcmp(bond_dev_list[i].bd_addr, bonded_addr_for_current_mode, sizeof(esp_bd_addr_t)) == 0) {
                should_remove = false; // 这是当前模式的配对，保留
                ESP_LOGI(TAG, "保留当前模式的配对设备");
            }
        }
        
        if (should_remove) {
            esp_err_t ret = esp_ble_remove_bond_device(bond_dev_list[i].bd_addr);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "已删除不相关的配对设备");
            } else {
                ESP_LOGE(TAG, "删除配对设备失败: %s", esp_err_to_name(ret));
            }
        }
    }

    free(bond_dev_list);
    ESP_LOGI(TAG, "配对信息管理完成。");
    return ESP_OK;
}

// 新增：模式切换功能
esp_err_t ble_hid_mouse_set_mode(ble_mouse_mode_t mode) {
    if (mode >= BLE_MOUSE_MODE_MAX) {
        ESP_LOGE(TAG, "无效的鼠标模式: %d", mode);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "请求切换鼠标模式从 %d 到 %d", g_config.mode, mode);
    
    // 保存新的模式配置到NVS
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret == ESP_OK) {
        nvs_set_u8(nvs_handle, NVS_KEY_MODE, (uint8_t)mode);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "已保存新模式 %d 到NVS，即将重启以应用更改。", mode);
    } else {
        ESP_LOGE(TAG, "无法保存模式到NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 延迟一下让日志输出完成
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // 重启系统来应用模式切换
    // 重启后，ble_hid_mouse_init会运行，并自动加载和管理配对信息
    esp_restart();
    
    // 这里不会执行到
    return ESP_OK;
}

ble_mouse_mode_t ble_hid_mouse_get_mode(void) {
    return g_config.mode;
}
