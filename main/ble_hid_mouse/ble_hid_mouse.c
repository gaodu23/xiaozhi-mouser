/*
 * ESP-IDF BLE HID 体感鼠标 - 单一模式精简版
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
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "qmi8658.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/i2c_master.h"

static const char *TAG = "BLE_HID_MOUSE";

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

#define CALIBRATION_DURATION_MS 2000

typedef enum
{
    BUTTON_STATE_IDLE = 0,
    BUTTON_STATE_LEFT,
    BUTTON_STATE_RIGHT,
    BUTTON_STATE_SCROLL_UP,
    BUTTON_STATE_SCROLL_DOWN
} button_state_t;

static ble_hid_mouse_config_t g_config;
static bool g_initialized = false, g_started = false;
static uint16_t hid_conn_id = 0;
static bool sec_conn = false;

static i2c_master_bus_handle_t i2c_master_bus = NULL;
static qmi8658_handle_t *imu_sensor = NULL;
static bool imu_initialized = false;
static TaskHandle_t mouse_task_handle = NULL;
static TaskHandle_t hid_task_handle = NULL;
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool adc_initialized = false;

static bool gyro_calibrated = false;
static float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
static button_state_t current_button_state = BUTTON_STATE_IDLE, last_button_state = BUTTON_STATE_IDLE;
static uint32_t button_state_start_time = 0;

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

static uint8_t hidd_service_uuid128[16] = {0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0xA1};

static esp_ble_adv_data_t hidd_adv_data = {.set_scan_rsp = false, .include_name = true, .include_txpower = true, .min_interval = 0x0006, .max_interval = 0x0010, .appearance = 0x03C2, .manufacturer_len = 0, .p_manufacturer_data = NULL, .service_data_len = 0, .p_service_data = NULL, .service_uuid_len = 16, .p_service_uuid = hidd_service_uuid128, .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)};
static esp_ble_adv_params_t hidd_adv_params = {.adv_int_min = 0x40, .adv_int_max = 0x60, .adv_type = ADV_TYPE_IND, .own_addr_type = BLE_ADDR_TYPE_PUBLIC, .channel_map = ADV_CHNL_ALL, .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY};

static bool init_imu_sensor()
{
    if (imu_sensor == NULL)
    {
        imu_sensor = malloc(sizeof(qmi8658_handle_t));
        if (!imu_sensor)
        {
            ESP_LOGE(TAG, "分配IMU失败");
            return false;
        }
    }
    if (i2c_master_bus == NULL)
    {
        i2c_master_bus_config_t cfg = {.clk_source = I2C_CLK_SRC_DEFAULT, .i2c_port = I2C_NUM_1, .scl_io_num = QMI8658_I2C_SCL_PIN, .sda_io_num = QMI8658_I2C_SDA_PIN, .flags.enable_internal_pullup = true};
        if (i2c_new_master_bus(&cfg, &i2c_master_bus) != ESP_OK)
        {
            ESP_LOGE(TAG, "I2C失败");
            return false;
        }
    }
    if (!qmi8658_init(imu_sensor, i2c_master_bus, QMI8658_ADDRESS_LOW))
    {
        ESP_LOGE(TAG, "QMI8658初始化失败");
        return false;
    }
    qmi8658_set_gyro_range(imu_sensor, QMI8658_GYRO_RANGE_128DPS);
    qmi8658_set_gyro_odr(imu_sensor, QMI8658_GYRO_ODR_125HZ);
    qmi8658_set_accel_range(imu_sensor, QMI8658_ACCEL_RANGE_2G);
    qmi8658_set_accel_odr(imu_sensor, QMI8658_ACCEL_ODR_125HZ);
    qmi8658_enable_sensors(imu_sensor, QMI8658_ENABLE_GYRO | QMI8658_ENABLE_ACCEL);
    vTaskDelay(pdMS_TO_TICKS(100));
    return true;
}

static void init_button_adc()
{
    if (adc_initialized)
    {
        return;
    }
    adc_oneshot_unit_init_cfg_t init_cfg = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));
    adc_oneshot_chan_cfg_t chan_cfg = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BUTTON_ADC_CHANNEL, &chan_cfg));
    adc_cali_curve_fitting_config_t cali_cfg = {.unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc1_cali_handle));
    adc_initialized = true;
}

static button_state_t read_button_state()
{
    int sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++)
    {
        int raw;
        if (adc_oneshot_read(adc1_handle, BUTTON_ADC_CHANNEL, &raw) == ESP_OK)
            sum += raw;
        vTaskDelay(1);
    }
    int adc_raw = sum / ADC_SAMPLES;
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

static esp_err_t gyro_calibration()
{
    if (!imu_initialized || !imu_sensor)
        return ESP_ERR_INVALID_STATE;
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
    if (cnt < 10)
        return ESP_ERR_INVALID_RESPONSE;
    gyro_offset_x = sx / cnt;
    gyro_offset_y = sy / cnt;
    gyro_offset_z = sz / cnt;
    gyro_calibrated = true;
    return ESP_OK;
}

static void mouse_sensor_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    uint32_t last_adc_print_time = 0;
    if (!gyro_calibrated)
    {
        if (gyro_calibration() != ESP_OK)
        {
            gyro_offset_x = gyro_offset_y = gyro_offset_z = 0;
            gyro_calibrated = true;
        }
    }
    while (1)
    {
        if (!imu_initialized || !imu_sensor)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        float gx, gy, gz, ax, ay, az;
        if (qmi8658_read_gyro_rads(imu_sensor, &gx, &gy, &gz) != ESP_OK || qmi8658_read_accel_mg(imu_sensor, &ax, &ay, &az) != ESP_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (gyro_calibrated)
        {
            gx -= gyro_offset_x;
            gy -= gyro_offset_y;
            gz -= gyro_offset_z;
        }
        int8_t mx = (fabs(gx) > g_config.gyro_threshold) ? (int8_t)(1.2 * gx * g_config.mouse_sensitivity) : 0;
        int8_t my = (fabs(gz) > g_config.gyro_threshold) ? (int8_t)(1.2 * gz * g_config.mouse_sensitivity) : 0;
        int adc_raw = 0;
        {
            int sum = 0;
            for (int i = 0; i < ADC_SAMPLES; i++)
            {
                int raw;
                if (adc_oneshot_read(adc1_handle, BUTTON_ADC_CHANNEL, &raw) == ESP_OK)
                    sum += raw;
                vTaskDelay(1);
            }
            adc_raw = sum / ADC_SAMPLES;
        }
        // 每秒打印一次ADC值
        uint32_t now = esp_log_timestamp();
        if (now - last_adc_print_time > 1000)
        {
            ESP_LOGI(TAG, "ADC原始值: %d", adc_raw);
            last_adc_print_time = now;
        }
        button_state_t new_state = read_button_state();
        uint32_t now_time = esp_log_timestamp();
        if (new_state != current_button_state && now_time - button_state_start_time > BUTTON_DEBOUNCE_TIME)
        {
            last_button_state = current_button_state;
            current_button_state = new_state;
            button_state_start_time = now_time;
        }
        if (sec_conn)
        {
            uint8_t buttons = 0;
            if (current_button_state == BUTTON_STATE_LEFT)
                buttons = 1;
            else if (current_button_state == BUTTON_STATE_RIGHT)
                buttons = 2;
            
            // 滚轮事件处理
            if (current_button_state == BUTTON_STATE_SCROLL_UP)
            {
                // 持续滚动或首次按下时滚动
                if (current_button_state == last_button_state || current_button_state != last_button_state)
                {
                    esp_hidd_send_mouse_value_with_wheel(hid_conn_id, 0, 0, 0, 1);
                }
            }
            else if (current_button_state == BUTTON_STATE_SCROLL_DOWN)
            {
                // 持续滚动或首次按下时滚动
                if (current_button_state == last_button_state || current_button_state != last_button_state)
                {
                    esp_hidd_send_mouse_value_with_wheel(hid_conn_id, 0, 0, 0, -1);
                }
            }
            
            // 鼠标移动和按键事件始终发送，无论是否按下按键
            esp_hidd_send_mouse_value(hid_conn_id, buttons, mx, my);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(g_config.sample_rate_ms));
    }
}

static void hid_demo_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    while (1)
    {
        if (!imu_initialized)
        {
            imu_initialized = init_imu_sensor();
            if (imu_initialized && mouse_task_handle == NULL)
            {
                xTaskCreate(mouse_sensor_task, "mouse_task", 4096, NULL, 6, &mouse_task_handle);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "广播启动失败 %d", param->adv_start_cmpl.status);
        }
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, 0);
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (param->ble_security.auth_cmpl.success)
        {
            sec_conn = true;
        }
        else
        {
            sec_conn = false;
            esp_ble_gap_disconnect(param->ble_security.auth_cmpl.bd_addr);
        }
        break;
    default:
        break;
    }
}

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch (event)
    {
    case ESP_HIDD_EVENT_REG_FINISH:
        if (param->init_finish.state == ESP_HIDD_INIT_OK)
        {
            esp_ble_gap_set_device_name(g_config.device_name);
            esp_ble_gap_config_adv_data(&hidd_adv_data);
        }
        break;
    case ESP_HIDD_EVENT_BLE_CONNECT:
        hid_conn_id = param->connect.conn_id;
        sec_conn = false;
        break;
    case ESP_HIDD_EVENT_BLE_DISCONNECT:
        sec_conn = false;
        hid_conn_id = 0;
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    default:
        break;
    }
}

esp_err_t ble_hid_mouse_init(const ble_hid_mouse_config_t *config)
{
    if (g_initialized)
        return ESP_ERR_INVALID_STATE;
    memcpy(&g_config, config, sizeof(g_config));
    if (strlen(g_config.device_name) == 0)
        strncpy(g_config.device_name, "XiaoZhi-Mouse", sizeof(g_config.device_name) - 1);
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_hidd_profile_init());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_hidd_register_callbacks(hidd_event_callback));
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
    g_initialized = true;
    return ESP_OK;
}

esp_err_t ble_hid_mouse_start(void)
{
    if (!g_initialized || g_started)
        return ESP_ERR_INVALID_STATE;
    init_button_adc();
    imu_initialized = init_imu_sensor();
    xTaskCreate(hid_demo_task, "hid_task", 2048, NULL, 4, &hid_task_handle);
    if (imu_initialized)
    {
        xTaskCreate(mouse_sensor_task, "mouse_task", 4096, NULL, 6, &mouse_task_handle);
    }
    g_started = true;
    return ESP_OK;
}

esp_err_t ble_hid_mouse_stop(void)
{
    if (!g_started)
        return ESP_ERR_INVALID_STATE;
    esp_ble_gap_stop_advertising();
    if (mouse_task_handle)
    {
        vTaskDelete(mouse_task_handle);
        mouse_task_handle = NULL;
    }
    if (hid_task_handle)
    {
        vTaskDelete(hid_task_handle);
        hid_task_handle = NULL;
    }
    sec_conn = false;
    hid_conn_id = 0;
    g_started = false;
    return ESP_OK;
}

esp_err_t ble_hid_mouse_deinit(void)
{
    if (!g_initialized)
        return ESP_ERR_INVALID_STATE;
    if (g_started)
        ble_hid_mouse_stop();
    if (imu_sensor)
    {
        free(imu_sensor);
        imu_sensor = NULL;
    }
    if (i2c_master_bus)
    {
        i2c_del_master_bus(i2c_master_bus);
        i2c_master_bus = NULL;
    }
    if (adc1_cali_handle)
    {
        adc_cali_delete_scheme_curve_fitting(adc1_cali_handle);
        adc1_cali_handle = NULL;
    }
    if (adc1_handle)
    {
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = NULL;
    }
    adc_initialized = false;
    g_initialized = false;
    return ESP_OK;
}

bool ble_hid_mouse_is_connected(void) { return sec_conn && imu_initialized; }
bool ble_hid_mouse_is_initialized(void) { return g_initialized; }

esp_err_t ble_hid_mouse_scroll_up(void)
{
    if (!g_initialized || !sec_conn)
        return ESP_ERR_INVALID_STATE;
    esp_hidd_send_mouse_value_with_wheel(hid_conn_id, 0, 0, 0, 1);
    return ESP_OK;
}
esp_err_t ble_hid_mouse_scroll_down(void)
{
    if (!g_initialized || !sec_conn)
        return ESP_ERR_INVALID_STATE;
    esp_hidd_send_mouse_value_with_wheel(hid_conn_id, 0, 0, 0, -1);
    return ESP_OK;
}

esp_err_t ble_hid_mouse_volume_up(void)
{
    if (!g_initialized || !sec_conn)
        return ESP_ERR_INVALID_STATE;
    esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, true);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, false);
    return ESP_OK;
}
esp_err_t ble_hid_mouse_volume_down(void)
{
    if (!g_initialized || !sec_conn)
        return ESP_ERR_INVALID_STATE;
    esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, true);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, false);
    return ESP_OK;
}

esp_err_t ble_hid_mouse_set_sensitivity(float sensitivity)
{
    if (!g_initialized)
        return ESP_ERR_INVALID_STATE;
    if (sensitivity < 10.0f || sensitivity > 100.0f)
        return ESP_ERR_INVALID_ARG;
    g_config.mouse_sensitivity = sensitivity;
    return ESP_OK;
}
float ble_hid_mouse_get_sensitivity(void) { return g_config.mouse_sensitivity; }

esp_err_t ble_hid_mouse_get_config(ble_hid_mouse_config_t *config)
{
    if (!g_initialized || !config)
        return ESP_ERR_INVALID_ARG;
    memcpy(config, &g_config, sizeof(g_config));
    return ESP_OK;
}
