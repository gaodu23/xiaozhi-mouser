/*
 * QMI8658 6-DOF IMU 传感器库 - ESP-IDF C语言实现
 * 适用于ESP32平台的C语言驱动实现
 * 
 * 功能实现:
 * - I2C通信
 * - 传感器配置和读取
 * - 数据转换和校准
 * 
 * 版本: 1.0
 * 日期: 2024
 */

#include "qmi8658.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define TAG "QMI8658"

// I2C配置参数
#define I2C_MASTER_TIMEOUT_MS    1000    // I2C超时时间

// 内部函数声明
static esp_err_t qmi8658_write_reg(qmi8658_handle_t *handle, uint8_t reg, uint8_t data);
static esp_err_t qmi8658_read_reg(qmi8658_handle_t *handle, uint8_t reg, uint8_t *data, uint8_t len);
static void qmi8658_update_lsb_div(qmi8658_handle_t *handle);
static int16_t qmi8658_combine_bytes(uint8_t lsb, uint8_t msb);

/**
 * @brief 初始化I2C并配置QMI8658传感器
 */
bool qmi8658_init(qmi8658_handle_t *handle, i2c_master_bus_handle_t i2c_master_bus, 
                  uint8_t device_addr) {
    if (handle == NULL) {
        ESP_LOGE(TAG, "句柄为NULL");
        return false;
    }

    // 初始化句柄
    handle->device_address = device_addr;
    handle->accel_range = QMI8658_ACCEL_RANGE_2G;
    handle->gyro_range = QMI8658_GYRO_RANGE_512DPS;
    handle->initialized = false;
    
    // 配置I2C设备
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_addr,
        .scl_speed_hz = 100000,  // 100kHz
    };
    
    // 创建I2C设备
    esp_err_t ret = i2c_master_bus_add_device(i2c_master_bus, &dev_cfg, &handle->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "创建I2C设备失败: %s", esp_err_to_name(ret));
        return false;
    }

    // 等待传感器启动
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // 检查WHO AM I
    uint8_t who_am_i = qmi8658_get_who_am_i(handle);
    if (who_am_i != 0x05) {
        ESP_LOGE(TAG, "WHO AM I校验失败: 0x%02X (期望: 0x05)", who_am_i);
        return false;
    }

    // 软复位
    if (!qmi8658_reset(handle)) {
        ESP_LOGE(TAG, "传感器复位失败");
        return false;
    }

    // 等待复位完成
    vTaskDelay(100 / portTICK_PERIOD_MS);

    qmi8658_write_reg(handle, QMI8658_CTRL1, 0x80);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    qmi8658_write_reg(handle, QMI8658_CTRL1, 0x40);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    
    qmi8658_set_accel_range(handle, QMI8658_ACCEL_RANGE_2G);
    qmi8658_set_gyro_range(handle, QMI8658_GYRO_RANGE_512DPS);
    
    qmi8658_enable_sensors(handle, QMI8658_ENABLE_GYRO | QMI8658_ENABLE_ACCEL);

    qmi8658_update_lsb_div(handle);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    handle->initialized = true;
    return true;
}

/**
 * @brief 设置加速度计量程
 */
bool qmi8658_set_accel_range(qmi8658_handle_t *handle, uint8_t range) {
    if (handle == NULL) return false;
    
    uint8_t ctrl2_val = (QMI8658_ACCEL_ODR_125HZ << 4) | range;
    esp_err_t ret = qmi8658_write_reg(handle, QMI8658_CTRL2, ctrl2_val);
    if (ret == ESP_OK) {
        handle->accel_range = range;
        qmi8658_update_lsb_div(handle);
        return true;
    }
    ESP_LOGE(TAG, "设置加速度计量程失败");
    return false;
}

/**
 * @brief 设置陀螺仪量程
 */
bool qmi8658_set_gyro_range(qmi8658_handle_t *handle, uint8_t range) {
    if (handle == NULL) return false;
    
    uint8_t ctrl3_val = (QMI8658_GYRO_ODR_125HZ << 4) | range;
    esp_err_t ret = qmi8658_write_reg(handle, QMI8658_CTRL3, ctrl3_val);
    if (ret == ESP_OK) {
        handle->gyro_range = range;
        qmi8658_update_lsb_div(handle);
        return true;
    }
    ESP_LOGE(TAG, "设置陀螺仪量程失败");
    return false;
}

/**
 * @brief 设置加速度计采样率
 */
bool qmi8658_set_accel_odr(qmi8658_handle_t *handle, uint8_t odr) {
    if (handle == NULL) return false;
    
    uint8_t ctrl2_val = (handle->accel_range << 4) | odr;
    esp_err_t ret = qmi8658_write_reg(handle, QMI8658_CTRL2, ctrl2_val);
    return (ret == ESP_OK);
}

/**
 * @brief 设置陀螺仪采样率
 */
bool qmi8658_set_gyro_odr(qmi8658_handle_t *handle, uint8_t odr) {
    if (handle == NULL) return false;
    
    uint8_t ctrl3_val = (handle->gyro_range << 4) | odr;
    esp_err_t ret = qmi8658_write_reg(handle, QMI8658_CTRL3, ctrl3_val);
    return (ret == ESP_OK);
}

/**
 * @brief 启用传感器
 */
bool qmi8658_enable_sensors(qmi8658_handle_t *handle, uint8_t enable_flags) {
    if (handle == NULL) return false;
    
    esp_err_t ret = qmi8658_write_reg(handle, QMI8658_CTRL7, enable_flags);
    return (ret == ESP_OK);
}

/**
 * @brief 读取陀螺仪数据 (dps)
 */
esp_err_t qmi8658_read_gyro_dps(qmi8658_handle_t *handle, float *x, float *y, float *z) {
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t data[6];
    esp_err_t ret = qmi8658_read_reg(handle, QMI8658_GX_L, data, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    int16_t raw_x = qmi8658_combine_bytes(data[0], data[1]);
    int16_t raw_y = qmi8658_combine_bytes(data[2], data[3]);
    int16_t raw_z = qmi8658_combine_bytes(data[4], data[5]);
    
    float scale_factor = (float)handle->gyro_lsb_div / 32768.0f;
    *x = (float)raw_x * scale_factor;
    *y = (float)raw_y * scale_factor;
    *z = (float)raw_z * scale_factor;
    
    return ESP_OK;
}

/**
 * @brief 读取陀螺仪数据 (rad/s)
 */
esp_err_t qmi8658_read_gyro_rads(qmi8658_handle_t *handle, float *x, float *y, float *z) {
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t data[6];
    esp_err_t ret = qmi8658_read_reg(handle, QMI8658_GX_L, data, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    int16_t raw_x = qmi8658_combine_bytes(data[0], data[1]);
    int16_t raw_y = qmi8658_combine_bytes(data[2], data[3]);
    int16_t raw_z = qmi8658_combine_bytes(data[4], data[5]);
    
    float scale_factor = (float)handle->gyro_lsb_div / 32768.0f;
    // 转换为弧度每秒：度 * π / 180
    float dps_to_rads = M_PI / 180.0f;
    *x = (float)raw_x * scale_factor * dps_to_rads;
    *y = (float)raw_y * scale_factor * dps_to_rads;
    *z = (float)raw_z * scale_factor * dps_to_rads;
    
    return ESP_OK;
}

/**
 * @brief 读取加速度计数据 (mg)
 */
esp_err_t qmi8658_read_accel_mg(qmi8658_handle_t *handle, float *x, float *y, float *z) {
    if (handle == NULL || !handle->initialized) return ESP_ERR_INVALID_STATE;
    
    uint8_t data[6];
    esp_err_t ret = qmi8658_read_reg(handle, QMI8658_AX_L, data, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    int16_t raw_x = qmi8658_combine_bytes(data[0], data[1]);
    int16_t raw_y = qmi8658_combine_bytes(data[2], data[3]);
    int16_t raw_z = qmi8658_combine_bytes(data[4], data[5]);
    
    float scale = (float)handle->accel_lsb_div / 32768.0f;
    *x = (float)raw_x / scale;
    *y = (float)raw_y / scale;
    *z = (float)raw_z / scale;
    
    return ESP_OK;
}

/**
 * @brief 读取温度数据
 */
bool qmi8658_read_temperature(qmi8658_handle_t *handle, float *temp) {
    if (handle == NULL || !handle->initialized) return false;
    
    uint8_t data[2];
    esp_err_t ret = qmi8658_read_reg(handle, QMI8658_TEMP_L, data, 2);
    if (ret != ESP_OK) {
        return false;
    }
    
    int16_t raw_temp = qmi8658_combine_bytes(data[0], data[1]);
    *temp = (float)raw_temp / 256.0f;
    
    return true;
}

/**
 * @brief 获取WHO AM I寄存器值
 */
uint8_t qmi8658_get_who_am_i(qmi8658_handle_t *handle) {
    if (handle == NULL) return 0;
    
    uint8_t who_am_i = 0;
    qmi8658_read_reg(handle, QMI8658_WHO_AM_I, &who_am_i, 1);
    return who_am_i;
}

/**
 * @brief 软复位传感器
 */
bool qmi8658_reset(qmi8658_handle_t *handle) {
    if (handle == NULL) return false;
    
    esp_err_t ret = qmi8658_write_reg(handle, QMI8658_CTRL1, 0x85);
    return (ret == ESP_OK);
}

/**
 * @brief 检查数据是否准备就绪
 */
bool qmi8658_is_data_ready(qmi8658_handle_t *handle) {
    if (handle == NULL) return false;
    
    uint8_t status;
    esp_err_t ret = qmi8658_read_reg(handle, QMI8658_STATUS1, &status, 1);
    if (ret != ESP_OK) return false;
    
    return (status & 0x03) == 0x03;
}

// ==================== 内部函数实现 ====================

/**
 * @brief 写入寄存器
 */
static esp_err_t qmi8658_write_reg(qmi8658_handle_t *handle, uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_transmit(handle->i2c_dev, write_buf, 2, I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief 读取寄存器
 */
static esp_err_t qmi8658_read_reg(qmi8658_handle_t *handle, uint8_t reg, uint8_t *data, uint8_t len) {
    // 先写入要读取的寄存器地址
    esp_err_t ret = i2c_master_transmit(handle->i2c_dev, &reg, 1, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 然后读取数据
    return i2c_master_receive(handle->i2c_dev, data, len, I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief 更新LSB除数（根据量程计算）
 */
static void qmi8658_update_lsb_div(qmi8658_handle_t *handle) {
    switch (handle->accel_range) {
        case QMI8658_ACCEL_RANGE_2G:  handle->accel_lsb_div = 2000; break;
        case QMI8658_ACCEL_RANGE_4G:  handle->accel_lsb_div = 4000; break;
        case QMI8658_ACCEL_RANGE_8G:  handle->accel_lsb_div = 8000; break;
        case QMI8658_ACCEL_RANGE_16G: handle->accel_lsb_div = 16000; break;
        default: handle->accel_lsb_div = 2000; break;
    }
    
    switch (handle->gyro_range) {
        case QMI8658_GYRO_RANGE_16DPS:   handle->gyro_lsb_div = 16; break;
        case QMI8658_GYRO_RANGE_32DPS:   handle->gyro_lsb_div = 32; break;
        case QMI8658_GYRO_RANGE_64DPS:   handle->gyro_lsb_div = 64; break;
        case QMI8658_GYRO_RANGE_128DPS:  handle->gyro_lsb_div = 128; break;
        case QMI8658_GYRO_RANGE_256DPS:  handle->gyro_lsb_div = 256; break;
        case QMI8658_GYRO_RANGE_512DPS:  handle->gyro_lsb_div = 512; break;
        case QMI8658_GYRO_RANGE_1024DPS: handle->gyro_lsb_div = 1024; break;
        case QMI8658_GYRO_RANGE_2048DPS: handle->gyro_lsb_div = 2048; break;
        default: handle->gyro_lsb_div = 512; break;
    }
}

/**
 * @brief 合并高低字节
 */
static int16_t qmi8658_combine_bytes(uint8_t lsb, uint8_t msb) {
    return (int16_t)((msb << 8) | lsb);
}
