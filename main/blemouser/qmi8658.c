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
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define TAG "QMI8658"

// I2C配置参数
#define I2C_MASTER_FREQ_HZ      100000   // 100kHz
#define I2C_MASTER_TX_BUF_DISABLE 0     // I2C master不需要buffer
#define I2C_MASTER_RX_BUF_DISABLE 0     // I2C master不需要buffer
#define WRITE_BIT  I2C_MASTER_WRITE     // I2C master写
#define READ_BIT   I2C_MASTER_READ      // I2C master读
#define ACK_CHECK_EN   0x1              // I2C master检查ack
#define ACK_CHECK_DIS  0x0              // I2C master不检查ack
#define ACK_VAL    0x0                  // I2C ack值
#define NACK_VAL   0x1                  // I2C nack值

// 内部函数声明
static esp_err_t qmi8658_write_reg(qmi8658_handle_t *handle, uint8_t reg, uint8_t data);
static esp_err_t qmi8658_read_reg(qmi8658_handle_t *handle, uint8_t reg, uint8_t *data, uint8_t len);
static void qmi8658_update_lsb_div(qmi8658_handle_t *handle);
static int16_t qmi8658_combine_bytes(uint8_t lsb, uint8_t msb);

/**
 * @brief 初始化I2C并配置QMI8658传感器
 */
bool qmi8658_init(qmi8658_handle_t *handle, i2c_port_t i2c_port, 
                  int sda_pin, int scl_pin, uint8_t device_addr) {
    if (handle == NULL) {
        ESP_LOGE(TAG, "句柄为NULL");
        return false;
    }

    // 初始化句柄
    handle->i2c_port = i2c_port;
    handle->device_address = device_addr;
    handle->accel_range = QMI8658_ACCEL_RANGE_2G;
    handle->gyro_range = QMI8658_GYRO_RANGE_512DPS;
    handle->initialized = false;

    // 配置I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(i2c_port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C参数配置失败: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = i2c_driver_install(i2c_port, conf.mode, 
                            I2C_MASTER_RX_BUF_DISABLE, 
                            I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C驱动安装失败: %s", esp_err_to_name(ret));
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
bool qmi8658_read_gyro_dps(qmi8658_handle_t *handle, float *x, float *y, float *z) {
    if (handle == NULL || !handle->initialized) {
        return false;
    }
    
    uint8_t data[6];
    esp_err_t ret = qmi8658_read_reg(handle, QMI8658_GX_L, data, 6);
    if (ret != ESP_OK) {
        return false;
    }
    
    int16_t raw_x = qmi8658_combine_bytes(data[0], data[1]);
    int16_t raw_y = qmi8658_combine_bytes(data[2], data[3]);
    int16_t raw_z = qmi8658_combine_bytes(data[4], data[5]);
    
    float scale_factor = (float)handle->gyro_lsb_div / 32768.0f;
    *x = (float)raw_x * scale_factor;
    *y = (float)raw_y * scale_factor;
    *z = (float)raw_z * scale_factor;
    
    return true;
}

/**
 * @brief 读取加速度计数据 (mg)
 */
bool qmi8658_read_accel_mg(qmi8658_handle_t *handle, float *x, float *y, float *z) {
    if (handle == NULL || !handle->initialized) return false;
    
    uint8_t data[6];
    esp_err_t ret = qmi8658_read_reg(handle, QMI8658_AX_L, data, 6);
    if (ret != ESP_OK) {
        return false;
    }
    
    int16_t raw_x = qmi8658_combine_bytes(data[0], data[1]);
    int16_t raw_y = qmi8658_combine_bytes(data[2], data[3]);
    int16_t raw_z = qmi8658_combine_bytes(data[4], data[5]);
    
    float scale = (float)handle->accel_lsb_div / 32768.0f;
    *x = (float)raw_x / scale;
    *y = (float)raw_y / scale;
    *z = (float)raw_z / scale;
    
    return true;
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
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->device_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(handle->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief 读取寄存器
 */
static esp_err_t qmi8658_read_reg(qmi8658_handle_t *handle, uint8_t reg, uint8_t *data, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->device_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->device_address << 1) | READ_BIT, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(handle->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
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
