/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef __ESP_HIDD_API_H__
#define __ESP_HIDD_API_H__

#include "esp_bt_defs.h"
#include "esp_gatt_defs.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ESP_HIDD_EVENT_REG_FINISH = 0,
    ESP_BAT_EVENT_REG,
    ESP_HIDD_EVENT_DEINIT_FINISH,
    ESP_HIDD_EVENT_BLE_CONNECT,
    ESP_HIDD_EVENT_BLE_DISCONNECT,
    ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT,
    ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT,
} esp_hidd_cb_event_t;

/// HID配置状态
typedef enum {
    ESP_HIDD_STA_CONN_SUCCESS = 0x00,
    ESP_HIDD_STA_CONN_FAIL    = 0x01,
} esp_hidd_sta_conn_state_t;

/// HID初始化状态
typedef enum {
    ESP_HIDD_INIT_OK = 0,
    ESP_HIDD_INIT_FAILED = 1,
} esp_hidd_init_state_t;

/// HID去初始化状态
typedef enum {
    ESP_HIDD_DEINIT_OK = 0,
    ESP_HIDD_DEINIT_FAILED = 0,
} esp_hidd_deinit_state_t;

#define LEFT_CONTROL_KEY_MASK        (1 << 0)
#define LEFT_SHIFT_KEY_MASK          (1 << 1)
#define LEFT_ALT_KEY_MASK            (1 << 2)
#define LEFT_GUI_KEY_MASK            (1 << 3)
#define RIGHT_CONTROL_KEY_MASK       (1 << 4)
#define RIGHT_SHIFT_KEY_MASK         (1 << 5)
#define RIGHT_ALT_KEY_MASK           (1 << 6)
#define RIGHT_GUI_KEY_MASK           (1 << 7)

typedef uint8_t key_mask_t;
/**
 * @brief HIDD回调参数联合体
 */
typedef union {
    /**
	 * @brief ESP_HIDD_EVENT_INIT_FINISH
	 */
    struct hidd_init_finish_evt_param {
        esp_hidd_init_state_t state;				/*!< 初始状态 */
        esp_gatt_if_t gatts_if;
    } init_finish;							      /*!< ESP_HIDD_EVENT_INIT_FINISH的HID回调参数 */

    /**
	 * @brief ESP_HIDD_EVENT_DEINIT_FINISH
	 */
    struct hidd_deinit_finish_evt_param {
        esp_hidd_deinit_state_t state;				/*!< 去初始化状态 */
    } deinit_finish;								/*!< ESP_HIDD_EVENT_DEINIT_FINISH的HID回调参数 */

    /**
     * @brief ESP_HIDD_EVENT_CONNECT
	 */
    struct hidd_connect_evt_param {
        uint16_t conn_id;
        esp_bd_addr_t remote_bda;                   /*!< HID远程蓝牙连接索引 */
    } connect;									    /*!< ESP_HIDD_EVENT_CONNECT的HID回调参数 */

    /**
     * @brief ESP_HIDD_EVENT_DISCONNECT
	 */
    struct hidd_disconnect_evt_param {
        esp_bd_addr_t remote_bda;                   /*!< HID远程蓝牙设备地址 */
    } disconnect;									/*!< ESP_HIDD_EVENT_DISCONNECT的HID回调参数 */

    /**
     * @brief ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT
	 */
    struct hidd_vendor_write_evt_param {
        uint16_t conn_id;                           /*!< HID连接索引 */
        uint16_t report_id;                         /*!< HID报告索引 */
        uint16_t length;                            /*!< 数据长度 */
        uint8_t  *data;                             /*!< 指向数据的指针 */
    } vendor_write;									/*!< ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT的HID回调参数 */

    /**
     * @brief ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT
     */
    struct hidd_led_write_evt_param {
        uint16_t conn_id;
        uint8_t report_id;
        uint8_t length;
        uint8_t *data;
    } led_write;
} esp_hidd_cb_param_t;


/**
 * @brief HID设备事件回调函数类型
 * @param event : 事件类型
 * @param param : 指向回调参数的指针，当前是联合体类型
 */
typedef void (*esp_hidd_event_cb_t) (esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);



/**
 *
 * @brief           此函数用于接收hid设备回调事件
 *
 * @param[in]    callbacks: 回调函数
 *
 * @return         ESP_OK - 成功，其他 - 失败
 *
 */
esp_err_t esp_hidd_register_callbacks(esp_hidd_event_cb_t callbacks);

/**
 *
 * @brief           此函数用于初始化hid设备配置文件
 *
 * @return          ESP_OK - 成功，其他 - 失败
 *
 */
esp_err_t esp_hidd_profile_init(void);

/**
 *
 * @brief           此函数用于去初始化hid设备配置文件
 *
 * @return          ESP_OK - 成功，其他 - 失败
 *
 */
esp_err_t esp_hidd_profile_deinit(void);

/**
 *
 * @brief           获取hidd配置文件版本
 *
 * @return          最高8位是主版本，最低8位是子版本
 *
 */
uint16_t esp_hidd_get_version(void);

void esp_hidd_send_consumer_value(uint16_t conn_id, uint8_t key_cmd, bool key_pressed);

void esp_hidd_send_keyboard_value(uint16_t conn_id, key_mask_t special_key_mask, uint8_t *keyboard_cmd, uint8_t num_key);

void esp_hidd_send_mouse_value(uint16_t conn_id, uint8_t mouse_button, int8_t mickeys_x, int8_t mickeys_y);

void esp_hidd_send_mouse_value_with_wheel(uint16_t conn_id, uint8_t mouse_button, int8_t mickeys_x, int8_t mickeys_y, int8_t wheel);

#ifdef __cplusplus
}
#endif

#endif /* __ESP_HIDD_API_H__ */
