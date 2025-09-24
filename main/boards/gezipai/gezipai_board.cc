#include "wifi_board.h"
#include "codecs/es8311_audio_codec.h"
#include "config.h"
#include "mcp_server.h"
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>

#define TAG "gezipai"

class GezipaiBoard : public WifiBoard
{
private:
    i2c_master_bus_handle_t codec_i2c_bus_;

    void InitializeCodecI2c()
    {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));
    }

    void InitializeTools()
    {
        auto &mcp = McpServer::GetInstance();
        mcp.AddTool("self.tv.turn_on", "打开电视", PropertyList(), [](const PropertyList&) -> ReturnValue {
            ESP_LOGI(TAG, "MCP: 打开电视");
            return std::string("电视已打开");
        });
        mcp.AddTool("self.air_conditioner.turn_on", "打开空调", PropertyList(), [](const PropertyList&) -> ReturnValue {
            ESP_LOGI(TAG, "MCP: 打开空调");
            return std::string("空调已打开");
        });
        mcp.AddTool("self.fan.turn_on", "打开风扇", PropertyList(), [](const PropertyList&) -> ReturnValue {
            ESP_LOGI(TAG, "MCP: 打开风扇");
            return std::string("风扇已打开");
        });
        mcp.AddTool("self.light.turn_on", "打开照明灯", PropertyList(), [](const PropertyList&) -> ReturnValue {
            ESP_LOGI(TAG, "MCP: 打开照明灯");
            return std::string("照明灯已打开");
        });
        mcp.AddTool("self.air_conditioner.set_cool_mode", "空调制冷模式并设定温度", PropertyList({
            Property("temperature", kPropertyTypeInteger, 16, 30)
        }), [](const PropertyList& props) -> ReturnValue {
            int temp = props["temperature"].value<int>();
            ESP_LOGI(TAG, "MCP: 空调制冷模式，设定温度%d度", temp);
            return std::string("空调已切换到制冷模式，温度设为" + std::to_string(temp) + "度");
        });
        mcp.AddTool("self.air_conditioner.set_heat_mode", "空调制热模式并设定温度", PropertyList({
            Property("temperature", kPropertyTypeInteger, 16, 30)
        }), [](const PropertyList& props) -> ReturnValue {
            int temp = props["temperature"].value<int>();
            ESP_LOGI(TAG, "MCP: 空调制热模式，设定温度%d度", temp);
            return std::string("空调已切换到制热模式，温度设为" + std::to_string(temp) + "度");
        });
        mcp.AddTool("self.fan.set_speed", "风扇风速调节", PropertyList({
            Property("speed", kPropertyTypeInteger, 1, 5)
        }), [](const PropertyList& props) -> ReturnValue {
            int speed = props["speed"].value<int>();
            ESP_LOGI(TAG, "MCP: 风扇风速设为%d", speed);
            return std::string("风扇风速已设为" + std::to_string(speed));
        });
        mcp.AddTool("self.fan.set_max_speed", "风扇最大风速", PropertyList(), [](const PropertyList&) -> ReturnValue {
            ESP_LOGI(TAG, "MCP: 风扇最大风速");
            return std::string("风扇已切换到最大风速");
        });
        mcp.AddTool("self.fan.set_min_speed", "风扇最小风速", PropertyList(), [](const PropertyList&) -> ReturnValue {
            ESP_LOGI(TAG, "MCP: 风扇最小风速");
            return std::string("风扇已切换到最小风速");
        });
        mcp.AddTool("self.fan.turn_off", "关闭风扇", PropertyList(), [](const PropertyList&) -> ReturnValue {
            ESP_LOGI(TAG, "MCP: 关闭风扇");
            return std::string("风扇已关闭");
        });
        mcp.AddTool("self.tv.turn_off", "关闭电视", PropertyList(), [](const PropertyList&) -> ReturnValue {
            ESP_LOGI(TAG, "MCP: 关闭电视");
            return std::string("电视已关闭");
        });
        mcp.AddTool("self.air_conditioner.turn_off", "关闭空调", PropertyList(), [](const PropertyList&) -> ReturnValue {
            ESP_LOGI(TAG, "MCP: 关闭空调");
            return std::string("空调已关闭");
        });
        mcp.AddTool("self.mouse.switch_mode_pc", "切换鼠标为电脑模式", PropertyList(), [](const PropertyList&) -> ReturnValue {
            ESP_LOGI(TAG, "MCP: 鼠标切换为电脑模式");
            // TODO: 实际切换代码
            return std::string("鼠标已切换为电脑模式");
        });
        mcp.AddTool("self.mouse.switch_mode_phone", "切换鼠标为手机模式", PropertyList(), [](const PropertyList&) -> ReturnValue {
            ESP_LOGI(TAG, "MCP: 鼠标切换为手机模式");
            // TODO: 实际切换代码
            return std::string("鼠标已切换为手机模式");
        });
        mcp.AddTool("self.mouse.switch_mode_tablet", "切换鼠标为平板模式", PropertyList(), [](const PropertyList&) -> ReturnValue {
            ESP_LOGI(TAG, "MCP: 鼠标切换为平板模式");
            // TODO: 实际切换代码
            return std::string("鼠标已切换为平板模式");
        });
    }

public:
    GezipaiBoard()
    {
        ESP_LOGI(TAG, "Initializing Gezipai Board");
        // init gpio led
        gpio_config_t led_config = {
            .pin_bit_mask = 1ULL << BUILTIN_LED_GPIO,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&led_config));

        InitializeCodecI2c();
        InitializeTools();
    }

    virtual AudioCodec *GetAudioCodec() override
    {
        static Es8311AudioCodec audio_codec(codec_i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                                            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
                                            AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR, true);
        return &audio_codec;
    }
};

DECLARE_BOARD(GezipaiBoard);
