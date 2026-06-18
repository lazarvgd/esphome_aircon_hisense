#pragma once

#define DEBUG_LOGGING 0

#include "esphome.h"
#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/gpio.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"

#include <deque>
#include <string>
#include <queue>
#include <cstring>
#include <vector>

#include "messages.h"
#include "device_status.h"

namespace esphome
{
    namespace airconintl
    {

        using climate::ClimateCall;
        using climate::ClimateFanMode;
        using climate::ClimateMode;
        using climate::ClimatePreset;
        using climate::ClimateSwingMode;
        using climate::ClimateTraits;
        using esphome::PollingComponent;
        using sensor::Sensor;
        using uart::UARTDevice;

        class AirconClimate : public esphome::PollingComponent, public climate::Climate, public uart::UARTDevice
        {
        public:
            AirconClimate() {
                mode = climate::CLIMATE_MODE_OFF;
                fan_mode = climate::CLIMATE_FAN_AUTO;
                swing_mode = climate::CLIMATE_SWING_OFF;
                preset = climate::CLIMATE_PRESET_NONE;
                current_temperature = 20.0f;
                target_temperature = 22.0f;
            }

            void dump_config() override {};
            void set_temperature_unit(const std::string &unit) {
                if (this->temperature_unit != unit) {
                    this->temperature_unit = unit;
                    if (unit == "F") {
                        std::vector<uint8_t> msg(temp_to_F, temp_to_F + sizeof(temp_to_F));
                        send_message("Switch to Fahrenheit", msg);
                    } else if (unit == "C") {
                        std::vector<uint8_t> msg(temp_to_C, temp_to_C + sizeof(temp_to_C));
                        send_message("Switch to Celsius", msg);
                    }
                }
            }
            void set_compressor_frequency_sensor(Sensor *sensor) { this->compressor_frequency = sensor; }
            void set_compressor_frequency_setting_sensor(Sensor *sensor) { this->compressor_frequency_setting = sensor; }
            void set_compressor_frequency_send_sensor(Sensor *sensor) { this->compressor_frequency_send = sensor; }
            void set_outdoor_temperature_sensor(Sensor *sensor) { this->outdoor_temperature = sensor; }
            void set_outdoor_condenser_temperature_sensor(Sensor *sensor) { this->outdoor_condenser_temperature = sensor; }
            void set_compressor_exhaust_temperature_sensor(Sensor *sensor) { this->compressor_exhaust_temperature = sensor; }
            void set_target_exhaust_temperature_sensor(Sensor *sensor) { this->target_exhaust_temperature = sensor; }
            void set_power_sensor(Sensor *sensor) { this->power_sensor = sensor; }
            void set_indoor_pipe_temperature_sensor(Sensor *sensor) { this->indoor_pipe_temperature = sensor; }
            void set_indoor_humidity_setting_sensor(Sensor *sensor) { this->indoor_humidity_setting = sensor; }
            void set_indoor_humidity_status_sensor(Sensor *sensor) { this->indoor_humidity_status = sensor; }
            void set_re_pin(GPIOPin *pin) { this->re_pin = pin; }
            void set_de_pin(GPIOPin *pin) { this->de_pin = pin; }

            void setup() override
            {
                while (available()) { read(); }
                if (re_pin != nullptr) {
                    re_pin->setup();
                    re_pin->digital_write(true);
                }
                if (de_pin != nullptr) {
                    de_pin->setup();
                    de_pin->digital_write(false);
                }
                request_update();
            }

            void loop() override
            {
                volatile int msg_size = 0;
                while (available())
                {
                    last_read_time = millis();
                    msg_size = get_response(read(), uart_buf);
                    if (msg_size > 0)
                    {
                        ESP_LOGD("aircon_climate", "compf: %d compf_set: %d compf_snd: %d",
                            ((Device_Status *)uart_buf)->compressor_frequency,
                            ((Device_Status *)uart_buf)->compressor_frequency_setting,
                            ((Device_Status *)uart_buf)->compressor_frequency_send);
                        ESP_LOGD("aircon_climate", "out_temp: %d out_cond_temp: %d comp_exh_temp: %d comp_exh_temp_tgt: %d",
                            ((Device_Status *)uart_buf)->outdoor_temperature,
                            ((Device_Status *)uart_buf)->outdoor_condenser_temperature,
                            ((Device_Status *)uart_buf)->compressor_exhaust_temperature,
                            ((Device_Status *)uart_buf)->target_exhaust_temperature);
                        ESP_LOGD("aircon_climate", "indoor_pipe_temp %d",
                            ((Device_Status *)uart_buf)->indoor_pipe_temperature);
                        ESP_LOGD("aircon_climate", "indor_humid_set: %d indoor_humid: %d",
                            ((Device_Status *)uart_buf)->indoor_humidity_setting,
                            ((Device_Status *)uart_buf)->indoor_humidity_status);
                        ESP_LOGD("aircon_climate", "wind_status: %d direction_status: %d run_status: %d mode_status: %d",
                            ((Device_Status *)uart_buf)->wind_status,
                            ((Device_Status *)uart_buf)->direction_status,
                            ((Device_Status *)uart_buf)->run_status,
                            ((Device_Status *)uart_buf)->mode_status);
                        ESP_LOGD("aircon_climate", "indoor_temp_set: %d indoor_temp_stat: %d",
                            ((Device_Status *)uart_buf)->indoor_temperature_setting,
                            ((Device_Status *)uart_buf)->indoor_temperature_status);
                        ESP_LOGD("aircon_climate", "left_right: %d up_down: %d",
                            ((Device_Status *)uart_buf)->left_right,
                            ((Device_Status *)uart_buf)->up_down);

                        float tgt_temp = (float)((Device_Status *)uart_buf)->indoor_temperature_setting;
                        float curr_temp = (float)((Device_Status *)uart_buf)->indoor_temperature_status;

                        if (tgt_temp > 7 && tgt_temp < 33)
                            target_temperature = tgt_temp;
                        if (curr_temp > 1 && curr_temp < 49)
                            current_temperature = curr_temp;

                        bool comp_running = ((Device_Status *)uart_buf)->compressor_frequency > 0;

                        if (((Device_Status *)uart_buf)->left_right && ((Device_Status *)uart_buf)->up_down)
                            swing_mode = climate::CLIMATE_SWING_BOTH;
                        else if (((Device_Status *)uart_buf)->left_right)
                            swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
                        else if (((Device_Status *)uart_buf)->up_down)
                            swing_mode = climate::CLIMATE_SWING_VERTICAL;
                        else
                            swing_mode = climate::CLIMATE_SWING_OFF;

                        if (((Device_Status *)uart_buf)->run_status == 0) {
                            mode = climate::CLIMATE_MODE_OFF;
                            action = climate::CLIMATE_ACTION_OFF;
                        } else if (((Device_Status *)uart_buf)->mode_status == 0) {
                            mode = climate::CLIMATE_MODE_FAN_ONLY;
                            action = climate::CLIMATE_ACTION_FAN;
                        } else if (((Device_Status *)uart_buf)->mode_status == 1) {
                            mode = climate::CLIMATE_MODE_HEAT;
                            action = comp_running ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
                        } else if (((Device_Status *)uart_buf)->mode_status == 2) {
                            mode = climate::CLIMATE_MODE_COOL;
                            action = comp_running ? climate::CLIMATE_ACTION_COOLING : climate::CLIMATE_ACTION_IDLE;
                        } else if (((Device_Status *)uart_buf)->mode_status == 3) {
                            mode = climate::CLIMATE_MODE_DRY;
                            action = comp_running ? climate::CLIMATE_ACTION_DRYING : climate::CLIMATE_ACTION_IDLE;
                        }

                        if (((Device_Status *)uart_buf)->wind_status == 18)
                            fan_mode = climate::CLIMATE_FAN_HIGH;
                        else if (((Device_Status *)uart_buf)->wind_status == 14)
                            fan_mode = climate::CLIMATE_FAN_MEDIUM;
                        else if (((Device_Status *)uart_buf)->wind_status == 10)
                            fan_mode = climate::CLIMATE_FAN_LOW;
                        else if (((Device_Status *)uart_buf)->wind_status == 2)
                            fan_mode = climate::CLIMATE_FAN_QUIET;
                        else if (((Device_Status *)uart_buf)->wind_status == 0)
                            fan_mode = climate::CLIMATE_FAN_AUTO;

                        if (this->mode == climate::CLIMATE_MODE_COOL && target_temperature > 0)
                            cool_tgt_temp = target_temperature;
                        else if (this->mode == climate::CLIMATE_MODE_HEAT && target_temperature > 0)
                            heat_tgt_temp = target_temperature;

                        send_state = IDLE;
                        this->publish_state();
                        while (available()) { read(); }
                    }
                }

                if (send_state == WAITING_ACK && (millis() - send_timestamp > 3000)) {
                    ESP_LOGE("aircon_climate", "UART ACK timeout for: %s", current_desc.empty() ? "unknown" : current_desc.c_str());
                    if (de_pin != nullptr) de_pin->digital_write(false);
                    if (re_pin != nullptr) re_pin->digital_write(true);
                    while (available()) { read(); }
                    send_state = IDLE;
                    if (!message_queue.empty()) message_queue.pop();
                }
                if (send_state == IDLE && !message_queue.empty() && (millis() - last_send_time >= 100) && (millis() - last_read_time >= 10) && (millis() >= idle_until)) {
                    auto item = message_queue.front();
                    ESP_LOGD("aircon_climate", "Sending: %s", item.description.c_str());
                    if (de_pin != nullptr) de_pin->digital_write(true);
                    if (re_pin != nullptr) re_pin->digital_write(true);
                    write_array(item.payload.data(), item.payload.size());
                    flush();
                    if (de_pin != nullptr) de_pin->digital_write(false);
                    if (re_pin != nullptr) re_pin->digital_write(false);
                    while (available()) { read(); }
                    current_desc = item.description;
                    send_state = WAITING_ACK;
                    send_timestamp = millis();
                    last_send_time = millis();
                    idle_until = millis() + 1500;
                    message_queue.pop();
                }
            }

            void update() override
            {
                request_update();
                this->publish_state();
                set_sensor(compressor_frequency, ((Device_Status *)uart_buf)->compressor_frequency);
                set_sensor(compressor_frequency_setting, ((Device_Status *)uart_buf)->compressor_frequency_setting);
                set_sensor(compressor_frequency_send, ((Device_Status *)uart_buf)->compressor_frequency_send);
                set_sensor(outdoor_temperature, ((Device_Status *)uart_buf)->outdoor_temperature);
                set_sensor(outdoor_condenser_temperature, ((Device_Status *)uart_buf)->outdoor_condenser_temperature);
                set_sensor(compressor_exhaust_temperature, ((Device_Status *)uart_buf)->compressor_exhaust_temperature);
                set_sensor(target_exhaust_temperature, ((Device_Status *)uart_buf)->target_exhaust_temperature);
                set_sensor(indoor_pipe_temperature, ((Device_Status *)uart_buf)->indoor_pipe_temperature);
                set_sensor(indoor_humidity_setting, ((Device_Status *)uart_buf)->indoor_humidity_setting);
                set_sensor(indoor_humidity_status, ((Device_Status *)uart_buf)->indoor_humidity_status);
                uint16_t uab = ((uint16_t)((Device_Status *)uart_buf)->UAB_HIGH << 8) | ((Device_Status *)uart_buf)->UAB_LOW;
                uint8_t ibc = ((Device_Status *)uart_buf)->IBC;
                ESP_LOGD("aircon_climate", "UAB: %d IAB: %d IBC: %d ICA: %d IUV: %d v_dc: %d",
                    uab,
                    ((Device_Status *)uart_buf)->IAB,
                    ibc,
                    ((Device_Status *)uart_buf)->ICA,
                    ((Device_Status *)uart_buf)->IUV,
                    (uint16_t)(((Device_Status *)uart_buf)->generatrix_voltage_high << 8) | ((Device_Status *)uart_buf)->generatrix_voltage_low);
                set_sensor(power_sensor, uab * ibc * 0.01f);
            }

            void control(const ClimateCall &call) override
            {
                ESP_LOGD("aircon_climate", "Control called");
                if (call.get_mode().has_value())
                {
                    if (this->mode == climate::CLIMATE_MODE_COOL && target_temperature > 0)
                        cool_tgt_temp = target_temperature;
                    else if (this->mode == climate::CLIMATE_MODE_HEAT && target_temperature > 0)
                        heat_tgt_temp = target_temperature;

                    ClimateMode md = *call.get_mode();

                    if (md != climate::CLIMATE_MODE_OFF && this->mode == climate::CLIMATE_MODE_OFF) {
                        std::vector<uint8_t> msg(on, on + sizeof(on));
                        send_message("Power On", msg);
                    }

                    switch (md)
                    {
                    case climate::CLIMATE_MODE_OFF: {
                        std::vector<uint8_t> msg(off, off + sizeof(off));
                        send_message("Power Off", msg);
                        break;
                    }
                    case climate::CLIMATE_MODE_COOL: {
                        std::vector<uint8_t> msg(mode_cool, mode_cool + sizeof(mode_cool));
                        send_message("Set Mode to Cool", msg);
                        set_temp(cool_tgt_temp);
                        break;
                    }
                    case climate::CLIMATE_MODE_HEAT: {
                        std::vector<uint8_t> msg(mode_heat, mode_heat + sizeof(mode_heat));
                        send_message("Set Mode to Heat", msg);
                        set_temp(heat_tgt_temp);
                        break;
                    }
                    case climate::CLIMATE_MODE_FAN_ONLY: {
                        std::vector<uint8_t> msg(mode_fan, mode_fan + sizeof(mode_fan));
                        send_message("Set Mode to Fan", msg);
                        break;
                    }
                    case climate::CLIMATE_MODE_DRY: {
                        std::vector<uint8_t> msg(mode_dry, mode_dry + sizeof(mode_dry));
                        send_message("Set Mode to Dry", msg);
                        break;
                    }
                    default: break;
                    }

                    this->mode = md;
                    this->publish_state();
                }

                if (call.get_target_temperature().has_value())
                {
                    float temp = *call.get_target_temperature();
                    set_temp(temp);
                    target_temperature = temp;
                    this->publish_state();
                }

                if (call.get_fan_mode().has_value())
                {
                    ClimateFanMode fm = *call.get_fan_mode();
                    switch (fm)
                    {
                    case climate::CLIMATE_FAN_AUTO: { std::vector<uint8_t> msg(speed_auto, speed_auto + sizeof(speed_auto)); send_message("Set Fan Speed to Auto", msg); break; }
                    case climate::CLIMATE_FAN_LOW: { std::vector<uint8_t> msg(speed_low, speed_low + sizeof(speed_low)); send_message("Set Fan Speed to Low", msg); break; }
                    case climate::CLIMATE_FAN_MEDIUM: { std::vector<uint8_t> msg(speed_med, speed_med + sizeof(speed_med)); send_message("Set Fan Speed to Medium", msg); break; }
                    case climate::CLIMATE_FAN_HIGH: { std::vector<uint8_t> msg(speed_max, speed_max + sizeof(speed_max)); send_message("Set Fan Speed to High", msg); break; }
                    case climate::CLIMATE_FAN_QUIET: { std::vector<uint8_t> msg(speed_mute, speed_mute + sizeof(speed_mute)); send_message("Set Fan Speed to Quiet", msg); break; }
                    default: break;
                    }
                    fan_mode = fm;
                    this->publish_state();
                }

                if (call.get_swing_mode().has_value())
                {
                    ClimateSwingMode sm = *call.get_swing_mode();
                    if (sm == climate::CLIMATE_SWING_OFF) {
                        std::vector<uint8_t> v(vert_dir, vert_dir + sizeof(vert_dir));
                        std::vector<uint8_t> h(hor_dir, hor_dir + sizeof(hor_dir));
                        send_message("Set Vertical Swing Off", v);
                        send_message("Set Horizontal Swing Off", h);
                    } else if (sm == climate::CLIMATE_SWING_BOTH) {
                        std::vector<uint8_t> v(vert_swing, vert_swing + sizeof(vert_swing));
                        std::vector<uint8_t> h(hor_swing, hor_swing + sizeof(hor_swing));
                        send_message("Set Vertical Swing On", v);
                        send_message("Set Horizontal Swing On", h);
                    } else if (sm == climate::CLIMATE_SWING_VERTICAL) {
                        std::vector<uint8_t> v(vert_swing, vert_swing + sizeof(vert_swing));
                        std::vector<uint8_t> h(hor_dir, hor_dir + sizeof(hor_dir));
                        send_message("Set Vertical Swing On", v);
                        send_message("Set Horizontal Swing Off", h);
                    } else if (sm == climate::CLIMATE_SWING_HORIZONTAL) {
                        std::vector<uint8_t> v(vert_dir, vert_dir + sizeof(vert_dir));
                        std::vector<uint8_t> h(hor_swing, hor_swing + sizeof(hor_swing));
                        send_message("Set Vertical Swing Off", v);
                        send_message("Set Horizontal Swing On", h);
                    }
                    swing_mode = sm;
                    this->publish_state();
                }

                if (call.get_preset().has_value())
                {
                    ClimatePreset pre = *call.get_preset();
                    switch (pre)
                    {
                    case climate::CLIMATE_PRESET_NONE: {
                        std::vector<uint8_t> t(turbo_off, turbo_off + sizeof(turbo_off));
                        std::vector<uint8_t> e(energysave_off, energysave_off + sizeof(energysave_off));
                        send_message("Disable Turbo", t);
                        send_message("Disable Energy Save", e);
                        break;
                    }
                    case climate::CLIMATE_PRESET_BOOST: {
                        std::vector<uint8_t> msg(turbo_on, turbo_on + sizeof(turbo_on));
                        send_message("Enable Turbo", msg);
                        break;
                    }
                    case climate::CLIMATE_PRESET_ECO: {
                        std::vector<uint8_t> msg(energysave_on, energysave_on + sizeof(energysave_on));
                        send_message("Enable Energy Save", msg);
                        break;
                    }
                    default: break;
                    }
                    preset = pre;
                    this->publish_state();
                }
            }

            ClimateTraits traits() override
            {
                auto traits = climate::ClimateTraits();
                traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
                traits.set_visual_min_temperature(16);
                traits.set_visual_max_temperature(32);
                traits.set_visual_temperature_step(1);
                traits.set_supported_modes({
                    climate::CLIMATE_MODE_OFF,
                    climate::CLIMATE_MODE_COOL,
                    climate::CLIMATE_MODE_HEAT,
                    climate::CLIMATE_MODE_FAN_ONLY,
                    climate::CLIMATE_MODE_DRY,
                });
                traits.set_supported_swing_modes({
                    climate::CLIMATE_SWING_OFF,
                    climate::CLIMATE_SWING_BOTH,
                    climate::CLIMATE_SWING_VERTICAL,
                    climate::CLIMATE_SWING_HORIZONTAL,
                });
                traits.set_supported_fan_modes({
                    climate::CLIMATE_FAN_AUTO,
                    climate::CLIMATE_FAN_LOW,
                    climate::CLIMATE_FAN_MEDIUM,
                    climate::CLIMATE_FAN_HIGH,
                    climate::CLIMATE_FAN_QUIET,
                });
                traits.set_supported_presets({
                    climate::CLIMATE_PRESET_NONE,
                    climate::CLIMATE_PRESET_BOOST,
                    climate::CLIMATE_PRESET_ECO,
                });
                traits.add_feature_flags(climate::CLIMATE_SUPPORTS_ACTION);
                return traits;
            }

            sensor::Sensor *compressor_frequency{nullptr};
            sensor::Sensor *compressor_frequency_setting{nullptr};
            sensor::Sensor *compressor_frequency_send{nullptr};
            sensor::Sensor *outdoor_temperature{nullptr};
            sensor::Sensor *outdoor_condenser_temperature{nullptr};
            sensor::Sensor *compressor_exhaust_temperature{nullptr};
            sensor::Sensor *target_exhaust_temperature{nullptr};
            sensor::Sensor *indoor_pipe_temperature{nullptr};
            sensor::Sensor *indoor_humidity_setting{nullptr};
            sensor::Sensor *indoor_humidity_status{nullptr};
            sensor::Sensor *power_sensor{nullptr};

            GPIOPin *re_pin{nullptr};
            GPIOPin *de_pin{nullptr};

        private:
            struct Message {
                std::string description;
                std::vector<uint8_t> payload;
            };
            enum SendState { IDLE, WAITING_ACK };
            SendState send_state = IDLE;
            uint32_t send_timestamp = 0;
            uint32_t last_send_time = 0;
            uint32_t last_read_time = 0;
            uint32_t idle_until = 0;
            std::string current_desc;
            std::queue<Message> message_queue;
            char desc_buffer[64];
            std::string temperature_unit = "F";
            const uint8_t* temp_f_messages[26] = {
                temp_61_F, temp_62_F, temp_63_F, temp_64_F, temp_65_F, temp_66_F, temp_67_F, temp_68_F, temp_69_F, temp_70_F,
                temp_71_F, temp_72_F, temp_73_F, temp_74_F, temp_75_F, temp_76_F, temp_77_F, temp_78_F, temp_79_F, temp_80_F,
                temp_81_F, temp_82_F, temp_83_F, temp_84_F, temp_85_F, temp_86_F
            };
            const uint8_t* temp_c_messages[17] = {
                temp_16_C, temp_17_C, temp_18_C, temp_19_C, temp_20_C, temp_21_C, temp_22_C, temp_23_C, temp_24_C, temp_25_C,
                temp_26_C, temp_27_C, temp_28_C, temp_29_C, temp_30_C, temp_31_C, temp_32_C
            };

            float heat_tgt_temp = 16.1111f;
            float cool_tgt_temp = 26.6667f;
            static const int UART_BUF_SIZE = 180;
            uint8_t uart_buf[UART_BUF_SIZE];

            int get_response(const uint8_t input, uint8_t *out)
            {
                static std::vector<uint8_t> msg_buffer;
                static uint16_t checksum = 0;
                static bool in_message = false;
                static int expected_msg_size = 0;

                if (DEBUG_LOGGING) ESP_LOGD("aircon_climate", "get_response: input=0x%02X, in_message=%d, buffer_size=%zu", input, in_message, msg_buffer.size());

                if (!in_message) {
                    if (input == 0xF4) {
                        msg_buffer.clear();
                        msg_buffer.push_back(input);
                        checksum = 0;
                        expected_msg_size = 0;
                        in_message = true;
                        if (send_state == WAITING_ACK) {
                            send_state = IDLE;
                        }
                    }
                    return 0;
                } else {
                    if (input == 0xF4 && !msg_buffer.empty() && msg_buffer.back() == 0xF4) {
                        return 0;
                    }

                    msg_buffer.push_back(input);
                    size_t idx = msg_buffer.size() - 1;
                    const uint8_t expected[16] = {0xF4,0xF5,0x01,0x40,0x97,0x01,0x00,0xFE,0x01,0x01,0x01,0x01,0x00,0x66,0x00,0x01};

                    if (idx >= 2 && idx < (size_t)(expected_msg_size - 4)) {
                        checksum += msg_buffer[idx];
                    }

                    if (idx < 16) {
                        if (idx != 4 && idx != 13 && msg_buffer[idx] != expected[idx]) {
                            ESP_LOGE("aircon_climate", "Header mismatch at byte %zu: expected %02X, got %02X", idx, expected[idx], msg_buffer[idx]);
                            in_message = false;
                            msg_buffer.clear();
                            return 0;
                        }
                        if (idx == 4) {
                            expected_msg_size = msg_buffer[4] + 9;
                            if (expected_msg_size > UART_BUF_SIZE) {
                                ESP_LOGE("aircon_climate", "Message size too large: %d", expected_msg_size);
                                in_message = false;
                                msg_buffer.clear();
                                return 0;
                            }
                        }
                    } else {
                        if (idx == (size_t)(expected_msg_size - 3)) {
                            uint16_t rxd_checksum = (msg_buffer[expected_msg_size - 4] << 8) | msg_buffer[expected_msg_size - 3];
                            if (checksum != rxd_checksum) {
                                ESP_LOGE("aircon_climate", "CRC check failed. Computed: %d Received: %d", checksum, rxd_checksum);
                                in_message = false;
                                msg_buffer.clear();
                                return 0;
                            }
                        } else if (idx == (size_t)(expected_msg_size - 2)) {
                            if (msg_buffer[idx] != 0xF4) {
                                ESP_LOGE("aircon_climate", "Frame end F4 mismatch");
                                in_message = false;
                                msg_buffer.clear();
                                return 0;
                            }
                        } else if (idx == (size_t)(expected_msg_size - 1)) {
                            if (msg_buffer[idx] != 0xFB) {
                                ESP_LOGE("aircon_climate", "Frame end FB mismatch");
                                in_message = false;
                                msg_buffer.clear();
                                return 0;
                            } else {
                                size_t msg_size = msg_buffer.size();
                                ESP_LOGD("aircon_climate", "Received %zu bytes.", msg_size);
                                memcpy(out, msg_buffer.data(), msg_size);
                                in_message = false;
                                msg_buffer.clear();
                                return msg_size;
                            }
                        }
                    }
                    return 0;
                }
            }

            void send_message(const std::string& desc, const std::vector<uint8_t>& msg)
            {
                if (msg.empty()) return;
                message_queue.push({desc, msg});
            }

            void request_update()
            {
                std::vector<uint8_t> req(request_status, request_status + sizeof(request_status));
                ESP_LOGD("aircon_climate", "Requesting update.");
                send_message("Status Request", req);
            }

            void set_sensor(Sensor *sensor, float value)
            {
                if (sensor != nullptr && (!sensor->has_state() || sensor->get_raw_state() != value))
                    sensor->publish_state(value);
            }

            void set_temp(float temp)
            {
                if (temperature_unit == "C")
                {
                    uint8_t temp_c = roundf(temp);
                    if (temp_c >= 16 && temp_c <= 32)
                    {
                        int index = temp_c - 16;
                        size_t msg_size = (index == 0) ? sizeof(temp_16_C) : sizeof(temp_17_C);
                        std::vector<uint8_t> msg(temp_c_messages[index], temp_c_messages[index] + msg_size);
                        snprintf(desc_buffer, sizeof(desc_buffer), "Set Temperature to %d°C", temp_c);
                        ESP_LOGD("aircon_climate", "Enqueuing %s", desc_buffer);
                        send_message(desc_buffer, msg);
                    }
                }
                else
                {
                    uint8_t temp_f = roundf(temp * 1.8f + 32);
                    if (temp_f >= 61 && temp_f <= 86)
                    {
                        int index = temp_f - 61;
                        std::vector<uint8_t> msg(temp_f_messages[index], temp_f_messages[index] + sizeof(temp_61_F));
                        snprintf(desc_buffer, sizeof(desc_buffer), "Set Temperature to %d°F", temp_f);
                        ESP_LOGD("aircon_climate", "Enqueuing %s", desc_buffer);
                        send_message(desc_buffer, msg);
                    }
                }
            }

        };
    }
}
