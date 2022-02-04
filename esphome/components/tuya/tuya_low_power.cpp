#include "tuya_low_power.h"
#include "esphome/components/network/util.h"
#include "esphome/core/util.h"

#include "tuya_base.cpp"

namespace esphome {
namespace tuya {

void TuyaLowPower::setup() {
  this->send_wifi_status_();
}

void TuyaLowPower::dump_config() {
  ESP_LOGCONFIG(TAG, "Tuya Low Power:");
  for (auto &info : this->datapoints_) {
    if (info.type == TuyaDatapointType::RAW) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: raw (value: %s)", info.id, format_hex_pretty(info.value_raw).c_str());
    } else if (info.type == TuyaDatapointType::BOOLEAN) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: switch (value: %s)", info.id, ONOFF(info.value_bool));
    } else if (info.type == TuyaDatapointType::INTEGER) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: int value (value: %d)", info.id, info.value_int);
    } else if (info.type == TuyaDatapointType::STRING) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: string value (value: %s)", info.id, info.value_string.c_str());
    } else if (info.type == TuyaDatapointType::ENUM) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: enum (value: %d)", info.id, info.value_enum);
    } else if (info.type == TuyaDatapointType::BITMASK) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: bitmask (value: %x)", info.id, info.value_bitmask);
    } else {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: unknown", info.id);
    }
  }
  ESP_LOGCONFIG(TAG, "  Product: '%s'", this->product_.c_str());
  this->check_uart_settings(9600);
}

void TuyaLowPower::enter_low_power_mode() {
  ESP_LOGI(TAG, "Entering low power mode");
  this->send_command_(TuyaLowPowerCommand{.cmd = TuyaLowPowerCommandType::WIFI_STATE, .payload = std::vector<uint8_t>{0x05}});
  this->expected_response_ = TuyaLowPowerCommandType::WIFI_STATE;
}

void TuyaLowPower::handle_command_(uint8_t command, uint8_t version, const uint8_t *buffer, size_t len) {
  TuyaLowPowerCommandType command_type = (TuyaLowPowerCommandType) command;

  if (this->expected_response_.has_value() && this->expected_response_ == command_type) {
    this->expected_response_.reset();
  }

  switch (command_type) {
    case TuyaLowPowerCommandType::PRODUCT_QUERY: {
      // check it is a valid string made up of printable characters
      bool valid = true;
      for (size_t i = 0; i < len; i++) {
        if (!std::isprint(buffer[i])) {
          valid = false;
          break;
        }
      }
      if (valid) {
        this->product_ = std::string(reinterpret_cast<const char *>(buffer), len);
      } else {
        this->product_ = R"({"p":"INVALID"})";
      }
      break;
    }

    case TuyaLowPowerCommandType::WIFI_STATE:
      this->send_wifi_status_(false);
      break;

    case TuyaLowPowerCommandType::WIFI_RESET:
      ESP_LOGI(TAG, "WIFI_RESET received");
      // ack WIFI_RESET
      this->send_empty_command_(TuyaLowPowerCommandType::WIFI_RESET);
      break;

    case TuyaLowPowerCommandType::WIFI_SELECT:
      ESP_LOGI(TAG, "WIFI_SELECT received with mode 0x%02X", buffer[0]);
      // ack WIFI_SELECT
      this->send_empty_command_(TuyaLowPowerCommandType::WIFI_SELECT);
      break;

    case TuyaLowPowerCommandType::DATAPOINT_REPORT:
      this->handle_datapoints_(buffer, len);
      // ack DATAPOINT_REPORT
      this->send_command_(TuyaLowPowerCommand{.cmd = TuyaLowPowerCommandType::DATAPOINT_REPORT, .payload = std::vector<uint8_t>{0x00}});
      break;

    case TuyaLowPowerCommandType::LOCAL_TIME_QUERY:
#ifdef USE_TIME
      if (this->time_id_.has_value()) {
        this->send_local_time_();
        auto *time_id = *this->time_id_;
        time_id->add_on_time_sync_callback([this] { this->send_local_time_(); });
      } else {
        ESP_LOGW(TAG, "LOCAL_TIME_QUERY is not handled because time is not configured");
        // TODO respond failure to MCU
      }
#else
      ESP_LOGE(TAG, "LOCAL_TIME_QUERY is not handled");
      // TODO respond failure to MCU
#endif
      break;

    case TuyaLowPowerCommandType::WIFI_TEST:
      // response: WIFI_TEST failed
      this->send_command_(TuyaLowPowerCommand{.cmd = TuyaLowPowerCommandType::WIFI_TEST, .payload = std::vector<uint8_t>{0x00, 0x00}});
      break;

    case TuyaLowPowerCommandType::DATAPOINT_REPORT_STORED:
      ESP_LOGE(TAG, "DATAPOINT_REPORT_STORED is not handled");
      break;

    case TuyaLowPowerCommandType::OBTAIN_DP_CACHE:
      // response: OBTAIN_DP_CACHE failed
      this->send_command_(TuyaLowPowerCommand{.cmd = TuyaLowPowerCommandType::OBTAIN_DP_CACHE, .payload = std::vector<uint8_t>{0x00}});
      break;

    case TuyaLowPowerCommandType::FIRMWARE_UPGRADE:
      // response: no firmware upgrade needed
      this->send_command_(TuyaLowPowerCommand{.cmd = TuyaLowPowerCommandType::FIRMWARE_UPGRADE, .payload = std::vector<uint8_t>{0x01}});
      break;

    case TuyaLowPowerCommandType::SIGNAL_STRENGTH_QUERY:
      // response: 100% signal strength
      this->send_command_(TuyaLowPowerCommand{.cmd = TuyaLowPowerCommandType::SIGNAL_STRENGTH_QUERY, .payload = std::vector<uint8_t>{0x01, (uint8_t)100}});
      break;

    default:
      ESP_LOGE(TAG, "Invalid low power command (0x%02X) received", command);
  }
}

void TuyaLowPower::send_wifi_status_(bool is_request) {
  uint8_t status = 0x02;
  if (network::is_connected()) {
    status = 0x03;

    if (remote_is_connected()) {
      status = 0x04;
    }
  }

  ESP_LOGD(TAG, "Sending WiFi Status");
  this->send_command_(TuyaLowPowerCommand{.cmd = TuyaLowPowerCommandType::WIFI_STATE, .payload = std::vector<uint8_t>{status}});

  if (is_request) {
    this->expected_response_ = TuyaLowPowerCommandType::WIFI_STATE;
  }
}

#ifdef USE_TIME
void TuyaLowPower::send_local_time_() {
  std::vector<uint8_t> payload;
  auto *time_id = *this->time_id_;
  time::ESPTime now = time_id->now();
  if (now.is_valid()) {
    uint8_t year = now.year - 2000;
    uint8_t month = now.month;
    uint8_t day_of_month = now.day_of_month;
    uint8_t hour = now.hour;
    uint8_t minute = now.minute;
    uint8_t second = now.second;
    // Tuya days starts from Monday, esphome uses Sunday as day 1
    uint8_t day_of_week = now.day_of_week - 1;
    if (day_of_week == 0) {
      day_of_week = 7;
    }
    ESP_LOGD(TAG, "Sending local time");
    payload = std::vector<uint8_t>{0x01, year, month, day_of_month, hour, minute, second, day_of_week};
  } else {
    // By spec we need to notify MCU that the time was not obtained if this is a response to a query
    ESP_LOGW(TAG, "Sending missing local time");
    payload = std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  }
  this->send_command_(TuyaLowPowerCommand{.cmd = TuyaLowPowerCommandType::LOCAL_TIME_QUERY, .payload = payload});
}
#endif

}  // namespace tuya
}  // namespace esphome
