#include "tuya_low_power.h"
#include "esphome/components/network/util.h"
#include "esphome/core/util.h"

namespace esphome {
namespace tuya {

static const char *const TAG = "tuya_low_power";
static const int COMMAND_DELAY = 10;
static const int RECEIVE_TIMEOUT = 300;

void TuyaLowPower::setup() {
  // do nothing
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

    case TuyaLowPowerCommandType::WIFI_RESET:
      ESP_LOGE(TAG, "WIFI_RESET is not handled");
      break;

    case TuyaLowPowerCommandType::WIFI_SELECT:
      ESP_LOGE(TAG, "WIFI_SELECT is not handled");
      break;

    case TuyaLowPowerCommandType::DATAPOINT_REPORT:
      this->handle_datapoints_(buffer, len);
      break;

    case TuyaLowPowerCommandType::LOCAL_TIME_QUERY:
#ifdef USE_TIME
      if (this->time_id_.has_value()) {
        this->send_local_time_();
        auto *time_id = *this->time_id_;
        time_id->add_on_time_sync_callback([this] { this->send_local_time_(); });
      } else {
        ESP_LOGW(TAG, "LOCAL_TIME_QUERY is not handled because time is not configured");
      }
#else
      ESP_LOGE(TAG, "LOCAL_TIME_QUERY is not handled");
#endif
      break;

    case TuyaLowPowerCommandType::WIFI_TEST:
      this->send_command_(TuyaLowPowerCommand{.cmd = TuyaLowPowerCommandType::WIFI_TEST, .payload = std::vector<uint8_t>{0x00, 0x00}});
      break;

    case TuyaLowPowerCommandType::DATAPOINT_REPORT_STORED:
      ESP_LOGE(TAG, "DATAPOINT_REPORT_STORED is not handled");
      break;

    case TuyaLowPowerCommandType::OBTAIN_DP_CACHE:
      this->send_command_(TuyaLowPowerCommand{.cmd = TuyaLowPowerCommandType::OBTAIN_DP_CACHE, .payload = std::vector<uint8_t>{0x00}});
      break;

    case TuyaLowPowerCommandType::FIRMWARE_UPGRADE:
      ESP_LOGE(TAG, "FIRMWARE_UPGRADE is not handled");
      break;

    case TuyaLowPowerCommandType::SIGNAL_STRENGTH_QUERY:
      this->send_command_(TuyaLowPowerCommand{.cmd = TuyaLowPowerCommandType::SIGNAL_STRENGTH_QUERY, .payload = std::vector<uint8_t>{0x01, 100}});
      break;

    default:
      ESP_LOGE(TAG, "Invalid low power command (0x%02X) received", command);
  }
}

void TuyaLowPower::send_raw_command_(TuyaLowPowerCommand command) {
  uint8_t len_hi = (uint8_t)(command.payload.size() >> 8);
  uint8_t len_lo = (uint8_t)(command.payload.size() & 0xFF);
  uint8_t version = 0;

  ESP_LOGV(TAG, "Sending Tuya: CMD=0x%02X VERSION=%u DATA=[%s] INIT_STATE=%u", static_cast<uint8_t>(command.cmd),
           version, format_hex_pretty(command.payload).c_str(), static_cast<uint8_t>(this->init_state_));

  this->write_array({0x55, 0xAA, version, (uint8_t) command.cmd, len_hi, len_lo});
  if (!command.payload.empty())
    this->write_array(command.payload.data(), command.payload.size());

  uint8_t checksum = 0x55 + 0xAA + (uint8_t) command.cmd + len_hi + len_lo;
  for (auto &data : command.payload)
    checksum += data;
  this->write_byte(checksum);
}

void TuyaLowPower::process_command_queue_() {
  uint32_t now = millis();
  uint32_t delay = now - this->last_command_timestamp_;

  if (now - this->last_rx_char_timestamp_ > RECEIVE_TIMEOUT) {
    this->rx_message_.clear();
  }

  if (this->expected_response_.has_value() && delay > RECEIVE_TIMEOUT) {
    this->expected_response_.reset();
  }

  // Left check of delay since last command in case there's ever a command sent by calling send_raw_command_ directly
  if (delay > COMMAND_DELAY && !this->command_queue_.empty() && this->rx_message_.empty() &&
      !this->expected_response_.has_value()) {
    this->send_raw_command_(command_queue_.front());
    this->command_queue_.erase(command_queue_.begin());
  }
}

void TuyaLowPower::send_command_(const TuyaLowPowerCommand &command) {
  command_queue_.push_back(command);
  process_command_queue_();
}

void TuyaLowPower::send_empty_command_(TuyaLowPowerCommandType command) {
  send_command_(TuyaLowPowerCommand{.cmd = command, .payload = std::vector<uint8_t>{}});
}

void TuyaLowPower::send_wifi_status_() {
  uint8_t status = 0x02;
  if (network::is_connected()) {
    status = 0x03;

    if (remote_is_connected()) {
      status = 0x04;
    }
  }

  ESP_LOGD(TAG, "Sending WiFi Status");
  this->wifi_status_ = status;
  this->send_command_(TuyaLowPowerCommand{.cmd = TuyaLowPowerCommandType::WIFI_STATE, .payload = std::vector<uint8_t>{status}});
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
