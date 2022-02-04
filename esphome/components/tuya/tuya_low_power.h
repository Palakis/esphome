#pragma once

#include "tuya.h"

namespace esphome {
namespace tuya {

enum class TuyaLowPowerCommandType : uint8_t {
  PRODUCT_QUERY = 0x01,
  WIFI_STATE = 0x02,
  WIFI_RESET = 0x03,
  WIFI_SELECT = 0x04,
  DATAPOINT_REPORT = 0x05,
  LOCAL_TIME_QUERY = 0x06,
  WIFI_TEST = 0x07,
  DATAPOINT_REPORT_STORED = 0x08,
  MODULE_COMMAND = 0x09,
  OBTAIN_DP_CACHE = 0x10,
  FIRMWARE_UPGRADE = 0x0a,
  SIGNAL_STRENGTH_QUERY = 0x0b
};

struct TuyaLowPowerCommand {
  TuyaLowPowerCommandType cmd;
  std::vector<uint8_t> payload;
};

class TuyaLowPower : public Tuya {
 public:
  void setup() override;
  void dump_config() override;

 protected:
  void handle_command_(uint8_t command, uint8_t version, const uint8_t *buffer, size_t len);
  void send_raw_command_(TuyaLowPowerCommand command);
  void process_command_queue_();
  void send_command_(const TuyaLowPowerCommand &command);
  void send_empty_command_(TuyaLowPowerCommandType command);
  void send_wifi_status_();

#ifdef USE_TIME
  void send_local_time_();
#endif
  std::vector<TuyaLowPowerCommand> command_queue_;
  optional<TuyaLowPowerCommandType> expected_response_{};
};

}  // namespace tuya
}  // namespace esphome
