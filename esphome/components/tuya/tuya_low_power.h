#pragma once

#include "tuya_base.h"

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

using TuyaLowPowerCommand = TuyaCommand<TuyaLowPowerCommandType>;

class TuyaLowPower : public TuyaBase<TuyaLowPowerCommandType> {
 public:
  void setup() override;
  void dump_config() override;

 protected:
  void handle_command_(uint8_t command, uint8_t version, const uint8_t *buffer, size_t len) override;
  void before_send_raw_command_(TuyaLowPowerCommand command) override {};
  void send_datapoint_command_(uint8_t datapoint_id, TuyaDatapointType datapoint_type, std::vector<uint8_t> data) override {};
  void send_wifi_status_();
#ifdef USE_TIME
  void send_local_time_();
#endif
};

}  // namespace tuya
}  // namespace esphome
