#pragma once

#include "tuya_base.h"

namespace esphome {
namespace tuya {

enum class TuyaCommandType : uint8_t {
  HEARTBEAT = 0x00,
  PRODUCT_QUERY = 0x01,
  CONF_QUERY = 0x02,
  WIFI_STATE = 0x03,
  WIFI_RESET = 0x04,
  WIFI_SELECT = 0x05,
  DATAPOINT_DELIVER = 0x06,
  DATAPOINT_REPORT = 0x07,
  DATAPOINT_QUERY = 0x08,
  WIFI_TEST = 0x0E,
  LOCAL_TIME_QUERY = 0x1C,
};

enum class TuyaInitState : uint8_t {
  INIT_HEARTBEAT = 0x00,
  INIT_PRODUCT,
  INIT_CONF,
  INIT_WIFI,
  INIT_DATAPOINT,
  INIT_DONE,
};

using TuyaNormalCommand = TuyaCommand<TuyaCommandType>;

class Tuya : public TuyaBase<TuyaCommandType> {
 public:
  void setup() override;
  void dump_config() override;
  TuyaInitState get_init_state();

 protected:
  void handle_command_(uint8_t command, uint8_t version, const uint8_t *buffer, size_t len) override;
  void before_send_raw_command_(TuyaNormalCommand command) override;
  void send_datapoint_command_(uint8_t datapoint_id, TuyaDatapointType datapoint_type, std::vector<uint8_t> data) override;
  void send_wifi_status_();
#ifdef USE_TIME
  void send_local_time_();
#endif

  TuyaInitState init_state_ = TuyaInitState::INIT_HEARTBEAT;
  uint8_t protocol_version_ = -1;
  int gpio_status_ = -1;
  int gpio_reset_ = -1;
  uint8_t wifi_status_ = -1;
};

}  // namespace tuya
}  // namespace esphome
