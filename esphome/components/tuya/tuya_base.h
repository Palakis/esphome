#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif

namespace esphome {
namespace tuya {

enum class TuyaDatapointType : uint8_t {
  RAW = 0x00,      // variable length
  BOOLEAN = 0x01,  // 1 byte (0/1)
  INTEGER = 0x02,  // 4 byte
  STRING = 0x03,   // variable length
  ENUM = 0x04,     // 1 byte
  BITMASK = 0x05,  // 1/2/4 bytes
};

struct TuyaDatapoint {
  uint8_t id;
  TuyaDatapointType type;
  size_t len;
  union {
    bool value_bool;
    int value_int;
    uint32_t value_uint;
    uint8_t value_enum;
    uint32_t value_bitmask;
  };
  std::string value_string;
  std::vector<uint8_t> value_raw;
};

struct TuyaDatapointListener {
  uint8_t datapoint_id;
  std::function<void(TuyaDatapoint)> on_datapoint;
};

template <typename TCommandType>
struct TuyaCommand {
  TCommandType cmd;
  std::vector<uint8_t> payload;
};

template <typename TCommandType>
class TuyaBase : public Component, public uart::UARTDevice {
 public:
  float get_setup_priority() const override { return setup_priority::LATE; }
  void loop() override;
  void register_listener(uint8_t datapoint_id, const std::function<void(TuyaDatapoint)> &func);
  void set_raw_datapoint_value(uint8_t datapoint_id, const std::vector<uint8_t> &value);
  void set_boolean_datapoint_value(uint8_t datapoint_id, bool value);
  void set_integer_datapoint_value(uint8_t datapoint_id, uint32_t value);
  void set_string_datapoint_value(uint8_t datapoint_id, const std::string &value);
  void set_enum_datapoint_value(uint8_t datapoint_id, uint8_t value);
  void set_bitmask_datapoint_value(uint8_t datapoint_id, uint32_t value, uint8_t length);
  void force_set_raw_datapoint_value(uint8_t datapoint_id, const std::vector<uint8_t> &value);
  void force_set_boolean_datapoint_value(uint8_t datapoint_id, bool value);
  void force_set_integer_datapoint_value(uint8_t datapoint_id, uint32_t value);
  void force_set_string_datapoint_value(uint8_t datapoint_id, const std::string &value);
  void force_set_enum_datapoint_value(uint8_t datapoint_id, uint8_t value);
  void force_set_bitmask_datapoint_value(uint8_t datapoint_id, uint32_t value, uint8_t length);
#ifdef USE_TIME
  void set_time_id(time::RealTimeClock *time_id) { this->time_id_ = time_id; }
#endif
  void add_ignore_mcu_update_on_datapoints(uint8_t ignore_mcu_update_on_datapoints) {
    this->ignore_mcu_update_on_datapoints_.push_back(ignore_mcu_update_on_datapoints);
  }
  void add_on_initialized_callback(std::function<void()> callback) {
    this->initialized_callback_.add(std::move(callback));
  }

 protected:
  void handle_char_(uint8_t c);
  void handle_datapoints_(const uint8_t *buffer, size_t len);
  optional<TuyaDatapoint> get_datapoint_(uint8_t datapoint_id);
  bool validate_message_();

  virtual void handle_command_(uint8_t command, uint8_t version, const uint8_t *buffer, size_t len) = 0;
  void send_raw_command_(TuyaCommand<TCommandType> command);
  virtual void before_send_raw_command_(TuyaCommand<TCommandType> command) = 0;
  void process_command_queue_();
  void send_command_(const TuyaCommand<TCommandType> &command);
  void send_empty_command_(TCommandType command);
  void set_numeric_datapoint_value_(uint8_t datapoint_id, TuyaDatapointType datapoint_type, uint32_t value,
                                    uint8_t length, bool forced);
  void set_string_datapoint_value_(uint8_t datapoint_id, const std::string &value, bool forced);
  void set_raw_datapoint_value_(uint8_t datapoint_id, const std::vector<uint8_t> &value, bool forced);
  virtual void send_datapoint_command_(uint8_t datapoint_id, TuyaDatapointType datapoint_type, std::vector<uint8_t> data) = 0;

#ifdef USE_TIME
  optional<time::RealTimeClock *> time_id_{};
#endif
  uint32_t last_command_timestamp_ = 0;
  uint32_t last_rx_char_timestamp_ = 0;
  std::string product_ = std::string("");
  std::vector<TuyaDatapointListener> listeners_;
  std::vector<TuyaDatapoint> datapoints_;
  std::vector<uint8_t> rx_message_;
  std::vector<uint8_t> ignore_mcu_update_on_datapoints_{};
  std::vector<TuyaCommand<TCommandType>> command_queue_;
  optional<TCommandType> expected_response_{};
  CallbackManager<void()> initialized_callback_{};
};

}  // namespace tuya
}  // namespace esphome
