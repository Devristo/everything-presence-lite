#pragma once

#include <unordered_set>
#include <unordered_map>
#include <iomanip>
#include <map>
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "frame_iterator.h"
#include "command_queue.h"

namespace esphome {
namespace ld6001a {

enum ProtocolMode {
  PROTOCOL_MODE_SIMPLE = 0,
  PROTOCOL_MODE_OUTPUT_STRING = 1,
  PROTOCOL_MODE_DEBUG = 2,
  PROTOCOL_MODE_DETAILED = 3,
};


class LD6001AComponent : public Component, public uart::UARTDevice, public FrameHandler {
 public:
  LD6001AComponent();
  void setup() override;
  void dump_config() override;
  void loop() override;

  void start();
  void stop();
  void reset();
  void set_protocol_mode(ProtocolMode mode);
  void config_factory_settings();
  void config_distance_sensitivity(uint8_t sensitivity);
  void config_heartbeat_interval(uint16_t interval_s);
  void config_ground_radius(uint16_t radius_cm);
  void config_vertical_distance(int distance_cm);
  void config_x_range(int x1, int x2);
  void config_y_range(int y1, int y2);
  void config_moving_target_disappearance_time(uint16_t time_ms);
  void config_static_target_disappearance_time(uint16_t time_ms);
  void config_exit_boundary_time(uint16_t time_ms);

  void on_ack_response() override;
  void on_simple_radar_response(const uint8_t people_counted);
  void on_detailed_radar_response(const std::vector<Person> people);
  void on_invalid_frame() override;
  
protected:
  FrameParser frame_parser_{*this};
  CommandQueue command_queue_{[this](const std::string &cmd) { this->write_str(cmd.c_str()); }};

  std::vector<Person> detailed_people_response_{};
  uint8_t people_counted_ = 0;
};

}  // namespace ld6001a
}  // namespace esphome
