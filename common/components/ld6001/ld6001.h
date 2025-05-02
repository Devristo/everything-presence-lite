#pragma once

#include <unordered_set>
#include <iomanip>
#include <map>
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

#ifndef M_PI
#define M_PI 3.14
#endif

namespace esphome {
namespace ld6001 {

// Constants
static const uint8_t DEFAULT_PRESENCE_TIMEOUT = 5;  // Timeout to reset presense status 5 sec.
static const uint16_t MAX_LINE_LENGTH = 1024;          // Max characters for serial buffer
static const uint8_t MAX_TARGETS = 10;               // Max 3 Targets in LD6001
static const uint8_t MAX_ZONES = 4;                 // Max 3 Zones in LD6001

struct Target {
  uint8_t id;
  uint8_t pitch_angle;
  uint8_t horizontal_angle;
  uint8_t distance;
  int16_t x;
  int16_t y;
};

struct TargetInfo {
  uint8_t targets;
  Target target_data[MAX_TARGETS];
};

// Zone coordinate struct
struct Zone {
  int16_t x1 = 0;
  int16_t y1 = 0;
  int16_t x2 = 0;
  int16_t y2 = 0;
};

#ifdef USE_NUMBER
struct ZoneOfNumbers {
  number::Number *x1 = nullptr;
  number::Number *y1 = nullptr;
  number::Number *x2 = nullptr;
  number::Number *y2 = nullptr;
};
#endif

class LD6001Component : public Component, public uart::UARTDevice {
#ifdef USE_SENSOR
  SUB_SENSOR(target_count)
#endif

#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(version)
  SUB_TEXT_SENSOR(mac)
#endif

 public:
  LD6001Component();
  void setup() override;
  void dump_config() override;
  void loop() override;

  void send_radar_request();
  void set_throttle(uint16_t value) { this->throttle_ = value; };
  void read_all_info();
  void query_zone_info();

#ifdef USE_SENSOR
  void set_move_x_sensor(uint8_t target, sensor::Sensor *s);
  void set_move_y_sensor(uint8_t target, sensor::Sensor *s);
  void set_move_pitch_angle_sensor(uint8_t target, sensor::Sensor *s);
  void set_move_horizontal_angle_sensor(uint8_t target, sensor::Sensor *s);
  void set_move_distance_sensor(uint8_t target, sensor::Sensor *s);
  void set_zone_target_count_sensor(uint8_t zone, sensor::Sensor *s);
#endif

 protected:
  void get_version_();

  void update_sensors();
  void read_version_frame(uint8_t *buffer);
  void read_radar_frame(uint8_t *buffer, uint8_t buffer_pos, uint8_t total_length);
  void update_last_seen(uint8_t target_id);

  TargetInfo target_info_ = {};
  Zone zone_config_[MAX_ZONES];
  uint8_t buffer_pos_ = 0;  // where to resume processing/populating buffer
  uint8_t buffer_data_[MAX_LINE_LENGTH];
  uint32_t last_periodic_millis_ = 0;
  uint32_t presence_millis_ = 0;
  uint32_t still_presence_millis_ = 0;
  uint32_t moving_presence_millis_ = 0;
  uint16_t throttle_ = 0;
  uint16_t timeout_ = 5;

  std::unordered_set<uint8_t> announce_entry;
  std::deque<uint8_t> removed_targets;
  std::unordered_map<uint8_t, uint32_t> entry_times;
  std::unordered_map<uint8_t, uint32_t> last_seen_times;

  uint8_t zone_type_ = 0;
  std::string version_{};
  std::string mac_{};
#ifdef USE_NUMBER
  ESPPreferenceObject pref_;  // only used when numbers are in use
  ZoneOfNumbers zone_numbers_[MAX_ZONES];
#endif
#ifdef USE_SENSOR
  std::vector<sensor::Sensor *> move_x_sensors_ = std::vector<sensor::Sensor *>(MAX_TARGETS);
  std::vector<sensor::Sensor *> move_y_sensors_ = std::vector<sensor::Sensor *>(MAX_TARGETS);
  std::vector<sensor::Sensor *> move_pitch_angle_sensors_ = std::vector<sensor::Sensor *>(MAX_TARGETS);
  std::vector<sensor::Sensor *> move_horizontal_angle_sensors_ = std::vector<sensor::Sensor *>(MAX_TARGETS);
  std::vector<sensor::Sensor *> move_distance_sensors_ = std::vector<sensor::Sensor *>(MAX_TARGETS);
  std::vector<sensor::Sensor *> zone_target_count_sensors_ = std::vector<sensor::Sensor *>(MAX_ZONES);
#endif
};

}  // namespace ld6001
}  // namespace esphome
