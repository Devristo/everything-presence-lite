#include "ld6001.h"
#include <utility>
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#include "esphome/core/component.h"

#define highbyte(val) (uint8_t)((val) >> 8)
#define lowbyte(val) (uint8_t)((val) &0xff)

namespace esphome {
namespace ld6001 {

static const char *const TAG = "ld6001";

static const uint8_t CMD_GET_VERSION[] = {0x44, 0x11, 0x00, 0x00, 0x55, 0x4B};
static const uint8_t CMD_RADAR_REQUEST_PRECISE[] = {0x44, 0x62, 0x08, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBE, 0x4B};

static inline uint16_t convert_seconds_to_ms(uint16_t value) { return value * 1000; };

static inline std::string convert_signed_int_to_hex(int value) {
  auto value_as_str = str_snprintf("%04x", 4, value & 0xFFFF);
  return value_as_str;
}

static inline void convert_int_values_to_hex(const int *values, uint8_t *bytes) {
  for (int i = 0; i < 4; i++) {
    std::string temp_hex = convert_signed_int_to_hex(values[i]);
    bytes[i * 2] = std::stoi(temp_hex.substr(2, 2), nullptr, 16);      // Store high byte
    bytes[i * 2 + 1] = std::stoi(temp_hex.substr(0, 2), nullptr, 16);  // Store low byte
  }
}

static inline int16_t decode_coordinate(uint8_t low_byte, uint8_t high_byte) {
  int16_t coordinate = (high_byte & 0x7F) << 8 | low_byte;
  if ((high_byte & 0x80) == 0) {
    coordinate = -coordinate;
  }
  return coordinate;  // mm
}

static inline int16_t decode_speed(uint8_t low_byte, uint8_t high_byte) {
  int16_t speed = (high_byte & 0x7F) << 8 | low_byte;
  if ((high_byte & 0x80) == 0) {
    speed = -speed;
  }
  return speed * 10;  // mm/s
}

static inline int16_t hex_to_signed_int(const uint8_t *buffer, uint8_t offset) {
  uint16_t hex_val = (buffer[offset + 1] << 8) | buffer[offset];
  int16_t dec_val = static_cast<int16_t>(hex_val);
  if (dec_val & 0x8000) {
    dec_val -= 65536;
  }
  return dec_val;
}

static inline float calculate_angle(float base, float hypotenuse) {
  if (base < 0.0 || hypotenuse <= 0.0) {
    return 0.0;
  }
  float angle_radians = std::acos(base / hypotenuse);
  float angle_degrees = angle_radians * (180.0 / M_PI);
  return angle_degrees;
}

LD6001Component::LD6001Component() {}

void LD6001Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HLK-LD6001...");
#ifdef USE_NUMBER
  if (this->presence_timeout_number_ != nullptr) {
    this->pref_ = global_preferences->make_preference<float>(this->presence_timeout_number_->get_object_id_hash());
    this->set_presence_timeout();
  }
#endif
  this->read_all_info();
}

void LD6001Component::dump_config() {
  ESP_LOGCONFIG(TAG, "HLK-LD6001 Human motion tracking radar module:");
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "TargetBinarySensor", this->target_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "MovingTargetBinarySensor", this->moving_target_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "StillTargetBinarySensor", this->still_target_binary_sensor_);
#endif
#ifdef USE_SWITCH
  LOG_SWITCH("  ", "BluetoothSwitch", this->bluetooth_switch_);
  LOG_SWITCH("  ", "MultiTargetSwitch", this->multi_target_switch_);
#endif
#ifdef USE_BUTTON
  LOG_BUTTON("  ", "ResetButton", this->reset_button_);
  LOG_BUTTON("  ", "RestartButton", this->restart_button_);
#endif
#ifdef USE_SENSOR
  LOG_SENSOR("  ", "TargetCountSensor", this->target_count_sensor_);
  LOG_SENSOR("  ", "StillTargetCountSensor", this->still_target_count_sensor_);
  LOG_SENSOR("  ", "MovingTargetCountSensor", this->moving_target_count_sensor_);
  for (sensor::Sensor *s : this->move_x_sensors_) {
    LOG_SENSOR("  ", "NthTargetXSensor", s);
  }
  for (sensor::Sensor *s : this->move_y_sensors_) {
    LOG_SENSOR("  ", "NthTargetYSensor", s);
  }
  for (sensor::Sensor *s : this->move_angle_sensors_) {
    LOG_SENSOR("  ", "NthTargetAngleSensor", s);
  }
  for (sensor::Sensor *s : this->move_distance_sensors_) {
    LOG_SENSOR("  ", "NthTargetDistanceSensor", s);
  }
  for (sensor::Sensor *s : this->zone_target_count_sensors_) {
    LOG_SENSOR("  ", "NthZoneTargetCountSensor", s);
  }
#endif
#ifdef USE_NUMBER
  for (auto n : this->zone_numbers_) {
    LOG_NUMBER("  ", "ZoneX1Number", n.x1);
    LOG_NUMBER("  ", "ZoneY1Number", n.y1);
    LOG_NUMBER("  ", "ZoneX2Number", n.x2);
    LOG_NUMBER("  ", "ZoneY2Number", n.y2);
  }
#endif
  ESP_LOGCONFIG(TAG, "  Throttle : %ums", this->throttle_);
  ESP_LOGCONFIG(TAG, "  MAC Address : %s", const_cast<char *>(this->mac_.c_str()));
  ESP_LOGCONFIG(TAG, "  Firmware version : %s", const_cast<char *>(this->version_.c_str()));
}

void LD6001Component::loop() {
  while (this->available()) {
    this->readline_(read(), this->buffer_data_, MAX_LINE_LENGTH);
  }
}

// Count targets in zone
uint8_t LD6001Component::count_targets_in_zone_(const Zone &zone) {
  uint8_t count = 0;
  for (auto &index : this->target_info_) {
    if (index.x > zone.x1 && index.x < zone.x2 && index.y > zone.y1 && index.y < zone.y2) {
      count++;
    }
  }
  return count;
}

// Read all info from LD6001 buffer
void LD6001Component::read_all_info() {
  this->get_version_();
}

// Get LD6001 firmware version
void LD6001Component::get_version_() {
  ESP_LOGV(TAG, "Sending get version request");
  this->write_array(CMD_GET_VERSION);
}

void LD6001Component::send_radar_request() {
  ESP_LOGV(TAG, "Sending precise radar request");
  this->write_array(CMD_RADAR_REQUEST_PRECISE);
}

// LD6001 Radar data message:
//  [AA FF 03 00] [0E 03 B1 86 10 00 40 01] [00 00 00 00 00 00 00 00] [00 00 00 00 00 00 00 00] [55 CC]
//   Header       Target 1                  Target 2                  Target 3                  End
void LD6001Component::handle_periodic_data_(uint8_t *buffer, uint8_t len) {
  if (len < 29) {  // header (4 bytes) + 8 x 3 target data + footer (2 bytes)
    ESP_LOGE(TAG, "Periodic data: invalid message length");
    return;
  }
  if (buffer[0] != 0xAA || buffer[1] != 0xFF || buffer[2] != 0x03 || buffer[3] != 0x00) {  // header
    ESP_LOGE(TAG, "Periodic data: invalid message header");
    return;
  }
  if (buffer[len - 2] != 0x55 || buffer[len - 1] != 0xCC) {  // footer
    ESP_LOGE(TAG, "Periodic data: invalid message footer");
    return;
  }

  auto current_millis = millis();
  if (current_millis - this->last_periodic_millis_ < this->throttle_) {
    ESP_LOGV(TAG, "Throttling: %d", this->throttle_);
    return;
  }

  this->last_periodic_millis_ = current_millis;

  int16_t target_count = 0;
  int16_t still_target_count = 0;
  int16_t moving_target_count = 0;
  int16_t start = 0;
  int16_t val = 0;
  uint8_t index = 0;
  int16_t tx = 0;
  int16_t ty = 0;
  int16_t td = 0;
  int16_t angle = 0;

#if defined(USE_BINARY_SENSOR) || defined(USE_SENSOR) || defined(USE_TEXT_SENSOR)
  // Loop thru targets
  for (index = 0; index < MAX_TARGETS; index++) {
#ifdef USE_SENSOR
    // X
    start = TARGET_X + index * 8;
    sensor::Sensor *sx = this->move_x_sensors_[index];
    if (sx != nullptr) {
      val = ld6001::decode_coordinate(buffer[start], buffer[start + 1]);
      tx = val;
      sx->publish_state(val);
    }
    // Y
    start = TARGET_Y + index * 8;
    sensor::Sensor *sy = this->move_y_sensors_[index];
    if (sy != nullptr) {
      val = ld6001::decode_coordinate(buffer[start], buffer[start + 1]);
      ty = val;
      sy->publish_state(val);
    }
#endif
    // DISTANCE
    val = (uint16_t) sqrt(
        pow(ld6001::decode_coordinate(buffer[TARGET_X + index * 8], buffer[(TARGET_X + index * 8) + 1]), 2) +
        pow(ld6001::decode_coordinate(buffer[TARGET_Y + index * 8], buffer[(TARGET_Y + index * 8) + 1]), 2));
    td = val;
    if (val > 0) {
      target_count++;
    }
#ifdef USE_SENSOR
    sensor::Sensor *sd = this->move_distance_sensors_[index];
    if (sd != nullptr) {
      sd->publish_state(val);
    }
    // ANGLE
    angle = calculate_angle(static_cast<float>(ty), static_cast<float>(td));
    if (tx > 0) {
      angle = angle * -1;
    }
    sensor::Sensor *sa = this->move_angle_sensors_[index];
    if (sa != nullptr) {
      sa->publish_state(angle);
    }
#endif

    // Store target info for zone target count
    this->target_info_[index].x = tx;
    this->target_info_[index].y = ty;

  }  // End loop thru targets
#endif

#ifdef USE_SENSOR
  // Loop thru zones
  uint8_t zone_still_targets = 0;
  uint8_t zone_moving_targets = 0;
  uint8_t zone_all_targets = 0;
  for (index = 0; index < MAX_ZONES; index++) {
    zone_all_targets = zone_still_targets + zone_moving_targets;

    // Publish All Target Count in Zones
    sensor::Sensor *sztc = this->zone_target_count_sensors_[index];
    if (sztc != nullptr) {
      sztc->publish_state(zone_all_targets);
    }

  }  // End loop thru zones

  // Target Count
  if (this->target_count_sensor_ != nullptr) {
    this->target_count_sensor_->publish_state(target_count);
  }
  // Still Target Count
  if (this->still_target_count_sensor_ != nullptr) {
    this->still_target_count_sensor_->publish_state(still_target_count);
  }
  // Moving Target Count
  if (this->moving_target_count_sensor_ != nullptr) {
    this->moving_target_count_sensor_->publish_state(moving_target_count);
  }
#endif

#ifdef USE_BINARY_SENSOR
  // Target Presence
  if (this->target_binary_sensor_ != nullptr) {
    if (target_count > 0) {
      this->target_binary_sensor_->publish_state(true);
    } else {
      if (this->get_timeout_status_(this->presence_millis_)) {
        this->target_binary_sensor_->publish_state(false);
      } else {
        ESP_LOGV(TAG, "Clear presence waiting timeout: %d", this->timeout_);
      }
    }
  }
  // Moving Target Presence
  if (this->moving_target_binary_sensor_ != nullptr) {
    if (moving_target_count > 0) {
      this->moving_target_binary_sensor_->publish_state(true);
    } else {
      if (this->get_timeout_status_(this->moving_presence_millis_)) {
        this->moving_target_binary_sensor_->publish_state(false);
      }
    }
  }
  // Still Target Presence
  if (this->still_target_binary_sensor_ != nullptr) {
    if (still_target_count > 0) {
      this->still_target_binary_sensor_->publish_state(true);
    } else {
      if (this->get_timeout_status_(this->still_presence_millis_)) {
        this->still_target_binary_sensor_->publish_state(false);
      }
    }
  }
#endif
#ifdef USE_SENSOR
  // For presence timeout check
  if (target_count > 0) {
    this->presence_millis_ = millis();
  }
  if (moving_target_count > 0) {
    this->moving_presence_millis_ = millis();
  }
  if (still_target_count > 0) {
    this->still_presence_millis_ = millis();
  }
#endif
}

#ifdef USE_SENSOR
void LD6001Component::set_move_x_sensor(uint8_t target, sensor::Sensor *s) { this->move_x_sensors_[target] = s; }
void LD6001Component::set_move_y_sensor(uint8_t target, sensor::Sensor *s) { this->move_y_sensors_[target] = s; }
void LD6001Component::set_move_angle_sensor(uint8_t target, sensor::Sensor *s) {
  this->move_angle_sensors_[target] = s;
}
void LD6001Component::set_move_distance_sensor(uint8_t target, sensor::Sensor *s) {
  this->move_distance_sensors_[target] = s;
}
void LD6001Component::set_zone_target_count_sensor(uint8_t zone, sensor::Sensor *s) {
  this->zone_target_count_sensors_[zone] = s;
}
#endif

}  // namespace ld6001
}  // namespace esphome
