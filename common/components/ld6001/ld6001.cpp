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

static const std::array<uint8_t, 6> CMD_GET_VERSION = {0x44, 0x11, 0x00, 0x00, 0x55, 0x4B};
static const std::array<uint8_t, 14> CMD_RADAR_REQUEST_PRECISE = {0x44, 0x62, 0x08, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBE, 0x4B};

LD6001Component::LD6001Component() {}

void LD6001Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HLK-LD6001...");
  this->read_all_info();

  this->set_interval("update", 500, [this]() { this->send_radar_request(); });
}

void LD6001Component::dump_config() {
  ESP_LOGCONFIG(TAG, "HLK-LD6001 Human motion tracking radar module:");
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "TargetBinarySensor", this->target_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "MovingTargetBinarySensor", this->moving_target_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "StillTargetBinarySensor", this->still_target_binary_sensor_);
#endif
#ifdef USE_SENSOR
  LOG_SENSOR("  ", "TargetCountSensor", this->target_count_sensor_);
  for (sensor::Sensor *s : this->move_x_sensors_) {
    LOG_SENSOR("  ", "NthTargetXSensor", s);
  }
  for (sensor::Sensor *s : this->move_y_sensors_) {
    LOG_SENSOR("  ", "NthTargetYSensor", s);
  }
  for (sensor::Sensor *s : this->move_pitch_angle_sensors_) {
    LOG_SENSOR("  ", "NthTargetPitchAngleSensor", s);
  }
  for (sensor::Sensor *s : this->move_horizontal_angle_sensors_) {
    LOG_SENSOR("  ", "NthTargetHorizontalAngleSensor", s);
  }
  for (sensor::Sensor *s : this->move_distance_sensors_) {
    LOG_SENSOR("  ", "NthTargetDistanceSensor", s);
  }
  for (sensor::Sensor *s : this->zone_target_count_sensors_) {
    LOG_SENSOR("  ", "NthZoneTargetCountSensor", s);
  }
#endif

  ESP_LOGCONFIG(TAG, "  Throttle : %ums", this->throttle_);
  ESP_LOGCONFIG(TAG, "  MAC Address : %s", const_cast<char *>(this->mac_.c_str()));
  ESP_LOGCONFIG(TAG, "  Firmware version : %s", const_cast<char *>(this->version_.c_str()));
}

void LD6001Component::loop() {
  size_t available_bytes = 0;

  while (available_bytes = this->available()) {
    this->readline_(read(), this->buffer_data_, MAX_LINE_LENGTH);
  }
}

void LD6001Component::readline_(int readch, uint8_t *buffer, uint16_t len) {
  ESP_LOGV(TAG, "Reading data");

  // All messages from the device should start with 0x4D
  if (readch != 0x4D) {
    return;
  }

  uint8_t message_type = this->read();

  if (message_type == 0x11) {
    optional<std::array<uint8_t, 12>> version_bytes = this->read_array<12>();

    if (!version_bytes.has_value()) {
      ESP_LOGW(TAG, "Could not read version bytes");
      return;
    }

    ESP_LOGV(TAG, "Handle module version information");
    uint8_t software_version_minor = version_bytes.value()[2];
    uint8_t software_version_major = version_bytes.value()[3];

    uint8_t hardware_version_minor = version_bytes.value()[4];
    uint8_t hardware_version_major = version_bytes.value()[5];

    std::string version = str_sprintf("HW v%d.%02d / SW v%d.%02d", hardware_version_major, hardware_version_minor, software_version_major, software_version_minor);

    bool is_ready = buffer[this->buffer_pos_ - 13 + 9] == 0x00;

    #ifdef USE_TEXT_SENSOR
        if (this->version_text_sensor_ != nullptr) {
          this->version_text_sensor_->publish_state(version);
        }
    #endif

  } else if (message_type == 0x62) {
    optional<std::array<uint8_t, 10>> radar_header = this->read_array<10>();
    if (!radar_header.has_value()) {
      ESP_LOGW(TAG, "Could not read version bytes");
      return;
    }

    uint8_t fault_status = radar_header.value()[2];
    uint8_t targets = radar_header.value()[3];
    uint8_t length = targets * 8 + 2;

    auto buffer_ptr = std::make_unique<uint8_t[]>(length);

    if(!this->read_array(buffer_ptr.get(), length)) {
      ESP_LOGW(TAG, "Could not read radar data");
      return;
    }

     auto buffer = buffer_ptr.get();

     uint8_t checksum = buffer[length - 2];
     uint8_t final = buffer[length - 1];
     
     if (final != 0x4A){
       std::string error = str_sprintf("Final byte not 0x4A: 0x%02X", final);
       ESP_LOGW(TAG, error.c_str());
       return;
     }

     for (int target = 0; target < MAX_TARGETS; target++) {
       uint8_t id = 0;
       uint8_t distance = 0;
       uint8_t pitch_angle = 0;
       uint8_t horizontal_angle = 0;
       uint8_t coord_x = 0;
       uint8_t coord_y = 0;

       if (target < targets) {
        id = buffer[target * 8 + 0];
        distance = buffer[target * 8 + 1] * 10;
        pitch_angle = buffer[target * 8 + 2];
        horizontal_angle = buffer[target * 8 + 3];
        coord_x = buffer[target * 8 + 6] * 10 ;
        coord_y = buffer[target * 8 + 7] * 10 ;
       }

       if (this->move_x_sensors_[target] != nullptr) {
         this->move_x_sensors_[target]->publish_state(target < targets ? coord_x : NAN);
       }

       if (this->move_y_sensors_[target] != nullptr) {
         this->move_y_sensors_[target]->publish_state(target < targets ? coord_y : NAN);
       }
       if(this->move_distance_sensors_[target] != nullptr) {
         this->move_distance_sensors_[target]->publish_state(target < targets ? distance : NAN);
       }
       if(this->move_pitch_angle_sensors_[target] != nullptr) {
         this->move_pitch_angle_sensors_[target]->publish_state(target < targets ? pitch_angle : NAN);
       }
       if(this->move_horizontal_angle_sensors_[target] != nullptr) {
         this->move_horizontal_angle_sensors_[target]->publish_state(target < targets ? horizontal_angle : NAN);
       }
     }

    // Target Count
    if (this->target_count_sensor_ != nullptr) {
      this->target_count_sensor_->publish_state(targets);
    }
  }
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


#ifdef USE_SENSOR
void LD6001Component::set_move_x_sensor(uint8_t target, sensor::Sensor *s) { this->move_x_sensors_[target] = s; }
void LD6001Component::set_move_y_sensor(uint8_t target, sensor::Sensor *s) { this->move_y_sensors_[target] = s; }
void LD6001Component::set_move_pitch_angle_sensor(uint8_t target, sensor::Sensor *s) {
  this->move_pitch_angle_sensors_[target] = s;
}
void LD6001Component::set_move_horizontal_angle_sensor(uint8_t target, sensor::Sensor *s) {
  this->move_horizontal_angle_sensors_[target] = s;
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
