#include "ld6001.h"
#include <utility>
#include "esphome/components/mqtt/mqtt_client.h"
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#include "esphome/core/component.h"

#define highbyte(val) (uint8_t)((val) >> 8)
#define lowbyte(val) (uint8_t)((val) & 0xff)

namespace esphome
{
  namespace ld6001
  {
    template <std::size_t N>
    uint8_t get_checksum(const std::array<uint8_t, N> data) {
      uint8_t sum = 0;

      for (auto val : data) {
          sum += val;
      }

      return sum;
    }

    void announce_mqtt_target_enter(uint8_t target_id) {
      #ifdef USE_MQTT
      if (mqtt::global_mqtt_client == nullptr) {
        return;
      }

      mqtt::global_mqtt_client->publish_json(
        mqtt::global_mqtt_client->get_topic_prefix() + "/target_entered",
        [&](JsonObject root)
        {
          root["target_id"] = target_id;
        }
      );
      #endif
    }

    void announce_mqtt_target_left(uint8_t target_id, uint32_t dwell_time) {
      #ifdef USE_MQTT
      if (mqtt::global_mqtt_client == nullptr) {
        return;
      }

      mqtt::global_mqtt_client->publish_json(
        mqtt::global_mqtt_client->get_topic_prefix() + "/target_left",
        [&](JsonObject root)
        {
          root["id"] = target_id;
          root["dwell_time"] = dwell_time;
        }
      );
      #endif
    }

    static const char *const TAG = "ld6001";

    static const std::array<uint8_t, 6> CMD_GET_VERSION = {0x44, 0x11, 0x00, 0x00, 0x55, 0x4B};
    static const std::array<uint8_t, 12> CMD_RADAR_REQUEST_NORMAL = {0x44, 0x62, 0x08, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    static const std::array<uint8_t, 12> CMD_RADAR_REQUEST_PRECISE = {0x44, 0x62, 0x08, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    LD6001Component::LD6001Component(): PollingComponent(500), frame_iter_(FrameIterator(*this)) {}

    void LD6001Component::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up HLK-LD6001...");
      this->read_all_info();
    }

    void LD6001Component::dump_config()
    {
      ESP_LOGCONFIG(TAG, "HLK-LD6001 Human motion tracking radar module:");
#ifdef USE_SENSOR
      LOG_SENSOR("  ", "TargetCountSensor", this->target_count_sensor_);
      for (sensor::Sensor *s : this->move_x_sensors_)
      {
        LOG_SENSOR("  ", "NthTargetXSensor", s);
      }
      for (sensor::Sensor *s : this->move_y_sensors_)
      {
        LOG_SENSOR("  ", "NthTargetYSensor", s);
      }
      for (sensor::Sensor *s : this->move_pitch_angle_sensors_)
      {
        LOG_SENSOR("  ", "NthTargetPitchAngleSensor", s);
      }
      for (sensor::Sensor *s : this->move_horizontal_angle_sensors_)
      {
        LOG_SENSOR("  ", "NthTargetHorizontalAngleSensor", s);
      }
      for (sensor::Sensor *s : this->move_distance_sensors_)
      {
        LOG_SENSOR("  ", "NthTargetDistanceSensor", s);
      }
      for (sensor::Sensor *s : this->zone_target_count_sensors_)
      {
        LOG_SENSOR("  ", "NthZoneTargetCountSensor", s);
      }
#endif

      ESP_LOGCONFIG(TAG, "  Throttle : %ums", this->throttle_);
      ESP_LOGCONFIG(TAG, "  MAC Address : %s", const_cast<char *>(this->mac_.c_str()));
      ESP_LOGCONFIG(TAG, "  Firmware version : %s", const_cast<char *>(this->version_.c_str()));
    }

    void LD6001Component::loop()
    {
      auto start = millis();
      while (frame_iter_.next()) {
        const auto& frame = frame_iter_.value();
        uint8_t msg_type = frame[1];
        switch (msg_type) {
          case 0x11:
            this->read_version_frame(frame.data(), frame.size());
            break;
          case 0x62:
            this->read_radar_frame(frame.data(), frame.size());
            break;
          default:
            ESP_LOGW(TAG, "Unknown message type: 0x%02X", msg_type);
            break;
        }
      }

      auto delta = millis() - start;
      if (delta > 5) {
        ESP_LOGW(TAG, "Frame processing took %d ms", delta);
      }
    }

    void LD6001Component::update() {
      this->send_radar_request();
      this->update_sensors();
    }

    void LD6001Component::update_sensors()
    {
      #ifdef USE_SENSOR
      /*
         Reduce data update rate to prevent home assistant database size grow fast
      */
      int32_t current_millis = millis();
      if (current_millis - last_periodic_millis_ < this->throttle_) {
        return;
      }

      last_periodic_millis_ = current_millis;

      
      while (!this->announce_entry.empty()) {
        auto target_id = this->announce_entry.back();
        this->announce_entry.pop_back();
        announce_mqtt_target_enter(target_id);
      }

      std::vector<std::tuple<uint8_t, uint32_t>> targets_left;
      while(!this->removed_targets.empty()) {
        auto key = this->removed_targets.back();
        this->removed_targets.pop_back();

        if (this->entry_times.find(key) == this->entry_times.end()) {
          ESP_LOGW(TAG, "Inconsistency detected: Target %d has a last_seen_time but no entry_time", key);
          continue; // Skip processing this target
        }

        uint32_t last_seen_time = this->last_seen_times[key];
        uint32_t first_seen = this->entry_times[key];
        uint32_t dwell_time = (last_seen_time - first_seen) / 1000;

        ESP_LOGW(TAG, "Target %d left, first seen at %d, dwell time is %d seconds", key, first_seen, dwell_time);
        announce_mqtt_target_left(key, dwell_time);
        
        this->last_seen_times.erase(key);
        this->entry_times.erase(key);
      }

      // Target Count
      if (this->target_count_sensor_ != nullptr && this->target_info_.targets != this->target_count_sensor_->state)
      {
        this->target_count_sensor_->publish_state(this->target_info_.targets);
      }

      if (mqtt::global_mqtt_client != nullptr)
      {

        mqtt::global_mqtt_client->publish_json(
            mqtt::global_mqtt_client->get_topic_prefix() + "/targets",
            [&](JsonObject root)
            {
              root["targets"] = this->target_info_.targets;
              auto target_data = root.createNestedArray("target_data");

              for (size_t i = 0; i < this->target_info_.targets; i++)
              {
                auto data = target_data.createNestedObject();
                data["id"] = this->target_info_.target_data[i].id;
                data["x"] = this->target_info_.target_data[i].x;
                data["y"] = this->target_info_.target_data[i].y;
                data["distance"] = this->target_info_.target_data[i].distance;
                data["pitch_angle"] = this->target_info_.target_data[i].pitch_angle;
                data["horizontal_angle"] = this->target_info_.target_data[i].horizontal_angle;
              }
            });
      }

      auto maybe_publish = [](auto* sensor, float new_value) {
        if (sensor == nullptr) return;
        float old_value = sensor->state;
        if ((std::isnan(old_value) && !std::isnan(new_value)) ||
            (!std::isnan(old_value) && (old_value != new_value))) {
          sensor->publish_state(new_value);
        }
      };

      uint8_t targets = this->target_info_.targets;
      for (size_t i = 0; i < MAX_TARGETS; i++)
      {
        Target target = this->target_info_.target_data[i];
        auto coord_x = i < targets ? target.x : NAN;
        auto coord_y = i < targets ? target.y : NAN;
        auto distance = i < targets ? target.distance : NAN;
        auto pitch_angle = i < targets ? target.pitch_angle : NAN;
        auto horizontal_angle = i < targets ? target.horizontal_angle : NAN;

        maybe_publish(this->move_x_sensors_[i], coord_x);
        maybe_publish(this->move_y_sensors_[i], coord_y);
        maybe_publish(this->move_distance_sensors_[i], distance);
        maybe_publish(this->move_pitch_angle_sensors_[i], pitch_angle);
        maybe_publish(this->move_horizontal_angle_sensors_[i], horizontal_angle);
      }
      #endif
    }

    void LD6001Component::read_version_frame(const uint8_t *buffer, const size_t length)
    {

      ESP_LOGV(TAG, "Handle module version information");
      uint8_t software_version_minor = buffer[4];
      uint8_t software_version_major = buffer[5];

      uint8_t hardware_version_minor = buffer[6];
      uint8_t hardware_version_major = buffer[7];

      std::string version = str_sprintf("HW v%d.%02d / SW v%d.%02d", hardware_version_major, hardware_version_minor, software_version_major, software_version_minor);

#ifdef USE_TEXT_SENSOR
      if (this->version_text_sensor_ != nullptr)
      {
        this->version_text_sensor_->publish_state(version);
      }
#endif
    }

    void LD6001Component::read_radar_frame(const uint8_t *buffer, const size_t length)
    {
      uint8_t fault_status = buffer[4];
      uint8_t targets = buffer[5];

      uint8_t checksum = buffer[11 + targets * 8 + 1];
      uint8_t final = buffer[11 + targets * 8 + 2];

      if (final != 0x4A)
      {
        std::string error = str_sprintf("Final byte not 0x4A: 0x%02X", final);
        ESP_LOGW(TAG, error.c_str());
        return;
      }


      std::unordered_set<uint8_t> missing_targets;
      for (int target = 0; target < this->target_info_.targets; target++) {
        missing_targets.insert(this->target_info_.target_data[target].id);
      }

      this->target_info_.targets = targets;
      for (int target = 0; target < MAX_TARGETS; target++)
      {
        size_t offset = 12 + target * 8;
        uint8_t id = 0;
        uint8_t distance = 0;
        uint8_t pitch_angle = 0;
        uint8_t horizontal_angle = 0;
        int8_t coord_x = 0;
        int8_t coord_y = 0;

        if (target < targets)
        {
          id = buffer[offset];
          distance = buffer[offset + 1] * 10;
          pitch_angle = buffer[offset + 2];
          horizontal_angle = buffer[offset + 3];
          coord_x = buffer[offset + 6] * 10;
          coord_y = buffer[offset + 7] * 10;
          this->update_last_seen(id);
          missing_targets.erase(id);
        }

        this->target_info_.target_data[target] = Target{
            .id = id,
            .pitch_angle = pitch_angle,
            .horizontal_angle = horizontal_angle,
            .distance = distance,
            .x = coord_x,
            .y = coord_y};
      }

      for (const auto id: missing_targets) {
        this->removed_targets.push_back(id);
      }
    }

    void LD6001Component::update_last_seen(uint8_t target_id) {
      uint32_t now = millis();
      
      if (this->entry_times.find(target_id) == this->entry_times.end()) {
        ESP_LOGW(TAG, "Target %d entered view", target_id);
        this->announce_entry.push_back(target_id);
        this->entry_times[target_id] = now;
      }

      this->last_seen_times[target_id] = now;
    }

    // Read all info from LD6001 buffer
    void LD6001Component::read_all_info()
    {
      this->get_version_();
    }

    // Get LD6001 firmware version
    void LD6001Component::get_version_()
    {
      ESP_LOGV(TAG, "Sending get version request");
      this->write_array(CMD_GET_VERSION);
    }

    void LD6001Component::send_radar_request()
    {
      ESP_LOGV(TAG, "Sending precise radar request");

      uint8_t checksum = get_checksum(CMD_RADAR_REQUEST_NORMAL);

      this->write_array(CMD_RADAR_REQUEST_NORMAL);
      this->write_byte(checksum);
      this->write_byte(0x4B);
    
    }

#ifdef USE_SENSOR
    void LD6001Component::set_move_x_sensor(uint8_t target, sensor::Sensor *s) { this->move_x_sensors_[target] = s; }
    void LD6001Component::set_move_y_sensor(uint8_t target, sensor::Sensor *s) { this->move_y_sensors_[target] = s; }
    void LD6001Component::set_move_pitch_angle_sensor(uint8_t target, sensor::Sensor *s)
    {
      this->move_pitch_angle_sensors_[target] = s;
    }
    void LD6001Component::set_move_horizontal_angle_sensor(uint8_t target, sensor::Sensor *s)
    {
      this->move_horizontal_angle_sensors_[target] = s;
    }
    void LD6001Component::set_move_distance_sensor(uint8_t target, sensor::Sensor *s)
    {
      this->move_distance_sensors_[target] = s;
    }
    void LD6001Component::set_zone_target_count_sensor(uint8_t zone, sensor::Sensor *s)
    {
      this->zone_target_count_sensors_[zone] = s;
    }
#endif

  } // namespace ld6001
} // namespace esphome
