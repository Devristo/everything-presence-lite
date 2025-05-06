#include "ld6001a.h"
#include <utility>
#include "esphome/core/component.h"

namespace esphome
{
  namespace ld6001a
  {
    static const char *const TAG = "ld6001a";

    LD6001AComponent::LD6001AComponent(): Component() {}

    void LD6001AComponent::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up HLK-LD6001A...");
    }

    void LD6001AComponent::dump_config()
    {
      ESP_LOGCONFIG(TAG, "HLK-LD6001A Human motion tracking radar module:");
    }

    void LD6001AComponent::loop()
    {
      uint8_t byte;

      // Read data from the UART and push it to the frame parser
      while(this->available()) {
        if (this->read_byte(&byte)) {
          this->frame_parser_.push_data(byte);
        }
      }
    }

    void LD6001AComponent::start() {
      ESP_LOGD(TAG, "Starting HLK-LD6001A...");
      
      this->command_queue_.enqueue(Command("AT+START\n", [this](const std::string &response) {
        ESP_LOGD(TAG, "HLK-LD6001A started");
      }));
    }

    void LD6001AComponent::stop() {
      ESP_LOGD(TAG, "Stopping HLK-LD6001A...");
      this->command_queue_.enqueue(Command("AT+STOP\n", [this](const std::string &response) {
        ESP_LOGD(TAG, "HLK-LD6001A stopped");
      }));
    }

    void LD6001AComponent::reset() {
      ESP_LOGD(TAG, "Resetting HLK-LD6001A...");
      this->command_queue_.enqueue(Command("AT+RESET\n", [this](const std::string &response) {
        ESP_LOGD(TAG, "HLK-LD6001A reset");
      }));
    }

    void LD6001AComponent::config_factory_settings() {
      ESP_LOGD(TAG, "Configuring factory settings...");
      this->command_queue_.enqueue(Command("AT+RESTORE\n", [this](const std::string &response) {
        ESP_LOGD(TAG, "Factory settings configured");
      }));
    }

    void LD6001AComponent::set_protocol_mode(ProtocolMode mode) {
      ESP_LOGD(TAG, "Setting protocol mode to %d", mode);
      this->command_queue_.enqueue(Command(str_sprintf("AT+DEBUG=%d\n", mode).c_str(), [this](const std::string &response) {
        ESP_LOGD(TAG, "Protocol mode set to %d", mode);
      }));
    }

    void LD6001AComponent::config_distance_sensitivity(uint8_t sensitivity) {
      assert(sensitivity >= 1 && sensitivity <= 9);

      ESP_LOGD(TAG, "Configuring distance sensitivity: %d", sensitivity);

      this->command_queue_.enqueue(Command(str_sprintf("AT+DPKTH=%d\n", sensitivity).c_str(), [this](const std::string &response) {
        ESP_LOGD(TAG, "Distance sensitivity configured: %d", sensitivity);
      }));
    }

    void LD6001AComponent::config_heartbeat_interval(uint16_t interval_s) {
      assert(interval_s >= 10 && interval_s <= 999);

      ESP_LOGD(TAG, "Configuring heartbeat interval: %d s", interval_s);

      this->command_queue_.enqueue(Command(str_sprintf("AT+HEATIME=%d\n", interval_s).c_str(), [this](const std::string &response) {
        ESP_LOGD(TAG, "Heart beat interval configured: %d s", interval_s);
      }));
    }

    void LD6001AComponent::config_vertical_distance(int distance_cm) {
      assert(distance_cm >= 50 && distance_cm <= 500);

      ESP_LOGD(TAG, "Configuring vertical distance: %d cm", distance_cm);

      this->command_queue_.enqueue(Command(str_sprintf("AT+HEIGHTD=%d\n", distance_cm).c_str(), [this](const std::string &response) {
        ESP_LOGD(TAG, "Vertical distance configured: %d cm", distance_cm);
      }));
    }

    void LD6001AComponent::config_ground_radius(uint16_t radius_cm) {
      assert(radius_cm >= 100 && radius_cm <= 500);

      ESP_LOGD(TAG, "Configuring ground radius: %d cm", radius_cm);

      this->command_queue_.enqueue(Command(str_sprintf("AT+RANGE=%d\n", radius_cm).c_str(), [this](const std::string &response) {
        ESP_LOGD(TAG, "Ground radius configured: %d cm", radius_cm);
      }));
    }

    void LD6001AComponent::config_x_range(int x1, int x2) {
      assert(x1 >= -500 && x1 <= -20);
      assert(x2 >= 20 && x2 <= 500);
      assert(x1 < x2);

      ESP_LOGD(TAG, "Configuring X range: (%d cm, %d cm)", x1, x2);

      this->command_queue_.enqueue(Command(str_sprintf("AT+XNega=%d\n", x1).c_str(), [this](const std::string &response) {
        ESP_LOGD(TAG, "XNega configured: %d cm", x1);
      }));

      this->command_queue_.enqueue(Command(str_sprintf("AT+XPosi=%d\n", x2).c_str(), [this](const std::string &response) {
        ESP_LOGD(TAG, "XPosi configured: %d cm", x2);
      }));
    }

    void LD6001AComponent::config_y_range(int y1, int y2){
      assert(y1 >= -500 && y1 <= -20);
      assert(y2 >= 20 && y2 <= 500);
      assert(y1 < y2);

      ESP_LOGD(TAG, "Configuring Y range: (%d cm, %d cm)", y1, y2);

      this->command_queue_.enqueue(Command(str_sprintf("AT+XNega=%d\n", y1).c_str(), [this](const std::string &response) {
        ESP_LOGD(TAG, "YNega configured: %d cm", y1);
      }));

      this->command_queue_.enqueue(Command(str_sprintf("AT+XPosi=%d\n", y2).c_str(), [this](const std::string &response) {
        ESP_LOGD(TAG, "YPosi configured: %d cm", y2);
      }));
    }

    void LD6001AComponent::config_moving_target_disappearance_time(uint16_t time_ms) {
      assert(time_ms % 100 == 0);
      assert(time_ms >= 500 && time_ms <= 1000 * 100);

      ESP_LOGD(TAG, "Configuring moving target disappearance time: %d ms", time_ms);

      this->command_queue_.enqueue(Command(str_sprintf("AT+Moving=%d\n", time_ms / 100).c_str(), [this](const std::string &response) {
        ESP_LOGD(TAG, "Moving target disappearance time configured: %d ms", time_ms);
      }));
    }

    void LD6001AComponent::config_static_target_disappearance_time(uint16_t time_ms) {
      assert(time_ms % 100 == 0);
      assert(time_ms >= 500 && time_ms <= 1000 * 100);

      ESP_LOGD(TAG, "Configuring static target disappearance time: %d ms", time_ms);
      this->command_queue_.enqueue(Command(str_sprintf("AT+Static=%d\n", time_ms / 100).c_str(), [this](const std::string &response) {
        ESP_LOGD(TAG, "Static target disappearance time configured: %d ms", time_ms);
      }));
    }

    void LD6001AComponent::config_exit_boundary_time(uint16_t time_ms) {
      assert(time_ms % 100 == 0);
      assert(time_ms >= 500 && time_ms <= 1000 * 100);

      ESP_LOGD(TAG, "Configuring exit boundary time: %d ms", time);
      this->command_queue_.enqueue(Command(str_sprintf("AT+Exit=%d\n", time_ms / 100).c_str(), [this](const std::string &response) {
        ESP_LOGD(TAG, "Exit boundary time configured: %d ms", time_ms);
      }));
    }

    void LD6001AComponent::on_ack_response() {
      this->command_queue_.handleResponse("");
    };

    void LD6001AComponent::on_simple_radar_response(const uint8_t people_counted) {
      this->people_counted_ = people_counted;
    }

    void LD6001AComponent::on_detailed_radar_response(const std::vector<Person> people) {
      this->detailed_people_response_ = people;
      this->people_counted_ = people.size();
    }

    void LD6001AComponent::on_invalid_frame() {

    }
  } // namespace ld6001a
} // namespace esphome
