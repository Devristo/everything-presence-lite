#pragma once

#include <vector>
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace ld6001 {
static const uint8_t MAX_TARGETS = 10;

struct Target {
  uint8_t id;
  uint8_t pitch_angle;
  uint8_t horizontal_angle;
  uint16_t distance;
  int16_t x;
  int16_t y;
};

class FrameHandler {
 public:
  virtual void on_radar_response(const RadarResponse &response) {};
  virtual void on_status_response(const StatusResponse &response) {};
  virtual ~FrameHandler() = default;
};

struct StatusResponse {
  uint8_t software_version_minor;
  uint8_t software_version_major;

  uint8_t hardware_version_minor;
  uint8_t hardware_version_major;
  bool initialized;

  static StatusResponse create(const std::vector<uint8_t> &buffer) {
    return StatusResponse{.software_version_minor = buffer[4],
                          .software_version_major = buffer[5],
                          .hardware_version_minor = buffer[6],
                          .hardware_version_major = buffer[7],
                          .initialized = buffer[9] == 0x00};
  }
};

struct RadarResponse {
  uint8_t targets;
  uint8_t fault_status;
  Target people[MAX_TARGETS];

  static RadarResponse create(const std::vector<uint8_t> &buffer) {
    RadarResponse response;
    response.fault_status = buffer[4];
    response.targets = buffer[5];

    for (int target = 0; target < MAX_TARGETS; target++) {
      size_t offset = 12 + target * 8;
      uint8_t id = 0;
      uint16_t distance = 0;
      uint8_t pitch_angle = 0;
      uint8_t horizontal_angle = 0;
      int16_t coord_x = 0;
      int16_t coord_y = 0;

      if (target < response.targets) {
        id = buffer[offset];
        distance = buffer[offset + 1] * 10;
        pitch_angle = buffer[offset + 2];
        horizontal_angle = buffer[offset + 3];
        coord_x = buffer[offset + 6] * 10;
        coord_y = buffer[offset + 7] * 10;
      }

      response.people[target] = Target{.id = id,
             .pitch_angle = pitch_angle,
             .horizontal_angle = horizontal_angle,
             .distance = distance,
             .x = coord_x,
             .y = coord_y};
    }

    return response;
  }
};

class FrameIterator {
 public:
  using value_type = std::vector<uint8_t>;

  explicit FrameIterator(esphome::ld6001::FrameHandler &handler);

  void push_data(const uint8_t byte);

  void push_byte(const uint8_t byte);

  const value_type &value() const;

 protected:
  void drain_until_frame_start_();
  void process_frame_(const std::vector<uint8_t> &buffer);
  bool try_parse_frame_();
  static uint8_t get_iterator_checksum(std::vector<uint8_t>::iterator begin, std::vector<uint8_t>::iterator end);

 private:
  std::vector<uint8_t> buffer_;
  value_type current_;
  FrameHandler *handler_;

  static constexpr uint8_t FRAME_START = 0x4D;
  static constexpr uint8_t FRAME_END = 0x4A;
  static constexpr size_t HEADER_SIZE = 6;
};

}  // namespace ld6001
}  // namespace esphome