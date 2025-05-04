#pragma once

#include <vector>
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace ld6001 {
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

class FrameIterator {
 public:
  using value_type = std::vector<uint8_t>;

  explicit FrameIterator(uart::UARTDevice &stream);

  // Fetch next frame if available
  bool next();

  const value_type &value() const;

 protected:
  void drain_until_frame_start_();
  static uint8_t get_iterator_checksum(std::vector<uint8_t>::iterator begin, std::vector<uint8_t>::iterator end);

 private:
  uart::UARTDevice *stream_;
  std::vector<uint8_t> buffer_;
  value_type current_;

  static constexpr uint8_t FRAME_START = 0x4D;
  static constexpr uint8_t FRAME_END = 0x4A;
  static constexpr size_t HEADER_SIZE = 6;
};

}  // namespace ld6001
}  // namespace esphome