#include "frame_iterator.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace ld6001 {
FrameIterator::FrameIterator(uart::UARTDevice &stream, FrameHandler &handler) : stream_(&stream), handler_(&handler) {
  current_.reserve(256);
  buffer_.reserve(256);
}


// Call this method to push data into the parser
void FrameIterator::push_data(const uint8_t byte) {
  buffer_.push_back(byte);  // Add byte to the buffer
  try_parse_frame_();       // Try to parse frame after every new byte
}

bool FrameIterator::try_parse_frame_() {
  // Skip until we have a frame start byte
  drain_until_frame_start_();

  // As long as we have a buffer of at least the header size, we can try to process it.
  while (buffer_.size() >= HEADER_SIZE) {
    uint8_t body_len = buffer_[2];
    size_t total_len = HEADER_SIZE + body_len;

    if (buffer_.size() < total_len)
      break;

    if (buffer_[total_len - 1] != FRAME_END) {
      drain_until_frame_start_();
      continue;
    }

    auto checksum = buffer_[total_len - 2];
    auto expected_checksum = get_iterator_checksum(buffer_.begin(), buffer_.begin() + total_len - 2);

    if (checksum != expected_checksum) {
      ESP_LOGW("ld6001", "Checksum mismatch: expected %02X, got %02X", expected_checksum, checksum);
      ESP_LOGW("ld6001", "%s", format_hex_pretty(buffer_).c_str());

      drain_until_frame_start_();
      continue;
    }

    current_.assign(buffer_.begin(), buffer_.begin() + total_len);
    buffer_.erase(buffer_.begin(), buffer_.begin() + total_len);

    this->process_frame_(current_);
  }
}

void FrameIterator::process_frame_(const std::vector<uint8_t> &frame) {
  uint8_t msg_type = frame[1];

  switch (msg_type) {
    case 0x11:
      this->handler_->on_status_response(StatusResponse::create(frame));
      break;
    case 0x62:
      this->handler_->on_radar_response(RadarResponse::create(frame));
      break;
    default:
      ESP_LOGW(TAG, "Unknown message type: 0x%02X", msg_type);
      break;
  }
}

const FrameIterator::value_type &FrameIterator::value() const { return current_; }

void FrameIterator::drain_until_frame_start_() {
  while (!buffer_.empty() && buffer_[0] != FRAME_START) {
    buffer_.erase(buffer_.begin());
  }
}

uint8_t FrameIterator::get_iterator_checksum(std::vector<uint8_t>::iterator begin, std::vector<uint8_t>::iterator end) {
  uint8_t sum = 0;

  for (auto it = begin; it != end; it++) {
    sum += *it;
  }

  return sum;
}
}  // namespace ld6001
}  // namespace esphome
