#pragma once

#include <array>
#include <vector>
#include <stdint.h>
#include <cstddef>
#include <cstring>

namespace esphome::ld6001a {

union FloatBytes {
  float f;
  uint8_t bytes[4];
};

union Uint32Bytes {
  uint32_t u;
  uint8_t bytes[4];
};

struct Person {
  uint32_t id;
  float x;
  float y;
  float z;
  float vx;
  float vy;
  float vz;
};

float read_float(const uint8_t *ptr) {
  float value;
  std::memcpy(&value, ptr, sizeof(float));
  return value;
}

uint32_t read_uint32(const uint8_t *ptr) {
  uint32_t value;
  std::memcpy(&value, ptr, sizeof(uint32_t));
  return value;
}

enum class ParseState { IDLE, READING_HEADER, READING_BODY, VALIDATING, COMPLETE, INVALID };

class FrameHandler {
 public:
  virtual void on_ack_response() {};
  virtual void on_simple_radar_response(const uint8_t people_counted) {};
  virtual void on_detailed_radar_response(const std::vector<Person> people_counted) {};
  virtual void on_invalid_frame() {};
  virtual ~FrameHandler() = default;
};

class FrameParser {
 public:
  FrameParser(FrameHandler &handler) : state_(ParseState::IDLE), frame_handler_(handler) {}
  ParseState state_;

  // Call this method to push data into the parser
  void push_data(const uint8_t byte) {
    buffer_.push_back(byte);  // Add byte to the buffer
    try_parse_frame_();       // Try to parse frame after every new byte
  }

  template<size_t N> void push_data(const std::array<uint8_t, N> &data) {
    for (const auto &byte : data) {
      push_data(byte);
    }
  }

  void push_float(float value) {
    FloatBytes fb = {value};
    for (const auto &byte : fb.bytes) {
      push_data(byte);
    }
  }

  void try_parse_frame_() {
    switch (state_) {
      case ParseState::IDLE:
        handle_idle_state();
        break;
      case ParseState::READING_HEADER:
        handle_reading_header_state();
        break;
      case ParseState::READING_BODY:
        handle_reading_body_state();
        break;
      case ParseState::VALIDATING:
        handle_validating_state();
        break;
      case ParseState::COMPLETE:
        handle_complete_state();
        break;
      case ParseState::INVALID:
        handle_invalid_state();
        break;
    }
  }

 private:
  std::vector<uint8_t> buffer_;         // Input buffer for incoming bytes
  std::vector<uint8_t> current_frame_;  // The current complete frame
  std::size_t body_len_ = 0;            // Length of the body for frames that include it
  FrameHandler &frame_handler_;         // Reference to the frame handler

  void handle_idle_state() {
    if (match_at_ok_()) {
      state_ = ParseState::COMPLETE;
    } else if (match_binary_type1_()) {
      state_ = ParseState::READING_BODY;
      auto frame_type = buffer_[2];
      body_len_ = buffer_[2];  // Body length is the 3rd byte in type 1
    } else if (match_binary_type2_()) {
      state_ = ParseState::READING_BODY;
      auto length = read_uint32(&buffer_[8]);
      body_len_ = length + 1;  // Body length is the 9th byte in type 2
    }
  }

  void handle_reading_header_state() {
    // Frame headers are already matched, now we start reading the body
    state_ = ParseState::READING_BODY;
  }

  void handle_reading_body_state() {
    if (buffer_.size() >= body_len_) {
      // Enough data in buffer for the full frame
      state_ = ParseState::VALIDATING;
      handle_validating_state();
    }
  }

  void handle_validating_state() {
    if (validate_frame()) {
      state_ = ParseState::COMPLETE;
      this->handle_complete_state();
    } else {
      state_ = ParseState::INVALID;
    }
  }

  void handle_complete_state() {
    // Process the complete frame
    process_frame();
    state_ = ParseState::IDLE;  // Reset state to idle after processing a frame
  }

  void handle_invalid_state() {
    // Handle invalid frame (log error, resync, etc.)
    state_ = ParseState::IDLE;  // Reset state after invalid frame
  }

  bool match_at_ok_() {
    if (buffer_.size() < 5)
      return false;  // AT+OK\r\n needs at least 5 bytes
    return std::equal(buffer_.begin(), buffer_.begin() + 5, "AT+OK\r\n");
  }

  bool match_binary_type1_() {
    if (buffer_.size() < 4)
      return false;  // Minimal 3-byte header for 0x55 0xAA
    return buffer_[0] == 0x55 && buffer_[1] == 0xAA;
  }

  bool match_binary_type2_() {
    if (buffer_.size() < 9)
      return false;  // Minimal 8-byte header for 0x01 0x02 ...
    static const std::array<uint8_t, 8> header = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    return std::equal(header.begin(), header.end(), buffer_.begin());
  }

  void drain_one_byte() {
    if (!buffer_.empty()) {
      buffer_.erase(buffer_.begin());
    }
  }

  bool validate_frame() {
    auto expected_checksum = buffer_[buffer_.size() - 1];

    if (buffer_[0] == 0x55 && buffer_[1] == 0xAA) {
      uint8_t calculated_checksum = 0;
      for (size_t i = 2; i < buffer_.size() - 1; ++i) {
        calculated_checksum ^= buffer_[i];
      }

      return expected_checksum == calculated_checksum;
    } else {
      uint8_t calculted_checksum = 0;
      for (size_t i = 12; i < 16; ++i) {
        calculted_checksum ^= buffer_[i];
      }
      for (size_t i = 32; i < buffer_.size() - 1; ++i) {
        calculted_checksum ^= buffer_[i];
      }

      return expected_checksum == calculted_checksum;
    }
  }

  void process_frame() {
    // Process the frame (e.g., call specific handlers)
    if (match_at_ok_()) {
      // Handle AT+OK frame response
      process_at_ok_response();
    } else if (match_binary_type1_()) {
      // Handle type 1 frame (0x55 0xAA)
      process_binary_type1_response();
    } else if (match_binary_type2_()) {
      // Handle type 2 frame (0x01 0x02 0x03 0x04 ...)
      process_binary_type2_response();
    }
  }

  void process_at_ok_response() { this->frame_handler_.on_ack_response(); }

  void process_binary_type1_response() {
    this->frame_handler_.on_simple_radar_response(buffer_[8]);  // Assuming 4th byte is people count
  }

  void process_binary_type2_response() {
    Uint32Bytes u32 = {.bytes{buffer_[28], buffer_[29], buffer_[30], buffer_[31]}};
    auto people_count = u32.u / 32;  // Assuming 4th byte is people count

    std::vector<Person> people;
    people.reserve(u32.u / 32);

    for (size_t i = 0; i < people_count; ++i) {
      auto offset = i * 32 + 32;  // Start reading from the 33rd byte
      Person person = {
          .id{read_uint32(&buffer_[offset + 4])},
          .x{read_float(&buffer_[offset + 8])},
          .y{read_float(&buffer_[offset + 12])},
          .z{read_float(&buffer_[offset + 16])},
          .vx{read_float(&buffer_[offset + 20])},
          .vy{read_float(&buffer_[offset + 24])},
          .vz{read_float(&buffer_[offset + 28])},
      };

      people.push_back(person);
    }

    this->frame_handler_.on_detailed_radar_response(people);  // Assuming 4th byte is people count
  }
};

}  // namespace esphome::ld6001a
