#pragma once

#include <algorithm>
#include <array>
#include <string>
#include <vector>
#include <stdint.h>
#include <cstddef>
#include <cstring>

namespace esphome::ld6001a {

enum class MatchResult { INVALID, PARTIAL, COMPLETE };

union FloatBytes {
  float f;
  std::array<uint8_t, 4> bytes;
};

union Uint32Bytes {
  uint32_t u;
  std::array<uint8_t, 4> bytes;
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

struct ReadParamsResponse {
  std::string softwareVersion;
  float range_res;
  float vel_res;
  int time;
  int prog;
  int range;
  int range_sensitivity;
  int heart_beat_interval;
  int protocol_mode;
  int detection_height;
  int x_nega;
  int x_posi;
  int y_nega;
  int y_posi;
  int moving_target_disappearance_time;
  int static_target_disappearance_time;
  int target_exit_time;
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
  virtual void on_detailed_radar_response(const std::vector<Person> people) {};
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
    push_data(fb.bytes);
  }

  void push_uint32(uint32_t value) {
    Uint32Bytes u32 = {value};
    push_data(u32.bytes);
  }

  void try_parse_frame_() {
    switch (state_) {
      case ParseState::READING_HEADER:
      case ParseState::IDLE:
        handle_idle_state();
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
    bool invalid = false;
    bool partial = false;
    bool complete = false;

    do {
      MatchResult ok_result = match_at_ok_();
      MatchResult simple_result = match_binary_type1_();
      MatchResult detailed_result = match_binary_type2_();

      invalid = ok_result == MatchResult::INVALID && simple_result == MatchResult::INVALID &&
                detailed_result == MatchResult::INVALID;

      partial = ok_result == MatchResult::PARTIAL || simple_result == MatchResult::PARTIAL ||
                detailed_result == MatchResult::PARTIAL;

      if (ok_result == MatchResult::COMPLETE) {
        state_ = ParseState::COMPLETE;
        process_at_ok_response();
        return;
      } else if (simple_result == MatchResult::COMPLETE) {
        state_ = ParseState::READING_BODY;
        auto frame_type = buffer_[2];
        body_len_ = buffer_[2];  // Body length is the 3rd byte in type 1
      } else if (detailed_result == MatchResult::COMPLETE) {
        state_ = ParseState::READING_BODY;
        auto length = read_uint32(&buffer_[8]);
        body_len_ = length + 1;  // Body length is the 9th byte in type 2
      } else if (partial) {
        this->state_ = ParseState::READING_HEADER;
      } else if (invalid) {
        this->state_ = ParseState::INVALID;
        drain_one_byte();
      }
    } while ((buffer_.size() > 0) && invalid);
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
    // drain_one_byte();  // Remove the invalid byte from the buffer
    state_ = ParseState::IDLE;  // Reset state after invalid frame
    handle_idle_state();        // Try to find the next valid frame
  }

  MatchResult match_at_ok_() {
    auto buffer_size = buffer_.size();
    std::string token = "AT+OK\n";

    if (buffer_size < 6) {
      return std::equal(buffer_.begin(), buffer_.begin() + buffer_size, token.substr(0, buffer_size).c_str())
                 ? MatchResult::PARTIAL
                 : MatchResult::INVALID;
    }

    return std::equal(buffer_.begin(), buffer_.begin() + 6, "AT+OK\n") ? MatchResult::COMPLETE : MatchResult::INVALID;
  }

  MatchResult match_binary_type1_() {
    auto buffer_size = buffer_.size();
    auto match_size = std::min<size_t>(2, buffer_size);
    static const std::array<uint8_t, 2> header = {0x55, 0xAA};

    return std::equal(header.begin(), header.begin() + match_size, buffer_.begin())
               ? (buffer_size < 4 ? MatchResult::PARTIAL : MatchResult::COMPLETE)
               : MatchResult::INVALID;
  }

  MatchResult match_binary_type2_() {
    auto buffer_size = buffer_.size();
    auto match_size = std::min<size_t>(8, buffer_size);
    static const std::array<uint8_t, 8> header = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

    return std::equal(header.begin(), header.begin() + match_size, buffer_.begin())
               ? (buffer_size < 9 ? MatchResult::PARTIAL : MatchResult::COMPLETE)
               : MatchResult::INVALID;
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
    if (match_at_ok_() == MatchResult::COMPLETE) {
      // Handle AT+OK frame response
      process_at_ok_response();
    } else if (match_binary_type1_() == MatchResult::COMPLETE) {
      // Handle type 1 frame (0x55 0xAA)
      process_binary_type1_response();
    } else if (match_binary_type2_() == MatchResult::COMPLETE) {
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
