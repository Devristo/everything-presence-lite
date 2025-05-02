#pragma once

#include <vector>
#include "esphome/components/uart/uart.h"

namespace esphome
{
  namespace ld6001
  {

    class FrameIterator
    {
      public:
      using value_type = std::vector<uint8_t>;

      explicit FrameIterator(uart::UARTDevice &stream) : stream_(&stream) {
        current_.reserve(256);
        buffer_.reserve(256);
      }

      // Fetch next frame if available
      bool next()
      {
        if (!stream_) {
          ESP_LOGW("ld6001", "UART stream not set");
          return false;
        }

        current_.clear();

        while (stream_->available())
        {
          
          // Read from the stream if available
          buffer_.push_back(stream_->read());
          
          // Skip until we have a frame start byte
          drain_until_frame_start();

          // As long as we have a buffer of at least the header size, we can try to process it.
          while (buffer_.size() >= HEADER_SIZE) {
            uint8_t body_len = buffer_[2];
            size_t total_len = HEADER_SIZE + body_len;

            if (buffer_.size() < total_len)
              break;


            if (buffer_[total_len - 1] != FRAME_END) {
              drain_until_frame_start();
              continue;
            }

            auto checksum = buffer_[total_len - 2];
            auto expected_checksum = get_iterator_checksum(buffer_.begin(), buffer_.begin() + total_len - 2);

            if (checksum != expected_checksum)
            {
              ESP_LOGW("ld6001", "Checksum mismatch: expected %02X, got %02X", expected_checksum, checksum);
              ESP_LOGW("ld6001", "%s", format_hex_pretty(buffer_).c_str());

              drain_until_frame_start();
              continue;
            }
          
            current_.assign(buffer_.begin(), buffer_.begin() + total_len);
            buffer_.erase(buffer_.begin(), buffer_.begin() + total_len);
            return true;
          }

        }

        return false;
      }

      const value_type &value() const { return current_; }
      protected:
        void drain_until_frame_start()
        {
          while (!buffer_.empty() && buffer_[0] != FRAME_START) {
            buffer_.erase(buffer_.begin());
          }
        }

        uint8_t get_iterator_checksum(std::vector<unsigned char>::iterator begin, std::vector<unsigned char>::iterator end) {
          uint8_t sum = 0;
    
          for (auto it = begin; it != end; it++) {
              sum += *it;
          }
          
          return sum;
        }
      private:
        uart::UARTDevice *stream_;
        std::vector<uint8_t> buffer_;
        value_type current_;

        static constexpr uint8_t FRAME_START = 0x4D;
        static constexpr uint8_t FRAME_END = 0x4A;
        static constexpr size_t HEADER_SIZE = 6;
    };
  }
}