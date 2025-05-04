#define UNITY_INCLUDE_PRINT_FORMATTED 1


#include "unity.h"
#include <queue>
#include <vector>
#include "frame_iterator.h" // Include the header file for the class being tested
#include <ArduinoFake.h>

// Fake UARTDevice that returns preloaded bytes
class FakeUARTDevice : public esphome::uart::UARTDevice {
  public:
    FakeUARTDevice() : esphome::uart::UARTDevice() {}
   
   void preload(const std::vector<uint8_t> &bytes) {
     for (auto b : bytes) {
       buffer_.push(b);
     }
   }
 
   int available() {
     return static_cast<int>(buffer_.size());
   }
 
   int read() {
     if (buffer_.empty()) {
       return -1;
     }
     int val = buffer_.front();
     buffer_.pop();
     return val;
   }
 
  private:
   std::queue<uint8_t> buffer_{};
 };

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void test_function_should_doBlahAndBlah(void) {
  auto fakeUart = std::make_unique<FakeUARTDevice>();
  fakeUart->preload({0x4D, 0x11, 0x04, 0x00, 0x00, 0x00, 0x4A});

  esphome::ld6001a::FrameIterator frame_iterator = esphome::ld6001a::FrameIterator(*fakeUart);

  auto frameCount = 0;

  while(frame_iterator.next()) {
    // const auto &frame = frame_iterator.value();
    // TEST_ASSERT_EQUAL(0x4D, frame[0]);
    // TEST_ASSERT_EQUAL(0x11, frame[1]);
    // TEST_ASSERT_EQUAL(0x00, frame[2]);
    // TEST_ASSERT_EQUAL(0x00, frame[3]);
    // TEST_ASSERT_EQUAL(0x00, frame[4]);
    // TEST_ASSERT_EQUAL(0x00, frame[5]);
    // TEST_ASSERT_EQUAL(0x4A, frame[6]);
    frameCount++;
  }

  TEST_ASSERT_EQUAL(0, frameCount);
}

void test_function_should_doAlsoDoBlah(void) {
  // more test stuff
}

int runUnityTests(void) {
  UNITY_BEGIN();
  RUN_TEST(test_function_should_doBlahAndBlah);
  RUN_TEST(test_function_should_doAlsoDoBlah);
  return UNITY_END();
}

// WARNING!!! PLEASE REMOVE UNNECESSARY MAIN IMPLEMENTATIONS //

/**
  * For native dev-platform or for some embedded frameworks
  */
int main(void) {
  return runUnityTests();
}

/**
  * For Arduino framework
  */
void setup() {
  // Wait ~2 seconds before the Unity test runner
  // establishes connection with a board Serial interface
  delay(2000);

  runUnityTests();
}
void loop() {}

/**
  * For ESP-IDF framework
  */
void app_main() {
  runUnityTests();
}