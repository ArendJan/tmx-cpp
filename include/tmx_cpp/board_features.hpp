#pragma once

#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdint.h>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace tmx_cpp {

class BoardFeatures {
public:
  int max_encoders = 0;
  int encoder_dirs = 0;
  int max_servos = 0;
  int max_sonar = 0;

  int digital_pins = 0;
  int analog_bits = 0;
  int analog_pins = 0;
  int analog_offset = 0;
  int pwm_max = 0;
  int i2c_count = 0;
  std::vector<int> analog_pins_list;
  void parse_features(std::vector<uint8_t> data);
};
}; // namespace tmx_cpp