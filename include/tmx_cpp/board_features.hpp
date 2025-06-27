#pragma once

#include <stdint.h>
#include <string>
#include <vector>
#include <utility>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <future>
#include <optional>
#include <iostream>

namespace tmx_cpp {

class BoardFeatures {
public:
    int max_encoders = 0;
    int encoder_dirs = 0;
    int max_servos = 0;
    int max_sonar = 0;

    int digital_pins = 0;
    int analog_pins = 0;
    int analog_offset = 0;
    std::vector<int> analog_pins_list;
    void parse_features(std::vector<uint8_t> data);
};
};