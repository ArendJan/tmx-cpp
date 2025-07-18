#pragma once
#include "tmx_cpp/sensors/Sensor_t.hpp"
#include "tmx_cpp/sensors/Sensor_types.hpp"
#include <algorithm>
#include <cassert>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <utility>
#include <vector>
namespace tmx_cpp {

class TMX;
class Sensors {
public:
  size_t add_sensor(uint8_t sens_num, SENSOR_TYPE type,
                    std::vector<uint8_t> data,
                    std::function<void(std::vector<uint8_t>)> callback);
  std::vector<std::pair<SENSOR_TYPE, std::function<void(std::vector<uint8_t>)>>>
      sensors;
  // void add_adxl345(uint8_t i2c_port,
  //                  std::function<void(std::vector<uint8_t>)>
  //                      callback); // todo: change to adxl data function
  // void add_veml6040(uint8_t i2c_port,
  //                   std::function<void(std::vector<uint8_t>)> callback);
  TMX *tmx;
  Sensors(TMX *tmx);
  void callback(std::vector<uint8_t> data);
  void add_sens(std::shared_ptr<Sensor_type> module);
  void check_features();
  void report_features(SENSOR_TYPE type, bool ok, std::vector<uint8_t> data);
  std::vector<std::pair<SENSOR_TYPE, std::vector<uint8_t>>> sensor_features;

  // void add_sensor(std::shared_ptr<Sensor_type> module);
private:
  // modules feature locking
  bool sensor_detected = false;
  std::mutex sensor_mutex;
  std::condition_variable sensor_cv;
};

} // namespace tmx_cpp

#include "tmx.hpp" // fix for circular dependency