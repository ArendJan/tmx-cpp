#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "tmx_cpp/modules/Module_t.hpp"
#include <tmx_cpp/modules/HiwonderServo.hpp>
#include <tmx_cpp/modules/PCA9685.hpp>

namespace tmx_cpp {

class TMX;
class Modules {
public:
  int add_module(uint8_t mod_num, MODULE_TYPE type, std::vector<uint8_t> data,
                 std::function<void(std::vector<uint8_t>)> callback);
  std::vector<std::pair<MODULE_TYPE, std::function<void(std::vector<uint8_t>)>>>
      modules;
  std::shared_ptr<TMX> tmx;
  std::vector<std::pair<MODULE_TYPE, std::vector<uint8_t>>> module_features;
  void report_features(MODULE_TYPE type, bool ok, std::vector<uint8_t> data);
  Modules(std::shared_ptr<TMX> tmx);
  void callback(std::vector<uint8_t> data);

  bool send_module(uint8_t module_num, std::vector<uint8_t> data);

  // std::shared_ptr<PCA9685_module> add_pca9685();
  void add_mod(std::shared_ptr<Module_type> module);

  // modules feature locking
  bool module_detected = false;
  std::mutex module_mutex;
  std::condition_variable module_cv;
  void check_features();

private:
};
void empty_callback(std::vector<uint8_t> data);

} // namespace tmx_cpp

#include "tmx.hpp" // fix for circular dependency