#pragma once
#include "tmx_cpp/sensors/Sensor_t.hpp"
#include <functional>
#include <stdint.h>
#include <tuple>
#include <vector>

namespace tmx_cpp {

using AS5600_cb_t = std::function<void(
    std::vector<std::tuple<int, float, int>>)>; // channel, angle(rad),
                                                // angle(ticks)
class AS5600_tmx_sensor : public Sensor_type {
public:
  std::function<void(std::vector<uint8_t>)> send_module;
  AS5600_tmx_sensor(uint8_t i2c_port, uint8_t address,
                    std::vector<int> channels, AS5600_cb_t data_cb);

  std::vector<uint8_t> init_data();
  void data_callback(std::vector<uint8_t> data);
  AS5600_cb_t data_cb;

private:
  uint8_t i2c_port = 0;
  //   uint8_t address = 0x40;
  uint8_t mux = 0x00; // Mux
  std::vector<int> channels = {};
};

} // namespace tmx_cpp