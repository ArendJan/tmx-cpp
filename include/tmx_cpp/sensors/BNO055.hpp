#pragma once
#include "tmx_cpp/sensors/Sensor_t.hpp"
#include <functional>
#include <stdint.h>
#include <vector>

namespace tmx_cpp {
struct BNO055_MOD_data {
  // data is already in the correct units (accel in m/s^2, gyro in rad/s, mag in
  // uT, temp in C) std::vector<float> euler;
  std::vector<float> gyro;
  // std::vector<float> linear_accel;
  std::vector<float> mag;
  std::vector<float> accel;
  // std::vector<float> gravity;
  struct {
    float x;
    float y;
    float z;
    float w;
  } quat;
  uint8_t temp;
};
using BNO055_cb_t = std::function<void(const BNO055_MOD_data &)>;
class BNO055_module : public Sensor_type {
public:
  std::function<void(std::vector<uint8_t>)> send_module;
  BNO055_module(uint8_t i2c_port, uint8_t address, BNO055_cb_t data_cb);

  std::vector<uint8_t> init_data();
  void data_callback(std::vector<uint8_t> data);
  BNO055_cb_t data_cb;

private:
  uint8_t i2c_port = 0;
};

} // namespace tmx_cpp