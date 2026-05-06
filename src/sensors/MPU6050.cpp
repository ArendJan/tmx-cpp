#include <cassert>
#include <ranges>
#include <span>

#include <math.h>
#include <tmx_cpp/sensors/MPU6050.hpp>
#include <tmx_cpp/serialization.hpp>

using namespace tmx_cpp;

MPU6050_module::MPU6050_module(uint8_t i2c_port, uint8_t address,
                               MPU6050_cb_t data_cb)
    : data_cb(data_cb), i2c_port(i2c_port), address(address) {
  this->type = SENSOR_TYPE::MPU6050_t;
}

std::vector<uint8_t> MPU6050_module::init_data() {
  this->type = SENSOR_TYPE::MPU6050_t;

  return {i2c_port, address};
}
#include <iostream>
void MPU6050_module::data_callback(std::vector<uint8_t> data) {
  static_assert(sizeof(float) == sizeof(uint32_t));
  static_assert(sizeof(float) == 4);
  // std::cout << "tmx MPU6050_module received data callback with " <<
  // data.size() << " bytes"
  //           << std::endl;
  // 2 measurements x 3 axes + Temperature (1)
  assert(data.size() == sizeof(uint16_t) * (3 * 2 + 1));
  auto data_span = std::span(data);

  std::array<float, 3> acceleration;
  std::array<float, 3> gyro;
  float temperature;

  for (int acc_idx = 0; acc_idx < 3; acc_idx++) {
    // pico sets it to +-2g range, so scale accordingly (see datasheet)
    acceleration[acc_idx] =
        decode_i16(data_span.subspan(acc_idx * sizeof(uint16_t))
                       .first<sizeof(uint16_t)>()) /
        std::pow(2, 15) * 2; // scale to g's
  }

  for (int gyro_idx = 3; gyro_idx < 6; gyro_idx++) {
    // pico sets it to +-250dps range, so scale accordingly (see datasheet)
    gyro[gyro_idx - 3] =
        decode_i16(data_span.subspan(gyro_idx * sizeof(uint16_t))
                       .first<sizeof(uint16_t)>()) /
        std::pow(2, 15) * 250; // scale to degrees/s
  }

  temperature =
      decode_i16(
          data_span.subspan(6 * sizeof(uint16_t)).first<sizeof(uint16_t)>()) /
          340.0 +
      36.53; // scale to degrees Celsius

  data_cb(acceleration, gyro, temperature);
}
