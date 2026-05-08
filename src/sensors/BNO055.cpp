#include <bit>
#include <cassert>
#include <iostream>
#include <span>

#include <tmx_cpp/sensors/BNO055.hpp>
#include <tmx_cpp/serialization.hpp>

using namespace tmx_cpp;

// struct BNO055_MOD_data {
//         float voltage;
//         float current;
// }
BNO055_module::BNO055_module(uint8_t i2c_port, uint8_t address,
                             BNO055_cb_t data_cb)
    : data_cb(data_cb), i2c_port(i2c_port) {
  std::cout << "Creating BNO055 module on port " << (int)i2c_port << std::endl;
  // this->
  this->type = SENSOR_TYPE::BNO055_t;
}

std::vector<uint8_t> BNO055_module::init_data() {
  std::cout << "bno port: " << (int)i2c_port << std::endl;
  this->type = SENSOR_TYPE::BNO055_t;

  return {i2c_port};
}

void BNO055_module::data_callback(std::vector<uint8_t> data) {
  static_assert(sizeof(float) == sizeof(uint32_t));
  static_assert(sizeof(float) == 4);
  // assert(data.size() == ((3*3+4)*sizeof(float) + 1)); // 3 axes, each with 3
  // floats (accel, gyro, mag) + temp
  auto data_span = std::span(data);
  // std::cout << "recevied bno data!!!" << std::endl;
  // for(auto byte : data) {
  //   std::cout << std::hex << (int)byte << " ";
  // }
  // std::cout << std::dec << std::endl;
  auto out_floats = std::vector<float>(data.size() / sizeof(float));
  for (size_t i = 0; i < out_floats.size(); i++) {
    out_floats[i] = decode_float(
        data_span.subspan(i * sizeof(float)).first<sizeof(float)>());
    // std::cout << "float " << i << ": " << out_floats[i] << std::endl;
  }
  // std::cout << "temp: " << (int) data[data.size()-1] << std::endl;

  std::vector<std::vector<float>> sub_vectors;
  // split out_floats into 3 vectors of 3 floats each
  size_t subvec_size = 3;
  size_t num_subvecs = 3;
  for (std::size_t i(0); i < num_subvecs; ++i) {
    sub_vectors.emplace_back(out_floats.begin() + i * subvec_size,
                             out_floats.begin() + (i + 1) * subvec_size);
  }
  // order of data sent by pico:
  // Adafruit_BNO055::VECTOR_GYROSCOPE,
  // Adafruit_BNO055::VECTOR_MAGNETOMETER,
  // Adafruit_BNO055::VECTOR_ACCELEROMETER,
  // quaternion (4 floats)
  // temp (1 byte)
  BNO055_MOD_data mod_data{
      // .euler = sub_vectors[0],
      .gyro = sub_vectors[0],
      // .linear_accel = sub_vectors[2],
      .mag = sub_vectors[1],
      .accel = sub_vectors[2],
      // .gravity = sub_vectors[5],
      .quat =
          {
              .x = out_floats[3 * 3],
              .y = out_floats[3 * 3 + 1],
              .z = out_floats[3 * 3 + 2],
              .w = out_floats[3 * 3 + 3],
          },
      .temp = data[data.size() - 1],
  };
  data_cb(mod_data);
}
