#pragma once
#include <cstdint>
#include <iostream>

namespace tmx_cpp {

enum struct MESSAGE_TYPE : uint8_t {
  SERIAL_LOOP_BACK = 0,
  SET_PIN_MODE = 1,
  DIGITAL_WRITE = 2,
  PWM_WRITE = 3,
  MODIFY_REPORTING = 4,
  FIRMWARE_VERSION = 5,
  GET_PICO_UNIQUE_ID = 6,
  SERVO_ATTACH = 7,
  SERVO_WRITE = 8,
  SERVO_DETACH = 9,
  I2C_BEGIN = 10,
  I2C_READ = 11,
  I2C_WRITE = 12,
  SONAR_NEW = 13,
  DHT_NEW = 14,
  STOP_ALL_REPORTS = 15,
  ENABLE_ALL_REPORTS = 16,
  RESET_DATA = 17,
  RESET_BOARD = 18,
  INITIALIZE_NEO_PIXELS = 19,
  SHOW_NEO_PIXELS = 20,
  SET_NEO_PIXEL = 21,
  CLEAR_ALL_NEO_PIXELS = 22,
  FILL_NEO_PIXELS = 23,
  SPI_INIT = 24,
  SPI_WRITE = 25,
  SPI_READ = 26,
  SPI_SET_FORMAT = 27,
  SPI_CS_CONTROL = 28,
  SET_SCAN_DELAY = 29,
  ENCODER_NEW = 30,
  SENSOR_NEW = 31,
  PING = 32,
  MODULE_NEW = 33,
  MODULE_DATA = 34,
  GET_ID = 35,
  SET_ID = 36,
  FEATURE_REQUEST = 37,
  BOOTLOADER_RESET = 38,
  MAX
};

enum struct MESSAGE_IN_TYPE : uint8_t {
  SERIAL_LOOP_BACK_REPORT = 0,
  DIGITAL_REPORT = 2,
  ANALOG_REPORT = 3,
  FIRMWARE_REPORT = (uint8_t)MESSAGE_TYPE::FIRMWARE_VERSION,
  REPORT_PICO_UNIQUE_ID = 6,
  SERVO_UNAVAILABLE = 7, // for the future
  I2C_WRITE_REPORT = 8,
  I2C_READ_FAILED = 9,
  I2C_READ_REPORT = 10,
  SONAR_DISTANCE = 11,
  DHT_REPORT = 12,
  SPI_REPORT = 13,
  ENCODER_REPORT = 14,
  DEBUG_PRINT = 99,
  SENSOR_REPORT = 20,
  SENSOR_MAIN_REPORT = 21, // data from the sensor system
  PONG_REPORT = 32,
  MODULE_MAIN_REPORT = 33, // data from the module system
  MODULE_REPORT = 34, // data from a module
  GET_ID_REPORT = (uint8_t)MESSAGE_TYPE::GET_ID,
  SET_ID_REPORT = (uint8_t)MESSAGE_TYPE::SET_ID,
  FEATURE_REQUEST_REPORT = (uint8_t)MESSAGE_TYPE::FEATURE_REQUEST,
};

} // namespace tmx_cpp
std::ostream& operator<<(std::ostream& out, const tmx_cpp::MESSAGE_IN_TYPE&value);

std::ostream& operator<<(std::ostream& out, const tmx_cpp::MESSAGE_TYPE&value);
