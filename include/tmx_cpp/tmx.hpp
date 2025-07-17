#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "async_serial/AsyncSerial.h"
#include <algorithm>
#include <cassert>
#include <ctime>
#include <functional>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "tmx_cpp/board_features.hpp"
#include "tmx_cpp/message_types.hpp"
#include "tmx_cpp/modules.hpp"
#include "tmx_cpp/sensors.hpp"
#include "tmx_cpp/types.hpp"
// #define TMX_TX_DEBUG 1
#ifdef TMX_TX_DEBUG
#define TMX_DEBUG 1 &&
#else
#define TMX_DEBUG 0 &&
#endif
namespace tmx_cpp {

using callback_func = std::function<void(std::vector<uint8_t>)>;
using callback_vec = std::vector<callback_func>;
using callback_opt_vec = std::vector<std::optional<callback_func>>;
using callback_opt_time_vec =
    std::vector<std::pair<std::time_t, std::optional<callback_func>>>;

using callback_func_pin = std::function<void(uint8_t, uint8_t)>;
using callback_func_pin16 = std::function<void(uint8_t, uint16_t)>;
using callback_func_pin_int = std::function<void(uint8_t, int8_t)>;

class TMX {
public:
  BoardFeatures board_features;
  std::vector<uint8_t> buffer;
  // normal callbacks
  callback_vec ping_callbacks;
  callback_vec firmware_callbacks;
  callback_vec pico_unique_id_callbacks;
  callback_vec digital_callbacks;
  callback_vec analog_callbacks;
  callback_vec servo_unavailable_callbacks;
  callback_opt_time_vec i2c_write_callbacks;
  // callback_opt_vec i2c_read_failed_callbacks;
  callback_opt_time_vec i2c_read_callbacks;
  callback_vec sonar_distance_callbacks;
  callback_vec dht_callbacks;
  callback_vec spi_callbacks;
  callback_vec encoder_callbacks;
  callback_vec debug_print_callbacks;
  callback_vec sensor_callbacks;
  callback_vec module_callbacks;
  callback_vec serial_loop_back_callbacks;
  // callbacks with a specific pin:
  std::vector<std::pair<uint8_t, callback_func_pin>> digital_callbacks_pin;
  std::vector<std::pair<uint8_t, callback_func_pin16>> analog_callbacks_pin;
  std::vector<std::pair<uint8_t, callback_func_pin_int>>
      encoder_callbacks_pin; // todo check types
  std::vector<std::pair<uint8_t, callback_func_pin16>> sonar_callbacks_pin;
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> features;
  void add_callback(MESSAGE_IN_TYPE type,
                    std::function<void(const std::vector<uint8_t> &)> callback);
  void add_digital_callback(uint8_t pin,
                            std::function<void(uint8_t, uint8_t)> callback);
  void add_analog_callback(uint8_t pin,
                           std::function<void(uint8_t, uint16_t)> callback);

  void parse(std::vector<uint8_t> &buffer);
  void parseOne(const std::vector<uint8_t> &buffer);
  void parseOne_task(const std::vector<uint8_t> &buffer);
  boost::asio::thread_pool parsePool;
  void stop();
  std::shared_ptr<Sensors> sensors_sys;
  std::shared_ptr<Modules> module_sys;

public:
  TMX(std::function<void()> stop_func, std::string port = "/dev/ttyACM0",
      size_t parse_pool_size = std::thread::hardware_concurrency()/2);
  ~TMX();
  enum PIN_MODES : uint8_t {
    DIGITAL_INPUT = 0,
    DIGITAL_OUTPUT = 1,
    PWM_OUTPUT = 2,
    DIGITAL_INPUT_PULL_UP = 3,
    DIGITAL_INPUT_PULL_DOWN = 4,
    ANALOG_INPUT = 5,
  };
  void callback(const char *data, size_t len);
  void sendPing(uint8_t num = 0);
  void sendMessage(const std::vector<uint8_t> &message);
  void sendMessage(MESSAGE_TYPE type, const std::vector<uint8_t> &message);
  // Normal functions for use by the user:
  void setPinMode(uint8_t pin, TMX::PIN_MODES mode, bool reporting = true,
                  uint16_t analog_differential = 0);
  void digitalWrite(uint8_t pin, bool value);
  void pwmWrite(uint8_t pin, uint16_t value);

  bool attach_encoder(uint8_t pin_A, uint8_t pin_B,
                      callback_func_pin_int callback);

  /// The sonar callback and return value is in centimeters, as specified by the
  /// original telemetrix protocol.
  bool attach_sonar(uint8_t trigger, uint8_t echo,
                    std::function<void(uint8_t, uint16_t)> callback);

  // TODO: Maybe add angle remapping
  void attach_servo(uint8_t pin, uint16_t min_pulse = 1000,
                    uint16_t max_pulse = 2000);
  void write_servo(uint8_t pin, uint16_t duty_cycle);
  void detach_servo(uint8_t pin);

  void setScanDelay(uint8_t delay);
  bool setI2CPins(uint8_t sda, uint8_t scl, uint8_t port);
  bool i2cWrite(uint8_t port, uint8_t address, std::vector<uint8_t> data,
                std::function<void(bool, std::vector<uint8_t>)> callback,
                bool nostop = false); // TODO: add timeout
  bool i2cRead(uint8_t port, uint8_t address, uint8_t len,
               std::vector<uint8_t> data,
               std::function<void(bool, std::vector<uint8_t>)>
                   callback); // TODO: add timeout
  std::shared_ptr<CallbackAsyncSerial> serial;
  bool reset_bootloader() {
    if (this->get_feature(MESSAGE_TYPE::BOOTLOADER_RESET).first) {
      sendMessage(MESSAGE_TYPE::BOOTLOADER_RESET, {});
      return true;
    }
    return false;
  }
  static bool check_port(const std::string &port);
  struct serial_port {
    std::string port_name;
    uint pid;
    uint vid;
  };
  static const std::vector<serial_port> accepted_ports;

  static std::vector<serial_port> get_available_ports();
  static bool is_accepted_port(const serial_port &port);
  static uint8_t get_id(const serial_port &port);
  static bool set_id(const serial_port &port, uint8_t id);
  static std::pair<bool, std::vector<uint8_t>>
  parse_buffer_for_message(std::vector<uint8_t> &buffer, uint8_t len,
                           uint8_t type);
  std::pair<bool, std::vector<uint8_t>>
  get_feature(const enum MESSAGE_TYPE type);
  bool is_stopped = false;

private:                   /* Ping related elements */
  std::thread ping_thread; // TODO: jthread from c++20
  uint8_t last_ping = 0;
  uint8_t magic = 0;
  bool first_magic = true;

  void ping_task();
  void ping_callback(const std::vector<uint8_t> msg);
  std::function<void()> stop_func;

  std::thread feature_detect_thread;
  void feature_detect_task();
  bool feature_detected = false;
  std::mutex feature_mutex;
  std::condition_variable feature_cv;
  int feature_index = 0;
};

} // namespace tmx_cpp