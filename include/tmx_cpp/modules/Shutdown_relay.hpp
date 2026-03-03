#pragma once
#include <chrono>
#include <future>
#include <string>

#include "tmx_cpp/modules/Module_t.hpp"

using namespace std::chrono_literals;


namespace tmx_cpp {

class Shutdown_relay_module : public Module_type {
public:
  std::function<void(std::vector<uint8_t>)> send_module;
  Shutdown_relay_module(uint8_t pin, uint8_t turn_off_time_sec, bool turn_off_value);

  virtual std::vector<uint8_t> init_data() override;
  virtual void data_callback(std::vector<uint8_t> data) override;
  void send_shutdown_signal(bool enable) {
    if (this->send_module) {
      this->send_module({enable ? 1 : 0});
    } else {
      std::cerr << "Error: send_module function is not set for shutdown relay module, cannot send shutdown signal." << std::endl;
    }
  }
  // FIXME: Maybe give this a default implementation, since all implementations
  // are the same.
  virtual void attach_send_module(
      std::function<void(std::vector<uint8_t>)> send_module) override;

private:
  uint8_t pin;
  uint8_t turn_off_time_sec;
  bool turn_off_value;

};

} // namespace tmx_cpp