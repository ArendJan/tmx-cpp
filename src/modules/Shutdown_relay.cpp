#include <chrono>
#include <iostream>

#include "tmx_cpp/modules/Shutdown_relay.hpp"
#include "tmx_cpp/types.hpp"

using namespace tmx_cpp;



Shutdown_relay_module::Shutdown_relay_module(uint8_t pin, uint8_t turn_off_time_sec, bool turn_off_value)
    : pin(pin), turn_off_time_sec(turn_off_time_sec),
      turn_off_value(turn_off_value) {
        if(turn_off_time_sec > 200) {
          std::cerr << "Turn off time is too high for the shutdown relay module, capping it at 200 seconds." << std::endl;
          this->turn_off_time_sec = 200;
        }
        if(turn_off_time_sec < 1) {
          std::cerr << "Turn off time is too low for the shutdown relay module, setting it to 20 seconds." << std::endl;
          this->turn_off_time_sec = 20;
        }
  type = MODULE_TYPE::SHUTDOWN_RELAY;
}


void Shutdown_relay_module::data_callback(std::vector<uint8_t> data) {
  // should not receive any data, since this module is write only, but just in case, ignore it
  std::cerr << "Warning: Shutdown relay module received data, but this module is write only. Ignoring data." << std::endl;
  return;
}


void Shutdown_relay_module::attach_send_module(
    std::function<void(std::vector<uint8_t>)> send_module) {
  this->send_module = send_module;
}


std::vector<uint8_t> Shutdown_relay_module::init_data() {
  return {this->pin, this->turn_off_value ? 1 : 0, this->turn_off_time_sec};
}