#include <cassert>
#include <iostream>
#include <span>

#include <tmx_cpp/tmx.hpp>

int main() {

  // Test the TMX class
  tmx_cpp::TMX tmx([]() { std::cout << "Stopping TMX..." << std::endl; },
                   "/dev/ttyACM0",
                   4); // Adjust the port and parse pool size as needed
  // Check if the TMX object is created successfully
  //     assert(tmx.is_valid());
  std::cout << "TMX object created successfully." << std::endl;

  // Test getting available ports
  auto ports = tmx.get_available_ports();
  assert(!ports.empty());
  std::cout << "Available ports found: " << ports.size() << std::endl;

  // Test if a specific port is accepted
  if (!ports.empty()) {
    assert(tmx.is_accepted_port(ports[0]));
    std::cout << "First port is accepted." << std::endl;
  }
  tmx.setPinMode(10, tmx_cpp::TMX::PIN_MODES::DIGITAL_INPUT);
  tmx.add_digital_callback(10, [](uint8_t pin, uint8_t value) {
    std::cout << "Digital callback for pin " << (int)pin << " with value "
              << (int)value << std::endl;
  });
  std::cout << "Digital callback added for pin 10." << std::endl;
  // sleep for a while to allow callbacks to be processed
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "Sending ping..." << std::endl;
  tmx.stop();
  return 0;
}