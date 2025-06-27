#include "tmx_cpp/board_features.hpp"
#include "tmx_cpp/message_types.hpp"
#include "tmx_cpp/serialization.hpp"
#include "tmx_cpp/modules/Module_types.hpp"

using namespace tmx_cpp;

void BoardFeatures::parse_features(std::vector<uint8_t> data) {
  if (data.size() < 2) {
    std::cout << "Error: not enough data to parse features" << std::endl;
    return;
  }
//   msg: type, ok, data
    auto type = (MESSAGE_TYPE)data[0];
    auto ok = data[1];
   
    switch (type) {
        case MESSAGE_TYPE::SERVO_ATTACH:
            this->max_servos = data[2];
            break;
        case MESSAGE_TYPE::ENCODER_NEW:
            this->max_encoders = data[2];
            this->encoder_dirs = data[3];
            break;
        case MESSAGE_TYPE::SONAR_NEW:
            this->max_sonar = data[2];
            break;
        case MESSAGE_TYPE::SET_PIN_MODE:
            this->digital_pins = data[2];
            this->analog_pins = data[3];
            // this->analog_offset = data[4];
            // std::cout << "analog offset " << (int) this->analog_offset << " adfadsf" << std::endl;
            for(size_t i = 4; i < data.size(); i++) {
                std::cout << "Analog pin: " << (int)data[i] << std::endl;
                this->analog_pins_list.push_back(data[i]);
            }
            break;
    }
    // print all features
    #define TMX_TX_DEBUG 1
    #ifdef TMX_TX_DEBUG
    std::cout << "Features: " << std::endl;
    std::cout << "max_servos: " << (int)this->max_servos << std::endl;
    std::cout << "max_encoders: " << (int)this->max_encoders << std::endl;
    std::cout << "encoder_dirs: " << (int)this->encoder_dirs << std::endl;
    std::cout << "max_sonar: " << (int)this->max_sonar << std::endl;
    std::cout << "digital_pins: " << (int)this->digital_pins << std::endl;
    std::cout << "analog_pins: " << (int)this->analog_pins << std::endl;
    std::cout << "analog_offset: " << (int)this->analog_offset << std::endl;
    #endif
}