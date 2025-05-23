#include <tmx_cpp/message_types.hpp>
#include <tmx_cpp/sensors.hpp>

using namespace tmx_cpp;

Sensors::Sensors(std::shared_ptr<TMX> tmx) {
  using namespace std::placeholders;

  this->tmx = tmx;
  tmx->add_callback(MESSAGE_IN_TYPE::SENSOR_REPORT, std::bind(&Sensors::callback, this, _1));
}

size_t Sensors::add_sensor(uint8_t sens_num, SENSOR_TYPE type, std::vector<uint8_t> data,
                           std::function<void(std::vector<uint8_t>)> callback) {
  // todo: send data to pico
  sensors.push_back(std::make_pair(type, callback));
  std::vector<uint8_t> addSensMsg(data.begin(), data.end());
  addSensMsg.insert(addSensMsg.begin(), {sens_num, (uint8_t)type});

  tmx->sendMessage(MESSAGE_TYPE::SENSOR_NEW, addSensMsg);
  return sensors.size() - 1;
}

// void Sensors::add_adxl345(uint8_t i2c_port,
//                           std::function<void(std::vector<uint8_t>)> callback)
//                           {
//   add_sensor(SENSOR_TYPE::ADXL345, {i2c_port},
//              [&callback](std::vector<uint8_t> data) {
//                // TODO: add adxl parsing
//                callback(data);
//              });
// }

// void Sensors::add_veml6040(uint8_t i2c_port,
//                            std::function<void(std::vector<uint8_t>)>
//                            callback) {
//   add_sensor(SENSOR_TYPE::VEML6040, {i2c_port},
//              [&callback](std::vector<uint8_t> data) {
//                // TODO: add veml parsing
//                callback(data);
//              });
// }

void Sensors::add_sens(std::shared_ptr<Sensor_type> sensor) {
  using namespace std::placeholders;

  auto mod_num = this->sensors.size();
  auto init_data = sensor->init_data();
  std::cout << "adding sensor" << sensor->type << "data" << init_data[1] << std::endl;
  auto act_mod_num = add_sensor(mod_num, sensor->type, init_data,
                                std::bind(&Sensor_type::data_callback, sensor, _1));

  if (act_mod_num != mod_num) {
    std::cout << "Error adding sensor" << std::endl;
    return;
  }
}

void Sensors::callback(std::vector<uint8_t> data) {
  uint8_t module_num = data[2];
  std::vector<uint8_t> module_data(data.begin() + 4, data.end());
  this->sensors[module_num].second(module_data);
}
