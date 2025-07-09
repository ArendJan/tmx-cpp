#include <tmx_cpp/message_types.hpp>
#include <tmx_cpp/sensors.hpp>

using namespace tmx_cpp;

Sensors::Sensors(std::shared_ptr<TMX> tmx) {
  using namespace std::placeholders;

  this->tmx = tmx;
  this->tmx->sensors_sys = this;
  tmx->add_callback(MESSAGE_IN_TYPE::SENSOR_REPORT,
                    std::bind(&Sensors::callback, this, _1));
}

size_t Sensors::add_sensor(uint8_t sens_num, SENSOR_TYPE type,
                           std::vector<uint8_t> data,
                           std::function<void(std::vector<uint8_t>)> callback) {
  // todo: send data to pico
  sensors.push_back(std::make_pair(type, callback));
  std::vector<uint8_t> addSensMsg(data.begin(), data.end());
  addSensMsg.insert(addSensMsg.begin(),
                    {1, sens_num, (uint8_t)type}); // 1 = add sensor

  tmx->sendMessage(MESSAGE_TYPE::SENSOR_NEW, addSensMsg);
  return sensors.size() - 1;
}

void Sensors::add_sens(std::shared_ptr<Sensor_type> sensor) {
  using namespace std::placeholders;
  if (!this->sensor_detected) {
#ifdef TMX_TX_DEBUG
    std::cout << "Sensor not detected yet, waiting..." << std::endl;
#endif
    std::unique_lock<std::mutex> lk(this->sensor_mutex);

    this->sensor_cv.wait(lk, [this] { return this->sensor_detected; });
  }
  auto mod_num = this->sensors.size();
  auto init_data = sensor->init_data();
  std::cout << "adding sensor" << sensor->type << "data" << init_data[1]
            << std::endl;
  auto act_mod_num =
      add_sensor(mod_num, sensor->type, init_data,
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

void Sensors::check_features() {
  // return;
  std::cout << "Checking for sensors" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  {
    for (auto i = 0; i < SENSOR_TYPE::SENSOR_MAX && !this->tmx->is_stopped; i++) {
#ifdef TMX_TX_DEBUG
      std::cout << "Checking for sensor type " << (int)i << std::endl;
#endif
      tmx->sendMessage(MESSAGE_TYPE::SENSOR_NEW, {0, (uint8_t)i});
      // std::cout << " sleeping for sensor type " << (int)i << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      // std::cout << "done sleeping for sensor type " << (int)i << std::endl;
    }
  }
  std::cout << "Waiting for sensors to report features" << std::endl;
}

void Sensors::report_features(SENSOR_TYPE type, bool ok,
                              std::vector<uint8_t> data) {
#ifdef TMX_TX_DEBUG
  if (ok) {
    std::cout << "Sensor " << (int)type << " is supported" << std::endl;
  } else {
    std::cout << "Sensor " << (int)type << " is not supported" << std::endl;
  }
#endif
  std::cout << "report_features sensors: " << (int)type << " ok: " << (int)ok
            << std::endl;
  if (type >= SENSOR_TYPE::SENSOR_MAX) {
    std::cout << "Sensor type out of range" << std::endl;
    return;
  }
  if (ok) {
#ifdef TMX_TX_DEBUG
    std::cout << "Sensor " << (int)type << " is supported" << std::endl;
#endif
    this->sensor_features.push_back({type, data});
  } else {
#ifdef TMX_TX_DEBUG
    std::cout << "Sensor " << (int)type << " is not supported" << std::endl;
#endif
  }

  // notify the sensor system that a sensor has been detected
  if (type == SENSOR_TYPE::SENSOR_MAX - 1) {
    std::unique_lock<std::mutex> lk(this->sensor_mutex);
    this->sensor_detected = true;
    this->sensor_cv.notify_all();
#ifdef TMX_TX_DEBUG
    std::cout << "All Sensor checked, notifying all waiting threads"
              << std::endl;
#endif
  }
}