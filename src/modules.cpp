#include <tmx_cpp/message_types.hpp>
#include <tmx_cpp/modules.hpp>

using namespace tmx_cpp;

Modules::Modules(std::shared_ptr<TMX> tmx) {
  using namespace std::placeholders;

  this->tmx = tmx;
  this->tmx->module_sys = this;
  tmx->add_callback(MESSAGE_IN_TYPE::MODULE_REPORT, std::bind(&Modules::callback, this, _1));
  // this->check_features();
}

void Modules::check_features() {
  // std::cout << "Checking for modules" << std::endl;
  {
    for(auto i = 0; i < MODULE_TYPE::MAX; i++) {
      #ifdef TMX_TX_DEBUG
      std::cout << "Checking for module type " << (int)i << std::endl;
      #endif
      tmx->sendMessage(MESSAGE_TYPE::MODULE_NEW, {0, (uint8_t)i});
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
}

void Modules::report_features(MODULE_TYPE type, bool ok, std::vector<uint8_t> data) {
  if (type >= MODULE_TYPE::MAX) {
    std::cout << "Module type out of range" << std::endl;
    return;
  }
  if (ok) {
    #ifdef TMX_TX_DEBUG
    std::cout << "Module " << (int)type << " is supported" << std::endl;
    #endif
    this->module_features.push_back({type, data});
  } else {
    #ifdef TMX_TX_DEBUG
    std::cout << "Module " << (int)type << " is not supported" << std::endl;
    #endif
  }
}

int Modules::add_module(uint8_t mod_num, MODULE_TYPE type, std::vector<uint8_t> data,
                        std::function<void(std::vector<uint8_t>)> callback) {
  // check if module feature is available
  if (type >= MODULE_TYPE::MAX) {
    std::cout << "Module type out of range" << std::endl;
    return -1;
  }
  bool found = false;
  for(auto i = 0; i < this->module_features.size(); i++) {
    if (this->module_features[i].first == type) {
      #ifdef TMX_TX_DEBUG
      std::cout << "Module " << (int)type << " found list" << std::endl;
      #endif
      found = true;
      break;
    }
  }
  if (!found) {
    std::cout << "Module type " << (int)type << " not found" << std::endl;
    return -1;
  }
  modules.push_back(std::make_pair(type, callback));
  std::vector<uint8_t> addModMsg(data.begin(), data.end());
  addModMsg.insert(addModMsg.begin(), {1, mod_num, (uint8_t)type}); // 1: add module

  tmx->sendMessage(MESSAGE_TYPE::MODULE_NEW, addModMsg);
  return modules.size() - 1;
}

void Modules::add_mod(std::shared_ptr<Module_type> module) {
  using namespace std::placeholders;

  auto mod_num = this->modules.size();
  auto init_data = module->init_data();

  auto act_mod_num = add_module(mod_num, module->type, init_data,
                                std::bind(&Module_type::data_callback, module, _1));
  module->attach_send_module(
      [this, act_mod_num](std::vector<uint8_t> data) { this->send_module(act_mod_num, data); });
  if (act_mod_num != mod_num) {
    std::cout << "Error adding module" << std::endl;
    return;
  }
}

void Modules::callback(std::vector<uint8_t> data) {
  uint8_t module_num = data[2];
  std::vector<uint8_t> module_data(data.begin() + 4, data.end());
  this->modules[module_num].second(module_data);
}

bool Modules::send_module(uint8_t module_num, std::vector<uint8_t> data) {
  if (module_num >= modules.size() || module_num < 0) {
    return false;
  }
  std::vector<uint8_t> addModMsg(data.begin(), data.end());
  addModMsg.insert(addModMsg.begin(), {(uint8_t)module_num});

  tmx->sendMessage(MESSAGE_TYPE::MODULE_DATA, addModMsg);
  return true;
}

void empty_callback(std::vector<uint8_t> data){};