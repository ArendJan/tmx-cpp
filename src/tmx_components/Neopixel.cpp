
#include "tmx_cpp/tmx.hpp"
#include "tmx_cpp/tmx_util.hpp"
using namespace tmx_cpp;

// neopixel
bool TMX::attach_neopixel(uint8_t pin, uint8_t num_pixels, uint8_t type,
                          std::tuple<uint8_t, uint8_t, uint8_t> color) {
  if (this->neopixel_attached) {
    std::cout << "Neopixel already attached, only one supported" << std::endl;
    return false;
  }
  auto feature = this->get_feature(MESSAGE_TYPE::INITIALIZE_NEO_PIXELS);
  if (!feature.first) {
    std::cout << "Neopixels not supported by hw" << std::endl;
    return false;
  }

  if (num_pixels == 0 || num_pixels > 255) {
    std::cout << "num_pixels out of range (1-255)" << std::endl;
    return false;
  }
  auto message = std::vector<uint8_t>{pin, num_pixels, type};
  auto [r, g, b] = color;
  auto l = {r, g, b};
  if (std::any_of(l.begin(), l.end(), [](uint8_t c) { return c > 255; })) {
    std::cout << "color values must be in range 0-255" << std::endl;
    return false;
  }
  message.push_back(r);
  message.push_back(g);
  message.push_back(b);

  this->sendMessage(MESSAGE_TYPE::INITIALIZE_NEO_PIXELS, message);

  this->neopixel_pin = pin;
  this->neopixel_len = num_pixels;
  this->neopixel_attached = true;
  return true;
}
bool TMX::set_neopixel_color(uint8_t pixel_num,
                             std::tuple<uint8_t, uint8_t, uint8_t> color,
                             bool autoshow) {
  if (!this->neopixel_attached) {
    std::cout << "Neopixel not attached" << std::endl;
    return false;
  }
  if (pixel_num >= this->neopixel_len) {
    std::cout << "Pixel number out of range" << std::endl;
    return false;
  }
  auto [r, g, b] = color;
  auto l = {r, g, b};
  if (std::any_of(l.begin(), l.end(), [](uint8_t c) { return c > 255; })) {
    std::cout << "color values must be in range 0-255" << std::endl;
    return false;
  }

  std::vector<uint8_t> message = {pixel_num, r, g, b, autoshow ? 1_uc : 0_uc};
  this->sendMessage(MESSAGE_TYPE::SET_NEO_PIXEL, message);
  return true;
}
bool TMX::set_neopixel_color(
    std::vector<std::pair<uint8_t, std::tuple<uint8_t, uint8_t, uint8_t>>>
        pixel_colors,
    bool autoshow) {
  for (auto &[pixel_num, color] : pixel_colors) {
    auto ok = this->set_neopixel_color(pixel_num, color, false);
    if (!ok) {
      return false;
    }
  }
  if (autoshow) {
    this->show_neopixels();
  }
  return true;
}
bool TMX::fill_neopixels(std::tuple<uint8_t, uint8_t, uint8_t> color,
                         bool autoshow) {
  if (!this->neopixel_attached) {
    std::cout << "Neopixel not attached" << std::endl;
    return false;
  }
  auto [r, g, b] = color;
  auto l = {r, g, b};
  if (std::any_of(l.begin(), l.end(), [](uint8_t c) { return c > 255; })) {
    std::cout << "color values must be in range 0-255" << std::endl;
    return false;
  }
  this->sendMessage(MESSAGE_TYPE::FILL_NEO_PIXELS,
                    {r, g, b, autoshow ? 1_uc : 0_uc});
  return true;
}
bool TMX::clear_neopixels() {
  if (!this->neopixel_attached) {
    std::cout << "Neopixel not attached" << std::endl;
    return false;
  }
  this->sendMessage(MESSAGE_TYPE::CLEAR_ALL_NEO_PIXELS, {});
  return true;
}
bool TMX::show_neopixels() {
  if (!this->neopixel_attached) {
    std::cout << "Neopixel not attached" << std::endl;
    return false;
  }
  this->sendMessage(MESSAGE_TYPE::SHOW_NEO_PIXELS, {});
  return true;
}
// private:
// uint8_t neopixel_len = 0;
// uint8_t neopixel_pin = 0;
// bool neopixel_attached = false;
