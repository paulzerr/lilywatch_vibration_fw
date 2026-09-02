#include "app.h"

#include <esp_timer.h>

namespace app {

void writeLe16(uint8_t *out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
}

void writeLe32(uint8_t *out, uint32_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
  out[2] = static_cast<uint8_t>(value >> 16);
  out[3] = static_cast<uint8_t>(value >> 24);
}

uint32_t uptimeMillis() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

int16_t err16(int err) {
  if (err > INT16_MAX) {
    return INT16_MAX;
  }
  if (err < INT16_MIN) {
    return INT16_MIN;
  }
  return static_cast<int16_t>(err);
}

}  // namespace app
