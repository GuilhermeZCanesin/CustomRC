#include "crc16.h"

namespace rc {

uint16_t crc16(const void *data, size_t len) {
  const uint8_t *p = static_cast<const uint8_t *>(data);
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= static_cast<uint16_t>(p[i]) << 8;
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

}  // namespace rc
