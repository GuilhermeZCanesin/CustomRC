#include "packets.h"
#include "crc16.h"

namespace rc {
namespace {

// O CRC e sempre o ultimo campo, entao cobre exatamente sizeof(T) - 2 bytes.
template <typename T>
inline size_t crc_span() {
  return sizeof(T) - sizeof(uint16_t);
}

template <typename T>
inline void finalize_impl(T &p, uint8_t magic) {
  p.magic = magic;
  p.ver   = PROTO_VERSION;
  p.crc   = crc16(&p, crc_span<T>());
}

template <typename T>
inline bool validate_impl(const T &p, size_t len, uint8_t magic) {
  if (len != sizeof(T))          return false;
  if (p.magic != magic)          return false;
  if (p.ver != PROTO_VERSION)    return false;
  return p.crc == crc16(&p, crc_span<T>());
}

}  // namespace

void finalize(ControlPacket &p)   { finalize_impl(p, MAGIC_CONTROL); }
void finalize(TelemetryPacket &p) { finalize_impl(p, MAGIC_TELEMETRY); }
void finalize(PairPacket &p)      { finalize_impl(p, MAGIC_PAIR); }

bool validate(const ControlPacket &p, size_t len)   { return validate_impl(p, len, MAGIC_CONTROL); }
bool validate(const TelemetryPacket &p, size_t len) { return validate_impl(p, len, MAGIC_TELEMETRY); }
bool validate(const PairPacket &p, size_t len)      { return validate_impl(p, len, MAGIC_PAIR); }

int16_t clamp_axis(int32_t v) {
  if (v < AXIS_MIN) return AXIS_MIN;
  if (v > AXIS_MAX) return AXIS_MAX;
  return static_cast<int16_t>(v);
}

}  // namespace rc
