#include "mixer.h"

namespace rc {
namespace {

inline uint16_t clamp_us(int32_t us, const MixerConfig &cfg) {
  if (us < cfg.us_min) return cfg.us_min;
  if (us > cfg.us_max) return cfg.us_max;
  return static_cast<uint16_t>(us);
}

inline uint8_t clamp_pct(uint8_t v) { return v > 100 ? 100 : v; }

}  // namespace

int32_t apply_expo(int32_t v, uint8_t expo) {
  if (expo == 0) return v;
  expo = clamp_pct(expo);

  const int32_t sign = (v < 0) ? -1 : 1;
  int32_t a = (v < 0) ? -v : v;
  if (a > AXIS_MAX) a = AXIS_MAX;

  // linear*(100-e) + cubica*e, tudo sobre 100.
  const int32_t linear = a * (100 - expo) / 100;
  const int32_t cubica = (a * a / AXIS_MAX) * a / AXIS_MAX * expo / 100;
  return sign * (linear + cubica);
}

Outputs neutral(const MixerConfig &cfg) {
  Outputs o;
  o.rear_left_us  = cfg.us_mid;
  o.rear_right_us = cfg.us_mid;
  // Dianteira e unidirecional: o "parado" dela e us_min, nao us_mid.
  o.front_left_us  = cfg.us_min;
  o.front_right_us = cfg.us_min;
  o.steering_us    = cfg.us_mid;
  return o;
}

Outputs mix(const Command &cmd, const MixerConfig &cfg) {
  Outputs o = neutral(cfg);

  const int32_t thr = apply_expo(clamp_axis(cmd.throttle), cfg.expo_throttle);

  // --- Traseira: bidirecional, centrada em us_mid ---
  if (thr >= 0) {
    const int32_t span = cfg.us_max - cfg.us_mid;
    o.rear_left_us = o.rear_right_us =
        clamp_us(cfg.us_mid + thr * span / AXIS_MAX, cfg);
  } else {
    const int32_t span = static_cast<int32_t>(cfg.us_mid - cfg.us_min) *
                         clamp_pct(cfg.reverse_limit) / 100;
    o.rear_left_us = o.rear_right_us =
        clamp_us(cfg.us_mid + thr * span / AXIS_MAX, cfg);
  }

  // --- Dianteira: unidirecional, so acelera para a frente ---
  // Em re ou freio fica em us_min, como manda a eletrica (ESC 30A sem re).
  if (thr > 0) {
    const int32_t span = static_cast<int32_t>(cfg.us_max - cfg.us_min) *
                         clamp_pct(cfg.front_factor) / 100;
    o.front_left_us = o.front_right_us =
        clamp_us(cfg.us_min + thr * span / AXIS_MAX, cfg);
  }

  // --- Direcao: trim antes do expo, depois endpoint ---
  int32_t str = clamp_axis(static_cast<int32_t>(cmd.steering) + cfg.steer_trim);
  str = apply_expo(str, cfg.expo_steering);
  if (cfg.steer_invert) str = -str;

  const int32_t span = static_cast<int32_t>(cfg.us_max - cfg.us_mid) *
                       clamp_pct(cfg.steer_endpoint) / 100;
  o.steering_us = clamp_us(cfg.us_mid + str * span / AXIS_MAX, cfg);

  return o;
}

}  // namespace rc
