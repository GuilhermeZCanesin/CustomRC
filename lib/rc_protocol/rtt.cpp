#include "rtt.h"

namespace rc {

void RttTracker::reset() {
  for (size_t i = 0; i < SLOTS; i++) {
    slots_[i].seq = 0;
    slots_[i].t_us = 0;
    slots_[i].used = false;
  }
  last_us_ = 0;
  min_us_ = 0xFFFFFFFFu;
  max_us_ = 0;
  soma_us_ = 0;
  samples_ = 0;
  unmatched_ = 0;
}

void RttTracker::on_send(uint16_t seq, uint32_t t_us) {
  Slot &s = slots_[seq & (SLOTS - 1)];
  s.seq = seq;
  s.t_us = t_us;
  s.used = true;
}

bool RttTracker::on_ack(uint16_t seq, uint32_t t_us, uint32_t &rtt_us) {
  Slot &s = slots_[seq & (SLOTS - 1)];

  // Guardar o seq junto do tempo e o que permite detectar slot sobrescrito:
  // sem isso, um ack antigo casaria com o envio de outro seq e produziria um
  // RTT inventado.
  if (!s.used || s.seq != seq) {
    unmatched_++;
    return false;
  }

  // Subtracao sem sinal: correta mesmo com o wraparound de micros().
  rtt_us = t_us - s.t_us;
  s.used = false;  // consome o slot: um envio gera no maximo uma amostra

  last_us_ = rtt_us;
  if (rtt_us < min_us_) min_us_ = rtt_us;
  if (rtt_us > max_us_) max_us_ = rtt_us;
  soma_us_ += rtt_us;
  samples_++;
  return true;
}

uint32_t RttTracker::avg_us() const {
  if (samples_ == 0) return 0;
  return static_cast<uint32_t>(soma_us_ / samples_);
}

}  // namespace rc
