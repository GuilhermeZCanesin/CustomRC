#include "link.h"

namespace rc {

void SeqTracker::reset() {
  last_ = 0;
  started_ = false;
  accepted_ = rejected_ = lost_ = 0;
  rejeicoesSeguidas_ = 0;
  resyncs_ = 0;
}

bool SeqTracker::accept(uint16_t seq) {
  if (!started_) {
    started_ = true;
    last_ = seq;
    accepted_ = 1;
    return true;
  }

  // Diferenca com sinal: positiva = avanco, <= 0 = repetido ou atrasado.
  const int16_t delta = static_cast<int16_t>(seq - last_);
  if (delta <= 0) {
    rejected_++;
    // Rejeicoes consecutivas o bastante = o transmissor reiniciou. Ressincroniza
    // em vez de ficar surdo esperando o contador antigo ser alcancado.
    if (++rejeicoesSeguidas_ >= REJEICOES_PARA_RESYNC) {
      rejeicoesSeguidas_ = 0;
      resyncs_++;
      last_ = seq;
      accepted_++;
      return true;
    }
    return false;
  }
  rejeicoesSeguidas_ = 0;

  // delta == 1 e a sequencia perfeita; o que passar disso sumiu no ar.
  lost_ += static_cast<uint32_t>(delta - 1);
  last_ = seq;
  accepted_++;
  return true;
}

uint8_t SeqTracker::lossPct() const {
  const uint32_t esperado = accepted_ + lost_;
  if (esperado == 0) return 0;
  return static_cast<uint8_t>((lost_ * 100 + esperado / 2) / esperado);
}

}  // namespace rc
