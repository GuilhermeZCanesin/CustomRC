#pragma once
#include <stdint.h>
#include <stddef.h>

namespace rc {

// Mede o round-trip real: o controle anota quando enviou cada seq, e a
// telemetria do carro devolve `ackSeq` com o ultimo seq que ele aceitou.
// RTT = agora - instante_do_envio(ackSeq).
//
// Fica na lib (e nao no firmware) para ser testado no PC: e a metrica que a F2
// usa como criterio de aceitacao (< 5 ms).
//
// Os tempos sao em microssegundos. A subtracao sem sinal ja trata o wraparound
// de micros(), que no ESP32 acontece a cada ~71,6 minutos -- nenhum caso
// especial e necessario, mas ha teste cobrindo isso.
class RttTracker {
 public:
  // Potencia de 2: o indice sai por mascara. Com envio a 100 Hz e telemetria a
  // 10 Hz, o ack chega ~10 pacotes depois; 256 slots e folga enorme.
  static constexpr size_t SLOTS = 256;

  RttTracker() { reset(); }

  void reset();

  // Registra o instante de envio de um seq.
  void on_send(uint16_t seq, uint32_t t_us);

  // Casa um ack com o envio correspondente. Retorna false (e nao conta amostra)
  // se aquele seq nao esta mais registrado -- ack antigo, ou slot sobrescrito.
  bool on_ack(uint16_t seq, uint32_t t_us, uint32_t &rtt_us);

  uint32_t last_us() const { return last_us_; }
  uint32_t min_us() const { return samples_ ? min_us_ : 0; }
  uint32_t max_us() const { return max_us_; }
  uint32_t avg_us() const;
  uint32_t samples() const { return samples_; }
  uint32_t unmatched() const { return unmatched_; }

 private:
  struct Slot {
    uint16_t seq;
    uint32_t t_us;
    bool     used;
  };

  Slot     slots_[SLOTS];
  uint32_t last_us_;
  uint32_t min_us_;
  uint32_t max_us_;
  uint64_t soma_us_;
  uint32_t samples_;
  uint32_t unmatched_;
};

}  // namespace rc
