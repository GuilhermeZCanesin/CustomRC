#pragma once
#include <stdint.h>

namespace rc {

// Contabiliza a sequencia dos pacotes recebidos: aceita, descarta atrasados e
// duplicados, e estima a perda. Fica aqui (e nao no firmware) para poder ser
// testado no PC -- e a metrica que a F2 usa como criterio de aceitacao.
//
// A comparacao e feita na diferenca com sinal de 16 bits, entao o wraparound de
// 65535 -> 0 e tratado como avanco normal, sem caso especial.
class SeqTracker {
 public:
  void reset();

  // true se o pacote e mais novo que o ultimo aceito. false para repetido ou
  // atrasado (chegou fora de ordem) -- esses nao devem virar comando.
  bool accept(uint16_t seq);

  uint32_t accepted() const { return accepted_; }
  uint32_t rejected() const { return rejected_; }

  // Quantos seq faltaram no meio do caminho.
  uint32_t lost() const { return lost_; }

  // Perda em % sobre o esperado (aceitos + perdidos). 0 quando nada chegou.
  uint8_t lossPct() const;

  uint16_t last() const { return last_; }
  bool started() const { return started_; }

  // Quantas vezes o tracker concluiu que o transmissor reiniciou.
  uint32_t resyncs() const { return resyncs_; }

  // Depois desta quantidade de rejeicoes CONSECUTIVAS, o tracker assume que o
  // transmissor reiniciou (o seq voltou para tras e nao vai voltar) e
  // ressincroniza no seq que estiver chegando.
  //
  // Por que isso e necessario: sem ressincronizar, um reset do controle (troca de
  // bateria, por exemplo) faz todo pacote novo parecer "atrasado", porque o seq
  // reinicia em 0 enquanto o carro espera um valor alto. O carro ficaria surdo
  // ate o contador alcancar o valor antigo -- a 100 Hz, ate ~11 minutos.
  //
  // 10 pacotes a 100 Hz = 0,1 s de failsafe, bem abaixo dos 200 ms da janela.
  // Um punhado de pacotes fora de ordem nao dispara o resync; um reset, sim.
  static constexpr uint32_t REJEICOES_PARA_RESYNC = 10;

 private:
  uint16_t last_ = 0;
  bool     started_ = false;
  uint32_t accepted_ = 0;
  uint32_t rejected_ = 0;
  uint32_t lost_ = 0;
  uint32_t rejeicoesSeguidas_ = 0;
  uint32_t resyncs_ = 0;
};

}  // namespace rc
