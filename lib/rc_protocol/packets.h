#pragma once
#include <stdint.h>
#include <stddef.h>

namespace rc {

// Versao do protocolo. Incrementar quebra compatibilidade: o receptor descarta
// pacotes com `ver` diferente, de proposito -- e melhor nao andar do que andar
// interpretando campos errados.
constexpr uint8_t PROTO_VERSION = 1;

// Magic distinto por tipo. O PLANO discrimina os pacotes por tamanho; um magic
// proprio por tipo e mais barato e mais seguro que so confiar no `len`, porque
// dois tipos podem coincidir em tamanho no futuro.
constexpr uint8_t MAGIC_CONTROL   = 0xC5;
constexpr uint8_t MAGIC_TELEMETRY = 0x7E;
constexpr uint8_t MAGIC_PAIR      = 0x9A;

// Bits de `flags` do ControlPacket.
namespace Flag {
constexpr uint8_t Arm   = 1 << 0;  // pedido de armar, validado pelo Safety (F3)
constexpr uint8_t Boost = 1 << 1;
constexpr uint8_t Mode  = 1 << 2;  // reservado
}  // namespace Flag

// Papel anunciado no pareamento.
enum class Role : uint8_t { Car = 1, Remote = 2 };

// Vai no campo `state` da telemetria. Armed so passa a existir na F3, quando o
// Safety e o PWM entrarem; na F2 o carro alterna entre Failsafe e Idle.
enum class CarState : uint8_t {
  Boot = 0,
  Failsafe = 1,   // sem comando valido dentro da janela
  Idle = 2,       // recebendo comandos, saidas em neutro
  Armed = 3,      // F3
};

// Faixa canonica dos eixos em todo o projeto.
constexpr int16_t AXIS_MIN = -1000;
constexpr int16_t AXIS_MAX = 1000;

#pragma pack(push, 1)

// controle -> carro, 100 Hz.
struct ControlPacket {
  uint8_t  magic;
  uint8_t  ver;
  uint16_t seq;
  uint8_t  flags;
  int16_t  throttle;   // AXIS_MIN..AXIS_MAX
  int16_t  steering;   // AXIS_MIN..AXIS_MAX
  uint8_t  aux;
  uint16_t crc;        // sempre o ultimo campo: cobre tudo que vem antes
};

// carro -> controle, 10 Hz.
struct TelemetryPacket {
  uint8_t  magic;
  uint8_t  ver;
  uint16_t seq;
  uint16_t ackSeq;     // ultimo seq de controle aceito: permite medir RTT real
  uint16_t batt_mV;
  int8_t   rssi;
  uint8_t  lossPct;
  uint8_t  state;
  uint16_t crc;
};

// broadcast, pareamento por botao (pos-F1).
struct PairPacket {
  uint8_t  magic;
  uint8_t  ver;
  uint8_t  role;       // Role
  uint16_t crc;
};

#pragma pack(pop)

// NOTA DE DIVERGENCIA COM O PLANO.md
// A tabela do PLANO lista ControlPacket com 13 B e TelemetryPacket com 14 B.
// Somando os campos que a propria tabela enumera, com `packed`, da 12 B e 13 B:
// cada um esta 1 byte a menos que o anunciado. O PairPacket, com o mesmo metodo,
// da 5 B e bate com o plano -- o que sugere erro de conta na tabela, e nao campo
// faltando. Os campos sao a fonte da verdade; os static_assert abaixo travam os
// tamanhos reais para que qualquer mudanca futura quebre o build em vez de
// quebrar o link silenciosamente.
static_assert(sizeof(ControlPacket)   == 12, "ControlPacket deve ter 12 bytes");
static_assert(sizeof(TelemetryPacket) == 13, "TelemetryPacket deve ter 13 bytes");
static_assert(sizeof(PairPacket)      ==  5, "PairPacket deve ter 5 bytes");

// Preenche magic/ver e calcula o CRC. Chamar sempre antes de enviar.
void finalize(ControlPacket &p);
void finalize(TelemetryPacket &p);
void finalize(PairPacket &p);

// Confere tamanho, magic, ver e CRC. `len` e o tamanho recebido pelo radio.
// Retorna false para qualquer inconsistencia -- o chamador descarta.
bool validate(const ControlPacket &p, size_t len);
bool validate(const TelemetryPacket &p, size_t len);
bool validate(const PairPacket &p, size_t len);

// Limita um eixo a AXIS_MIN..AXIS_MAX.
int16_t clamp_axis(int32_t v);

}  // namespace rc
