#pragma once
#include <stdint.h>
#include "packets.h"

namespace rc {

// Comando normalizado que circula entre Arbiter -> Safety -> Mixer.
struct Command {
  int16_t  throttle = 0;      // AXIS_MIN..AXIS_MAX
  int16_t  steering = 0;      // AXIS_MIN..AXIS_MAX
  uint8_t  flags = 0;
  uint32_t timestamp_ms = 0;
};

struct MixerConfig {
  // Janela de PWM. 1000/1500/2000 us e o padrao de ESC e servo.
  uint16_t us_min = 1000;
  uint16_t us_mid = 1500;
  uint16_t us_max = 2000;

  uint8_t expo_throttle = 0;   // 0..100 (%)
  uint8_t expo_steering = 0;   // 0..100 (%)

  // Limite do curso de re, em % -- a traseira e bidirecional e a re nao precisa
  // (nem deve) ter a mesma autoridade da frente.
  uint8_t reverse_limit = 100;

  // Dianteira em % da traseira. E o fator que a F6 vai ajustar para nao haver
  // arrasto entre os eixos.
  uint8_t front_factor = 100;

  int16_t steer_trim = 0;      // somado ao eixo, AXIS_MIN..AXIS_MAX
  uint8_t steer_endpoint = 100;// % do curso do servo
  bool    steer_invert = false;
};

struct Outputs {
  uint16_t rear_left_us;
  uint16_t rear_right_us;
  uint16_t front_left_us;
  uint16_t front_right_us;
  uint16_t steering_us;
};

// Curva expo de RC, em inteiros (sem float: deterministico e barato).
// expo=0 -> linear. expo=100 -> cubica pura. Preserva o sinal.
int32_t apply_expo(int32_t v, uint8_t expo);

// Converte um Command em larguras de pulso. Funcao pura: nao toca em hardware,
// nao le relogio, nao decide seguranca (isso e do Safety, na F3).
Outputs mix(const Command &cmd, const MixerConfig &cfg);

// Estado de failsafe do PLANO: traseira em neutro, dianteira no minimo (ela e
// unidirecional: 1000 us = parada), direcao centrada.
Outputs neutral(const MixerConfig &cfg);

}  // namespace rc
