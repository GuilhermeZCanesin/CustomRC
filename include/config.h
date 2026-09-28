#pragma once
#include <stdint.h>

// Configuracao de hardware das duas placas. Numeros medidos na F0 (24-27/09/2026);
// ver a secao "Pinagem" e os resultados da F0 no CLAUDE.md.

namespace cfg {

// As duas placas conhecidas, listadas por IDENTIDADE e nao por papel.
//
// O papel de cada placa e definido pelo firmware que ela recebe (ambiente `car` ou
// `remote`), e o par e descoberto em tempo de execucao: e a OUTRA placa da lista.
//
// Por que assim: com MAC_CAR/MAC_REMOTE fixos, trocar as placas de papel exigia
// editar este arquivo, e um engano fazia a placa transmitir para o proprio MAC --
// o link simplesmente nao fechava, sem nenhuma mensagem de erro apontando a causa.
// Agora inverter papeis e so gravar o outro ambiente.
//
// Placa nova: substitua o MAC da que saiu. Nada mais precisa mudar.
constexpr uint8_t PLACA_A[6] = {0xF8, 0xB3, 0xB7, 0x22, 0x1D, 0x94};
// Substituiu a f4:65:0b:48:02:2c, cuja flash morreu com os 9 V no VIN (28/09/2026).
constexpr uint8_t PLACA_B[6] = {0x3C, 0x8A, 0x1F, 0x63, 0x5E, 0x6C};

// Devolve o MAC do par (a outra placa da lista), ou nullptr se o MAC local nao
// estiver cadastrado acima -- caso em que o firmware avisa em vez de falhar calado.
// Funcao pura: recebe o proprio MAC, para nao arrastar esp_wifi.h para este header.
inline const uint8_t *macDoPar(const uint8_t *meuMac) {
  bool ehA = true, ehB = true;
  for (int i = 0; i < 6; i++) {
    if (meuMac[i] != PLACA_A[i]) ehA = false;
    if (meuMac[i] != PLACA_B[i]) ehB = false;
  }
  if (ehA) return PLACA_B;
  if (ehB) return PLACA_A;
  return nullptr;
}

// Canal fixo nos dois lados: sem isso o ESP-NOW nao fecha o link.
constexpr uint8_t WIFI_CHANNEL = 1;

constexpr uint32_t SERIAL_BAUD = 115200;

// ------------------------------------------------------------------ Carro --
namespace car {

constexpr int PIN_ESC_REAR_LEFT   = 27;
constexpr int PIN_ESC_REAR_RIGHT  = 26;
constexpr int PIN_ESC_FRONT_LEFT  = 32;
constexpr int PIN_ESC_FRONT_RIGHT = 33;
constexpr int PIN_SERVO           = 25;
constexpr int PIN_VBAT            = 36;
constexpr int PIN_SDA             = 21;
constexpr int PIN_SCL             = 22;
constexpr int PIN_LED             = 2;

// Divisor 100k/22k medido na F0: serial 15,47 V contra 15,39 V no multimetro.
constexpr float VBAT_DIV_RATIO = (100.0f + 22.0f) / 22.0f;  // 5,545
constexpr int   VBAT_CELLS     = 4;

constexpr uint32_t CONTROL_HZ   = 200;  // laco Arbiter->Safety->Mixer->Outputs
constexpr uint32_t FAILSAFE_MS  = 200;  // sem comando valido por isso -> neutro

}  // namespace car

// --------------------------------------------------------------- Controle --
namespace remote {

constexpr int PIN_AXIS_THROTTLE = 35;  // VRx do joystick de aceleracao
constexpr int PIN_AXIS_STEERING = 34;  // VRx do joystick de direcao
constexpr int PIN_TRIM_ACC_DOWN = 5;   // strapping: nao segurar durante o boot
constexpr int PIN_TRIM_ACC_UP   = 18;
constexpr int PIN_TRIM_STR_UP   = 19;
constexpr int PIN_TRIM_STR_DOWN = 23;
constexpr int PIN_BTN_ARM       = 32;  // previsto, ainda sem fio
constexpr int PIN_BTN_MENU      = 33;  // previsto, ainda sem fio
constexpr int PIN_SDA           = 21;
constexpr int PIN_SCL           = 22;

constexpr uint32_t SEND_HZ  = 100;
constexpr uint32_t INPUT_HZ = 200;

// Limites reais do ADC medidos na F0. O conversor satura MUITO antes dos 3,3 V:
// trava em 4095 ja aos 3129 mV e tem zona morta abaixo de ~142 mV. A calibracao
// da F4 tem que mapear ESTES pontos, nao o fim de curso mecanico do joystick --
// senao trabalha com uma faixa que o ADC nunca entrega.
constexpr int ADC_FLOOR_MV   = 142;
constexpr int ADC_CEILING_MV = 3129;

// Repouso medido na F0, em contagens brutas.
constexpr int THROTTLE_CENTER_RAW = 1852;
constexpr int STEERING_CENTER_RAW = 1801;

}  // namespace remote
}  // namespace cfg
