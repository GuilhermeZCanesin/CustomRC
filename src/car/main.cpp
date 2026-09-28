// CustomRC - firmware do CARRO. Fase F2: link.
//
// Tarefas (como no PLANO.md):
//   control   core 1, 200 Hz  -> Arbiter -> Safety(parcial) -> Mixer
//   telemetry core 0,  10 Hz  -> bateria, perda, RSSI, estado -> controle
//   radio     callbacks       -> valida pacote, grava comando e estatisticas
//   cli       core 0          -> serial: z zera, s estatisticas, ? ajuda
//
// O que AINDA NAO existe, de proposito: saida de PWM. Os pinos dos ESCs ficam em
// LOW fixo e as larguras calculadas so vao para a serial. LEDC, arming e
// calibracao de ESC sao F3. Enquanto o link esta sendo medido, o carro nao pode
// se mover por engano.

#include <Arduino.h>
#include <atomic>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <rc_protocol.h>
#include "config.h"

using namespace rc;

namespace {

MixerConfig mixCfg;
SeqTracker  seq;

// ---- estado partilhado entre o callback do radio (core 0) e o control (core 1)
portMUX_TYPE  mux = portMUX_INITIALIZER_UNLOCKED;
ControlPacket ultimoPacote{};
volatile bool     temComando = false;
volatile uint32_t ultimoComandoUs = 0;
std::atomic<int8_t> ultimoRssi{0};

// Contadores de diagnostico. std::atomic e nao volatile: `volatile` NAO torna o
// ++ atomico, e estes sao incrementados no callback do radio (core 0) enquanto o
// laco principal le -- com volatile puro daria para perder incrementos.
std::atomic<uint32_t> rxTotal{0};        // pacotes entregues pelo radio
std::atomic<uint32_t> rxLenRuim{0};
std::atomic<uint32_t> rxInvalido{0};     // magic/ver/CRC
std::atomic<uint32_t> rxForaDeOrdem{0};
std::atomic<uint32_t> rxOutroMac{0};
std::atomic<uint32_t> txTelemOk{0}, txTelemErr{0};

uint16_t seqTelem = 0;

// A telemetria e acordada pela chegada de um pacote de controle, nao por um
// temporizador solto. Ver taskTelemetry: e o que mantem o `ackSeq` fresco e
// tira a quantizacao da medida de RTT.
TaskHandle_t hTelemetry = nullptr;

// Bateria em cache. Ler o ADC custa ~1-2 ms (16 amostras) e NAO pode ficar no
// caminho da telemetria: aquele tempo entrava inteiro no RTT medido. O control
// atualiza este valor a cada 0,5 s e a telemetria so le.
std::atomic<uint16_t> battMvCache{0};

// Escritos pelo control (core 1), lidos pelo loop e pela telemetria (core 0).
std::atomic<uint8_t> estado{static_cast<uint8_t>(CarState::Boot)};
Outputs ultimaSaida{};  // protegido por `mux`: struct nao cabe num atomico

// MAC do par, resolvido no radioInit a partir do proprio MAC.
uint8_t macPar[6] = {0};

bool mesmoMac(const uint8_t *a, const uint8_t *b) {
  for (int i = 0; i < 6; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

// ------------------------------------------------------------------- radio --

// Core 3.x: a assinatura mudou e traz esp_now_recv_info_t, que e tambem de onde
// sai o RSSI -- no 2.x era so o MAC e nao havia como ler a potencia aqui.
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  rxTotal++;

  // Aceita so o MAC pareado (PLANO). Broadcast de vizinho nao entra.
  if (!info || !info->src_addr || !mesmoMac(info->src_addr, macPar)) {
    rxOutroMac++;
    return;
  }
  if (len != static_cast<int>(sizeof(ControlPacket))) {
    rxLenRuim++;
    return;
  }

  ControlPacket p{};
  memcpy(&p, data, sizeof(p));
  if (!validate(p, static_cast<size_t>(len))) {
    rxInvalido++;
    return;
  }
  // Repetido ou atrasado nao vira comando: andar com comando velho e pior que
  // nao andar.
  if (!seq.accept(p.seq)) {
    rxForaDeOrdem++;
    return;
  }

  const int8_t rssi = info->rx_ctrl ? static_cast<int8_t>(info->rx_ctrl->rssi) : 0;

  portENTER_CRITICAL_ISR(&mux);
  ultimoPacote = p;
  ultimoComandoUs = micros();
  ultimoRssi = rssi;
  temComando = true;
  portEXIT_CRITICAL_ISR(&mux);

  // Acorda a telemetria, que decide se ja e hora de enviar (teto de 10 Hz).
  // Enviar logo apos aceitar o pacote e o que faz o `ackSeq` referir-se a um seq
  // recem-aceito, em vez de um que ficou esperando o proximo tick do temporizador
  // -- era essa espera que aparecia como ~11 ms de RTT falso.
  if (hTelemetry) {
    BaseType_t precisaTrocar = pdFALSE;
    vTaskNotifyGiveFromISR(hTelemetry, &precisaTrocar);
    if (precisaTrocar) portYIELD_FROM_ISR();
  }
}

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  if (status == ESP_NOW_SEND_SUCCESS) {
    txTelemOk++;
  } else {
    txTelemErr++;
  }
}

void radioInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(cfg::WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(80);  // 20 dBm: alcance e o que a F2 mede

  // WiFi.macAddress() devolveu 00:00:.. em teste; esp_wifi_get_mac e confiavel.
  uint8_t meuMac[6] = {0};
  esp_wifi_get_mac(WIFI_IF_STA, meuMac);

  // O par e descoberto pelo proprio MAC: trocar as placas de papel nao exige
  // editar config.h. Ver a nota em PLACA_A/PLACA_B.
  const uint8_t *par = cfg::macDoPar(meuMac);
  if (par == nullptr) {
    Serial.printf("ERRO: MAC local %02X:%02X:%02X:%02X:%02X:%02X nao esta em config.h.\n",
                  meuMac[0], meuMac[1], meuMac[2], meuMac[3], meuMac[4], meuMac[5]);
    Serial.println(F("Cadastre-o em PLACA_A/PLACA_B. Sem par, nao ha link."));
    return;
  }
  memcpy(macPar, par, 6);

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("ERRO: esp_now_init falhou"));
    return;
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, macPar, 6);
  peer.channel = cfg::WIFI_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println(F("ERRO: esp_now_add_peer falhou"));
  }

  // Ler o canal de volta, em vez de imprimir a constante: se o set_channel falhar,
  // as duas placas ficam em canais diferentes, nada passa, e um print da constante
  // continuaria afirmando que esta tudo certo.
  uint8_t canalReal = 0;
  wifi_second_chan_t seg;
  esp_wifi_get_channel(&canalReal, &seg);

  Serial.printf("Radio 20 dBm. MAC local %02X:%02X:%02X:%02X:%02X:%02X, par %02X:%02X:%02X:%02X:%02X:%02X\n",
                meuMac[0], meuMac[1], meuMac[2], meuMac[3], meuMac[4], meuMac[5],
                macPar[0], macPar[1], macPar[2], macPar[3], macPar[4], macPar[5]);
  Serial.printf("Canal: pedido %u, REAL %u%s\n", cfg::WIFI_CHANNEL, canalReal,
                canalReal == cfg::WIFI_CHANNEL ? "" : "   <<< DIVERGENTE: o link nao vai fechar");
}

// ------------------------------------------------------------ diagnostico --

// Varre redes WiFi da vizinhanca. E um teste do RECEPTOR contra o mundo real,
// fora do nosso protocolo: se esta placa nao ve nenhuma rede e a outra ve varias,
// o problema e o radio desta placa, nao o ESP-NOW nem a configuracao.
void scanWifi() {
  Serial.println(F("Varrendo WiFi... (interrompe o ESP-NOW por alguns segundos)"));
  const int n = WiFi.scanNetworks();
  if (n <= 0) {
    Serial.printf("  %d redes. Se o outro lado ve varias, o RADIO DESTA PLACA nao recebe.\n", n);
  } else {
    Serial.printf("  %d redes. Receptor OK. As mais fortes:\n", n);
    for (int i = 0; i < n && i < 5; i++) {
      Serial.printf("    %-24s canal %2d  %d dBm\n",
                    WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i));
    }
  }
  WiFi.scanDelete();

  // A varredura muda de canal; restaurar, senao o link nao volta.
  esp_wifi_set_channel(cfg::WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  uint8_t canal = 0;
  wifi_second_chan_t seg;
  esp_wifi_get_channel(&canal, &seg);
  Serial.printf("  canal restaurado: %u\n", canal);
}

// ----------------------------------------------------------------- bateria --

uint16_t lerBateriaMv() {
  uint32_t acc = 0;
  for (int i = 0; i < 16; i++) acc += analogReadMilliVolts(cfg::car::PIN_VBAT);
  const float pinoMv = acc / 16.0f;
  return static_cast<uint16_t>(pinoMv * cfg::car::VBAT_DIV_RATIO);
}

// -------------------------------------------------------------- seguranca --

// F2 mantem a regra da F0: ESC sem sinal nenhum nao arma.
void escPinsSeguros() {
  const int pinos[] = {cfg::car::PIN_ESC_REAR_LEFT, cfg::car::PIN_ESC_REAR_RIGHT,
                       cfg::car::PIN_ESC_FRONT_LEFT, cfg::car::PIN_ESC_FRONT_RIGHT,
                       cfg::car::PIN_SERVO};
  for (int p : pinos) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
}

// ----------------------------------------------------------------- tarefas --

void taskControl(void *) {
  const TickType_t periodo = pdMS_TO_TICKS(1000 / cfg::car::CONTROL_HZ);  // 5 ms
  TickType_t ultimoWake = xTaskGetTickCount();
  uint32_t ciclos = 0;
  battMvCache = lerBateriaMv();  // primeira leitura antes de qualquer telemetria

  for (;;) {
    ControlPacket p{};
    bool     valido;
    uint32_t idadeUs;

    portENTER_CRITICAL(&mux);
    p = ultimoPacote;
    valido = temComando;
    idadeUs = micros() - ultimoComandoUs;
    portEXIT_CRITICAL(&mux);

    const bool emFailsafe =
        !valido || (idadeUs / 1000u) > cfg::car::FAILSAFE_MS;

    Command cmd;
    if (!emFailsafe) {
      cmd.throttle = p.throttle;
      cmd.steering = p.steering;
      cmd.flags = p.flags;
      cmd.timestamp_ms = millis();
    }

    const Outputs saida = emFailsafe ? neutral(mixCfg) : mix(cmd, mixCfg);
    estado = static_cast<uint8_t>(emFailsafe ? CarState::Failsafe : CarState::Idle);

    portENTER_CRITICAL(&mux);
    ultimaSaida = saida;
    portEXIT_CRITICAL(&mux);

    // Bateria fora do caminho de latencia: a cada 100 ciclos = 0,5 s.
    if (++ciclos >= 100) {
      ciclos = 0;
      battMvCache = lerBateriaMv();
    }

    // F3: aqui entram as escritas de LEDC.

    vTaskDelayUntil(&ultimoWake, periodo);
  }
}

void taskTelemetry(void *) {
  uint32_t ultimoEnvioMs = 0;

  for (;;) {
    // Dorme ate um pacote de controle ser aceito. O timeout de 200 ms garante que
    // a telemetria continue saindo (a ~5 Hz) mesmo sem link, para o controle saber
    // que o carro esta vivo.
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));

    const uint32_t agora = millis();
    if (agora - ultimoEnvioMs < 100) continue;  // teto de 10 Hz
    ultimoEnvioMs = agora;

    TelemetryPacket t{};
    t.seq = seqTelem++;
    // O ack e o que permite ao controle medir o RTT real.
    t.ackSeq = seq.last();
    t.batt_mV = battMvCache.load();  // cache: ler o ADC aqui inflaria o RTT
    t.rssi = ultimoRssi.load();
    t.lossPct = seq.lossPct();
    t.state = estado.load();
    finalize(t);

    esp_now_send(macPar, reinterpret_cast<const uint8_t *>(&t), sizeof(t));
  }
}

void imprimeEstatisticas() {
  Serial.println(F("--- carro: link ---"));
  Serial.printf("  estado      : %s\n",
                static_cast<CarState>(estado.load()) == CarState::Failsafe ? "FAILSAFE"
                : static_cast<CarState>(estado.load()) == CarState::Idle   ? "idle (recebendo)"
                                             : "boot");
  Serial.printf("  rx aceitos  : %lu   perdidos: %lu   perda: %u%%   resyncs: %lu\n",
                (unsigned long)seq.accepted(), (unsigned long)seq.lost(), seq.lossPct(),
                (unsigned long)seq.resyncs());
  Serial.printf("  descartados : len %lu | invalido %lu | fora de ordem %lu | outro MAC %lu\n",
                (unsigned long)rxLenRuim, (unsigned long)rxInvalido,
                (unsigned long)rxForaDeOrdem, (unsigned long)rxOutroMac);
  Serial.printf("  RSSI        : %d dBm\n", (int)ultimoRssi.load());
  Serial.printf("  bateria     : %u mV (%.2f V/celula)\n",
                battMvCache.load(), battMvCache.load() / 1000.0f / cfg::car::VBAT_CELLS);
  Serial.printf("  telemetria  : tx %lu ok / %lu erro\n",
                (unsigned long)txTelemOk, (unsigned long)txTelemErr);
  Serial.printf("  saidas us   : RL/RR %u/%u  FL/FR %u/%u  SRV %u  (PWM desligado na F2)\n",
                ultimaSaida.rear_left_us, ultimaSaida.rear_right_us,
                ultimaSaida.front_left_us, ultimaSaida.front_right_us,
                ultimaSaida.steering_us);
}

void trataSerial() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'z':
        seq.reset();
        rxTotal = rxLenRuim = rxInvalido = rxForaDeOrdem = rxOutroMac = 0;
        txTelemOk = txTelemErr = 0;
        Serial.println(F("Contadores zerados"));
        break;
      case 's': imprimeEstatisticas(); break;
      case 'w': scanWifi(); break;
      case '?': Serial.println(F("Comandos: z=zerar | s=estatisticas | ?=ajuda")); break;
      default: break;
    }
  }
}

}  // namespace

void setup() {
  escPinsSeguros();  // antes de qualquer outra coisa
  Serial.begin(cfg::SERIAL_BAUD);
  delay(300);

  Serial.println();
  Serial.println(F("=== CustomRC CARRO - F2 link ==="));
  Serial.println(F("PWM DESLIGADO nesta fase: ESCs em LOW, larguras so na serial."));
  Serial.println(F("Comandos: z=zerar | s=estatisticas | ?=ajuda"));

  radioInit();

  // control no core 1, sozinho: o radio e o WiFi vivem no core 0 e nao podem
  // atrasar o laco de controle.
  xTaskCreatePinnedToCore(taskControl, "control", 4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(taskTelemetry, "telemetry", 4096, nullptr, 3, &hTelemetry, 0);
}

void loop() {
  static uint32_t ultimoPrint = 0;
  trataSerial();

  const uint32_t agora = millis();
  if (agora - ultimoPrint >= 1000) {
    ultimoPrint = agora;
    Serial.printf("%6lus | %s | rx %lu perda %u%% rs%lu | RSSI %d | thr %5d str %5d | RL %u SRV %u\n",
                  (unsigned long)(agora / 1000),
                  static_cast<CarState>(estado.load()) == CarState::Failsafe ? "FAILSAFE" : "idle    ",
                  (unsigned long)seq.accepted(), seq.lossPct(), (unsigned long)seq.resyncs(), (int)ultimoRssi.load(),
                  ultimoPacote.throttle, ultimoPacote.steering,
                  ultimaSaida.rear_left_us, ultimaSaida.steering_us);
  }
  delay(10);
}
