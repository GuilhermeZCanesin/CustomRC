// CustomRC - firmware do CONTROLE. Fase F2: link.
//
// Tarefas:
//   input core 1, 200 Hz -> ADC com oversampling, mapeamento para -1000..1000
//   send  core 1, 100 Hz -> ControlPacket com vTaskDelayUntil
//   radio callbacks      -> telemetria do carro; ack fecha a conta de RTT
//   cli   core 0         -> serial: z zera, s estatisticas, ? ajuda
//
// Aqui e onde o critério da F2 e lido: perda < 1% e RTT < 5 ms.
// O RTT sai do par envio/ack: cada seq enviado e anotado com micros(), e a
// telemetria devolve `ackSeq` com o ultimo seq que o carro aceitou.
//
// AINDA NAO existe: calibracao em NVS, deadzone, expo e OLED -- tudo F4. Os
// eixos usam o centro medido na F0 como valor fixo, so para haver o que enviar.

#include <Arduino.h>
#include <atomic>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <Wire.h>

#include <rc_protocol.h>
#include "config.h"

using namespace rc;

namespace {

// O RttTracker e mutado de DOIS contextos: on_send pela task de envio (core 1) e
// on_ack pelo callback do radio (core 0). Sem trava, os dois podem escrever o
// mesmo slot e os acumuladores de estatistica sairiam rasgados. Daqui em diante,
// todo acesso ao `rtt` passa por `mux`.
RttTracker rtt;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// std::atomic e nao volatile: `volatile` nao torna o ++ atomico, e estes
// contadores sao tocados pelos callbacks do radio enquanto o loop le.
std::atomic<int16_t> eixoThrottle{0};
std::atomic<int16_t> eixoSteering{0};

uint16_t seqAtual = 0;
std::atomic<uint32_t> txOk{0}, txErr{0};

// Ultima telemetria recebida do carro.
std::atomic<bool>     temTelemetria{false};
std::atomic<uint32_t> ultimaTelemetriaMs{0};
std::atomic<uint16_t> telemBattMv{0};
std::atomic<int8_t>   telemRssi{0};    // RSSI que o CARRO ve do controle
std::atomic<uint8_t>  telemLossPct{0};
std::atomic<uint8_t>  telemState{0};
std::atomic<int8_t>   rssiLocal{0};    // RSSI que o CONTROLE ve do carro
std::atomic<uint32_t> telemRx{0}, telemInvalida{0};

// --- metricas de ALCANCE (teste de 50 m) ---
// Num teste de alcance o RTT medio nao e o que denuncia o limite: e a queda de
// link. Por isso rastreamos o pior RSSI, o maior intervalo sem telemetria e
// quantas vezes o link sumiu por mais de 500 ms.
std::atomic<int8_t>   rssiMin{0};
std::atomic<uint32_t> maiorGapMs{0};
std::atomic<uint32_t> quedas{0};

// Modo campo: grava as metricas na NVS a cada 3 s, para sobreviverem ao reset
// que a abertura da porta serial provoca. Mesmo truque do contador de brownout
// da F0. So liga sob comando, para nao desgastar a flash sem necessidade.
Preferences prefs;
std::atomic<bool>     modoCampo{false};
std::atomic<uint32_t> campoInicioMs{0};

struct RegistroCampo {
  uint32_t duracaoMs;
  uint32_t enviados, semAck, telemRecebidas;
  uint32_t rttMin, rttMed, rttMax, rttAmostras;
  int8_t   rssiMin, rssiUlt, rssiNoCarro;
  uint8_t  perdaCarro;
  uint32_t quedas, maiorGapMs;
};

// Foto consistente das estatisticas de RTT, tirada sob a trava.
struct RttSnapshot {
  uint32_t last_us, min_us, avg_us, max_us, samples, unmatched;
};

RttSnapshot rttSnapshot() {
  portENTER_CRITICAL(&mux);
  const RttSnapshot r{rtt.last_us(), rtt.min_us(), rtt.avg_us(),
                      rtt.max_us(), rtt.samples(), rtt.unmatched()};
  portEXIT_CRITICAL(&mux);
  return r;
}

struct Trim {
  int pin;
  const char *nome;
  bool anterior;
};

Trim trims[] = {
    {cfg::remote::PIN_TRIM_ACC_DOWN, "ACC-", true},
    {cfg::remote::PIN_TRIM_ACC_UP, "ACC+", true},
    {cfg::remote::PIN_TRIM_STR_UP, "STR+", true},
    {cfg::remote::PIN_TRIM_STR_DOWN, "STR-", true},
};

// MAC do par, resolvido no radioInit a partir do proprio MAC.
uint8_t macPar[6] = {0};

bool mesmoMac(const uint8_t *a, const uint8_t *b) {
  for (int i = 0; i < 6; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

// ------------------------------------------------------------------- radio --

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !info->src_addr || !mesmoMac(info->src_addr, macPar)) return;
  if (len != static_cast<int>(sizeof(TelemetryPacket))) {
    telemInvalida++;
    return;
  }

  TelemetryPacket t{};
  memcpy(&t, data, sizeof(t));
  if (!validate(t, static_cast<size_t>(len))) {
    telemInvalida++;
    return;
  }

  // Fecha o RTT: o ack casa com o instante em que aquele seq foi enviado.
  // Le micros() aqui, no callback, para nao somar a latencia do agendador.
  const uint32_t agoraUs = micros();
  uint32_t rttUs = 0;
  portENTER_CRITICAL_ISR(&mux);
  rtt.on_ack(t.ackSeq, agoraUs, rttUs);
  portEXIT_CRITICAL_ISR(&mux);

  // Metricas de alcance, antes de atualizar o marcador de tempo.
  const uint32_t agoraMs = millis();
  if (temTelemetria.load()) {
    const uint32_t gap = agoraMs - ultimaTelemetriaMs.load();
    if (gap > maiorGapMs.load()) maiorGapMs = gap;
    if (gap > 500) quedas++;  // 5 telemetrias perdidas seguidas = link caiu
  }
  const int8_t rssiAgora = info->rx_ctrl ? static_cast<int8_t>(info->rx_ctrl->rssi) : 0;
  if (rssiMin.load() == 0 || rssiAgora < rssiMin.load()) rssiMin = rssiAgora;

  telemRx++;
  telemBattMv = t.batt_mV;
  telemRssi = t.rssi;
  telemLossPct = t.lossPct;
  telemState = t.state;
  rssiLocal = info->rx_ctrl ? static_cast<int8_t>(info->rx_ctrl->rssi) : 0;
  ultimaTelemetriaMs = millis();
  temTelemetria = true;
}

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  // Com unicast, este callback reflete o ACK da camada MAC: e a medida mais
  // honesta de qualidade de link do lado do transmissor.
  if (status == ESP_NOW_SEND_SUCCESS) {
    txOk++;
  } else {
    txErr++;
  }
}

void radioInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(cfg::WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(80);  // 20 dBm

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

// ------------------------------------------------------------------- eixos --

int lerEixoMv(int pin) {
  uint32_t acc = 0;
  for (int i = 0; i < 16; i++) acc += analogReadMilliVolts(pin);
  return static_cast<int>(acc / 16);
}

// Mapeia mV -> -1000..1000 usando o piso e o teto REAIS do conversor.
// Usar 0..4095 seria errado: a F0 mediu que o ADC trava em 4095 ja aos 3129 mV
// e tem zona morta abaixo de ~142 mV.
int16_t eixoParaComando(int mv, int centroMv) {
  if (mv > centroMv) {
    const int span = cfg::remote::ADC_CEILING_MV - centroMv;
    if (span <= 0) return 0;
    return clamp_axis(static_cast<int32_t>(mv - centroMv) * AXIS_MAX / span);
  }
  const int span = centroMv - cfg::remote::ADC_FLOOR_MV;
  if (span <= 0) return 0;
  return clamp_axis(static_cast<int32_t>(mv - centroMv) * AXIS_MAX / span);
}

// Centro fixo medido na F0. A F4 troca por calibracao gravada em NVS.
int centroThrottleMv() {
  return cfg::remote::THROTTLE_CENTER_RAW * cfg::remote::ADC_CEILING_MV / 4095;
}
int centroSteeringMv() {
  return cfg::remote::STEERING_CENTER_RAW * cfg::remote::ADC_CEILING_MV / 4095;
}

// ----------------------------------------------------------------- tarefas --

void taskInput(void *) {
  const TickType_t periodo = pdMS_TO_TICKS(1000 / cfg::remote::INPUT_HZ);  // 5 ms
  TickType_t ultimoWake = xTaskGetTickCount();
  const int cThr = centroThrottleMv();
  const int cStr = centroSteeringMv();

  for (;;) {
    const int16_t thr = eixoParaComando(lerEixoMv(cfg::remote::PIN_AXIS_THROTTLE), cThr);
    const int16_t str = eixoParaComando(lerEixoMv(cfg::remote::PIN_AXIS_STEERING), cStr);
    eixoThrottle = thr;
    eixoSteering = str;

    for (Trim &t : trims) {
      const bool apertado = (digitalRead(t.pin) == LOW);
      if (apertado && t.anterior) Serial.printf("trim %s\n", t.nome);
      t.anterior = !apertado;
    }

    vTaskDelayUntil(&ultimoWake, periodo);
  }
}

void taskSend(void *) {
  const TickType_t periodo = pdMS_TO_TICKS(1000 / cfg::remote::SEND_HZ);  // 10 ms
  TickType_t ultimoWake = xTaskGetTickCount();

  for (;;) {
    ControlPacket p{};
    p.seq = seqAtual++;
    p.throttle = eixoThrottle;
    p.steering = eixoSteering;
    p.flags = 0;  // arming e F3/F4
    p.aux = 0;
    finalize(p);

    // Anota ANTES de enviar: o RTT tem de incluir o tempo de transmissao.
    portENTER_CRITICAL(&mux);
    rtt.on_send(p.seq, micros());
    portEXIT_CRITICAL(&mux);
    esp_now_send(macPar, reinterpret_cast<const uint8_t *>(&p), sizeof(p));

    vTaskDelayUntil(&ultimoWake, periodo);
  }
}

// ------------------------------------------------------------------ saidas --

const char *nomeEstado(uint8_t s) {
  switch (static_cast<CarState>(s)) {
    case CarState::Boot:     return "boot";
    case CarState::Failsafe: return "FAILSAFE";
    case CarState::Idle:     return "idle";
    case CarState::Armed:    return "armado";
    default:                 return "?";
  }
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

// --------------------------------------------------------------- I2C/OLED --

// O firmware da F2 NAO desenha no OLED -- isso e F4. O scan existe so para
// distinguir "display sem codigo que o acione" de "display com problema", que foi
// exatamente a confusao do OLED do carro na F0.
// Chamar com (22, 21) testa a hipotese de SDA/SCL trocados sem mexer nos fios: foi
// o que resolveu o carro.
void i2cScan(int sda, int scl) {
  Wire.end();
  Wire.begin(sda, scl, 100000);
  Serial.printf("I2C scan (SDA %d / SCL %d): ", sda, scl);
  int achados = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("0x%02X ", addr);
      achados++;
    }
  }
  if (achados == 0) Serial.print(F("nada encontrado"));
  Serial.println();
  if (achados > 0) {
    Serial.println(F("  (OLED vivo. A tela fica preta porque a F2 nao desenha nela -- isso e F4.)"));
  }
}

// ------------------------------------------------------- modo campo (NVS) --

RegistroCampo montaRegistro() {
  const RttSnapshot r = rttSnapshot();
  RegistroCampo g{};
  g.duracaoMs = millis() - campoInicioMs.load();
  g.enviados = txOk.load() + txErr.load();
  g.semAck = txErr.load();
  g.telemRecebidas = telemRx.load();
  g.rttMin = r.min_us;
  g.rttMed = r.avg_us;
  g.rttMax = r.max_us;
  g.rttAmostras = r.samples;
  g.rssiMin = rssiMin.load();
  g.rssiUlt = rssiLocal.load();
  g.rssiNoCarro = telemRssi.load();
  g.perdaCarro = telemLossPct.load();
  g.quedas = quedas.load();
  g.maiorGapMs = maiorGapMs.load();
  return g;
}

// Duas chaves na NVS:
//   "atual"  -> sessao em andamento, reescrita a cada 3 s
//   "ultima" -> sessao encerrada; e o que o comando 'r' le
// A separacao existe porque a placa reinicia DUAS vezes numa caminhada: ao trocar
// o PC pelo power bank e ao voltar. Sem ela, o reinicio da volta sobrescreveria a
// caminhada com uma sessao de 3 segundos, apagando justamente o dado coletado.
void gravaRegistro() {
  const RegistroCampo g = montaRegistro();
  prefs.begin("f2link", false);
  prefs.putBytes("atual", &g, sizeof(g));
  prefs.end();
}

// Fecha a sessao: promove "atual" a "ultima".
void encerraSessao() {
  RegistroCampo ant{};
  prefs.begin("f2link", false);
  if (prefs.getBytes("atual", &ant, sizeof(ant)) == sizeof(ant) && ant.enviados > 0) {
    prefs.putBytes("ultima", &ant, sizeof(ant));
  }
  RegistroCampo vazio{};
  prefs.putBytes("atual", &vazio, sizeof(vazio));
  prefs.end();
}

// Retoma o modo campo apos um reinicio, preservando o que ja tinha sido gravado.
void retomaModoCampo() {
  prefs.begin("f2link", false);
  const bool armado = prefs.getUChar("armado", 0) != 0;
  prefs.end();
  if (!armado) return;

  encerraSessao();  // a sessao anterior vira "ultima" antes de comecar a nova
  modoCampo = true;
  campoInicioMs = millis();
  Serial.println(F(">>> MODO CAMPO ativo (sobreviveu ao reinicio)."));
  Serial.println(F(">>> Mande 'r' para ler a caminhada, ou 'g' para encerrar o modo campo."));
}

void imprimeRegistro(const RegistroCampo &g, const char *titulo) {
  Serial.printf("--- %s ---\n", titulo);
  if (g.rttAmostras == 0 && g.enviados == 0) {
    Serial.println(F("  (vazio: nenhuma caminhada gravada ainda)"));
    return;
  }
  const uint32_t perdaMil = g.enviados ? (g.semAck * 1000u / g.enviados) : 0;
  Serial.printf("  duracao     : %lu s\n", (unsigned long)(g.duracaoMs / 1000));
  Serial.printf("  enviados    : %lu (sem ACK %lu = %lu,%lu%%)\n",
                (unsigned long)g.enviados, (unsigned long)g.semAck,
                (unsigned long)(perdaMil / 10), (unsigned long)(perdaMil % 10));
  Serial.printf("  RTT us      : min %lu | med %lu | max %lu | amostras %lu\n",
                (unsigned long)g.rttMin, (unsigned long)g.rttMed,
                (unsigned long)g.rttMax, (unsigned long)g.rttAmostras);
  Serial.printf("  RSSI        : pior %d | ultimo %d | no carro %d dBm\n",
                (int)g.rssiMin, (int)g.rssiUlt, (int)g.rssiNoCarro);
  Serial.printf("  telemetria  : %lu recebidas | perda no carro %u%%\n",
                (unsigned long)g.telemRecebidas, (unsigned)g.perdaCarro);
  Serial.printf("  ALCANCE     : quedas (>500ms) %lu | maior gap %lu ms\n",
                (unsigned long)g.quedas, (unsigned long)g.maiorGapMs);
  Serial.println(F("  --- criterio F2 (media) ---"));
  Serial.printf("  perda < 1%%   : %s\n", perdaMil < 10 ? "OK" : "FALHOU");
  Serial.printf("  RTT med < 5ms: %s\n",
                g.rttAmostras == 0 ? "sem amostras" : (g.rttMed < 5000 ? "OK" : "FALHOU"));
}

void leRegistro() {
  RegistroCampo g{};
  prefs.begin("f2link", true);
  const size_t n = prefs.getBytes("ultima", &g, sizeof(g));
  prefs.end();
  if (n != sizeof(g)) {
    Serial.println(F("--- NVS: nenhum registro gravado ---"));
    return;
  }
  imprimeRegistro(g, "registro da NVS (ultima caminhada)");
}

void imprimeEstatisticas() {
  const uint32_t enviados = txOk + txErr;
  const uint32_t perdaTx = enviados ? (txErr * 1000u / enviados) : 0;  // por mil

  Serial.println(F("--- controle: link ---"));
  const RttSnapshot r = rttSnapshot();
  Serial.printf("  enviados    : %lu   (ok %lu / sem ACK %lu = %lu,%lu%%)\n",
                (unsigned long)enviados, (unsigned long)txOk, (unsigned long)txErr,
                (unsigned long)(perdaTx / 10), (unsigned long)(perdaTx % 10));
  Serial.printf("  RTT us      : ult %lu | min %lu | med %lu | max %lu | amostras %lu\n",
                (unsigned long)r.last_us, (unsigned long)r.min_us,
                (unsigned long)r.avg_us, (unsigned long)r.max_us,
                (unsigned long)r.samples);
  Serial.printf("  acks sem par: %lu\n", (unsigned long)r.unmatched);
  Serial.printf("  telemetria  : %lu recebidas, %lu invalidas\n",
                (unsigned long)telemRx, (unsigned long)telemInvalida);
  Serial.printf("  carro diz   : estado %s | perda %u%% | bat %u mV | RSSI do controle %d dBm\n",
                nomeEstado(telemState.load()), telemLossPct.load(), telemBattMv.load(), (int)telemRssi.load());
  Serial.printf("  RSSI do carro visto aqui: %d dBm\n", (int)rssiLocal.load());

  Serial.println(F("  --- critério F2 ---"));
  const bool okPerda = (perdaTx < 10);  // < 1,0%
  const bool okRtt = (r.samples > 0 && r.max_us < 5000);
  Serial.printf("  perda < 1%%   : %s\n", okPerda ? "OK" : "FALHOU");
  Serial.printf("  RTT max < 5ms: %s\n",
                r.samples == 0 ? "sem amostras" : (okRtt ? "OK" : "FALHOU"));
}

void trataSerial() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'z':
        portENTER_CRITICAL(&mux);
        rtt.reset();
        portEXIT_CRITICAL(&mux);
        txOk = txErr = 0;
        telemRx = telemInvalida = 0;
        Serial.println(F("Contadores zerados"));
        break;
      case 's': imprimeEstatisticas(); break;
      case 'g': {
        const bool ligar = !modoCampo.load();
        modoCampo = ligar;
        if (ligar) {
          // Zera tudo: uma caminhada comeca limpa, senao mistura bancada e campo.
          portENTER_CRITICAL(&mux);
          rtt.reset();
          portEXIT_CRITICAL(&mux);
          txOk = txErr = 0;
          telemRx = telemInvalida = 0;
          rssiMin = 0;
          maiorGapMs = 0;
          quedas = 0;
          campoInicioMs = millis();
          // O flag vai para a NVS: e o que faz o modo campo sobreviver a troca do
          // PC pelo power bank, que necessariamente reinicia a placa.
          prefs.begin("f2link", false);
          prefs.putUChar("armado", 1);
          prefs.end();
          gravaRegistro();
          Serial.println(F(">>> MODO CAMPO ARMADO. Contadores zerados, gravando na NVS a cada 3 s."));
          Serial.println(F(">>> Sobrevive a reinicio: pode trocar o USB pelo power bank e caminhar."));
          Serial.println(F(">>> Ao voltar, pluge no PC e mande 'r'."));
        } else {
          encerraSessao();
          prefs.begin("f2link", false);
          prefs.putUChar("armado", 0);
          prefs.end();
          Serial.println(F(">>> modo campo desligado. A sessao virou 'ultima': mande 'r'."));
        }
        break;
      }
      case 'r': leRegistro(); break;
      case 'w': scanWifi(); break;
      case 'i': i2cScan(cfg::remote::PIN_SDA, cfg::remote::PIN_SCL); break;
      case 'I': i2cScan(cfg::remote::PIN_SCL, cfg::remote::PIN_SDA); break;
      case '?':
        Serial.println(F("Comandos: z=zerar | s=estatisticas | g=modo campo on/off"));
        Serial.println(F("          r=ler registro da NVS | i=scan I2C | I=scan trocado | ?=ajuda"));
        break;
      default: break;
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(cfg::SERIAL_BAUD);
  delay(300);

  for (Trim &t : trims) pinMode(t.pin, INPUT_PULLUP);
  pinMode(cfg::remote::PIN_BTN_ARM, INPUT_PULLUP);
  pinMode(cfg::remote::PIN_BTN_MENU, INPUT_PULLUP);

  Serial.println();
  Serial.println(F("=== CustomRC CONTROLE - F2 link ==="));
  Serial.println(F("Sem calibracao em NVS, deadzone, expo ou desenho no OLED: isso e F4."));
  Serial.println(F("Comandos: z=zerar | s=estatisticas | g=modo campo | r=ler NVS"));
  Serial.println(F("          i=scan I2C | I=scan trocado | ?=ajuda"));

  i2cScan(cfg::remote::PIN_SDA, cfg::remote::PIN_SCL);
  radioInit();

  // Antes das tarefas: se o modo campo estava armado, a sessao anterior vira
  // "ultima" agora, ANTES que a gravacao periodica da nova sessao a sobrescreva.
  retomaModoCampo();

  xTaskCreatePinnedToCore(taskInput, "input", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(taskSend, "send", 4096, nullptr, 3, nullptr, 1);
}

void loop() {
  static uint32_t ultimoPrint = 0;
  trataSerial();

  const uint32_t agora = millis();

  // Modo campo: persiste a cada 3 s. Sem isso, o reset que a abertura da porta
  // serial provoca apagaria a caminhada inteira.
  static uint32_t ultimaGravacao = 0;
  if (modoCampo.load() && agora - ultimaGravacao >= 3000) {
    ultimaGravacao = agora;
    gravaRegistro();
  }

  if (agora - ultimoPrint >= 1000) {
    ultimoPrint = agora;
    const uint32_t enviados = txOk + txErr;
    const uint32_t perdaTx = enviados ? (txErr * 1000u / enviados) : 0;
    const bool linkVivo = temTelemetria.load() && (agora - ultimaTelemetriaMs.load()) < 1000;
    const RttSnapshot r = rttSnapshot();

    Serial.printf(
        "%6lus | thr %5d str %5d | tx %lu perda %lu,%lu%% | RTT med %lu max %lu us | RSSI %d/%d | %s\n",
        (unsigned long)(agora / 1000), eixoThrottle.load(), eixoSteering.load(),
        (unsigned long)enviados, (unsigned long)(perdaTx / 10), (unsigned long)(perdaTx % 10),
        (unsigned long)r.avg_us, (unsigned long)r.max_us,
        (int)rssiLocal.load(), (int)telemRssi.load(),
        linkVivo ? nomeEstado(telemState.load()) : "SEM TELEMETRIA");
  }
  delay(10);
}
