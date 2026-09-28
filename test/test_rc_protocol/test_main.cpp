#include <unity.h>
#include <stdio.h>
#include <string.h>
#include <rc_protocol.h>

using namespace rc;

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- CRC16 ----

// Vetor canonico do CRC-16/CCITT-FALSE. Se isto passa, a implementacao bate
// com qualquer referencia externa.
static void test_crc_vetor_conhecido() {
  const char *s = "123456789";
  TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16(s, 9));
}

static void test_crc_vazio_e_init() {
  TEST_ASSERT_EQUAL_HEX16(0xFFFF, crc16("", 0));
}

static void test_crc_muda_com_um_bit() {
  uint8_t a[4] = {1, 2, 3, 4};
  uint8_t b[4] = {1, 2, 3, 5};
  TEST_ASSERT_NOT_EQUAL(crc16(a, 4), crc16(b, 4));
}

// -------------------------------------------------------------- Pacotes ----

static void test_tamanhos_travados() {
  TEST_ASSERT_EQUAL_UINT32(12, sizeof(ControlPacket));
  TEST_ASSERT_EQUAL_UINT32(13, sizeof(TelemetryPacket));
  TEST_ASSERT_EQUAL_UINT32(5, sizeof(PairPacket));
}

static void test_control_ida_e_volta() {
  ControlPacket p{};
  p.seq = 1234;
  p.flags = Flag::Arm;
  p.throttle = -500;
  p.steering = 250;
  p.aux = 7;
  finalize(p);

  TEST_ASSERT_EQUAL_HEX8(MAGIC_CONTROL, p.magic);
  TEST_ASSERT_EQUAL_HEX8(PROTO_VERSION, p.ver);
  TEST_ASSERT_TRUE(validate(p, sizeof(p)));

  // Os campos sobrevivem intactos.
  TEST_ASSERT_EQUAL_INT16(-500, p.throttle);
  TEST_ASSERT_EQUAL_INT16(250, p.steering);
  TEST_ASSERT_EQUAL_UINT16(1234, p.seq);
}

static void test_control_rejeita_len_errado() {
  ControlPacket p{};
  finalize(p);
  TEST_ASSERT_FALSE(validate(p, sizeof(p) - 1));
  TEST_ASSERT_FALSE(validate(p, sizeof(p) + 1));
}

static void test_control_rejeita_magic_e_versao() {
  ControlPacket p{};
  finalize(p);

  ControlPacket q = p;
  q.magic = 0x00;
  TEST_ASSERT_FALSE(validate(q, sizeof(q)));

  ControlPacket r = p;
  r.ver = PROTO_VERSION + 1;
  TEST_ASSERT_FALSE(validate(r, sizeof(r)));
}

// O ponto do CRC: corromper QUALQUER byte do payload tem que reprovar.
static void test_control_rejeita_corrupcao_em_cada_byte() {
  ControlPacket base{};
  base.seq = 9;
  base.throttle = 321;
  base.steering = -77;
  base.aux = 3;
  finalize(base);

  for (size_t i = 0; i < sizeof(base) - 2; i++) {
    ControlPacket c = base;
    reinterpret_cast<uint8_t *>(&c)[i] ^= 0x01;  // vira um bit
    char msg[64];
    snprintf(msg, sizeof(msg), "byte %u corrompido passou", (unsigned)i);
    TEST_ASSERT_FALSE_MESSAGE(validate(c, sizeof(c)), msg);
  }
}

static void test_telemetria_e_pair() {
  TelemetryPacket t{};
  t.seq = 5;
  t.ackSeq = 4;
  t.batt_mV = 15470;
  t.rssi = -62;
  t.lossPct = 2;
  t.state = 1;
  finalize(t);
  TEST_ASSERT_TRUE(validate(t, sizeof(t)));
  TEST_ASSERT_EQUAL_UINT16(15470, t.batt_mV);
  TEST_ASSERT_EQUAL_INT8(-62, t.rssi);

  PairPacket pr{};
  pr.role = static_cast<uint8_t>(Role::Car);
  finalize(pr);
  TEST_ASSERT_TRUE(validate(pr, sizeof(pr)));

  // Tipos diferentes nao se confundem: telemetria nao valida como controle.
  ControlPacket falso{};
  memcpy(&falso, &t, sizeof(falso));
  TEST_ASSERT_FALSE(validate(falso, sizeof(falso)));
}

static void test_clamp_axis() {
  TEST_ASSERT_EQUAL_INT16(1000, clamp_axis(5000));
  TEST_ASSERT_EQUAL_INT16(-1000, clamp_axis(-5000));
  TEST_ASSERT_EQUAL_INT16(0, clamp_axis(0));
}

// ----------------------------------------------------------- SeqTracker ----

static void test_seq_sequencia_perfeita() {
  SeqTracker t;
  for (uint16_t s = 1; s <= 100; s++) {
    TEST_ASSERT_TRUE(t.accept(s));
  }
  TEST_ASSERT_EQUAL_UINT32(100, t.accepted());
  TEST_ASSERT_EQUAL_UINT32(0, t.lost());
  TEST_ASSERT_EQUAL_UINT8(0, t.lossPct());
}

static void test_seq_conta_perdidos() {
  SeqTracker t;
  t.accept(1);
  t.accept(5);  // 2, 3 e 4 sumiram
  TEST_ASSERT_EQUAL_UINT32(3, t.lost());
  TEST_ASSERT_EQUAL_UINT32(2, t.accepted());
}

static void test_seq_descarta_repetido_e_atrasado() {
  SeqTracker t;
  t.accept(10);
  TEST_ASSERT_FALSE(t.accept(10));  // repetido
  TEST_ASSERT_FALSE(t.accept(9));   // atrasado
  TEST_ASSERT_EQUAL_UINT32(2, t.rejected());
  TEST_ASSERT_EQUAL_UINT16(10, t.last());
}

// O wraparound de 16 bits nao pode ser lido como "atrasado": a 100 Hz ele
// acontece a cada ~11 minutos de operacao continua.
static void test_seq_wraparound() {
  SeqTracker t;
  TEST_ASSERT_TRUE(t.accept(65534));
  TEST_ASSERT_TRUE(t.accept(65535));
  TEST_ASSERT_TRUE(t.accept(0));
  TEST_ASSERT_TRUE(t.accept(1));
  TEST_ASSERT_EQUAL_UINT32(0, t.lost());
  TEST_ASSERT_EQUAL_UINT32(4, t.accepted());
}

static void test_seq_loss_pct() {
  SeqTracker t;
  t.accept(1);
  t.accept(3);  // 1 perdido, 2 aceitos -> 1/3 = 33%
  TEST_ASSERT_EQUAL_UINT8(33, t.lossPct());
}

// ----------------------------------------------------------------- Expo ----

static void test_expo_zero_e_linear() {
  for (int32_t v = -1000; v <= 1000; v += 250) {
    TEST_ASSERT_EQUAL_INT32(v, apply_expo(v, 0));
  }
}

static void test_expo_preserva_extremos_e_sinal() {
  // Qualquer expo tem que manter 0 em 0 e os extremos nos extremos, senao o
  // curso util encolhe.
  for (uint8_t e = 0; e <= 100; e = static_cast<uint8_t>(e + 25)) {
    TEST_ASSERT_EQUAL_INT32(0, apply_expo(0, e));
    TEST_ASSERT_EQUAL_INT32(1000, apply_expo(1000, e));
    TEST_ASSERT_EQUAL_INT32(-1000, apply_expo(-1000, e));
  }
}

static void test_expo_suaviza_o_centro() {
  // expo=100 e cubica pura: 0.5^3 * 1000 = 125.
  TEST_ASSERT_EQUAL_INT32(125, apply_expo(500, 100));
  TEST_ASSERT_EQUAL_INT32(-125, apply_expo(-500, 100));
  // Mais expo => resposta menor perto do centro.
  TEST_ASSERT_TRUE(apply_expo(500, 50) < apply_expo(500, 0));
}

// ---------------------------------------------------------------- Mixer ----

static void test_neutro_e_failsafe() {
  MixerConfig cfg;
  Outputs o = neutral(cfg);
  TEST_ASSERT_EQUAL_UINT16(1500, o.rear_left_us);
  TEST_ASSERT_EQUAL_UINT16(1500, o.rear_right_us);
  // Dianteira unidirecional: parada e 1000, nao 1500.
  TEST_ASSERT_EQUAL_UINT16(1000, o.front_left_us);
  TEST_ASSERT_EQUAL_UINT16(1000, o.front_right_us);
  TEST_ASSERT_EQUAL_UINT16(1500, o.steering_us);
}

static void test_comando_zerado_da_neutro() {
  MixerConfig cfg;
  Command c;
  Outputs o = mix(c, cfg);
  Outputs n = neutral(cfg);
  TEST_ASSERT_EQUAL_UINT16(n.rear_left_us, o.rear_left_us);
  TEST_ASSERT_EQUAL_UINT16(n.front_left_us, o.front_left_us);
  TEST_ASSERT_EQUAL_UINT16(n.steering_us, o.steering_us);
}

static void test_aceleracao_total_frente() {
  MixerConfig cfg;
  Command c;
  c.throttle = 1000;
  Outputs o = mix(c, cfg);
  TEST_ASSERT_EQUAL_UINT16(2000, o.rear_left_us);
  TEST_ASSERT_EQUAL_UINT16(2000, o.front_left_us);
}

static void test_re_respeita_o_limite() {
  MixerConfig cfg;
  cfg.reverse_limit = 50;
  Command c;
  c.throttle = -1000;
  Outputs o = mix(c, cfg);
  // Metade do curso de re: 1500 - 500*0.5 = 1250.
  TEST_ASSERT_EQUAL_UINT16(1250, o.rear_left_us);
  // E a dianteira nao pode andar para tras: fica no minimo.
  TEST_ASSERT_EQUAL_UINT16(1000, o.front_left_us);
}

static void test_dianteira_so_para_frente() {
  MixerConfig cfg;
  const int16_t negativos[] = {-1000, -500, 0};
  for (int16_t t : negativos) {
    Command c;
    c.throttle = t;
    TEST_ASSERT_EQUAL_UINT16(1000, mix(c, cfg).front_left_us);
  }
  Command c;
  c.throttle = 1;
  TEST_ASSERT_TRUE(mix(c, cfg).front_left_us > 1000);
}

static void test_fator_dianteira() {
  MixerConfig cfg;
  cfg.front_factor = 50;
  Command c;
  c.throttle = 1000;
  Outputs o = mix(c, cfg);
  TEST_ASSERT_EQUAL_UINT16(1500, o.front_left_us);  // 1000 + 1000*0.5
  TEST_ASSERT_EQUAL_UINT16(2000, o.rear_left_us);   // traseira nao muda
}

static void test_direcao_trim_endpoint_inversao() {
  MixerConfig cfg;
  Command c;

  cfg.steer_trim = 100;  // trim desloca o centro
  TEST_ASSERT_EQUAL_UINT16(1550, mix(c, cfg).steering_us);

  cfg.steer_trim = 0;
  cfg.steer_endpoint = 50;  // metade do curso
  c.steering = 1000;
  TEST_ASSERT_EQUAL_UINT16(1750, mix(c, cfg).steering_us);

  cfg.steer_endpoint = 100;
  cfg.steer_invert = true;
  TEST_ASSERT_EQUAL_UINT16(1000, mix(c, cfg).steering_us);
}

// Invariante mais importante do mixer: nenhuma combinacao de entrada, nem com
// config absurda, pode produzir pulso fora da janela do ESC/servo.
static void test_saidas_nunca_saem_da_janela() {
  MixerConfig cfg;
  cfg.steer_trim = 1000;  // trim absurdo de proposito
  for (int32_t t = -3000; t <= 3000; t += 137) {
    for (int32_t s = -3000; s <= 3000; s += 291) {
      Command c;
      c.throttle = clamp_axis(t);
      c.steering = clamp_axis(s);
      Outputs o = mix(c, cfg);
      const uint16_t us[] = {o.rear_left_us, o.rear_right_us, o.front_left_us,
                             o.front_right_us, o.steering_us};
      for (uint16_t v : us) {
        TEST_ASSERT_GREATER_OR_EQUAL_UINT16(cfg.us_min, v);
        TEST_ASSERT_LESS_OR_EQUAL_UINT16(cfg.us_max, v);
      }
    }
  }
}

// Definidos em test_rtt.cpp -- mesmo binario, arquivo separado por legibilidade.
void test_rtt_medida_simples();
void test_rtt_min_max_media();
void test_rtt_ack_desconhecido();
void test_rtt_ack_repetido_nao_conta_duas_vezes();
void test_rtt_slot_sobrescrito_nao_inventa_medida();
void test_rtt_wraparound_de_micros();
void test_rtt_reset_limpa_tudo();
void test_rtt_cenario_100hz();

// Definidos em test_resync.cpp -- ressincronizacao apos reinicio do transmissor.
void test_resync_apos_reinicio_do_transmissor();
void test_resync_dentro_da_janela_de_failsafe();
void test_fora_de_ordem_isolado_nao_dispara_resync();
void test_wraparound_nao_conta_como_reinicio();
void test_reset_limpa_contagem_de_resync();

int main(int, char **) {
  UNITY_BEGIN();

  RUN_TEST(test_crc_vetor_conhecido);
  RUN_TEST(test_crc_vazio_e_init);
  RUN_TEST(test_crc_muda_com_um_bit);

  RUN_TEST(test_tamanhos_travados);
  RUN_TEST(test_control_ida_e_volta);
  RUN_TEST(test_control_rejeita_len_errado);
  RUN_TEST(test_control_rejeita_magic_e_versao);
  RUN_TEST(test_control_rejeita_corrupcao_em_cada_byte);
  RUN_TEST(test_telemetria_e_pair);
  RUN_TEST(test_clamp_axis);

  RUN_TEST(test_seq_sequencia_perfeita);
  RUN_TEST(test_seq_conta_perdidos);
  RUN_TEST(test_seq_descarta_repetido_e_atrasado);
  RUN_TEST(test_seq_wraparound);
  RUN_TEST(test_seq_loss_pct);

  RUN_TEST(test_expo_zero_e_linear);
  RUN_TEST(test_expo_preserva_extremos_e_sinal);
  RUN_TEST(test_expo_suaviza_o_centro);

  RUN_TEST(test_neutro_e_failsafe);
  RUN_TEST(test_comando_zerado_da_neutro);
  RUN_TEST(test_aceleracao_total_frente);
  RUN_TEST(test_re_respeita_o_limite);
  RUN_TEST(test_dianteira_so_para_frente);
  RUN_TEST(test_fator_dianteira);
  RUN_TEST(test_direcao_trim_endpoint_inversao);
  RUN_TEST(test_saidas_nunca_saem_da_janela);

  RUN_TEST(test_rtt_medida_simples);
  RUN_TEST(test_rtt_min_max_media);
  RUN_TEST(test_rtt_ack_desconhecido);
  RUN_TEST(test_rtt_ack_repetido_nao_conta_duas_vezes);
  RUN_TEST(test_rtt_slot_sobrescrito_nao_inventa_medida);
  RUN_TEST(test_rtt_wraparound_de_micros);
  RUN_TEST(test_rtt_reset_limpa_tudo);
  RUN_TEST(test_rtt_cenario_100hz);

  RUN_TEST(test_resync_apos_reinicio_do_transmissor);
  RUN_TEST(test_resync_dentro_da_janela_de_failsafe);
  RUN_TEST(test_fora_de_ordem_isolado_nao_dispara_resync);
  RUN_TEST(test_wraparound_nao_conta_como_reinicio);
  RUN_TEST(test_reset_limpa_contagem_de_resync);

  return UNITY_END();
}
