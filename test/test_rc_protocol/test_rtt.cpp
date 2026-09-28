// Testes do RttTracker. Separados do test_main.cpp para manter os arquivos
// legiveis; o Unity roda os dois no mesmo binario (ver test_main.cpp).
#include <unity.h>
#include <rc_protocol.h>

using namespace rc;

void test_rtt_medida_simples() {
  RttTracker r;
  r.on_send(100, 1000);
  uint32_t rtt = 0;
  TEST_ASSERT_TRUE(r.on_ack(100, 3500, rtt));
  TEST_ASSERT_EQUAL_UINT32(2500, rtt);
  TEST_ASSERT_EQUAL_UINT32(2500, r.last_us());
  TEST_ASSERT_EQUAL_UINT32(1, r.samples());
}

void test_rtt_min_max_media() {
  RttTracker r;
  const uint32_t rtts[] = {1000, 5000, 3000};
  uint32_t out = 0;
  for (uint16_t i = 0; i < 3; i++) {
    r.on_send(i, 0);
    r.on_ack(i, rtts[i], out);
  }
  TEST_ASSERT_EQUAL_UINT32(1000, r.min_us());
  TEST_ASSERT_EQUAL_UINT32(5000, r.max_us());
  TEST_ASSERT_EQUAL_UINT32(3000, r.avg_us());  // (1000+5000+3000)/3
  TEST_ASSERT_EQUAL_UINT32(3, r.samples());
}

// Um ack para um seq que nunca foi enviado nao pode virar amostra.
void test_rtt_ack_desconhecido() {
  RttTracker r;
  uint32_t rtt = 0;
  TEST_ASSERT_FALSE(r.on_ack(42, 1000, rtt));
  TEST_ASSERT_EQUAL_UINT32(0, r.samples());
  TEST_ASSERT_EQUAL_UINT32(1, r.unmatched());
}

// O mesmo envio nao pode gerar duas amostras: o slot e consumido no primeiro ack.
void test_rtt_ack_repetido_nao_conta_duas_vezes() {
  RttTracker r;
  r.on_send(7, 0);
  uint32_t rtt = 0;
  TEST_ASSERT_TRUE(r.on_ack(7, 1000, rtt));
  TEST_ASSERT_FALSE(r.on_ack(7, 2000, rtt));
  TEST_ASSERT_EQUAL_UINT32(1, r.samples());
}

// O caso que mais importa: com 256 slots, o seq 7 e o 263 caem no MESMO indice.
// Sem guardar o seq junto do tempo, um ack tardio do 7 casaria com o envio do
// 263 e produziria um RTT inventado -- exatamente o tipo de numero bonito e
// falso que estragaria a medicao da F2.
void test_rtt_slot_sobrescrito_nao_inventa_medida() {
  RttTracker r;
  r.on_send(7, 1000);
  r.on_send(7 + RttTracker::SLOTS, 50000);  // mesmo indice, seq diferente

  uint32_t rtt = 0;
  TEST_ASSERT_FALSE(r.on_ack(7, 60000, rtt));  // ack do antigo: rejeitado
  TEST_ASSERT_EQUAL_UINT32(0, r.samples());

  // O envio novo continua medindo normalmente.
  TEST_ASSERT_TRUE(r.on_ack(7 + RttTracker::SLOTS, 52000, rtt));
  TEST_ASSERT_EQUAL_UINT32(2000, rtt);
}

// micros() estoura a cada ~71,6 min no ESP32. A subtracao sem sinal tem que
// atravessar o estouro sem produzir um RTT gigante.
void test_rtt_wraparound_de_micros() {
  RttTracker r;
  const uint32_t antes = 0xFFFFF000u;
  r.on_send(1, antes);
  uint32_t rtt = 0;
  // 0x800 us depois do estouro: 0x1000 + 0x800 = 0x1800 us de intervalo real.
  TEST_ASSERT_TRUE(r.on_ack(1, 0x800u, rtt));
  TEST_ASSERT_EQUAL_UINT32(0x1800u, rtt);
}

void test_rtt_reset_limpa_tudo() {
  RttTracker r;
  r.on_send(1, 0);
  uint32_t rtt = 0;
  r.on_ack(1, 1000, rtt);
  r.reset();
  TEST_ASSERT_EQUAL_UINT32(0, r.samples());
  TEST_ASSERT_EQUAL_UINT32(0, r.min_us());
  TEST_ASSERT_EQUAL_UINT32(0, r.max_us());
  TEST_ASSERT_EQUAL_UINT32(0, r.avg_us());
  TEST_ASSERT_FALSE(r.on_ack(1, 2000, rtt));  // slot foi limpo
}

// Cenario realista: 100 Hz de envio, telemetria a 10 Hz ackando o ultimo seq,
// RTT constante de 3 ms. Tem que fechar dentro do critério da F2.
void test_rtt_cenario_100hz() {
  RttTracker r;
  uint32_t rtt = 0;
  uint32_t t = 0;
  for (uint16_t seq = 0; seq < 500; seq++) {
    r.on_send(seq, t);
    if (seq % 10 == 0) {                 // telemetria a cada 10 pacotes
      TEST_ASSERT_TRUE(r.on_ack(seq, t + 3000, rtt));
    }
    t += 10000;                          // 100 Hz -> 10 ms por pacote
  }
  TEST_ASSERT_EQUAL_UINT32(50, r.samples());
  TEST_ASSERT_EQUAL_UINT32(3000, r.avg_us());
  TEST_ASSERT_TRUE(r.max_us() < 5000);   // critério da F2
}
