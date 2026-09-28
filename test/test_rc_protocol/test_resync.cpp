// Testes da ressincronizacao do SeqTracker apos reinicio do transmissor.
//
// Bug real encontrado em bancada na F2: ao resetar o controle, o seq voltou para
// 0 enquanto o carro esperava ~4300. Todo pacote novo parecia "atrasado" e era
// descartado, deixando o carro em FAILSAFE por 43 s. A 100 Hz, no pior caso
// (seq proximo de 65535) isso seriam ~11 minutos de carro surdo.
#include <unity.h>
#include <rc_protocol.h>

using namespace rc;

// O cenario exato do bug: transmissor reiniciou e recomeca do zero.
void test_resync_apos_reinicio_do_transmissor() {
  SeqTracker t;
  for (uint16_t s = 1; s <= 4300; s++) t.accept(s);
  TEST_ASSERT_EQUAL_UINT16(4300, t.last());

  // Transmissor reinicia: seq volta a 0 e sobe de novo.
  uint32_t aceitosDepois = 0;
  for (uint16_t s = 0; s < 40; s++) {
    if (t.accept(s)) aceitosDepois++;
  }

  // Tem de voltar a aceitar, e rapido.
  TEST_ASSERT_TRUE(aceitosDepois > 0);
  TEST_ASSERT_EQUAL_UINT32(1, t.resyncs());
  // Apos o resync, o tracker segue a nova sequencia.
  TEST_ASSERT_EQUAL_UINT16(39, t.last());
}

// O resync nao pode custar mais que a janela de failsafe (200 ms = 20 pacotes).
void test_resync_dentro_da_janela_de_failsafe() {
  SeqTracker t;
  for (uint16_t s = 1; s <= 5000; s++) t.accept(s);

  int pacotesAteAceitar = 0;
  for (uint16_t s = 0; s < 100; s++) {
    pacotesAteAceitar++;
    if (t.accept(s)) break;
  }
  // 10 rejeicoes para o resync => aceita no 10o pacote, ou seja 0,1 s a 100 Hz.
  TEST_ASSERT_EQUAL_INT(10, pacotesAteAceitar);
  TEST_ASSERT_TRUE(pacotesAteAceitar < 20);
}

// Nao pode ser gatilho facil: poucos pacotes fora de ordem sao normais no ar e
// NAO devem fazer o tracker abandonar a sequencia boa.
void test_fora_de_ordem_isolado_nao_dispara_resync() {
  SeqTracker t;
  for (uint16_t s = 1; s <= 100; s++) t.accept(s);

  // 9 atrasados seguidos: um abaixo do limite.
  for (int i = 0; i < 9; i++) TEST_ASSERT_FALSE(t.accept(50));
  TEST_ASSERT_EQUAL_UINT32(0, t.resyncs());
  TEST_ASSERT_EQUAL_UINT16(100, t.last());  // sequencia boa preservada

  // Um pacote bom no meio zera a contagem de rejeicoes consecutivas...
  TEST_ASSERT_TRUE(t.accept(101));
  // ...e por isso outros 9 atrasados ainda nao disparam o resync.
  for (int i = 0; i < 9; i++) TEST_ASSERT_FALSE(t.accept(50));
  TEST_ASSERT_EQUAL_UINT32(0, t.resyncs());
  TEST_ASSERT_EQUAL_UINT16(101, t.last());
}

// O wraparound legitimo continua sendo avanco, nunca resync.
void test_wraparound_nao_conta_como_reinicio() {
  SeqTracker t;
  t.accept(65530);
  for (uint16_t s = 65531; s != 5; s++) TEST_ASSERT_TRUE(t.accept(s));
  TEST_ASSERT_EQUAL_UINT32(0, t.resyncs());
}

void test_reset_limpa_contagem_de_resync() {
  SeqTracker t;
  for (uint16_t s = 1; s <= 1000; s++) t.accept(s);
  for (uint16_t s = 0; s < 15; s++) t.accept(s);
  TEST_ASSERT_TRUE(t.resyncs() > 0);
  t.reset();
  TEST_ASSERT_EQUAL_UINT32(0, t.resyncs());
  TEST_ASSERT_FALSE(t.started());
}
