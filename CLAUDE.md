# RC Controller — contexto para o Claude Code

Carrinho RC elétrico com dois ESP32 falando por ESP-NOW. Reescrita completa do firmware, feita em fases curtas e validadas por medição.
Responda em português, de forma técnica e direta. Avance um passo por vez e espere o resultado medido antes do próximo.

- Plano completo (doc vivo): https://claude.ai/code/artifact/330d488b-713b-48ed-b494-2caad611b0ed
- Esquemas de fiação: https://claude.ai/artifact/VPzxJWqbEcKgfehvBZo1Fo (cópia local em `docs/fiacao.html`)
- Cópia do plano no repo: `docs/PLANO.md`.
- Código antigo em `legacy/`: só referência de funcionalidade; não reaproveitar.

## Estado atual

- Parte elétrica montada pelo usuário.
- **F0 concluída e aprovada**, carro e controle. Critérios do carro fechados em
  27/09/2026 com o critério 4 revalidado sem USB. Nenhum bloqueador em aberto.
- **F1 concluída em 27/09/2026:** PlatformIO com ambientes `car`/`remote`/`native`, lib
  `rc_protocol` compartilhada, 26 testes passando no PC e os dois firmwares compilando.
- **F2 aprovada em bancada em 27/09/2026:** perda 0%, RTT médio 4,0 ms.
  **Pendente: repetir a 50 m** (modo campo já implementado, comando `g`).
- Depois: F3 Carro — LEDC, mixer, arming, failsafe, calibração de ESC via CLI.
- Ambiente: Windows 11, VS Code, Arduino IDE.

### Toolchain (instalada e validada em 24/09/2026)

- `arduino-cli` 1.5.2 em `%LOCALAPPDATA%\Programs\arduino-cli` (já no PATH do usuário).
- **Core `esp32:esp32` 3.3.12** — é a 3.x, então a API nova de ESP-NOW vale
  (callback de recepção com `esp_now_recv_info_t*`, não mais `const uint8_t* mac`).
- Libs: ESP32Servo 3.2.1 · Adafruit SSD1306 2.5.17 · Adafruit SH110X 2.1.15 ·
  Adafruit GFX 1.12.6 · Adafruit BusIO 1.17.4.
- Carro em COM5 (CP210x). FQBN `esp32:esp32:esp32`.

Ciclo: `arduino-cli compile --fqbn esp32:esp32:esp32 <sketch>` e
`arduino-cli upload -p COM5 --fqbn esp32:esp32:esp32 <sketch>`.

### PlatformIO: usar o fork pioarduino, não a plataforma oficial

A plataforma **oficial** do PlatformIO (`platform = espressif32`, até a 7.1.3) ainda entrega
o Arduino core **2.0.17** — ela não acompanhou o 3.x. Isso quebra o projeto de forma sutil:
a F0 foi validada no core **3.3.12** pelo arduino-cli, e entre o 2.x e o 3.x mudaram as
assinaturas de callback do ESP-NOW (recepção passou a `esp_now_recv_info_t*`, envio a
`wifi_tx_info_t*`). Dois caminhos de build em cores diferentes = código que compila num e
não no outro.

Solução no `platformio.ini`: apontar `platform` para o fork **pioarduino**, na tag que
corresponde ao mesmo 3.3.12:

```
platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.312-1/platform-espressif32.zip
```

Também é preciso `-std=gnu++17` com `build_unflags = -std=gnu++11`: o core usa gnu++11 por
padrão, onde uma struct com inicializador de membro deixa de ser agregado e listas de
inicialização não compilam.

**Rodar o `pio` pelo PowerShell, nunca pelo Git Bash.** A plataforma pioarduino instala o
toolchain via `idf_tools.py`, que aborta com `ERROR: MSys/Mingw is not supported`. O core e o
toolchain até baixam, mas o `xtensa-esp32-elf-g++` não vai para o PATH e a compilação falha
com "não é reconhecido como um comando". Os testes `native` (g++ do MinGW) funcionam nos dois
shells; só o build para ESP32 exige PowerShell.

### Endereços ESP-NOW — papel descoberto em tempo de execução

As placas são cadastradas em `include/config.h` por **identidade**, não por papel:
`PLACA_A` e `PLACA_B`. Cada firmware lê o próprio MAC no boot e conclui que o par é a
outra da lista. Consequência prática: **inverter papéis é só gravar o outro ambiente**,
sem editar nada.

```
pio run -e car    -t upload --upload-port COMx
pio run -e remote -t upload --upload-port COMx
```

Antes disso os MACs eram fixos como `MAC_CAR`/`MAC_REMOTE`, e trocar as placas de papel
exigia editar o header. O engano fácil era a placa acabar transmitindo para o próprio MAC:
o link simplesmente não fechava, sem nenhuma mensagem apontando a causa. Agora, MAC não
cadastrado é avisado na serial.

Placa nova: trocar só o MAC da que saiu, em `PLACA_A` ou `PLACA_B`.

| Placa | MAC | Estado |
| --- | --- | --- |
| A | `f8:b3:b7:22:1d:94` | **viva** (ESP32-D0WD-V3 rev 3.1) — a única boa |
| — | `f4:65:0b:48:02:2c` | **flash morta** (9 V no VIN) |
| B | `3c:8a:1f:63:5e:6c` | **receptor de RF morto** — vê 0 redes; CPU e flash OK |
| Controle original | não lida | ponte CP210x morta; ESP32 provavelmente íntegro |

### Diagnóstico de rádio: varredura WiFi (comando `w`)

Os dois firmwares têm o comando **`w`**, que varre as redes da vizinhança. É um teste do
receptor **contra o mundo real, fora do nosso protocolo** — não depende de ESP-NOW, canal,
MAC nem CRC. Foi ele que isolou o defeito da placa `3c:8a:1f` em um comando:

```
CARRO    -> 31 redes, mais forte -53 dBm
CONTROLE ->  0 redes
```

Duas placas na mesma sala, a 2 m. Quando uma vê dezenas e a outra vê zero, o rádio da
segunda está morto. **Use `w` antes de investigar qualquer falha de link** — teria poupado
horas aqui.

Cadeia de eliminação usada (vale reaproveitar): contadores de descarte do RX (separa
protocolo de RF) → leitura REAL do canal (não a constante) → afastar as placas 2 m
(saturação) → **trocar as antenas entre as placas** (separa antena de placa) →
`erase-flash` + regravar (calibração de RF na NVS) → `w` nas duas.

## Hardware e decisões fechadas

| Item | Decisão |
| --- | --- |
| Placas | 2× ESP32-WROOM-32U (antena externa), carro usa shield de expansão |
| Tração traseira | 2× ESC 40A bidirecional ZMR + motores 2450kv (ré e freio só aqui) |
| Tração dianteira | 2× ESC 30A unidirecional + motores 1000kv (zero = 1000 µs; em ré/freio fica em 1000 µs) |
| Bateria | LiPo 4S 2200 mAh → chave geral → multiplexador (distribuição) |
| Alimentação ESP carro | Saída 5 V do multiplexador → VIN, C1 470 µF 50 V (Yageo) entre VIN e GND |
| Alimentação servo | BEC do ESC traseiro esquerdo (vermelho ligado direto no servo, não no shield). Vermelho dos outros 3 ESCs isolado |
| Leitura de bateria | R1 100 kΩ (B+ após chave) / R2 22 kΩ → IO36; C2 1 µF 63 V eletrolítico no nó. Razão 5,545. R2 e C2 aterram no GND do ESP |
| Joysticks | 2× HW-504 alimentados em 3V3 (nunca 5 V) |
| Alimentação ESP controle | **5 V regulados** — power bank pelo micro-USB, ou BEC de 5 V no VIN |
| Displays | Controle: OLED 1,3" SH1106 128×64 (DST-013). Carro: OLED 0,91" SSD1306 128×32, opcional |
| Controle PS4 | Adiado para pós-implementação (Bluepad32 no ESP do carro) |

Atenção ao shield: a fileira V dos conectores G-V-S costuma ser o 5V/VIN. Qualquer conector de ESC encaixado ali liga o BEC no ESP.

### NUNCA ligar 9 V no VIN — matou duas placas em 28/09/2026

O VIN alimenta o regulador AMS1117, que dissipa a diferença até 3,3 V. Com 9 V de entrada
e o ESP32 puxando 300–500 mA nos picos de TX, são **1,7 a 2,9 W** num SOT-223 que aguenta
perto de 1 W. Ele entra em proteção térmica e, ao ceder de vez, costuma falhar **em curto**
— jogando 9 V no trilho de 3,3 V.

Dois modos de falha observados, mesma causa:

| Placa | O que morreu | Onde fica | Sintoma |
| --- | --- | --- | --- |
| Controle original | ponte CP210x | **na placa** | não enumera no USB; ESP32 intacto |
| `f4:65:0b:48:02:2c` | chip de flash | **dentro do módulo** | `invalid header: 0xffffffff`, boot loop |

Na primeira os 9 V voltaram pelo trilho de 5 V (muitas placas baratas não têm diodo entre
VIN e o 5 V do USB) e queimaram a ponte. Na segunda o regulador cedeu e a sobretensão
entrou no 3,3 V, que alimenta ESP32 e flash juntos — a flash não aguentou, o ESP32 sim.

Diagnóstico que confirma flash morta, sem ambiguidade:
```
esptool --port COMx --baud 115200 flash-id
  ->  Manufacturer: ff   Device: ffff   Detected flash size: Unknown
```
`ff`/`ffff` é linha sem resposta. O esptool ainda lê chip, revisão e MAC (o ESP32 responde),
mas a flash não. **Sem conserto prático:** a flash fica soldada *dentro* do módulo
WROOM-32U, sob a blindagem, junto do ESP32 e do cristal. Trocar exige ar quente e remover a
blindagem; trocar a placa inteira sai mais barato.

**Por que só o controle morreu:** no carro a bateria passa por chave → divisor, e o ESP
recebe **5 V já regulados** no VIN. O controle não tem divisor, e os 9 V foram ligados
direto no VCC/GND do shield — ou seja, direto no trilho que deveria ser de 5 V.
**A placa do carro nunca foi exposta aos 9 V** (confirmado em 28/09/2026).

**Regra:** o controle é alimentado por 5 V regulados — power bank pelo micro-USB (mais
simples e seguro) ou BEC de 5 V no VIN. Nunca bateria bruta no VCC/VIN. No pino 3V3 nunca
se liga bateria: ele contorna o regulador e exige 3,3 V já estabilizados.

**Se for usar a entrada de 6,5–16 V do shield**, duas checagens antes:
1. **O regulador do shield é linear ou chaveado?** Procure um **indutor** (bobina quadrada)
   perto da entrada de força: se houver, é chaveado e 9 V não é problema. Se houver só um
   chip de 3 pernas (tipo AMS1117-5.0) e capacitores, é **linear** — com 9 V ele dissipa
   1,2–2 W nos picos de TX e repete o problema térmico noutro lugar. Nesse caso usar
   **7,4 V (2S LiPo)**, que derruba a dissipação para ~1 W.
   Aceitar 16 V é o limite absoluto do chip, não recomendação térmica.
2. **Nada de bateria PP3 de 9 V.** Resistência interna de 1–2 Ω não sustenta os picos de
   300–500 mA do ESP32 transmitindo: a tensão afunda e vira brownout.

## Pinagem

Carro: IO27 ESC tras. esq. · IO26 ESC tras. dir. · IO32 ESC diant. esq. · IO33 ESC diant. dir. · IO25 servo · IO36 bateria · IO21/22 OLED · IO0 BOOT (pareamento futuro) · IO2 LED.

Controle: IO35 VRx aceleração · IO34 VRx direção · IO5 trim ACC− (strapping: não segurar no boot) · IO18 trim ACC+ · IO19 trim STR+ · IO23 trim STR− · IO21/22 OLED · **IO32 ARM e IO33 MENU (previstos, ainda sem fio)**. Botões com INPUT_PULLUP.

**Os dois joysticks usam o VRx** (o VRy de ambos fica sem uso): o stick da aceleração está
montado girado, então frente/trás cai no VRx dele. Não confundir na calibração da F4.

Os cliques dos sticks (IO13/IO14) **não serão usados** — decisão de 24/09/2026. ARM e MENU
vão para IO32/IO33 quando forem ligados. Escolha correta: 32 e 33 têm pull-up interno,
enquanto **IO34, IO35, IO36 e IO39 são só de entrada e não têm pull-up** (exigiriam resistor
externo). No carro o 32/33 são os ESCs dianteiros, mas são placas distintas — sem conflito.

## F0 — roteiro e critérios

Carro (`F0_car_diag`, lib ESP32Servo). ESC pins ficam em LOW fixo (sem sinal, não armam). Rodas fora do chão.
1. Scan I2C mostra 0x3C.
2. Tensão da bateria no serial bate com multímetro ±0,1–0,2 V; IO36 < 3,2 V.
3. `c` (servo centro) e segurar o braço do servo; depois `s` (varredura) e segurar.
4. 10 min rodando, apertar EN: boot seguinte com **0 brownout**. `z` zera contadores.

Controle (`F0_remote_diag`, libs Adafruit SH110X + GFX).
1. Os **4 trims** aparecem (ARM/MENU em IO32/33 ainda sem fio, não contam).
2. Anotar bruto mín/centro/máx de cada eixo (vai para a calibração da F4).
3. Multímetro em VRx/VRy no fim de curso: ≤ 3,3 V.

Resultados a registrar: brownouts, tensão (serial vs multímetro), mín/centro/máx dos eixos, versão do core.

### F0 do carro — resultados (24/09/2026)

| # | Critério | Resultado |
| --- | --- | --- |
| 1 | Scan I2C → 0x3C | **Passou.** 0x3C em SDA 21 / SCL 22, com o display desenhando |
| 2 | Tensão ±0,1–0,2 V; IO36 < 3,2 V | **Passou.** Serial 15,47 V vs multímetro 15,39 V (+0,08 V). Pino 2790 mV |
| 3 | Servo centro e varredura | **Passou.** Trava firme; afundamento de só ~120 mV sob carga |
| 4 | 10 min sem brownout | **Passou, com e sem USB.** Ver abaixo |

- Rádio: ~36 mil TX ESP-NOW a 100 Hz / 20 dBm, **0 erros**. Tensão 15,29–15,48 V o tempo todo.
- Bateria no teste: 3,86 V/célula (parcialmente descarregada, não é defeito).
### Critério 4 — aprovado com e sem USB (revalidado em 27/09/2026)

Com USB: 7,7 min, 0 brownout. **Sem USB, só na saída de 5 V do multiplexador, mais de
10 min: `Boot #1 | 0 brownout(s), 0 crash/watchdog`** — zero reboots no período. O ESP não
reiniciou nem na transição de tirar o USB: a saída do multiplexador assumiu a alimentação
sem interrupção. O C1 de 470 µF e o rail de 5 V seguram os picos de TX a 20 dBm com folga.

Ressalva menor: no teste os 2 ESCs **dianteiros** estavam desconectados para reduzir o
barulho; os traseiros ficaram ligados. Os ESCs tomam B+ bruto e não carregam a saída de 5 V
que alimenta o ESP, então isso não enfraquece o resultado de forma relevante.

**Os 14 brownouts da primeira tentativa eram todos artefato.** Naquela sessão o usuário
cortou a alimentação várias vezes, incomodado pelo apito dos ESCs. **Cortar a alimentação
produz a mesma assinatura de um brownout real:** o rail de 3,3 V decai, o detector dispara
em ~2,8 V e grava `ESP_RST_BROWNOUT`, e o C1 de 470 µF segura carga suficiente para o chip
reiniciar e reportar esse motivo. Só contam brownouts de um período em que ninguém encostou
na alimentação — vale para qualquer teste futuro de brownout.

**Apito dos ESCs:** com o carro na bateria eles apitam sem parar. É o alarme de "sem sinal",
consequência esperada de manter os pinos em LOW fixo para não armarem. Desconectar a
alimentação deles silencia (perde-se o servo, que vive do BEC do traseiro esquerdo). Efeito
colateral útil: o alarme confirma que os ESCs recebem energia e **não estão armados**.

**Protocolo, caso precise repetir:** `z` com o USB ligado → tirar só o USB, sem mexer na
chave nem na bateria → deixar ~10 min sem tocar em nada → religar e ler o cabeçalho.

**Correção de fiação aplicada:** a perna de cima do R1 (100 kΩ) estava no BEC / rail de
5 V em vez do B+ depois da chave — a serial lia 4,98 V no lugar de 15,4 V. O divisor em si
estava correto (4,88 × 22/122 = 0,880 V, exatamente o medido). Movida para o B+.

**OLED do carro — resolvido.** Nunca esteve queimado nem sem alimentação: os dois fios de
sinal estavam **trocados** (SDA no IO22, SCL no IO21). O comando `I`, que varre com os pinos
invertidos sem mexer na fiação, isolou isso em segundos — achava 0x3C em 22/21 e nada em
21/22. Ligação correta e validada: **SDA → IO21, SCL → IO22**.

Duas lições para não repetir:
- `IO21=HIGH / IO22=HIGH` no scan **não prova** que há algo conectado — são só os pull-ups
  internos que o `Wire.begin` liga. Serve para achar curto para GND, nada além disso.
- Um `begin()` da Adafruit_SSD1306 retornando OK também não prova presença: ele dá falso
  positivo com o barramento vazio. Só o `beginTransmission`/`endTransmission` cru decide.

Ainda vale **soldar** o OLED em vez de deixar em jumper: a vibração volta quando o carro andar.

### F0 do controle — concluída (24/09/2026)

| # | Critério | Resultado |
| --- | --- | --- |
| 1 | Os 4 trims aparecem | **Passou.** ACC−, ACC+, STR+, STR− todos detectados |
| 2 | Mín/centro/máx dos eixos | **Anotado** (tabela abaixo) |
| 3 | Fim de curso ≤ 3,3 V | **Passou.** 3,28 V de um lado, 0 V do outro |

| Eixo | Pino | Repouso bruto | Repouso mV | Faixa bruta | Faixa mV |
| --- | --- | --- | --- | --- | --- |
| ACC | IO35 | 1852 | 1625 | 0..4095 | **142..3129** |
| STR | IO34 | 1801 | 1584 | 0..4095 | **142..3129** |

Os dois eixos saturam nos **mesmos** 142 e 3129 mV — prova de que o limite é do conversor,
não dos potenciômetros (dois pots independentes não falhariam de forma idêntica).


- Placa: **MAC `f4:65:0b:48:02:2c`**, ESP32-D0WD-V3 rev 3.1. A placa original do controle
  não enumerava no USB (nenhum dispositivo aparecia, nem com cabo de dados comprovado) —
  suspeita de ponte CP210x queimada. **Foi substituída** por uma placa que vinha com o
  firmware AT de fábrica (ESP-IDF v3.0.3). Guardar a antiga: o ESP32 em si deve estar bom,
  e dá para gravá-la por UART externo.
- OLED SH1106 do controle: **0x3C, OK**, desenhando.
- Repouso: ACC (IO35) bruto ~1850 / 1615 mV · STR (IO34) bruto ~1795 / 1577 mV.
  Ruído de só 4–5 contagens em 4095 com média de 16 amostras.
- Multímetro no repouso: 1,5–1,6 V nos dois, batendo com o ADC. **O conversor é fiel no
  centro da escala.**

**Critério 3 passou:** fim de curso mede **3,28 V** (≤ 3,3 V) de um lado e **0 V** do outro.

**Saturação do ADC — número que a F4 precisa.** O potenciômetro varre 0 → 3,28 V inteiros,
mas o ADC do ESP32 com atenuação de 11 dB **trava em `4095` já aos 3129 mV** e tem zona
morta abaixo de ~0,15 V. Resultado: ~150 mV mortos em cada ponta, **~9% do curso perdido,
91% utilizável**. Não é defeito do joystick.
A calibração da F4 **precisa mapear os pontos de saturação, não o fim de curso mecânico** —
senão mapeia uma faixa que o conversor nunca entrega. Anotar `0` e `4095` como mín/máx
brutos é inútil sozinho; o que vale é onde o valor para de mudar.
Se um dia incomodar: ~1 kΩ em série entre o 3V3 e o terminal de cima do HW-504 (10 kΩ)
comprime o topo para ~3,0 V e devolve linearidade no curso inteiro.

**Alerta para a F4 (telemetria de bateria):** com a razão de 5,545, uma 4S cheia (16,8 V)
põe o IO36 em 3,03 V, e o ADC do ESP32 com atenuação de 11 dB comprime acima de ~3,1 V.
A precisão vai piorar justamente com a bateria cheia. Se incomodar, trocar R2 de 22 kΩ
para 18 kΩ (razão 6,56 → 2,56 V no topo).

### Comandos extras no `F0_car_diag` (acrescentados nesta sessão)

`i` refaz o scan I2C · `I` testa com SDA/SCL trocados (sem mexer em fio) ·
`d` tenta reconectar o OLED. O scan também reporta o nível de repouso das linhas.
O sketch desenha tensão e contadores no OLED 128×32 e detecta sozinho quando ele
some ou volta ao barramento — o `begin()` da Adafruit_SSD1306 dá falso positivo,
então a presença é conferida por `beginTransmission` cru a cada 500 ms.

## F1 — base (concluída em 27/09/2026)

```
pio test -e native            # 26 testes, ~3 s
pio run -e car -e remote      # os dois firmwares -- pelo PowerShell
pio run -e car -t upload
```

Resultado: `car` SUCCESS (RAM 14,0% / Flash 69,3%), `remote` SUCCESS, 26/26 testes.
Zero warnings com `-Wall -Wextra`.

Estrutura: `platformio.ini` / `include/config.h` (pinos, MACs e os limites de ADC medidos
na F0) / `lib/rc_protocol/` / `src/car/` / `src/remote/` / `test/test_rc_protocol/`.

A lib e **C++ puro** (so `<stdint.h>`/`<stddef.h>`), sem `Arduino.h`: e isso que faz os
testes `native` valerem algo, porque sao os mesmos arquivos que rodam no ESP32.

| Modulo | Conteudo |
| --- | --- |
| `crc16` | CRC-16/CCITT-FALSE, conferido contra o vetor canonico `0x29B1` |
| `packets` | `ControlPacket`/`TelemetryPacket`/`PairPacket` + `finalize`/`validate` |
| `link` | `SeqTracker`: aceita, descarta atrasado/repetido, conta perda, trata wraparound |
| `mixer` | expo inteiro, traseira bidirecional, dianteira unidirecional, trim/endpoint |

**Divergencia com o PLANO.md nos tamanhos de pacote.** A tabela do plano diz 13 B para o
`ControlPacket` e 14 B para o `TelemetryPacket`. Somando os campos que ela propria enumera,
com `packed`, da **12 B** e **13 B** -- cada um 1 byte a menos. O `PairPacket` da 5 B pelo
mesmo metodo e bate com o plano, o que indica erro de conta na tabela, nao campo faltando.
Os campos sao a fonte da verdade; `static_assert` trava os tamanhos reais.

**Decisao alem do plano:** magic distinto por tipo (`0xC5`/`0x7E`/`0x9A`) em vez de
discriminar so por tamanho. Custa zero byte e ha um teste que forca isso -- um
`TelemetryPacket` copiado sobre um `ControlPacket` tem de ser rejeitado.

**Os firmwares da F1 nao acionam PWM.** Pinos de ESC em LOW fixo, como na F0; as larguras
calculadas so vao para a serial. LEDC, arming e calibracao de ESC sao F3. Deliberado: ate a
F2 validar o protocolo, o carro nao pode se mover por engano.

## F2 — link (bancada aprovada em 27/09/2026)

Resultado com as duas placas na bancada (RSSI ~-25 dBm), janela de 60 s:

| Critério | Medido | Veredito |
| --- | --- | --- |
| Perda < 1% | **0%** nas duas contagens independentes | **OK** |
| RTT < 5 ms | **médio 4,0 ms** (mín 3,1 · máx 15-29 ms) | **OK pela média** |

As duas contagens de perda são independentes e concordam: sequência no carro
(`rx 5035, perdidos 0, descartados 0`) e ACK da camada MAC no controle
(`tx 5078, sem ACK 0`).

**Decisão do usuário (27/09/2026): o critério de RTT vale para a MÉDIA, não o máximo.**
O máximo de 15-29 ms é o rabo da distribuição — retransmissões de MAC e jitter de
escalonamento — e é o preço da perda zero: cada retransmissão bem-sucedida evita um
pacote perdido ao custo de alguns ms. A tabela de latência do próprio PLANO orça
~15-20 ms no total, então picos nessa ordem cabem.

### O RTT medido caiu 3x corrigindo a MEDIÇÃO, não o rádio

| Etapa | mín | médio |
| --- | --- | --- |
| Primeira implementação | 10,9 ms | 12,0 ms |
| Telemetria sincronizada com a recepção | 4,2 ms | 5,2 ms |
| Leitura de bateria fora do caminho crítico | **3,1 ms** | **4,0 ms** |

Dois artefatos de implementação, os dois meus:
1. **Telemetria fora de fase.** Ela disparava num temporizador fixo de 100 ms, sem
   sincronia com a chegada do controle. O `ackSeq` que ela levava já estava velho de
   0-10 ms, e essa espera entrava no RTT como se fosse latência. Corrigido: a
   telemetria é acordada por `vTaskNotifyGiveFromISR` no `onRecv`, com teto de 10 Hz.
2. **`lerBateriaMv()` no caminho crítico** — 16 leituras de ADC, ~1-2 ms, somadas
   direto ao RTT. Agora a bateria fica em cache, atualizada pelo `control` a cada 0,5 s.

Lição: um mínimo alto e muito próximo da média (10,9 vs 12,0) é assinatura de
quantização, não de latência de rádio. ESP-NOW faz 1-3 ms por trecho.

### BUG CORRIGIDO: link não recuperava de reset do transmissor

Encontrado em bancada. Ao resetar o controle, o `seq` volta a 0, mas o `SeqTracker` do
carro ainda tem `last_` num valor alto. Todo pacote novo chega com seq *menor* e é
classificado como **atrasado** — descartado. Medido: **43 s em FAILSAFE**; no pior caso
(seq perto de 65535) seriam ~11 minutos. Em campo: trocar a bateria do controle deixaria
o carro surdo.

Correção no `SeqTracker`: após `REJEICOES_PARA_RESYNC = 10` rejeições **consecutivas**,
assume reinício do transmissor e ressincroniza no seq que está chegando. A 100 Hz isso é
0,1 s, bem dentro da janela de failsafe de 200 ms. Um punhado de pacotes fora de ordem não
dispara o resync (a contagem zera em qualquer aceite); um reset, sim. O contador
`resyncs()` aparece como `rs<n>` na serial do carro.

Validado em bancada: reset do controle → `rs0` virou `rs1` e o carro **nunca saiu de
`idle`**, com o `rx` crescendo sem interrupção. 5 testes `native` cobrem o caso.

### Modo campo (para o teste de 50 m)

O RTT é medido no controle, que é quem se afasta — e abrir a porta serial reseta o ESP32,
apagando as estatísticas. Solução: mesmo truque do contador de brownout da F0, gravar na
NVS.

Comandos no controle: **`g`** liga/desliga o modo campo (zera tudo e grava a cada 3 s),
**`r`** lê o registro gravado, `s` estatísticas ao vivo, `z` zera.

Roteiro: `g` com o USB ligado → desligar o USB → caminhar 50 m com o controle num power
bank → voltar → plugar → `r`. O registro guarda duração, enviados/sem-ACK, RTT
mín/méd/máx, **pior RSSI**, **quedas (>500 ms sem telemetria)** e **maior gap** — que são
o que de fato denuncia o limite de alcance, não a latência média.

### Estrutura da F2

Carro: `control` core 1 @200 Hz · `telemetry` core 0, acordada pela recepção, teto 10 Hz ·
callbacks de rádio · CLI. Controle: `input` core 1 @200 Hz · `send` core 1 @100 Hz com
`vTaskDelayUntil` · callbacks · CLI.

Acrescentado à lib: `RttTracker` (guarda o seq junto do tempo em cada slot, para um ack
atrasado não casar com outro envio e inventar um RTT), `CarState`, e o resync no
`SeqTracker`. **39 testes `native`**, todos passando.

Concorrência: contadores tocados por callbacks de rádio usam `std::atomic`, não
`volatile` — `volatile` não torna o `++` atômico. O `RttTracker` é mutado por dois
contextos (`on_send` no core 1, `on_ack` no callback do core 0) e todo acesso passa por
`portMUX`.

**PWM continua desligado.** Pinos de ESC em LOW, larguras só na serial. É F3.

## Pendências

- Relação de engrenagens de cada eixo e diâmetro das rodas (fator dianteira/traseira do mixer).
- Comportamento do 40A bidirecional (direto para ré ou freio→neutro→ré).
- ESCs aceitam PWM > 50 Hz?
- **Placa nova para o controle** (a `f4:65:0b` perdeu a flash). Ao chegar: trocar o MAC em
  `PLACA_B` no `include/config.h` e gravar `-e remote`. Nada mais.
- **Confirmar se a placa viva (`f8:b3:b7`) foi exposta aos 9 V.** Dano latente de
  sobretensão aparece sob carga de rádio — justamente no teste de alcance.
- Avaliar recuperar o controle original por **UART externo** (TX0/RX0/GND + BOOT no reset):
  a ponte USB morreu, mas o ESP32 e a flash dele devem estar íntegros.
- **Teste de alcance de 50 m da F2**: usar o modo campo (`g` … caminhar … `r`).
- **Centro dos eixos está fora de zero**: em repouso o controle envia thr ~118 / str ~115,
  o que daria 1559 µs na traseira em vez de 1500 — o carro sairia andando sozinho. É o
  centro fixo da F0 em `config.h`, que não corresponde ao repouso real. **A F4 tem de
  resolver isso antes de qualquer PWM ser ligado na F3.**

Resolvido em 27/09/2026: **as duas placas estão com antena externa conectada** — o PA
sempre transmitiu carregado, sem risco de degradação. E **tudo foi soldado ou colado**,
incluindo o OLED do carro; nada mais em jumper frouxo.

Resolvido na F0: o ESC traseiro esquerdo **tem BEC** e alimenta o servo com folga
(120 mV de afundamento segurando o braço), então o ZMR não é OPTO.
