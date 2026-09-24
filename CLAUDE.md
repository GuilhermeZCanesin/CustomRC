# RC Controller — contexto para o Claude Code

Carrinho RC elétrico com dois ESP32 falando por ESP-NOW. Reescrita completa do firmware, feita em fases curtas e validadas por medição.
Responda em português, de forma técnica e direta. Avance um passo por vez e espere o resultado medido antes do próximo.

- Plano completo (doc vivo): https://claude.ai/code/artifact/330d488b-713b-48ed-b494-2caad611b0ed
- Esquemas de fiação: https://claude.ai/artifact/VPzxJWqbEcKgfehvBZo1Fo (cópia local em `docs/fiacao.html`)
- Cópia do plano no repo: `docs/PLANO.md`.
- Código antigo em `legacy/`: só referência de funcionalidade; não reaproveitar.

## Estado atual

- Parte elétrica montada pelo usuário.
- **Agora: executar a F0 (diagnóstico elétrico) passo a passo** com `firmware/f0/`.
- Depois: F1 (base PlatformIO + lib `rc_protocol` com testes `native`).
- Ambiente: Windows 11, VS Code, Arduino IDE. Se `arduino-cli` estiver instalado, usar para compilar, gravar e ler a serial (115200).
- Versão do core "esp32 by Espressif" ainda não informada: perguntar. Define a API de ESP-NOW (callback de recepção mudou no 3.x) e de LEDC.

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
| Displays | Controle: OLED 1,3" SH1106 128×64 (DST-013). Carro: OLED 0,91" SSD1306 128×32, opcional |
| Controle PS4 | Adiado para pós-implementação (Bluepad32 no ESP do carro) |

Atenção ao shield: a fileira V dos conectores G-V-S costuma ser o 5V/VIN. Qualquer conector de ESC encaixado ali liga o BEC no ESP.

## Pinagem

Carro: IO27 ESC tras. esq. · IO26 ESC tras. dir. · IO32 ESC diant. esq. · IO33 ESC diant. dir. · IO25 servo · IO36 bateria · IO21/22 OLED · IO0 BOOT (pareamento futuro) · IO2 LED.

Controle: IO35 VRy aceleração · IO34 VRx direção · IO13 SW aceleração (armar) · IO14 SW direção (menu) · IO5 trim ACC− (strapping: não segurar no boot) · IO18 trim ACC+ · IO19 trim STR+ · IO23 trim STR− · IO21/22 OLED. Botões com INPUT_PULLUP.

## F0 — roteiro e critérios

Carro (`F0_car_diag`, lib ESP32Servo). ESC pins ficam em LOW fixo (sem sinal, não armam). Rodas fora do chão.
1. Scan I2C mostra 0x3C.
2. Tensão da bateria no serial bate com multímetro ±0,1–0,2 V; IO36 < 3,2 V.
3. `c` (servo centro) e segurar o braço do servo; depois `s` (varredura) e segurar.
4. 10 min rodando, apertar EN: boot seguinte com **0 brownout**. `z` zera contadores.

Controle (`F0_remote_diag`, libs Adafruit SH110X + GFX).
1. Todos os 6 botões aparecem.
2. Anotar bruto mín/centro/máx de cada eixo (vai para a calibração da F4).
3. Multímetro em VRx/VRy no fim de curso: ≤ 3,3 V.

Resultados a registrar: brownouts, tensão (serial vs multímetro), mín/centro/máx dos eixos, versão do core.
Os sketches não foram compilados na sessão de origem (toolchain bloqueado lá): compile primeiro e corrija o que aparecer.

## Pendências

- Relação de engrenagens de cada eixo e diâmetro das rodas (fator dianteira/traseira do mixer).
- Quais ESCs têm BEC (o ZMR pode ser OPTO); comportamento do 40A bidirecional (direto para ré ou freio→neutro→ré).
- ESCs aceitam PWM > 50 Hz?
- Fonte de alimentação atual do controle.
