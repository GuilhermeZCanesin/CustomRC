# CustomRC

Carrinho RC elétrico com dois ESP32-WROOM-32U comunicando por ESP-NOW: controle próprio (2× HW-504 + OLED SH1106) e carro (4 ESCs, servo MG996R, LiPo 4S).

| Pasta | Conteúdo |
| --- | --- |
| `CLAUDE.md` | Contexto, decisões fechadas, pinagem e fase atual (lido pelo Claude Code) |
| `docs/PLANO.md` | Plano de reescrita: diagnóstico, arquitetura, protocolo, fases |
| `docs/fiacao.html` | Esquemas de ligação do carro e do controle (abrir no navegador) |
| `firmware/f0/` | Sketches de diagnóstico elétrico da fase F0 |
| `legacy/` | Firmware antigo, só como referência de funcionalidade |

## Fase atual

F0 — diagnóstico elétrico. Roteiro e critérios em `CLAUDE.md`.
