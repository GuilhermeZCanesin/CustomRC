#pragma once
#include <stdint.h>
#include <stddef.h>

namespace rc {

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, sem reflexao de entrada ou
// saida, sem XOR final. Escolhido por ser o mais facil de conferir contra
// implementacoes de referencia: crc16("123456789") == 0x29B1.
uint16_t crc16(const void *data, size_t len);

}  // namespace rc
