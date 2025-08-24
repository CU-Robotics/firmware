#include "bitwise_operations.hpp"

uint16_t combine_bytes(uint8_t high, uint8_t low) {
  uint16_t result = 0;
  return ((result | high) << 8) | low;
}

