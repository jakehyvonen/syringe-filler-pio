#pragma once
#include <cstddef>   // <- for std::size_t
#include <stdint.h>

namespace EEStore {
  void begin(std::size_t bytes = 512);
  void saveBasePositions(const long* arr, uint8_t n);
  void loadBasePositions(long* arr, uint8_t n);
}
