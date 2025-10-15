#pragma once
#include <stdint.h>

namespace EEStore {
  void begin(size_t bytes = 512);
  void saveBasePositions(const long* arr, uint8_t n);
  void loadBasePositions(long* arr, uint8_t n);
}
