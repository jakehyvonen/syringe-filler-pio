#pragma once
#include <stdint.h>

namespace AxisPair {
  void init();
  void setSpeedSPS(long sps);
  void move2(long steps);
  void move3(long steps);

  void moveSync(long steps2, long steps3); // m23
  void link(long steps); // s2=+steps, s3=-steps
  long pos2();
  long pos3();
  bool move2UntilPotSimple(uint16_t target_adc, long sps);
}
