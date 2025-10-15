#include "hw/Drivers.hpp"
#include <Wire.h>

namespace Drivers {
  static bool g_hasPCA=false, g_hasADS=false;
  Adafruit_PWMServoDriver PCA;
  Adafruit_ADS1115 ADS;

  bool i2cPresent(uint8_t addr){
    Wire.beginTransmission(addr);
    return (Wire.endTransmission()==0);
  }

  bool initI2C(int sda, int scl, uint32_t freq){
    Wire.begin(sda, scl);
    Wire.setClock(freq);

    // ADS1115
    if (i2cPresent(0x48) && ADS.begin(0x48)) {
      ADS.setGain(GAIN_ONE);
      g_hasADS = true;
    }

    // PCA9685
    if (i2cPresent(0x40)) {
      PCA = Adafruit_PWMServoDriver(0x40);
      PCA.begin();
      PCA.setPWMFreq(60);
      g_hasPCA = true;
    }
    return true;
  }

  bool hasPCA(){ return g_hasPCA; }
  bool hasADS(){ return g_hasADS; }
}
