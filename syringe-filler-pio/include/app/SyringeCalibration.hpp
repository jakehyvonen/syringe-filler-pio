#pragma once
#include <Arduino.h>
#include "app/Syringe.hpp"

namespace App {

class SyringeCalibration {
public:
  SyringeCalibration(Syringe& toolhead,
                     Syringe* bases,
                     uint8_t baseCount,
                     const uint8_t* baseToPot,
                     int8_t& currentSlot);

  void initializeBaseFromTag(uint8_t slot, uint32_t tag);
  bool initializeToolheadFromTag(uint32_t tag);

  bool captureToolheadCalibrationPoint(float ml, String& message);
  bool saveToolheadCalibration();
  bool clearCurrentBaseCalibrationPoints(String& message);
  bool clearToolheadCalibrationPoints(String& message);
  bool forceCurrentBaseCalibrationZero(String& message);
  bool forceToolheadCalibrationZero(String& message);
  bool captureBaseCalibrationPoint(uint8_t slot, float ml, String& message);
  bool saveCurrentBaseToNVS();
  void printBaseInfo(uint8_t slot, Stream& s);
  void printToolheadInfo(Stream& out);
  float readToolheadVolumeMl();
  float readBaseVolumeMl(uint8_t slot);

private:
  int8_t getBasePotIndex(uint8_t baseSlot) const;
  float readToolheadRatio();
  float readBaseRatio(uint8_t slot);
  bool readBasePotRatio(uint8_t slot, float& ratio, String& message) const;
  static float interpolateVolumeFromPoints(const App::CalibrationPoints& points, float ratio, bool& ok);
  static float mlFromRatio_(const App::PotCalibration& cal, float ratio);

  Syringe& m_toolhead;
  Syringe* m_bases;
  uint8_t m_baseCount;
  const uint8_t* m_baseToPot;
  int8_t& m_currentSlot;
  App::PotCalibration m_toolCal;
  bool m_toolCalValid = false;
};

} // namespace App
