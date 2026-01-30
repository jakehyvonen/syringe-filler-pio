/**
 * @file RfidReader.hpp
 * @brief PN532 RFID reader helper.
 */
#pragma once

#include <Arduino.h>

class RfidReader {
 public:
  bool begin();
  void poll();
  uint32_t currentTag() const { return m_currentTag; }
  bool hasTag() const { return m_currentTag != 0; }

 private:
  uint32_t m_currentTag = 0;
};
