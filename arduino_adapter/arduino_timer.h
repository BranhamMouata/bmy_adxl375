#pragma once
#include <Arduino.h>

namespace bmy {
class ArduinoTimerAdapter {
public:
  ArduinoTimerAdapter() = default;
  void delay_ms(uint32_t t) { delay(t); }
  void delay_us(uint32_t t) { delayMicroseconds(t); }
};
} // namespace bmy
