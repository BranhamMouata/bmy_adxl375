#pragma once
#include "arduino_wire.h"
#include "interrupt_concept.h"
#include "wire_concept.h"
#include <Arduino.h>

namespace bmy {
class ArduinoInterruptAdapter {
public:
  ArduinoInterruptAdapter() = default;
  ArduinoInterruptAdapter(const ArduinoInterruptAdapter &) = delete;
  ArduinoInterruptAdapter(ArduinoInterruptAdapter &&) = delete;
  ~ArduinoInterruptAdapter() = default;
  void attachInterrupt(uint8_t interruptNum, bmy::voidFuncPtrParam userFunc, wire::PinStatus mode,
                       void *param) {
    arduino::attachInterrupt<void>(interruptNum, userFunc, arduino_pin_status_converter(mode),
                                   param);
  }
};
} // namespace bmy