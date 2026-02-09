#pragma once
#include "arduino_iohandler.h"
#include "interrupt_concept.h"
#include "iohandler_concept.h"
#include <Arduino.h>

namespace bmy {
class ArduinoInterruptAdapter {
public:
  ArduinoInterruptAdapter() = default;
  ArduinoInterruptAdapter(const ArduinoInterruptAdapter &) = delete;
  ArduinoInterruptAdapter(ArduinoInterruptAdapter &&) = delete;
  ~ArduinoInterruptAdapter() = default;
  void attachGpioInterrupt(uint8_t interruptNum, bmy::voidFuncPtr userFunc,
                           iohandler::PinStatus mode) {
    attachInterrupt(interruptNum, userFunc, arduino_pin_status_converter(mode));
  }
  void detachGpioInterrupt(uint8_t interruptNum) { detachInterrupt(interruptNum); }
  void enableInterrupt(bool status) {
    // if interrupt is already enabled or the previous disabling status is false, do nothing
    if (!status) {
      return;
    }
    interrupt_enabled_ = true;
    interrupts();
  }

  bool disableInterrupt() {
    if (!interrupt_enabled_) {
      return false;
    }
    interrupt_enabled_ = false;
    noInterrupts();
    return true;
  }

private:
  bool interrupt_enabled_{true};
};
} // namespace bmy