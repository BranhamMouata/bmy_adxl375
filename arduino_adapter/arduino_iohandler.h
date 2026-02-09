#pragma once
#include "iohandler_concept.h"
#include <Arduino.h>

namespace bmy {
constexpr PinMode arduino_pin_mode_converter(iohandler::PinMode mode) {
  switch (mode) {
  case iohandler::PinMode::INPUT:
    return INPUT;
  case iohandler::PinMode::OUTPUT:
    return OUTPUT;
  case iohandler::PinMode::INPUT_PULLUP:
    return INPUT_PULLUP;
  case iohandler::PinMode::INPUT_PULLDOWN:
    return INPUT_PULLDOWN;
  case iohandler::PinMode::OUTPUT_OPENDRAIN:
    return OUTPUT_OPENDRAIN;
  default:
    return INPUT;
  }
}
constexpr PinStatus arduino_pin_status_converter(iohandler::PinStatus status) {
  switch (status) {
  case iohandler::PinStatus::LOW:
    return LOW;
  case iohandler::PinStatus::HIGH:
    return HIGH;
  case iohandler::PinStatus::CHANGE:
    return CHANGE;
  case iohandler::PinStatus::FALLING:
    return FALLING;
  case iohandler::PinStatus::RISING:
    return RISING;
  default:
    return LOW;
  }
}

constexpr iohandler::PinStatus bmy_pin_status_converter(PinStatus status) {
  switch (status) {
  case LOW:
    return iohandler::PinStatus::LOW;
  case HIGH:
    return iohandler::PinStatus::HIGH;
  case CHANGE:
    return iohandler::PinStatus::CHANGE;
  case FALLING:
    return iohandler::PinStatus::FALLING;
  case RISING:
    return iohandler::PinStatus::RISING;
  default:
    return iohandler::PinStatus::LOW;
  }
}

class ArduinoIoHandlerAdapter {
public:
  ArduinoIoHandlerAdapter() = default;
  void mode(uint8_t pin, iohandler::PinMode mode) {
    pinMode(pin, arduino_pin_mode_converter(mode));
  }
  void write(uint8_t pin, iohandler::PinStatus status) {
    digitalWrite(pin, arduino_pin_status_converter(status));
  }
  iohandler::PinStatus read(uint8_t pin) { return bmy_pin_status_converter(digitalRead(pin)); }
};
} // namespace bmy