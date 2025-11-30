#pragma once
#include "wire_concept.h"
#include <Arduino.h>

namespace bmy {
constexpr PinMode arduino_pin_mode_converter(wire::PinMode mode) {
  switch (mode) {
  case wire::PinMode::INPUT:
    return INPUT;
  case wire::PinMode::OUTPUT:
    return OUTPUT;
  case wire::PinMode::INPUT_PULLUP:
    return INPUT_PULLUP;
  case wire::PinMode::INPUT_PULLDOWN:
    return INPUT_PULLDOWN;
  case wire::PinMode::OUTPUT_OPENDRAIN:
    return OUTPUT_OPENDRAIN;
  default:
    return INPUT;
  }
}
constexpr PinStatus arduino_pin_status_converter(wire::PinStatus status) {
  switch (status) {
  case wire::PinStatus::LOW:
    return LOW;
  case wire::PinStatus::HIGH:
    return HIGH;
  case wire::PinStatus::CHANGE:
    return CHANGE;
  case wire::PinStatus::FALLING:
    return FALLING;
  case wire::PinStatus::RISING:
    return RISING;
  default:
    return LOW;
  }
}

constexpr wire::PinStatus bmy_pin_status_converter(PinStatus status) {
  switch (status) {
  case LOW:
    return wire::PinStatus::LOW;
  case HIGH:
    return wire::PinStatus::HIGH;
  case CHANGE:
    return wire::PinStatus::CHANGE;
  case FALLING:
    return wire::PinStatus::FALLING;
  case RISING:
    return wire::PinStatus::RISING;
  default:
    return wire::PinStatus::LOW;
  }
}

class ArduinoWireAdapter {
public:
  ArduinoWireAdapter() = default;
  void mode(uint8_t pin, wire::PinMode mode) { pinMode(pin, arduino_pin_mode_converter(mode)); }
  void write(uint8_t pin, wire::PinStatus status) {
    digitalWrite(pin, arduino_pin_status_converter(status));
  }
  wire::PinStatus read(uint8_t pin) { return bmy_pin_status_converter(digitalRead(pin)); }
};
} // namespace bmy