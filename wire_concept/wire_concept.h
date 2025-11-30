#pragma once
#include <concepts>
#include <cstdint>
namespace bmy {
namespace wire {
enum class PinMode { INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN, OUTPUT_OPENDRAIN };
enum class PinStatus { LOW, HIGH, CHANGE, FALLING, RISING };
} // namespace wire
template <class T>
concept wire_handler = requires(T wire, uint8_t pin, wire::PinMode mode, wire::PinStatus status) {
  { wire.mode(pin, mode) } -> std::same_as<void>;
  { wire.write(pin, status) } -> std::same_as<void>;
  { wire.read(pin) } -> std::same_as<wire::PinStatus>;
};
} // namespace bmy