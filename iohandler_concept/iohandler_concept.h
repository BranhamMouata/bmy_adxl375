#pragma once
#include <concepts>
#include <cstdint>
namespace bmy {
namespace wire {
enum class PinMode { INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN, OUTPUT_OPENDRAIN };
enum class PinStatus { LOW, HIGH, CHANGE, FALLING, RISING };
} // namespace wire
template <class T>
concept iohandler_concept =
    requires(T gpio, uint8_t pin, wire::PinMode mode, wire::PinStatus status) {
      { gpio.mode(pin, mode) } -> std::same_as<void>;
      { gpio.write(pin, status) } -> std::same_as<void>;
      { gpio.read(pin) } -> std::same_as<wire::PinStatus>;
    };
} // namespace bmy