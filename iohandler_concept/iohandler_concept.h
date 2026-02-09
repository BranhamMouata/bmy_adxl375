#pragma once
#include <concepts>
#include <cstdint>
namespace bmy {
namespace iohandler {
enum class PinMode { INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN, OUTPUT_OPENDRAIN };
enum class PinStatus { LOW, HIGH, CHANGE, FALLING, RISING };
} // namespace iohandler
template <class T>
concept iohandler_concept =
    requires(T gpio, uint8_t pin, iohandler::PinMode mode, iohandler::PinStatus status) {
      { gpio.mode(pin, mode) } -> std::same_as<void>;
      { gpio.write(pin, status) } -> std::same_as<void>;
      { gpio.read(pin) } -> std::same_as<iohandler::PinStatus>;
    };
} // namespace bmy