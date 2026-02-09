#pragma once
#include "iohandler_concept.h"
#include <concepts>
#include <cstdint>

namespace bmy {
using voidFuncPtr = void (*)(void);

template <class T>
concept interrupt_handler = requires(T inter, uint8_t pin, voidFuncPtr callback,
                                     iohandler::PinStatus mode, bool interrupt_disabling_status) {
  { inter.attachGpioInterrupt(pin, callback, mode) } -> std::same_as<void>;
  { inter.detachGpioInterrupt(pin) } -> std::same_as<void>;
  { inter.enableInterrupt(interrupt_disabling_status) } -> std::same_as<void>;
  { inter.disableInterrupt() } -> std::same_as<bool>;
};
} // namespace bmy