#pragma once
#include "iohandler_concept.h"
#include <concepts>
#include <cstdint>

namespace bmy {
using voidFuncPtrParam = void (*)(void *);

template <class T>
concept interrupt_handler =
    requires(T inter, uint8_t pin, voidFuncPtrParam callback, wire::PinStatus mode, void *param) {
      { inter.attachInterrupt(pin, callback, mode, param) } -> std::same_as<void>;
    };
} // namespace bmy