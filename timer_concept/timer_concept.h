#pragma once
#include <concepts>
#include <cstdint>

template <class T>
concept time_handler = requires(T timer, uint32_t t) {
  // delay in milliseconds
  { timer.delay_ms(t) } -> std::same_as<void>;
  // delay in microseconds
  { timer.delay_us(t) } -> std::same_as<void>;
};