#pragma once
#include "cstdint"
#include <concepts>
#include <stddef.h>
namespace bmy {
namespace spi {
enum class BitOrder { LSBFIRST, MSBFIRST };
enum class Mode { SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3 };
} // namespace spi
template <class T>
concept spi_com = requires(T spi, uint32_t clock_speed, spi::BitOrder bit_order, spi::Mode spi_mode,
                           void *buf, size_t count) {
  { spi.begin() } -> std::same_as<void>;
  { spi.beginTransaction(clock_speed, bit_order, spi_mode) } -> std::same_as<void>;
  { spi.transfer(buf, count) };
  { spi.endTransaction() } -> std::same_as<void>;
};
} // namespace bmy