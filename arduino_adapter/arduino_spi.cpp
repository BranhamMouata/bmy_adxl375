#include "arduino_spi.h"

namespace bmy {
/**
 * @brief Convert generic `spi::BitOrder` to Arduino `BitOrder` enum.
 */
constexpr BitOrder ArduinoSpiAdapter::convert_to_arduino_bit_order(spi::BitOrder bit_order) {
  switch (bit_order) {
  case spi::BitOrder::LSBFIRST:
    return LSBFIRST;
  case spi::BitOrder::MSBFIRST:
    return MSBFIRST;
  default:
    return MSBFIRST;
  }
}
/**
 * @brief Convert generic `spi::Mode` to Arduino `SPIMode` enum.
 */
constexpr arduino::SPIMode ArduinoSpiAdapter::convert_to_arduino_mode(spi::Mode spi_mode) {
  switch (spi_mode) {
  case spi::Mode::SPI_MODE0:
    return arduino::SPI_MODE0;
  case spi::Mode::SPI_MODE1:
    return arduino::SPI_MODE1;
  case spi::Mode::SPI_MODE2:
    return arduino::SPI_MODE2;
  case spi::Mode::SPI_MODE3:
    return arduino::SPI_MODE3;
  default:
    return arduino::SPI_MODE3;
  }
}
/**
 * @brief Begin an SPI transaction using Arduino's SPISettings wrapper.
 */
void ArduinoSpiAdapter::beginTransaction(uint32_t clock_speed, spi::BitOrder bit_order,
                                         spi::Mode spi_mode) {
  const auto arduino_bit_order = convert_to_arduino_bit_order(bit_order);
  const auto arduino_mode = convert_to_arduino_mode(spi_mode);
  const SPISettings settings(clock_speed, arduino_bit_order, arduino_mode);
  SPI.beginTransaction(settings);
}
/**
 * @brief Transfer `count` bytes over SPI; wrapper around Arduino `SPI.transfer`.
 */
void ArduinoSpiAdapter::transfer(void *buf, size_t count) { SPI.transfer(buf, count); }
} // namespace bmy