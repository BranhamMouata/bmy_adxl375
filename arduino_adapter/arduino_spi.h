#pragma once
#include "spi_concept.h"
#include <SPI.h>
namespace bmy {

class ArduinoSpiAdapter {

public:
  ArduinoSpiAdapter() = default;
  ArduinoSpiAdapter(const ArduinoSpiAdapter &) = delete;
  ArduinoSpiAdapter(ArduinoSpiAdapter &&) noexcept = delete;
  ArduinoSpiAdapter &operator=(const ArduinoSpiAdapter &) = delete;
  ArduinoSpiAdapter &operator=(ArduinoSpiAdapter &&) = delete;
  ~ArduinoSpiAdapter() = default;
  void begin() { SPI.begin(); }
  void beginTransaction(uint32_t clock_speed, spi::BitOrder bit_order, spi::Mode spi_mode);
  void transfer(void *buf, size_t count);
  void endTransaction(void) { SPI.endTransaction(); }

private:
  constexpr BitOrder convert_to_arduino_bit_order(spi::BitOrder bit_order);
  constexpr arduino::SPIMode convert_to_arduino_mode(spi::Mode spi_mode);
};
} // namespace bmy