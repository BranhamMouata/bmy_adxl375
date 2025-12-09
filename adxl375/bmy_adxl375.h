#pragma once
#include "acc_data.h"
#include "adxl375_params.h"
#include "interrupt_concept.h"
#include "iohandler_concept.h"
#include "spi_concept.h"
#include "timer_concept.h"
namespace bmy {
// This class is a wrapper only for testing purpose
template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
class Adxl375Test;

/**
 * @brief Driver wrapper for the ADXL375 accelerometer.
 *
 * This is a hardware-abstracted template so it can be used with different
 * platform-specific SPI and GPIO/time/interrupt helpers.
 *
 * @tparam SPI_COM   SPI implementation type satisfying the `spi_com` concept.
 * @tparam IOHANDLER      GPIO helper type satisfying the `iohandler_concept` concept.
 * @tparam TIMER     Timer/delay helper satisfying the `time_handler` concept.
 * @tparam INTERRUPT Interrupt controller helper satisfying the `interrupt_handler` concept.
 *
 * Example:
 * @code{.cpp}
 * bmy::Adxl375<SpiImpl, WireImpl, TimerImpl, InterruptImpl> accel(&spi, &wire, &timer, &irq);
 * accel.init(CHIP_SELECT_PIN, 400, INTERRUPT_PIN);
 * auto measurements = accel.data();
 * @endcode
 *
 * The class exposes initialization, device id readout and helpers to read
 * processed (`acc::Data`) or raw (`acc::RawData`) samples from the device
 * FIFO. Private helpers handle low-level register access, calibration and ISR
 * handling.
 */
template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
class Adxl375 {
  friend Adxl375Test<SPI_COM, IOHANDLER, TIMER, INTERRUPT>;

public:
  using SPI_TYPE = SPI_COM;
  using IOHANDLER_TYPE = IOHANDLER;
  using TIMER_TYPE = TIMER;
  using INTERRUPT_TYPE = INTERRUPT;

  Adxl375(SPI_COM *spi, IOHANDLER *wire, TIMER *timer, INTERRUPT *interrupt)
      : spi_(spi), wire_(wire), timer_(timer), interrupt_(interrupt) {}
  Adxl375(const Adxl375 &) = delete;
  Adxl375(Adxl375 &&) noexcept = delete;
  Adxl375 &operator=(const Adxl375 &) = delete;
  Adxl375 &operator=(const Adxl375 &&) noexcept = delete;

  /**
   * @brief Destructor — ensures any hardware resources are cleaned up.
   */
  ~Adxl375();

  /**
   * @brief Initialize the device and internal state.
   *
   * Configures the chip-select pin, sets the sensor data rate and installs
   * the interrupt handler on `interrupt_pin`.
   *
   * @param chip_select GPIO pin used as chip-select for SPI transactions.
   * @param interrupt_pin GPIO pin number used to receive data-ready interrupts.
   * @param data_rate   Desired sensor data rate (device-specific units).
   */
  void init(uint8_t chip_select, uint8_t interrupt_pin, uint32_t clock_speed, uint16_t data_rate);

  /**
   * @brief Read the device ID register.
   * @return 8-bit device identifier read from the sensor.
   */
  uint8_t device_id();

  /**
   * @brief Close the driver and put the device into an idle/safe state.
   *
   * Currently a no-op placeholder — concrete cleanup can be added as needed.
   */
  void close() {}

  /**
   * @brief Return the last processed accelerometer sample as `acc::Data`.
   * @return Processed sensor sample (physical units applied where appropriate).
   */
  acc::Data data() const;

  /**
   * @brief Return the latest raw FIFO sample(s) as `acc::RawData`.
   * @return Raw sensor bytes/values as read from the device.
   */
  const acc::RawData *raw_data();

  /**
   * @brief Return current FIFO fill level (number of samples stored).
   * @return FIFO size in samples (0..kFifoSize).
   */
  uint8_t data_count() const { return data_number_; }

  /**
   * @brief Return true if new data has been read into the FIFO since last call.
   * @return true if new data is available.
   */
  bool new_data() const { return new_data_; }

protected:
  Adxl375() = default;
  // The calibration should be done before setting the fifo
private:
  /**
   * @brief Perform sensor calibration (offset computation) before enabling FIFO.
   */
  void calibrate();

  /**
   * @brief Return true if FIFO watermark level has been reached.
   */
  bool watermark();

  /**
   * @brief Clear the device FIFO.
   */
  void clear_fifo();
  /**
   * @brief Initialize FIFO settings on the device (thresholds, modes).
   */
  void init_fifo();

  /**
   * @brief Configure interrupt mapping on the device (which events trigger IRQs).
   */
  void map_interupt();

  /**
   * @brief Configure and attach the interrupt for data-ready/FIFO events.
   * @param interrupt_pin GPIO pin used for the interrupt.
   */
  void init_interupt(uint8_t interrupt_pin);

  /**
   * @brief Low-level read of `size` bytes starting from device register `addr`.
   * @param addr Register address to read from.
   * @param size Number of bytes to read.
   * @param ret_data Pointer to output buffer (must have `size` capacity).
   */
  void read(uint8_t addr, uint8_t size, uint8_t *ret_data) const;

  /**
   * @brief Read a single register byte.
   * @param addr Register address.
   * @return Byte read from the register.
   */
  uint8_t read_byte(uint8_t addr) const;

  /**
   * @brief Low-level write of `size` bytes to device register `addr`.
   */
  void write(uint8_t addr, uint8_t *data, uint8_t size) const;

  /**
   * @brief Write a single byte to register `addr`.
   */
  void write_byte(uint8_t addr, uint8_t data) const;

  /**
   * @brief ISR helper called when an interrupt indicates FIFO data is ready.
   * @param adxl375 Pointer to the driver instance (passed through the IRQ attach).
   */
  static void isr_read_fifo(void *adxl375);

private:
  acc::RawData fifo_[adxl375::kFifoSize]{};
  acc::Data offset_{};
  uint32_t clock_speed_;
  uint16_t data_rate_;
  uint8_t data_number_{};
  uint8_t chip_select_;
  bool new_data_{false};
  SPI_COM *spi_;
  IOHANDLER *wire_;
  TIMER *timer_;
  INTERRUPT *interrupt_;
};
} // namespace bmy
#include "bmy_adxl375.tpp"
