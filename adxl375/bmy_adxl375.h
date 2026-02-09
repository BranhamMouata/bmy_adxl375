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
 * bmy::Adxl375<SpiImpl, IohandlerImpl, TimerImpl, InterruptImpl> accel(&spi, &iohandler, &timer,
 * &irq); accel.init(CHIP_SELECT_PIN, 400, INTERRUPT_PIN); auto measurements = accel.data();
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

  Adxl375(SPI_COM *spi, IOHANDLER *iohandler, TIMER *timer, INTERRUPT *interrupt)
      : spi_(spi), iohandler_(iohandler), timer_(timer), interrupt_(interrupt) {}
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
  void close();

  /**
   * @brief Return the last processed accelerometer sample as `acc::Data`.
   * @return Processed sensor sample (physical units applied where appropriate).
   */
  acc::Data data();

  /**
   * @brief Return the latest raw FIFO sample(s) as `acc::RawData`.
   * @return Raw sensor bytes/values as read from the device.
   */
  acc::RawData *raw_data();

  /**
   * @brief Return current FIFO fill level (number of samples stored).
   * @return FIFO size in samples (0..kFifoSize).
   */
  uint8_t data_count() const { return data_count_; }

  /**
   * @brief Return true if new data has been read into the FIFO since last call.
   * @return true if new data is available.
   */
  bool data_ready() const { return data_ready_; }

  /**
   * @brief Return the configured data rate.
   * @return Data rate configured for the device.
   */
  uint16_t data_rate() const { return adxl375::DATA_RATE(data_rate_); }

protected:
  Adxl375() = default;

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
  void init_interupt();

  /**
   * @brief Low-level read of `size` bytes starting from device register `addr`.
   * @param addr Register address to read from.
   * @param size Number of bytes to read.
   * @param ret_data Pointer to output buffer (must have `size` capacity).
   */
  void read(uint8_t addr, uint8_t size, volatile uint8_t *ret_data) const;

  /**
   * @brief Read a single register byte.
   * @param addr Register address.
   * @return Byte read from the register.
   */
  uint8_t read_byte(uint8_t addr) const;

  /**
   * @brief Low-level write of `size` bytes to device register `addr`.
   */
  void write(uint8_t addr, volatile uint8_t *data, uint8_t size) const;

  /**
   * @brief Write a single byte to register `addr`.
   */
  void write_byte(uint8_t addr, uint8_t data) const;

  /**
   * @brief ISR helper called when an interrupt indicates FIFO data is ready.
   * @param adxl375 Pointer to the driver instance (passed through the IRQ attach).
   */
  static void isr_read_fifo();

  /**
   * @brief Update the FIFO with new data.
   * @return void.
   */
  void update_fifo();

private:
  acc::RawData fifo_[adxl375::kFifoSize]{};
  acc::Data offset_{};
  uint32_t clock_speed_;
  uint16_t data_rate_;
  uint8_t data_count_{};
  uint8_t chip_select_;
  uint8_t interrupt_pin_;
  volatile bool data_ready_{false};
  SPI_COM *spi_;
  IOHANDLER *iohandler_;
  TIMER *timer_;
  INTERRUPT *interrupt_;
  static inline Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT> *isr_caller_ = nullptr;
};
} // namespace bmy
#include "bmy_adxl375.tpp"
