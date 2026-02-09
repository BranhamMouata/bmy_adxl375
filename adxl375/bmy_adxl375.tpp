#include "adxl375_registers.h"
#include "spi_concept.h"
namespace bmy {

template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief Destructor: ensure the SPI transaction is closed and chip-select released.
 */
Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::~Adxl375() {
  // end the transaction and release CS
  iohandler_->write(chip_select_, iohandler::PinStatus::HIGH);
  spi_->endTransaction();
}

template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief Clear the device FIFO by reading out all stored samples.
 */
void Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::clear_fifo() {
  const auto interrupt_status = interrupt_->disableInterrupt();
  // Clear the fifo before starting measurements
  const auto max_element = adxl375::kFifoSize;
  constexpr uint8_t size = sizeof(acc::RawData);
  for (uint8_t idx = 0; idx < max_element; idx++) {
    acc::RawData dummy{};
    auto *value = reinterpret_cast<uint8_t *>(&dummy);
    read(adxl375::reg::kDataX0, size, value);
    // wait 5us for the fifo to pop
    timer_->delay_us(adxl375::kFifoPopDelayUs);
  }
  interrupt_->enableInterrupt(interrupt_status);
}
template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief Configure the device FIFO control register with watermark and mode.
 */
void Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::init_fifo() {
  // set the fifo sample watermark
  uint8_t fifo_ctl{adxl375::kFifoWatermark};
  // set the fifo mode (bits 7:6)
  constexpr auto mode = static_cast<uint8_t>(static_cast<uint8_t>(adxl375::FifoMode::kFifo) << 6);
  fifo_ctl |= mode;
  write_byte(adxl375::reg::kFifoCtl, fifo_ctl);
}
template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief Configure interrupt mapping on the device (which events trigger IRQs).
 */
void Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::map_interupt() {
  // map the watermark to pin 1 and the other to pin 2
  constexpr uint8_t map = adxl375::kIntMapping;
  write_byte(adxl375::reg::kIntMap, map);
}

template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief Configure interrupt and attach the FIFO-read ISR.
 */
void Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::init_interupt() {
  // set interrupt caller
  if (isr_caller_ == nullptr) {
    isr_caller_ = this;
  }
  // Enable watermark interrupt
  constexpr uint8_t enable = adxl375::kEnableInt;
  write_byte(adxl375::reg::kIntEnable, enable);
  // attach the ISR (RISING edge)
  interrupt_->attachGpioInterrupt(interrupt_pin_, &Adxl375::isr_read_fifo,
                                  iohandler::PinStatus::RISING);
}
// Should be done before setting the fifo
template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief Compute sensor offsets by averaging a number of samples while the
 * device is stationary.
 */
void Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::calibrate() {
  // number of samples to average
  constexpr uint8_t samples = 100;
  auto total = 0;
  acc::RawData raw_offset{};
  while (total < samples) {
    while (!data_ready_) {
      // wait for data ready
    }
    const auto *buff = raw_data();
    const auto count = data_count_;
    for (uint8_t i = 0; i < count; ++i) {
      raw_offset += buff[i];
      total += count;
    }
  }
  // convert the raw data to acceleration
  offset_ = acc::Data(raw_offset, adxl375::kDataScale);
  // Since the Z axis measures the +1g, remove that component
  offset_.z -= samples * adxl375::kDataSensitivity * adxl375::kDataScale;
  // average the offset and invert sign so offsets subtract from raw readings
  offset_ /= (-1 * static_cast<float>(samples));
  // The calibration register is not used to compensate the data because the
  // resolution (0.196 g/LSB) is too low for this application.
}
template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief Initialize the accelerometer: configure CS pin, data rate, format,
 * calibrate and enable FIFO and interrupts.
 */
void Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::init(uint8_t chip_select, uint8_t interrupt_pin,
                                                         uint32_t clock_speed, uint16_t data_rate) {
  //------- Init the device
  chip_select_ = chip_select;
  interrupt_pin_ = interrupt_pin;
  clock_speed_ = clock_speed;
  data_rate_ = data_rate;
  iohandler_->mode(chip_select_, iohandler::PinMode::OUTPUT);
  // deselect device
  iohandler_->write(chip_select_, iohandler::PinStatus::HIGH);
  //------- init operating parameters
  // set the data rate
  uint8_t value = adxl375::ODR_CODE(data_rate_);
  write_byte(adxl375::reg::kBwRate, value);
  // set the data format
  value = adxl375::kDataFormat;
  write_byte(adxl375::reg::kDataFormat, value);
  // map interrupts
  map_interupt();
  // init the fifo
  init_fifo();
  // init interrupt
  init_interupt();
  // wait 10ms for settings to take effect
  timer_->delay_ms(10);
  // clear fifo
  clear_fifo();
  // toggle the measurement mode (enable measurements)
  value = 0x08;
  write_byte(adxl375::reg::kPowerCtl, value);
  // device calibration
  calibrate();
  // clear fifo
  clear_fifo();
}
template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief Read the device ID register and return it.
 */
uint8_t Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::device_id() {
  uint8_t id{};
  read(adxl375::reg::kDevId, 1, &id);
  return id;
}
template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief Low-level multi-byte read starting at register `addr`.
 */
void Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::read(uint8_t addr, uint8_t size,
                                                         volatile uint8_t *ret_data) const {
  const auto interrupt_status = interrupt_->disableInterrupt();
  const bool multiple_bytes = size > 1;
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  iohandler_->write(chip_select_, iohandler::PinStatus::LOW);
  // format the header (read command with multi-byte flag)
  uint8_t header = 0x80;
  header |= (multiple_bytes << 6);
  header |= addr;
  // transfer the header
  spi_->transfer(&header, 1);
  // read data
  spi_->transfer(const_cast<uint8_t *>(ret_data), size);
  // disable the device
  iohandler_->write(chip_select_, iohandler::PinStatus::HIGH);
  spi_->endTransaction();
  interrupt_->enableInterrupt(interrupt_status);
}
template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief Read a single register byte from `addr`.
 * @return register value
 */
uint8_t Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::read_byte(uint8_t addr) const {
  const auto interrupt_status = interrupt_->disableInterrupt();
  constexpr bool multiple_bytes = false;
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  iohandler_->write(chip_select_, iohandler::PinStatus::LOW);
  // format the header and the first data
  uint8_t header = 0x80 | (multiple_bytes << 6);
  header |= addr;
  // transfer header
  spi_->transfer(&header, 1);
  // read data. NB: header is overwritten with read value
  spi_->transfer(&header, 1);
  // disable the device
  iohandler_->write(chip_select_, iohandler::PinStatus::HIGH);
  spi_->endTransaction();
  interrupt_->enableInterrupt(interrupt_status);
  return header;
}
template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief Low-level multi-byte write to register `addr`.
 */
void Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::write(uint8_t addr, volatile uint8_t *data,
                                                          uint8_t size) const {
  const auto interrupt_status = interrupt_->disableInterrupt();
  const bool multiple_bytes = size > 1;
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  iohandler_->write(chip_select_, iohandler::PinStatus::LOW);
  // format the header
  uint8_t header = (multiple_bytes << 6);
  header |= addr;
  // transfer the header
  spi_->transfer(&header, 1);
  // transfer data
  spi_->transfer(const_cast<uint8_t *>(data), size);
  // disable the device
  iohandler_->write(chip_select_, iohandler::PinStatus::HIGH);
  spi_->endTransaction();
  interrupt_->enableInterrupt(interrupt_status);
}
template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief Write a single byte `data` to register `addr`.
 */
void Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::write_byte(uint8_t addr, uint8_t data) const {
  const auto interrupt_status = interrupt_->disableInterrupt();
  constexpr bool multiple_bytes = false;
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  iohandler_->write(chip_select_, iohandler::PinStatus::LOW);
  // format header
  uint8_t header = (multiple_bytes << 6);
  header |= addr;
  // transfer the header
  spi_->transfer(&header, 1);
  // transfer data
  spi_->transfer(&data, 1);
  // disable the device
  iohandler_->write(chip_select_, iohandler::PinStatus::HIGH);
  spi_->endTransaction();
  interrupt_->enableInterrupt(interrupt_status);
}
template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief Read and return processed acceleration data (physical units).
 */
acc::Data Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::data() {
  // update the raw data first
  const auto *raw_data_ptr = raw_data();
  return acc::Data(*raw_data_ptr, adxl375::kDataScale) + offset_;
}

/**
 * @brief Update the FIFO with new data.
 * @return void.
 */
template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
void Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::update_fifo() {
  const auto interrupt_status = interrupt_->disableInterrupt();
  if (data_ready_) {
    // get the number of data in the fifo (mask out non-count bits)
    auto count = read_byte(adxl375::reg::kFifoStatus) & 0x3F;
    data_count_ = count <= adxl375::kFifoSize ? count : 0;
    constexpr uint8_t size = sizeof(acc::RawData);
    for (uint8_t idx = 0; idx < data_count_; idx++) {
      volatile auto *value = reinterpret_cast<volatile uint8_t *>(&fifo_[idx]);
      read(adxl375::reg::kDataX0, size, value);
      // wait 5us for the fifo to pop
      timer_->delay_us(adxl375::kFifoPopDelayUs);
    }
  }
  interrupt_->enableInterrupt(interrupt_status);
}

template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief Read and return raw FIFO sample values.
 */
acc::RawData *Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::raw_data() {
  update_fifo();
  data_ready_ = false;
  return &fifo_[0];
}
template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief ISR handler that reads available FIFO samples when an interrupt occurs.
 */
void Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::isr_read_fifo() {
  isr_caller_->data_ready_ = true;
}
template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief Return true when the FIFO watermark interrupt flag is set.
 */
bool Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::watermark() {
  return static_cast<bool>(read_byte(adxl375::reg::kIntSource) & 0x2);
}

template <spi_com SPI_COM, iohandler_concept IOHANDLER, time_handler TIMER,
          interrupt_handler INTERRUPT>
/**
 * @brief Close the driver and put the device into an idle/safe state.
 *
 * Currently a no-op placeholder — concrete cleanup can be added as needed.
 */
void Adxl375<SPI_COM, IOHANDLER, TIMER, INTERRUPT>::close() {
  interrupt_->detachGpioInterrupt(interrupt_pin_);
  isr_caller_ = nullptr;
  // toggle the measurement mode (disbale measurements)
  write_byte(adxl375::reg::kPowerCtl, 0x00);
  // clear fifo
  clear_fifo();
  // end the transaction and release CS
  iohandler_->write(chip_select_, iohandler::PinStatus::HIGH);
}
} // namespace bmy