#include "adxl375_params.h"
#include "adxl375_registers.h"
#include "spi_concept.h"
namespace bmy {

template <spi_com SPI_COM, wire_handler WIRE, time_handler TIMER, interrupt_handler INTERRUPT>
/**
 * @brief Destructor: ensure the SPI transaction is closed and chip-select released.
 */
Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT>::~Adxl375() {
  // end the transaction and release CS
  wire_->write(chip_select_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}

template <spi_com SPI_COM, wire_handler WIRE, time_handler TIMER, interrupt_handler INTERRUPT>
/**
 * @brief Clear the device FIFO by reading out all stored samples.
 */
void Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT>::clear_fifo() {
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
}
template <spi_com SPI_COM, wire_handler WIRE, time_handler TIMER, interrupt_handler INTERRUPT>
/**
 * @brief Configure the device FIFO control register with watermark and mode.
 */
void Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT>::init_fifo() {
  // set the fifo sample watermark
  uint8_t fifo_ctl{adxl375::kFifoWatermark};
  // set the fifo mode (bits 7:6)
  constexpr auto mode = static_cast<uint8_t>(static_cast<uint8_t>(adxl375::FifoMode::kFifo) << 6);
  fifo_ctl |= mode;
  write_byte(adxl375::reg::kFifoCtl, fifo_ctl);
}
template <spi_com SPI_COM, wire_handler WIRE, time_handler TIMER, interrupt_handler INTERRUPT>
/**
 * @brief Configure interrupt mapping on the device (which events trigger IRQs).
 */
void Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT>::map_interupt() {
  // map the watermark to pin 1 and the other to pin 2
  constexpr uint8_t map = adxl375::kIntMapping;
  write_byte(adxl375::reg::kIntMap, map);
}

template <spi_com SPI_COM, wire_handler WIRE, time_handler TIMER, interrupt_handler INTERRUPT>
/**
 * @brief Configure interrupt and attach the FIFO-read ISR.
 */
void Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT>::init_interupt(uint8_t interrupt_pin) {
  // Enable watermark interrupt
  constexpr uint8_t enable = adxl375::kEnableInt;
  write_byte(adxl375::reg::kIntEnable, enable);
  // attach the ISR (RISING edge)
  interrupt_->attachInterrupt(interrupt_pin, &Adxl375::isr_read_fifo, wire::PinStatus::RISING,
                              this);
}
// Should be done before setting the fifo
template <spi_com SPI_COM, wire_handler WIRE, time_handler TIMER, interrupt_handler INTERRUPT>
/**
 * @brief Compute sensor offsets by averaging a number of samples while the
 * device is stationary.
 */
void Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT>::calibrate() {
  // number of samples to average
  constexpr uint8_t samples = 100;
  acc::RawData raw_offset{};
  for (uint8_t idx = 0; idx < samples; idx++) {
    acc::RawData raw_data{};
    auto *value = reinterpret_cast<uint8_t *>(&raw_data);
    read(adxl375::reg::kDataX0, sizeof(acc::RawData), value);
    raw_offset += raw_data;
    // wait 300 us to have new data
    timer_->delay_us(300);
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
template <spi_com SPI_COM, wire_handler WIRE, time_handler TIMER, interrupt_handler INTERRUPT>
/**
 * @brief Initialize the accelerometer: configure CS pin, data rate, format,
 * calibrate and enable FIFO and interrupts.
 */
void Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT>::init(uint8_t chip_select, uint8_t interrupt_pin,
                                                    uint32_t clock_speed, uint16_t data_rate) {
  //------- Init the device
  chip_select_ = chip_select;
  clock_speed_ = clock_speed;
  data_rate_ = data_rate;
  wire_->mode(chip_select_, wire::PinMode::OUTPUT);
  // deselect device
  wire_->write(chip_select_, wire::PinStatus::HIGH);
  //------- init operating parameters
  // set the data rate
  uint8_t value = adxl375::ODR_CODE(data_rate_);
  write_byte(adxl375::reg::kBwRate, value);
  // set the data format
  value = adxl375::kDataFormat;
  write_byte(adxl375::reg::kDataFormat, value);
  // device calibration
  calibrate();
  // map interrupts
  map_interupt();
  // init the fifo
  init_fifo();
  // init interrupt
  init_interupt(interrupt_pin);
  // wait 10ms for settings to take effect
  timer_->delay_ms(10);
  // clear fifo
  clear_fifo();
  // toggle the measurement mode (enable measurements)
  value = 0x08;
  write_byte(adxl375::reg::kPowerCtl, value);
}
template <spi_com SPI_COM, wire_handler WIRE, time_handler TIMER, interrupt_handler INTERRUPT>
/**
 * @brief Read the device ID register and return it.
 */
uint8_t Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT>::device_id() {
  uint8_t id{};
  read(adxl375::reg::kDevId, 1, &id);
  return id;
}
template <spi_com SPI_COM, wire_handler WIRE, time_handler TIMER, interrupt_handler INTERRUPT>
/**
 * @brief Low-level multi-byte read starting at register `addr`.
 */
void Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT>::read(uint8_t addr, uint8_t size,
                                                    uint8_t *ret_data) const {
  const bool multiple_bytes = size > 1;
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_, wire::PinStatus::LOW);
  // format the header (read command with multi-byte flag)
  uint8_t header = 0x80;
  header |= (multiple_bytes << 6);
  header |= addr;
  // transfer the header
  spi_->transfer(&header, 1);
  // read data
  spi_->transfer(ret_data, size);
  // disable the device
  wire_->write(chip_select_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
template <spi_com SPI_COM, wire_handler WIRE, time_handler TIMER, interrupt_handler INTERRUPT>
/**
 * @brief Read a single register byte from `addr`.
 * @return register value
 */
uint8_t Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT>::read_byte(uint8_t addr) const {
  constexpr bool multiple_bytes = false;
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_, wire::PinStatus::LOW);
  // format the header and the first data
  uint8_t header = 0x80 | (multiple_bytes << 6);
  header |= addr;
  // transfer header
  spi_->transfer(&header, 1);
  // read data. NB: header is overwritten with read value
  spi_->transfer(&header, 1);
  // disable the device
  wire_->write(chip_select_, wire::PinStatus::HIGH);
  spi_->endTransaction();
  return header;
}
template <spi_com SPI_COM, wire_handler WIRE, time_handler TIMER, interrupt_handler INTERRUPT>
/**
 * @brief Low-level multi-byte write to register `addr`.
 */
void Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT>::write(uint8_t addr, uint8_t *data,
                                                     uint8_t size) const {
  const bool multiple_bytes = size > 1;
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_, wire::PinStatus::LOW);
  // format the header
  uint8_t header = (multiple_bytes << 6);
  header |= addr;
  // transfer the header
  spi_->transfer(&header, 1);
  // transfer data
  spi_->transfer(data, size);
  // disable the device
  wire_->write(chip_select_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
template <spi_com SPI_COM, wire_handler WIRE, time_handler TIMER, interrupt_handler INTERRUPT>
/**
 * @brief Write a single byte `data` to register `addr`.
 */
void Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT>::write_byte(uint8_t addr, uint8_t data) const {
  constexpr bool multiple_bytes = false;
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_, wire::PinStatus::LOW);
  // format header
  uint8_t header = (multiple_bytes << 6);
  header |= addr;
  // transfer the header
  spi_->transfer(&header, 1);
  // transfer data
  spi_->transfer(&data, 1);
  // disable the device
  wire_->write(chip_select_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
template <spi_com SPI_COM, wire_handler WIRE, time_handler TIMER, interrupt_handler INTERRUPT>
/**
 * @brief Read and return processed acceleration data (physical units).
 */
acc::Data Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT>::data() const {
  acc::RawData raw_data{};
  auto *val = reinterpret_cast<uint8_t *>(&raw_data);
  read(adxl375::reg::kDataX0, sizeof(acc::RawData), val);
  return acc::Data(raw_data, adxl375::kDataScale) + offset_;
}
template <spi_com SPI_COM, wire_handler WIRE, time_handler TIMER, interrupt_handler INTERRUPT>
/**
 * @brief Read and return raw FIFO sample values.
 */
const acc::RawData *Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT>::raw_data() {
  new_data_ = false;
  return &fifo_[0];
}
template <spi_com SPI_COM, wire_handler WIRE, time_handler TIMER, interrupt_handler INTERRUPT>
/**
 * @brief ISR handler that reads available FIFO samples when an interrupt occurs.
 */
void Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT>::isr_read_fifo(void *accel) {
  auto &adxl375_type = *static_cast<Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT> *>(accel);
  // get the number of data in the fifo (mask out non-count bits)
  adxl375_type.data_number_ = adxl375_type.read_byte(adxl375::reg::kFifoStatus) & 0x3F;
  constexpr uint8_t size = sizeof(acc::RawData);
  for (uint8_t idx = 0; idx < adxl375_type.data_number_; idx++) {
    auto *value = reinterpret_cast<uint8_t *>(&adxl375_type.fifo_[idx]);
    adxl375_type.read(adxl375::reg::kDataX0, size, value);
    // wait 5us for the fifo to pop
    adxl375_type.timer_->delay_us(adxl375::kFifoPopDelayUs);
  }
  adxl375_type.new_data_ = true;
}
template <spi_com SPI_COM, wire_handler WIRE, time_handler TIMER, interrupt_handler INTERRUPT>
/**
 * @brief Return true when the FIFO watermark interrupt flag is set.
 */
bool Adxl375<SPI_COM, WIRE, TIMER, INTERRUPT>::watermark() {
  return static_cast<bool>(read_byte(adxl375::reg::kIntSource) & 0x2);
}

} // namespace bmy