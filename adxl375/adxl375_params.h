#pragma once
#include <cstdint>

namespace bmy::adxl375 {
inline constexpr uint32_t kClkSpeed = 5000000; // 5MHz
inline constexpr uint8_t kChipSelectPin = 10;
inline constexpr uint8_t kHeaderBits = 8;
inline constexpr uint8_t kFifoPopDelayUs = 5; // us
// data rate
inline constexpr uint8_t ODR_CODE(const uint16_t freq) {
  switch (freq) {
  case 800:
    return 0x0D;
  case 1600:
    return 0x0E;
  case 3200:
    return 0x0F;
  default:
    return 0x0A;
  }
}
// Fifo mode
enum class FifoMode { kByPass = 0x0, kFifo, kStream, kTrigger };
// Fifo interupt pin
inline constexpr uint8_t kInterPin = 0X2;
inline constexpr auto kDataRate = 3200; // output data rate
inline constexpr auto kDataFormat = 0X0B;
// The scale of the acceleration read in the data register
inline constexpr float kDataScale = 0.049;      // g/LSB
inline constexpr float kDataSensitivity = 20.5; // LSB/g
// Scale for calibration
inline constexpr float kCalibScale = 0.196;                 // g/LSB
inline constexpr float kCalibSensitivity = 1 / kCalibScale; // LSB/g
// Interrupt crtl
inline constexpr uint8_t kIntMapping = 0xF9;
inline constexpr uint8_t kEnableInt = 0X02;
// The number of sample in fifo to trigger the interupt
inline constexpr uint8_t kFifoWatermark = 19;
// The maximum size of the fifo
inline constexpr uint8_t kFifoSize = 32;
} // namespace bmy::adxl375