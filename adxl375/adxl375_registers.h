#pragma once
#include <cstdint>

namespace bmy::adxl375::reg {
inline constexpr uint8_t kDevId = 0x00;
inline constexpr uint8_t kThreshShock = 0x1D;
inline constexpr uint8_t kOfsx = 0x1E;
inline constexpr uint8_t kOfsy = 0x1F;
inline constexpr uint8_t kOfsz = 0x20;
inline constexpr uint8_t kDur = 0x21;
inline constexpr uint8_t kLatent = 0x22;
inline constexpr uint8_t kWindow = 0x23;
inline constexpr uint8_t kThreshAct = 0x24;
inline constexpr uint8_t kThreshInact = 0x25;
inline constexpr uint8_t kTimeInact = 0x26;
inline constexpr uint8_t kActInactCtl = 0x27;
inline constexpr uint8_t kShockAxis = 0x2A;
inline constexpr uint8_t kActShockStatus = 0x2B;
inline constexpr uint8_t kBwRate = 0x2C;
inline constexpr uint8_t kPowerCtl = 0x2D;
inline constexpr uint8_t kIntEnable = 0x2E;
inline constexpr uint8_t kIntMap = 0x2F;
inline constexpr uint8_t kIntSource = 0x30;
inline constexpr uint8_t kDataFormat = 0x31;
inline constexpr uint8_t kDataX0 = 0x32;
inline constexpr uint8_t kDataX1 = 0x33;
inline constexpr uint8_t kDataY0 = 0x34;
inline constexpr uint8_t kDataY1 = 0x35;
inline constexpr uint8_t kDataZ0 = 0x36;
inline constexpr uint8_t kDataZ1 = 0x37;
inline constexpr uint8_t kFifoCtl = 0x38;
inline constexpr uint8_t kFifoStatus = 0x39;
} // namespace bmy::adxl375::reg