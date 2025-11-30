#pragma once
#include <stdint.h>
namespace bmy::acc {

struct RawData {
  RawData() = default;
  RawData(const RawData &) = default;
  RawData(RawData &&) noexcept = default;
  explicit RawData(int16_t x, int16_t y, int16_t z) : xi(x), yi(y), zi(z) {}
  ~RawData() = default;
  RawData &operator=(const RawData &) = default;
  RawData &operator=(RawData &&) noexcept = default;

  RawData operator+(const RawData &other) {
    RawData result{};
    result.xi = xi + other.xi;
    result.yi = yi + other.yi;
    result.zi = zi + other.zi;
    return result;
  }
  RawData &operator+=(const RawData &other) {
    xi += other.xi;
    yi += other.yi;
    zi += other.zi;
    return *this;
  }
  int16_t xi;
  int16_t yi;
  int16_t zi;
};

struct Data {
  Data() = default;
  Data(const Data &) = default;
  Data(Data &&) noexcept = default;
  explicit Data(float x1, float y1, float z1) : x(x1), y(y1), z(z1) {}
  explicit Data(const RawData &raw_data, float scale) {
    x = static_cast<float>(raw_data.xi) * scale;
    y = static_cast<float>(raw_data.yi) * scale;
    z = static_cast<float>(raw_data.zi) * scale;
  }
  ~Data() = default;
  Data &operator=(const Data &) = default;
  Data &operator=(Data &&) noexcept = default;
  Data operator+(const Data &other) { return Data{x + other.x, y + other.y, z + other.z}; }
  Data &operator+=(const Data &other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }
  Data operator*(const float scale) { return Data{x * scale, y * scale, z * scale}; }
  Data &operator*=(const float scale) {
    x *= scale;
    y *= scale;
    z *= scale;
    return *this;
  }
  Data operator/(const float scale) { return (*this) * (1 / scale); }
  Data &operator/=(const float scale) {
    (*this) *= (1 / scale);
    return *this;
  }
  float x;
  float y;
  float z;
};
} // namespace bmy::acc
