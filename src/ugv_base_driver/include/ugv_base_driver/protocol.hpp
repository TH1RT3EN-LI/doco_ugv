#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "ugv_base_driver/driver_types.hpp"

namespace duojin01
{

inline uint8_t xor_checksum(const uint8_t * data, std::size_t len) noexcept
{
  uint8_t checksum = 0;
  for (std::size_t index = 0; index < len; ++index) {
    checksum ^= data[index];
  }
  return checksum;
}

inline int16_t saturate_i16_from_scaled(double value, double scale) noexcept
{
  const double scaled_value = value * scale;
  if (!std::isfinite(scaled_value)) {
    return 0;
  }

  const auto rounded = static_cast<long long>(std::llround(scaled_value));
  if (rounded > std::numeric_limits<int16_t>::max()) {
    return std::numeric_limits<int16_t>::max();
  }
  if (rounded < std::numeric_limits<int16_t>::min()) {
    return std::numeric_limits<int16_t>::min();
  }
  return static_cast<int16_t>(rounded);
}

inline void encode_i16_be(int16_t value, uint8_t & high, uint8_t & low) noexcept
{
  const auto encoded = static_cast<uint16_t>(value);
  high = static_cast<uint8_t>((encoded >> 8U) & 0xFFU);
  low = static_cast<uint8_t>(encoded & 0xFFU);
}

inline int16_t decode_i16_be(uint8_t high, uint8_t low) noexcept
{
  const auto encoded = static_cast<uint16_t>((static_cast<uint16_t>(high) << 8U) | static_cast<uint16_t>(low));
  return static_cast<int16_t>(encoded);
}

inline float decode_vel_mps_be(uint8_t high, uint8_t low) noexcept
{
  constexpr float k_inv_1000 = 0.001F;
  return static_cast<float>(decode_i16_be(high, low)) * k_inv_1000;
}

inline std::array<uint8_t, SEND_DATA_SIZE> build_command_frame(
  double linear_x_mps,
  double linear_y_mps,
  double angular_z_radps) noexcept
{
  constexpr double k_milli_units = 1000.0;

  std::array<uint8_t, SEND_DATA_SIZE> frame{};
  frame[0] = FRAME_HEADER;
  frame[1] = 0;
  frame[2] = 0;

  encode_i16_be(saturate_i16_from_scaled(linear_x_mps, k_milli_units), frame[3], frame[4]);
  encode_i16_be(saturate_i16_from_scaled(linear_y_mps, k_milli_units), frame[5], frame[6]);
  encode_i16_be(saturate_i16_from_scaled(angular_z_radps, k_milli_units), frame[7], frame[8]);

  frame[9] = xor_checksum(frame.data(), SEND_DATA_SIZE - 2U);
  frame[10] = FRAME_TAIL;
  return frame;
}

}  // namespace duojin01
