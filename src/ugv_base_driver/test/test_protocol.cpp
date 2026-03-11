#include <gtest/gtest.h>

#include "ugv_base_driver/protocol.hpp"

TEST(Protocol, SaturateScaledValues)
{
  EXPECT_EQ(duojin01::saturate_i16_from_scaled(1.0, 1000.0), 1000);
  EXPECT_EQ(duojin01::saturate_i16_from_scaled(-0.5, 1000.0), -500);
  EXPECT_EQ(
    duojin01::saturate_i16_from_scaled(1e9, 1000.0),
    std::numeric_limits<int16_t>::max());
}

TEST(Protocol, EncodeDecodeRoundTrip)
{
  uint8_t high = 0;
  uint8_t low = 0;
  duojin01::encode_i16_be(-1234, high, low);
  EXPECT_EQ(duojin01::decode_i16_be(high, low), -1234);
}

TEST(Protocol, BuildCommandFrame)
{
  const auto frame = duojin01::build_command_frame(1.0, -0.5, 0.25);

  EXPECT_EQ(frame[0], duojin01::FRAME_HEADER);
  EXPECT_EQ(frame[10], duojin01::FRAME_TAIL);
  EXPECT_EQ(duojin01::decode_i16_be(frame[3], frame[4]), 1000);
  EXPECT_EQ(duojin01::decode_i16_be(frame[5], frame[6]), -500);
  EXPECT_EQ(duojin01::decode_i16_be(frame[7], frame[8]), 250);
  EXPECT_EQ(frame[9], duojin01::xor_checksum(frame.data(), duojin01::SEND_DATA_SIZE - 2U));
}

TEST(Protocol, BuildStopFrame)
{
  const auto frame = duojin01::build_command_frame(0.0, 0.0, 0.0);
  EXPECT_EQ(duojin01::decode_i16_be(frame[3], frame[4]), 0);
  EXPECT_EQ(duojin01::decode_i16_be(frame[5], frame[6]), 0);
  EXPECT_EQ(duojin01::decode_i16_be(frame[7], frame[8]), 0);
}
