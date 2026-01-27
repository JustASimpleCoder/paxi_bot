#include <gtest/gtest.h>

#include "paxi_hardware/encoder.hpp"

// Test fixture for encoder tests
class EncoderKinematicsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    encoder = std::make_unique<paxi_hardware::EncoderKinematics>();
  }

  std::unique_ptr<paxi_hardware::EncoderKinematics> encoder;
};

TEST(PaxiHardwareTest, sanity_check)
{
  ASSERT_EQ(4, 2 + 2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
