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

TEST_F(EncoderKinematicsTest, GetterInitValues)
{
  /// Test getters after initialization should be zero
  EXPECT_DOUBLE_EQ(encoder->get_hover_speed(), 0.0);
  EXPECT_DOUBLE_EQ(encoder->get_hover_steer(), 0.0);
}

TEST_F(EncoderKinematicsTest, SetVelocity)
{
  EXPECT_TRUE(encoder->set_max_velocity(1.0));
  EXPECT_FALSE(encoder->set_max_velocity(-2.4));
}

TEST_F(EncoderKinematicsTest, SetWheelRadius)
{
  EXPECT_TRUE(encoder->set_wheel_radius(0.7));
  EXPECT_FALSE(encoder->set_wheel_radius(-2.4));
}

TEST_F(EncoderKinematicsTest, SetWheelSeparation)
{
  EXPECT_TRUE(encoder->set_wheel_separation(0.5));
  EXPECT_FALSE(encoder->set_wheel_separation(-2.4));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
