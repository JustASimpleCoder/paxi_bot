#include <gtest/gtest.h>

#include "paxi_hardware/encoder.hpp"
#include "paxi_hardware/utility.hpp"

// Test fixture for encoder tests
namespace paxi_hardware
{
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


TEST_F(EncoderKinematicsTest, UpdateEncoderKnownValues)
{
  // dummy state positions, should be zero to start
  std::vector<double, std::allocator<double>> state_positions{0, 0};

  // set random wheel radius/ wheel seperation for dummy callculations
  encoder->set_wheel_radius(1.0);
  encoder->set_wheel_separation(1.0);

  rclcpp::Time time = rclcpp::Time{0, 0};

  //first read nothing should be udpated, should initilize time, assume were already moving at 1.0 rpm for first pass
  encoder->update_encoders(time, 1.0, 1.0, state_positions);
  EXPECT_DOUBLE_EQ(
    state_positions[to_index(Wheel::LEFT)], 0.0);

  EXPECT_DOUBLE_EQ(
    state_positions[to_index(Wheel::RIGHT)], 0.0);

  //assume constant 10 revolutions per minute should so accumalted position is constant
  const double delta_time_change = 60; //60 seconds to somulate a minute
  const double constant_rpm = 1.0;
  const double omega = constant_rpm * RPM_TO_RAD_S;

  time += rclcpp::Duration::from_seconds(delta_time_change);
  encoder->update_encoders(time, constant_rpm, constant_rpm, state_positions);

  double expected_position = omega * delta_time_change; // assume wheel radius is 1

  EXPECT_NEAR(
    state_positions[to_index(Wheel::LEFT)],
    expected_position,
    1e-6
  );
  EXPECT_NEAR(
    state_positions[to_index(Wheel::RIGHT)],
    expected_position,
    1e-6
  );

  for (std::size_t i = 0u; i < 100; ++i) {
    time += rclcpp::Duration::from_seconds(delta_time_change);
    encoder->update_encoders(time, constant_rpm, constant_rpm, state_positions);
    expected_position += omega * delta_time_change;
    EXPECT_NEAR(
      state_positions[to_index(Wheel::LEFT)],
      expected_position,
      1e-6);
    EXPECT_NEAR(
      state_positions[to_index(Wheel::RIGHT)],
      expected_position,
      1e-6);
  }
}
}//end of paci_hardware namepace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
