#include <gtest/gtest.h>
#include "paxi_hardware/encoder.hpp"
//#include "paxi_hardware/utility.hpp"

// Test fixture for encoder_kintests
namespace paxi_hardware
{
class EncoderKinematicsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    encoder_kin= std::make_unique<paxi_hardware::EncoderKinematics>();
  }

  std::unique_ptr<paxi_hardware::EncoderKinematics> encoder_kin;
};

TEST(PaxiHardwareTest, sanity_check)
{
  ASSERT_EQ(4, 2 + 2);
}

TEST_F(EncoderKinematicsTest, GetterInitValues)
{
  /// Test getters after initialization should be zero
  EXPECT_DOUBLE_EQ(encoder_kin->get_hover_speed(), 0.0);
  EXPECT_DOUBLE_EQ(encoder_kin->get_hover_steer(), 0.0);
}

TEST_F(EncoderKinematicsTest, SetVelocity)
{
  EXPECT_TRUE(encoder_kin->set_max_velocity(1.0));
  EXPECT_FALSE(encoder_kin->set_max_velocity(-2.4));
}

TEST_F(EncoderKinematicsTest, SetWheelRadius)
{
  EXPECT_TRUE(encoder_kin->set_wheel_radius(0.7));
  EXPECT_FALSE(encoder_kin->set_wheel_radius(-2.4));
}

TEST_F(EncoderKinematicsTest, SetWheelSeparation)
{
  EXPECT_TRUE(encoder_kin->set_wheel_separation(0.5));
  EXPECT_FALSE(encoder_kin->set_wheel_separation(-2.4));
}

class EncoderKinematicsConstRPMTest : public EncoderKinematicsTest,
  public ::testing::WithParamInterface<int>
{};

TEST_P(EncoderKinematicsConstRPMTest, UpdateEncoderConstRPM)
{
  // dummy state positions, should be zero to start
  std::vector<double, std::allocator<double>> state_positions{0, 0};

  //assume constant 10 revolutions per minute should so accumalted position is constant
  const double delta_time_change = 60; //60 seconds to somulate a minute
  const int16_t constant_rpm = GetParam(); // get test parameters
  const double omega = constant_rpm * RPM_TO_RAD_S;
  double expected_position = 0.0; // initilize for now but will change after each for loop after initialization
  rclcpp::Time time = rclcpp::Time{0, 0};

  // set random wheel seperation for dummy callculations
  const double wheel_separation = 1.0;

  encoder_kin->set_wheel_separation(wheel_separation);

  // First read nothing should be udpated, should initilize time & not update state positions
  // Assume were already moving at the constant rpm for first pass
  encoder_kin->update_angular_position(time, constant_rpm, constant_rpm, state_positions);
  EXPECT_DOUBLE_EQ(state_positions[to_index(Wheel::LEFT)], 0.0);
  EXPECT_DOUBLE_EQ(state_positions[to_index(Wheel::RIGHT)], 0.0);

  // expected anuglar position is based on v=r*omega = angular_pos/time so angular_pos = omega*time
  for (std::size_t i = 0u; i < 100; ++i) {
    time += rclcpp::Duration::from_seconds(delta_time_change);
    encoder_kin->update_angular_position(time, constant_rpm, constant_rpm, state_positions);
    expected_position += omega * delta_time_change;
    EXPECT_NEAR(state_positions[to_index(Wheel::LEFT)], expected_position, 1e-6);
    EXPECT_NEAR(state_positions[to_index(Wheel::RIGHT)], expected_position, 1e-6);
  }
}

INSTANTIATE_TEST_SUITE_P(
  ForwardBackwardConstRPM, EncoderKinematicsConstRPMTest,
  ::testing::Values(1, -1, 2, -2));

}//end of paxi_hardware namepace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
