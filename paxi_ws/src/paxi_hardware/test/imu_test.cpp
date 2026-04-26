// Copyright 2026 JustASimpleCoder
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "test/imu_test.hpp"

namespace paxi_hardware
{

using paxi_common::math::DEG_TO_RAD;

const SerialFeedback ImuFeedbackTest::create_serial_feedback(std::int16_t val, std::uint16_t uval)
{
  SerialFeedback feedback;
  feedback.start = uval;
  feedback.cmd_l = val;
  feedback.cmd_r = val;
  feedback.speed_r_meas = val;
  feedback.speed_l_meas = val;
  feedback.bat_voltage = val;
  feedback.board_temp = val;
  feedback.gyro_x = val;
  feedback.gyro_y = val;
  feedback.gyro_z = val;
  feedback.accel_x = val;
  feedback.accel_y = val;
  feedback.accel_z = val;
  feedback.quat_w_low = uval;
  feedback.quat_w_high = val;
  feedback.quat_x_low = uval;
  feedback.quat_x_high = val;
  feedback.quat_y_low = uval;
  feedback.quat_y_high = val;
  feedback.quat_z_low = uval;
  feedback.quat_z_high = val;
  feedback.euler_pitch = val;
  feedback.euler_roll = val;
  feedback.euler_yaw = val;
  feedback.temperature = val;
  feedback.sensors = uval;
  feedback.cmd_led = uval;
  feedback.checksum = uval;
  return feedback;
}


TEST_F(ImuSettersTest, SetLinkName)
{
  std::string empty_link_name = "";
  std::string non_empty_link_name = "imu_link";

  EXPECT_FALSE(imu_->set_imu_link_name(empty_link_name));
  EXPECT_TRUE(imu_->set_imu_link_name(non_empty_link_name));
}


TEST_F(ImuAllZeroFeedbackTest, SetLinkName)
{
  SerialFeedback all_zero_feedback;
  all_zero_feedback.start = 0;
  all_zero_feedback.cmd_l = 0;
  all_zero_feedback.cmd_r = 0;
  all_zero_feedback.speed_r_meas = 0;
  all_zero_feedback.speed_l_meas = 0;
  all_zero_feedback.bat_voltage = 0;
  all_zero_feedback.board_temp = 0;
  all_zero_feedback.gyro_x = 0;
  all_zero_feedback.gyro_y = 0;
  all_zero_feedback.gyro_z = 0;
  all_zero_feedback.accel_x = 0;
  all_zero_feedback.accel_y = 0;
  all_zero_feedback.accel_z = 0;
  all_zero_feedback.quat_w_low = 0;
  all_zero_feedback.quat_w_high = 0;
  all_zero_feedback.quat_x_low = 0;
  all_zero_feedback.quat_x_high = 0;
  all_zero_feedback.quat_y_low = 0;
  all_zero_feedback.quat_y_high = 0;
  all_zero_feedback.quat_z_low = 0;
  all_zero_feedback.quat_z_high = 0;
  all_zero_feedback.euler_pitch = 0;
  all_zero_feedback.euler_roll = 0;
  all_zero_feedback.euler_yaw = 0;
  all_zero_feedback.temperature = 0;
  all_zero_feedback.sensors = 0;
  all_zero_feedback.cmd_led = 0;
  all_zero_feedback.checksum = 0;

  EXPECT_TRUE(imu_->is_all_zero_imu_data(all_zero_feedback));
  all_zero_feedback.accel_x = 1;
  EXPECT_FALSE(imu_->is_all_zero_imu_data(all_zero_feedback));
}

TEST_P(ImuUpdateTest, UpdateTime)
{
  rclcpp::Time time{0};
  time += std::chrono::seconds(10 * GetParam());
  imu_->update_imu_msg_time(time);
  EXPECT_EQ(imu_->get_imu_msg().header.stamp, time);
}

TEST_P(ImuSetLinkNameTest, UpdateEncoderConstRPM)
{
  inline const char * test_link_names = GetParam();
  imu_->set_imu_link_name(test_link_names);

  const sensor_msgs::msg::Imu imu_msg = imu_->get_imu_msg();
  EXPECT_EQ(imu_msg.header.frame_id, test_link_names);
}

TEST_P(ImuFeedbackTest, UpdateMsgFromFeedback)
{
  std::pair<std::int16_t, std::uint16_t> pair = GetParam();
  SerialFeedback test_feedback = create_serial_feedback(pair.first, pair.second);
  imu_->update_imu_msg_data(test_feedback);

  sensor_msgs::msg::Imu test_imu_msg;
  test_imu_msg.header.frame_id = "imu_hover";
  test_imu_msg.orientation_covariance = {1e-7, 0, 0, 0, 1e-7, 0, 0, 0, 1e-7};
  test_imu_msg.angular_velocity_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};
  test_imu_msg.linear_acceleration_covariance = {1e-5, 0, 0, 0, 1e-5, 0, 0, 0, 1e-5};

  if (imu_->is_all_zero_imu_data(test_feedback)) {
    // super unlikely imu data will actually be all zero (always some accel)
    // used to indicate a bad data read/smooths imu data overall
    GTEST_SKIP();
  }

  // Fixed point conversion scaling factor 2^30 (float to 32 bit signed int)
  static constexpr double Q30 = static_cast<double>(1 << 30);  // 1073741824.0;

  // Converts raw acceleration data (m/s^2) from the MPU6050 to approriate gravity units
  // (16,384 LSB/g)
  static constexpr double ACCEL_TO_G = 16384.00;

  // Converts raw gyro data (degree / S units) from the MPU6050 to degree per second
  // (16.4 LSB/(degree/s))
  static constexpr double GYRO_TO_DEG_S = 16.4;

  // Standard gravity constant 9.81 m/s^2
  static constexpr double STD_GRAVITY = 9.81;

  // IMU's sideboard computes quaternions as as floats between [-1,1]. Using fixed-point
  // conversion, with scaling factor Q30 = 2^30, qauternions are represented as 32 bits. The
  // communication feedback protocol requires all data in the feedback structure ot be same size
  // so we send as low/ high bit, where the low bit is a unsigned integer to maintain data and
  // not accidently interpret first bit 1 as a negative
  auto recover_quat_32_bit = [](std::int16_t high, std::uint16_t low) -> std::int32_t {
      return (static_cast<std::int32_t>(high) << 16) | static_cast<std::int32_t>(low);
    };

  double q_w =
    static_cast<double>(recover_quat_32_bit(
      test_feedback.quat_w_high,
      test_feedback.quat_w_low)) / Q30;
  double q_x =
    static_cast<double>(recover_quat_32_bit(
      test_feedback.quat_x_high,
      test_feedback.quat_x_low)) / Q30;
  double q_y =
    static_cast<double>(recover_quat_32_bit(
      test_feedback.quat_y_high,
      test_feedback.quat_y_low)) / Q30;
  double q_z =
    static_cast<double>(recover_quat_32_bit(
      test_feedback.quat_z_high,
      test_feedback.quat_z_low)) / Q30;

  test_imu_msg.angular_velocity.x = static_cast<double>(test_feedback.gyro_x) / GYRO_TO_DEG_S *
    DEG_TO_RAD;
  test_imu_msg.angular_velocity.y = static_cast<double>(test_feedback.gyro_y) / GYRO_TO_DEG_S *
    DEG_TO_RAD;
  test_imu_msg.angular_velocity.z = static_cast<double>(test_feedback.gyro_z) / GYRO_TO_DEG_S *
    DEG_TO_RAD;

  // imu hover seems to be off by factor of 10
  test_imu_msg.angular_velocity.x /= 10.0;
  test_imu_msg.angular_velocity.y /= 10.0;
  test_imu_msg.angular_velocity.z /= 10.0;

  test_imu_msg.linear_acceleration.x = static_cast<double>(test_feedback.accel_x) / ACCEL_TO_G *
    STD_GRAVITY;
  test_imu_msg.linear_acceleration.y = static_cast<double>(test_feedback.accel_y) / ACCEL_TO_G *
    STD_GRAVITY;
  test_imu_msg.linear_acceleration.z = static_cast<double>(test_feedback.accel_z) / ACCEL_TO_G *
    STD_GRAVITY;

  test_imu_msg.orientation.w = q_w;
  test_imu_msg.orientation.x = q_x;
  test_imu_msg.orientation.y = q_y;
  test_imu_msg.orientation.z = q_z;

  EXPECT_EQ(test_imu_msg, imu_->get_imu_msg());
}

INSTANTIATE_TEST_SUITE_P(
  SetLinkName, ImuSetLinkNameTest, ::testing::Values("test1234", "header", "imu", "imu_hover"));

INSTANTIATE_TEST_SUITE_P(
  UpdateImuMsg, ImuFeedbackTest, ::testing::Values(
    std::make_pair<std::int16_t, std::uint16_t>(0, 0),
    std::make_pair<std::int16_t, std::uint16_t>(1, 1),
    std::make_pair<std::int16_t, std::uint16_t>(5, 5),
    std::make_pair<std::int16_t, std::uint16_t>(100, 100),
    std::make_pair<std::int16_t, std::uint16_t>(1000, 1000),
    std::make_pair<std::int16_t, std::uint16_t>(10000, 10000),
    std::make_pair<std::int16_t, std::uint16_t>(INT16_MAX, UINT16_MAX),
    std::make_pair<std::int16_t, std::uint16_t>(-1, 0),
    std::make_pair<std::int16_t, std::uint16_t>(-1, 1),
    std::make_pair<std::int16_t, std::uint16_t>(-5, 5),
    std::make_pair<std::int16_t, std::uint16_t>(-100, 100),
    std::make_pair<std::int16_t, std::uint16_t>(-1000, 1000),
    std::make_pair<std::int16_t, std::uint16_t>(-10000, 10000),
    std::make_pair<std::int16_t, std::uint16_t>(INT16_MIN, UINT16_MAX))
);
}  // namespace paxi_hardware

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
