#include "paxi_hardware/encoder.hpp"

namespace paxi_hardware
{
EncoderKinematics::EncoderKinematics()
: wheel_radius_{0.1},        // cannot be zero cause we are dividing these numbers
  wheel_separation_{0.1},    // cannot be zero cause we are dividing these numbers
  max_velocity_{0.0},
  wheel_omega_l_{0.0},
  wheel_omega_r_{0.0},
  wheel_vel_l_{0.0},
  wheel_vel_r_{0.0},
  hoverboard_steer_{0.0},
  hoverboard_speed_{0.0},
  prev_l_rad_per_sec_{0},
  prev_r_rad_per_sec_{0},
  first_read_enc_{true},
  last_read_time_enc_{0}
{}

void EncoderKinematics::update_encoders(
  const rclcpp::Time & time,
  int16_t r_rpm,
  int16_t l_rpm,
  std::vector<double> & state_positions)
{
  if (first_read_enc_) {
    prev_l_rad_per_sec_ = l_rpm * RPM_TO_RAD_S;
    prev_r_rad_per_sec_ = r_rpm * RPM_TO_RAD_S;
    last_read_time_enc_ = time;
    first_read_enc_ = false;
    return;
  }

  const double delta_time = time.seconds() - last_read_time_enc_.seconds();
  last_read_time_enc_ = time;

  if (delta_time <= 0.0) {
    RCLCPP_WARN(
      rclcpp::get_logger(LOGGER_ENCODER),
      "Failed to update encoder as delta time is negative or zero [%f]",
      delta_time
    );
    return;
  }

  const double l_rad_per_sec = l_rpm * RPM_TO_RAD_S;
  const double r_rad_per_sec = r_rpm * RPM_TO_RAD_S;

  const double avg_l_rad_per_sec = (prev_l_rad_per_sec_ + l_rad_per_sec) / 2.0;
  const double avg_r_rad_per_sec = (prev_r_rad_per_sec_ + r_rad_per_sec) / 2.0;

  const double delta_l_pos = avg_l_rad_per_sec * delta_time * wheel_radius_;
  const double delta_r_pos = avg_r_rad_per_sec * delta_time * wheel_radius_;

  state_positions[to_index(Wheel::LEFT)] += delta_l_pos;
  state_positions[to_index(Wheel::RIGHT)] += delta_r_pos;

  prev_l_rad_per_sec_ = l_rad_per_sec;
  prev_r_rad_per_sec_ = r_rad_per_sec;
}

void EncoderKinematics::forward_kinematics(const std::vector<double> & hw_commands)
{
  wheel_omega_l_ = hw_commands[to_index(Wheel::LEFT)];
  wheel_omega_r_ = hw_commands[to_index(Wheel::RIGHT)];

  wheel_vel_l_ = wheel_omega_l_ * wheel_radius_;
  wheel_vel_r_ = wheel_omega_r_ * wheel_radius_;

  hoverboard_speed_ = (wheel_vel_r_ + wheel_vel_l_) / 2.0;
  hoverboard_steer_ = (wheel_vel_r_ - wheel_vel_l_) / wheel_separation_;
}

bool EncoderKinematics::set_wheel_radius(double radius)
{
  if (radius < 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(LOGGER_ENCODER),
      "Failed to set encoder radius as received value [%.3f] is negative", radius);

    return false;
  }

  wheel_radius_ = radius;
  return true;
}

bool EncoderKinematics::set_wheel_separation(double separation)
{
  if (separation < 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(LOGGER_ENCODER),
      "Failed to set encoder wheel separation as received value [%.3f] is negative", separation);

    return false;
  }

  wheel_separation_ = separation;
  return true;
}

bool EncoderKinematics::set_max_velocity(double velocity)
{
  if (velocity < 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(LOGGER_ENCODER),
      "Failed to set encoder max velocity as received value [%.3f] is negative", velocity);

    return false;
  }

  max_velocity_ = velocity;
  return true;
}
}  // end of namespace paxi_hardware
