#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "paxi_hardware/utility.hpp"
#include "rclcpp/rclcpp.hpp"

namespace paxi_hardware
{
class EncoderKinematics
{
public:
  EncoderKinematics();
  ~EncoderKinematics() = default;

  EncoderKinematics(const EncoderKinematics &) = delete;
  EncoderKinematics & operator=(const EncoderKinematics &) = delete;

  EncoderKinematics(EncoderKinematics &&) noexcept = default;
  EncoderKinematics & operator=(EncoderKinematics &&) noexcept = default;

  void update_angular_position(
    const rclcpp::Time & time,
    int16_t r_rpm,
    int16_t l_rpm,
    std::vector<double> & state_positions
  );

  bool set_wheel_radius(double radius);
  bool set_max_velocity(double velocity);
  bool set_wheel_separation(double separation);

private:
  double wheel_radius_;
  double wheel_separation_;
  double max_velocity_;

  double wheel_omega_l_;
  double wheel_omega_r_;

  double prev_l_rad_per_sec_;
  double prev_r_rad_per_sec_;

  bool first_read_enc_;
  rclcpp::Time last_read_time_enc_;
};
}  // namespace paxi_hardware
#endif
