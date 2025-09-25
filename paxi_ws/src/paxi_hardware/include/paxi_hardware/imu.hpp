#ifndef IMU_HPP
#define IMU_HPP

#include <string>

#include "paxi_hardware/hoverboard_protocol_struct.hpp"
#include "paxi_hardware/utility.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"


namespace paxi_hardware
{



class ImuProcessing
{
  public:
      ImuProcessing();
      ~ImuProcessing() = default;

      ImuProcessing(const ImuProcessing &) = delete;
      ImuProcessing & operator=(const ImuProcessing &) = delete;

      ImuProcessing(ImuProcessing &&) noexcept = default;
      ImuProcessing & operator=(ImuProcessing &&) noexcept = default;

      void update_imu(const rclcpp::Time time, const SerialFeedback & feedback);
      bool set_imu_link_name(const std::string & link_name);

      inline sensor_msgs::msg::Imu get_imu_msg() const { return imu_msg_; }
      inline bool is_all_zero_imu_data(const SerialFeedback & feedback) const{
        return !(
          feedback.gyro_x | feedback.gyro_y | feedback.gyro_z | feedback.accel_x | feedback.accel_y |
          feedback.accel_z | feedback.quat_w_low | feedback.quat_x_low | feedback.quat_y_low |
          feedback.quat_z_low | feedback.quat_w_high | feedback.quat_x_high | feedback.quat_y_high |
          feedback.quat_z_high);
        // Bitwise 'OR' operation on IMU feedback data will result in zero if all bits are zeroed bits.
        // Negating result gives us true if its all zero, false otherwise.
      }

  private:
      sensor_msgs::msg::Imu imu_msg_;
      std::string imu_link_name_;
  };
}  //end of namespace paxi_hardware

#endif