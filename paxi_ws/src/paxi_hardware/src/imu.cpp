#include "paxi_hardware/imu.hpp"


namespace paxi_hardware
{

  ImuProcessing::ImuProcessing(){
    imu_msg_.header.frame_id = imu_link_name_;

    imu_msg_.orientation_covariance = {
      0.1,  0,    0,
      0,    0.1,  0, 
      0,    0,    0.1,
    };

    imu_msg_.angular_velocity_covariance = {
      0.1,  0,    0,
      0,    0.1,  0,
      0,    0,    0.1,
    };

    imu_msg_.linear_acceleration_covariance = {
      0.1,  0,    0,
      0,    0.1,  0,
      0,    0,    0.1,
    };
  }

  bool ImuProcessing::set_imu_link_name(const std::string & link_name){
    if (link_name == "") {
      return false;
    }
    imu_link_name_ = link_name;
    imu_msg_.header.frame_id = imu_link_name_;
    return true;
  }

  void ImuProcessing::update_imu(const rclcpp::Time time, const SerialFeedback & feedback){
      auto recover_quat_32_bit = [](int16_t high, int16_t low) -> int32_t {
        return (static_cast<int32_t>(high) << 16) | static_cast<int32_t>(low);
      };

      auto validate_quaternion = [](double w, double x, double y, double z) -> bool{
          if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)){
            return false;
          }

          double norm = std::sqrt(w*w + x*x + y*y + z*z);
          // valid quaternion will have a norm of 1 (its okay its is slightly off from 1)
          if(norm < 0.9 || norm > 1.1) return false;
          
          return true;
      };

      if (is_all_zero_imu_data(feedback)) {
            return;
      }

      double q_w = static_cast<double>(recover_quat_32_bit(feedback.quat_w_high, feedback.quat_w_low)) / Q30;
      double q_x = static_cast<double>(recover_quat_32_bit(feedback.quat_x_high, feedback.quat_x_low)) / Q30;
      double q_y = static_cast<double>(recover_quat_32_bit(feedback.quat_y_high, feedback.quat_y_low)) / Q30;
      double q_z = static_cast<double>(recover_quat_32_bit(feedback.quat_z_high, feedback.quat_z_low)) / Q30;
      
      if(!validate_quaternion(q_w, q_x, q_y, q_z)){
          return;
      }

      imu_msg_.header.stamp = time;
      imu_msg_.angular_velocity.x = static_cast<double>(feedback.gyro_x) / GYRO_TO_DEG_S * (M_PI / 180.0);
      imu_msg_.angular_velocity.y = static_cast<double>(feedback.gyro_y) / GYRO_TO_DEG_S * (M_PI / 180.0);
      imu_msg_.angular_velocity.z = static_cast<double>(feedback.gyro_z) / GYRO_TO_DEG_S * (M_PI / 180.0);

      imu_msg_.linear_acceleration.x = static_cast<double>(feedback.accel_x) / ACCEL_TO_G * 9.81;
      imu_msg_.linear_acceleration.y = static_cast<double>(feedback.accel_y) / ACCEL_TO_G * 9.81;
      imu_msg_.linear_acceleration.z = static_cast<double>(feedback.accel_z) / ACCEL_TO_G * 9.81;

      imu_msg_.orientation.w = q_w;
      imu_msg_.orientation.x = q_x;
      imu_msg_.orientation.y = q_y;
      imu_msg_.orientation.z = q_z;
  }
}  // end of namespace paxi_hardware