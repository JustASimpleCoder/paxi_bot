#include "paxi_hardware/imu.hpp"

namespace paxi_hardware
{

   ImuProcessing::ImuProcessing()
   {
        imu_msg_.header.frame_id = imu_link_name_;

        imu_msg_.orientation_covariance = {
            0.1,    0,      0,  
            0,      0.1,    0, 
            0,      0,      0.1,
        };

        imu_msg_.angular_velocity_covariance = {
            0.1,    0,      0,  
            0,      0.1,    0, 
            0,      0,      0.1,
        };

        imu_msg_.linear_acceleration_covariance = {
            0.1,    0,      0,  
            0,      0.1,    0, 
            0,      0,      0.1,
        };
    } 


    bool ImuProcessing::set_imu_link_name(const std::string& link_name){
        if(link_name == "" ){
            return false;
        }
        imu_link_name_ = link_name;
        imu_msg_.header.frame_id = imu_link_name_;
        return true;
    }

    void ImuProcessing::update_imu(const rclcpp::Time time, const SerialFeedback& feedback){

        if(is_all_zero_imu_data(feedback)){
            return;
        }

        imu_msg_.header.stamp = time;
        imu_msg_.angular_velocity.x = static_cast<double>(feedback.gyro_x) / GYRO_TO_DEG_S * (M_PI / 180.0);
        imu_msg_.angular_velocity.y = static_cast<double>(feedback.gyro_y) / GYRO_TO_DEG_S * (M_PI / 180.0);
        imu_msg_.angular_velocity.z = static_cast<double>(feedback.gyro_z) / GYRO_TO_DEG_S * (M_PI / 180.0);
        
        imu_msg_.linear_acceleration.x = static_cast<double>(feedback.accel_x) / ACCEL_TO_G * 9.81;
        imu_msg_.linear_acceleration.y = static_cast<double>(feedback.accel_y) / ACCEL_TO_G * 9.81;
        imu_msg_.linear_acceleration.z = static_cast<double>(feedback.accel_z) / ACCEL_TO_G * 9.81;
        
        // recover quatersions -> stored in MCU as 32 bits, protocol only allows to send 16 bit data packets, 
        // send as two 16 bits, "high" upper 16 bits into quat_<axis>_high and "low" lower 16 bits into quat_<axis>_low

        int32_t quat_32_w, quat_32_x, quat_32_y, quat_32_z;
        quat_32_w = (static_cast<int32_t>(feedback.quat_w_high) << 16) | static_cast<int32_t>(feedback.quat_w_low);
        quat_32_x = (static_cast<int32_t>(feedback.quat_x_high) << 16) | static_cast<int32_t>(feedback.quat_x_low);
        quat_32_y = (static_cast<int32_t>(feedback.quat_y_high) << 16) | static_cast<int32_t>(feedback.quat_y_low);
        quat_32_z = (static_cast<int32_t>(feedback.quat_z_high) << 16) | static_cast<int32_t>(feedback.quat_z_low);

        imu_msg_.orientation.w = static_cast<double>(quat_32_w) / Q30;
        imu_msg_.orientation.x = static_cast<double>(quat_32_x) / Q30;
        imu_msg_.orientation.y = static_cast<double>(quat_32_y) / Q30;
        imu_msg_.orientation.z = static_cast<double>(quat_32_z) / Q30;
    }
} // end of namespace paxi_hardware