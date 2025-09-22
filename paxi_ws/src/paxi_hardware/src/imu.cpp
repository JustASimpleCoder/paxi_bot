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
        
        imu_msg_.orientation.w = static_cast<double>(feedback.quat_w) / QUAT_SCALE_FROM_PROTOCOL;
        imu_msg_.orientation.x = static_cast<double>(feedback.quat_x) / QUAT_SCALE_FROM_PROTOCOL;
        imu_msg_.orientation.y = static_cast<double>(feedback.quat_y) / QUAT_SCALE_FROM_PROTOCOL;
        imu_msg_.orientation.z = static_cast<double>(feedback.quat_z) / QUAT_SCALE_FROM_PROTOCOL;
    }
} // end of namespace paxi_hardware