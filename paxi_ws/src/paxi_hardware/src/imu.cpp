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

    bool ImuProcessing::set_imu_link_name(const std::string & link_name){
        if(link_name == "" ){
            return false;
        }
        imu_link_name_ = link_name;
        return true;
    }
    
    void ImuProcessing::update_imu(const rclcpp::Time time, const SerialFeedback &feedback){

        imu_msg_.header.stamp = time;

        imu_msg_.angular_velocity.x = feedback.gyro_x;
        imu_msg_.angular_velocity.y = feedback.gyro_y;
        imu_msg_.angular_velocity.z = feedback.gyro_z;

        imu_msg_.linear_acceleration.x = feedback.accel_x;
        imu_msg_.linear_acceleration.y = feedback.accel_y;
        imu_msg_.linear_acceleration.z = feedback.accel_z;

        imu_msg_.orientation.w = static_cast<double>(feedback.quat_w) / Q30;
        imu_msg_.orientation.x = static_cast<double>(feedback.quat_x) / Q30;
        imu_msg_.orientation.y = static_cast<double>(feedback.quat_y) / Q30;
        imu_msg_.orientation.z = static_cast<double>(feedback.quat_z) / Q30;

    }

} // end of namespace paxi_hardware