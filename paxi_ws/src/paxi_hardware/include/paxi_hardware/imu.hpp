#ifndef IMU_HPP
#define IMU_HPP

#include <string>

#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"


#include "paxi_hardware/hoverboard_protocol_struct.hpp"
#include "paxi_hardware/utility.hpp"

namespace paxi_hardware{

    class ImuProcessing{
        public:

            ImuProcessing();
            ~ImuProcessing() = default;

            void update_imu(const rclcpp::Time time, const SerialFeedback& feedback);
            bool is_all_zero_imu_data(const SerialFeedback &feedbac);
            bool set_imu_link_name(const std::string& link_name);

            inline sensor_msgs::msg::Imu get_imu_msg() const {return imu_msg_;}
            inline bool is_all_zero_imu_data(const SerialFeedback& feedback) const{
                return !(feedback.gyro_x  | feedback.gyro_y  | feedback.gyro_z  |     
                        feedback.accel_x | feedback.accel_y | feedback.accel_z |  
                        feedback.quat_w  | feedback.quat_x  | feedback.quat_y  | feedback.quat_z );    
            }


        private:
           sensor_msgs::msg::Imu imu_msg_;
           std::string imu_link_name_;
    };
}//end of namespace paxi_hardware

#endif