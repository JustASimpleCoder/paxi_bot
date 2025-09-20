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

            void update_imu(const rclcpp::Time time, const SerialFeedback &feedback);

            bool set_imu_link_name(const std::string & link_name);

            inline sensor_msgs::msg::Imu get_imu_msg() const {return imu_msg_;}

        private:
           sensor_msgs::msg::Imu imu_msg_;
           std::string imu_link_name_;

    };

}//end of namespace paxi_hardware

#endif