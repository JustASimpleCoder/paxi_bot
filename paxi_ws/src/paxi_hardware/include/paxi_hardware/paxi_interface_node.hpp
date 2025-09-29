#ifndef PAXI_INTERFACE__NODE_HPP
#define PAXI_INTERFACE__NODE_HPP

#include <memory>

#include "paxi_hardware/hoverboard_protocol_struct.hpp"
#include "paxi_hardware/utility.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

namespace paxi_hardware
{
    class PaxiInterfaceNode : public rclcpp::Node{
      public:

        PaxiInterfaceNode();
        ~PaxiInterfaceNode() = default;

        template <typename MsgT>
        void publish_data(
            const std::shared_ptr<typename rclcpp::Publisher<MsgT>> & pub, 
            const MsgT & msg) const
        {
            pub->publish(msg);
        }

        template <typename MsgT, typename ValueT>
        void publish_data(
            const std::shared_ptr<typename rclcpp::Publisher<MsgT>> & pub, 
            const ValueT & value) const
        {
            MsgT msg;
            msg.data = value;
            pub->publish(msg);
        }

        template <typename MsgT, typename ValLeftT, typename ValRightT>
        void publish_data(
            const std::array<std::shared_ptr<typename rclcpp::Publisher<MsgT>>, WHEEL_COUNT> & pub,
            const ValLeftT & l_value, 
            const ValRightT & r_value) const
        {
          static_assert(WHEEL_COUNT == 2, "Wheel count needs to be 2, please check enum class Wheel");
          publish_data(pub[to_index(Wheel::LEFT)], l_value);
          publish_data(pub[to_index(Wheel::RIGHT)], r_value);
        }

        void publish_real_time( const SerialFeedback& feedback, 
                                bool connected, 
                                const sensor_msgs::msg::Imu & imu_msg,
                                const std::vector<double> & state_positions) const;

      private:
        std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> position_pubs_;
        std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> velocity_pubs_;
        std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> command_pubs_;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pubs_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pubs_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_pubs_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pubs_;
    };

}  // namespace paxi_hardware
#endif