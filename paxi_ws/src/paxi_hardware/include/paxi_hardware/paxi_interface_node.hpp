#ifndef PAXI_INTERFACE__NODE_HPP
#define PAXI_INTERFACE__NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "paxi_hardware/utility.hpp"

//#include <array>
#include <memory>
//#include <numbers>



namespace paxi_hardware{


    class PaxiInterfaceNode : public rclcpp::Node{
        public:
            PaxiInterfaceNode();
            ~PaxiInterfaceNode() = default;
            template<typename MsgT>
            void publish_data(const std::shared_ptr<typename rclcpp::Publisher<MsgT>>& pub, const MsgT& msg){
                pub->publish(msg);
            }

            template<typename MsgT, typename ValueT>
            void publish_data(const std::shared_ptr<typename rclcpp::Publisher<MsgT>>& pub, const ValueT& value){
                MsgT msg;
                msg.data = value;
                pub->publish(msg);
            }

            template<typename MsgT, typename ValLeftT, typename ValRightT>
            void publish_data(
                const std::array<std::shared_ptr<typename rclcpp::Publisher<MsgT>>, WHEEL_COUNT> &pub, 
                const ValLeftT& l_value, 
                const ValRightT& r_value
            )
            {
                static_assert(WHEEL_COUNT == 2, "Wheel count needs to be 2, please check enum class Wheel");
                publish_data(pub[to_index(Wheel::LEFT)],  l_value);
                publish_data(pub[to_index(Wheel::RIGHT)], r_value);
            }

            const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> get_position_pubs() const;
            const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> get_velocity_pubs() const;
            const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> get_command_pubs() const;

            const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr get_voltage_pubs() const;
            const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr get_temp_pubs() const;
            const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr get_connected_pubs() const;
            const rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr get_imu_pubs() const;
            
        private:

            std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> position_pubs_;
            std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> velocity_pubs_;
            std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> command_pubs_;
            
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pubs_;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pubs_;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_pubs_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pubs_;
        };

}//end of namepsace paxi_hardware
#endif