#include "paxi_hardware/paxi_interface_node.hpp"

namespace paxi_hardware{

    PaxiInterfaceNode::PaxiInterfaceNode() : Node("paxi_interface_node")  {

        velocity_pubs_[to_index(Wheel::LEFT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/left_wheel/velocity", 3);
        velocity_pubs_[to_index(Wheel::RIGHT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/right_wheel/velocity", 3);

        position_pubs_[to_index(Wheel::LEFT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/left_wheel/position", 3);
        position_pubs_[to_index(Wheel::RIGHT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/right_wheel/position", 3);

        command_pubs_[to_index(Wheel::LEFT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/left_wheel/cmd", 3);
        command_pubs_[to_index(Wheel::RIGHT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/right_wheel/cmd", 3);
        
        voltage_pubs_ = this->create_publisher<std_msgs::msg::Float64>("paxi/battery_voltage", 3);
        temp_pubs_ = this->create_publisher<std_msgs::msg::Float64>("paxi/temperature", 3);     
        connected_pubs_ = this->create_publisher<std_msgs::msg::Bool>("paxi/connected", 3);
    }

    const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> PaxiInterfaceNode::get_position_pubs() const
    {
        return position_pubs_;
    }

    const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> PaxiInterfaceNode::get_velocity_pubs() const
    {
        return velocity_pubs_;
    }
    const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> PaxiInterfaceNode::get_command_pubs() const
    {
        return command_pubs_;
    }

    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr PaxiInterfaceNode::get_voltage_pubs() const
    {
        return voltage_pubs_;
    }

    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr PaxiInterfaceNode::get_temp_pubs() const
    {
        return temp_pubs_;
    }
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr PaxiInterfaceNode::get_connected_pubs() const
    {
        return connected_pubs_;
    }

}//end of namespace paxi_hardware