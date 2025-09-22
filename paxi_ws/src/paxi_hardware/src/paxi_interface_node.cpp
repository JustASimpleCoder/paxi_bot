#include "paxi_hardware/paxi_interface_node.hpp"

namespace paxi_hardware{

    PaxiInterfaceNode::PaxiInterfaceNode() : Node("paxi_interface_node")  {

        velocity_pubs_[to_index(Wheel::LEFT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/left_wheel/velocity", 3);
        velocity_pubs_[to_index(Wheel::RIGHT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/right_wheel/velocity", 3);

        position_pubs_[to_index(Wheel::LEFT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/left_wheel/position", 3);
        position_pubs_[to_index(Wheel::RIGHT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/right_wheel/position", 3);

        command_pubs_[to_index(Wheel::LEFT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/left_wheel/cmd", 3);
        command_pubs_[to_index(Wheel::RIGHT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/right_wheel/cmd", 3);


        imu_pubs_= this->create_publisher<sensor_msgs::msg::Imu>("paxi/imu_raw", 3);        
        voltage_pubs_ = this->create_publisher<std_msgs::msg::Float64>("paxi/battery_voltage", 3);
        temp_pubs_ = this->create_publisher<std_msgs::msg::Float64>("paxi/temperature", 3);     
        connected_pubs_ = this->create_publisher<std_msgs::msg::Bool>("paxi/connected", 3);
    }


   void PaxiInterfaceNode::publish_real_time(
        const SerialFeedback & feedback, bool connected, 
        const sensor_msgs::msg::Imu & imu_msg, const std::vector<double> & state_positions) const
    {
 
        publish_data<std_msgs::msg::Float64>(command_pubs_, feedback.cmd_l, feedback.cmd_r);
        publish_data<std_msgs::msg::Float64>(velocity_pubs_, feedback.speed_l_meas,feedback.speed_r_meas);
        publish_data<std_msgs::msg::Float64>(voltage_pubs_,feedback.bat_voltage);
        publish_data<std_msgs::msg::Float64>(temp_pubs_,feedback.board_temp);
        publish_data<std_msgs::msg::Float64>(
            position_pubs_, 
            state_positions[to_index(Wheel::LEFT)], 
            state_positions[to_index(Wheel::RIGHT)]
        );

        publish_data<std_msgs::msg::Bool>(connected_pubs_,connected);

        publish_data<sensor_msgs::msg::Imu>(imu_pubs_, imu_msg);
    }

}//end of namespace paxi_hardware