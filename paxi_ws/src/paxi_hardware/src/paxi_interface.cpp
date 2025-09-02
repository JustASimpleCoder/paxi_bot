#include "paxi_hardware/paxi_interface.hpp"

namespace paxi_hardware{

    namespace hw = hardware_interface;

    PaxiInterfaceNode::PaxiInterfaceNode() : Node("paxi_interface_node")  {

        velocity_pubs_[to_index(Wheel::LEFT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/left_wheel/velocity", 3);
        velocity_pubs_[to_index(Wheel::RIGHT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/right_wheel/velocity", 3);
        position_pubs_[to_index(Wheel::LEFT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/left_wheel/position", 3);
        position_pubs_[to_index(Wheel::RIGHT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/right_wheel/position", 3);
        command_pubs_[to_index(Wheel::LEFT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/left_wheel/cmd", 3);
        command_pubs_[to_index(Wheel::RIGHT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/right_wheel/cmd", 3);
        current_pubs_[to_index(Wheel::LEFT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/left_wheel/dc_current", 3);
        current_pubs_[to_index(Wheel::RIGHT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/right_wheel/dc_current", 3);
        
        voltage_pubs_ = this->create_publisher<std_msgs::msg::Float64>("paxi/battery_voltage", 3);
        temp_pubs_ = this->create_publisher<std_msgs::msg::Float64>("paxi/temperature", 3);     
        connected_pubs_ = this->create_publisher<std_msgs::msg::Bool>("paxi/connected", 3);

    }


    const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, to_index(Wheel::COUNT)> PaxiInterfaceNode::get_position_pubs() const
    {
        return position_pubs_;
    }

    const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, to_index(Wheel::COUNT)> PaxiInterfaceNode::get_velocity_pubs() const
    {
        return velocity_pubs_;
    }
    const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, to_index(Wheel::COUNT)> PaxiInterfaceNode::get_command_pubs() const
    {
        return command_pubs_;
    }
    
    const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, to_index(Wheel::COUNT)> PaxiInterfaceNode::get_current_pubs() const

    {
        return current_pubs_;
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


    hw::CallbackReturn  PaxiInterface::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {


        return hw::CallbackReturn::SUCCESS;
    } 

    hw::CallbackReturn  PaxiInterface::on_cleanup(
        const rclcpp_lifecycle::State &previous_state)
    {
        return hw::CallbackReturn::SUCCESS;
    } 

    hw::CallbackReturn  PaxiInterface::on_shutdown(
        const rclcpp_lifecycle::State &previous_state)
    {
         return hw::CallbackReturn::SUCCESS;
    } 

    hw::CallbackReturn  PaxiInterface::on_activate(
        const rclcpp_lifecycle::State &previous_state)
  
    {
        for (auto i = 0u; i < command_interface_commands_.size(); i++){
            if (std::isnan(command_interface_commands_[i])){
                command_interface_commands_[i] = 0;
                state_interface_velocities_[i] = 0;
                state_interface_positions_[i] = 0;
            }
        }

       if(!serial_communication_->open_port()){
            RCLCPP_ERROR(rclcpp::get_logger("paxi_interface"), "Failed to open serial port to hoverboard");
            return hw::CallbackReturn::ERROR;
       }


        RCLCPP_INFO(rclcpp::get_logger("paxi_interface"), "Sucessfully opened serial port to hoverboard, paxi hardware activated!");
        
        return hw::CallbackReturn::SUCCESS;
    } 

    hw::CallbackReturn  PaxiInterface::on_deactivate(
        const rclcpp_lifecycle::State &previous_state)
    {
        //TODO: actually deactivate.....

        serial_communication_->close_port();
        if(serial_communication_->is_open()){
            RCLCPP_INFO(rclcpp::get_logger("paxi_interface"), "Failed to close port, paxi hardware still active!");
        }


        RCLCPP_INFO(rclcpp::get_logger("paxi_interface"), "Successfully closed port, paxi hardware deactivated!");
        
        
        return hw::CallbackReturn::SUCCESS;
    } 

    hw::CallbackReturn  PaxiInterface::on_error(
        const rclcpp_lifecycle::State &previous_state)
    {
        return hw::CallbackReturn::SUCCESS;
    }


    bool PaxiInterface::get_params_from_xacro(const hw::HardwareInfo &hardware_info){
        //try catch here necessary since .at() can throw std::cerr, if not defined check xacro file
        try{
            serial_port_ = hardware_info.hardware_parameters.at("serial_port");
            baud_rate_ = hardware_info.hardware_parameters.at("baud_rate");

            serial_communication_->set_port(hardware_info.hardware_parameters.at("serial_port"));
            serial_communication_->set_baud(
              std::stoul(hardware_info.hardware_parameters.at("baud_rate"))
            );

            wheel_radius_ = std::stod(hardware_info.hardware_parameters.at("wheel_radius"));
            wheel_separation_ = std::stod(hardware_info.hardware_parameters.at("wheel_separation"));
            max_velocity_ = std::stod(hardware_info.hardware_parameters.at("max_velocity"));


            state_interface_positions_.resize(hardware_info.joints.size(), std::numeric_limits<double>::quiet_NaN());
            state_interface_velocities_.resize(hardware_info.joints.size(), std::numeric_limits<double>::quiet_NaN());  
            command_interface_commands_.resize(hardware_info.joints.size(), std::numeric_limits<double>::quiet_NaN());

        }catch(const std::out_of_range &e){
            RCLCPP_ERROR(rclcpp::get_logger("paxi_interface"), "Unable to parse parameters required from XACRO file:  %s", e.what());
            //return hw::CallbackReturn::ERROR;
            return false;
        }

        return true;
    }

    bool PaxiInterface::check_joints_and_state(const hw::HardwareInfo &hardware_info){

    for (const hw::ComponentInfo &joint : hardware_info.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("paxi_interface"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());

            return false;
        }
    
        if (joint.command_interfaces[0].name != hw::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("paxi_interface"),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hw::HW_IF_VELOCITY);
            return false;
        }

        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("paxi_interface"),
                "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return false;
        }

        if (joint.state_interfaces[0].name != hw::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("paxi_interface"),
                "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hw::HW_IF_POSITION);
            return false;
        }

        if (joint.state_interfaces[1].name != hw::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("paxi_interface"),
                "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hw::HW_IF_VELOCITY);
            return false;
        }
        }


        return true;

    }   

    hw::CallbackReturn  PaxiInterface::on_init(const hw::HardwareInfo &hardware_info)
    {
        if( 
            hw::SystemInterface::on_init(hardware_info) !=
            hw::CallbackReturn::SUCCESS)
        {
            return hw::CallbackReturn::ERROR;
        }

        // TO_DO turn this into template for  save file parsing
        serial_communication_ = std::make_shared<SerialPort>();

        if(!get_params_from_xacro(hardware_info) ){
            return hw::CallbackReturn::ERROR;
        }

        if(!check_joints_and_state(hardware_info) ){
            return hw::CallbackReturn::ERROR;
        }

        //baud rate and port name should be parsed in xacro file (get from get_params_from_xacro())
        // if(!serial_communication_.open_port()){
        //     return hw::CallbackReturn::ERROR;
        // }

        protocol_ = std::make_shared<HoverboardProtocol>(serial_communication_);

        return hw::CallbackReturn::SUCCESS;
    }

    std::vector<hw::StateInterface> PaxiInterface::export_state_interfaces() {
        std::vector<hw::StateInterface> state_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(
            hw::StateInterface(
                info_.joints[i].name, hw::HW_IF_POSITION, &state_interface_positions_[i]));
            state_interfaces.emplace_back(
            hw::StateInterface(
                info_.joints[i].name, hw::HW_IF_VELOCITY, &state_interface_velocities_[i]));
        }

        return state_interfaces;
    }

    std::vector<hw::CommandInterface> PaxiInterface::export_command_interfaces(){
        std::vector<hw::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(
            hw::CommandInterface(
                info_.joints[i].name, hw::HW_IF_VELOCITY, &command_interface_commands_[i]));
        }

        return command_interfaces;
    }

    hw::return_type PaxiInterface::prepare_command_mode_switch(    
        const std::vector<std::string> &,
        const std::vector<std::string> &)
    {
        return hw::return_type::OK;

    }

    hw::return_type PaxiInterface::perform_command_mode_switch(    
        const std::vector<std::string> &,
        const std::vector<std::string> &)
    {
            return hw::return_type::OK;
    }

    hw::return_type PaxiInterface::read(
        const rclcpp::Time & time, const rclcpp::Duration &period)
    {

        if(!serial_communication_->is_open()){
            return hw::return_type::ERROR;
        }

        protocol_->receive();
        publish_real_time();

        return hw::return_type::OK;
    }

    void PaxiInterface::publish_real_time() const
    {

            paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
                paxi_interface_node_->get_command_pubs(),
                protocol_->get_feedback().cmd1,
                to_index(Wheel::LEFT)
            );

            paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
                paxi_interface_node_->get_command_pubs(),
                protocol_->get_feedback().cmd2,
                to_index(Wheel::RIGHT)
            );

            paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
                paxi_interface_node_->get_velocity_pubs(),
                protocol_->get_feedback().speedL_meas,
                to_index(Wheel::RIGHT)
            );

            paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
                paxi_interface_node_->get_velocity_pubs(),
                protocol_->get_feedback().speedR_meas,
                to_index(Wheel::RIGHT)
            );

            paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
                paxi_interface_node_->get_position_pubs(),
                protocol_->get_feedback().wheelL_cnt,
                to_index(Wheel::RIGHT)
            );

            paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
                paxi_interface_node_->get_position_pubs(),
                protocol_->get_feedback().wheelR_cnt,
                to_index(Wheel::RIGHT)
            );
            
            paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
                paxi_interface_node_->get_current_pubs(),
                protocol_->get_feedback().left_dc_curr,
                to_index(Wheel::RIGHT)
            );

            paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
                paxi_interface_node_->get_current_pubs(),
                protocol_->get_feedback().right_dc_curr,
                to_index(Wheel::RIGHT)
            );

            paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
                paxi_interface_node_->get_voltage_pubs(),
                protocol_->get_feedback().left_dc_curr
            );

            paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
                paxi_interface_node_->get_temp_pubs(),
                protocol_->get_feedback().right_dc_curr
            );

            paxi_interface_node_->publish_data<std_msgs::msg::Bool>(
                paxi_interface_node_->get_connected_pubs(),
                protocol_->get_feedback().right_dc_curr
            );
    }

    hw::return_type PaxiInterface::write(
        const rclcpp::Time & time, const rclcpp::Duration &period)
    {
        //TODO: fix magic numbers
        
        const double wheel_vel_r = command_interface_commands_[to_index(Wheel::LEFT)] / deg_to_rad;
        const double wheel_vel_l =  command_interface_commands_[to_index(Wheel::RIGHT)] / deg_to_rad;
        
        const double total_vel = (wheel_vel_r + wheel_vel_l) / 2.0;
        const double total_omega = (wheel_vel_r - wheel_vel_r) / (wheel_separation_ );


        const double wheel_omega_r =  (total_vel  + total_omega*(1/2) ) ;
        const double wheel_omgea_l = 1;



        const double steer = (wheel_vel_r - total_vel) * 2.0;

        const double speed = total_vel;

        if(!protocol_->send( static_cast<uint16_t>(steer), static_cast<uint16_t>(speed))){
            return hw::return_type::ERROR;
        }

        return hw::return_type::OK;
    }




}//end of namespace paxi_hardware


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  paxi_hardware::PaxiInterface,
  hardware_interface::SystemInterface
)