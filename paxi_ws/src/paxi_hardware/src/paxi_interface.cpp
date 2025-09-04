#include "paxi_hardware/paxi_interface.hpp"

namespace paxi_hardware{

   // namespace hw = hardware_interface;

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

    const std::array<
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 
        to_index(Wheel::COUNT)> PaxiInterfaceNode::get_position_pubs() const
    {
        return position_pubs_;
    }

    const std::array<
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 
        to_index(Wheel::COUNT)> PaxiInterfaceNode::get_velocity_pubs() const
    {
        return velocity_pubs_;
    }
    const std::array<
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 
        to_index(Wheel::COUNT)> PaxiInterfaceNode::get_command_pubs() const
    {
        return command_pubs_;
    }
    
    const std::array<
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 
        to_index(Wheel::COUNT)> PaxiInterfaceNode::get_current_pubs() const

    {
        return current_pubs_;
    }

    const rclcpp::Publisher<
        std_msgs::msg::Float64>::SharedPtr PaxiInterfaceNode::get_voltage_pubs() const
    {
        return voltage_pubs_;
    }

    const rclcpp::Publisher<
        std_msgs::msg::Float64>::SharedPtr PaxiInterfaceNode::get_temp_pubs() const
    {
        return temp_pubs_;
    }
    const rclcpp::Publisher<
        std_msgs::msg::Bool>::SharedPtr PaxiInterfaceNode::get_connected_pubs() const
    {
        return connected_pubs_;
    }


    hardware_interface::CallbackReturn  PaxiInterface::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {


        return hardware_interface::CallbackReturn::SUCCESS;
    } 

    hardware_interface::CallbackReturn  PaxiInterface::on_cleanup(
        const rclcpp_lifecycle::State &previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    } 

    hardware_interface::CallbackReturn  PaxiInterface::on_shutdown(
        const rclcpp_lifecycle::State &previous_state)
    {
         return hardware_interface::CallbackReturn::SUCCESS;
    } 

    hardware_interface::CallbackReturn  PaxiInterface::on_activate(
        const rclcpp_lifecycle::State &previous_state)
  
    {
        for (auto i = 0u; i < hw_commands_.size(); i++){
            if (std::isnan(hw_commands_[i])){
                hw_commands_[i] = 0;
                state_interface_velocities_[i] = 0;
                state_interface_positions_[i] = 0;
            }
        }

       if(!serial_communication_->open_port()){
            RCLCPP_ERROR(rclcpp::get_logger("paxi_interface"), "Failed to open serial port to hoverboard");
            return hardware_interface::CallbackReturn::ERROR;
       }


        RCLCPP_INFO(rclcpp::get_logger("paxi_interface"), 
            "Sucessfully opened serial port [%s] to hoverboard, paxi hardware activated!",
            serial_communication_->get_port().c_str()
        );
        
        return hardware_interface::CallbackReturn::SUCCESS;
    } 

    hardware_interface::CallbackReturn  PaxiInterface::on_deactivate(
        const rclcpp_lifecycle::State &previous_state)
    {
        //TODO: actually deactivate.....

        serial_communication_->close_port();
        if(serial_communication_->is_open()){
            RCLCPP_INFO(rclcpp::get_logger("paxi_interface"), "Failed to close port, paxi hardware still active!");
        }


        RCLCPP_INFO(rclcpp::get_logger("paxi_interface"), "Successfully closed port, paxi hardware deactivated!");
        
        
        return hardware_interface::CallbackReturn::SUCCESS;
    } 

    hardware_interface::CallbackReturn  PaxiInterface::on_error(
        const rclcpp_lifecycle::State &previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }


    bool PaxiInterface::get_params_from_xacro(const hardware_interface::HardwareInfo &hardware_info){
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
            hw_commands_.resize(hardware_info.joints.size(), std::numeric_limits<double>::quiet_NaN());

        }catch(const std::out_of_range &e){
            RCLCPP_ERROR(rclcpp::get_logger("paxi_interface"), "Unable to parse parameters required from XACRO file:  %s", e.what());
            //return hardware_interface::CallbackReturn::ERROR;
            return false;
        }

        return true;
    }

    bool PaxiInterface::check_joints_and_state(const hardware_interface::HardwareInfo &hardware_info){

    for (const hardware_interface::ComponentInfo &joint : hardware_info.joints)
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
    
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("paxi_interface"),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
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

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("paxi_interface"),
                "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return false;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("paxi_interface"),
                "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return false;
        }
        }


        return true;

    }   

    hardware_interface::CallbackReturn  PaxiInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        if( 
            hardware_interface::SystemInterface::on_init(hardware_info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // TO_DO turn this into template for  save file parsing
        serial_communication_ = std::make_shared<SerialPort>();

        if(!get_params_from_xacro(hardware_info) ){
            return hardware_interface::CallbackReturn::ERROR;
        }

        if(!check_joints_and_state(hardware_info) ){
            return hardware_interface::CallbackReturn::ERROR;
        }

        //baud rate and port name should be parsed in xacro file (get from get_params_from_xacro())
        // if(!serial_communication_.open_port()){
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        protocol_ = std::make_shared<HoverboardProtocol>(serial_communication_);

        paxi_interface_node_ = std::make_unique<PaxiInterfaceNode>();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> PaxiInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto i = 0u; i < info_.joints.size(); ++i)
        {
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_interface_positions_[i]));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_interface_velocities_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> PaxiInterface::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); ++i)
        {
            command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::return_type PaxiInterface::prepare_command_mode_switch(    
        const std::vector<std::string> &,
        const std::vector<std::string> &)
    {
        return hardware_interface::return_type::OK;

    }

    hardware_interface::return_type PaxiInterface::perform_command_mode_switch(    
        const std::vector<std::string> &,
        const std::vector<std::string> &)
    {
            return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type PaxiInterface::read(
        const rclcpp::Time & time, const rclcpp::Duration &period)
    {
        
        if(!serial_communication_->is_open()){
            return hardware_interface::return_type::ERROR;
        }


        std::vector<uint8_t> feedback_buf(1024);
        ssize_t bytes_read = serial_communication_->read_port_binary(feedback_buf.data(), feedback_buf.size());
        uint8_t prev_byte;


        //debug stuff: 
        // std::string hex_debug;
        // for(int i = 0; i < bytes_read; ++i) {
        //     char hex_str[4];
        //     snprintf(hex_str, sizeof(hex_str), "%02X ", feedback_buf[i]);
        //     hex_debug += hex_str;
        // }
        // RCLCPP_INFO(rclcpp::get_logger("paxi_interface"),
        //             "Raw data from feedback port: %s", hex_debug.c_str());


        for(auto i = 0u; i < bytes_read; ++i){
            uint8_t incoming_byte = feedback_buf[i];
            if(protocol_->validate_checksum(prev_byte, incoming_byte)){

                state_interface_velocities_[to_index(Wheel::LEFT)] = 
                    std::abs(protocol_->get_feedback().speedL_meas) * RPM_to_rad_s  * direction_correction_;

                state_interface_velocities_[to_index(Wheel::RIGHT)] = 
                    std::abs(protocol_->get_feedback().speedR_meas) * RPM_to_rad_s  * direction_correction_; 

                publish_real_time();
                update_encoders(
                    last_read_, 
                    protocol_->get_feedback().wheelL_cnt, 
                    protocol_->get_feedback().wheelR_cnt
                );
                RCLCPP_INFO(
                    rclcpp::get_logger("paxi_interface"),"Hardware feedback and checksum complete, published data"
                );
            }

            prev_byte = incoming_byte;
        }

        return hardware_interface::return_type::OK;
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
            to_index(Wheel::LEFT)
        );

        paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
            paxi_interface_node_->get_velocity_pubs(),
            protocol_->get_feedback().speedR_meas,
            to_index(Wheel::RIGHT)
        );

        paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
            paxi_interface_node_->get_position_pubs(),
            protocol_->get_feedback().wheelL_cnt,
            to_index(Wheel::LEFT)
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
            true
        );
    }

    void PaxiInterface::update_encoders(const rclcpp::Time &time, int16_t right, int16_t left){
            double posL = 0.0, posR = 0.0;

            // Calculate wheel position in ticks, factoring in encoder wraps
            if (right < low_wrap_ && last_wheel_count_l_ > high_wrap_){
                mult_r_++;
            }
            else if (right > high_wrap_ && last_wheel_count_l_ < low_wrap_){
                mult_r_--;
                posR = right + mult_r_ * (ENCODER_MAX - ENCODER_MIN);
                last_wheel_count_l_ = right;
            }

            if (left < low_wrap_ && last_wheel_count_l_ > high_wrap_){
                mult_l_++;
            }
            else if (left > high_wrap_ && last_wheel_count_l_ < low_wrap_){
                mult_l_--;
                posL = left + mult_l_ * (ENCODER_MAX - ENCODER_MIN);
                last_wheel_count_l_ = left;
            }

            // When the board shuts down and restarts, wheel ticks are reset to zero so the robot can be suddently lost
            // This section accumulates ticks even if board shuts down and is restarted
            static double lastPosL = 0.0, lastPosR = 0.0;
            static double lastPubPosL = 0.0, lastPubPosR = 0.0;
            static bool nodeStartFlag = true;

            // IF there has been a pause in receiving data AND the new number of ticks is close to zero, indicates a board restard
            //(the board seems to often report 1-3 ticks on startup instead of zero)
            // reset the last read ticks to the startup values
            if ((time - last_read_).seconds() > 0.2 && abs(posL) < 5 && abs(posR) < 5)
            {
                lastPosL = posL;
                lastPosR = posR;
            }
            double posLDiff = 0;
            double posRDiff = 0;

            // if node is just starting keep odom at zeros
            if (nodeStartFlag)
            {
                nodeStartFlag = false;
            }
            else
            {
                posLDiff = posL - lastPosL;
                posRDiff = posR - lastPosR;
            }

            lastPubPosL += posLDiff;
            lastPubPosR += posRDiff;
            lastPosL = posL;
            lastPosR = posR;

            // Convert position in accumulated ticks to position in radians
            state_interface_positions_[to_index(Wheel::LEFT)] = 2.0 * M_PI * lastPubPosL / (double)TICKS_PER_ROTATION;
            state_interface_positions_[to_index(Wheel::RIGHT)] = 2.0 * M_PI * lastPubPosR / (double)TICKS_PER_ROTATION;

            // paxi_interface_node_->publish_pos(left_wheel, hw_positions_[left_wheel]);
            // paxi_interface_node_>publish_pos(right_wheel, hw_positions_[right_wheel]);
    }

    void PaxiInterface::forward_kinematics(){

        wheel_omega_l_ = hw_commands_[to_index(Wheel::LEFT)];
        wheel_omega_r_ = hw_commands_[to_index(Wheel::RIGHT)];

        wheel_vel_l_ = wheel_omega_l_ * wheel_radius_;
        wheel_vel_r_ = wheel_omega_r_ * wheel_radius_;
    
        hoverboard_speed_ = (wheel_vel_r_ + wheel_vel_l_) / 2.0;
        hoverboard_steer_ = (wheel_vel_l_ - wheel_vel_l_) / wheel_separation_;
    }

    void PaxiInterface::inverse_kinematics(){
        wheel_vel_l_ = hoverboard_speed_ - (wheel_separation_ / 2.0)* hoverboard_steer_;
        wheel_vel_r_ = hoverboard_speed_ + (wheel_separation_ / 2.0)* hoverboard_steer_;

        wheel_omega_l_ = wheel_vel_l_ / wheel_radius_;
        wheel_omega_r_ = wheel_vel_r_ / wheel_radius_;
    }
    hardware_interface::return_type PaxiInterface::write(
        const rclcpp::Time & time, const rclcpp::Duration &period)
    {


        forward_kinematics();

        if(!protocol_->send( 
            static_cast<int16_t>(hoverboard_steer_ * STEER_SCALE), 
            static_cast<int16_t>(hoverboard_speed_ * SPEED_SCALE)))
        {
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }
}//end of namespace paxi_hardware


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  paxi_hardware::PaxiInterface,
  hardware_interface::SystemInterface
)