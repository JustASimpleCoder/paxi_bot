#include "paxi_hardware/paxi_interface.hpp"

namespace paxi_hardware{

    hardware_interface::CallbackReturn  PaxiInterface::on_configure(
        const rclcpp_lifecycle::State &/*previous_state*/)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    } 

    hardware_interface::CallbackReturn  PaxiInterface::on_cleanup(
        const rclcpp_lifecycle::State &/*previous_state*/)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    } 

    hardware_interface::CallbackReturn  PaxiInterface::on_shutdown(
        const rclcpp_lifecycle::State &/*previous_state*/)
    {
         return hardware_interface::CallbackReturn::SUCCESS;
    } 

    hardware_interface::CallbackReturn  PaxiInterface::on_activate(
        const rclcpp_lifecycle::State &/*previous_state*/)
  
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
            is_connected_ = false;
            return hardware_interface::CallbackReturn::ERROR;
       }

        RCLCPP_INFO(rclcpp::get_logger("paxi_interface"), 
            "Sucessfully opened serial port [%s] to hoverboard, paxi hardware activated!",
            serial_communication_->get_port().c_str()
        );

        is_connected_ = true;

        return hardware_interface::CallbackReturn::SUCCESS;
    } 

    hardware_interface::CallbackReturn  PaxiInterface::on_deactivate(
        const rclcpp_lifecycle::State &/*previous_state*/)
    {
        serial_communication_->close_port();
        if(serial_communication_->is_open()){
            RCLCPP_INFO(rclcpp::get_logger("paxi_interface"), "Failed to close port, paxi hardware still active!");
        }


        RCLCPP_INFO(rclcpp::get_logger("paxi_interface"), "Successfully closed port, paxi hardware deactivated!");
        is_connected_ = false;
        
        return hardware_interface::CallbackReturn::SUCCESS;
    } 

    hardware_interface::CallbackReturn  PaxiInterface::on_error(
        const rclcpp_lifecycle::State &/*previous_state*/)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn  PaxiInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        if( 
            hardware_interface::SystemInterface::on_init(hardware_info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // serial communication must be defined befor parsing from xacro
        serial_communication_ = std::make_shared<SerialPort>();

        if(!get_params_from_xacro(hardware_info) ){
            return hardware_interface::CallbackReturn::ERROR;
        }

        if(!check_joints_and_state(hardware_info) ){
            return hardware_interface::CallbackReturn::ERROR;
        }


        protocol_ = std::make_shared<HoverboardProtocol>(serial_communication_);
        paxi_interface_node_ = std::make_unique<PaxiInterfaceNode>();

        first_read_enc_ = true;
        first_read_pass_ = true;

        last_read_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        last_read_enc_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        last_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

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

        ssize_t bytes_read = serial_communication_->read_into_uint8_buf(feedback_buf_.data(), feedback_buf_.size());

        for(auto i = 0u; i < bytes_read; ++i){
            if(protocol_->process_byte(feedback_buf_[i])){

                const SerialFeedback& feedback = protocol_->get_feedback();
                // right velocit id given in opp direction to the left,
                // for whatever reason right velocity needs to be flipped (instead of left psotion below)
                state_interface_velocities_[to_index(Wheel::LEFT)] = feedback.speed_l_meas * RPM_TO_RAD_S;
                state_interface_velocities_[to_index(Wheel::RIGHT)] = -feedback.speed_r_meas * RPM_TO_RAD_S; 
                
                //TODO: move this to thread or its own node to not affect the main control loop
                if(time - last_publish_time_ > std::chrono::minutes(1)){
                    publish_real_time();
                    last_publish_time_ = time;
                }
                update_encoders(time, period, feedback.speed_l_meas, feedback.speed_r_meas);
            }
        }

        is_connected_ = ((time - last_read_).seconds() > 1) ? false : true;
        return hardware_interface::return_type::OK;
    }

    void PaxiInterface::publish_real_time() const
    {
        const SerialFeedback& feedback = protocol_->get_feedback();

        paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
            paxi_interface_node_->get_command_pubs(),
            feedback.cmd_l,
            feedback.cmd_r
        );

        paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
            paxi_interface_node_->get_velocity_pubs(),
            feedback.speed_l_meas,
            feedback.speed_r_meas
        );

        paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
            paxi_interface_node_->get_voltage_pubs(),
            feedback.bat_voltage
        );

        paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
            paxi_interface_node_->get_temp_pubs(),
            feedback.board_temp
        );

        paxi_interface_node_->publish_data<std_msgs::msg::Bool>(
            paxi_interface_node_->get_connected_pubs(),
            is_connected_
        );

        paxi_interface_node_->publish_data<std_msgs::msg::Float64>(
            paxi_interface_node_->get_position_pubs(),
            state_interface_positions_[to_index(Wheel::LEFT)],
            state_interface_positions_[to_index(Wheel::RIGHT)]
        );
    }

    void PaxiInterface::update_encoders(const rclcpp::Time &time, const rclcpp::Duration & duration, int16_t r_rpm, int16_t l_rpm){

            if (first_read_enc_) {
                prev_l_rad_per_sec_ = l_rpm * RPM_TO_RAD_S;
                prev_r_rad_per_sec_ = r_rpm * RPM_TO_RAD_S;
                first_read_enc_ = false;
                return; 
            }

            const double delta_time = duration.seconds();
            last_read_enc_ = time; 

            const double l_rad_per_sec = l_rpm * RPM_TO_RAD_S;
            const double r_rad_per_sec = r_rpm * RPM_TO_RAD_S;

            const double avg_l_rad_per_sec = (prev_l_rad_per_sec_ + l_rad_per_sec) / 2.0;
            const double avg_r_rad_per_sec = (prev_r_rad_per_sec_ + r_rad_per_sec) / 2.0;

            const double delta_l_pos = avg_l_rad_per_sec * delta_time * wheel_radius_;
            const double delta_r_pos = avg_r_rad_per_sec * delta_time * wheel_radius_;


            // left position is given in opp direction to the right,
            // for whatever reason left postition needs to be flipped (instead of right velocity above)
            state_interface_positions_[to_index(Wheel::LEFT)] += -delta_l_pos;
            state_interface_positions_[to_index(Wheel::RIGHT)] += delta_r_pos;

            prev_l_rad_per_sec_ = l_rad_per_sec;
            prev_r_rad_per_sec_ = r_rad_per_sec;
    }

    void PaxiInterface::forward_kinematics(){

        wheel_omega_l_ = hw_commands_[to_index(Wheel::LEFT)];
        wheel_omega_r_ = hw_commands_[to_index(Wheel::RIGHT)];

        wheel_vel_l_ = wheel_omega_l_ * wheel_radius_;
        wheel_vel_r_ = wheel_omega_r_ * wheel_radius_;
    
        hoverboard_speed_ = (wheel_vel_r_ + wheel_vel_l_) / 2.0;
        hoverboard_steer_ = (wheel_vel_r_ - wheel_vel_l_) / wheel_separation_;
    }

    void PaxiInterface::inverse_kinematics(){
        wheel_vel_l_ = hoverboard_speed_ - (wheel_separation_ / 2.0)* hoverboard_steer_;
        wheel_vel_r_ = hoverboard_speed_ + (wheel_separation_ / 2.0)* hoverboard_steer_;

        wheel_omega_l_ = wheel_vel_l_ / wheel_radius_;
        wheel_omega_r_ = wheel_vel_r_ / wheel_radius_;
    }
    hardware_interface::return_type PaxiInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
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