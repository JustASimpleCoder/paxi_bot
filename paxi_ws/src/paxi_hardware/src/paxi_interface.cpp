#include "paxi_hardware/paxi_interface.hpp"

namespace paxi_hardware{

    hardware_interface::CallbackReturn  PaxiInterface::on_error(
        const rclcpp_lifecycle::State &/*previous_state*/)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

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

       if(!serial_port_.open_port()){
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_HARDWARE), "Failed to open serial port to hoverboard");
            is_connected_ = false;
            return hardware_interface::CallbackReturn::ERROR;
       }

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_HARDWARE), 
            "Sucessfully opened serial port [%s] to hoverboard, paxi hardware activated!",
            serial_port_.get_port().c_str()
        );

        is_connected_ = true;

        return hardware_interface::CallbackReturn::SUCCESS;
    } 

    hardware_interface::CallbackReturn  PaxiInterface::on_deactivate(
        const rclcpp_lifecycle::State &/*previous_state*/)
    {
        serial_port_.close_port();
        if(serial_port_.is_open()){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_HARDWARE), "Failed to close port, paxi hardware still active!");
        }


        RCLCPP_INFO(rclcpp::get_logger(LOGGER_HARDWARE), "Successfully closed port, paxi hardware deactivated!");
        is_connected_ = false;
        
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

        if(!get_params_from_xacro(hardware_info) ){
            return hardware_interface::CallbackReturn::ERROR;
        }

        if(!check_joints_and_state(hardware_info) ){
            return hardware_interface::CallbackReturn::ERROR;
        }

        first_read_pass_ = true;


        imu_msg_.header.frame_id = "imu_link";
        imu_msg_.orientation_covariance[0] = 0.01; 
        imu_msg_.orientation_covariance[4] = 0.01; 
        imu_msg_.orientation_covariance[8] = 0.01;  

        imu_msg_.angular_velocity_covariance[0] = 0.001;  
        imu_msg_.angular_velocity_covariance[4] = 0.001;  
        imu_msg_.angular_velocity_covariance[8] = 0.001;  

        imu_msg_.linear_acceleration_covariance[0] = 0.1; 
        imu_msg_.linear_acceleration_covariance[4] = 0.1;  
        imu_msg_.linear_acceleration_covariance[8] = 0.1; 

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    bool PaxiInterface::get_params_from_xacro(const hardware_interface::HardwareInfo &hardware_info){
       
        bool validate_params = true;
       
        //try catch here necessary since .at() can throw std::cerr, if not defined check xacro file
        try{
            serial_port_name_ = hardware_info.hardware_parameters.at("serial_port");
            baud_rate_ = hardware_info.hardware_parameters.at("baud_rate");

            validate_params &= serial_port_.set_port(
                hardware_info.hardware_parameters.at("serial_port")
            );
             validate_params &= serial_port_.set_baud(
              std::stoul(hardware_info.hardware_parameters.at("baud_rate"))
            );

            validate_params &= encoder_.set_wheel_radius(
                std::stod(hardware_info.hardware_parameters.at("wheel_radius"))
            );
            validate_params &= encoder_.set_wheel_separation(
                std::stod(hardware_info.hardware_parameters.at("wheel_separation"))
            );
            validate_params &= encoder_.set_max_velocity(
                std::stod(hardware_info.hardware_parameters.at("max_velocity"))
            );


            state_interface_positions_.resize(hardware_info.joints.size(), std::numeric_limits<double>::quiet_NaN());
            state_interface_velocities_.resize(hardware_info.joints.size(), std::numeric_limits<double>::quiet_NaN());  
            hw_commands_.resize(hardware_info.joints.size(), std::numeric_limits<double>::quiet_NaN());

        }catch(const std::out_of_range &e){
            RCLCPP_ERROR(
                rclcpp::get_logger(LOGGER_HARDWARE), 
                "Unable to parse parameters required from XACRO file:  %s", 
                e.what()
            );
            //return hardware_interface::CallbackReturn::ERROR;
            return false;
        }

        if(!validate_params){
            RCLCPP_ERROR(
                rclcpp::get_logger(LOGGER_HARDWARE), 
                "One or more XACRO parameters failed to set, please look atprevious errors for specific paramters"
            );
        }

        return validate_params;
    }

    bool PaxiInterface::check_joints_and_state(const hardware_interface::HardwareInfo &hardware_info){

    for (const hardware_interface::ComponentInfo &joint : hardware_info.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger(LOGGER_HARDWARE),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());

            return false;
        }
    
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger(LOGGER_HARDWARE),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return false;
        }

        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger(LOGGER_HARDWARE),
                "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return false;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger(LOGGER_HARDWARE),
                "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return false;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger(LOGGER_HARDWARE),
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
        
        if(!serial_port_.is_open()){
            RCLCPP_ERROR(
                rclcpp::get_logger(LOGGER_HARDWARE),
                "Serial port [%s] to hoverboard is closed",
                serial_port_name_.c_str()
            );
            return hardware_interface::return_type::ERROR;
        }

        serial_port_.update_connection();
        if(!serial_port_.is_connected()){
            RCLCPP_ERROR_THROTTLE(
                rclcpp::get_logger(LOGGER_HARDWARE),
                *paxi_interface_node_->get_clock(),
                5000,
                "USB device at Serial port [%s] has been disconnected",
                serial_port_name_.c_str()
            );

            return hardware_interface::return_type::ERROR;
        }
       
        ssize_t bytes_read = serial_port_.read_into_uint8_buf(feedback_buf_.data(), feedback_buf_.size());
        for(auto i = 0u; i < bytes_read; ++i){
            if(protocol_.process_byte(feedback_buf_[i])){

                const SerialFeedback& feedback = protocol_.get_feedback();
                state_interface_velocities_[to_index(Wheel::LEFT)] = feedback.speed_l_meas * RPM_TO_RAD_S;  // right velocity is given in opp direction to the left,you can set direction correcetion in diff drive yaml but wont help because of line below (still will be "wrong direcetion in one of the wheels")
                state_interface_velocities_[to_index(Wheel::RIGHT)] = -feedback.speed_r_meas * RPM_TO_RAD_S; // for whatever reason right velocity needs to be flipped (instead of left psotion in update_encoder)

                encoder_.update_encoders(
                    period, 
                    feedback.speed_l_meas, 
                    feedback.speed_r_meas, 
                    state_interface_positions_
                );
                update_imu(time, feedback);
                publish_real_time();
            }
        }
        
        is_connected_ = (serial_port_.is_connected()) ? true : false;

        return hardware_interface::return_type::OK;
    }



    void PaxiInterface::publish_real_time() const
    {
        const SerialFeedback& feedback = protocol_.get_feedback();

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

        paxi_interface_node_->publish_data<sensor_msgs::msg::Imu>(
            paxi_interface_node_->get_imu_pubs(),
            imu_msg_
        );
    }
    void PaxiInterface::update_imu(const rclcpp::Time time, const SerialFeedback &feedback){

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


    hardware_interface::return_type PaxiInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        encoder_.forward_kinematics(hw_commands_);

        const SerialCommand& hover_cmd = protocol_.to_serial_command(
            static_cast<int16_t>(encoder_.get_hover_steer() * STEER_SCALE), 
            static_cast<int16_t>(encoder_.get_hover_speed() * SPEED_SCALE)
        );

        if(serial_port_.write_port(hover_cmd) < 0){
            RCLCPP_WARN(
                rclcpp::get_logger(LOGGER_HARDWARE),
                "Protocol failed to send feedback command to port [%s], with steer [%d] and speed [%d]",
                serial_port_name_.c_str(),
                hover_cmd.steer,
                hover_cmd.speed
            );
        }

        return hardware_interface::return_type::OK;
    }
}//end of namespace paxi_hardware


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  paxi_hardware::PaxiInterface,
  hardware_interface::SystemInterface
)