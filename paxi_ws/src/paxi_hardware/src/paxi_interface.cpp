#include "paxi_hardware/paxi_interface.hpp"

namespace paxi_hardware
{

    PaxiInterface::PaxiInterface()
    :   serial_port_{},
        protocol_{},
        encoder_{},
        imu_{},  
        hw_commands_{},
        protocol_worker_{serial_port_, protocol_, encoder_,imu_,}
    {}

    hardware_interface::return_type PaxiInterface::prepare_command_mode_switch(
      const std::vector<std::string> &, const std::vector<std::string> &)
    {
      return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type PaxiInterface::perform_command_mode_switch(
      const std::vector<std::string> &, const std::vector<std::string> &)
    {
      return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn PaxiInterface::on_error(
      const rclcpp_lifecycle::State & /*previous_state*/)
    {
      return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PaxiInterface::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
    {
      return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PaxiInterface::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
    {
      return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PaxiInterface::on_shutdown(
      const rclcpp_lifecycle::State & /*previous_state*/)
    {
      return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PaxiInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)

    {

      for (auto i = 0u; i < hw_commands_.size(); i++) {
          if (std::isnan(hw_commands_[i])) {
            hw_commands_[i] = 0.0;
          }
      }


      if (!serial_port_.open_port()) {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_HARDWARE), "Failed to open serial port to hoverboard");
        return hardware_interface::CallbackReturn::ERROR;
      }

      RCLCPP_INFO(
        rclcpp::get_logger(LOGGER_HARDWARE),
        "Sucessfully opened serial port [%s] to hoverboard, paxi hardware activated!",
        serial_port_.get_port_name().c_str()
      );

      protocol_worker_.start_worker();
    
      return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PaxiInterface::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
    {
      serial_port_.close_port();
      if (serial_port_.is_open()) {
        RCLCPP_INFO(
          rclcpp::get_logger(LOGGER_HARDWARE), "Failed to close port, paxi hardware still active!");
      }

      RCLCPP_INFO(
        rclcpp::get_logger(LOGGER_HARDWARE), "Sucessfully closed port [%s]!",
        serial_port_.get_port_name().c_str()
      );

      protocol_worker_.stop_worker();

      return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PaxiInterface::on_init(
      const hardware_interface::HardwareInfo & hardware_info)
    {
      if( hardware_interface::SystemInterface::on_init(hardware_info) != 
          hardware_interface::CallbackReturn::SUCCESS)
      {
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (!get_params_from_xacro(hardware_info)) {
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (!check_joints_and_state(hardware_info)) {
        return hardware_interface::CallbackReturn::ERROR;
      }

      protocol_worker_.init_zero_state_interfaces(hardware_info);

      return hardware_interface::CallbackReturn::SUCCESS;
    }

    bool PaxiInterface::get_params_from_xacro(const hardware_interface::HardwareInfo & hardware_info)
    {
      bool validate_params = true;

      //try catch here necessary since .at() can throw std::cerr, if not defined check xacro file
      try {
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

        validate_params &= imu_.set_imu_link_name(
              hardware_info.hardware_parameters.at("imu_link_name")
        );

        hw_commands_.resize(hardware_info.joints.size(), std::numeric_limits<double>::quiet_NaN());

      } catch (const std::out_of_range & e) {

          RCLCPP_ERROR(
              rclcpp::get_logger(LOGGER_HARDWARE),
              "Unable to parse parameters required from XACRO file:  %s", 
              e.what()
          );
        
          return false;
      }

      if (!validate_params) {
          RCLCPP_ERROR(
              rclcpp::get_logger(LOGGER_HARDWARE),
              "One or more XACRO parameters failed to set, please look at previous errors for specific paramters"
          );
      }

      return validate_params;
    }

    bool PaxiInterface::check_joints_and_state(const hardware_interface::HardwareInfo& hardware_info)
    {
      for (const hardware_interface::ComponentInfo & joint : hardware_info.joints) {
        // taken from DiffBotSystem which has exactly two states and one command interface on each joint
        if (joint.command_interfaces.size() != 1) {
          RCLCPP_FATAL(
            rclcpp::get_logger(LOGGER_HARDWARE),
            "Joint '%s' has %zu command interfaces found. 1 expected.", 
            joint.name.c_str(),
            joint.command_interfaces.size()
          );

          return false;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
          RCLCPP_FATAL(
            rclcpp::get_logger(LOGGER_HARDWARE),
            "Joint '%s' have %s command interfaces found. '%s' expected.", 
            joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), 
            hardware_interface::HW_IF_VELOCITY
          );
          return false;
        }

        if (joint.state_interfaces.size() != 2) {
          RCLCPP_FATAL(
            rclcpp::get_logger(LOGGER_HARDWARE),
            "Joint '%s' has %zu state interface. 2 expected.",
            joint.name.c_str(), 
            joint.state_interfaces.size()
          );

          return false;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
          RCLCPP_FATAL(
            rclcpp::get_logger(LOGGER_HARDWARE),
            "Joint '%s' have '%s' as first state interface. '%s' expected.",
            joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), 
            hardware_interface::HW_IF_POSITION
          );
          return false;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
          RCLCPP_FATAL(
            rclcpp::get_logger(LOGGER_HARDWARE),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", 
            joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), 
            hardware_interface::HW_IF_VELOCITY
          );
          return false;
        }
      }

      return true;
    }


    std::vector<hardware_interface::StateInterface> PaxiInterface::export_state_interfaces()
    {
      std::vector<hardware_interface::StateInterface> state_interfaces;

      std::vector<double> state_positions;
      std::vector<double> state_velocities;

      protocol_worker_.get_state_interface(state_positions, state_velocities);

      for (auto i = 0u; i < info_.joints.size(); ++i) {
          state_interfaces.emplace_back(
              hardware_interface::StateInterface(
                  info_.joints[i].name, 
                  hardware_interface::HW_IF_POSITION, 
                  protocol_worker_.get_state_interface_position_ptr(i)
              )
          );

          state_interfaces.emplace_back(
              hardware_interface::StateInterface(
                  info_.joints[i].name, 
                  hardware_interface::HW_IF_VELOCITY, 
                  protocol_worker_.get_state_interface_velocity_ptr(i)
              )
          );
      }

      return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> PaxiInterface::export_command_interfaces()
    {
      std::vector<hardware_interface::CommandInterface> command_interfaces;
      for (auto i = 0u; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
      }

      return command_interfaces;
    }

    hardware_interface::return_type PaxiInterface::read(
      const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {

      return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type PaxiInterface::write( 
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {

      SerialCommand hover_cmd;
      {
        std::scoped_lock<std::mutex> lock(protocol_worker_.get_state_mutex());
        encoder_.forward_kinematics(hw_commands_);
      
        hover_cmd = protocol_.to_serial_command(
          static_cast<int16_t>(encoder_.get_hover_steer() * STEER_SCALE),
          static_cast<int16_t>(encoder_.get_hover_speed() * SPEED_SCALE)
        );
      }

      {
        std::scoped_lock<std::mutex> lock(protocol_worker_.get_serial_mutex());
        if (serial_port_.write_port(hover_cmd) < 0) {
          RCLCPP_WARN(
            rclcpp::get_logger(LOGGER_HARDWARE),
            "Protocol failed to send feedback command to port [%s], with steer [%d] and speed [%d]",
            serial_port_.get_port_name().c_str(), 
            hover_cmd.steer, 
            hover_cmd.speed
          );
        }
      }

      return hardware_interface::return_type::OK;
    }
}  //end of namespace paxi_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(paxi_hardware::PaxiInterface, hardware_interface::SystemInterface)