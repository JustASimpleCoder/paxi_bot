#include "paxi_hardware/paxi_interface.hpp"

namespace paxi_hardware{

    namespace hw = hardware_interface;

    PaxiInterfaceNode::PaxiInterfaceNode() : Node("paxi_interface_node") {

        velocity_pubs[to_index(Wheel::LEFT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/left_wheel/velocity", 3);
        velocity_pubs[to_index(Wheel::RIGHT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/right_wheel/velocity", 3);
        position_pubs[to_index(Wheel::LEFT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/left_wheel/position", 3);
        position_pubs[to_index(Wheel::RIGHT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/right_wheel/position", 3);
        command_pubs[to_index(Wheel::LEFT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/left_wheel/cmd", 3);
        command_pubs[to_index(Wheel::RIGHT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/right_wheel/cmd", 3);


        current_pubs[to_index(Wheel::LEFT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/left_wheel/dc_current", 3);
        current_pubs[to_index(Wheel::RIGHT)] = this->create_publisher<std_msgs::msg::Float64>("paxi/right_wheel/dc_current", 3);
        
        voltage_pubs = this->create_publisher<std_msgs::msg::Float64>("paxi/battery_voltage", 3);
        temp_pubs = this->create_publisher<std_msgs::msg::Float64>("paxi/temperature", 3);
        
        
        
        connected_pubs = this->create_publisher<std_msgs::msg::Bool>("paxi/connected", 3);

    }

    void PaxiInterfaceNode::publish_data(){
        std_msgs::msg::Float64 msg;

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

       if(!serial_communication_.open_port()){
            RCLCPP_ERROR(rclcpp::get_logger("paxi_interface"), "Failed to open serial port to hoverboard");
       }


        RCLCPP_INFO(rclcpp::get_logger("paxi_interface"), "Sucessfully opened serial port to hoverboard, paxi hardware activated!");
        
        return hw::CallbackReturn::SUCCESS;
    } 

    hw::CallbackReturn  PaxiInterface::on_deactivate(
        const rclcpp_lifecycle::State &previous_state)
    {
        //TODO: actually deactivate.....

        serial_communication_.close_port();
        if(serial_communication_.is_open()){
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

            serial_communication_.set_port(hardware_info.hardware_parameters.at("serial_port"));
            serial_communication_.set_baud(
              std::stoul(hardware_info.hardware_parameters.at("baud_rate"))
            );

            wheel_radius_ = std::stod(hardware_info.hardware_parameters.at("wheel_radius"));
            wheel_separation_ = std::stod(hardware_info.hardware_parameters.at("wheel_separation"));
            max_velocity_ = std::stod(hardware_info.hardware_parameters.at("wheel_radius"));


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

            serial_communication_.read_port();
            return hw::return_type::OK;
    }

    hw::return_type PaxiInterface::write(
        const rclcpp::Time & time, const rclcpp::Duration &period)
    {
            return hw::return_type::OK;
    }




}//end of namespace paxi_hardware