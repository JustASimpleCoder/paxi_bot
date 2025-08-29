#ifndef PAXI_INTERFACE_HPP
#define PAXI_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "hardware_interface/handle.hpp"

//#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp_lifecycle/state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"


#include <array>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "paxi_hardware/serial_communication.hpp"

namespace paxi_hardware{

    namespace hw = hardware_interface;

    
    enum class Wheel: std::size_t {
        LEFT = 0,
        RIGHT = 1,
        COUNT = 2
    };

    // helper function to conver WheelPostion enum to appropriate index
    template<typename T>
    constexpr std::size_t enum_to_index(T pos) noexcept{
        return static_cast<std::size_t>(pos);
    }

    class PaxiInterfaceNode : public rclcpp::Node{
        public:
            PaxiInterfaceNode();
            ~PaxiInterfaceNode() = default;
            void publish_data();
        
        private:

            std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, enum_to_index(Wheel::COUNT)> position_pubs;
            std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, enum_to_index(Wheel::COUNT)> velocity_pubs;
            std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, enum_to_index(Wheel::COUNT)> command_pubs;
            std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, enum_to_index(Wheel::COUNT)> current_pubs;
            
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pubs;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_pubs;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pubs;

        };

    class PaxiInterface : public hw::SystemInterface{

        public:
            PaxiInterface() = default;


            hw::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
            hw::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
            hw::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
            hw::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
            hw::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
            hw::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
            hw::CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;

            std::vector<hw::StateInterface> export_state_interfaces() override;
            std::vector<hw::CommandInterface> export_command_interfaces() override;
            
            hw::return_type prepare_command_mode_switch(    
                const std::vector<std::string> &,
                const std::vector<std::string> &) override;
            
            hw::return_type perform_command_mode_switch(    
                const std::vector<std::string> &,
                const std::vector<std::string> &) override;

            hw::return_type read(
                const rclcpp::Time & time, const rclcpp::Duration &period) override;

            hw::return_type write(
                const rclcpp::Time & time, const rclcpp::Duration &period) override;

            bool get_params_from_xacro(const hw::HardwareInfo &hardware_info);
            bool check_joints_and_state(const hw::HardwareInfo &hardware_info);

        private:


            paxi_serial::SerialPort serial_communication_;

            std::string serial_port_;
            std::string baud_rate_;
            int port_fd_;

            double wheel_radius_;
            double wheel_separation_;
            double max_velocity_;


            std::vector<double> state_interface_positions_;
            std::vector<double> state_interface_velocities_;
            std::vector<double> command_interface_commands_;

            std::unique_ptr<PaxiInterfaceNode> paxi_interface_node_;
    };
}// namespace paxi_hardware

#endif